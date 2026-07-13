#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <esp_camera.h>
#include <esp_heap_caps.h>
#include <math.h>

#include "pins.h"

// Step 1: blink/serial-print bring-up (heartbeat + shutter button) — done.
// Step 2: TFT alone, SPI init + test pattern — done.
// Step 3: camera alone, JPEG capture dumped over serial — done.
// Step 4: camera frames rendering live to the TFT — done. Camera runs
// continuous RGB565 QVGA (320x240) capture, which matches the TFT's
// landscape resolution 1:1, so each frame blits straight to the display
// with no scaling or rotation.
// Step 5: printer UART independently, raw ESC/POS test line — done.
// Step 6 of the build order in CLAUDE.md: wire the button into a full
// capture -> preview -> print state machine. Button press freezes the
// live frame on the TFT, dithers it to 1-bit and prints it as an ESC/POS
// raster image (GS v 0), then resumes the live feed.
//
// DP-628 baud rate and print width are NOT confirmed from an official
// datasheet (see CLAUDE.md known gotchas): 9600 8N1 is the common default
// for these TTL thermal printer modules, and 384 dots (58mm @ 8 dots/mm,
// 203dpi) is the common print width for this printer size. At 9600 baud
// a full 384x288 image is ~14s of transfer — try 19200/115200 if that's
// too slow and the printer accepts it.
//
// TFT_DC_PIN == STATUS_LED_PIN (GPIO2): the ILI9341 library now owns that
// pin and toggles it on every SPI transfer, so there's no manual heartbeat
// LED blink — the onboard IO2 LED just flickers along with display traffic
// (cosmetic only, per CLAUDE.md).
//
// The camera->TFT blit below is the confirmed-working per-pixel path
// (swapBytes + drawRGBBitmap). A bulk writePixels() version was tried for
// speed but never actually verified on hardware before later steps got
// built on top of it — it turned out to render nothing at all. Left as
// per-pixel (flickery but working) until that's revisited and verified
// properly on real hardware before moving on.

// 9600 produced garbled/near-blank output on both GS v 0 and ESC * bit
// image commands, despite plain text and the short ESC @ init working
// fine — trying 19200 next in case long binary payloads are hitting UART
// framing errors that short commands tolerate. Other candidates if this
// doesn't help: 38400, 115200.
static const uint32_t PRINTER_BAUD = 19200;
static const int PRINTER_WIDTH_DOTS = 384; // unconfirmed — adjust if prints come out wrong width
static const uint32_t PREVIEW_HOLD_MS = 1500; // how long to show the frozen capture before printing

static SPIClass tftSPI(FSPI);
static Adafruit_ILI9341 tft(&tftSPI, TFT_DC_PIN, TFT_CS_PIN, TFT_RST_PIN);
static HardwareSerial printerSerial(1);

enum AppState { STATE_LIVE, STATE_CAPTURED, STATE_PRINTING };
static AppState appState = STATE_LIVE;
static camera_fb_t *capturedFrame = nullptr;
static uint32_t capturedAtMs = 0;

static bool lastButtonState = false;
static bool cameraReady = false;
static uint32_t frameCount = 0;
static uint32_t frameGetFailures = 0;

static void drawTestPattern() {
  tft.fillScreen(ILI9341_BLACK);

  // Color bars across the width, like a video test card.
  static const uint16_t colors[] = {
      ILI9341_WHITE, ILI9341_YELLOW, ILI9341_CYAN,   ILI9341_GREEN,
      ILI9341_MAGENTA, ILI9341_RED,  ILI9341_BLUE,
  };
  const int numBars = sizeof(colors) / sizeof(colors[0]);
  const int barWidth = tft.width() / numBars;
  const int barHeight = tft.height() * 2 / 3;
  for (int i = 0; i < numBars; i++) {
    tft.fillRect(i * barWidth, 0, barWidth, barHeight, colors[i]);
  }

  // Border + status text below the bars.
  tft.drawRect(0, 0, tft.width(), tft.height(), ILI9341_WHITE);
  tft.setCursor(4, barHeight + 8);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.println("ILI9341 OK");
  tft.setTextSize(1);
  tft.printf("%dx%d\n", tft.width(), tft.height());
}

static bool initCamera() {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = CAM_Y2_PIN;
  config.pin_d1 = CAM_Y3_PIN;
  config.pin_d2 = CAM_Y4_PIN;
  config.pin_d3 = CAM_Y5_PIN;
  config.pin_d4 = CAM_Y6_PIN;
  config.pin_d5 = CAM_Y7_PIN;
  config.pin_d6 = CAM_Y8_PIN;
  config.pin_d7 = CAM_Y9_PIN;
  config.pin_xclk = CAM_XCLK_PIN;
  config.pin_pclk = CAM_PCLK_PIN;
  config.pin_vsync = CAM_VSYNC_PIN;
  config.pin_href = CAM_HREF_PIN;
  config.pin_sccb_sda = CAM_SIOD_PIN;
  config.pin_sccb_scl = CAM_SIOC_PIN;
  config.pin_pwdn = CAM_PWDN_PIN;
  config.pin_reset = CAM_RESET_PIN;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_RGB565;
  config.frame_size = FRAMESIZE_QVGA; // 320x240 — matches TFT landscape exactly
  config.fb_count = psramFound() ? 2 : 1;
  config.fb_location = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
  config.grab_mode = CAMERA_GRAB_LATEST; // drop stale queued frames, always render the newest

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("CAM: init failed, err=0x%x\n", err);
    return false;
  }

  // OV2640 is mounted upside down on this board — flip vertically.
  sensor_t *sensor = esp_camera_sensor_get();
  sensor->set_vflip(sensor, 1);

  return true;
}

static void blitFrameToTFT(camera_fb_t *fb) {
  // OV2640 RGB565 output is big-endian; ESP32 is little-endian, so a
  // straight cast to uint16_t* scrambles colors — swap in place first.
  uint16_t *pixels = (uint16_t *)fb->buf;
  tft.swapBytes(pixels, fb->len / 2);
  tft.drawRGBBitmap(0, 0, pixels, fb->width, fb->height);
}

static void renderCameraFrame() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    frameGetFailures++;
    return;
  }
  blitFrameToTFT(fb);
  esp_camera_fb_return(fb);
  frameCount++;
}

// The sensor free-runs into its DMA buffer even when idle, so the first
// fb_get() after a period of no captures returns a stale frame from
// before the button press. Throw one away, then grab a fresh one.
static camera_fb_t *captureFreshFrame() {
  camera_fb_t *stale = esp_camera_fb_get();
  if (stale) {
    esp_camera_fb_return(stale);
  }
  return esp_camera_fb_get();
}

static void drawStatusBanner(const char *text) {
  int16_t barHeight = 24;
  tft.fillRect(0, tft.height() - barHeight, tft.width(), barHeight, ILI9341_BLACK);
  tft.setCursor(4, tft.height() - barHeight + 4);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.print(text);
}

static void initPrinter() {
  printerSerial.begin(PRINTER_BAUD, SERIAL_8N1, PRINTER_RX_PIN, PRINTER_TX_PIN);
  pinMode(PRINTER_DTR_PIN, INPUT); // busy/flow-control line, active meaning TBD — just observing for now
  printerSerial.write(0x1B); // ESC
  printerSerial.write('@');  // @ — initialize printer
}

static inline uint8_t clampByte(int v) {
  if (v < 0) return 0;
  if (v > 255) return 255;
  return (uint8_t)v;
}

// Camera RGB565 bytes are big-endian in memory (see blitFrameToTFT).
static inline uint16_t readFramePixel(camera_fb_t *fb, int x, int y) {
  int idx = (y * fb->width + x) * 2;
  return (fb->buf[idx] << 8) | fb->buf[idx + 1];
}

// Sends a grayscale buffer (0=black, 255=white per byte, already dithered
// or otherwise pure black/white) as an ESC/POS raster bit image (GS v 0).
static void sendRasterImage(uint8_t *gray, int width, int height) {
  int widthBytes = width / 8;

  Serial.printf("PRINTER: DTR pin reads %d before send\n", digitalRead(PRINTER_DTR_PIN));

  // ESC/POS raster bit image: GS v 0 m xL xH yL yH [data]
  printerSerial.write(0x1D);
  printerSerial.write('v');
  printerSerial.write('0');
  printerSerial.write((uint8_t)0); // m = normal
  printerSerial.write((uint8_t)(widthBytes & 0xFF));
  printerSerial.write((uint8_t)((widthBytes >> 8) & 0xFF));
  printerSerial.write((uint8_t)(height & 0xFF));
  printerSerial.write((uint8_t)((height >> 8) & 0xFF));

  uint8_t *rowBuf = (uint8_t *)alloca(widthBytes);
  for (int y = 0; y < height; y++) {
    memset(rowBuf, 0, widthBytes);
    for (int x = 0; x < width; x++) {
      if (gray[y * width + x] == 0) { // black dot
        rowBuf[x / 8] |= 0x80 >> (x % 8);
      }
    }
    printerSerial.write(rowBuf, widthBytes);
  }

  printerSerial.write('\n');
  printerSerial.write('\n');
  printerSerial.write('\n'); // feed clear of the tear bar

  Serial.printf("PRINTER: DTR pin reads %d after send\n", digitalRead(PRINTER_DTR_PIN));
  Serial.printf("PRINTER: raster image sent (%dx%d dots)\n", width, height);
}

// Procedurally draws a big smiley face into a gray buffer (0=black,
// 255=white), for isolating printer/raster-command bugs from the
// camera-capture path.
static void drawSmiley(uint8_t *gray, int w, int h) {
  int cx = w / 2, cy = h / 2;
  double faceR = fmin(w, h) * 0.42;
  double stroke = fmax(2.0, faceR / 18.0);
  double eyeR = faceR * 0.10;
  double eyeOffsetX = faceR * 0.38;
  double eyeOffsetY = faceR * 0.25;
  double smileR = faceR * 0.55;
  double smileCy = cy + faceR * 0.05;

  for (int y = 0; y < h; y++) {
    for (int x = 0; x < w; x++) {
      bool black = false;

      double dist = sqrt((double)(x - cx) * (x - cx) + (double)(y - cy) * (y - cy));
      if (fabs(dist - faceR) < stroke) black = true; // face outline

      double ldx = x - (cx - eyeOffsetX), ldy = y - (cy - eyeOffsetY);
      if (sqrt(ldx * ldx + ldy * ldy) < eyeR) black = true; // left eye

      double rdx = x - (cx + eyeOffsetX), rdy = y - (cy - eyeOffsetY);
      if (sqrt(rdx * rdx + rdy * rdy) < eyeR) black = true; // right eye

      double sdx = x - cx, sdy = y - smileCy;
      double sdist = sqrt(sdx * sdx + sdy * sdy);
      if (y > cy && fabs(sdist - smileR) < stroke) black = true; // smile arc

      gray[y * w + x] = black ? 0 : 255;
    }
  }
}

static void printSmileyTest() {
  int size = PRINTER_WIDTH_DOTS; // square test image
  uint8_t *gray = (uint8_t *)heap_caps_malloc(size * size, MALLOC_CAP_SPIRAM);
  if (!gray) {
    Serial.println("PRINTER: out of memory building smiley buffer, skipping print");
    return;
  }
  drawSmiley(gray, size, size);
  sendRasterImage(gray, size, size);
  heap_caps_free(gray);
}

// GS v 0 printed garbled ASCII instead of an image, suggesting the DP-628
// doesn't recognize that raster command at all. This probes the older,
// more universally-supported ESC * bit image command instead, printing
// two labeled solid-black test bars (8-dot and 24-dot density) so it's
// obvious from the paper which command (if either) the printer actually
// understands.
static void printEscStarProbe() {
  const int columns = 64;

  printerSerial.print("ESC*8dot below:\n");
  printerSerial.write(0x1B); // ESC
  printerSerial.write('*');
  printerSerial.write((uint8_t)0); // m=0, 8-dot single density
  printerSerial.write((uint8_t)(columns & 0xFF));
  printerSerial.write((uint8_t)((columns >> 8) & 0xFF));
  for (int i = 0; i < columns; i++) {
    printerSerial.write((uint8_t)0xFF); // solid black column, 8 dots tall
  }
  printerSerial.write('\n');
  printerSerial.write('\n');

  printerSerial.print("ESC*24dot below:\n");
  printerSerial.write(0x1B); // ESC
  printerSerial.write('*');
  printerSerial.write((uint8_t)33); // m=33, 24-dot double density
  printerSerial.write((uint8_t)(columns & 0xFF));
  printerSerial.write((uint8_t)((columns >> 8) & 0xFF));
  for (int i = 0; i < columns * 3; i++) {
    printerSerial.write((uint8_t)0xFF); // solid black column, 24 dots tall (3 bytes/column)
  }
  printerSerial.write('\n');
  printerSerial.write('\n');
  printerSerial.write('\n');

  Serial.println("PRINTER: ESC* probe sent");
}

static void printPhoto(camera_fb_t *fb) {
  int printHeight = PRINTER_WIDTH_DOTS * fb->height / fb->width;

  uint8_t *gray = (uint8_t *)heap_caps_malloc(PRINTER_WIDTH_DOTS * printHeight, MALLOC_CAP_SPIRAM);
  if (!gray) {
    Serial.println("PRINTER: out of memory building print buffer, skipping print");
    return;
  }

  // Nearest-neighbor scale + grayscale (standard luma weights).
  for (int y = 0; y < printHeight; y++) {
    int srcY = y * fb->height / printHeight;
    for (int x = 0; x < PRINTER_WIDTH_DOTS; x++) {
      int srcX = x * fb->width / PRINTER_WIDTH_DOTS;
      uint16_t px = readFramePixel(fb, srcX, srcY);
      uint8_t r = ((px >> 11) & 0x1F) * 255 / 31;
      uint8_t g = ((px >> 5) & 0x3F) * 255 / 63;
      uint8_t b = (px & 0x1F) * 255 / 31;
      gray[y * PRINTER_WIDTH_DOTS + x] = (r * 299 + g * 587 + b * 114) / 1000;
    }
  }

  // Floyd-Steinberg dithering to 1-bit black/white — thermal printers have
  // no grayscale, so dithering keeps detail that a hard threshold would lose.
  for (int y = 0; y < printHeight; y++) {
    for (int x = 0; x < PRINTER_WIDTH_DOTS; x++) {
      int i = y * PRINTER_WIDTH_DOTS + x;
      uint8_t oldPixel = gray[i];
      uint8_t newPixel = oldPixel < 128 ? 0 : 255;
      int16_t error = (int16_t)oldPixel - newPixel;
      gray[i] = newPixel;
      if (x + 1 < PRINTER_WIDTH_DOTS)
        gray[i + 1] = clampByte(gray[i + 1] + error * 7 / 16);
      if (y + 1 < printHeight) {
        if (x > 0)
          gray[i + PRINTER_WIDTH_DOTS - 1] = clampByte(gray[i + PRINTER_WIDTH_DOTS - 1] + error * 3 / 16);
        gray[i + PRINTER_WIDTH_DOTS] = clampByte(gray[i + PRINTER_WIDTH_DOTS] + error * 5 / 16);
        if (x + 1 < PRINTER_WIDTH_DOTS)
          gray[i + PRINTER_WIDTH_DOTS + 1] = clampByte(gray[i + PRINTER_WIDTH_DOTS + 1] + error * 1 / 16);
      }
    }
  }

  sendRasterImage(gray, PRINTER_WIDTH_DOTS, printHeight);
  heap_caps_free(gray);
}

void setup() {
  Serial.begin(115200);
  delay(300); // let the CH343 USB-UART bridge settle

  pinMode(BUTTON1_PIN, INPUT); // module has its own pull resistor

  Serial.println("Thermal Printer Camera — booting");
  Serial.printf("PSRAM size: %u bytes\n", ESP.getPsramSize());

  tftSPI.begin(TFT_SCK_PIN, -1 /* MISO not wired, write-only */, TFT_MOSI_PIN, TFT_CS_PIN);
  tft.begin();
  tft.setRotation(1); // landscape
  drawTestPattern();
  Serial.println("TFT init + test pattern drawn");

  cameraReady = initCamera();
  Serial.printf("CAM: init %s\n", cameraReady ? "OK" : "FAILED");

  initPrinter();
  Serial.println("PRINTER: UART init done");
}

void loop() {
  static uint32_t lastBeat = 0;

  bool buttonState = digitalRead(BUTTON1_PIN) == HIGH;
  bool buttonPressedEdge = buttonState && !lastButtonState;
  lastButtonState = buttonState;

  switch (appState) {
  case STATE_LIVE:
    if (cameraReady) {
      renderCameraFrame();
    }
    // printer-test branch: button press probes ESC * bit-image support,
    // since GS v 0 (printSmileyTest) came out as garbled text instead of
    // an image — the printer likely doesn't recognize that command.
    if (buttonPressedEdge) {
      Serial.println("PRINTER TEST: ESC* probe");
      printEscStarProbe();
    }
    break;

  case STATE_CAPTURED:
    if (millis() - capturedAtMs >= PREVIEW_HOLD_MS) {
      drawStatusBanner("PRINTING...");
      appState = STATE_PRINTING;
    }
    break;

  case STATE_PRINTING:
    printPhoto(capturedFrame);
    esp_camera_fb_return(capturedFrame);
    capturedFrame = nullptr;
    Serial.println("STATE: print done, back to live preview");
    appState = STATE_LIVE;
    break;
  }

  uint32_t now = millis();
  if (now - lastBeat >= 1000) {
    lastBeat = now;
    Serial.printf("heartbeat t=%lu ms, frames=%lu, fb_get failures=%lu\n", now,
                  frameCount, frameGetFailures);
  }
}

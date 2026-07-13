#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <esp_camera.h>
#include <esp_heap_caps.h>

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

static const uint32_t PRINTER_BAUD = 9600;
static const int PRINTER_WIDTH_DOTS = 384; // unconfirmed — adjust if prints come out wrong width
static const int PRINTER_WIDTH_BYTES = PRINTER_WIDTH_DOTS / 8;
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

  Serial.printf("PRINTER: DTR pin reads %d before send\n", digitalRead(PRINTER_DTR_PIN));

  // ESC/POS raster bit image: GS v 0 m xL xH yL yH [data]
  printerSerial.write(0x1D);
  printerSerial.write('v');
  printerSerial.write('0');
  printerSerial.write((uint8_t)0); // m = normal
  printerSerial.write((uint8_t)(PRINTER_WIDTH_BYTES & 0xFF));
  printerSerial.write((uint8_t)((PRINTER_WIDTH_BYTES >> 8) & 0xFF));
  printerSerial.write((uint8_t)(printHeight & 0xFF));
  printerSerial.write((uint8_t)((printHeight >> 8) & 0xFF));

  uint8_t rowBuf[PRINTER_WIDTH_BYTES];
  for (int y = 0; y < printHeight; y++) {
    memset(rowBuf, 0, sizeof(rowBuf));
    for (int x = 0; x < PRINTER_WIDTH_DOTS; x++) {
      if (gray[y * PRINTER_WIDTH_DOTS + x] == 0) { // black dot
        rowBuf[x / 8] |= 0x80 >> (x % 8);
      }
    }
    printerSerial.write(rowBuf, PRINTER_WIDTH_BYTES);
  }

  printerSerial.write('\n');
  printerSerial.write('\n');
  printerSerial.write('\n'); // feed clear of the tear bar

  Serial.printf("PRINTER: DTR pin reads %d after send\n", digitalRead(PRINTER_DTR_PIN));
  Serial.printf("PRINTER: photo printed (%dx%d dots)\n", PRINTER_WIDTH_DOTS, printHeight);

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
    if (buttonPressedEdge && cameraReady) {
      capturedFrame = captureFreshFrame();
      if (capturedFrame) {
        blitFrameToTFT(capturedFrame);
        capturedAtMs = millis();
        appState = STATE_CAPTURED;
        Serial.println("STATE: captured, showing preview");
      } else {
        Serial.println("STATE: capture failed, staying live");
      }
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
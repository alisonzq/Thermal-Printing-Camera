#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <esp_camera.h>
#include <esp_heap_caps.h>
#include <soc/gpio_sig_map.h> // FSPID_OUT_IDX, for mirroring MOSI onto the touch DIN pin

#include "pins.h"

// The DP-628 printer has been removed and its pins (19/20/21) now belong to
// the touch controller. Printer code is kept below for a future printer but
// compiled out.
#define ENABLE_PRINTER 0
#if ENABLE_PRINTER
#error "GPIO19/20/21 are now the touch controller — assign new printer pins in pins.h first"
#endif

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
// Step 7: 3 extra buttons drive a settings menu (preview hold time,
// dither threshold, vertical flip) and a session-only gallery of the
// last few captures (kept in PSRAM, reprintable). MENU always backs out
// to live camera view from anywhere in the UI.
// Step 8: printer removed, XPT2046 touch added on its old pins. Touch works
// alongside the physical buttons: tap live view = shutter, tap a menu row
// to select / tap again to open, tap left/right in the gallery to browse.
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
static const uint32_t LONG_PRESS_MS = 500; // hold time for BUTTON_DOWNSELECT to register as select/confirm

static const int GALLERY_CAPACITY = 6; // last N captures kept in PSRAM, ring buffer, lost on reset
static const int GALLERY_W = 320;      // matches FRAMESIZE_QVGA / TFT resolution
static const int GALLERY_H = 240;
static const int GALLERY_FRAME_BYTES = GALLERY_W * GALLERY_H * 2; // RGB565

// --- Touch (XPT2046) ---
// Calibration: every tap logs "TOUCH raw=(..) screen=(..)" to Serial. If taps
// land in the wrong place, tap the four corners, read the raw values, and
// adjust these. If moving your finger right moves the point down, flip
// TOUCH_SWAP_XY; if it moves the wrong way along an axis, flip that axis's
// INVERT flag.
static const uint32_t TOUCH_SPI_HZ = 2000000;  // XPT2046 max is ~2.5MHz
static const int TOUCH_Z_THRESHOLD = 400;      // pressure below this = not touched
static const int TOUCH_RAW_X_MIN = 300, TOUCH_RAW_X_MAX = 3800; // raw range along screen X
static const int TOUCH_RAW_Y_MIN = 300, TOUCH_RAW_Y_MAX = 3800; // raw range along screen Y
static const bool TOUCH_SWAP_XY = true;   // panel is used in landscape (rotation 1)
static const bool TOUCH_INVERT_X = false;
static const bool TOUCH_INVERT_Y = false;
static const uint32_t TOUCH_RELEASE_MS = 80; // ignore brief dropouts mid-press

// Menu layout, shared by drawMenu() and touch hit-testing. 8 rows of 23px
// fit between the header and the footer hint on the 240px-tall screen.
static const int MENU_TOP = 36;
static const int MENU_ROW_H = 23;

static SPIClass tftSPI(FSPI);
static Adafruit_ILI9341 tft(&tftSPI, TFT_DC_PIN, TFT_CS_PIN, TFT_RST_PIN);
#if ENABLE_PRINTER
static HardwareSerial printerSerial(1);
#endif
static sensor_t *camSensor = nullptr;

enum AppState { STATE_LIVE, STATE_CAPTURED, STATE_PRINTING };
static AppState appState = STATE_LIVE;
static camera_fb_t *capturedFrame = nullptr;
static uint32_t capturedAtMs = 0;

enum UIMode { MODE_CAMERA, MODE_MENU, MODE_GALLERY };
static UIMode uiMode = MODE_CAMERA;

enum MenuItem {
  MENU_PREVIEW_HOLD,
  MENU_DITHER_THRESHOLD,
  MENU_MANUAL_EXPOSURE,
  MENU_EXPOSURE_VALUE,
  MENU_GAIN_VALUE,
  MENU_FLIP_VERTICAL,
  MENU_VIEW_GALLERY,
  MENU_EXIT,
  MENU_ITEM_COUNT
};
static int menuSelectedIndex = 0;
static bool menuEditing = false;

// Settings, adjustable from the menu.
static uint32_t previewHoldMs = 1500; // how long to show the frozen capture before printing
static uint8_t ditherThreshold = 128; // Floyd-Steinberg black/white cutoff
static bool flipVertical = true;      // OV2640 is mounted upside down on this board

// Manual exposure/gain — sensor defaults to auto (AEC/AGC) at boot.
// Enabling this calls set_exposure_ctrl(0)/set_gain_ctrl(0) to hand control
// over to the values below instead. Ranges match the sensor_t API:
// AEC value 0-1200, AGC gain 0-30 (see esp32-camera sensor.h).
static bool manualExposureEnabled = false;
static int manualExposureValue = 300;
static int manualGainValue = 0;

struct GalleryImage {
  uint8_t *pixels; // raw camera RGB565 bytes (big-endian, as captured), GALLERY_FRAME_BYTES long
  bool valid;
};
static GalleryImage gallery[GALLERY_CAPACITY];
static bool galleryAllocOk = false;
static int galleryCount = 0;     // number of filled slots, up to GALLERY_CAPACITY
static int galleryNextWrite = 0; // ring index for the next capture
static int galleryViewIndex = 0; // 0 = newest, increasing = older

static bool lastShutterState = false;
static bool lastMenuState = false;
static bool lastUpState = false;
static bool lastDownSelectState = false;
static uint32_t downSelectPressStartMs = 0;
static bool lastTouchState = false;
static uint32_t lastTouchSeenMs = 0;

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

  camSensor = esp_camera_sensor_get();
  camSensor->set_vflip(camSensor, flipVertical ? 1 : 0);

  return true;
}

static void blitFrameToTFT(camera_fb_t *fb) {
  // OV2640 RGB565 output is big-endian; ESP32 is little-endian, so a
  // straight cast to uint16_t* scrambles colors — swap in place first.
  uint16_t *pixels = (uint16_t *)fb->buf;
  tft.swapBytes(pixels, fb->len / 2);
  tft.drawRGBBitmap(0, 0, pixels, fb->width, fb->height);
}

// Same idea as blitFrameToTFT, but for a standalone raw buffer (gallery
// storage) that needs to survive being drawn more than once — swap back
// afterwards so the stored copy stays in original camera byte order.
static void blitRawFrameToTFT(uint8_t *buf, int len, int w, int h) {
  uint16_t *pixels = (uint16_t *)buf;
  tft.swapBytes(pixels, len / 2);
  tft.drawRGBBitmap(0, 0, pixels, w, h);
  tft.swapBytes(pixels, len / 2);
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

// --- Touch (XPT2046) ---
// Shares the TFT's SPI bus (same SCK, MISO on TOUCH_DO_PIN, MOSI mirrored
// onto TOUCH_DIN_PIN in setup()). Only the chip whose CS is low listens, so
// the two never collide. Polled once per loop — T_IRQ isn't wired.

// Last raw readings, printed once a second with the heartbeat for debugging.
static int touchDbgZ1 = 0, touchDbgZ2 = 0, touchDbgX = 0, touchDbgY = 0;

static uint16_t touchRead12(uint8_t cmd) {
  tftSPI.transfer(cmd);
  return tftSPI.transfer16(0) >> 3; // 12-bit result, MSB first after one busy bit
}

static bool readTouchRaw(int &rawX, int &rawY) {
  tftSPI.beginTransaction(SPISettings(TOUCH_SPI_HZ, MSBFIRST, SPI_MODE0));
  digitalWrite(TOUCH_CS_PIN, LOW);

  // Pressure from the two Z measurements — light or no contact reads low.
  int z1 = touchRead12(0xB1);
  int z2 = touchRead12(0xC1);
  int z = z1 + 4095 - z2;
  // z1 == z2 == 0 means MISO is stuck low (controller not answering), which
  // would otherwise compute as maximum pressure and look permanently pressed.
  if (z1 == 0 && z2 == 0) z = 0;

  long sumX = 0, sumY = 0;
  const int samples = 4; // average a few readings — resistive touch is noisy
  if (z >= TOUCH_Z_THRESHOLD) {
    for (int i = 0; i < samples; i++) {
      sumX += touchRead12(0xD1);
      sumY += touchRead12(0x91);
    }
  }
  touchRead12(0xD0); // power down between reads

  digitalWrite(TOUCH_CS_PIN, HIGH);
  tftSPI.endTransaction();

  touchDbgZ1 = z1;
  touchDbgZ2 = z2;
  touchDbgX = z >= TOUCH_Z_THRESHOLD ? sumX / samples : -1;
  touchDbgY = z >= TOUCH_Z_THRESHOLD ? sumY / samples : -1;

  if (z < TOUCH_Z_THRESHOLD) return false;
  rawX = sumX / samples;
  rawY = sumY / samples;
  return true;
}

// Converts a raw reading into TFT pixel coordinates (landscape, 320x240).
static bool readTouchScreen(int &screenX, int &screenY, int &rawX, int &rawY) {
  if (!readTouchRaw(rawX, rawY)) return false;
  int alongX = TOUCH_SWAP_XY ? rawY : rawX;
  int alongY = TOUCH_SWAP_XY ? rawX : rawY;
  screenX = constrain(map(alongX, TOUCH_RAW_X_MIN, TOUCH_RAW_X_MAX, 0, tft.width() - 1), 0, tft.width() - 1);
  screenY = constrain(map(alongY, TOUCH_RAW_Y_MIN, TOUCH_RAW_Y_MAX, 0, tft.height() - 1), 0, tft.height() - 1);
  if (TOUCH_INVERT_X) screenX = tft.width() - 1 - screenX;
  if (TOUCH_INVERT_Y) screenY = tft.height() - 1 - screenY;
  return true;
}

// Which menu row a screen Y coordinate falls in, or -1 for header/footer.
static int menuRowAt(int y) {
  if (y < MENU_TOP - 2) return -1;
  int row = (y - (MENU_TOP - 2)) / MENU_ROW_H;
  return row < MENU_ITEM_COUNT ? row : -1;
}

// --- Gallery (session-only, PSRAM ring buffer) ---

static bool allocateGalleryBuffers() {
  for (int i = 0; i < GALLERY_CAPACITY; i++) {
    gallery[i].pixels = (uint8_t *)heap_caps_malloc(GALLERY_FRAME_BYTES, MALLOC_CAP_SPIRAM);
    gallery[i].valid = false;
    if (!gallery[i].pixels) {
      Serial.printf("GALLERY: alloc failed at slot %d, gallery disabled\n", i);
      return false;
    }
  }
  return true;
}

static GalleryImage &galleryImageAt(int viewIndex) {
  int physical = (galleryNextWrite - 1 - viewIndex + GALLERY_CAPACITY * 2) % GALLERY_CAPACITY;
  return gallery[physical];
}

// Copies the raw (pre-swap) camera bytes, so both blitRawFrameToTFT and
// printPhoto — which both expect the original big-endian camera format —
// can read gallery images the same way they read a live capture.
static void pushToGallery(camera_fb_t *fb) {
  if (!galleryAllocOk) return;
  GalleryImage &slot = gallery[galleryNextWrite];
  size_t copyLen = min((size_t)fb->len, (size_t)GALLERY_FRAME_BYTES);
  memcpy(slot.pixels, fb->buf, copyLen);
  slot.valid = true;
  galleryNextWrite = (galleryNextWrite + 1) % GALLERY_CAPACITY;
  if (galleryCount < GALLERY_CAPACITY) galleryCount++;
}

// --- Menu + gallery screens ---

static const char *menuItemLabel(int idx) {
  switch (idx) {
    case MENU_PREVIEW_HOLD: return "Preview Hold";
    case MENU_DITHER_THRESHOLD: return "Dither Thresh";
    case MENU_MANUAL_EXPOSURE: return "Manual Exp/Gain";
    case MENU_EXPOSURE_VALUE: return "Exposure";
    case MENU_GAIN_VALUE: return "Gain";
    case MENU_FLIP_VERTICAL: return "Flip Vertical";
    case MENU_VIEW_GALLERY: return "View Gallery";
    case MENU_EXIT: return "Exit";
    default: return "";
  }
}

static void menuItemValue(int idx, char *buf, size_t bufLen) {
  switch (idx) {
    case MENU_PREVIEW_HOLD: snprintf(buf, bufLen, "%lums", (unsigned long)previewHoldMs); break;
    case MENU_DITHER_THRESHOLD: snprintf(buf, bufLen, "%d", ditherThreshold); break;
    case MENU_MANUAL_EXPOSURE: snprintf(buf, bufLen, "%s", manualExposureEnabled ? "ON" : "OFF"); break;
    case MENU_EXPOSURE_VALUE: snprintf(buf, bufLen, "%d%s", manualExposureValue, manualExposureEnabled ? "" : " (auto)"); break;
    case MENU_GAIN_VALUE: snprintf(buf, bufLen, "%d%s", manualGainValue, manualExposureEnabled ? "" : " (auto)"); break;
    case MENU_FLIP_VERTICAL: snprintf(buf, bufLen, "%s", flipVertical ? "ON" : "OFF"); break;
    case MENU_VIEW_GALLERY: snprintf(buf, bufLen, "%d photo%s", galleryCount, galleryCount == 1 ? "" : "s"); break;
    default: buf[0] = '\0'; break;
  }
}

static void drawMenu() {
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(8, 6);
  tft.print("SETTINGS");
  tft.drawFastHLine(0, 28, tft.width(), ILI9341_WHITE);

  char valueBuf[24];
  for (int i = 0; i < MENU_ITEM_COUNT; i++) {
    int y = MENU_TOP + i * MENU_ROW_H;
    bool selected = (i == menuSelectedIndex);
    if (selected) {
      uint16_t bg = menuEditing ? ILI9341_ORANGE : ILI9341_BLUE;
      tft.fillRect(0, y - 2, tft.width(), MENU_ROW_H - 1, bg);
    }
    tft.setCursor(8, y + 2);
    tft.setTextColor(ILI9341_WHITE);
    tft.print(menuItemLabel(i));
    menuItemValue(i, valueBuf, sizeof(valueBuf));
    if (valueBuf[0]) {
      int16_t x1, y1;
      uint16_t w, h;
      tft.getTextBounds(valueBuf, 0, 0, &x1, &y1, &w, &h);
      tft.setCursor(tft.width() - w - 8, y + 2);
      tft.print(valueBuf);
    }
  }

  tft.setTextSize(1);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setCursor(8, tft.height() - 12);
  tft.print(menuEditing ? "Tap row right +, left -. Tap elsewhere: done"
                        : "Tap a row to select, tap again to open");
}

static void drawGallery() {
  tft.fillScreen(ILI9341_BLACK);

  if (galleryCount == 0) {
    tft.setTextSize(2);
    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(16, tft.height() / 2 - 12);
    tft.print("No photos yet");
    tft.setTextSize(1);
    tft.setCursor(16, tft.height() / 2 + 12);
    tft.print("Capture one from live view first");
  } else {
    GalleryImage &img = galleryImageAt(galleryViewIndex);
    blitRawFrameToTFT(img.pixels, GALLERY_FRAME_BYTES, GALLERY_W, GALLERY_H);
  }

  tft.fillRect(0, tft.height() - 20, tft.width(), 20, ILI9341_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(4, tft.height() - 14);
  if (galleryCount > 0) {
    tft.printf("%d/%d", galleryViewIndex + 1, galleryCount);
  }
  tft.setCursor(tft.width() - 162, tft.height() - 14);
  tft.print("Tap left/right to browse");
}

static void menuMoveSelection(int delta) {
  menuSelectedIndex = (menuSelectedIndex + delta + MENU_ITEM_COUNT) % MENU_ITEM_COUNT;
}

static void menuAdjustValue(int direction) {
  switch (menuSelectedIndex) {
    case MENU_PREVIEW_HOLD:
      previewHoldMs = (uint32_t)constrain((int)previewHoldMs + direction * 250, 500, 5000);
      break;
    case MENU_DITHER_THRESHOLD:
      ditherThreshold = (uint8_t)constrain((int)ditherThreshold + direction * 8, 32, 224);
      break;
    case MENU_EXPOSURE_VALUE:
      manualExposureValue = constrain(manualExposureValue + direction * 50, 0, 1200);
      if (camSensor && manualExposureEnabled) camSensor->set_aec_value(camSensor, manualExposureValue);
      break;
    case MENU_GAIN_VALUE:
      manualGainValue = constrain(manualGainValue + direction * 2, 0, 30);
      if (camSensor && manualExposureEnabled) camSensor->set_agc_gain(camSensor, manualGainValue);
      break;
    default:
      break;
  }
}

// Called on a long-press of SELECT while not already editing a value.
static void menuActivateSelected() {
  switch (menuSelectedIndex) {
    case MENU_PREVIEW_HOLD:
    case MENU_DITHER_THRESHOLD:
    case MENU_EXPOSURE_VALUE:
    case MENU_GAIN_VALUE:
      menuEditing = true;
      break;
    case MENU_MANUAL_EXPOSURE:
      manualExposureEnabled = !manualExposureEnabled;
      if (camSensor) {
        // enable=1 means the sensor's own auto-exposure/auto-gain runs;
        // enable=0 hands control to our manual aec/agc values below.
        camSensor->set_exposure_ctrl(camSensor, manualExposureEnabled ? 0 : 1);
        camSensor->set_gain_ctrl(camSensor, manualExposureEnabled ? 0 : 1);
        if (manualExposureEnabled) {
          camSensor->set_aec_value(camSensor, manualExposureValue);
          camSensor->set_agc_gain(camSensor, manualGainValue);
        }
      }
      break;
    case MENU_FLIP_VERTICAL:
      flipVertical = !flipVertical;
      if (camSensor) camSensor->set_vflip(camSensor, flipVertical ? 1 : 0);
      break;
    case MENU_VIEW_GALLERY:
      uiMode = MODE_GALLERY;
      galleryViewIndex = 0;
      break;
    case MENU_EXIT:
      uiMode = MODE_CAMERA;
      appState = STATE_LIVE;
      break;
  }
}

#if ENABLE_PRINTER
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
static inline uint16_t readFramePixel(const uint8_t *buf, int width, int x, int y) {
  int idx = (y * width + x) * 2;
  return (buf[idx] << 8) | buf[idx + 1];
}

static void printPhoto(const uint8_t *buf, int width, int height) {
  int printHeight = PRINTER_WIDTH_DOTS * height / width;

  uint8_t *gray = (uint8_t *)heap_caps_malloc(PRINTER_WIDTH_DOTS * printHeight, MALLOC_CAP_SPIRAM);
  if (!gray) {
    Serial.println("PRINTER: out of memory building print buffer, skipping print");
    return;
  }

  // Nearest-neighbor scale + grayscale (standard luma weights).
  for (int y = 0; y < printHeight; y++) {
    int srcY = y * height / printHeight;
    for (int x = 0; x < PRINTER_WIDTH_DOTS; x++) {
      int srcX = x * width / PRINTER_WIDTH_DOTS;
      uint16_t px = readFramePixel(buf, width, srcX, srcY);
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
      uint8_t newPixel = oldPixel < ditherThreshold ? 0 : 255;
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
#endif // ENABLE_PRINTER

void setup() {
  Serial.begin(115200);
  delay(300); // let the CH343 USB-UART bridge settle

  // Bare buttons wired GPIO->GND only, using the ESP32's internal pull-up.
  // Idle reads HIGH, pressed reads LOW (opposite polarity from the old
  // button-module wiring) — see the == LOW checks in loop() below.
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON_MENU_PIN, INPUT_PULLUP);
  pinMode(BUTTON_UP_PIN, INPUT_PULLUP);
  pinMode(BUTTON_DOWNSELECT_PIN, INPUT_PULLUP);

  Serial.println("Thermal Printer Camera — booting");
  Serial.printf("PSRAM size: %u bytes\n", ESP.getPsramSize());

  // Keep the touch controller deselected so it ignores display traffic.
  pinMode(TOUCH_CS_PIN, OUTPUT);
  digitalWrite(TOUCH_CS_PIN, HIGH);

  // MISO is the touch controller's T_DO — the display itself is still write-only.
  tftSPI.begin(TFT_SCK_PIN, TOUCH_DO_PIN, TFT_MOSI_PIN, TFT_CS_PIN);
  tft.begin();
  tft.setRotation(1); // landscape
  drawTestPattern();
  Serial.println("TFT init + test pattern drawn");

  // T_DIN is wired to its own pin rather than sharing the display's MOSI.
  // The GPIO matrix can route one SPI output signal to several pins, so
  // drive GPIO21 with the same MOSI signal — both chips see identical data,
  // and only the one whose CS is low acts on it.
  pinMode(TOUCH_DIN_PIN, OUTPUT);
  pinMatrixOutAttach(TOUCH_DIN_PIN, FSPID_OUT_IDX, false, false);
  Serial.println("TOUCH: XPT2046 on shared SPI bus");

  cameraReady = initCamera();
  Serial.printf("CAM: init %s\n", cameraReady ? "OK" : "FAILED");

  galleryAllocOk = allocateGalleryBuffers();
  Serial.printf("GALLERY: PSRAM alloc %s (%d slots)\n", galleryAllocOk ? "OK" : "FAILED", GALLERY_CAPACITY);

#if ENABLE_PRINTER
  initPrinter();
  Serial.println("PRINTER: UART init done");
#endif
}

void loop() {
  static uint32_t lastBeat = 0;

  // INPUT_PULLUP: pin reads LOW while pressed, HIGH at rest — inverted from
  // the old active-high button-module wiring.
  bool shutterState = digitalRead(BUTTON1_PIN) == LOW;
  bool shutterEdge = shutterState && !lastShutterState;
  lastShutterState = shutterState;

  bool menuState = digitalRead(BUTTON_MENU_PIN) == LOW;
  bool menuEdge = menuState && !lastMenuState;
  lastMenuState = menuState;

  bool upState = digitalRead(BUTTON_UP_PIN) == LOW;
  bool upEdge = upState && !lastUpState;
  lastUpState = upState;

  bool downSelectState = digitalRead(BUTTON_DOWNSELECT_PIN) == LOW;
  bool downSelectRisingEdge = downSelectState && !lastDownSelectState;
  bool downSelectFallingEdge = !downSelectState && lastDownSelectState;
  if (downSelectRisingEdge) downSelectPressStartMs = millis();
  bool downSelectShort = false;
  bool downSelectLong = false;
  if (downSelectFallingEdge) {
    uint32_t heldMs = millis() - downSelectPressStartMs;
    if (heldMs >= LONG_PRESS_MS) downSelectLong = true;
    else downSelectShort = true;
  }
  lastDownSelectState = downSelectState;

  // Touch: a "tap" fires once when a finger first lands. Brief dropouts
  // mid-press (common on resistive panels) are bridged by TOUCH_RELEASE_MS
  // so one press doesn't register as several taps.
  int touchX = 0, touchY = 0, touchRawX = 0, touchRawY = 0;
  bool touchNow = readTouchScreen(touchX, touchY, touchRawX, touchRawY);
  if (touchNow) lastTouchSeenMs = millis();
  bool touchState = touchNow || (millis() - lastTouchSeenMs < TOUCH_RELEASE_MS);
  bool touchTap = touchState && !lastTouchState && touchNow;
  lastTouchState = touchState;
  if (touchTap) {
    Serial.printf("TOUCH raw=(%d,%d) screen=(%d,%d)\n", touchRawX, touchRawY, touchX, touchY);
  }

  // MENU button is the master mode toggle: from live camera it opens the
  // menu, and from anywhere in the menu/gallery it goes straight back to
  // live camera view. Only opens from STATE_LIVE so it can't interrupt an
  // in-flight capture/print.
  if (menuEdge) {
    if (uiMode == MODE_CAMERA && appState == STATE_LIVE) {
      uiMode = MODE_MENU;
      menuSelectedIndex = 0;
      menuEditing = false;
      drawMenu();
    } else if (uiMode == MODE_MENU || uiMode == MODE_GALLERY) {
      uiMode = MODE_CAMERA;
      appState = STATE_LIVE;
    }
  }

  if (uiMode == MODE_MENU) {
    bool needsRedraw = false;
    if (upEdge) {
      if (menuEditing) menuAdjustValue(1);
      else menuMoveSelection(-1);
      needsRedraw = true;
    }
    if (downSelectShort) {
      if (menuEditing) menuAdjustValue(-1);
      else menuMoveSelection(1);
      needsRedraw = true;
    }
    if (downSelectLong) {
      if (menuEditing) menuEditing = false;
      else menuActivateSelected(); // may switch uiMode to MODE_GALLERY or MODE_CAMERA
      needsRedraw = true;
    }
    if (touchTap) {
      int row = menuRowAt(touchY);
      if (menuEditing) {
        // While editing: right half of the row = +, left half = -, anywhere else = done.
        if (row == menuSelectedIndex) menuAdjustValue(touchX >= tft.width() / 2 ? 1 : -1);
        else menuEditing = false;
      } else if (row >= 0) {
        // First tap selects a row, tapping the selected row opens it.
        if (row == menuSelectedIndex) menuActivateSelected();
        else menuSelectedIndex = row;
      }
      needsRedraw = true;
    }
    if (needsRedraw) {
      if (uiMode == MODE_MENU) drawMenu();
      else if (uiMode == MODE_GALLERY) drawGallery();
      // MODE_CAMERA: live view redraws itself every loop, nothing to do here
    }
  } else if (uiMode == MODE_GALLERY) {
    bool needsRedraw = false;
    if (upEdge && galleryCount > 0) {
      galleryViewIndex = min(galleryViewIndex + 1, galleryCount - 1);
      needsRedraw = true;
    }
    if (downSelectShort && galleryCount > 0) {
      galleryViewIndex = max(galleryViewIndex - 1, 0);
      needsRedraw = true;
    }
#if ENABLE_PRINTER
    if (downSelectLong && galleryCount > 0) {
      drawStatusBanner("PRINTING...");
      GalleryImage &img = galleryImageAt(galleryViewIndex);
      printPhoto(img.pixels, GALLERY_W, GALLERY_H);
      needsRedraw = true;
    }
#endif
    if (touchTap && galleryCount > 0) {
      // Left half = newer photo, right half = older.
      if (touchX < tft.width() / 2) galleryViewIndex = max(galleryViewIndex - 1, 0);
      else galleryViewIndex = min(galleryViewIndex + 1, galleryCount - 1);
      needsRedraw = true;
    }
    if (needsRedraw) drawGallery();
  } else { // MODE_CAMERA
    switch (appState) {
    case STATE_LIVE:
      if (cameraReady) {
        renderCameraFrame();
      }
      if ((shutterEdge || touchTap) && cameraReady) { // tap anywhere on live view = shutter
        capturedFrame = captureFreshFrame();
        if (capturedFrame) {
          pushToGallery(capturedFrame); // raw copy before blit swaps bytes in place
          blitFrameToTFT(capturedFrame);
          drawStatusBanner(galleryAllocOk ? "SAVED TO GALLERY" : "CAPTURED");
          capturedAtMs = millis();
          appState = STATE_CAPTURED;
          Serial.println("STATE: captured, showing preview");
        } else {
          Serial.println("STATE: capture failed, staying live");
        }
      }
      break;

    case STATE_CAPTURED:
      if (millis() - capturedAtMs >= previewHoldMs) {
#if ENABLE_PRINTER
        drawStatusBanner("PRINTING...");
#endif
        appState = STATE_PRINTING;
      }
      break;

    case STATE_PRINTING: // with no printer, this just releases the frame and returns to live view
#if ENABLE_PRINTER
      printPhoto(capturedFrame->buf, capturedFrame->width, capturedFrame->height);
#endif
      esp_camera_fb_return(capturedFrame);
      capturedFrame = nullptr;
      Serial.println("STATE: preview done, back to live view");
      appState = STATE_LIVE;
      break;
    }
  }

  uint32_t now = millis();
  if (now - lastBeat >= 1000) {
    lastBeat = now;
    Serial.printf("heartbeat t=%lu ms, frames=%lu, fb_get failures=%lu\n", now,
                  frameCount, frameGetFailures);
    Serial.printf("TOUCH dbg z1=%d z2=%d x=%d y=%d touched=%d%s\n", touchDbgZ1, touchDbgZ2,
                  touchDbgX, touchDbgY, lastTouchState,
                  (touchDbgZ1 == 0 && touchDbgZ2 == 0) ? "  <- controller not answering, check wiring" : "");
  }
}

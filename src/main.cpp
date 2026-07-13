#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <esp_camera.h>

#include "pins.h"

// Step 1: blink/serial-print bring-up (heartbeat + shutter button) — done.
// Step 2: TFT alone, SPI init + test pattern — done.
// Step 3: camera alone, JPEG capture dumped over serial — done.
// Step 4: camera frames rendering live to the TFT — done.
// Step 5 of the build order in CLAUDE.md: bring up the printer UART
// independently — send a raw ESC/POS test line, before wiring it into the
// capture/print flow (step 6). Shutter button press triggers a test print,
// decoupled from the camera for now.
//
// DP-628 baud rate is NOT confirmed from an official datasheet (see
// CLAUDE.md known gotchas) — 9600 8N1 is the common default for these TTL
// thermal printer modules; if the test line prints garbage or nothing,
// try 19200 or 115200 next.
//
// TFT_DC_PIN == STATUS_LED_PIN (GPIO2): the ILI9341 library now owns that
// pin and toggles it on every SPI transfer, so there's no manual heartbeat
// LED blink — the onboard IO2 LED just flickers along with display traffic
// (cosmetic only, per CLAUDE.md).

static const uint32_t PRINTER_BAUD = 9600;

static SPIClass tftSPI(FSPI);
static Adafruit_ILI9341 tft(&tftSPI, TFT_DC_PIN, TFT_CS_PIN, TFT_RST_PIN);
static HardwareSerial printerSerial(1);

static bool lastButtonState = false;
static bool cameraReady = false;
static uint32_t testPrintCount = 0;

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

static void renderCameraFrame() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    return;
  }
  // drawRGBBitmap() resets the address window on every pixel, which is
  // far too slow for a live feed (visible as flicker/tearing). Set the
  // window once and burst the whole frame instead. Camera RGB565 bytes
  // are already in the big-endian order the SPI panel wants, so pass
  // bigEndian=true and skip a manual byte-swap pass.
  tft.startWrite();
  tft.setAddrWindow(0, 0, fb->width, fb->height);
  tft.writePixels((uint16_t *)fb->buf, fb->width * fb->height, true, true);
  tft.endWrite();
  esp_camera_fb_return(fb);
}

static void initPrinter() {
  printerSerial.begin(PRINTER_BAUD, SERIAL_8N1, PRINTER_RX_PIN, PRINTER_TX_PIN);
  pinMode(PRINTER_DTR_PIN, INPUT); // busy/flow-control line, active meaning TBD — just observing for now
  printerSerial.write(0x1B); // ESC
  printerSerial.write('@');  // @ — initialize printer
}

static void sendTestPrint() {
  testPrintCount++;
  Serial.printf("PRINTER: DTR pin reads %d before send\n", digitalRead(PRINTER_DTR_PIN));

  printerSerial.printf("Thermal Printer Camera - UART test #%lu\n", testPrintCount);
  printerSerial.print("ESC/POS link OK\n");
  printerSerial.write('\n');
  printerSerial.write('\n');
  printerSerial.write('\n'); // feed a few lines so the line is clear of the tear bar

  Serial.printf("PRINTER: DTR pin reads %d after send\n", digitalRead(PRINTER_DTR_PIN));
  Serial.println("PRINTER: test line sent");
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

  if (cameraReady) {
    renderCameraFrame();
  }

  uint32_t now = millis();
  if (now - lastBeat >= 1000) {
    lastBeat = now;
    Serial.printf("heartbeat t=%lu ms\n", now);
  }

  bool buttonState = digitalRead(BUTTON1_PIN) == HIGH;
  if (buttonState != lastButtonState) {
    lastButtonState = buttonState;
    Serial.printf("button1 %s\n", buttonState ? "PRESSED" : "released");
    if (buttonState) {
      sendTestPrint();
    }
  }
}

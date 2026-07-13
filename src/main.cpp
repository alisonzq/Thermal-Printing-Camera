#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <esp_camera.h>
#include <base64.h>

#include "pins.h"

// Step 1: blink/serial-print bring-up (heartbeat + shutter button) — done.
// Step 2: TFT alone, SPI init + test pattern — done.
// Step 3 of the build order in CLAUDE.md: bring up the camera alone.
// Shutter button press triggers a JPEG capture, dumped as base64 over
// serial (marked with CAM:BEGIN/CAM:END) so it can be decoded and viewed
// on a PC without SD or the TFT-render step (step 4).
//
// TFT_DC_PIN == STATUS_LED_PIN (GPIO2): the ILI9341 library now owns that
// pin and toggles it on every SPI transfer, so there's no manual heartbeat
// LED blink — the onboard IO2 LED just flickers along with display traffic
// (cosmetic only, per CLAUDE.md).

static SPIClass tftSPI(FSPI);
static Adafruit_ILI9341 tft(&tftSPI, TFT_DC_PIN, TFT_CS_PIN, TFT_RST_PIN);

static bool lastButtonState = false;
static bool cameraReady = false;

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
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_QVGA; // 320x240 — small/fast for bring-up
  config.jpeg_quality = 12;
  config.fb_count = psramFound() ? 2 : 1;
  config.fb_location = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

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

static void captureAndDump() {
  if (!cameraReady) {
    Serial.println("CAM: not initialized, skipping capture");
    return;
  }

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("CAM: capture failed");
    return;
  }

  Serial.printf("CAM:BEGIN len=%u\n", fb->len);
  Serial.println(base64::encode(fb->buf, fb->len));
  Serial.println("CAM:END");

  esp_camera_fb_return(fb);
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
}

void loop() {
  static uint32_t lastBeat = 0;

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
      captureAndDump();
    }
  }
}

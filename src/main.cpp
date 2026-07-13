#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

#include "pins.h"

// Step 1: blink/serial-print bring-up (heartbeat + shutter button) — done.
// Step 2 of the build order in CLAUDE.md: bring up the TFT alone (SPI init,
// draw test pattern). No touch controller assumed.
//
// TFT_DC_PIN == STATUS_LED_PIN (GPIO2): the ILI9341 library now owns that
// pin and toggles it on every SPI transfer, so the old heartbeat LED blink
// is gone — the onboard IO2 LED will just flicker along with display
// traffic (cosmetic only, per CLAUDE.md).

static SPIClass tftSPI(FSPI);
static Adafruit_ILI9341 tft(&tftSPI, TFT_DC_PIN, TFT_CS_PIN, TFT_RST_PIN);

static bool lastButtonState = false;

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
  }
}

#include <Arduino.h>

#include "pins.h"

// Step 1 of the build order in CLAUDE.md: basic blink/serial-print sketch
// to confirm the board flashes and runs before bringing up TFT, camera,
// or printer. Also exercises the shutter button since it's already wired.

static bool lastButtonState = false;

void setup() {
  Serial.begin(115200);
  delay(300); // let the CH343 USB-UART bridge settle

  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(BUTTON1_PIN, INPUT); // module has its own pull resistor

  Serial.println("Thermal Printer Camera — booting");
  Serial.printf("PSRAM size: %u bytes\n", ESP.getPsramSize());
}

void loop() {
  static uint32_t lastBlink = 0;
  static bool ledState = false;

  uint32_t now = millis();
  if (now - lastBlink >= 500) {
    lastBlink = now;
    ledState = !ledState;
    digitalWrite(STATUS_LED_PIN, ledState);
    Serial.printf("heartbeat t=%lu ms\n", now);
  }

  bool buttonState = digitalRead(BUTTON1_PIN) == HIGH;
  if (buttonState != lastButtonState) {
    lastButtonState = buttonState;
    Serial.printf("button1 %s\n", buttonState ? "PRESSED" : "released");
  }
}

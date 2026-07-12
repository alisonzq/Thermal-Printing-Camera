#pragma once

// Pin assignments for this project, kept in sync with CLAUDE.md.
// "wired" pins are physically connected; others are the planned
// assignment but not yet wired — see CLAUDE.md before changing any of
// these, and check the reserved-GPIO table there before adding new ones.

// --- Onboard ---
#define STATUS_LED_PIN 2   // "IO2" onboard LED; also planned as TFT DC (shared, cosmetic flicker only)
#define WS2812_PIN     48  // onboard addressable status LED

// --- Button 1 (shutter) — wired ---
#define BUTTON1_PIN 38     // active-high: idle LOW, pressed HIGH. Plain INPUT, no INPUT_PULLUP (onboard resistor on the module).

// --- DP-628 thermal printer (TTL/ESC-POS) — wired ---
#define PRINTER_TX_PIN  19 // ESP32 TX -> printer RX
#define PRINTER_RX_PIN  20 // ESP32 RX <- printer TX (through external voltage divider, printer TX is 5V)
#define PRINTER_DTR_PIN 21 // busy/flow-control, active meaning TBD

// --- TFT (ILI9341, SPI) — not yet wired ---
#define TFT_MOSI_PIN 41
#define TFT_SCK_PIN  14
#define TFT_CS_PIN   1
#define TFT_DC_PIN   2   // shares STATUS_LED_PIN
#define TFT_RST_PIN  42
// Backlight: tied straight to 3.3V (no PWM dimming pin free yet).

// --- Free pins available for more buttons / backlight PWM later ---
// GPIO39, GPIO40, GPIO47

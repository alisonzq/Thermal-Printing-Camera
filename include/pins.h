#pragma once

// Pin assignments for this project, kept in sync with CLAUDE.md.
// "wired" pins are physically connected; others are the planned
// assignment but not yet wired — see CLAUDE.md before changing any of
// these, and check the reserved-GPIO table there before adding new ones.

// --- Onboard ---
#define STATUS_LED_PIN 2   // "IO2" onboard LED; also planned as TFT DC (shared, cosmetic flicker only)
#define WS2812_PIN     48  // onboard addressable status LED

// --- Camera (OV2640, soldered on-board — Freenove FNK0085) ---
#define CAM_SIOD_PIN  4
#define CAM_SIOC_PIN  5
#define CAM_VSYNC_PIN 6
#define CAM_HREF_PIN  7
#define CAM_Y4_PIN    8  // D2
#define CAM_Y3_PIN    9  // D1
#define CAM_Y5_PIN    10 // D3
#define CAM_Y2_PIN    11 // D0
#define CAM_Y6_PIN    12 // D4
#define CAM_PCLK_PIN  13
#define CAM_XCLK_PIN  15
#define CAM_Y9_PIN    16 // D7
#define CAM_Y8_PIN    17 // D6
#define CAM_Y7_PIN    18 // D5
#define CAM_PWDN_PIN  -1 // not broken out on this board
#define CAM_RESET_PIN -1 // not broken out on this board

// --- Button 1 (shutter) — wired ---
// Bare push button, GPIO -> GND only, ESP32 internal pull-up (INPUT_PULLUP).
// Active-low: idle HIGH, pressed LOW. No external resistor, no VCC leg.
#define BUTTON1_PIN 38

// --- UI buttons (menu/gallery navigation) — wired ---
// Same bare-button / INPUT_PULLUP / active-low wiring as BUTTON1.
#define BUTTON_MENU_PIN       39 // toggles settings menu open/closed; also backs out of menu/gallery to live camera view
#define BUTTON_UP_PIN         40 // menu: previous item / increase value while editing. gallery: older photo
#define BUTTON_DOWNSELECT_PIN 47 // short press: menu next item / decrease value, gallery newer photo. Long press (>=500ms): select/confirm, or print in gallery

// --- DP-628 thermal printer (TTL/ESC-POS) — wired ---
#define PRINTER_TX_PIN  19 // ESP32 TX -> printer RX
#define PRINTER_RX_PIN  20 // ESP32 RX <- printer TX (through external voltage divider, printer TX is 5V)
#define PRINTER_DTR_PIN 21 // busy/flow-control, active meaning TBD

// --- TFT (ILI9341, SPI) — wired ---
#define TFT_MOSI_PIN 41
#define TFT_SCK_PIN  14
#define TFT_CS_PIN   1
#define TFT_DC_PIN   2   // shares STATUS_LED_PIN — library drives this pin during SPI transfers
#define TFT_RST_PIN  42
// Backlight: tied straight to 3.3V (no PWM dimming pin free yet).

// --- Free pins remaining ---
// none currently free without reassigning something above

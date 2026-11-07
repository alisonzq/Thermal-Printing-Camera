# Thermal Printer Camera — Project Brief

A camera that captures a photo, previews it on a TFT, and prints it (or a
receipt-style layout) on a thermal printer via button press.

## Hardware

- **MCU**: Freenove ESP32-S3-WROOM CAM board (FNK0085) — OV2640 camera
  on-board, 8MB PSRAM (OPI mode), 8/16MB flash, microSD slot, USB-C
  (via CH343 USB-UART bridge) for programming.
- **Display**: 2.8" SPI TFT, ILI9341 driver, with an XPT2046 resistive
  touch controller on the same module (T_IRQ not wired — touch is polled).
- **Buttons**: bare 2-terminal push buttons (4-leg tactile switches,
  internally paired). GPIO -> GND only, no VCC leg, no external
  resistor — ESP32 internal pull-up (`INPUT_PULLUP`). Active-low:
  idle = HIGH, pressed = LOW.
- **Printer**: REMOVED from the build for now. Was a DAPPER DP-628 58mm
  thermal printer (Xiamen Dapu Electronic Technology), TTL/ESC-POS. Its
  pins (19/20/21) now belong to the touch controller. Printer code is
  kept in main.cpp but compiled out with `ENABLE_PRINTER 0` — a future
  printer needs new pins before re-enabling.

## Reserved GPIOs — do not reassign

Confirmed against the official Freenove ESP32-S3 WROOM pinout diagram.

| Function | GPIOs | Notes |
|---|---|---|
| Camera (soldered on-board, already wired) | 4 (SIOD), 5 (SIOC), 6 (VSYNC), 7 (HREF), 8 (Y4), 9 (Y3), 10 (Y5), 11 (Y2), 12 (Y6), 13 (PCLK), 15 (XCLK), 16 (Y9), 17 (Y8), 18 (Y7) | GPIO5 was mistakenly wired to a button early on — moved off. |
| PSRAM (OPI, used for camera framebuffer) | 35, 36, 37 | Marked with `*` on the official diagram. |
| USB-UART bridge / programming serial | 43 (U0TXD), 44 (U0RXD) | Has onboard LED_TX/LED_RX indicators. |
| Strapping pins (boot mode / flash voltage / JTAG select) | 0 (Boot button), 3 (JTAG_EN), 45 (VSPI), 46 (LOG) | Avoid for general I/O. |
| Onboard WS2812 status LED | 48 | |
| Onboard "IO2" status LED | 2 | Still usable as a normal GPIO (e.g. TFT DC) — LED will just flicker along with pin activity, cosmetic only. |
| JTAG debug pins | 41 (MTDI), 42 (MTMS) | Free to use for anything as long as you're not JTAG-debugging. Used here for TFT RST/backlight. |
| microSD slot | 38 (SD_CMD), 39 (SD_CLK), 40 (SD_DATA) | Only reserved if the SD slot is actually used. |
| Native USB-OTG | 19 (USB_D+), 20 (USB_D-) | Physically wired to the board's second USB-C port. Currently repurposed for the touch controller (T_CS / T_DO) — do not plug a cable into that port or init the USB-OTG stack while doing so. |
| PWM-capable pins | marked with `~` on the diagram | Most GPIO4-18 plus a few others; not a constraint, just informational. |

## Pin assignments (this project) — CURRENT / as actually wired

| Component | Signal | ESP32-S3 GPIO | Status |
|---|---|---|---|
| Touch (XPT2046) | T_CS | GPIO19 | wired |
| Touch | T_DO (MISO) | GPIO20 — no voltage divider (the old printer divider must be removed; XPT2046 is 3.3V) | wired |
| Touch | T_DIN | GPIO21 — its own wire, NOT shared with TFT MOSI; firmware mirrors the SPI MOSI signal onto it via the GPIO matrix | wired |
| Touch | T_CLK | GPIO14 (shared with TFT SCK) | wired |
| Touch | T_IRQ | not wired — touch is polled | — |
| Button 1 (shutter) | Signal | GPIO38 | wired |
| Button 1 | GND | GND | wired |
| Button 2 (menu) | Signal | GPIO39 | wired |
| Button 3 (up) | Signal | GPIO40 | wired |
| Button 4 (down/select) | Signal | GPIO47 | wired |
| Buttons 2–4 | GND | GND, same as Button 1 | wired |
| TFT | MOSI | GPIO41 | wired |
| TFT | SCK | GPIO14 | wired |
| TFT | CS | GPIO1 | wired |
| TFT | DC/RS | GPIO2 — note: shares the pin with the onboard "IO2" LED, which will flicker with SPI traffic (cosmetic only) | wired |
| TFT | RST | GPIO42 | wired |
| TFT | LED (backlight) | 3.3V direct (GPIO41 now taken by MOSI, so no PWM dimming option unless moved to another free pin) | wired |

No free pins remaining without reassigning something above — GPIO39,
40, 47 are the menu/up/down-select UI buttons, 19/20/21 are touch.

## Known gotchas (verified in prior discussion, don't relitigate)

- Touch T_DIN is on GPIO21, separate from the TFT's MOSI (GPIO41). setup()
  routes the FSPI MOSI signal to both pins with
  `pinMatrixOutAttach(21, FSPID_OUT_IDX, ...)` — keep that if the SPI
  init changes, or touch reads will return garbage.
- Touch calibration constants (raw min/max, swap/invert flags) are at the
  top of main.cpp and are defaults, not measured. Every tap logs
  `TOUCH raw=(..) screen=(..)` to Serial for tuning.
- Touch and the physical buttons both work: tap live view = shutter, tap a
  menu row to select / again to open (while editing: right half +, left
  half -, elsewhere = done), tap left/right half in the gallery to browse.

Printer notes below only apply if a printer comes back:

- DP-628's exact physical pin order (GND/RX/TX/DTR/VH) is **not
  confirmed from an official datasheet** — match against the labels
  silkscreened on the actual board before wiring. The manufacturer
  (Xiamen Dapu Electronic Technology) can be reached at
  lxk@dpfastprint.com for the datasheet if needed.
- Printer's TTL TX output is 5V logic; ESP32-S3 GPIOs are 3.3V-only —
  needs a divider or level shifter on that line.
- Printer's VH pin needs its own external 5–9V/2A-capable supply,
  grounds tied together with the ESP32.
- Printer's USB-C port is a separate, independent data interface
  (same ESC/POS commands) meant for direct PC/phone use — not needed
  and not used in this build.
- Buttons are bare tactile switches wired GPIO->GND only — code uses
  `INPUT_PULLUP` and reads pressed as `LOW` (inverted from the earlier
  active-high button-module wiring; switched over deliberately to cut
  the wiring down to 2 wires per button).
- Confirm whether the Freenove board is actually the camera variant
  (FNK0085) — this brief assumes it is, since no separate camera
  module was in the parts list.

## Toolchain

- Arduino framework via PlatformIO (preferred) or Arduino IDE.
- Board: ESP32-S3 Dev Module / Freenove ESP32-S3 WROOM CAM target.
- Libraries: `esp32-camera` (OV2640), `Adafruit_ILI9341` or `TFT_eSPI`
  (display), `HardwareSerial` for the printer's ESC/POS TTL link.
- Arduino IDE build settings that matter: USB CDC on Boot disabled,
  PSRAM = OPI PSRAM, Flash Size matches board (8 or 16MB), Partition
  Scheme with enough app space for the camera + graphics libraries.

## Suggested build order

1. Scaffold a PlatformIO project targeting the ESP32-S3, get a basic
   blink/serial-print sketch flashing and running.
2. Bring up the TFT alone (SPI init, draw test pattern).
3. Bring up the camera alone (capture + dump over serial or to SD).
4. Get camera frame rendering to the TFT.
5. Bring up printer UART independently (send a raw ESC/POS test line)
   before wiring it into the camera flow.
6. Wire buttons, add capture → preview → print state machine.
7. Wire the 3 extra UI buttons, add a settings menu and an in-memory
   (PSRAM, session-only) gallery of recent captures with reprint.
8. Remove the printer, wire the TFT's XPT2046 touch controller on its old
   pins, add touch controls alongside the buttons.

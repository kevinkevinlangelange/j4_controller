# j4_controller_v0_6_4

## What is Johnny 4?

Johnny 4 is a prop robot controlled wirelessly via ESP-NOW. This repository contains the firmware for the **controller/transmitter** — the handheld unit that reads potentiometers and a keypad, then sends control data to the robot's receiver board over ESP-NOW.

## What this controller does

- Reads 7 analog potentiometers via two ADS1115 ADC modules (volume, eyes, spot, left arm, right arm, neck, jaw)
- Reads a 4×4 keypad matrix via PCF8574 I2C expander to select audio phrases (jukebox-style: letter + digit, e.g. `A4`)
- Transmits all control data to the receiver continuously via ESP-NOW
- Displays live pot values, keypress info, and battery voltage on the built-in TFT display
- Monitors controller battery voltage from GPIO 34 and displays it in real time

## Hardware

| Component | Details |
|-----------|---------|
| Microcontroller | LILYGO TTGO T-Display v1.1 (ESP32 with built-in 1.14" TFT) |
| ADC modules | Two ADS1115 (I2C addresses 0x48 and 0x49) |
| Keypad expander | PCF8574 I2C I/O expander (address 0x20) |
| Keypad | 4×4 matrix keypad |
| Display | Built-in ST7789 TFT (135×240), driven by TFT_eSPI with sprite rendering |

**Pin assignments:**
- I2C bus: SDA = GPIO 21, SCL = GPIO 22
- Battery voltage sense: GPIO 34 (ADC input, read-only)
- ADC inputs all routed through the ADS1115 modules (on ADC1-capable pins: 32, 33, 36, 37, 38)

> Note: GPIO 12 causes boot/flash issues when connected to a potentiometer. GPIO 17 does not work reliably as an input. Pins 36, 37, and 38 are input-only.

## Dependencies

Managed via PlatformIO (`platformio.ini`):

| Library | Version |
|---------|---------|
| RobTillaart/PCF8574 | 0.3.9 |
| RobTillaart/I2CKeyPad | 0.3.3 |
| RobTillaart/ADS1X15 | 0.3.13 |
| TFT_eSPI | bundled in `lib/` (pre-configured for TTGO T-Display v1.1) |

Built-in ESP-IDF components used: `esp_now`, `WiFi`

## Building and uploading

This project uses PlatformIO. From the project root:

```bash
# Build
pio run

# Upload
pio run --target upload

# Monitor serial output
pio device monitor
```

Upload speed is 921600 baud; serial monitor is 115200 baud (both set in `platformio.ini`).

The TFT_eSPI library requires a `User_Setup.h` — the local copy in `lib/TFT_eSPI/` is already configured for the TTGO T-Display v1.1. Do not replace it with a registry version without reconfiguring.

## Receiver

The receiver's MAC address is set in `broadcastAddress[]` near the top of `main.cpp`. Update this if the receiver board is swapped.

# j4_controller

Firmware for the **Johnny 4 robot controller/transmitter**. Runs on a LILYGO TTGO T-Display v1.1 (ESP32). Reads potentiometers and a 4x4 keypad, ingests four more pots from the j4_display_right board, transmits control data to the robot receiver over ESP-NOW, and mirrors live data to the two XIAO display boards over UART.

## What is Johnny 4?

Johnny 4 is a prop robot controlled wirelessly. This board is the handheld controller that the operator uses to drive the robot's servos, select audio phrases, and monitor battery status.

## What this firmware does

- Reads analog potentiometers via two local ADS1115 ADC modules: eye-pop, an eyes joystick (eyes_x / eyes_y), and the neck joystick (neck X / jaw Y)
- Ingests four more pots (iris, color, brightness, volume) from [j4_display_right](https://github.com/kevinkevinlangelange/j4_display_right)'s dedicated ADS1115 over Serial2 as 25 Hz `P:` lines
- Reads a 4x4 matrix keypad via PCF8574 I2C expander using a custom scan routine
- Supports jukebox-style audio phrase selection: press a letter (A-D) then a digit (0-9) to queue a phrase, or press a digit alone to queue a 0x phrase (e.g. pressing 8 queues "08.wav")
- Press `*` to send a STOP command, press `#` to clear the buffer
- Mixes the joystick into differential neck-left / neck-right values over a 0-3200 range
- Transmits all control data to the robot receiver via ESP-NOW (fixed-size packed structs, no `String` members so they survive the wireless `memcpy`)
- Receives the jukebox file list from the robot receiver in chunks and forwards it to the display board; carries a `need_filelist` flag so the receiver re-sends the list if this board boots late
- Answers `LIST?` requests from the display board out of its cached copy, so a display reboot recovers the list without a full round trip
- Displays live pot values, keypress state, and battery voltage on the built-in TFT
- The TTGO's built-in button (GPIO 35) cycles the TFT through three screens: live data, WiFi MAC address, and a connection-status screen showing ESP-NOW LINK, j4_stepper_neck, j4_stepper_eyes, j4_talk, j4_display_left, and j4_display_right as CONNECTED (green) / DISCONNECTED (red)
- Sends a 49-byte binary status packet to the j4_display_left board at 25 fps over UART
- Shows a STATUS line on the built-in TFT: "ONLINE" when the ESP-NOW link is up and the steppers are healthy, "OFFLINE" if no status packet arrives, or the reported stepper fault (e.g. "NL OT", "EYES OFFLINE"); green when healthy, red on any fault

## Hardware

| Component | Details |
|-----------|---------|
| Microcontroller | LILYGO TTGO T-Display v1.1 (ESP32 with built-in 1.14" ST7789 TFT) |
| ADC modules | Two ADS1115 (I2C addresses 0x48 and 0x49); a third lives on j4_display_right |
| Keypad expander | PCF8574 I2C I/O expander (address 0x20) |
| Keypad | 4x4 matrix keypad, headers soldered directly into P0-P7 |
| Displays | j4_display_left (jukebox, Serial1) + j4_display_right (pot labels, Serial2) |

## Pin assignments

| GPIO | Function |
|------|----------|
| 21 | I2C SDA (ADS1115 x2, PCF8574 keypad) |
| 22 | I2C SCL |
| 17 | Serial1 TX to j4_display_left (XIAO D7) |
| 27 | Serial1 RX from j4_display_left (XIAO D6) |
| 25 | Serial2 TX to j4_display_right (XIAO D7, reserved) |
| 26 | Serial2 RX from j4_display_right (XIAO D6, pot feed) |
| 34 | Battery voltage sense (ADC input, read-only) |
| 35 | Screen-cycle button (TTGO built-in; data / MAC / connection status) |

GPIO 35 is the TTGO's on-board button, so it needs no wiring. Potentiometer inputs are routed through the ADS1115 modules. GPIO 12 causes boot/flash issues when connected to a pot and is avoided. GPIO 17 is unreliable as an input and is used only as TX. Pins 36, 37, and 38 are input-only.

## Pin diagram

Physical layout, display facing AWAY from you (rails visible), USB-C at the
TOP (matching how the board is mounted). The two header rails read
top-to-bottom as shown.

```
            LEFT rail                            RIGHT rail
                                +--[ USB-C ]--+
I2C SDA (ADS x2 + keypad) -- 21 |             |  5V
                  I2C SCL -- 22 |             |  GND  ground
     DISP-L TX -> XIAO D7 -- 17 |     TTGO    |  27   DISP-L RX <- XIAO D6
                   (free) -- 2  |  T-Display  |  26   DISP-R RX <- XIAO D6
                   (free) -- 15 |     v1.1    |  25   DISP-R TX (reserved)
                   (free) -- 13 |             |  33   (free)
      (avoid: boot/flash) -- 12 |   [ ST7789  |  32   (free)
                   ground -- GND|    TFT on   |  39   (input only)
                     3.3V -- 3V3|    back ]   |  38   (input only)
                       5V -- 5V |             |  37   (input only)
                   ground -- GND|             |  36   (input only)
                                +-------------+

   on-board (no header pin): GPIO34 = battery sense, BOOT = GPIO0,
       GPIO35 = screen-cycle button, TFT on 4/5/16/18/19/23
     I2C bus: ADS_01 @ 0x48, ADS_02 @ 0x49, PCF8574 keypad @ 0x20
```

ESP-NOW (wireless) links this board to j4_receiver.

## Analog inputs

Local, on the two ADS1115 modules over I2C:

| Module | Channel | Control |
|--------|---------|---------|
| ADS_01 (0x48) | A0 | (free -- was volume, moved to j4_display_right) |
| ADS_01 (0x48) | A1 | (free -- was iris, moved to j4_display_right) |
| ADS_01 (0x48) | A2 | Eye-pop (0-3200, was spot) |
| ADS_02 (0x49) | A0 | Eyes joystick X / eyes_x (was left arm) |
| ADS_02 (0x49) | A1 | Eyes joystick Y / eyes_y (was right arm) |
| ADS_02 (0x49) | A2 | Neck joystick X |
| ADS_02 (0x49) | A3 | Neck joystick Y (jaw, feeds neck mixer) |

Remote, streamed from j4_display_right's dedicated ADS1115 (raw counts over Serial2, scaled here with the same `processPot()`):

| Channel | Control | Destination |
|---------|---------|-------------|
| A0 | Iris | 270-deg iris servo (PCA9685 on j4_receiver) |
| A1 | Color | WS2812B strip color (j4_receiver) |
| A2 | Brightness | WS2812B strip brightness (j4_receiver) |
| A3 | Volume | j4_talk audio volume |

eyes_x, eyes_y, and iris drive servos on the receiver's PCA9685. eye-pop is forwarded down the stepper chain (j4_stepper_neck to j4_stepper_eyes), where it drives the eye-pop steppers with the neck's motion settings. The neck joystick mixes into neck-L / neck-R as before. color and brightness drive the receiver's WS2812B strip.

## Keypad wiring

The keypad is plugged straight into PCF8574 P0-P7 (keypad pin 1 into P0, in order). The I2CKeyPad library's fixed scan pattern does not match this wiring, so the firmware uses a custom scan via the PCF8574 library directly.

Actual pin-to-wire mapping:

| PCF pin | Keypad wire |
|---------|-------------|
| P0 | Column 3 (3, 6, 9, #) |
| P1 | Column 2 (2, 5, 8, 0) |
| P2 | Column 1 (1, 4, 7, *) |
| P3 | Row 4 (*, 0, #, D) |
| P4 | Row 3 (7, 8, 9, C) |
| P5 | Row 2 (4, 5, 6, B) |
| P6 | Row 1 (1, 2, 3, A) |
| P7 | Column 4 (A, B, C, D) |

The scanner drives P0, P1, P2, P7 one at a time LOW and reads P3, P4, P5, P6.

## Dependencies

Managed via PlatformIO (`platformio.ini`):

| Library | Version |
|---------|---------|
| RobTillaart/PCF8574 | 0.3.9 |
| RobTillaart/ADS1X15 | 0.3.13 |
| TFT_eSPI | bundled in `lib/` (pre-configured for TTGO T-Display v1.1) |

Built-in ESP-IDF components used: `esp_now`, `WiFi`.

## Building and uploading

```bash
# Build
pio run

# Upload
pio run --target upload

# Monitor serial output
pio device monitor
```

Upload speed is 921600 baud. Serial monitor is 115200 baud. Both are set in `platformio.ini`.

The TFT_eSPI copy in `lib/TFT_eSPI/` is pre-configured for the TTGO T-Display v1.1. Do not replace it with the registry version without reconfiguring.

## Receiver

The robot receiver's MAC address is set in `broadcastAddress[]` near the top of `main.cpp`. Update this if the receiver board is swapped.

## Related projects

- **[j4_display_left](https://github.com/kevinkevinlangelange/j4_display_left)** -- the XIAO ESP32S3 jukebox display (portrait, left of the panel)
- **[j4_display_right](https://github.com/kevinkevinlangelange/j4_display_right)** -- the XIAO ESP32S3 pot-label display (landscape, right of the panel) that streams the iris/color/brightness/volume pots to this board
- **[j4_receiver](https://github.com/kevinkevinlangelange/j4_receiver)** -- the robot-side TTGO that receives this controller's ESP-NOW packets and returns status

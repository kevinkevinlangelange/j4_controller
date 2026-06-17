# j4_controller

Firmware for the **Johnny 4 robot controller/transmitter**. Runs on a LILYGO TTGO T-Display v1.1 (ESP32). Reads potentiometers and a 4x4 keypad, transmits control data to the robot receiver over ESP-NOW, and mirrors live data to a secondary display board over UART.

## What is Johnny 4?

Johnny 4 is a prop robot controlled wirelessly. This board is the handheld controller that the operator uses to drive the robot's servos, select audio phrases, and monitor battery status.

## What this firmware does

- Reads analog potentiometers via two ADS1115 ADC modules: volume, iris (100K linear pot), eye-pop, an eyes joystick (eyes_x / eyes_y), and the neck joystick (neck X / jaw Y)
- Reads a 4x4 matrix keypad via PCF8574 I2C expander using a custom scan routine
- Supports jukebox-style audio phrase selection: press a letter (A-D) then a digit (0-9) to queue a phrase, or press a digit alone to queue a 0x phrase (e.g. pressing 8 queues "08.wav")
- Press `*` to send a STOP command, press `#` to clear the buffer
- Mixes the joystick into differential neck-left / neck-right values over a 0-3200 range
- Transmits all control data to the robot receiver via ESP-NOW (fixed-size packed structs, no `String` members so they survive the wireless `memcpy`)
- Receives the jukebox file list from the robot receiver in chunks and forwards it to the display board; carries a `need_filelist` flag so the receiver re-sends the list if this board boots late
- Answers `LIST?` requests from the display board out of its cached copy, so a display reboot recovers the list without a full round trip
- Displays live pot values, keypress state, and battery voltage on the built-in TFT
- The TTGO's built-in button (GPIO 35) cycles the TFT through three screens: live data, WiFi MAC address, and a connection-status screen showing ESP-NOW LINK, j4_stepper_neck, j4_stepper_eyes, j4_talk, and j4_display as CONNECTED (green) / DISCONNECTED (red)
- Sends a 49-byte binary status packet to the XIAO ESP32S3 display board at 25 fps over UART
- Shows a STATUS line on the built-in TFT: "ONLINE" when the ESP-NOW link is up and the steppers are healthy, "OFFLINE" if no status packet arrives, or the reported stepper fault (e.g. "NL OT", "EYES OFFLINE"); green when healthy, red on any fault

## Hardware

| Component | Details |
|-----------|---------|
| Microcontroller | LILYGO TTGO T-Display v1.1 (ESP32 with built-in 1.14" ST7789 TFT) |
| ADC modules | Two ADS1115 (I2C addresses 0x48 and 0x49) |
| Keypad expander | PCF8574 I2C I/O expander (address 0x20) |
| Keypad | 4x4 matrix keypad, headers soldered directly into P0-P7 |

## Pin assignments

| GPIO | Function |
|------|----------|
| 21 | I2C SDA (ADS1115 x2, PCF8574 keypad) |
| 22 | I2C SCL |
| 17 | UART TX to XIAO display board |
| 27 | UART RX from XIAO display board |
| 34 | Battery voltage sense (ADC input, read-only) |

Potentiometer inputs are routed through the ADS1115 modules. GPIO 12 causes boot/flash issues when connected to a pot and is avoided. GPIO 17 is unreliable as an input and is used only as TX. Pins 36, 37, and 38 are input-only.

## Pin diagram

```
                  TTGO T-Display v1.1 (ESP32)  -  j4_controller
                  +--------------------------------------+
   I2C SDA <----->| 21    ADS1115 x2 (0x48/0x49)         |
   I2C SCL <----->| 22    PCF8574 keypad (0x20)          |
                  |                            [ ST7789 1.14" TFT ]
   XIAO D7 RX <---| 17 (UART TX)                         |
   XIAO D6 TX --->| 27 (UART RX)                  34 |<-- battery sense (ADC in)
                  |              3V3   GND   USB         |
                  +--------------------------------------+
                          |
                          +-- ESP-NOW (wireless) <--> j4_receiver
```

## Analog inputs (ADS1115)

The pots feed two ADS1115 modules over I2C. The eye controls reuse the channels that previously read the eyes/spot/arm pots:

| Module | Channel | Control |
|--------|---------|---------|
| ADS_01 (0x48) | A0 | Volume |
| ADS_01 (0x48) | A1 | Iris (100K linear pot, was eyes) |
| ADS_01 (0x48) | A2 | Eye-pop (0-3200, was spot) |
| ADS_02 (0x49) | A0 | Eyes joystick X / eyes_x (was left arm) |
| ADS_02 (0x49) | A1 | Eyes joystick Y / eyes_y (was right arm) |
| ADS_02 (0x49) | A2 | Neck joystick X |
| ADS_02 (0x49) | A3 | Neck joystick Y (jaw, feeds neck mixer) |

eyes_x, eyes_y, and iris are sent to j4_receiver over ESP-NOW and drive servos on its PCA9685. eye-pop is sent on the same packet and is forwarded down the stepper daisy chain (j4_stepper_neck to j4_stepper_eyes), where it drives the eye-pop steppers with the neck's motion settings. The neck joystick still mixes into neck-L / neck-R as before.

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

- **[j4_display](https://github.com/kevinkevinlangelange/j4_display)** -- the XIAO ESP32S3 secondary display board that receives data from this controller
- **[j4_receiver](https://github.com/kevinkevinlangelange/j4_receiver)** -- the robot-side TTGO that receives this controller's ESP-NOW packets and returns status

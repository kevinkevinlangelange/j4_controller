//******************************************************************************
//       file name:  j4_controller.ino
//     v0_1 created:  2023-11-08 -- 1209 CST
//     v0_6 created:  2023-11-16 -- 2226 CST
//   v0_6_4 created:  2026-05-20 -- 0700 CDT
//     last updated:  2026-05-31 -- 1752 CDT
//     last updated:  2026-06-02 -- 1045 CDT
//     last updated:  2026-06-07 -- CDT
//     last updated:  2026-06-10 -- CDT
//     last updated:  2026-06-15 -- CDT
//     last updated:  2026-06-16 -- CDT
//     last updated:  2026-06-17 -- CDT
//     last updated:  2026-07-02 -- CDT
//     last updated:  2026-07-03 -- CDT
//     last updated:  2026-07-04 -- CDT
//     last updated:  2026-07-08 -- CDT
//     last updated:  2026-07-12 -- CDT
//     last updated:  2026-07-14 -- CDT
//     last updated:  2026-08-21 -- CDT
//     last updated:  2026-08-23 -- CDT
//     version increment:  20260823--007
//
//
//           author:  Kevin Lange
//      description:  Main code for Johnny 4 controller/transmitter
//                    running on a LILYGO TTGO T-Display v1.1 ESP32 board
//       update log:  v0_3 -- Changed potentiometer inputs to GPIOs on ADC1
//                    v0_4 -- Implemented ESP-NOW
//                    v0_5 -- Added ADS1115 ADC modules
//                    v0_6 -- Converted to sprite display
//                   v0_6b -- Migrated to PlatformIO; fixed keypad wiring
//                  v0_6_3 -- Comment cleanup and style normalization
//                  v0_6_4 -- Condensed; removed dead code; pot processing
//                            extracted into processPot() helper
//                  v0_6_5 -- Added UART link to XIAO ESP32S3 display board
//                  v0_6_6 -- Jukebox: receive file list chunks from j4_receiver,
//                            forward to j4_display via second UART packet type
//                  v0_6_7 -- Replaced String members in ESP-NOW structs with fixed
//                            char arrays (matching j4_receiver v0_7r_6)
//                         -- Control packet now carries need_filelist flag so the
//                            receiver re-sends the list if we boot late
//                         -- Answer LIST? requests from j4_display so a display
//                            reboot recovers the file list from our cached copy
//                  v0_6_8 -- Repurposed the eyes/spot/left-arm/right-arm pots into
//                            eye controls: iris (was eyes pot), eye-pop (was spot
//                            pot, 0-3200 like neck), eyes_x (was left-arm), eyes_y
//                            (was right-arm). ESP-NOW control packet now carries
//                            iris/eyes_x/eyes_y/eye_pop in place of eyes/spot/
//                            left_arm/right_arm. Neck joystick, volume, and the
//                            jukebox are unchanged. disp_pkt_t to j4_display keeps
//                            its layout (the new controls reuse the old slots).
//                  v0_6_9 -- Added a STATUS line to the TFT. The ESP-NOW status
//                            packet now carries a stepper_status field (from
//                            j4_stepper_neck via j4_receiver). Shows "ONLINE" when
//                            the link is up and all drivers are healthy, "OFFLINE"
//                            if no status packet arrives, or the reported fault
//                            (e.g. "NL OT", "EYES OFFLINE"). Green when healthy,
//                            red otherwise.
//                 v0_6_10 -- Added a screen-cycle button (TTGO built-in GPIO 35,
//                            like j4_receiver): data -> MAC address -> connection
//                            status. The connection screen shows ESP-NOW LINK,
//                            j4_stepper_neck, j4_stepper_eyes, j4_talk, and
//                            j4_display as CONNECTED/DISCONNECTED. The status packet
//                            carries neck_ok/eyes_ok/talk_ok from the receiver, the
//                            control packet carries display_ok, and j4_display +
//                            j4_talk send a "PING" heartbeat so they can be seen.
//                 v0_6_11 -- The stepper boards were consolidated: j4_stepper
//                            (renamed from j4_stepper_neck) now drives all five
//                            steppers and j4_stepper_eyes is retired. The status
//                            packet's neck_ok/eyes_ok became a single stepper_ok
//                            (matching j4_receiver v0_7r_12), and the connection
//                            screen lists j4_stepper instead of the two boards.
//                 v0_6_12 -- Second XIAO display board: j4_display_right (the old
//                            j4_display is renamed j4_display_left). It carries
//                            its own ADS1115 reading the four top-right pots and
//                            streams raw counts here over Serial2 (TX 25 / RX 26)
//                            as "P:<iris>,<color>,<brightness>,<volume>" at 25Hz.
//                            iris and volume now come from that feed (freeing
//                            ADS_01 A0/A1 for future pots), and two new controls
//                            ride the ESP-NOW control packet: color + brightness
//                            for the WS2812B strip on j4_receiver. display_ok
//                            split into display_l_ok / display_r_ok; the conn
//                            screen shows both displays.
//                 v0_6_13 -- The stepper boards split back into j4_stepper_neck +
//                            j4_stepper_eyes (per-endpoint limit switches needed
//                            the extra pins). Restored the eyes_ok field in the
//                            ESP-NOW status packet (matching j4_receiver
//                            v0_7r_14) and the j4_stepper_eyes row on the
//                            connection screen.
//                 v0_6_14 -- RF hardening for crowded venues (Open Sauce prep),
//                            matching j4_receiver v0_7r_15: ESP-NOW now runs on
//                            the ESP32 long-range (LR) PHY (~4dB more sensitive;
//                            ordinary WiFi gear cannot decode it -- both ends
//                            must be ESP32s in LR mode), TX power maxed at
//                            19.5dBm, and OnDataRecv drops any packet whose
//                            sender MAC is not our receiver. ESP-NOW channel
//                            pinned to 6 (ESPNOW_CHANNEL) to dodge other ESP-NOW
//                            projects on the default channel 1.
//                 v0_6_15 -- All control surfaces accounted for. Added ADS_03
//                            (0x4A) + ADS_04 (0x4B): the eight middle face pots
//                            (eyebrow L/R, basket eyebrow L/R, nose, nose basket,
//                            bottom eyelid L/R) now read on ADS_01 + ADS_03 and
//                            ride the ESP-NOW control packet to eight new PCA9685
//                            servo channels on j4_receiver (ch 6-13). ADS_04 is
//                            wired and initialized but its four channels are
//                            spares for the future pots (neck-pivot etc.).
//                            Added the four panel toggles on GPIO 32/33/13/15
//                            (LASER, VENT, EYE POP, AUX by the right joystick;
//                            INPUT_PULLUP, switch closes to GND). LASER + VENT
//                            drive PCA9685 ch 14/15 on j4_receiver; EYE POP
//                            replaces the old eye-pop pot (ADS_01 A2 freed) and
//                            sends 0 or 3200 through the existing stepper path;
//                            AUX is read + transmitted but unassigned. The
//                            control packet gains eight pot fields + a toggle
//                            bitmask (matching j4_receiver v0_7r_16).
//                 v0_6_16 -- Fixed a total lockup when any ADS1115 is absent:
//                            the ADS1X15 library's readADC() has no timeout,
//                            so with no chip on the bus isBusy() never clears
//                            and the first read in loop() spun forever (frozen
//                            screen, dead button, no ESP-NOW; yield() kept the
//                            watchdog fed so it never rebooted). Every module
//                            is now probed with isConnected() before its
//                            channels are read (adsReady()); absent modules
//                            read 0 (joystick axes centre at 1600) and are
//                            re-probed each pass, so the board runs fine bare
//                            on USB power and ADCs can even be hot-plugged.
//                 v0_6_17 -- Fixed the second lockup (~2s after boot, bare
//                            board): esp_now_send() ran every loop pass, only
//                            ever throttled by the old blocking ADC reads.
//                            With no ADCs the loop spun at multi-kHz, sends
//                            fired thousands of times a second, and
//                            OnDataSent (WiFi task) assigned to the
//                            connectStatus String each time -- heap ops
//                            racing loop()'s own, corrupting the heap in
//                            seconds. Control TX now runs on a 40ms timer
//                            (25 Hz, matching the receiver's 20ms gate), the
//                            callbacks only store plain flags, and the
//                            file-list forward to the display moved from
//                            OnDataRecv into loop() so Serial1 is written
//                            from one context only.
//                 v0_6_18 -- Fixed the ~2s-per-cycle stall on a bare board:
//                            with no I2C modules attached the bus has no
//                            pull-ups (they live on the breakouts), so a
//                            probe doesn't fast-NACK -- it eats the driver
//                            timeout + bus recovery, hundreds of ms each,
//                            and three probes per 40ms control cycle kept
//                            loop() almost always blocked (button worked
//                            only in the tiny gap between cycles).
//                            Wire.setTimeOut(10) caps every transaction,
//                            absent devices are re-probed only once per
//                            second (I2C_REPROBE_MS), and the keypad scan
//                            is gated on the PCF8574 ACKing (a floating bus
//                            read looks like a key held down forever).
//                 v0_6_19 -- Disconnect-audit residual: only "PING" / "LIST?"
//                            count as the j4_display_left heartbeat. Any line
//                            used to count, so garbage from the floating RX
//                            pin (display unplugged) faked CONNECTED.
//                 v0_6_20 -- Second 4x4 keypad. keypad_left is the NEW keypad
//                            (own PCF8574 backpack at 0x21 -- A0 jumper high;
//                            a PCF8574A backpack would land at 0x39 instead);
//                            keypad_right is the original at 0x20. Both drive
//                            the same phrase-select logic for now, scanned
//                            left-first, one key per pass, each with its own
//                            keymap string (keymap_left starts as a copy of
//                            keymap_right -- re-derive it if the new model's
//                            matrix comes out scrambled). Each pad has its
//                            own presence guard, so either can be absent or
//                            hot-plugged.
//                 v0_6_21 -- ADS_04 (0x4B) goes live: A0 is the neck-pivot
//                            pot (silver knob below j4_display_left, 0-3200
//                            like neck L/R, rides the control packet in the
//                            new neck_pivot field -> receiver -> nP on the
//                            stepper link), A1/A2 are two linear fader pots.
//                            fader_right doubles the IRIS pot (iris servo),
//                            fader_left doubles the Nose Basket pot (PCA9685
//                            ch 11). Each pair is arbitrated last-mover-wins:
//                            whichever control moved most recently is the
//                            active source and its value is used, so the two
//                            never fight. A source that is absent (module
//                            unplugged, display feed down) can neither claim
//                            nor hold active status.
//                 v0_6_22 -- FACE PRESETS. The right keypad stops doubling the
//                            phrase-select pad and becomes the face keypad:
//                            hold any key (except *) for 3s and j4_display_right
//                            asks "SAVE FACE ON <key>? PRESS * TO CONFIRM"
//                            (any other key cancels, 10s timeout). Confirming
//                            snapshots the current face (iris, color,
//                            brightness, the 8 face pots, and the LASER/VENT/
//                            EYE POP toggle states -- no volume, no neck, no
//                            eyes X/Y) into a 16-slot RAM table keyed by the
//                            keypad character, and pushes it through
//                            j4_receiver to j4_talk, which persists it in
//                            FACES.TXT on its microSD. Tapping a key recalls
//                            its face: a preset overlay holds every face
//                            channel at the recalled value until that
//                            channel's own physical control moves (frozen-
//                            baseline takeover, DUAL_CLAIM_COUNTS), and each
//                            toggle until its switch is flipped. On boot (or
//                            whenever the talk link appears) the controller
//                            re-requests the saved-face dump in the
//                            background until it has it; saves made while
//                            talk is offline stay in RAM flagged dirty and
//                            auto-sync when the link comes up. All of it is
//                            timer-driven ESP-NOW packet type 0x04 traffic --
//                            no board ever blocks or waits at boot for any
//                            of this. dualPick() baselines now freeze while
//                            a source is inactive so a slowly-moved control
//                            can still accumulate enough travel to claim.
//                            The controller now also talks TO j4_display_right
//                            (Serial2 TX GPIO 25, previously reserved):
//                            "M:<line1>|<line2>" shows a message, "X:" clears.
//                 v0_6_23 -- Screen-cycle button gains four more pages (still
//                            wraps: data -> MAC -> status -> ADS_01..ADS_04),
//                            showing each ADS1115 module's live raw pot
//                            counts and CONNECTED/DISCONNECTED for bench
//                            testing without a laptop on the I2C bus. Also
//                            corrected the README pin diagram: the physical
//                            header pin order was wrong on one rail (left
//                            and right rail assignment was fine, but pin
//                            order top-to-bottom on the left rail was never
//                            transformed for the 180-degree mounting
//                            rotation) and two GND pins were missing
//                            entirely (board is 12+12 pins, diagram only
//                            showed 11+11); verified against LilyGO's own
//                            pinout image, not just the schematic.
//                 v0_6_24 -- ADS_01 and ADS_02 swap payloads to match a
//                            rewire: both joysticks (eyes X/Y, neck X/Y)
//                            now land on ADS_01 (0x48) and the four
//                            left-bank face pots (Eyebrow L/R, Basket
//                            Eyebrow L/R) on ADS_02 (0x49). Addresses are
//                            unchanged -- ADS_01 is still 0x48 and ADS_02
//                            still 0x49, so no ADDR strap moves; only
//                            which pot wires land on which module's A0-A3.
//
//
//
//      J4_CONTROLLER (RC Controller / Transmitter for Johnny 4 project)
//      ------------------------------------------------------------------
//      LilyGO TTGO T-Display v1.1 (ESP32) Module's Pin Connections
//      ------------------------------------------------------------------
//      VIN:
//      GND:  Make sure all grounds are connected together
//     3.3V:
//        0:  button 1 / BOOT      [USED BY TTGO]
//        4:  TFT backlight        [USED BY TTGO]
//        5:  TFT CS               [USED BY TTGO]
//       16:  TFT DC               [USED BY TTGO]
//       17:  DISPLAY-L TX  →  left XIAO D7 / GPIO44   (Serial1)
//       18:  TFT SCLK             [USED BY TTGO]
//       19:  TFT MOSI             [USED BY TTGO]
//
//       13:  EYE POP toggle (INPUT_PULLUP, switch closes to GND)
//       15:  AUX toggle, next to the right joystick (INPUT_PULLUP, unassigned)
//
//       21:  SDA  [I2C BUS] (ADS_01 0x48, ADS_02 0x49, ADS_03 0x4A,
//                            ADS_04 0x4B, keypad 0x20)
//       22:  SCL  [I2C BUS]
//
//       23:  TFT RST              [USED BY TTGO]
//
//       25:  DISPLAY-R TX  →  right XIAO D7 / GPIO44  (Serial2, face messages)
//       26:  DISPLAY-R RX  ←  right XIAO D6 / GPIO43  (Serial2, pot feed)
//       27:  DISPLAY-L RX  ←  left XIAO D6 / GPIO43   (Serial1)
//
//       32:  LASER toggle (INPUT_PULLUP, switch closes to GND)
//       33:  VENT  toggle (INPUT_PULLUP, switch closes to GND)
//
//       34:  battery voltage sense
//
//       35:  button 2             [USED BY TTGO]
//      ------------------------------------------------------------------
//      ------------------------------------------------------------------
//
//      XIAO LINK WIRING  (3.3V logic on both sides - no level shifter needed)
//      ------------------------------------------------------------------
//      j4_display_left  (Serial1):
//        TTGO GPIO17  →  XIAO D7 (GPIO44)    TTGO TX → XIAO RX
//        TTGO GPIO27  ←  XIAO D6 (GPIO43)    TTGO RX ← XIAO TX
//      j4_display_right (Serial2):
//        TTGO GPIO25  →  XIAO D7 (GPIO44)    TTGO TX → XIAO RX (face messages)
//        TTGO GPIO26  ←  XIAO D6 (GPIO43)    TTGO RX ← XIAO TX (pot feed)
//      TTGO GND  -  both XIAO GNDs
//      ------------------------------------------------------------------
//
//      ANALOG INPUTS
//      ------------------------------------------------------------------
//      Local, on the four ADS1115 ADCs (I2C):
//      ADS_01 (0x48) A0:  eyes joystick X / eyes_x        -> eyes pan servo
//      ADS_01 (0x48) A1:  eyes joystick Y / eyes_y        -> eyes tilt servo
//      ADS_01 (0x48) A2:  neck joystick X
//      ADS_01 (0x48) A3:  neck joystick Y (jaw)  -> mixed into neck-L / neck-R
//      ADS_02 (0x49) A0:  Eyebrow L pot         -> PCA9685 ch 6
//      ADS_02 (0x49) A1:  Eyebrow R pot         -> PCA9685 ch 7
//      ADS_02 (0x49) A2:  Basket Eyebrow L pot  -> PCA9685 ch 8
//      ADS_02 (0x49) A3:  Basket Eyebrow R pot  -> PCA9685 ch 9
//      ADS_03 (0x4A) A0:  Nose pot (up/down)    -> PCA9685 ch 10
//      ADS_03 (0x4A) A1:  Nose Basket pot       -> PCA9685 ch 11
//      ADS_03 (0x4A) A2:  Bottom Eyelid L pot   -> PCA9685 ch 12
//      ADS_03 (0x4A) A3:  Bottom Eyelid R pot   -> PCA9685 ch 13
//      ADS_04 (0x4B) A0:  neck-pivot pot (silver knob below j4_display_left)
//                         -> nP on the stepper link, 0-3200
//      ADS_04 (0x4B) A1:  fader_left  (linear fader) -> Nose Basket, shared
//                         with ADS_03 A1 (last-mover-wins)
//      ADS_04 (0x4B) A2:  fader_right (linear fader) -> iris, shared with
//                         j4_display_right's IRIS pot (last-mover-wins)
//      ADS_04 (0x4B) A3:  spare
//
//      TOGGLE INPUTS (INPUT_PULLUP, switch closes to GND, ON = LOW):
//      GPIO 32:  LASER   -> PCA9685 ch 14 servo on j4_receiver
//      GPIO 33:  VENT    -> PCA9685 ch 15 servo on j4_receiver
//      GPIO 13:  EYE POP -> eye-pop steppers, sends 0 (normal) or 3200 (popped)
//      GPIO 15:  AUX (next to the right joystick, unassigned; sent as a spare bit)
//
//      Remote, from j4_display_right's ADS1115 (0x48 on its own bus) over
//      Serial2 ("P:" lines):
//        iris        -> iris servo (PCA9685 on j4_receiver)
//        color       -> WS2812B strip color   (j4_receiver)
//        brightness  -> WS2812B strip brightness (j4_receiver)
//        volume      -> j4_talk audio volume
//
//      Everything is sent to j4_receiver over ESP-NOW. The receiver drives
//      the face/eye servos + LED strip, forwards neck + eye-pop to
//      j4_stepper_neck, and relays volume to j4_talk.
//      ------------------------------------------------------------------
//
//
//
//******************************************************************************


#include <Wire.h>
#include <PCF8574.h>
#include <TFT_eSPI.h>
#include <ADS1X15.h>
#include <SPI.h>
#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include "kevco_labs_logo_02.h" // 135 x 37 pixels

// ----------  FUNCTION PROTOTYPES ----------
void OnDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len);
void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status);
void labelsDisplaySprite();
void dataDisplaySprite();
void tftDisplayUpdate();
void controllerScreenModeDetect();
void macAddressDisplay();
void connectionDisplay();
void drawConnLine(const char *name, bool ok, int row);
void adsPotsDisplay(int idx);
void sendToXIAO();
void sendFileListToXIAO();
// ------------------------------------------


#define SDA 21
#define SCL 22

// --- XIAO LINKS ---
#define XIAO_TX_PIN  17    // Serial1: GPIO17 output → left XIAO D7 (GPIO44)
#define XIAO_RX_PIN  27    // Serial1: GPIO27 input  ← left XIAO D6 (GPIO43)
#define XIAOR_TX_PIN 25    // Serial2: GPIO25 output → right XIAO D7 (reserved)
#define XIAOR_RX_PIN 26    // Serial2: GPIO26 input  ← right XIAO D6 (pot feed)
#define XIAO_BAUD    115200
#define JOYSTICK_DEAD_ZONE 200  // counts either side of 1600 to snap to center (scaled for 0-3200 range)

// Panel toggle switches (INPUT_PULLUP, switch closes to GND, ON = LOW)
#define LASER_TOGGLE_PIN    32  // -> laser servo, PCA9685 ch 14 on j4_receiver
#define VENT_TOGGLE_PIN     33  // -> vent servo,  PCA9685 ch 15 on j4_receiver
#define EYE_POP_TOGGLE_PIN  13  // -> eye-pop steppers, 0 (normal) / 3200 (popped)
#define AUX_TOGGLE_PIN      15  // next to the right joystick; unassigned spare

// Toggle bit positions in xmitData.toggles_xmit (1 = switch ON)
#define TOGGLE_BIT_LASER    0
#define TOGGLE_BIT_VENT     1
#define TOGGLE_BIT_EYE_POP  2
#define TOGGLE_BIT_AUX      3

// Binary packet sent to the XIAO display board over Serial1.
// Both ends must keep this struct identical.
typedef struct __attribute__((packed)) {
  uint8_t  magic[2];       // 0xAB, 0xCD - frame sync marker
  uint8_t  volume;         // 0-100
  uint8_t  eyes;           // 0-255
  uint8_t  spot;           // 0-255
  uint8_t  left_arm;       // 0-255
  uint8_t  right_arm;      // 0-255
  uint8_t  neck;           // 0-255
  uint8_t  jaw;            // 0-255
  uint16_t bat1_mv;        // controller battery in millivolts
  int16_t  bat2_raw;       // robot battery 2 - raw value from receiver
  int16_t  bat3_raw;       // robot battery 3 - raw value from receiver
  uint8_t  connect_ok;     // 1 = ESP-NOW link healthy, 0 = error
  char     phrase[32];     // null-terminated phrase name (e.g. "A12")
  uint8_t  checksum;       // XOR of all preceding bytes in the packet
} disp_pkt_t;

// File list packet - second packet type on the same UART link.
// Magic 0xBE, 0xCD distinguishes it from the control packet (0xAB, 0xCD).
// Sent once after the file list is fully received from j4_receiver.
#define MAX_FILES      50
#define FILE_ID_LEN     2
#define FILE_NAME_MAX  22
#define FILES_PER_CHUNK 5

typedef struct __attribute__((packed)) {
  uint8_t magic[2];        // 0xBE, 0xCD
  uint8_t total_files;
  struct {
    char id[FILE_ID_LEN];
    char name[FILE_NAME_MAX];
  } files[MAX_FILES];
  uint8_t checksum;
} filelist_pkt_t;

// ESP-NOW packet type byte - first byte of every payload
#define ESPNOW_PKT_CONTROL  0x01  // controller -> receiver (sticks, pots, phrase select)
#define ESPNOW_PKT_FILELIST 0x02  // receiver -> controller (file list chunk)
#define ESPNOW_PKT_STATUS   0x03  // receiver -> controller (now playing, batteries)
#define ESPNOW_PKT_FACE     0x04  // face presets, both directions (op says which)

// Face packet ops (espnow_face_pkt_t.op). The receiver translates these
// to/from text lines on the Teensy UART; this struct MUST stay byte-identical
// with j4_receiver's copy.
#define FACE_OP_SAVE 1  // controller -> receiver: write this face to the SD
#define FACE_OP_REQ  2  // controller -> receiver: send me the saved-face dump
#define FACE_OP_DATA 3  // receiver -> controller: one saved face from the dump
#define FACE_OP_END  4  // receiver -> controller: dump complete, v[0] = count
#define FACE_OP_ACK  5  // receiver -> controller: Teensy wrote the slot to SD
#define FACE_OP_ERR  6  // receiver -> controller: Teensy SD write failed

#define FACE_VALUES 11  // iris, color, brightness, the 8 face pots (FI_* order)

typedef struct __attribute__((packed)) {
  uint8_t pkt_type;          // ESPNOW_PKT_FACE
  uint8_t op;                // FACE_OP_*
  uint8_t key;               // keypad character ('0'-'9','A'-'D','#'); 0 = unused
  uint8_t toggles;           // bit 0 LASER, 1 VENT, 2 EYE POP
  int16_t v[FACE_VALUES];
} espnow_face_pkt_t;

typedef struct __attribute__((packed)) {
  uint8_t pkt_type;
  uint8_t chunk_index;
  uint8_t total_chunks;
  uint8_t entry_count;
  struct {
    char id[FILE_ID_LEN];
    char name[FILE_NAME_MAX];
  } entries[FILES_PER_CHUNK];
} espnow_filelist_chunk_t;
// --- END XIAO LINK ---


ADS1115 ADS_01(0x48);  // ADDRESS PIN TO GND
ADS1115 ADS_02(0x49);  // ADDRESS PIN TO VDD
ADS1115 ADS_03(0x4A);  // ADDRESS PIN TO SDA
ADS1115 ADS_04(0x4B);  // ADDRESS PIN TO SCL (A0 neck-pivot, A1/A2 faders, A3 spare)


// --- ESP-NOW RELATED ---
uint8_t broadcastAddress[] = { 0xA0, 0xDD, 0x6C, 0x74, 0xDA, 0x74 };  //MAY 2026 TTGO 2026-05-01--1238-KL

// ESP-NOW WiFi channel -- MUST match j4_receiver. 6 avoids the ESP32 power-on
// default (1) that every unconfigured ESP-NOW project sits on. Use 1/6/11 only.
#define ESPNOW_CHANNEL 6

// Both ends must keep these structs identical to the ones in j4_receiver.
// Packed with fixed-size char arrays -- no String members, they don't survive
// the memcpy across ESP-NOW (the receiver would get a pointer, not the text).
typedef struct __attribute__((packed)) struct_message_rcv {
  uint8_t pkt_type;                  // ESPNOW_PKT_STATUS
  char    phrase_playing_rcv[32];
  int16_t battery_02_voltage_rcv;
  int16_t battery_03_voltage_rcv;
  char    stepper_status_rcv[24];    // from j4_stepper_neck via j4_receiver
  uint8_t stepper_ok_rcv;            // 1 = j4_stepper_neck responding
  uint8_t eyes_ok_rcv;               // 1 = j4_stepper_eyes responding (per neck's EY:)
  uint8_t talk_ok_rcv;               // 1 = j4_talk (Teensy) responding
} struct_message_rcv;

struct_message_rcv rcvData;

typedef struct __attribute__((packed)) struct_message_xmit {
  uint8_t pkt_type;                  // ESPNOW_PKT_CONTROL
  char    phrase_select_xmit[8];
  int16_t volume_xmit;
  int16_t iris_xmit;                 // 270-deg iris servo (pot on j4_display_right)
  int16_t color_xmit;                // WS2812B strip color, 0-255 (j4_display_right)
  int16_t brightness_xmit;           // WS2812B strip brightness, 0-255 (j4_display_right)
  int16_t eyes_x_xmit;               // eyes joystick X -> eyes_x servo
  int16_t eyes_y_xmit;               // eyes joystick Y -> eyes_y servo
  int16_t eye_pop_xmit;              // eye-pop steppers, 0 or 3200 (EYE POP toggle)
  int16_t neck_left_xmit;
  int16_t neck_right_xmit;
  int16_t neck_pivot_xmit;           // neck-pivot pot -> nP on the stepper link
  int16_t eyebrow_l_xmit;            // Eyebrow L pot        -> PCA9685 ch 6
  int16_t eyebrow_r_xmit;            // Eyebrow R pot        -> PCA9685 ch 7
  int16_t basket_brow_l_xmit;        // Basket Eyebrow L pot -> PCA9685 ch 8
  int16_t basket_brow_r_xmit;        // Basket Eyebrow R pot -> PCA9685 ch 9
  int16_t nose_xmit;                 // Nose pot (up/down)   -> PCA9685 ch 10
  int16_t nose_basket_xmit;          // Nose Basket pot      -> PCA9685 ch 11
  int16_t eyelid_l_xmit;             // Bottom Eyelid L pot  -> PCA9685 ch 12
  int16_t eyelid_r_xmit;             // Bottom Eyelid R pot  -> PCA9685 ch 13
  uint8_t toggles_xmit;              // bit 0 LASER, 1 VENT, 2 EYE POP, 3 AUX
  uint8_t need_filelist_xmit;        // 1 = still waiting on the file list
  uint8_t display_l_ok_xmit;         // 1 = controller sees j4_display_left heartbeat
  uint8_t display_r_ok_xmit;         // 1 = controller sees j4_display_right pot feed
} struct_message_xmit;

struct_message_xmit xmitData;
esp_now_peer_info_t peerInfo;

volatile bool connectError = LOW;
String connectStatus = "NO INFO";

// Status line: tracks the last status packet from j4_receiver. If none arrives
// within the timeout, the ESP-NOW link is treated as down ("OFFLINE").
unsigned long lastStatusRecvMs = 0;
#define STATUS_LINK_TIMEOUT_MS  1500

// j4_display_left heartbeat ("PING" or any line on the Serial1 link)
unsigned long lastDisplayMs = 0;
// j4_display_right heartbeat (its 25Hz "P:" pot lines on the Serial2 link)
unsigned long lastDisplayRMs = 0;
#define DISPLAY_TIMEOUT_MS  3000

// Screen cycling via the TTGO's built-in button on GPIO 35 (same as j4_receiver).
// 0 = data, 1 = MAC address, 2 = connection status, 3-6 = live ADS1115 pot
// counts (one screen per module, ADS_01 through ADS_04).
#define SCREEN_BUTTON  35
#define NUM_SCREENS    7
int  screen_mode = 0;
bool screen_button_prev = HIGH;
unsigned long screen_button_previousMillis = 0;
const unsigned long screen_debounce_ms = 50;
// --- END ESP-NOW RELATED ---

// --- JUKEBOX FILE LIST ---
filelist_pkt_t jukeboxPkt;
bool           jukeboxReady       = false;
uint8_t        chunksReceived     = 0;
uint8_t        chunksExpected     = 0;
String         xiaoSerialBuf      = "";
// --- END JUKEBOX FILE LIST ---


// Potentiometer values
int volume_value    = 0;   // from j4_display_right (was ADS_01 A0)
int iris_value      = 0;   // from j4_display_right (was ADS_01 A1)
int color_value     = 0;   // from j4_display_right -> WS2812B color
int brightness_value = 0;  // from j4_display_right -> WS2812B brightness
int eyes_x_value    = 0;   // eyes joystick X (was left-arm pot)
int eyes_y_value    = 0;   // eyes joystick Y (was right-arm pot)
int eye_pop_value   = 0;   // eye-pop steppers, 0 or 3200 (EYE POP toggle)
int neck_value       = 0;  // joystick X-axis raw
int jaw_value        = 0;  // joystick Y-axis raw
int neck_left_value  = 0;
int neck_right_value = 0;
int neck_pivot_value = 1600;  // neck-pivot pot (ADS_04 A0); centre if absent

// Linear fader pots (ADS_04 A1/A2). Each doubles an existing rotary pot:
// fader_left pairs with Nose Basket (ADS_03 A1), fader_right with the IRIS
// pot on j4_display_right. See dualPick() for the arbitration.
int fader_left_value  = 0;
int fader_right_value = 0;

// Middle face pots (0-255, mapped to PCA9685 servo channels on j4_receiver)
int eyebrow_l_value     = 0;  // ADS_02 A0
int eyebrow_r_value     = 0;  // ADS_02 A1
int basket_brow_l_value = 0;  // ADS_02 A2
int basket_brow_r_value = 0;  // ADS_02 A3
int nose_value          = 0;  // ADS_03 A0
int nose_basket_value   = 0;  // ADS_03 A1
int eyelid_l_value      = 0;  // ADS_03 A2
int eyelid_r_value      = 0;  // ADS_03 A3

// Panel toggles (true = switch ON, pin pulled to GND)
bool laser_toggle   = false;
bool vent_toggle    = false;
bool eye_pop_toggle = false;
bool aux_toggle     = false;

// Raw ADS1115 counts streamed from j4_display_right's "P:" lines. Held raw so
// the same processPot() scaling applies as for the local ADS channels.
int dispR_iris_raw       = 0;
int dispR_color_raw      = 0;
int dispR_brightness_raw = 0;
int dispR_volume_raw     = 0;
String xiaoRSerialBuf    = "";

// Controller battery (millivolts), updated by battery timer, read by sendToXIAO()
uint16_t bat1_mv = 0;


// --- FACE PRESETS ---
// A face is the 11 pot channels below plus the LASER/VENT/EYE POP toggle
// states. No volume, no neck, no eyes X/Y. Slots are keyed by the keypad
// CHARACTER (not the scan code) so a re-derived keymap keeps every saved
// face on the same printed key. '*' is the confirm key and cannot be a slot.
enum {
  FI_IRIS = 0, FI_COLOR, FI_BRIGHT,
  FI_BROW_L, FI_BROW_R, FI_BBROW_L, FI_BBROW_R,
  FI_NOSE, FI_NOSE_BASKET, FI_LID_L, FI_LID_R
};

#define FACE_SLOTS 16
struct FaceSlot {
  char    key;         // keypad character, 0 = slot never used
  bool    valid;       // has data (recallable)
  bool    dirty;       // not yet confirmed on the Teensy SD
  bool    sd_failed;   // Teensy reported a write error -- stop auto-retrying
  int16_t v[FACE_VALUES];
  uint8_t toggles;     // bit 0 LASER, 1 VENT, 2 EYE POP
};
FaceSlot faces[FACE_SLOTS];

bool          faces_synced   = false;  // true once a complete SD dump landed
uint8_t       faceDumpCount  = 0;      // FACE_OP_DATA packets since last REQ
unsigned long faceReq_previousMillis   = 0;
unsigned long faceDirty_previousMillis = 0;
const unsigned long faceReq_interval   = 2500;  // re-request dump until synced
const unsigned long faceDirty_interval = 2000;  // push one dirty face per tick

// Preset overlay: a recalled face holds each channel until that channel's own
// physical control moves (same frozen-baseline takeover as dualPick), and
// each toggle until its physical switch is flipped.
int16_t preset_v[FACE_VALUES];
bool    preset_on[FACE_VALUES] = { false };
uint8_t preset_toggles     = 0;   // recalled toggle states
uint8_t preset_toggle_mask = 0;   // bit set = that toggle still preset-driven
int16_t phys_baseline[FACE_VALUES];
bool    phys_baseline_init = false;
uint8_t toggles_phys_last  = 0;

// Right keypad (face keypad) press/hold tracking + save-confirm prompt
#define FACE_HOLD_MS            3000   // hold this long to open the save prompt
#define FACE_PROMPT_TIMEOUT_MS 10000   // unanswered prompt cancels itself
int8_t        rk_down       = -1;     // scan code currently held, -1 = none
unsigned long rk_down_since = 0;
bool          rk_hold_fired = false;  // this press already opened the prompt
bool          rk_consumed   = false;  // this press answered the prompt
char          face_prompt_key     = 0;   // 0 = no prompt showing
unsigned long face_prompt_started = 0;
unsigned long faceMsg_clearAt     = 0;   // 0 = message is not timed

// Incoming face packets: OnDataRecv (WiFi task) only memcpys into this ring;
// loop() drains it. Single producer / single consumer, volatile indexes.
#define FACE_RXQ 8
espnow_face_pkt_t faceRxQ[FACE_RXQ];
volatile uint8_t  faceRxHead = 0;
volatile uint8_t  faceRxTail = 0;
// --- END FACE PRESETS ---


// Timers
unsigned long tft_update_previousMillis = 0;
unsigned long battery_01_previousMillis = 0;
unsigned long keypad_previousMillis     = 0;
unsigned long control_tx_previousMillis = 0;
const unsigned long tft_update_interval = 40;   // 25 fps
const unsigned long battery_01_interval = 500;
const unsigned long keypad_interval     = 150;
const unsigned long control_tx_interval = 40;   // 25 Hz control packets

// Set by OnDataRecv (WiFi task), consumed by loop(): forward the completed
// jukebox file list to j4_display_left from loop context only.
volatile bool filelistForwardPending = false;


// Two 4x4 matrix keypads, each on its own PCF8574 I2C backpack:
//   keypad_left  -- the NEW keypad, left side of the panel. ADDR jumpered to
//                   0x21 (A0 high). If its backpack is a PCF8574A, the same
//                   jumper lands at 0x39 instead -- change the two 0x21s.
//   keypad_right -- the original keypad at 0x20 (all jumpers low).
// For now both drive exactly the same phrase-select logic.
PCF8574 pcf_left(0x21);
PCF8574 pcf_right(0x20);
TFT_eSPI tft = TFT_eSPI();
TFT_eSprite screen_bottom_sprite_203 = TFT_eSprite(&tft);

// Keypad wiring: keypad pin 1 plugs into P0 straight through.
// Actual pin→signal: P0=Row3, P1=Row2, P2=Row1, P3=Col4, P4=Col3, P5=Col2, P6=Col1, P7=Row4
// (P3 and P7 are swapped vs the I2CKeyPad library's expectation, so we scan manually.)
// Keymaps indexed by (drive_index*4 + read_index), drive order: P0,P1,P2,P7;
// read order: P3,P4,P5,P6. N = NoKey, F = Fail.
// keymap_right was derived on the bench for the original keypad. The left
// keypad is a different model on the same backpack wiring, so it starts with
// the same map -- if its keys come out scrambled, re-derive just that string
// (press each key, note the character that appears, rearrange to match).
char keymap_left[19]  = "#9630852*741DCBANF";
char keymap_right[19] = "#9630852*741DCBANF";
char last_key_char    = 'N';   // last decoded key, shown on the data screen
int key      = -2;
int old_key  = -1;
String phrase_select_buffer = "";
bool ready_message = true;


// Drive row pins LOW one at a time, check if any col pin reads LOW.
// Drive order: P0,P1,P2,P7 (= Row3,Row2,Row1,Row4)
// Read  order: P3,P4,P5,P6 (= Col4,Col3,Col2,Col1)
static const uint8_t KP_DRIVE_WRITE[4] = { 0xFE, 0xFD, 0xFB, 0x7F }; // ~(1<<P) for P=0,1,2,7
static const uint8_t KP_READ_MASK      = 0x78;  // bits 3,4,5,6 = P3,P4,P5,P6

bool kpIsPressed(PCF8574 &pcf) {
  pcf.write8(0x78);  // all row pins LOW, all col pins HIGH-Z
  return (pcf.read8() & KP_READ_MASK) != KP_READ_MASK;
}

uint8_t kpGetKey(PCF8574 &pcf) {
  static const uint8_t READ_BIT[4] = { 3, 4, 5, 6 };  // P3,P4,P5,P6
  for (uint8_t d = 0; d < 4; d++) {
    pcf.write8(KP_DRIVE_WRITE[d]);
    uint8_t val = pcf.read8();
    for (uint8_t r = 0; r < 4; r++) {
      if (!(val & (1 << READ_BIT[r]))) {
        pcf.write8(0xFF);
        return d * 4 + r;
      }
    }
  }
  pcf.write8(0xFF);
  return 16;  // NoKey
}

// raw <= 100: noise floor; raw >= 65000: ADC overflow; 17000: pot physical max
int processPot(int raw, int out_max) {
  if (raw <= 100 || raw >= 65000) return 0;
  if (raw > 17000) raw = 17000;
  return map(raw, 0, 17000, 0, out_max);
}

// Dual-control arbitration: two pots drive one function (rotary + fader) and
// must never fight. Whichever control moved last (by more than the claim
// threshold, on the processPot 0-255 scale) becomes the active source and
// its value is used until the other one moves. A source that is not ok
// (module unplugged, display feed down) cannot claim or hold active status.
// First sighting of a source only baselines it -- it has to actually MOVE
// to claim, so power-up doesn't randomly hand control to a fader.
#define DUAL_CLAIM_COUNTS 4   // ~1.5% of travel; above ADS1115 pot noise

struct DualPot {
  int  lastA, lastB;    // last seen values (-1 = not seen yet)
  bool bActive;         // true = source B (the fader) is active
};
DualPot dual_iris        = { -1, -1, false };
DualPot dual_nose_basket = { -1, -1, false };

int dualPick(DualPot &d, int a, bool aOk, int b, bool bOk) {
  // The ACTIVE source's baseline tracks its value; the INACTIVE source's
  // baseline stays frozen where it was released, so even a slow turn
  // accumulates enough travel to claim (per-cycle deltas never would).
  if (aOk) {
    if (d.lastA < 0 || !d.bActive) d.lastA = a;   // first sight or active: track
    else if (abs(a - d.lastA) >= DUAL_CLAIM_COUNTS) { d.bActive = false; d.lastA = a; }
  } else {
    d.lastA = -1;
  }
  if (bOk) {
    if (d.lastB < 0 || d.bActive) d.lastB = b;    // first sight or active: track
    else if (abs(b - d.lastB) >= DUAL_CLAIM_COUNTS) { d.bActive = true; d.lastB = b; }
  } else {
    d.lastB = -1;
  }
  if (d.bActive && !bOk) d.bActive = false;   // active source vanished
  if (!d.bActive && !aOk && bOk) d.bActive = true;
  return d.bActive ? b : a;
}

// I2C presence guards. Two traps when a device is missing:
//  1. The ADS1X15 library's readADC() has NO timeout: with no chip on the
//     bus, isBusy() never clears and readADC() spins forever, freezing
//     loop() (screen dead, button dead, no ESP-NOW).
//  2. With NO modules attached at all, the bus has no pull-ups (they live on
//     the breakout boards), so a transaction doesn't fast-NACK -- it eats the
//     driver timeout + bus recovery, hundreds of ms EACH. Probing every
//     absent device every cycle made loop() spend ~2s blocked per pass.
// So: transactions are capped short (Wire.setTimeOut in setup), a device
// must ACK before it is trusted, and an ABSENT device is only re-probed
// every I2C_REPROBE_MS. A present device ACKs in microseconds, so verifying
// it on every use costs nothing, and hot-plugged modules start working
// within a second.
#define I2C_REPROBE_MS 1000

bool i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

struct AdsGuard {
  ADS1115 &ads;
  uint8_t addr;
  bool configured;
  unsigned long lastProbe;
  bool probedOnce;
  AdsGuard(ADS1115 &a, uint8_t adr)
    : ads(a), addr(adr), configured(false), lastProbe(0), probedOnce(false) {}
};

AdsGuard adsg_01(ADS_01, 0x48);
AdsGuard adsg_02(ADS_02, 0x49);
AdsGuard adsg_03(ADS_03, 0x4A);
AdsGuard adsg_04(ADS_04, 0x4B);

// Table driving the four ADS1115 pot-value screens (screen_mode 3-6), one
// row per module so adsPotsDisplay() doesn't need four near-duplicate
// functions. A null value pointer (ADS_04 A3, spare) just prints "--".
struct AdsPotScreen {
  AdsGuard   &guard;
  const char *title;
  const char *labels[4];
  int        *values[4];
};

AdsPotScreen adsPotScreens[4] = {
  { adsg_01, "ADS_01  0x48",
    { "EYES X", "EYES Y", "NECK X", "JAW Y" },
    { &eyes_x_value, &eyes_y_value, &neck_value, &jaw_value } },
  { adsg_02, "ADS_02  0x49",
    { "BROW L", "BROW R", "BBRW L", "BBRW R" },
    { &eyebrow_l_value, &eyebrow_r_value, &basket_brow_l_value, &basket_brow_r_value } },
  { adsg_03, "ADS_03  0x4A",
    { "NOSE", "NOSE BK", "EYELID L", "EYELID R" },
    { &nose_value, &nose_basket_value, &eyelid_l_value, &eyelid_r_value } },
  { adsg_04, "ADS_04  0x4B",
    { "NECK PIV", "FADER L", "FADER R", "SPARE" },
    { &neck_pivot_value, &fader_left_value, &fader_right_value, nullptr } },
};

bool adsReady(AdsGuard &g) {
  if (!g.configured) {   // absent (or never seen): probe only once a second
    unsigned long now = millis();
    if (g.probedOnce && (now - g.lastProbe < I2C_REPROBE_MS)) return false;
    g.probedOnce = true;
    g.lastProbe  = now;
  }
  if (!i2cPresent(g.addr)) {
    g.configured = false;
    return false;
  }
  if (!g.configured) {
    g.ads.begin();
    g.ads.setGain(0);      //  0 is ±6.144V    1 is ±4.096V    2 is ±2.048V
    g.ads.setDataRate(7);  //  0 = slow   4 = medium   7 = fast
    g.ads.setMode(1);      //  0 = continuous mode   1 = single mode
    g.ads.requestADC(0);   //  first read to trigger
    g.configured = true;
  }
  return true;
}

// Same guard for the PCF8574 keypad expanders: absent, their floating-bus
// reads look like a key held down forever, so no scan unless the chip ACKs.
struct KeypadGuard {
  PCF8574      &pcf;
  uint8_t       addr;
  const char   *keymap;
  bool          present;
  unsigned long lastProbe;
  bool          probedOnce;
  KeypadGuard(PCF8574 &p, uint8_t a, const char *km)
    : pcf(p), addr(a), keymap(km), present(false), lastProbe(0), probedOnce(false) {}
};

KeypadGuard kpg_left (pcf_left,  0x21, keymap_left);
KeypadGuard kpg_right(pcf_right, 0x20, keymap_right);

bool keypadReady(KeypadGuard &g) {
  if (!g.present) {   // absent (or never seen): probe only once a second
    unsigned long now = millis();
    if (g.probedOnce && (now - g.lastProbe < I2C_REPROBE_MS)) return false;
    g.probedOnce = true;
    g.lastProbe  = now;
  }
  g.present = i2cPresent(g.addr);
  return g.present;
}


// --- FACE PRESET HELPERS ---

// The talk chain is usable when the receiver's status packets are fresh AND
// the receiver reports the Teensy alive. Never a blocking check.
bool talkLinkUp() {
  return (millis() - lastStatusRecvMs < STATUS_LINK_TIMEOUT_MS)
      && rcvData.talk_ok_rcv;
}

FaceSlot *faceFind(char kc) {
  for (uint8_t i = 0; i < FACE_SLOTS; i++)
    if (faces[i].key == kc) return &faces[i];
  return NULL;
}

FaceSlot *faceAlloc(char kc) {
  FaceSlot *f = faceFind(kc);
  if (f) return f;
  for (uint8_t i = 0; i < FACE_SLOTS; i++)
    if (faces[i].key == 0) return &faces[i];
  return NULL;   // cannot happen: 15 possible keys, 16 slots
}

// Message on j4_display_right (Serial2 TX). showMs = 0 leaves it up until
// replaced or cleared; otherwise loop() sends "X:" after showMs. Fire and
// forget: if the display is unplugged the bytes just fall on the floor, and
// the display self-clears a stale message after 15s anyway.
void faceMsg(const char *l1, const char *l2, unsigned long showMs) {
  Serial2.printf("M:%s|%s\n", l1, l2);
  faceMsg_clearAt = showMs ? millis() + showMs : 0;
}

void facePromptOpen(char kc) {
  face_prompt_key     = kc;
  face_prompt_started = millis();
  char l1[24];
  snprintf(l1, sizeof(l1), "SAVE FACE ON %c?", kc);
  faceMsg(l1, "PRESS * TO CONFIRM", 0);
}

void facePromptCancel(const char *why) {
  face_prompt_key = 0;
  faceMsg("SAVE CANCELLED", why, 2500);
}

// Snapshot the CURRENT face -- the effective values (post arbitration and
// preset overlay), i.e. exactly what the robot's face looks like right now.
void faceSaveConfirmed() {
  char kc = face_prompt_key;
  face_prompt_key = 0;
  FaceSlot *f = faceAlloc(kc);
  if (!f) { faceMsg("SAVE FAILED", "NO FREE SLOTS", 2500); return; }
  f->key       = kc;
  f->valid     = true;
  f->dirty     = true;
  f->sd_failed = false;
  f->v[FI_IRIS]        = iris_value;
  f->v[FI_COLOR]       = color_value;
  f->v[FI_BRIGHT]      = brightness_value;
  f->v[FI_BROW_L]      = eyebrow_l_value;
  f->v[FI_BROW_R]      = eyebrow_r_value;
  f->v[FI_BBROW_L]     = basket_brow_l_value;
  f->v[FI_BBROW_R]     = basket_brow_r_value;
  f->v[FI_NOSE]        = nose_value;
  f->v[FI_NOSE_BASKET] = nose_basket_value;
  f->v[FI_LID_L]       = eyelid_l_value;
  f->v[FI_LID_R]       = eyelid_r_value;
  f->toggles = (laser_toggle   << TOGGLE_BIT_LASER)
             | (vent_toggle    << TOGGLE_BIT_VENT)
             | (eye_pop_toggle << TOGGLE_BIT_EYE_POP);
  char l1[24];
  snprintf(l1, sizeof(l1), "FACE SAVED ON %c", kc);
  faceMsg(l1, talkLinkUp() ? "WRITING TO SD..." : "PENDING SD: TALK OFFLINE", 2500);
}

void faceRecall(char kc) {
  char l1[24];
  FaceSlot *f = faceFind(kc);
  if (!f || !f->valid) {
    snprintf(l1, sizeof(l1), "NO FACE ON %c", kc);
    faceMsg(l1, "HOLD 3s TO SAVE ONE", 2500);
    return;
  }
  for (uint8_t i = 0; i < FACE_VALUES; i++) {
    preset_v[i]  = f->v[i];
    preset_on[i] = true;
  }
  preset_toggles     = f->toggles;
  preset_toggle_mask = (1 << TOGGLE_BIT_LASER) | (1 << TOGGLE_BIT_VENT)
                     | (1 << TOGGLE_BIT_EYE_POP);
  // phys_baseline stops updating while preset_on, so takeover measures total
  // travel since this exact moment.
  snprintf(l1, sizeof(l1), "FACE %c RECALLED", kc);
  faceMsg(l1, "MOVE A POT TO RETAKE IT", 2000);
}

void faceSendPkt(uint8_t op, const FaceSlot *f) {
  espnow_face_pkt_t pkt;
  memset(&pkt, 0, sizeof(pkt));
  pkt.pkt_type = ESPNOW_PKT_FACE;
  pkt.op       = op;
  if (f) {
    pkt.key     = (uint8_t)f->key;
    pkt.toggles = f->toggles;
    memcpy(pkt.v, f->v, sizeof(pkt.v));
  }
  esp_now_send(broadcastAddress, (uint8_t *)&pkt, sizeof(pkt));
}

// Drain face packets staged by OnDataRecv. Runs in loop context only.
void processFacePackets() {
  char l1[24];
  while (faceRxTail != faceRxHead) {
    espnow_face_pkt_t pkt;
    memcpy(&pkt, (const void *)&faceRxQ[faceRxTail], sizeof(pkt));
    faceRxTail = (uint8_t)((faceRxTail + 1) % FACE_RXQ);

    if (pkt.op == FACE_OP_DATA) {
      faceDumpCount++;
      FaceSlot *f = faceAlloc((char)pkt.key);
      // A locally-dirty slot is newer than the SD copy -- keep ours, it will
      // be pushed by the dirty timer and come back clean.
      if (f && !f->dirty) {
        f->key     = (char)pkt.key;
        f->valid   = true;
        f->dirty   = false;
        f->sd_failed = false;
        f->toggles = pkt.toggles;
        memcpy(f->v, pkt.v, sizeof(f->v));
      }

    } else if (pkt.op == FACE_OP_END) {
      // Complete only if every face in the dump actually landed; otherwise
      // stay unsynced and the REQ timer asks again (idempotent).
      faces_synced = (faceDumpCount == (uint8_t)pkt.v[0]);

    } else if (pkt.op == FACE_OP_ACK) {
      FaceSlot *f = faceFind((char)pkt.key);
      if (f) f->dirty = false;
      snprintf(l1, sizeof(l1), "FACE %c ON SD", (char)pkt.key);
      faceMsg(l1, "", 1500);

    } else if (pkt.op == FACE_OP_ERR) {
      FaceSlot *f = faceFind((char)pkt.key);
      if (f) f->sd_failed = true;   // keep valid + dirty, stop auto-retrying
      faceMsg("SD WRITE FAILED", "FACE KEPT IN CONTROLLER", 3000);
    }
  }
}
// --- END FACE PRESET HELPERS ---


void setup() {
  Wire.begin(SDA, SCL);
  // Cap each I2C transaction. With no modules attached the bus has no
  // pull-ups and a transaction eats driver timeout + recovery instead of a
  // fast NACK; uncapped that is hundreds of ms per attempt. A healthy
  // transaction completes in well under 1ms, so 10ms is generous.
  Wire.setTimeOut(10);

  pcf_left.begin();    // both keypad expanders; fine if either is absent
  pcf_right.begin();
  Serial.begin(115200);

  // Probe + configure whichever ADS1115 modules are actually on the bus.
  // Missing ones are fine: their channels read 0 and they are re-probed
  // every I2C_REPROBE_MS, so plugging one in just starts working.
  adsReady(adsg_01);
  adsReady(adsg_02);
  adsReady(adsg_03);
  adsReady(adsg_04);  // A0 neck-pivot, A1 fader_left, A2 fader_right, A3 spare

  // Panel toggles: switch closes to GND, so ON reads LOW
  pinMode(LASER_TOGGLE_PIN,   INPUT_PULLUP);
  pinMode(VENT_TOGGLE_PIN,    INPUT_PULLUP);
  pinMode(EYE_POP_TOGGLE_PIN, INPUT_PULLUP);
  pinMode(AUX_TOGGLE_PIN,     INPUT_PULLUP);

  // XIAO display links - init after ADS1115 to avoid I2C/UART peripheral conflict
  Serial1.begin(XIAO_BAUD, SERIAL_8N1, XIAO_RX_PIN,  XIAO_TX_PIN);    // j4_display_left
  Serial2.begin(XIAO_BAUD, SERIAL_8N1, XIAOR_RX_PIN, XIAOR_TX_PIN);   // j4_display_right

  WiFi.mode(WIFI_STA);
  Serial.println("MAC Address: ");
  Serial.println(WiFi.macAddress());
  WiFi.setSleep(false);

  // RF hardening for crowded 2.4GHz venues (must match j4_receiver):
  // LR = Espressif's proprietary long-range PHY. Both ends are ESP32s, so the
  // link gains ~4dB sensitivity and ordinary WiFi gear cannot even decode it.
  // Max TX power = 19.5dBm (units of 0.25dBm).
  // Channel 6 (not the power-on default of 1) dodges every other ESP-NOW
  // project left on its default channel. Both ends must match; if the venue
  // is ugly on 6, change ESPNOW_CHANNEL on BOTH boards (1/6/11 only) + reflash.
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);
  esp_wifi_set_max_tx_power(78);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    connectStatus = "init error";
    connectError = HIGH;
    return;
  }
  connectStatus = "init OK";

  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    connectStatus = "no peer added";
    connectError = HIGH;
    return;
  }
  connectStatus = "peer added";

  xmitData.pkt_type = ESPNOW_PKT_CONTROL;
  xmitData.phrase_select_xmit[0] = '\0';

  tft.init();
  tft.setRotation(2);
  tft.fillScreen(TFT_BLACK);
  tft.setSwapBytes(true);

  screen_bottom_sprite_203.createSprite(135, 203);
  tft.pushImage(0, 0, 135, 37, kevco_labs_logo_02);
  screen_bottom_sprite_203.setTextColor(TFT_GREEN, TFT_BLACK);
  labelsDisplaySprite();
  screen_bottom_sprite_203.drawString("Ready...", 0, 0, 1);

  pinMode(SCREEN_BUTTON, INPUT_PULLUP);   // TTGO built-in button (GPIO 35)
}


void loop() {
  unsigned long currentMillis = millis();

  controllerScreenModeDetect();

  if (currentMillis - tft_update_previousMillis >= tft_update_interval) {
    tft_update_previousMillis = currentMillis;
    tftDisplayUpdate();
    sendToXIAO();
  }

  // XIAO display: file-list requests + heartbeat ("PING") for connection status
  while (Serial1.available()) {
    char c = (char)Serial1.read();
    if (c == '\n') {
      xiaoSerialBuf.trim();
      // Only lines j4_display_left actually sends count as its heartbeat --
      // with the display unplugged the floating RX pin generates garbage
      // lines, which used to fake a CONNECTED status.
      if (xiaoSerialBuf == "PING" || xiaoSerialBuf == "LIST?") lastDisplayMs = millis();
      if (xiaoSerialBuf == "LIST?" && jukeboxReady) sendFileListToXIAO();
      xiaoSerialBuf = "";
    } else if (c != '\r') {
      if (xiaoSerialBuf.length() > 16) xiaoSerialBuf = "";  // line noise guard
      xiaoSerialBuf += c;
    }
  }

  // j4_display_right: "P:<iris>,<color>,<brightness>,<volume>" pot feed (raw
  // ADS1115 counts from its dedicated ADS1115). Any valid line = board alive.
  while (Serial2.available()) {
    char c = (char)Serial2.read();
    if (c == '\n') {
      int i, col, b, v;
      if (sscanf(xiaoRSerialBuf.c_str(), "P:%d,%d,%d,%d", &i, &col, &b, &v) == 4) {
        dispR_iris_raw       = i;
        dispR_color_raw      = col;
        dispR_brightness_raw = b;
        dispR_volume_raw     = v;
        lastDisplayRMs = millis();
      }
      xiaoRSerialBuf = "";
    } else if (c != '\r') {
      if (xiaoRSerialBuf.length() > 40) xiaoRSerialBuf = "";  // line noise guard
      xiaoRSerialBuf += c;
    }
  }

  // --- CONTROL TX (25 Hz): read inputs, fill the packet, send ---
  // This whole block MUST stay rate-limited. When it ran every loop pass the
  // pass rate was only held down by the blocking ADC reads; with no ADCs
  // attached the loop spun at multi-kHz, esp_now_send() fired thousands of
  // times a second, and the OnDataSent callback storm corrupted the heap
  // (lockup ~2s after boot). 25 Hz matches the receiver's 20ms packet gate.
  if (currentMillis - control_tx_previousMillis >= control_tx_interval) {
  control_tx_previousMillis = currentMillis;

  // --- ADC READS ---
  // Each module is probed before its channels are read: readADC() on an
  // absent chip never returns (see adsReady()). ADS_04 (0x4B) is spare.
  if (adsReady(adsg_01)) {
    eyes_x_value    = processPot(ADS_01.readADC(0), 255);   // eyes joystick X
    eyes_y_value    = processPot(ADS_01.readADC(1), 255);   // eyes joystick Y
    neck_value      = processPot(ADS_01.readADC(2), 3200);  // neck joystick X
    jaw_value       = processPot(ADS_01.readADC(3), 3200);  // neck joystick Y
  } else {
    eyes_x_value = eyes_y_value = 0;
    neck_value   = jaw_value    = 1600;   // joystick centre, not hard-over
  }
  if (adsReady(adsg_02)) {
    eyebrow_l_value     = processPot(ADS_02.readADC(0), 255);  // Eyebrow L
    eyebrow_r_value     = processPot(ADS_02.readADC(1), 255);  // Eyebrow R
    basket_brow_l_value = processPot(ADS_02.readADC(2), 255);  // Basket Eyebrow L
    basket_brow_r_value = processPot(ADS_02.readADC(3), 255);  // Basket Eyebrow R
  } else {
    eyebrow_l_value = eyebrow_r_value = basket_brow_l_value = basket_brow_r_value = 0;
  }
  bool ads3_ok = adsReady(adsg_03);
  if (ads3_ok) {
    nose_value          = processPot(ADS_03.readADC(0), 255);  // Nose (up/down)
    nose_basket_value   = processPot(ADS_03.readADC(1), 255);  // Nose Basket (up/down)
    eyelid_l_value      = processPot(ADS_03.readADC(2), 255);  // Bottom Eyelid L
    eyelid_r_value      = processPot(ADS_03.readADC(3), 255);  // Bottom Eyelid R
  } else {
    nose_value = nose_basket_value = eyelid_l_value = eyelid_r_value = 0;
  }
  bool ads4_ok = adsReady(adsg_04);
  if (ads4_ok) {
    neck_pivot_value  = processPot(ADS_04.readADC(0), 3200);  // neck-pivot pot
    fader_left_value  = processPot(ADS_04.readADC(1), 255);   // fader_left  -> Nose Basket
    fader_right_value = processPot(ADS_04.readADC(2), 255);   // fader_right -> iris
  } else {
    neck_pivot_value = 1600;   // hold centre (matches the receiver's old placeholder)
    fader_left_value = fader_right_value = 0;
  }
  // Remote pots from j4_display_right (same raw scale -> same processPot)
  bool dispR_ok    = (millis() - lastDisplayRMs < DISPLAY_TIMEOUT_MS);
  iris_value       = processPot(dispR_iris_raw, 255);
  color_value      = processPot(dispR_color_raw, 255);
  brightness_value = processPot(dispR_brightness_raw, 255);
  volume_value     = processPot(dispR_volume_raw, 100);
  // Dual-control arbitration: iris = IRIS pot vs fader_right, Nose Basket =
  // rotary pot vs fader_left. Last mover wins; see dualPick().
  iris_value        = dualPick(dual_iris, iris_value, dispR_ok,
                               fader_right_value, ads4_ok);
  nose_basket_value = dualPick(dual_nose_basket, nose_basket_value, ads3_ok,
                               fader_left_value, ads4_ok);
  // --- END ADC READS ---

  // Panel toggles: INPUT_PULLUP, switch closes to GND, so ON = LOW
  laser_toggle   = (digitalRead(LASER_TOGGLE_PIN)   == LOW);
  vent_toggle    = (digitalRead(VENT_TOGGLE_PIN)    == LOW);
  eye_pop_toggle = (digitalRead(EYE_POP_TOGGLE_PIN) == LOW);
  aux_toggle     = (digitalRead(AUX_TOGGLE_PIN)     == LOW);

  // --- FACE PRESET OVERLAY ---
  // A recalled face holds each channel until that channel's own physical
  // control moves, and each toggle until its switch is flipped. Baselines
  // only track while no preset holds the channel, so takeover measures
  // total travel since the recall (a slow turn still gets there).
  {
    int16_t phys[FACE_VALUES] = {
      (int16_t)iris_value, (int16_t)color_value, (int16_t)brightness_value,
      (int16_t)eyebrow_l_value, (int16_t)eyebrow_r_value,
      (int16_t)basket_brow_l_value, (int16_t)basket_brow_r_value,
      (int16_t)nose_value, (int16_t)nose_basket_value,
      (int16_t)eyelid_l_value, (int16_t)eyelid_r_value
    };
    uint8_t tphys = (laser_toggle   << TOGGLE_BIT_LASER)
                  | (vent_toggle    << TOGGLE_BIT_VENT)
                  | (eye_pop_toggle << TOGGLE_BIT_EYE_POP);
    if (!phys_baseline_init) {
      memcpy(phys_baseline, phys, sizeof(phys_baseline));
      toggles_phys_last  = tphys;
      phys_baseline_init = true;
    }
    for (uint8_t i = 0; i < FACE_VALUES; i++) {
      if (preset_on[i] && abs(phys[i] - phys_baseline[i]) >= DUAL_CLAIM_COUNTS)
        preset_on[i] = false;                     // pot moved: it takes over
      if (!preset_on[i]) phys_baseline[i] = phys[i];
    }
    preset_toggle_mask &= ~(tphys ^ toggles_phys_last);   // flipped = reclaimed
    toggles_phys_last = tphys;

    if (preset_on[FI_IRIS])        iris_value          = preset_v[FI_IRIS];
    if (preset_on[FI_COLOR])       color_value         = preset_v[FI_COLOR];
    if (preset_on[FI_BRIGHT])      brightness_value    = preset_v[FI_BRIGHT];
    if (preset_on[FI_BROW_L])      eyebrow_l_value     = preset_v[FI_BROW_L];
    if (preset_on[FI_BROW_R])      eyebrow_r_value     = preset_v[FI_BROW_R];
    if (preset_on[FI_BBROW_L])     basket_brow_l_value = preset_v[FI_BBROW_L];
    if (preset_on[FI_BBROW_R])     basket_brow_r_value = preset_v[FI_BBROW_R];
    if (preset_on[FI_NOSE])        nose_value          = preset_v[FI_NOSE];
    if (preset_on[FI_NOSE_BASKET]) nose_basket_value   = preset_v[FI_NOSE_BASKET];
    if (preset_on[FI_LID_L])       eyelid_l_value      = preset_v[FI_LID_L];
    if (preset_on[FI_LID_R])       eyelid_r_value      = preset_v[FI_LID_R];
    if (preset_toggle_mask & (1 << TOGGLE_BIT_LASER))
      laser_toggle   = (preset_toggles >> TOGGLE_BIT_LASER)   & 1;
    if (preset_toggle_mask & (1 << TOGGLE_BIT_VENT))
      vent_toggle    = (preset_toggles >> TOGGLE_BIT_VENT)    & 1;
    if (preset_toggle_mask & (1 << TOGGLE_BIT_EYE_POP))
      eye_pop_toggle = (preset_toggles >> TOGGLE_BIT_EYE_POP) & 1;
  }
  // --- END FACE PRESET OVERLAY ---

  eye_pop_value  = eye_pop_toggle ? 3200 : 0;  // popped / normal

  // Dead zone: snap joystick axes to center if within threshold
  if (abs(neck_value - 1600) <= JOYSTICK_DEAD_ZONE) neck_value = 1600;
  if (abs(jaw_value  - 1600) <= JOYSTICK_DEAD_ZONE) jaw_value  = 1600;

  // Neck mixer: Y sets base height, X steers left/right differentially
  neck_left_value  = constrain(jaw_value + (neck_value - 1600), 0, 3200);
  neck_right_value = constrain(jaw_value - (neck_value - 1600), 0, 3200);

  xmitData.volume_xmit      = volume_value;
  xmitData.iris_xmit        = iris_value;
  xmitData.color_xmit       = color_value;
  xmitData.brightness_xmit  = brightness_value;
  xmitData.eyes_x_xmit      = eyes_x_value;
  xmitData.eyes_y_xmit      = eyes_y_value;
  xmitData.eye_pop_xmit     = eye_pop_value;
  xmitData.neck_left_xmit   = neck_left_value;
  xmitData.neck_right_xmit  = neck_right_value;
  xmitData.neck_pivot_xmit  = neck_pivot_value;
  xmitData.eyebrow_l_xmit     = eyebrow_l_value;
  xmitData.eyebrow_r_xmit     = eyebrow_r_value;
  xmitData.basket_brow_l_xmit = basket_brow_l_value;
  xmitData.basket_brow_r_xmit = basket_brow_r_value;
  xmitData.nose_xmit          = nose_value;
  xmitData.nose_basket_xmit   = nose_basket_value;
  xmitData.eyelid_l_xmit      = eyelid_l_value;
  xmitData.eyelid_r_xmit      = eyelid_r_value;
  xmitData.toggles_xmit       = (laser_toggle   << TOGGLE_BIT_LASER)
                              | (vent_toggle    << TOGGLE_BIT_VENT)
                              | (eye_pop_toggle << TOGGLE_BIT_EYE_POP)
                              | (aux_toggle     << TOGGLE_BIT_AUX);
  xmitData.need_filelist_xmit = jukeboxReady ? 0 : 1;
  xmitData.display_l_ok_xmit  = (millis() - lastDisplayMs  < DISPLAY_TIMEOUT_MS) ? 1 : 0;
  xmitData.display_r_ok_xmit  = (millis() - lastDisplayRMs < DISPLAY_TIMEOUT_MS) ? 1 : 0;

  esp_now_send(broadcastAddress, (uint8_t *)&xmitData, sizeof(xmitData));

  // Render the last send result (flag set by OnDataSent in the WiFi task)
  // into the display String from loop context only -- String ops inside the
  // callback raced loop()'s heap use and corrupted it.
  connectStatus = connectError ? "xmit failed" : "xmit success";
  }
  // --- END CONTROL TX ---

  // File list completed by OnDataRecv (WiFi task): forward it to the display
  // from here so Serial1 is only ever written from loop context.
  if (filelistForwardPending) {
    filelistForwardPending = false;
    sendFileListToXIAO();
  }

  // --- BATTERY RELATED ---
  if (currentMillis - battery_01_previousMillis >= battery_01_interval) {
    battery_01_previousMillis = currentMillis;

    float product = 0.0018276 * analogRead(34);  // IO34 is battery voltage
    bat1_mv = (uint16_t)(product * 1000.0f);

    char str[10];
    dtostrf(product, 4, 2, str);

    String voltage_battery_01 = String(str) + "V";
    String voltage_battery_02 = "6.00V";   // PLACEHOLDER
    String voltage_battery_03 = "12.00V";  // PLACEHOLDER

    screen_bottom_sprite_203.setTextColor(TFT_BLACK);
    screen_bottom_sprite_203.fillRect( 0, 185, 40, 15, TFT_GREEN);
    screen_bottom_sprite_203.drawString(voltage_battery_01,  2, 185, 2);
    screen_bottom_sprite_203.fillRect(43, 185, 40, 15, TFT_GREEN);
    screen_bottom_sprite_203.drawString(voltage_battery_02, 45, 185, 2);
    screen_bottom_sprite_203.fillRect(86, 185, 49, 15, TFT_GREEN);
    screen_bottom_sprite_203.drawString(voltage_battery_03, 88, 185, 2);
  }
  // --- END BATTERY RELATED ---

  if (currentMillis - keypad_previousMillis >= keypad_interval) {
    keypad_previousMillis = currentMillis;

    // LEFT keypad -- phrase select (jukebox). The right keypad became the
    // face keypad in v0_6_22 and is scanned separately below.
    KeypadGuard &pad = kpg_left;
    if (keypadReady(pad) && kpIsPressed(pad.pcf)) {
      uint8_t rawKey = kpGetKey(pad.pcf);

      if (rawKey <= 15) {  // 16 = NoKey - only promote to global on valid read
      key = (int)rawKey;

      if (ready_message) {
        screen_bottom_sprite_203.fillRect(0, 0, 135, 20, TFT_BLACK);
        ready_message = false;
      } else {
        screen_bottom_sprite_203.fillRect(70, 0, 65, 20, TFT_BLACK);
      }

      if (key != old_key) {
        char kc = pad.keymap[key];
        last_key_char = kc;

        if (kc == '#') {  // reset same-key lock and clear buffer
          old_key = -1;
          phrase_select_buffer = "";
          screen_bottom_sprite_203.fillRect(70,  0, 65, 20, TFT_BLACK);
          screen_bottom_sprite_203.fillRect(70, 20, 65, 20, TFT_BLACK);

        } else if (kc == '*') {  // stop playback
          old_key = -1;
          phrase_select_buffer = "STOP";
          strncpy(xmitData.phrase_select_xmit, phrase_select_buffer.c_str(), sizeof(xmitData.phrase_select_xmit) - 1);
          xmitData.phrase_select_xmit[sizeof(xmitData.phrase_select_xmit) - 1] = '\0';
          screen_bottom_sprite_203.setTextColor(TFT_GREEN);
          screen_bottom_sprite_203.fillRect(70,  0, 65, 20, TFT_BLACK);
          screen_bottom_sprite_203.drawString("*",                  70,  0, 2);
          screen_bottom_sprite_203.fillRect(70, 20, 65, 20, TFT_BLACK);
          screen_bottom_sprite_203.drawString(phrase_select_buffer, 70, 20, 2);
          phrase_select_buffer = "";

        } else {
          old_key = key;

          if (kc >= 'A' && kc <= 'D') {  // Letter prefix - wait for digit
            phrase_select_buffer = String(kc);
            screen_bottom_sprite_203.setTextColor(TFT_GREEN);
            screen_bottom_sprite_203.fillRect(70, 20, 65, 20, TFT_BLACK);
            screen_bottom_sprite_203.drawString(String(kc) + "_", 70, 20, 2);
          } else {
            if (phrase_select_buffer.length() == 0) phrase_select_buffer = "0";
            phrase_select_buffer += String(kc);

            screen_bottom_sprite_203.setTextColor(TFT_GREEN);
            screen_bottom_sprite_203.fillRect(70, 20, 65, 20, TFT_BLACK);
            screen_bottom_sprite_203.drawString(phrase_select_buffer + ".wav", 70, 20, 2);

            strncpy(xmitData.phrase_select_xmit, phrase_select_buffer.c_str(), sizeof(xmitData.phrase_select_xmit) - 1);
            xmitData.phrase_select_xmit[sizeof(xmitData.phrase_select_xmit) - 1] = '\0';
            phrase_select_buffer = "";
          }
        }
      }
      }  // end key <= 15 guard
    }

    // RIGHT keypad -- FACE PRESETS. Tap = recall that key's face. Hold >= 3s
    // (any key but '*') = save prompt on j4_display_right; while the prompt
    // is up, '*' confirms and any other key cancels. Pure state tracking at
    // the same 150ms scan cadence -- nothing here blocks.
    bool rk_pressed = keypadReady(kpg_right) && kpIsPressed(kpg_right.pcf);
    uint8_t rk_raw  = 16;
    if (rk_pressed) {
      rk_raw = kpGetKey(kpg_right.pcf);
      if (rk_raw > 15) rk_pressed = false;   // ghost read -- treat as none
    }
    if (rk_pressed) {
      char kc = kpg_right.keymap[rk_raw];
      if ((int8_t)rk_raw != rk_down) {       // new key down
        rk_down       = (int8_t)rk_raw;
        rk_down_since = currentMillis;
        rk_hold_fired = false;
        rk_consumed   = false;
        last_key_char = kc;
        if (face_prompt_key) {               // this press answers the prompt
          rk_consumed = true;
          if (kc == '*') faceSaveConfirmed();
          else           facePromptCancel("CANCELLED");
        }
      } else if (!rk_hold_fired && !rk_consumed && !face_prompt_key
                 && currentMillis - rk_down_since >= FACE_HOLD_MS) {
        rk_hold_fired = true;
        if (kc != '*') facePromptOpen(kc);   // '*' is confirm-only, never a slot
      }
    } else if (rk_down >= 0) {               // key released
      if (!rk_hold_fired && !rk_consumed && !face_prompt_key
          && kpg_right.keymap[rk_down] != '*')
        faceRecall(kpg_right.keymap[rk_down]);
      rk_down = -1;
    }
  }

  // --- FACE PRESET BACKGROUND WORK (all timer-driven, never blocking) ---
  processFacePackets();

  if (face_prompt_key && currentMillis - face_prompt_started >= FACE_PROMPT_TIMEOUT_MS)
    facePromptCancel("TIMED OUT");

  if (faceMsg_clearAt && currentMillis >= faceMsg_clearAt) {
    faceMsg_clearAt = 0;
    Serial2.print("X:\n");
  }

  // Re-request the saved-face dump until a complete one lands. Fires only
  // while the whole talk chain is up; a robot with no talk board simply
  // never syncs and everything else keeps working.
  if (currentMillis - faceReq_previousMillis >= faceReq_interval) {
    faceReq_previousMillis = currentMillis;
    if (!faces_synced && talkLinkUp()) {
      faceDumpCount = 0;
      faceSendPkt(FACE_OP_REQ, NULL);
    }
  }

  // Push one dirty face per tick toward the SD; FACE_OP_ACK clears the flag.
  // Saves made while talk was offline sync themselves this way.
  if (currentMillis - faceDirty_previousMillis >= faceDirty_interval) {
    faceDirty_previousMillis = currentMillis;
    if (talkLinkUp()) {
      for (uint8_t i = 0; i < FACE_SLOTS; i++) {
        if (faces[i].valid && faces[i].dirty && !faces[i].sd_failed) {
          faceSendPkt(FACE_OP_SAVE, &faces[i]);
          break;
        }
      }
    }
  }
  // --- END FACE PRESET BACKGROUND WORK ---

}


// --- ESP-NOW RELATED ---
// Runs in the WiFi task: flag stores only. No String/heap work and no
// peripheral writes in here -- both raced loop() and corrupted the heap.
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  connectError = (status != ESP_NOW_SEND_SUCCESS);
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  // Only accept packets from our receiver -- at a crowded event another
  // ESP-NOW project's broadcast could otherwise be mistaken for ours.
  if (memcmp(mac, broadcastAddress, 6) != 0) return;
  if (len < 1) return;

  uint8_t pkt_type = incomingData[0];

  if (pkt_type == ESPNOW_PKT_FILELIST && len == sizeof(espnow_filelist_chunk_t)) {
    const espnow_filelist_chunk_t *chunk = (const espnow_filelist_chunk_t *)incomingData;

    if (chunk->chunk_index == 0) {
      // First chunk - reset and initialise the accumulator
      memset(&jukeboxPkt, 0, sizeof(jukeboxPkt));
      jukeboxPkt.magic[0] = 0xBE;
      jukeboxPkt.magic[1] = 0xCD;
      jukeboxPkt.total_files = 0;
      chunksReceived = 0;
      jukeboxReady   = false;
    }

    chunksExpected = chunk->total_chunks;

    for (uint8_t i = 0; i < chunk->entry_count && jukeboxPkt.total_files < MAX_FILES; i++) {
      uint8_t idx = jukeboxPkt.total_files;
      memcpy(jukeboxPkt.files[idx].id,   chunk->entries[i].id,   FILE_ID_LEN);
      memcpy(jukeboxPkt.files[idx].name, chunk->entries[i].name, FILE_NAME_MAX);
      jukeboxPkt.total_files++;
    }

    chunksReceived++;

    if (chunksReceived >= chunksExpected) {
      jukeboxReady = true;
      filelistForwardPending = true;   // forwarded from loop(), not this WiFi-task context
    }

  } else if (pkt_type == ESPNOW_PKT_STATUS && len == sizeof(struct_message_rcv)) {
    memcpy(&rcvData, incomingData, sizeof(rcvData));
    lastStatusRecvMs = millis();

  } else if (pkt_type == ESPNOW_PKT_FACE && len == sizeof(espnow_face_pkt_t)) {
    // memcpy-into-ring only (WiFi task context); loop() drains via
    // processFacePackets(). A full ring drops the packet -- the REQ/dirty
    // timers make every face exchange retryable, so nothing is lost for good.
    uint8_t next = (uint8_t)((faceRxHead + 1) % FACE_RXQ);
    if (next != faceRxTail) {
      memcpy((void *)&faceRxQ[faceRxHead], incomingData, sizeof(espnow_face_pkt_t));
      faceRxHead = next;
    }
  }
}
// --- END ESP-NOW RELATED ---


// --- XIAO LINK ---
void sendToXIAO() {
  disp_pkt_t pkt;
  pkt.magic[0]   = 0xAB;
  pkt.magic[1]   = 0xCD;
  // disp_pkt_t is kept byte-for-byte compatible with j4_display, so the new
  // controls reuse existing slots: eyes<-iris, spot<-eye_pop, left_arm<-eyes_x,
  // right_arm<-eyes_y.
  pkt.volume     = (uint8_t)volume_value;
  pkt.eyes       = (uint8_t)iris_value;
  pkt.spot       = (uint8_t)map(eye_pop_value, 0, 3200, 0, 255);
  pkt.left_arm   = (uint8_t)eyes_x_value;
  pkt.right_arm  = (uint8_t)eyes_y_value;
  pkt.neck       = (uint8_t)map(neck_left_value,  0, 3200, 0, 255);
  pkt.jaw        = (uint8_t)map(neck_right_value, 0, 3200, 0, 255);
  pkt.bat1_mv    = bat1_mv;
  pkt.bat2_raw   = rcvData.battery_02_voltage_rcv;
  pkt.bat3_raw   = rcvData.battery_03_voltage_rcv;
  pkt.connect_ok = connectError ? 0 : 1;
  strncpy(pkt.phrase, rcvData.phrase_playing_rcv, sizeof(pkt.phrase) - 1);
  pkt.phrase[sizeof(pkt.phrase) - 1] = '\0';

  // XOR checksum over all bytes except the final checksum byte
  uint8_t cs = 0;
  const uint8_t *p = (const uint8_t *)&pkt;
  for (size_t i = 0; i < sizeof(pkt) - 1; i++) cs ^= p[i];
  pkt.checksum = cs;

  Serial1.write((const uint8_t *)&pkt, sizeof(pkt));
}
// --- END XIAO LINK ---


void sendFileListToXIAO() {
  uint8_t cs = 0;
  const uint8_t *p = (const uint8_t *)&jukeboxPkt;
  for (size_t i = 0; i < sizeof(jukeboxPkt) - 1; i++) cs ^= p[i];
  jukeboxPkt.checksum = cs;
  Serial1.write((const uint8_t *)&jukeboxPkt, sizeof(jukeboxPkt));
}
// --- END XIAO LINK ---


void labelsDisplaySprite() {
  screen_bottom_sprite_203.fillRect(0, 20, 70, 160, TFT_BLACK);
  screen_bottom_sprite_203.setTextColor(TFT_GREEN);
  screen_bottom_sprite_203.drawString("Playing: ", 0,  20, 2);
  screen_bottom_sprite_203.drawString("VOL: ",     0,  40, 2);
  screen_bottom_sprite_203.drawString("IRIS: ",    0,  60, 2);
  screen_bottom_sprite_203.drawString("EYE-P: ",   0,  80, 2);
  screen_bottom_sprite_203.drawString("EYE-X: ",   0, 100, 2);
  screen_bottom_sprite_203.drawString("EYE-Y: ",   0, 120, 2);
  screen_bottom_sprite_203.drawString("NK-L: ",    0, 140, 2);
  screen_bottom_sprite_203.drawString("NK-R: ",    0, 160, 2);
}


// Combined system status: OFFLINE if the receiver link is silent, otherwise
// ONLINE, or whatever fault the stepper chain reported (e.g. "NL OT").
String buildStatusLine() {
  if (millis() - lastStatusRecvMs > STATUS_LINK_TIMEOUT_MS) return "OFFLINE";
  String s = rcvData.stepper_status_rcv;
  if (s.length() == 0 || s == "OK") return "ONLINE";
  return s;
}


void dataDisplaySprite() {
  screen_bottom_sprite_203.setTextColor(TFT_GREEN);
  screen_bottom_sprite_203.fillRect(70, 40, 65, 140, TFT_BLACK);

  if (!ready_message) {
    screen_bottom_sprite_203.drawString("Keypress: ",        0,  0, 2);
    screen_bottom_sprite_203.drawString(String(last_key_char), 70, 0, 2);
  }

  screen_bottom_sprite_203.drawString(String(volume_value),    70,  40, 2);
  screen_bottom_sprite_203.drawString(String(iris_value),      70,  60, 2);
  screen_bottom_sprite_203.drawString(String(eye_pop_value),   70,  80, 2);
  screen_bottom_sprite_203.drawString(String(eyes_x_value),    70, 100, 2);
  screen_bottom_sprite_203.drawString(String(eyes_y_value),    70, 120, 2);
  screen_bottom_sprite_203.drawString(String(neck_left_value),  70, 140, 2);
  screen_bottom_sprite_203.drawString(String(neck_right_value), 70, 160, 2);

  // System status line (green when healthy, red on any fault / link loss)
  String st = buildStatusLine();
  bool ok = (st == "ONLINE");
  screen_bottom_sprite_203.fillRect(0, 182, 135, 18, TFT_BLACK);
  screen_bottom_sprite_203.setTextColor(ok ? TFT_GREEN : TFT_RED);
  screen_bottom_sprite_203.drawString("STATUS: " + st, 0, 182, 1);
  screen_bottom_sprite_203.setTextColor(TFT_GREEN);
}


void tftDisplayUpdate() {
  if (screen_mode == 0) {
    dataDisplaySprite();
    screen_bottom_sprite_203.pushSprite(0, 38);
  } else if (screen_mode == 1) {
    macAddressDisplay();
  } else if (screen_mode == 2) {
    connectionDisplay();
  } else {
    adsPotsDisplay(screen_mode - 3);   // 3-6: ADS_01 .. ADS_04
  }
}


// Cycle screens with the TTGO's built-in GPIO 35 button:
// data -> MAC -> status -> ADS_01 pots -> ADS_02 pots -> ADS_03 pots -> ADS_04 pots.
void controllerScreenModeDetect() {
  if (millis() - screen_button_previousMillis >= screen_debounce_ms) {
    bool cur = digitalRead(SCREEN_BUTTON);
    if (screen_button_prev == HIGH && cur == LOW) {
      screen_mode = (screen_mode + 1) % NUM_SCREENS;
      if (screen_mode == 0) {
        tft.fillScreen(TFT_BLACK);
        tft.pushImage(0, 0, 135, 37, kevco_labs_logo_02);
        labelsDisplaySprite();
      } else if (screen_mode == 1) {
        macAddressDisplay();
      } else {
        tft.fillScreen(TFT_BLACK);   // clear once; connectionDisplay()/adsPotsDisplay() repaint values
      }
      screen_button_previousMillis = millis();
    }
    screen_button_prev = cur;
  }
}


void macAddressDisplay() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawString("WiFi MAC:", 10, 60, 2);
  tft.drawString(WiFi.macAddress(), 10, 90, 2);
}


// One "<name>: CONNECTED/DISCONNECTED" row (green/red), label and state stacked.
void drawConnLine(const char *name, bool ok, int row) {
  int y = 10 + row * 40;
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(name, 6, y, 2);
  tft.setTextColor(ok ? TFT_GREEN : TFT_RED, TFT_BLACK);
  tft.drawString(ok ? "CONNECTED   " : "DISCONNECTED", 6, y + 17, 2);
}


// Connection-status screen: how this controller sees each link right now.
void connectionDisplay() {
  unsigned long now = millis();
  bool espnow  = (now - lastStatusRecvMs) < STATUS_LINK_TIMEOUT_MS;   // receiver link
  bool dispLOk = (now - lastDisplayMs)    < DISPLAY_TIMEOUT_MS;
  bool dispROk = (now - lastDisplayRMs)   < DISPLAY_TIMEOUT_MS;
  bool neckOk  = espnow && rcvData.stepper_ok_rcv;                    // relayed by receiver
  bool eyesOk  = espnow && rcvData.eyes_ok_rcv;
  bool talkOk  = espnow && rcvData.talk_ok_rcv;
  drawConnLine("ESP-NOW LINK",     espnow,  0);
  drawConnLine("j4_stepper_neck",  neckOk,  1);
  drawConnLine("j4_stepper_eyes",  eyesOk,  2);
  drawConnLine("j4_talk",          talkOk,  3);
  drawConnLine("j4_display_left",  dispLOk, 4);
  drawConnLine("j4_display_right", dispROk, 5);
}


// Live raw counts off one ADS1115 module (screen_mode 3-6), for bench
// testing without needing a laptop on the I2C bus. Module presence reuses
// the same adsReady() guard the control-tx loop already probes with.
void adsPotsDisplay(int idx) {
  AdsPotScreen &s = adsPotScreens[idx];
  bool ok = adsReady(s.guard);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(s.title, 6, 10, 2);
  tft.setTextColor(ok ? TFT_GREEN : TFT_RED, TFT_BLACK);
  tft.drawString(ok ? "CONNECTED   " : "DISCONNECTED", 6, 30, 2);

  for (int i = 0; i < 4; i++) {
    int y = 60 + i * 40;
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString(s.labels[i], 0, y, 2);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.fillRect(70, y, 65, 20, TFT_BLACK);
    tft.drawString(s.values[i] ? String(*s.values[i]) : "--", 70, y, 2);
  }
}

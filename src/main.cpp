//******************************************************************************
//       file name:  j4_controller_v0_6_4.ino
//     v0_1 created:  2023-11-08 -- 1209 CST
//     v0_6 created:  2023-11-16 -- 2226 CST
//   v0_6_4 created:  2026-05-20 -- 0700 CDT
//     last updated:  2026-05-22 -- CDT
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
//       17:  XIAO LINK TX  →  XIAO D7 / GPIO44   (output only; unreliable as input per README)
//       18:  TFT SCLK             [USED BY TTGO]
//       19:  TFT MOSI             [USED BY TTGO]
//
//       21:  SDA  [I2C BUS] (ADS_01 0x48, ADS_02 0x49, keypad 0x20)
//       22:  SCL  [I2C BUS]
//
//       23:  TFT RST              [USED BY TTGO]
//
//       27:  XIAO LINK RX  ←  XIAO D6 / GPIO43
//
//       34:  battery voltage sense
//
//       35:  button 2             [USED BY TTGO]
//      ------------------------------------------------------------------
//      ------------------------------------------------------------------
//
//      XIAO LINK WIRING  (3.3V logic on both sides - no level shifter needed)
//      ------------------------------------------------------------------
//      TTGO GPIO17  →  XIAO D7 (GPIO44)    TTGO TX → XIAO RX
//      TTGO GPIO27  ←  XIAO D6 (GPIO43)    TTGO RX ← XIAO TX
//      TTGO GND     -  XIAO GND
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
#include <WiFi.h>
#include "kevco_labs_logo_02.h" // 135 x 37 pixels

// ----------  FUNCTION PROTOTYPES ----------
void OnDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len);
void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status);
void labelsDisplaySprite();
void dataDisplaySprite();
void tftDisplayUpdate();
void sendToXIAO();
// ------------------------------------------


#define SDA 21
#define SCL 22

// --- XIAO LINK ---
#define XIAO_TX_PIN  17   // GPIO17 output → XIAO D7 (GPIO44)
#define XIAO_RX_PIN  27   // GPIO27 input  ← XIAO D6 (GPIO43)
#define XIAO_BAUD    115200

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
// --- END XIAO LINK ---


ADS1115 ADS_01(0x48);  // ADDRESS PIN TO GND
ADS1115 ADS_02(0x49);  // ADDRESS PIN TO VDD


// --- ESP-NOW RELATED ---
uint8_t broadcastAddress[] = { 0xA0, 0xDD, 0x6C, 0x74, 0xDA, 0x74 };  //MAY 2026 TTGO 2026-05-01--1238-KL

typedef struct struct_message_rcv {
  String phrase_playing_rcv;
  int battery_02_voltage_rcv;
  int battery_03_voltage_rcv;
} struct_message_rcv;

struct_message_rcv rcvData;

typedef struct struct_message_xmit {
  String phrase_select_xmit;
  int volume_xmit;
  int eyes_xmit;
  int spot_xmit;
  int left_arm_xmit;
  int right_arm_xmit;
  int neck_xmit;
  int jaw_xmit;
} struct_message_xmit;

struct_message_xmit xmitData;
esp_now_peer_info_t peerInfo;

volatile bool connectError = LOW;
String connectStatus = "NO INFO";
// --- END ESP-NOW RELATED ---


// Potentiometer values
int volume_value    = 0;
int eyes_value      = 0;
int spot_value      = 0;
int left_arm_value  = 0;
int right_arm_value = 0;
int neck_value      = 0;
int jaw_value       = 0;

// Controller battery (millivolts), updated by battery timer, read by sendToXIAO()
uint16_t bat1_mv = 0;


// Timers
unsigned long tft_update_previousMillis = 0;
unsigned long battery_01_previousMillis = 0;
unsigned long keypad_previousMillis     = 0;
const unsigned long tft_update_interval = 40;   // 25 fps
const unsigned long battery_01_interval = 500;
const unsigned long keypad_interval     = 150;


PCF8574 pcf8574(0x20);
TFT_eSPI tft = TFT_eSPI();
TFT_eSprite screen_bottom_sprite_203 = TFT_eSprite(&tft);

// Keypad wiring: keypad pin 1 plugs into P0 straight through.
// Actual pin→signal: P0=Row3, P1=Row2, P2=Row1, P3=Col4, P4=Col3, P5=Col2, P6=Col1, P7=Row4
// (P3 and P7 are swapped vs the I2CKeyPad library's expectation, so we scan manually.)
// Keymap indexed by (drive_index*4 + read_index), drive order: P0,P1,P2,P7; read order: P3,P4,P5,P6
char keymap[19] = "#9630852*741DCBANF";  // N = NoKey, F = Fail
int key      = -2;
int old_key  = -1;
String phrase_select_buffer = "";
bool ready_message = true;


// Drive row pins LOW one at a time, check if any col pin reads LOW.
// Drive order: P0,P1,P2,P7 (= Row3,Row2,Row1,Row4)
// Read  order: P3,P4,P5,P6 (= Col4,Col3,Col2,Col1)
static const uint8_t KP_DRIVE_WRITE[4] = { 0xFE, 0xFD, 0xFB, 0x7F }; // ~(1<<P) for P=0,1,2,7
static const uint8_t KP_READ_MASK      = 0x78;  // bits 3,4,5,6 = P3,P4,P5,P6

bool kpIsPressed() {
  pcf8574.write8(0x78);  // all row pins LOW, all col pins HIGH-Z
  return (pcf8574.read8() & KP_READ_MASK) != KP_READ_MASK;
}

uint8_t kpGetKey() {
  static const uint8_t READ_BIT[4] = { 3, 4, 5, 6 };  // P3,P4,P5,P6
  for (uint8_t d = 0; d < 4; d++) {
    pcf8574.write8(KP_DRIVE_WRITE[d]);
    uint8_t val = pcf8574.read8();
    for (uint8_t r = 0; r < 4; r++) {
      if (!(val & (1 << READ_BIT[r]))) {
        pcf8574.write8(0xFF);
        return d * 4 + r;
      }
    }
  }
  pcf8574.write8(0xFF);
  return 16;  // NoKey
}

// raw <= 100: noise floor; raw >= 65000: ADC overflow; 17000: pot physical max
int processPot(int raw, int out_max) {
  if (raw <= 100 || raw >= 65000) return 0;
  if (raw > 17000) raw = 17000;
  return map(raw, 0, 17000, 0, out_max);
}


void setup() {
  Wire.begin(SDA, SCL);
  pcf8574.begin();
  Serial.begin(115200);

  ADS_01.begin();
  delay(10);
  ADS_01.setGain(0);      //  0 is ±6.144V    1 is ±4.096V    2 is ±2.048V
  ADS_01.setDataRate(7);  //  0 = slow   4 = medium   7 = fast
  ADS_01.setMode(1);      //  0 = continuous mode   1 = single mode
  ADS_01.requestADC(0);   //  first read to trigger

  ADS_02.begin();
  delay(10);
  ADS_02.setGain(0);
  ADS_02.setDataRate(7);
  ADS_02.setMode(1);
  ADS_02.requestADC(0);

  // XIAO display link - init after ADS1115 to avoid I2C/UART peripheral conflict
  Serial1.begin(XIAO_BAUD, SERIAL_8N1, XIAO_RX_PIN, XIAO_TX_PIN);

  WiFi.mode(WIFI_STA);
  Serial.println("MAC Address: ");
  Serial.println(WiFi.macAddress());
  WiFi.setSleep(false);

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

  tft.init();
  tft.setRotation(2);
  tft.fillScreen(TFT_BLACK);
  tft.setSwapBytes(true);

  screen_bottom_sprite_203.createSprite(135, 203);
  tft.pushImage(0, 0, 135, 37, kevco_labs_logo_02);
  screen_bottom_sprite_203.setTextColor(TFT_GREEN, TFT_BLACK);
  labelsDisplaySprite();
  screen_bottom_sprite_203.drawString("Ready...", 0, 0, 1);

}


void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - tft_update_previousMillis >= tft_update_interval) {
    tft_update_previousMillis = currentMillis;
    tftDisplayUpdate();
    sendToXIAO();
  }

  // --- ADC READS ---
  volume_value    = processPot(ADS_01.readADC(0), 100);
  eyes_value      = processPot(ADS_01.readADC(1), 255);
  spot_value      = processPot(ADS_01.readADC(2), 255);
  left_arm_value  = processPot(ADS_02.readADC(0), 255);
  right_arm_value = processPot(ADS_02.readADC(1), 255);
  neck_value      = 255 - processPot(ADS_02.readADC(2), 255);
  jaw_value       = processPot(ADS_02.readADC(3), 255);
  // --- END ADC READS ---

  xmitData.volume_xmit      = volume_value;
  xmitData.eyes_xmit        = eyes_value;
  xmitData.spot_xmit        = spot_value;
  xmitData.left_arm_xmit    = left_arm_value;
  xmitData.right_arm_xmit   = right_arm_value;
  xmitData.neck_xmit        = neck_value;
  xmitData.jaw_xmit         = jaw_value;

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

    if (kpIsPressed()) {
      uint8_t rawKey = kpGetKey();

      if (rawKey <= 15) {  // 16 = NoKey - only promote to global on valid read
      key = (int)rawKey;

      if (ready_message) {
        screen_bottom_sprite_203.fillRect(0, 0, 135, 20, TFT_BLACK);
        ready_message = false;
      } else {
        screen_bottom_sprite_203.fillRect(70, 0, 65, 20, TFT_BLACK);
      }

      if (key != old_key) {
        char kc = keymap[key];

        if (kc == '#') {  // reset same-key lock and clear buffer
          old_key = -1;
          phrase_select_buffer = "";
          screen_bottom_sprite_203.fillRect(70,  0, 65, 20, TFT_BLACK);
          screen_bottom_sprite_203.fillRect(70, 20, 65, 20, TFT_BLACK);

        } else if (kc == '*') {  // stop playback
          old_key = -1;
          phrase_select_buffer = "STOP";
          xmitData.phrase_select_xmit = phrase_select_buffer;
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

            xmitData.phrase_select_xmit = phrase_select_buffer;
            phrase_select_buffer = "";
          }
        }
      }
      }  // end key <= 15 guard
    }
  }

  // --- ESP-NOW RELATED ---
  esp_now_send(broadcastAddress, (uint8_t *)&xmitData, sizeof(xmitData));
}


// --- ESP-NOW RELATED ---
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    connectStatus = "xmit success";
    connectError = LOW;
  } else {
    connectStatus = "xmit failed";
    connectError = HIGH;
  }
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&rcvData, incomingData, sizeof(rcvData));
}
// --- END ESP-NOW RELATED ---


// --- XIAO LINK ---
void sendToXIAO() {
  disp_pkt_t pkt;
  pkt.magic[0]   = 0xAB;
  pkt.magic[1]   = 0xCD;
  pkt.volume     = (uint8_t)volume_value;
  pkt.eyes       = (uint8_t)eyes_value;
  pkt.spot       = (uint8_t)spot_value;
  pkt.left_arm   = (uint8_t)left_arm_value;
  pkt.right_arm  = (uint8_t)right_arm_value;
  pkt.neck       = (uint8_t)neck_value;
  pkt.jaw        = (uint8_t)jaw_value;
  pkt.bat1_mv    = bat1_mv;
  pkt.bat2_raw   = (int16_t)rcvData.battery_02_voltage_rcv;
  pkt.bat3_raw   = (int16_t)rcvData.battery_03_voltage_rcv;
  pkt.connect_ok = connectError ? 0 : 1;
  strncpy(pkt.phrase, rcvData.phrase_playing_rcv.c_str(), sizeof(pkt.phrase) - 1);
  pkt.phrase[sizeof(pkt.phrase) - 1] = '\0';

  // XOR checksum over all bytes except the final checksum byte
  uint8_t cs = 0;
  const uint8_t *p = (const uint8_t *)&pkt;
  for (size_t i = 0; i < sizeof(pkt) - 1; i++) cs ^= p[i];
  pkt.checksum = cs;

  Serial1.write((const uint8_t *)&pkt, sizeof(pkt));
}
// --- END XIAO LINK ---


void labelsDisplaySprite() {
  screen_bottom_sprite_203.fillRect(0, 20, 70, 160, TFT_BLACK);
  screen_bottom_sprite_203.setTextColor(TFT_GREEN);
  screen_bottom_sprite_203.drawString("Playing: ", 0,  20, 2);
  screen_bottom_sprite_203.drawString("VOL: ",     0,  40, 2);
  screen_bottom_sprite_203.drawString("EYES: ",    0,  60, 2);
  screen_bottom_sprite_203.drawString("SPOT: ",    0,  80, 2);
  screen_bottom_sprite_203.drawString("L-ARM: ",   0, 100, 2);
  screen_bottom_sprite_203.drawString("R-ARM: ",   0, 120, 2);
  screen_bottom_sprite_203.drawString("NECK: ",    0, 140, 2);
  screen_bottom_sprite_203.drawString("JAW: ",     0, 160, 2);
}


void dataDisplaySprite() {
  screen_bottom_sprite_203.setTextColor(TFT_GREEN);
  screen_bottom_sprite_203.fillRect(70, 40, 65, 140, TFT_BLACK);

  if (!ready_message) {
    screen_bottom_sprite_203.drawString("Keypress: ",        0,  0, 2);
    screen_bottom_sprite_203.drawString(String(keymap[key]), 70, 0, 2);
  }

  screen_bottom_sprite_203.drawString(String(volume_value),    70,  40, 2);
  screen_bottom_sprite_203.drawString(String(eyes_value),      70,  60, 2);
  screen_bottom_sprite_203.drawString(String(spot_value),      70,  80, 2);
  screen_bottom_sprite_203.drawString(String(left_arm_value),  70, 100, 2);
  screen_bottom_sprite_203.drawString(String(right_arm_value), 70, 120, 2);
  screen_bottom_sprite_203.drawString(String(neck_value),      70, 140, 2);
  screen_bottom_sprite_203.drawString(String(jaw_value),       70, 160, 2);
}


void tftDisplayUpdate() {
  dataDisplaySprite();
  screen_bottom_sprite_203.pushSprite(0, 38);
}

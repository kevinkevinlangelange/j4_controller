//******************************************************************************
//       file name:  j4_controller_v0_6_4.ino
//     v0_1 created:  2023-11-08 -- 1209 CST
//     v0_6 created:  2023-11-16 -- 2226 CST
//   v0_6_4 created:  2026-05-20 -- 0700 CDT
//     last updated:  2026-05-20 -- 0700 CDT
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
//       18:  TFT SCLK             [USED BY TTGO]
//       19:  TFT MOSI             [USED BY TTGO]
//
//       21:  SDA  [I2C BUS] (ADS_01 0x48, ADS_02 0x49, keypad 0x20)  
//       22:  SCL  [I2C BUS]
//
//       23:  TFT RST              [USED BY TTGO]
//
//       34:  battery voltage sense
//
//       35:  button 2             [USED BY TTGO]
//       ------------------------------------------------------------------
//       ------------------------------------------------------------------
//
//
//
//******************************************************************************


#include <Wire.h>
#include <PCF8574.h>
#include <I2CKeyPad.h>
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
// ------------------------------------------


#define SDA 21
#define SCL 22

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


// Timers
unsigned long tft_update_previousMillis = 0;
unsigned long battery_01_previousMillis = 0;
unsigned long keypad_previousMillis     = 0;
const unsigned long tft_update_interval = 40;   // 25 fps
const unsigned long battery_01_interval = 500;
const unsigned long keypad_interval     = 150;


PCF8574 pcf8574(0x20);
I2CKeyPad keyPad(0x20);
TFT_eSPI tft = TFT_eSPI();
TFT_eSprite screen_bottom_sprite_203 = TFT_eSprite(&tft);

char keymap[19] = "123A456B789C*0#DNF";  // N = NoKey, F = Fail
int key      = -2;
int old_key  = -1;
String phrase_select_buffer = "";
bool ready_message = true;


// raw <= 100: noise floor; raw >= 65000: ADC overflow; 17000: pot physical max
int processPot(int raw, int out_max) {
  if (raw <= 100 || raw >= 65000) return 0;
  if (raw > 17000) raw = 17000;
  return map(raw, 0, 17000, 0, out_max);
}


void setup() {
  Wire.begin(SDA, SCL);
  pcf8574.begin();
  keyPad.begin();
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

  keyPad.loadKeyMap(keymap);
}


void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - tft_update_previousMillis >= tft_update_interval) {
    tft_update_previousMillis = currentMillis;
    tftDisplayUpdate();
  }

  // --- ADC READS ---
  volume_value    = processPot(ADS_01.readADC(0), 100);
  eyes_value      = processPot(ADS_01.readADC(1), 255);
  spot_value      = processPot(ADS_01.readADC(2), 255);
  left_arm_value  = processPot(ADS_02.readADC(0), 255);
  right_arm_value = processPot(ADS_02.readADC(1), 255);
  neck_value      = processPot(ADS_02.readADC(2), 255);
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

    if (keyPad.isPressed()) {
      key = keyPad.getLastKey();

      if (ready_message) {
        screen_bottom_sprite_203.fillRect(0, 0, 135, 20, TFT_BLACK);
        ready_message = false;
      } else {
        screen_bottom_sprite_203.fillRect(70, 0, 65, 20, TFT_BLACK);
      }

      if (key != old_key) {
        if (key == 14) {  // "#" — reset same-key lock and clear buffer
          old_key = -1;
          phrase_select_buffer = "";
          screen_bottom_sprite_203.fillRect(70,  0, 65, 20, TFT_BLACK);
          screen_bottom_sprite_203.fillRect(70, 20, 65, 20, TFT_BLACK);
        } else if (key == 12) {  // "*" — stop playback
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

          // Letter prefix (A/B/C/D) — wait for digit
          if (key == 3 || key == 7 || key == 11 || key == 15) {
            switch (key) {
              case  3: phrase_select_buffer = "A"; break;
              case  7: phrase_select_buffer = "B"; break;
              case 11: phrase_select_buffer = "C"; break;
              case 15: phrase_select_buffer = "D"; break;
            }
          } else {
            if (phrase_select_buffer.length() == 0) phrase_select_buffer = "0";
            phrase_select_buffer += String(keymap[key]);

            screen_bottom_sprite_203.setTextColor(TFT_GREEN);
            screen_bottom_sprite_203.fillRect(70, 20, 65, 20, TFT_BLACK);
            screen_bottom_sprite_203.drawString(phrase_select_buffer + ".wav", 70, 20, 2);

            xmitData.phrase_select_xmit = phrase_select_buffer;
            phrase_select_buffer = "";
          }
        }
      }
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

  if (key != 14 && !ready_message) {
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

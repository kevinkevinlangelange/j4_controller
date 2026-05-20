//******************************************************************************
//       file name:  j4_controller_v0_6_3.ino
//     v0_1 created:  2023-11-08 -- 1209 CST
//     v0_6 created:  2023-11-16 -- 2226 CST
//     v0_b created:  2025-05-19 -- 0025 CST
//     last updated:  2025-05-19 -- 0025 CST
//           author:  Kevin Lange
//      description:  Main code for Johnny 4 controller/transmitter
//                    running on a LILYGO TTGO T-Display v1.1 ESP32 board
//       update log:  v0_3 -- Changed potentiometer inputs to GPIOs that
//                            run on ADC1, as ADC2 will be dedicated to
//                            WiFi/ESP-NOW functionality in future iterations
//                    v0_4 -- Implemented ESP-NOW functionality for the first time
//                    v0_5 -- Tried and failed to display ESP-NOW status info on TFT display.
//                            Added ADC functionality w/ two ADS1115 modules.
//                    v0_6 -- Converting to sprite display for efficiency
//                   v0_6b -- Migrated from Arduino IDE to Platform.io
//                          - Fix keypad matrix (it had gotten wired half backwards)
//
//******************************************************************************



#include <Wire.h>
#include <PCF8574.h>
#include <I2CKeyPad.h>
#include <TFT_eSPI.h>
#include <ADS1X15.h>
#include <SPI.h>
#include <Arduino.h>
#include <esp_now.h>             // For ESP-NOW
#include <WiFi.h>                // Also for ESP-NOW
#include <esp_adc_cal.h>         // --- BATTERY RELATED ---
#include "kevco_labs_logo_02.h"  //135x37

// ----------  FUNCTION PROTOTYPES ----------
void OnDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len);
void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status);

void labelsDisplaySprite();
void dataDisplaySprite();
void tftDisplayUpdate();
// ------------------------------------------


// Define the pin connections
#define SDA 21
#define SCL 22

ADS1115 ADS_01(0x48);  // I2C address of the first ADS1115  //ADDRESS PIN TO GND
ADS1115 ADS_02(0x49);  // I2C address of the second ADS1115 //ADDRESS PIN TO VDD
//PCF8574 pcf(0x27); // I2C address of the PCF8574

// assign the potentiometers (GPIOs 2, 13, 15, 25, 26, 27, 32, 33, 36, 37, 38, & 39 have been tested and work for this)
// (NOTE - GPIO pin 12 "works", but if it's connected to a potentiometer, it causes the ESP32 not to be able to flash/boot)
// (NOTE - These GPIO pins do NOT seem to work:  17)

// PINS CURRENTLY IN USE:    32, 33, 36, 37, 386
// PINS PREVIOUSLY IN USE:    15, 21, 22, 25, 26, 27, 32, 33, 36
// For UART, try 17 as TX and 37 as RX (May need to alter Arduino IDE config something or other)(I've read that 36, 37. & 38 may only be used as inputs.) -KL 2023-11-04-0433a
// PINS "32-39" (which on this board are 32, 33, 36, 37, 38) use ADC1. ADC2 will be tied up by WiFi in future iterations.

// Initialize Variables
int volume_value = 0;
int eyes_value = 0;
int spot_value = 0;
int unused_value = 0;
int left_arm_value = 0;
int right_arm_value = 0;
int neck_value = 0;
int jaw_value = 0;


//    -----ESP-NOW RELATED
// MAC Address of receiver - edit as required [OG: 0xB0, 0xB2, 0x1C, 0x4F, 0x16, 0xC4]
uint8_t broadcastAddress[] = { 0xA0, 0xDD, 0x6C, 0x74, 0xDA, 0x74 }; //MAY 2026 TTGO 2026-05-01--1238-KL


typedef struct struct_message_rcv {
  String phrase_playing_rcv;
  int battery_02_voltage_rcv;
  int battery_03_voltage_rcv;
} struct_message_rcv;

// Create a structured object for received data
struct_message_rcv rcvData;


// Create a structured object for sent data
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

// Create a structured object for sent data
struct_message_xmit xmitData;

// ESP-NOW Peer info
esp_now_peer_info_t peerInfo;


//-----END ESP-NOW RELATED

// timer for refreshing tft display 25 times per second
unsigned long tft_update_previousMillis = 0;   // Store the last time the timer was updated
const unsigned long tft_update_interval = 40;  // Interval in milliseconds (40ms) (25fps) (1000 / 25 = 40)

// timer for battery_01 refresh (to calm it down from being so spazzy)
unsigned long battery_01_previousMillis = 0;    // Store the last time the timer was updated
const unsigned long battery_01_interval = 500;  // Interval in milliseconds (500ms)

// timer for keypad debouncing
unsigned long keypad_previousMillis = 0;    // Store the last time the timer was updated
const unsigned long keypad_interval = 150;  // Interval in milliseconds (150ms)

// IO34 seems to be the battery voltage (haven't confirmed yet)

PCF8574 pcf8574(0x20);
I2CKeyPad keyPad(0x20);
TFT_eSPI tft = TFT_eSPI();  // Create an instance of the TFT_eSPI library

// Create sprites
TFT_eSprite screen_sprite_01 = TFT_eSprite(&tft);
TFT_eSprite screen_bottom_sprite_203 = TFT_eSprite(&tft);
//char keymap[19] = "147*2580369#ABCDNF"; // N = NoKey, F = Fail (UPDATED 2025-05-18--0042 -KL)
char keymap[19] = "123A456B789C*0#DNF";  // N = NoKey, F = Fail
//char keymap[19] = "1234567890ABCD*#NF";  // N = NoKey, F = Fail
//char keymap[19] = "N123A456B789C*0#DF";  // N = NoKey, F = Fail
int key = -2;
int old_key = -1;
String phrase_select_buffer = "";

// --- ESP-NOW RELATED ---
// Variable for connection error  - HIGH is error state
volatile bool connectError = LOW;

// Variable for connection status string
String connectStatus = "NO INFO";
String old_connectStatus = "OLD NO INFO";
// --- END ESP-NOW RELATED ---

// Variables to keep track of the current ADC channels
uint8_t ADS_01_currentChannel = 0;
uint8_t ADS_02_currentChannel = 0;


bool ready_message = true;


void setup() {
  Wire.begin(SDA, SCL);
  pcf8574.begin();
  keyPad.begin();
  Serial.begin(115200);

  //ads.setGain(ADS1115_GAIN_FULL_SCALE); // Set gain to full scale
  //ads.setConversionMode(ADS1115_CONTINUOUS_CONVERSION); // Set conversion mode to continuous conversion
  //ads.begin(); // Initialize the ADS1115
  ADS_01.begin();
  delay(10);
  ADS_01.setGain(0);                         //  0 is ±6.144V    1 is ±4.096V    2 is ±2.048V
  ADS_01.setDataRate(7);                     //  0 = slow   4 = medium   7 = fast
  ADS_01.setMode(1);                         //  0 = continuous mode   1 = single mode
  ADS_01.requestADC(ADS_01_currentChannel);  //  first read to trigger

  ADS_02.begin();
  delay(10);
  ADS_02.setGain(0);                         //  0 is ±6.144V    1 is ±4.096V    2 is ±2.048V
  ADS_02.setDataRate(7);                     //  0 = slow   4 = medium   7 = fast
  ADS_02.setMode(1);                         //  0 = continuous mode   1 = single mode
  ADS_02.requestADC(ADS_02_currentChannel);  //  first read to trigger

  //
  // --- ESP-NOW RELATED ---
  // Set ESP32 as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  Serial.println("MAC Address: ");
  Serial.println(WiFi.macAddress());  // Print the MAC address

  // Disable WiFi Sleep mode
  WiFi.setSleep(false);

  // Initilize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    connectStatus = "init error";
    connectError = HIGH;
    return;
  } else {
    connectStatus = "init OK";
    connectError = LOW;
  }

  // Register receive callback function
  esp_now_register_recv_cb(OnDataRecv);

  // Register the send callback
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    connectStatus = "no peer added";
    connectError = HIGH;
    return;
  } else {
    connectStatus = "peer added";
    connectError = LOW;
  }
  // --- END ESP-NOW RELATED ---


  // Set the potentiometer pins as an input
  //pinMode(volume_pot, INPUT);
  //pinMode(eyes_pot, INPUT);

  tft.init();                 // Initialize the display
  tft.setRotation(2);         // Rotate the display 180 degrees
  tft.fillScreen(TFT_BLACK);  // Clear the screen
  tft.setSwapBytes(true);     //needed for image display for some reason

  // Actually create sprite
  screen_bottom_sprite_203.createSprite(135, 203);


  tft.pushImage(0, 0, 135, 37, kevco_labs_logo_02);  //135x37
  screen_bottom_sprite_203.setTextColor(TFT_GREEN, TFT_BLACK);
  labelsDisplaySprite();
  screen_bottom_sprite_203.drawString("Ready...", 0, 0, 1);

  //tft.println("Ready...");

  keyPad.loadKeyMap(keymap);
}


void loop() {
  // USED BY ALL TIMERS
  unsigned long currentMillis = millis();  // Get the current time

  // Something is happening with the potentiometer where it will hop from 0 straight to 40 near the bottom.
  // Will probably need to come up with a work-around/safety thing so the steppers don't go nuts if this
  // happens with the "arms" controllers.


  // Display the live controller data
  //tft.pushImage(0, 0, 135, 37, kevco_labs_logo_02);  //135x37

  //tft.setTextColor(TFT_GREEN);
  //tft.drawString("Playing: ", 0, 60, 2);

  // Check if it's time to perform an action
  if (currentMillis - tft_update_previousMillis >= tft_update_interval) {
    // It's time to perform your action
    tft_update_previousMillis = currentMillis;  // Save the last time the action was performed
    // Updates the tft display (This function automatically detects the current screen mode)
    tftDisplayUpdate();
  }

  //******* ADC RELATED *******
  volume_value = ADS_01.readADC(0);
  //Serial.println(String(volume_value));
  eyes_value = ADS_01.readADC(1);
  spot_value = ADS_01.readADC(2);
  unused_value = ADS_01.readADC(3);

  left_arm_value = ADS_02.readADC(0);
  right_arm_value = ADS_02.readADC(1);
  neck_value = ADS_02.readADC(2);
  jaw_value = ADS_02.readADC(3);




  //*********************************************
  // --- VOLUME ---
  // Read the value of the potentiometer
  //uint16_t value = ads.readADC_SingleEnded(0); // Read the value from A0
  //Serial.print("A0: ");
  //Serial.println(value);
  //volume_value = analogRead(volume_pot);
  //volume_value = (int)ADS_01.readADC(0);  // Read the value from A0
  //Serial.println(String(volume_value_ads));
  //volume_value = (int)volume_value_ads;  // Converts volume_value_ads to type int (and stores it in volume_value)

  if (volume_value <= 100 || volume_value >= 65000) {
    volume_value = 0;
  }
  if (volume_value >= 17000) {
    volume_value = 17000;
  }

  volume_value = map(volume_value, 0, 17000, 0, 100);  //Remaps the values from 0-32767 to 0-100
                                                       /*
  screen_bottom_sprite_203.drawString("VOL: ", 0, 40, 2);
  screen_bottom_sprite_203.fillRect(70, 40, 24, 20, TFT_BLACK);
  screen_bottom_sprite_203.setTextColor(TFT_GREEN);
  screen_bottom_sprite_203.drawString(String(volume_value), 70, 40, 2);
*/

  // ---END VOLUME ---
  //*********************************************



  //*********************************************
  // --- EYES ---
  if (eyes_value <= 100 || eyes_value >= 65000) {
    eyes_value = 0;
  }
  if (eyes_value >= 17000) {
    eyes_value = 17000;
  }

  eyes_value = map(eyes_value, 0, 17000, 0, 255);  //Remaps the values from 0-32767 to 0-100

  /*
  screen_bottom_sprite_203.setTextColor(TFT_GREEN);
  screen_bottom_sprite_203.drawString("EYES: ", 0, 60, 2);
  screen_bottom_sprite_203.fillRect(70, 60, 24, 20, TFT_BLACK);
  screen_bottom_sprite_203.drawString(String(eyes_value), 70, 60, 2);
  */
  // ---END EYES ---
  //*********************************************



  //*********************************************
  // --- SPOT ---
  // Read the value of the potentiometer
  if (spot_value <= 100 || spot_value >= 65000) {
    spot_value = 0;
  }
  if (spot_value >= 17000) {
    spot_value = 17000;
  }

  spot_value = map(spot_value, 0, 17000, 0, 255);  //Remaps the values from 0-32767 to 0-100
  /*
  screen_bottom_sprite_203.setTextColor(TFT_GREEN);
  screen_bottom_sprite_203.drawString("SPOT: ", 0, 80, 2);
  screen_bottom_sprite_203.fillRect(70, 80, 24, 20, TFT_BLACK);
  screen_bottom_sprite_203.drawString(String(spot_value), 70, 80, 2);
  */
  // ---END SPOT ---
  //*********************************************



  //*********************************************
  // --- LEFT_ARM ---
  // Read the value of the potentiometer
  if (left_arm_value <= 100 || left_arm_value >= 65000) {
    left_arm_value = 0;
  }
  if (left_arm_value >= 17000) {
    left_arm_value = 17000;
  }

  left_arm_value = map(left_arm_value, 0, 17000, 0, 255);  //Remaps the values from 0-32767 to 0-100
                                                           /*
  screen_bottom_sprite_203.setTextColor(TFT_GREEN);
  screen_bottom_sprite_203.drawString("L-ARM: ", 0, 100, 2);
  screen_bottom_sprite_203.fillRect(70, 100, 24, 20, TFT_BLACK);
  screen_bottom_sprite_203.drawString(String(left_arm_value), 70, 100, 2);
  */
  // ---END LEFT_ARM ---
  //*********************************************



  //*********************************************
  // --- RIGHT_ARM ---
  // Read the value of the potentiometer
  if (right_arm_value <= 100 || right_arm_value >= 65000) {
    right_arm_value = 0;
  }
  if (right_arm_value >= 17000) {
    right_arm_value = 17000;
  }

  right_arm_value = map(right_arm_value, 0, 17000, 0, 255);  //Remaps the values from 0-32767 to 0-100

  /*
  screen_bottom_sprite_203.setTextColor(TFT_GREEN);
  screen_bottom_sprite_203.drawString("R-ARM: ", 0, 120, 2);
  screen_bottom_sprite_203.fillRect(70, 120, 24, 20, TFT_BLACK);
  screen_bottom_sprite_203.drawString(String(right_arm_value), 70, 120, 2);
  */
  // ---END RIGHT_ARM ---
  //*********************************************



  //*********************************************
  // --- NECK ---
  // Read the value of the potentiometer
  if (neck_value <= 100 || neck_value >= 65000) {
    neck_value = 0;
  }
  if (neck_value >= 17000) {
    neck_value = 17000;
  }

  neck_value = map(neck_value, 0, 17000, 0, 255);  //Remaps the values from 0-32767 to 0-100

  /*
  screen_bottom_sprite_203.setTextColor(TFT_GREEN);
  screen_bottom_sprite_203.drawString("NECK: ", 0, 140, 2);
  screen_bottom_sprite_203.fillRect(70, 140, 24, 20, TFT_BLACK);
  screen_bottom_sprite_203.drawString(String(neck_value), 70, 140, 2);
  */
  // ---END NECK ---
  //*********************************************



  //*********************************************
  // --- JAW ---
  // Read the value of the potentiometer
  if (jaw_value <= 100 || jaw_value >= 65000) {
    jaw_value = 0;
  }
  if (jaw_value >= 17000) {
    jaw_value = 17000;
  }

  jaw_value = map(jaw_value, 0, 17000, 0, 255);  //Remaps the values from 0-32767 to 0-100

  /*
  screen_bottom_sprite_203.setTextColor(TFT_GREEN);
  screen_bottom_sprite_203.drawString("JAW: ", 0, 160, 2);
  screen_bottom_sprite_203.fillRect(70, 160, 24, 20, TFT_BLACK);
  screen_bottom_sprite_203.drawString(String(jaw_value), 70, 160, 2);
  */
  // ---END JAW ---
  //*********************************************



  // ------- ESP-NOW RELATED
  //***********************************************************
  // --- READY ALL TRANSMIT DATA WITH ALL POTENTIOMETER VALUES TO SEND TO RECEIVER ---
  xmitData.volume_xmit = volume_value;
  xmitData.eyes_xmit = eyes_value;
  xmitData.spot_xmit = spot_value;
  xmitData.left_arm_xmit = left_arm_value;
  xmitData.right_arm_xmit = right_arm_value;
  xmitData.neck_xmit = neck_value;
  xmitData.jaw_xmit = jaw_value;
  // --- END READY ALL TRANSMIT DATA WITH ALL POTENTIOMETER VALUES TO SEND TO RECEIVER ---
  //***********************************************************
  //------- ESP-NOW RELATED
  //

  //unsigned long currentMillis = millis(); // Get the current time

  // --- BATTERY-RELATED ---

  // Check if it's time to perform an action
  if (currentMillis - battery_01_previousMillis >= battery_01_interval) {
    // It's time to perform your action
    battery_01_previousMillis = currentMillis;  // Save the last time the action was performed


    // Read the IO34 pin
    int raw = analogRead(34);  // Read the raw ADC value from IO34
    //Serial.print("Raw ADC Value: ");
    //Serial.println(raw);

    float float_value = 0.0018276;
    //int int_value = 10;

    // Multiply the int and float together
    float product = float_value * raw;

    // Convert the float to a string, with 2 decimal places
    char str[10];
    dtostrf(product, 4, 2, str);


    // Display battery voltage
    String voltage_battery_01 = str;
    voltage_battery_01 += "V";

    // PLACEHOLDER VALUES FOR BATTERY_02 AND BATTERY_03
    String voltage_battery_02 = "6.00V";
    String voltage_battery_03 = "12.00V";

    //tft.fillRect(70, 220, 65, 20, TFT_BLACK);
    //tft.drawString("Battery_01: ",0,220,1);
    //tft.drawString(voltage_battery_01,70,220,1);

    screen_bottom_sprite_203.setTextColor(TFT_BLACK);  //Set the text color to black
    screen_bottom_sprite_203.fillRect(0, 185, 40, 15, TFT_GREEN);
    screen_bottom_sprite_203.drawString(voltage_battery_01, 2, 185, 2);

    /*
    tft.fillRect(47, 225, 40, 15, TFT_GREEN);
    tft.drawString(voltage_battery_02,49,225,2);

    tft.fillRect(94, 225, 40, 15, TFT_GREEN);
    tft.drawString(voltage_battery_03,96,225,2);
    */

    screen_bottom_sprite_203.fillRect(43, 185, 40, 15, TFT_GREEN);
    screen_bottom_sprite_203.drawString(voltage_battery_02, 45, 185, 2);

    screen_bottom_sprite_203.fillRect(86, 185, 49, 15, TFT_GREEN);
    screen_bottom_sprite_203.drawString(voltage_battery_03, 88, 185, 2);
  }

  // --- END BATTERY RELATED ---









  // Check if it's time to perform an action
  if (currentMillis - keypad_previousMillis >= keypad_interval) {
    // It's time to perform your action
    keypad_previousMillis = currentMillis;  // Save the last time the action was performed

    if (keyPad.isPressed()) {

      char ch = keyPad.getChar();  // note we want the translated char
      key = keyPad.getLastKey();

      // Display Keypress to TFT Screen
      if (ready_message == true) {
        screen_bottom_sprite_203.fillRect(0, 0, 135, 20, TFT_BLACK);  // Changed width from 135 to 80 to use the other 55 pixels 2023-10-30--2003 -KL
        ready_message = false;
      } else {
        screen_bottom_sprite_203.fillRect(70, 0, 65, 20, TFT_BLACK);
      }


      //  screen_bottom_sprite_203.drawString("Keypress: ", 0, 0, 2);
      //tft.drawString(String(ch),70,40,2);
      // screen_bottom_sprite_203.drawString(String(keymap[key]), 70, 0, 2);





      // Prevents hitting the same key twice in a row (maybe code an override by pressing "#" later in case you want to do this)
      if (key != old_key) {

        // "#" Resets "hitting same key twice lock" so you may play the same single-digit (0-9) sound clip twice in a row if desired.
        if (key == 14) {
          old_key = -1;               // Resets old_key to guarentee key will not equal old_key
          phrase_select_buffer = "";  // Clears phrase_select_buffer
          screen_bottom_sprite_203.fillRect(70, 0, 65, 20, TFT_BLACK);
          screen_bottom_sprite_203.fillRect(70, 20, 65, 20, TFT_BLACK);
        } else if (key == 12) {
          old_key = -1;  // Resets old_key to guarentee key will not equal old_key
          phrase_select_buffer = "STOP";
          //************SEND "STOP" TO RECEIVER****************
          xmitData.phrase_select_xmit = phrase_select_buffer;  // ESP-NOW

          screen_bottom_sprite_203.setTextColor(TFT_GREEN);
          screen_bottom_sprite_203.fillRect(70, 0, 65, 20, TFT_BLACK);
          screen_bottom_sprite_203.drawString(String("*"), 70, 0, 2);
          screen_bottom_sprite_203.fillRect(70, 20, 65, 20, TFT_BLACK);
          screen_bottom_sprite_203.drawString(String(phrase_select_buffer), 70, 20, 2);


          //Serial.println("------------key = 12-----------");
          //Serial.println("    KEY:  " + String(key));
          //Serial.println("OLD KEY:  " + String(old_key));
          //Serial.println("    PSB:  " + String(phrase_select_buffer));



          phrase_select_buffer = "";  // // Clear phrase_select_buffer for new entry
        } else {
          old_key = key;

          // If keypress is "A", "B", "C", or "D", wait for a second digit (0-9) before proceeding.
          if (key == 3 || key == 7 || key == 11 || key == 15) {

            // Check to see if phrase_select_buffer already has a letter in it, if it does, ignore the keypress
            //if (phrase_select_buffer != "A" && phrase_select_buffer != "B" && phrase_select_buffer != "C" && phrase_select_buffer != "D")

            //ENTER CHOICE PHRASE_SELECT_BUFFER MODE (So you can select like a jukebox "A4" or "C7")
            //Serial.println("PHRASE_SELECT_BUFFER MODE");

            // This should automatically prevent a selection like "AC" from being selected because it overrides the old phrase_select_buffer
            // every time.

            switch (key) {
              case 3:
                phrase_select_buffer = "A";
                //Serial.println("phrase_select_buffer:  " + phrase_select_buffer);
                break;
              case 7:
                phrase_select_buffer = "B";
                //Serial.println("phrase_select_buffer:  " + phrase_select_buffer);
                break;
              case 11:
                phrase_select_buffer = "C";
                //Serial.println("phrase_select_buffer:  " + phrase_select_buffer);
                break;
              case 15:
                phrase_select_buffer = "D";
                //Serial.println("phrase_select_buffer:  " + phrase_select_buffer);
                break;
            }
          } else {
            if (phrase_select_buffer.length() == 0) {
              // Adds a zero before single digit file names (i.e. - 1 becomes 01)
              phrase_select_buffer = "0";
            }

            phrase_select_buffer += String(keymap[key]);

            screen_bottom_sprite_203.setTextColor(TFT_GREEN);
            screen_bottom_sprite_203.fillRect(70, 20, 65, 20, TFT_BLACK);
            screen_bottom_sprite_203.drawString(String(phrase_select_buffer + ".wav"), 70, 20, 2);

            //Serial.println(String(phrase_select_buffer));

            //*****************SEND PHRASE_SELECT_BUFFER TO RECEIVER**************
            xmitData.phrase_select_xmit = phrase_select_buffer;  // ESP-NOW

            // Clear phrase_select_buffer for new entry
            phrase_select_buffer = "";
          }
        }
      }
    }
  }

  // --- ESP-NOW RELATED ---
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&xmitData, sizeof(xmitData));
}


// ------- ESP-NOW RELATED
// Callback function called when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {

  if (status == ESP_NOW_SEND_SUCCESS) {
    connectStatus = "xmit success";
    connectError = LOW;
  } else {
    connectStatus = "xmit failed";
    connectError = HIGH;
  }
}

// Callback function executed when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {

  // Get receievd data
  memcpy(&rcvData, incomingData, sizeof(rcvData));

  // Pass received values to local variables
}
//
// -------- END ESP-NOW RELATED


void labelsDisplaySprite() {
  // Clear the old displayed data (65px w, 160px h)
  screen_bottom_sprite_203.fillRect(0, 20, 70, 160, TFT_BLACK);
  screen_bottom_sprite_203.setTextColor(TFT_GREEN);

  screen_bottom_sprite_203.drawString("Playing: ", 0, 20, 2);
  screen_bottom_sprite_203.drawString("VOL: ", 0, 40, 2);
  screen_bottom_sprite_203.drawString("EYES: ", 0, 60, 2);
  screen_bottom_sprite_203.drawString("SPOT: ", 0, 80, 2);
  screen_bottom_sprite_203.drawString("L-ARM: ", 0, 100, 2);
  screen_bottom_sprite_203.drawString("R-ARM: ", 0, 120, 2);
  screen_bottom_sprite_203.drawString("NECK: ", 0, 140, 2);
  screen_bottom_sprite_203.drawString("JAW: ", 0, 160, 2);
}


void dataDisplaySprite() {
  // Set the text color to green
  screen_bottom_sprite_203.setTextColor(TFT_GREEN);

  // Clear the old displayed data (65px w, 160px h)(starting 70px over)
  screen_bottom_sprite_203.fillRect(70, 40, 65, 140, TFT_BLACK);

  if (key != 14) {
    if (ready_message == false) {
      screen_bottom_sprite_203.drawString("Keypress: ", 0, 0, 2);
      screen_bottom_sprite_203.drawString(String(keymap[key]), 70, 0, 2);
    }
  }


  // Display all the received data on the sprite
  //screen_bottom_sprite_203.drawString(String(phrase_select_buffer + ".wav"), 70, 20, 2);
  screen_bottom_sprite_203.drawString(String(volume_value), 70, 40, 2);
  screen_bottom_sprite_203.drawString(String(eyes_value), 70, 60, 2);
  screen_bottom_sprite_203.drawString(String(spot_value), 70, 80, 2);
  screen_bottom_sprite_203.drawString(String(left_arm_value), 70, 100, 2);
  screen_bottom_sprite_203.drawString(String(right_arm_value), 70, 120, 2);
  screen_bottom_sprite_203.drawString(String(neck_value), 70, 140, 2);
  screen_bottom_sprite_203.drawString(String(jaw_value), 70, 160, 2);
}


void tftDisplayUpdate() {
  // Updates the tft display with the sprite we've been drawing on
  dataDisplaySprite();
  screen_bottom_sprite_203.pushSprite(0, 38);
}

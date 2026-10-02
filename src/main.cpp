/*
  Arduino Nesso N1 - Display Battery Status

  This example will enable battery charging and display its charging state.

  created: December 11 2025
  by: Ubi de Feo

  This example code is in the public domain.
*/
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include <Arduino_Nesso_N1.h>
#include <RadioLib.h>
#include <Adafruit_GPS.h>
#include <esp_mac.h>  // to get the full 8-byte MAC address
#include <ArduinoJson.h> // JSON library
#include "secrets.h"  // MOD_NAME and such

#define GPSSerial Serial1
Adafruit_GPS GPS(&GPSSerial);

void renderstatusSprite();

#define LORA_XMT 1  // 1 for transmit, 0 for receive

#if defined(LORA_XMT) && (LORA_XMT > 0)
  String modName = "N1-Xmt";

#else
  String modName = "N1-Rcv";

#endif
NessoBattery battery;
NessoDisplay display;

const int DISPLAY_WIDTH = 240;
const int DISPLAY_HEIGHT = 135;
const uint16_t COLOR_TEAL = 0x0410;
const uint16_t COLOR_BLACK = 0x0000;
const uint16_t COLOR_GREEN = 0x1e85;
const uint16_t COLOR_ORANGE = 0xed03;
const uint16_t COLOR_RED = 0xe841;
const uint16_t COLOR_BLUE = 0x001F;
const int ANIMATION_DELAY = 30;
const int COLS = 20;
const int ROWS = 1;
const int REGION_WIDTH = 12;
const int REGION_HEIGHT = 135;
const float LORA_FREQUENCY = 915.0; // Set the LoRa® frequency based on your region
bool display_on = false;

// Initialize the radio module, passing RADIOLIB_NC for the reset pin.
// The reset will be handled manually.
SX1262 radio = new Module(LORA_CS, LORA_IRQ, RADIOLIB_NC, LORA_BUSY);

LGFX_Sprite statusSprite(&display);

// Counter for transmitted packets
int packetCounter = 0;

// Flag to indicate a packet has been received
// Must be volatile because it is changed inside an interrupt
volatile bool receivedFlag = false;

float batteryVoltage = 0.0;
int counter = 0;
char uptimeString[26];
bool ledStatus = false;
unsigned long lastLEDflip = 0;
int progressEdge = 240;
bool progressExpanding = true;

// Interrupt Service Routine (ISR) to set flag if message received
// Keep this function as short as possible
#if defined(ESP8266) || defined(ESP32) || defined(ARDUINO_NESSO_N1)
  ICACHE_RAM_ATTR
#endif
void setFlag(void) {
  receivedFlag = true;
}
// Allocate an 8-byte array for the EUI-64 / IEEE 802.15.4 ID
uint8_t mac8[8];
String globalDeviceId = "ID NOT SET";

// Function that fetches the 8-byte ID and returns it as a formatted String
String getUniqueIdString() {
  uint8_t mac8[8];
  char buffer[24]; // Big enough to hold "XX:XX:XX:XX:XX:XX:XX:XX\0"
  
  if (esp_read_mac(mac8, ESP_MAC_IEEE802154) == ESP_OK) {
    snprintf(buffer, sizeof(buffer), "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
             mac8[0], mac8[1], mac8[2], mac8[3], 
             mac8[4], mac8[5], mac8[6], mac8[7]);
    return String(buffer);
  }
  
  return String("ERROR_READING_ID");
}

void setup() {
  Serial.begin(115200);
  // Turn on external/Grove power output
  pinMode(GROVE_POWER_EN, OUTPUT);
  digitalWrite(GROVE_POWER_EN, HIGH);

  display.begin();

  battery.begin();
  battery.enableCharge();

  display.setRotation(1);
  display.setEpdMode(epd_mode_t::epd_fastest);

  display.fillScreen(TFT_WHITE);
  display.setTextColor(COLOR_TEAL);
  display.setTextSize(5);
  display.drawString(modName, 6, 11);

  // statusSprite.createSprite(240, 81);
  statusSprite.createSprite(240, 135);
  delay(1000);  // wait for Serial to finish configuring with computer
  // pinMode(LED_BUILTIN, OUTPUT);  // All these are defined in Nesso header, do not redefine
  // pinMode(KEY1, INPUT);
  // pinMode(KEY2, INPUT);

  Serial.println("Starting...");

  globalDeviceId = getUniqueIdString();
  Serial.println("ID: " + globalDeviceId);

  // Enable the SX1262 module
  pinMode(LORA_ENABLE, OUTPUT);
  digitalWrite(LORA_ENABLE, HIGH);

  // Enable the LNA (required for receiving - antenna signal won't pass through if LOW)
  pinMode(LORA_LNA_ENABLE, OUTPUT);
  digitalWrite(LORA_LNA_ENABLE, HIGH);

  // Enable the RF antenna switch power
  pinMode(LORA_ANTENNA_SWITCH, OUTPUT);
  digitalWrite(LORA_ANTENNA_SWITCH, HIGH);

  // Initialize the LoRa® module
  Serial.print(F("[SX1262] LoRa Initializing... "));
  int state = radio.begin(LORA_FREQUENCY);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }
  Serial.println(F("success!"));

  // Tell the SX1262 to use its DIO2 pin to control the antenna Rx/Tx switch
  radio.setDio2AsRfSwitch(true);
  // Setup LoRa parameters
  radio.setBandwidth(125000);
  radio.setSpreadingFactor(9);
  radio.setCodingRate(7);
  radio.setPreambleLength(8);
  radio.setSyncWord(0x12);
  // Set the function to call when DIO1 goes HIGH
  radio.setPacketReceivedAction(setFlag);
  // Start the first non-blocking receive operation
  Serial.print(F("[SX1262] Starting to listen ... "));
  state = radio.startReceive();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }

  Serial.println("Initializing IMU...");
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.print("Accel Rate: ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");

  Serial.print("Gyro Rate: ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");

  // 9600 baud is the default rate for the Ultimate GPS
  GPSSerial.begin(9600, SERIAL_8N1, GROVE_IO_0);
  // CANNOT SEND TO GPS SINCE OTHER PIN USED AS PPS
  
  // Initialize the GPS
  if(!GPS.begin(9600)) {
    Serial.println("!!!!! GPS FAILED TO INITIALIZE !!!!!");
  }
  Serial.println("Pausing to let GPS start");
  delay(1000);
  display.sleep();
}

String GPSDataString = "";
bool doXmt = true;
void loop() {
  static bool doBeep = false;
  unsigned long msNow = millis();
  static uint8_t lastKey1 = false;
  static uint8_t lastKey2 = false;
  static uint8_t lastPwrIn = false;
  float ax, ay, az;
  float gx, gy, gz;

  uint8_t curKey1 = digitalRead(KEY1);
  uint8_t curKey2 = digitalRead(KEY2);
  uint8_t curPwrIn = digitalRead(VIN_DETECT);

  while(GPS.available()) {
    GPS.read();
    // Serial.print(GPS.read());
  }
  // Serial.println();

  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if(GPS.parse(GPS.lastNMEA()) && GPS.fix) {
      GPSDataString = GPS.year;
      GPSDataString += "-";
      GPSDataString += GPS.month;
      GPSDataString += "-";
      GPSDataString += GPS.day;
      GPSDataString += " ";
      GPSDataString += GPS.hour;
      GPSDataString += ":";
      GPSDataString += GPS.minute;
      GPSDataString += ":";
      GPSDataString += GPS.seconds;
      GPSDataString += ",";
      GPSDataString += GPS.satellites;
      GPSDataString += ",";
      GPSDataString += String(GPS.latitudeDegrees, 10);
      GPSDataString += ",";
      GPSDataString += String(GPS.longitudeDegrees, 10);
      Serial.println(GPSDataString);
    }
    // if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
    //   return; // we can fail to parse a sentence in which case we should just wait for another
  }
  // while(GPSSerial.available()) {
  //   char c = GPSSerial.read();
  //   Serial.write(c);
  // }

  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);

    if(ay > 0.1) {
      display.setRotation(3);
    } 
    if(ay < -0.1) {
      display.setRotation(1);
    }
  }

  if(curKey1 != lastKey1 && !curKey1) {
    Serial.println("KEY1 pressed");
    doXmt = !doXmt;
    if(doXmt == true) {
      lastLEDflip = 0; // reset to start with a transmit
    }
  }
  if(curKey2 != lastKey2 && !curKey2) {
    Serial.println("KEY2 pressed");
    // doBeep = !doBeep;
    // tone(BEEP_PIN, 4000, 400);
    if(display_on) {
      display.sleep();
      display_on = false;
    } else {
      display.wakeup();
      display_on = true;
    } 
  }
  if(curPwrIn != lastPwrIn && !curPwrIn) {
    delay(5000);
    Serial.println("Power in is LOW");
  }
  if(curPwrIn != lastPwrIn && curPwrIn) {
    delay(5000);
    Serial.println("Power in is HIGH (plugged in)");
  }
   
  float chargeLevel = battery.getChargeLevel();
  batteryVoltage = battery.getVoltage();

  // Create a string to store the received message.
  String str;
  
#if defined(LORA_XMT) && (LORA_XMT == 0)
  if(receivedFlag) { // flag set in interrupt
    receivedFlag = false; // clear flag immediately
    // Try to receive a packet.
    // int state = radio.receive(str);
    int state = radio.readData(str);

    if (state == RADIOLIB_ERR_NONE) {
      // Packet was received successfully.
      Serial.print(F("\n[SX1262] Received packet: "));
      Serial.println(str);
      sprintf(uptimeString, "%s", str.substring(0,25).c_str());
      if(doBeep) {
        tone(BEEP_PIN, 1000, 400);
      }

      // Print packet statistics.
      Serial.print(F("[SX1262] RSSI: "));
      Serial.print(radio.getRSSI());
      Serial.print(F(" dBm, SNR: "));
      Serial.print(radio.getSNR());
      Serial.println(F(" dB"));

    } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
      Serial.println(F("[SX1262] CRC error!"));
      sprintf(uptimeString, "CRC Error");
    } else if (state != RADIOLIB_ERR_RX_TIMEOUT) {
      // Some other error occurred. Timeout is expected and ignored.
      Serial.print(F("[SX1262] Failed, code "));
      Serial.println(state);
      sprintf(uptimeString, "Other Error");
    }  else {
      sprintf(uptimeString, "No packet Received");
    }
    Serial.println(uptimeString);
  }

#endif

  char batteryStatusTicker[16];
  sprintf(batteryStatusTicker, "%4.2f %6.2f%%", batteryVoltage, chargeLevel);
  if (msNow - lastLEDflip > XMT_PERIOD || lastLEDflip == 0) {
    ledStatus = !ledStatus;
    // LED_BUILTIN currently disabled for failures
    digitalWrite(LED_BUILTIN, ledStatus);
    Serial.print(batteryVoltage);
    Serial.print(" ");
    Serial.print(chargeLevel);
    Serial.println("%");

#if defined(LORA_XMT) && (LORA_XMT > 0)
    sprintf(uptimeString, "uptime:\n%012d\n", millis() / 1000);
    Serial.println(uptimeString);

    if(doXmt) {
      JsonDocument doc;
      doc["ModName"] = MOD_NAME;
      doc["MAC"] = globalDeviceId;
      doc["batVolt"] = batteryVoltage;
      doc["batPct"] = chargeLevel;
      doc["pwr"] = battery.getAvgPower();
      if(lastLEDflip == 0) {
        doc["boot"] = "true";
      }
      int state;
      Serial.print(F("[SX1262] Transmitting packet... "));
      char batteryString[7];
      sprintf(batteryString, "%4.2fV", batteryVoltage);
      String xmtStr = "Nesso," + globalDeviceId + "," + String(batteryString);
      if(GPSDataString.length() > 0) {
        doc["GPS"] = GPSDataString;
        // state = radio.transmit(xmtStr + "," + GPSDataString);
      } else {
        // Create a packet with a counter
        doc["Count"] = packetCounter++;
        // String packet = xmtStr + ",#" + String(packetCounter++);
        // state = radio.transmit(packet);
      }
      String myJSONstr;
      serializeJson(doc, myJSONstr);
      state = radio.transmit(myJSONstr, myJSONstr.length());

      if (state == RADIOLIB_ERR_NONE) {
        Serial.println(F("success!"));
      } else {
        Serial.print(F("failed, code "));
        Serial.println(state);
      }
    }
#endif
    lastLEDflip = msNow;
  }
  renderstatusSprite();
  lastKey1 = curKey1;
  lastKey2 = curKey2;
  lastPwrIn = curPwrIn;
}

void renderstatusSprite() {
  int offsetY = 18;
  int fullOffsetY = 54;
  statusSprite.fillSprite(TFT_WHITE);
  if(doXmt) {
    statusSprite.setTextColor(COLOR_RED);
  } else {
    statusSprite.setTextColor(COLOR_TEAL);
  }
  statusSprite.setTextSize(5);
  statusSprite.drawString(modName, 6, 11);
  if(!digitalRead(KEY1)) {
    statusSprite.setColor(COLOR_BLUE);
  } else if(!digitalRead(KEY2)) {
    statusSprite.setColor(COLOR_GREEN);
  } else {
    statusSprite.setColor(COLOR_ORANGE);
  }
  if (progressExpanding) {
    statusSprite.fillRect(progressEdge, 0 + fullOffsetY, 240, 8);
  } else {
    statusSprite.fillRect(0, 0 + fullOffsetY, progressEdge, 8);
  }

  progressEdge -= 1;
  if (progressEdge <= 0) {
    progressEdge = 240;
    progressExpanding = !progressExpanding;
  }
  statusSprite.setTextSize(3);
  statusSprite.setTextColor(COLOR_BLACK);
  statusSprite.drawString("Battery:", 6, offsetY + fullOffsetY);
  uint16_t textColor = 0x0000;
  if (batteryVoltage > 3.7) {
    textColor = COLOR_GREEN;
  } else if (batteryVoltage <= 3.7 && batteryVoltage >= 3.3) {
    textColor = COLOR_ORANGE;
  } else {
    textColor = COLOR_RED;
  }
  char batteryString[6];
  sprintf(batteryString, "%4.2f", batteryVoltage);
  statusSprite.setTextColor(textColor);
  statusSprite.drawString(batteryString, 165, offsetY + fullOffsetY);
  statusSprite.setTextColor(COLOR_BLACK);
  statusSprite.setTextSize(2);

  statusSprite.setTextColor(COLOR_TEAL);
  statusSprite.drawString(uptimeString, 6, offsetY + 38 + fullOffsetY);
  // statusSprite.pushSprite(0, 54);
  statusSprite.pushSprite(0, 0);
  //
}

/* 
 *  Bluetooth Dongle for Scales example with ESP32:
 *  
 *  Publish message "/weight/measure" 
 *  
 */

#include "M1128.h"
#include <BLEDevice.h>
#include <BLEScan.h>

// Debugging Configuration
#define DEBUG true
#define DEBUG_BAUD 115200

// WiFi and MQTT Credentials
#define DEVELOPER_ROOT "197"
#define DEVELOPER_USER "zxAXF9nbvNBfBw9fIWRHILGRL0VClWqU"
#define DEVELOPER_PASS "u0R99elF2XVEt4Tc0LUIxAzjAvVWslQWmH6wWrcZo7DSbx5biZUQzu9a046spXpMFviBXuBmBeeNLkY4jSUTn34n7bQx8fjNKCpI604Ifjv0Dt4WmYQK2U7a5YHed11Y"

#define WIFI_DEFAULT_SSID "Dongle Bluetooth"
#define WIFI_DEFAULT_PASS "abcd1234"

// BLE Configuration
#define SCAN_TIME 3
const char targetMacAddress[] = "10:96:1a:38:b5:2d";

// LED Indikator
#define LED_YELLOW 16 // WiFi belum terhubung
#define LED_BLUE 17   // WiFi sudah terhubung

// Button Reset
#define DEVICE_PIN_RESET 0 // Gunakan 33 jika ingin diubah ke pin lain

int buttonState = 0;


HardwareSerial *SerialDEBUG = &Serial;
M1128 iot;

BLEScan *pBLEScan = nullptr;
bool bleInitialized = false;
bool wifiConnected = false;

// Define a custom callback class
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
  void onResult(BLEAdvertisedDevice advertisedDevice)
  {
    if (advertisedDevice.getAddress().toString() == targetMacAddress)
    {
      Serial.println("Target device found!");
      std::string manufacturerData = advertisedDevice.getManufacturerData();
      if (manufacturerData.length() >= 4)
      {
        uint8_t byte3 = manufacturerData[2];
        uint8_t byte4 = manufacturerData[3];
        uint16_t weightRaw = (byte3 << 8) | byte4;
        float weightKg = weightRaw / 10.0;
        Serial.printf("Weight: %.1f kg\n", weightKg);
        if (iot.mqtt->connected())
        {
          iot.mqtt->publish(iot.constructTopic("DongleBluetooth/Scales"), String(weightKg).c_str(), true);
        }
      }
    }
  }
};

void initializeBLE()
{
  Serial.println("Starting BLE scan...");

  // Initialize BLE
  BLEDevice::init("");

  // Create a BLE scan object
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true); // Active scan to request more data
  pBLEScan->start(SCAN_TIME, false);
}
void callbackOnReceive(char* topic, byte* payload, unsigned int length) {
  String strPayload;
  strPayload.reserve(length);
  for (uint32_t i = 0; i < length; i++) strPayload += (char)payload[i];

  if (DEBUG) {
    SerialDEBUG->print(F("Receiving topic: "));
    SerialDEBUG->println(topic);
    SerialDEBUG->print("With value: ");
    SerialDEBUG->println(strPayload);
  }
  if (strcmp(topic,iot.constructTopic("reset"))==0 && strPayload=="true") iot.reset();
  else if (strcmp(topic,iot.constructTopic("restart"))==0 && strPayload=="true") iot.restart();
}

void initPublish()
{
  if (iot.mqtt->connected())
  {
    iot.mqtt->publish(iot.constructTopic("$state"), "init", false);
    iot.mqtt->publish(iot.constructTopic("$sammy"), "1.0.0", false);
    iot.mqtt->publish(iot.constructTopic("$name"), "Dongle Bluetooth", false);
    iot.mqtt->publish(iot.constructTopic("$model"), "SAM-DB01", false);
    iot.mqtt->publish(iot.constructTopic("$mac"), WiFi.macAddress().c_str(), false);
    iot.mqtt->publish(iot.constructTopic("$localip"), WiFi.localIP().toString().c_str(), false);
    iot.mqtt->publish(iot.constructTopic("$fw/name"), "DB01", false);
    iot.mqtt->publish(iot.constructTopic("$fw/version"), "1.00", false);
    iot.mqtt->publish(iot.constructTopic("$reset"), "true", false);
    iot.mqtt->publish(iot.constructTopic("$restart"), "true", false);
    iot.mqtt->publish(iot.constructTopic("$nodes"), "DongleBluetooth", false);

    // Define node "DongleBluetooth"
    iot.mqtt->publish(iot.constructTopic("DongleBluetooth/$name"), "DB", false);
    iot.mqtt->publish(iot.constructTopic("DongleBluetooth/$type"), "DB-01", false);
    iot.mqtt->publish(iot.constructTopic("DongleBluetooth/$properties"), "Scales", false);

    // Define node "temp"
    iot.mqtt->publish(iot.constructTopic("DongleBluetooth/Scales/$name"), "Weight Measurement", false);
    iot.mqtt->publish(iot.constructTopic("DongleBluetooth/Scales/$settable"), "false", false);
    iot.mqtt->publish(iot.constructTopic("DongleBluetooth/Scales/$retained"), "true", false);
    iot.mqtt->publish(iot.constructTopic("DongleBluetooth/Scales/$datatype"), "float", false);
    iot.mqtt->publish(iot.constructTopic("DongleBluetooth/Scales/$unit"), "kg", false);
    iot.mqtt->publish(iot.constructTopic("DongleBluetooth/Scales/$format"), "0:150", false);

    iot.mqtt->publish(iot.constructTopic("$state"), "ready", false);
  }
}

void callbackOnConnect(){
  initPublish();
}

void callbackOnAPConfigTimeout(){
  iot.restart();
}

void callbackOnWiFiConnectTimeout(){
  iot.restart();
}

void publishState(const char *state){
  if (iot.mqtt->connected())
    iot.mqtt->publish(iot.constructTopic("$state"), state, true);
}


void setup() {
  if (DEBUG) {
    SerialDEBUG->begin(DEBUG_BAUD, SERIAL_8N1); // for ESP32
    while (!SerialDEBUG);
      SerialDEBUG->println(F("Initializing Dongle Bluetooth..."));
  }

  // Setup LED indicators for Wifi
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_YELLOW, HIGH); // Default: Yellow ON (WiFi not connected)
  digitalWrite(LED_BLUE, LOW);

  // Setup Reset WiFi Button
  pinMode(DEVICE_PIN_RESET, INPUT);
  iot.pinReset = DEVICE_PIN_RESET;


// Initialize Wi-Fi and MQTT
iot.prod = true;
iot.cleanSession = true;
iot.setWill = true;
iot.apConfigTimeout = 300000;
iot.wifiConnectTimeout = 120000;
iot.devConfig(DEVELOPER_ROOT, DEVELOPER_USER, DEVELOPER_PASS);
iot.wifiConfig(WIFI_DEFAULT_SSID, WIFI_DEFAULT_PASS);

// Set up callback functions
iot.onReceive = callbackOnReceive;
iot.onConnect = callbackOnConnect;
iot.onAPConfigTimeout = callbackOnAPConfigTimeout;
iot.onWiFiConnectTimeout = callbackOnWiFiConnectTimeout;

iot.init(DEBUG ? SerialDEBUG : NULL);
// delay(10);
}

bool bleRunning = false;

void loop() {
  iot.loop();

  if (iot.mqtt->connected()) {
      digitalWrite(LED_YELLOW, LOW);
      digitalWrite(LED_BLUE, HIGH);
      initializeBLE();
  } else {
      digitalWrite(LED_YELLOW, HIGH);
      digitalWrite(LED_BLUE, LOW);
  }
}




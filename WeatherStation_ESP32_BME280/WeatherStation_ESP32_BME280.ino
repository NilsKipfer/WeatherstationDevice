/***************************************************************************
  Weatherstation

  These sensors use I2C to communicate, 2 pins are required
  to interface. The device's I2C address is either 0x76.
  The sensor connects to the WiFi and reads the MAC address.
  A time server is used to set the time in the corresponding time zone.
  The sensor data is summarized as JSON.
  The JSON is sent via MQTT with the defined topic to the corresponding MQTT broker. 

  Read Data from the BME280 and send them to a MQTT Broker.

  Written by Nils Kipfer for TEKO Project.
 ***************************************************************************/

#include <WiFi.h>             // https://github.com/arduino-libraries/WiFi
#include <Adafruit_BME280.h>  // https://github.com/adafruit/Adafruit_BME280_Library
#include <time.h>             // https://github.com/PaulStoffregen/Time
#include <ArduinoJson.h>      // https://github.com/bblanchon/ArduinoJson
#include <PubSubClient.h>     // https://github.com/knolleary/pubsubclient
// I2C
Adafruit_BME280 bme;
// Network-Configuration
const char* ssid = "REPLACE_WITH_YOUR_SSID";
const char* password = "REPLACE_WITH_YOUR_SSID_PASSWORD";
// MQTT-Broker
const char* mqttServer = "REPLACE_WITH_YOUR_MQTT-Broker_IP";
// Configutration-Time
const char* ntpServer1 = "pool.ntp.org";
const char* ntpServer2 = "time.nist.gov";
const long gmtOffset_sec = 3600;
const int daylightOffset_sec = 3600;
const char* time_zone = "CET-1CEST,M3.5.0,M10.5.0/3";
// MQTT-Variables
#define MQTTConnect "RFIDTempSensorClient"
#define MQTTPort 1883
#define MQTTPUB "REPLACE_WITH_YOUR_MQTT-TOPIC"
// Timer-Variables
unsigned long delayStart = millis();       // Millisecons
unsigned long delayTime = 15 * 60 * 1000;  //Minutes, Seconds, Milliseconds  15 * 60 * 1000
// MQTT
WiFiClient espClient;
PubSubClient client(espClient);
StaticJsonDocument<256> doc;
// Setup
void setup() {
  Serial.begin(115200);
  // Sensor Configuration
  unsigned status;
  status = bme.begin(0x76);
  // Connect to Wi-Fi network with SSID and password
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
  }
  // Connect to MQTT
  client.setServer(mqttServer, MQTTPort);
  // Connect to NTP-Server
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1, ntpServer2);
}
// Loop
void loop() {
  //Connect to MQTT
  if (!client.connected()) {
    if (client.connect("MQTTConnect")) {
    }
  }
  // MQTT Loop
  client.loop();
  // Read and send Sensor Data
  if (delayStart != 0) {
    if (delayStart + delayTime <= millis()) {
      readValues();
      delayStart = millis();
    }
  }
}
//MQTT Functions
void MQTT_Publish(char* topic, char* payload) {
  client.publish(topic, payload);
}
// Read Sensor Data Function
void readValues() {
  String MAC = WiFi.macAddress();
  float Temp = bme.readTemperature();
  float Pressure = (bme.readPressure() / 100.0F);
  float Humidity = bme.readHumidity();
  struct tm timeinfo;
  getLocalTime(&timeinfo);
  char datetimeString[20];  // Platz für das Datum und die Uhrzeit im Format "%Y-%m-%d %H:%M:%S"
  strftime(datetimeString, sizeof(datetimeString), "%Y-%m-%d %H:%M:%S", &timeinfo);
  createJSON(MAC, datetimeString, Temp, Pressure, Humidity);
}
// Create and Work with JSON
void createJSON(String MAC, String datetimeString, float Temp, float Pressure, float Humidity) {
  // Clear JSON
  doc.clear();
  // Generate a JSON
  doc["MAC"] = MAC;
  doc["Time"] = datetimeString;
  doc["Temperature"] = Temp;
  doc["Pressure"] = Pressure;
  doc["Humidity"] = Humidity;
  // JSON to char*
  char payload[256];
  serializeJson(doc, payload);
  // MQTT Publish
  MQTT_Publish(MQTTPUB, payload);
}
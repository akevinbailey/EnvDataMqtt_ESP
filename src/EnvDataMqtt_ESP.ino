/*
 * Copyright (c) 2025. Andrew Kevin Bailey
 * This code, firmware, and software is released under the MIT License (http://opensource.org/licenses/MIT).
 *
 * The MIT License (MIT)
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or significant portions of
 * the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 * BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/*
  Hardware connections:
  BME280 -> ESB32 C3
  GND -> GND
  3.3 -> 3.3
  SDA -> GP20
  SCL -> GP21
*/
#include <NimBLEDevice.h>
#define XIAO
#define BATTERY
//#define ESP32

#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <SparkFunBME280.h>
#include <ESP_SSLClient.h>
#include <WiFiClient.h>
#include <ArduinoMqttClient.h> // https://github.com/arduino-libraries/ArduinoMqttClient
#include <Arduino_JSON.h>      // https://github.com/arduino-libraries/Arduino_JSON
#include <HTTPClient.h>
#include <NetworkClientSecure.h>

#include <algorithm>
#include "time.h"
#include "esp_sntp.h"
#include "BleConfigService.h"

#define ENABLE_DEBUG        // To enable debugging 
#define ENABLE_ERROR_STRING // To show details in error
constexpr auto uS_TO_mS_FACTOR = 1000ULL /* Conversion factor for micro seconds to milliseconds */;

/***** Application and device information *****/
#define APP_NAME          "MQTT Environmental Sensor"
#define VERSION           "1.0"
#define AUTHOR            "Andrew Kevin Bailey"
#define DEFAULT_SENSOR_ID "xiaoc3-bme280"

/***** Define I2C GPIO pins and sensor address *****/
#if defined(XIAO)
  // For XIAO-C3
#define I2C_SDA0 6
constexpr auto I2C_SCL0 = 7;
constexpr auto ADC1_0 = 2;
constexpr auto ADC_MAX = 4095;
#elif defined(ESP32)
  // For ESP32-C3-DEVKIT
  #define I2C_SDA0 2
  #define I2C_SCL0 3
  #define ADC1_0 0
  #define ADC_MAX 4095
  // For ESP32-S2-DEVKIT
  //#define I2C_SDA0 4
  //#define I2C_SCL0 5
  //#define ADC1_0 1
  //#define ADC_MAX 8191
#endif

/***** Sensor I2C init *****/
constexpr auto BME280_I2C_ADDR = (0x76);
BME280 envSensor; // I2C Environment sensor
bool sensorReady = false;

/**** Bluetooth and Properties init *****/
BleConfigService configService(DEFAULT_SENSOR_ID);

/***** Wifi init *****/
String strEnv = "";
String strWiFi = "";
ESP_SSLClient sslClient;
WiFiClient wifiClient;

/***** MQTT init *****/
bool mqttReady = false;
MqttClient mqttClient(sslClient);

double operatingVoltage = 3.3; // Device's operating power voltage
unsigned long sleepTimeMs = 0; // milliseconds
unsigned long awakeTimeMs = 0; // milliseconds
unsigned long sampleFrequencyMs = 5000; // milliseconds
unsigned long ntpFrequencyMs = 86400000; // milliseconds
String ntpAddress = "";
bool sendVoltage = false;
bool sendTime = false;
bool sendTemperature = true;
bool sendHumidity = true;
bool sendPressure = true;
bool sendLight = false;
bool sendAirQualityIndex = false;
bool sendTvoc = false;
bool sendCo2eq = false;
bool sendLocation = false;
bool sendAcceleration = false;
bool enableBluetooth = true;

double voltageCorrection = 0;
double temperatureCorrection = 0;
double humidityCorrection = 0;
long pressureCorrection = 0;
long lightCorrection = 0;
long airQualityIndexCorrection = 0;
long tvocCorrection = 0;
long co2eqCorrection = 0;
double accelerationCorrection = 0;

/***** NTP parameters *****/
constexpr long gmtOffsetSec = 0; // UTC time
constexpr int daylightOffsetSec = 0; // UTC time 
unsigned long timeSyncCount = 0;
String backupNtpAddress = "pool.ntp.org";
String nowIso8601 = "";

/***** Timer init *****/
constexpr unsigned long poleFrequencyMs = 1000; // and MQTT keep-alive
unsigned long prePoleMs = 0;
unsigned long prePublishMs = 0;
unsigned long preAwakeMs = 0;

/***** Global functions *****/
void wifiSetup();
void configSetup();
void ntpSetup();
void bme280Setup();
void mqttSetup();
String getIsoTime();
float getVoltage();
void timeCallback(struct timeval* tv);

/***** The setup function runs once when you press reset or power the board *****/
void setup() {
  Serial.begin(115200);
  Wire.setClock(100000);
  Wire.begin();
  delay(1000);

  #if !defined(XIAO) || !defined(BATTERY) // XIAO does not have a LED
    // Enable the LED, so we know the board is on
    pinMode(LED_BUILTIN, OUTPUT);
    //digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  #endif  
  // Enable voltage sensor
  pinMode(ADC1_0, INPUT);

  /***** Begin load data from properties or Bluetooth *****/
  if (!configService.loadProperties()) {
    Serial.println("ERROR:: loading setup properties from non-volatile storage.");
    while (true); // freeze the microcontroller because we cannot go any further
  }

  if (!configService.isSavedData()) {
    // Get config from Bluetooth and save it to the EEPROM
    // TODO:  

  } 
  /***** End load data from properties or Bluetooth *****/ 

  // Set up WiFi
  wifiSetup();

  // Set up TLS
  sslClient.setClient(&wifiClient);
  sslClient.setInsecure();  // ignore server ssl certificate verification
  // Set receive and transmit buffers size in bytes for memory allocation (512 to 16384)
  sslClient.setBufferSizes(1024 /* rx */, 512 /* tx */);
  /** Call setDebugLevel(level) to set the debug
  * esp_ssl_debug_none = 0
  * esp_ssl_debug_error = 1
  * esp_ssl_debug_warn = 2
  * esp_ssl_debug_info = 3
  * esp_ssl_debug_dump = 4
  */
  sslClient.setDebugLevel(1);  

  // Set up config
  configSetup();  // uses a REST API and needs WiFi and SSL

  // Set up sleep time
  if (sleepTimeMs > 0) {
	  sleepTimeMs = std::max<unsigned long>(sleepTimeMs, 1000); // minimum sleep time is 1 sec
    // For reduced power level use external 32k oscillator (CONFIG_ESP32S3_RTC_CLK_SRC_EXT_OSC)
    esp_sleep_enable_timer_wakeup(sleepTimeMs * uS_TO_mS_FACTOR);
  }

  // Set up voltage sensor calibration 
  #if defined(XIAO) || defined(BATTERY)
    // TODO:
  #endif

  // Start Bluetooth if enabled
  if (enableBluetooth) configService.beginBle();
  
  // Set up NTP if sendTime
  if (sendTime) ntpSetup();
  
  // Set up environment sensor
  bme280Setup();
  
  // Set up MQTT
  mqttSetup();  

  Serial.println("****************************** Program Started ******************************");
}

/***** The loop function runs over and over again forever *****/
void loop() {

  if (!(mqttReady && sensorReady)) {
    #if !defined(XIAO) || !defined(BATTERY)
      digitalWrite(LED_BUILTIN, LOW);
    #endif
    return; // Stop executing
  }
  // To avoid having delays in loop, we'll use the strategy from BlinkWithoutDelay
  // see: File -> Examples -> 02.Digital -> BlinkWithoutDelay for more info
  const unsigned long currentMs = millis();

  // Flash the LED on and off every 2 * flashInterval and poll the MQTT server
  if ((currentMs - prePoleMs) >= poleFrequencyMs) {
    prePoleMs = currentMs;
    #if !defined(XIAO) || !defined(BATTERY) // XIAO does not have a LED
      digitalWrite(LED_BUILTIN, digitalRead(LED_BUILTIN) == HIGH ? LOW : HIGH);  // toggle LED
    #endif
    // Poll to prevent dropping the connection
    mqttClient.poll();
  }

  // Publish an environmental reading every publishInterval
  if ((currentMs - prePublishMs) >= sampleFrequencyMs) {
    prePublishMs = currentMs;

    // Create JSON string:
    String mqttMessage = "{";
    
    mqttMessage += R"("sensor_id": ")" + configService.getSensorId() + "\",";
    if (sendTime) {
      nowIso8601 = getIsoTime();
      if (nowIso8601.length() != 0) mqttMessage += R"("time": ")" + nowIso8601 + "\",";
    }
    if (sendVoltage) mqttMessage += "\"voltage\": " + String(getVoltage() + voltageCorrection, 2) + ",";
    if (sendTemperature) mqttMessage += "\"temperature\": " + String(envSensor.readTempC() + temperatureCorrection, 2) + ",";
    if (sendHumidity) mqttMessage += "\"humidity\": " + String(envSensor.readFloatHumidity() + humidityCorrection, 2) + ",";
    if (sendPressure) mqttMessage += "\"pressure\": " + String(envSensor.readFloatPressure() + static_cast<float>(pressureCorrection) , 0) + ",";

    /***** The sensors below are not avalable for every project *****/
    // if (sendLight) mqttMessage += "\"light\": " + String("") + ",";
    // if (sendAirQualityIndex) mqttMessage += "\"air_quality_index\": " + String("") + ",";
    // if (sendTvoc) mqttMessage += "\"tvoc\": " + String("") + ",";
    // if (sendCo2eq) mqttMessage += "\"co2eq\": " + String("") + ",";
    // if (sendLocation) {
    //   mqttMessage += "\"latitude\": " + String("\"\"") + ",";
    //   mqttMessage += "\"longitude\": " + String("\"\"") + ",";
    // }
    // if (sendAcceleration) mqttMessage += "\"acceleration\": " + String("") + ",";

    mqttMessage.remove(mqttMessage.length() - 1, 1); // remove the comma on the last line
    mqttMessage += "}";
        
    // Send message with retain=false, QoS=0, duplicates=false 
    mqttClient.beginMessage(configService.getMqttTopic(), false, 0, false);
    // The Print interface can be used to set the message contents
    mqttClient.print(mqttMessage);
    mqttClient.endMessage();
    
    Serial.println("Published message to topic '" + configService.getMqttTopic() + "':  " + mqttMessage);
  }

  // If there is a sleep time, go to sleep after the awake time expires
  if (sleepTimeMs > 0 && (currentMs - preAwakeMs) >= awakeTimeMs) {
    preAwakeMs = currentMs;
    #if !defined(XIAO) || !defined(BATTERY) // XIAO does not have a LED
      digitalWrite(LED_BUILTIN, LOW);
    #endif
    mqttClient.flush();
    Serial.flush();
    mqttClient.stop();

    Serial.println();
    Serial.printf("Device going to sleep: wakes in %lu seconds\n", sleepTimeMs/1000);
    Serial.println();
    esp_deep_sleep_start();
  }
}

/***** Set up WiFi *****/ 
void wifiSetup() {
  // Operate in WiFi Station mode
  WiFiClass::mode(WIFI_STA);

  if (!configService.isDhcp()) {
    WiFi.config(configService.getLocalIp(), configService.getGatewayIp(), configService.getSubnet(), configService.getDns1Ip(), configService.getDns2Ip());
  }
 
  // Start WiFi with supplied parameters
  WiFiClass::hostname(configService.getSensorId());
  WiFi.begin(configService.getWifiSsid(), configService.getWifiPassword());
 
  // Print periods on monitor while establishing connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    delay(500);
  }
  Serial.print("\n\n");
  // Connection established
  Serial.print(configService.getSensorId());
  Serial.print(" is connected to WiFi network ");
  Serial.println(WiFi.SSID());
  Serial.print("Assigned IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.println();
}

/***** Set up the device's configuration using the settings in the central configuration server *****/ 
void configSetup() {
  const auto tlsConfigClient = new NetworkClientSecure;
  String responseBody = "";

  if (tlsConfigClient) {
    tlsConfigClient->setCACert(configService.getCaCertificate().c_str());
    String params = "?configName=" + configService.getConfigName();
    String fullUrl = configService.getHttpConfigUrl() + params;
    Serial.println("Attempting to get configuration using URL: " + fullUrl);
    
    {
      // Add a scoping block for HTTPClient https to make sure it is destroyed before NetworkClientSecure *tlsConfigClient is
      Serial.println("[HTTPS] begin...");
      if (HTTPClient https; https.begin(*tlsConfigClient, fullUrl.c_str())) {  // HTTPS
        Serial.println("[HTTPS] GET...");
        // start connection and send HTTP header

        // httpCode will be negative on error
        if (const int httpCode = https.GET(); httpCode > 0) {
          // HTTP header has been sent and Server response header has been handled
          Serial.print("[HTTPS] GET code: ");
          Serial.println(httpCode);

          // HTTP GET successful
          if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {
            responseBody = https.getString();
            Serial.println("[HTTPS] response body:  " + responseBody);
          }
        }
        else {
          Serial.print("[HTTPS] GET failed, error: ");
          Serial.println(HTTPClient::errorToString(httpCode).c_str());
        }

        https.end();
      }
      else {
        Serial.println("[HTTPS] Unable to connect.");
      }
      // End extra scoping block
    }

    delete tlsConfigClient;
  }
  else {
    Serial.println("Unable to create client");
  }

  JSONVar jsonConfig = JSON.parse(responseBody);

  if (jsonConfig["sleepTimeMs"] != null)  
    sleepTimeMs = jsonConfig["sleepTimeMs"];
  if (jsonConfig["awakeTimeMs"] != null)  
    awakeTimeMs = jsonConfig["awakeTimeMs"];
  if (jsonConfig["sampleFrequencyMs"] != null)  
    sampleFrequencyMs = jsonConfig["sampleFrequencyMs"];
  if (jsonConfig["ntpFrequencyMs"] != null)
    ntpFrequencyMs = jsonConfig["ntpFrequencyMs"];
  if (jsonConfig["ntpAddress"] != null)
    ntpAddress = String(jsonConfig["ntpAddress"]);
  if (jsonConfig["sendVoltage"] != null)  
    sendVoltage = jsonConfig["sendVoltage"];
  if (jsonConfig["sendTime"] != null)  
    sendTime = jsonConfig["sendTime"];
  if (jsonConfig["sendTemperature"] != null)  
    sendTemperature = jsonConfig["sendTemperature"];
  if (jsonConfig["sendHumidity"] != null)  
    sendHumidity = jsonConfig["sendHumidity"];
  if (jsonConfig["sendPressure"] != null)  
    sendPressure = jsonConfig["sendPressure"];
  if (jsonConfig["sendLight"] != null)  
    sendLight = jsonConfig["sendLight"];
  if (jsonConfig["sendAirQualityIndex"] != null)  
    sendAirQualityIndex = jsonConfig["sendAirQualityIndex"];
  if (jsonConfig["sendTvoc"] != null)  
    sendTvoc = jsonConfig["sendTvoc"];
  if (jsonConfig["sendCo2eq"] != null)  
    sendCo2eq = jsonConfig["sendCo2eq"];
  if (jsonConfig["sendLocation"] != null)  
    sendLocation = jsonConfig["sendLocation"];
  if (jsonConfig["sendAcceleration"] != null)  
    sendAcceleration = jsonConfig["sendAcceleration"];
  if (jsonConfig["enableBluetooth"] != null)  
    enableBluetooth = jsonConfig["enableBluetooth"];

  if (jsonConfig["voltageCorrection"] != null)  
    voltageCorrection = jsonConfig["voltageCorrection"];
  if (jsonConfig["temperatureCorrection"] != null)  
    temperatureCorrection = jsonConfig["temperatureCorrection"];
  if (jsonConfig["humidityCorrection"] != null)  
    humidityCorrection = jsonConfig["humidityCorrection"];
  if (jsonConfig["pressureCorrection"] != null)  
    pressureCorrection = jsonConfig["pressureCorrection"];
  if (jsonConfig["lightCorrection"] != null)  
    lightCorrection = jsonConfig["lightCorrection"];
  if (jsonConfig["airQualityIndexCorrection"] != null)  
    airQualityIndexCorrection = jsonConfig["airQualityIndexCorrection"];
  if (jsonConfig["tvocCorrection"] != null)  
    tvocCorrection = jsonConfig["tvocCorrection"];
  if (jsonConfig["co2eqCorrection"] != null)  
    co2eqCorrection = jsonConfig["co2eqCorrection"];
  if (jsonConfig["accelerationCorrection"] != null)  
    accelerationCorrection = jsonConfig["accelerationCorrection"];

  Serial.println();
}

/***** Set up NTP *****/ 
void ntpSetup() {
  esp_sntp_setoperatingmode(SNTP_OPMODE_POLL); // Act as a client
  sntp_set_sync_mode(SNTP_SYNC_MODE_IMMED); // Correct time immediately
  sntp_set_sync_interval(ntpFrequencyMs); // How often to check the SNTP server

  // Set NTP return notification call-back function
  sntp_set_time_sync_notification_cb(timeCallback);

  // Start the SNTP sync service
  if (ntpAddress.length() == 0) configTime(gmtOffsetSec, daylightOffsetSec, backupNtpAddress.c_str());
  else configTime(gmtOffsetSec, daylightOffsetSec, ntpAddress.c_str(), backupNtpAddress.c_str());
}

/***** Set up MQTT *****/ 
void mqttSetup() {
  // Assign the basic WIFI client
  // Due to the wifiClient pointer is assigned, to avoid dangling pointer, wifiClient should be existed
  // as long as it was used by sslClient for transportation.
  Serial.print("Attempting to connect to the MQTT broker over TLS: ");
  Serial.println(configService.getMqttServer());
  
  mqttClient.setUsernamePassword(configService.getMqttUsername().c_str(), configService.getMqttPassword().c_str());

  if (!mqttClient.connect(configService.getMqttServer().c_str(), configService.getMqttPort()))
  {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());
    return;
  }

  Serial.print("Connected to the MQTT broker on TLS port ");
  Serial.println(wifiClient.remotePort());
  Serial.println();
  mqttReady = true;
}

/***** Set up environment sensor *****/ 
void bme280Setup() {
  Wire.setPins(I2C_SDA0, I2C_SCL0);

  Wire.begin();
  envSensor.setI2CAddress(BME280_I2C_ADDR);

 // Start environment sensor
  bool sensorStarted = envSensor.beginI2C(Wire);

  if (!sensorStarted) {
      Serial.println("Could not find a valid BME280 sensor, check wiring!");
      return;
  }

  Serial.print("I2C environmental sensor ready in mode ");
  Serial.println(envSensor.getMode());
  Serial.println();  
  sensorReady = true;
}

/***** Callback function: gets called when the the time is synced by the SNTP service  *****/
void timeCallback(struct timeval *t) {
  timeSyncCount++;
  Serial.println();  
  if (ntpAddress.length() == 0)
    Serial.printf("Time synchronized with %s.  Synchronization count %lu\n", backupNtpAddress.c_str(), timeSyncCount);
  else
    Serial.printf("Time synchronized with %s or %s.  Synchronization count %lu\n", ntpAddress.c_str(), backupNtpAddress.c_str(), timeSyncCount);
  Serial.println();  
}

/***** Get current time in ISO 8601 format  *****/
String getIsoTime()
{
 struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("No time available (yet)");
    return "";
  }
  /*
  struct tm
  {
    int	tm_sec;
    int	tm_min;
    int	tm_hour;
    int	tm_mday;
    int	tm_mon;
    int	tm_year;
    int	tm_wday;
    int	tm_yday;
    int	tm_isdst;
  }
  */
  char isoTime[26]; // 25 characters for the ISO 8601 string plus a null terminator
  // The tm_year is given from 1900 and the tm_month is 0 based (0 = January)
  sprintf(isoTime, "%04d-%02d-%02dT%02d:%02d:%02d+00:00", timeinfo.tm_year + 1900, timeinfo.tm_mon + 1,  // NOLINT(cert-err33-c)
          timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);

  return {isoTime};
}

/***** Get the estimated power voltage  *****/
float getVoltage() {
  const int adcValue = analogRead(ADC1_0);
  const float voltage = static_cast<float>(adcValue) * static_cast<float>(operatingVoltage) / ADC_MAX;
  return voltage;
}
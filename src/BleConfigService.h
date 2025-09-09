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
// ReSharper disable CppClangTidyClangDiagnosticPadded
// ReSharper disable CppClangTidyClangDiagnosticOverloadedVirtual
#pragma once

#include <NimBLEDevice.h> // need to include first
#if __has_include(<NimBLEConnInfo.h>)
#define HAS_NIMBLE_CONNINFO 1
#endif

#include <array>
#include <string>   // for rxBuffer_
#include <Arduino.h>
#include <IPAddress.h>

/*
FYI:
Typical emulated EEPROM size is no more than 8 KB (8192 bytes).
Larger sizes may result in inefficiencies.
*/

// Struct for the Ethernet settings
typedef struct {
  // Must use std::array<std::uint8_t, 4> in place of the Arduino IPAddress class
  // to be able to store the struct in EEPROM.
  std::array<std::uint8_t, 4> localIp;
  std::array<std::uint8_t, 4> subnet;
  std::array<std::uint8_t, 4> dns1Ip;
  std::array<std::uint8_t, 4> dns2Ip;
  std::array<std::uint8_t, 4> gatewayIp;
} Ethernet_Properties_t;

// Location of data in the EEPROM
#define EERPOM_SAVED_ADDR      0  // NOLINT(modernize-macro-to-enum)
#define EERPOM_WRITES_ADDR     (EERPOM_SAVED_ADDR      + sizeof(uint8_t))
#define EEPROM_ETHERNET_ADDR   (EERPOM_WRITES_ADDR     + sizeof(uint32_t))
#define EEPROM_WIFISSID_ADDR   (EEPROM_ETHERNET_ADDR   + sizeof(Ethernet_Properties_t))
#define EEPROM_WIFIPASS_ADDR   (EEPROM_WIFISSID_ADDR   + sizeof(char) * 33)
#define EEPROM_SENSORID_ADDR   (EEPROM_WIFIPASS_ADDR   + sizeof(char) * 65)
#define EEPROM_CONFIGNAME_ADDR (EEPROM_SENSORID_ADDR   + sizeof(char) * 37)
#define EEPROM_CONFIGURL_ADDR  (EEPROM_CONFIGNAME_ADDR + sizeof(char) * 37)
#define EEPROM_MQTTSERVER_ADDR (EEPROM_CONFIGURL_ADDR  + sizeof(char) * 1025)
#define EEPROM_MQTTPORT_ADDR   (EEPROM_MQTTSERVER_ADDR + sizeof(char) * 257)
#define EEPROM_MQTTUSER_ADDR   (EEPROM_MQTTPORT_ADDR   + sizeof(uint16_t))
#define EEPROM_MQTTPASS_ADDR   (EEPROM_MQTTUSER_ADDR   + sizeof(char) * 25)
#define EEPROM_MQTTTOPIC_ADDR  (EEPROM_MQTTPASS_ADDR   + sizeof(char) * 1025)
#define EEPROM_CACERT_ADDR     (EEPROM_MQTTTOPIC_ADDR  + sizeof(char) * 1025)
#define EEPROM_DATASIZE        (EEPROM_CACERT_ADDR     + sizeof(char) * 3072 + 256) // add 256 to make sure to erase all the data from previous builds

// A self-contained BLE configuration service for ESP32-C3 using NimBLE.
// Upload a single JSON payload in chunks, then COMMIT to apply & persist.
// Provides getters/setters for all fields so your app can safely read/update.

class BleConfigService {
public:
  /***** Lifecycle *****/
  // deviceName appears in BLE scan results
  explicit BleConfigService(const char* sensorId);

  void beginBle(); // Call once in setup()
  static void stopBle();
  static void clearEEPROM();

  [[nodiscard]] uint32_t getEepromWrites() const;
  [[nodiscard]] Ethernet_Properties_t getEthernetProperties() const;
  [[nodiscard]] IPAddress getLocalIp() const;
  [[nodiscard]] IPAddress getSubnet() const;
  [[nodiscard]] IPAddress getDns1Ip() const;
  [[nodiscard]] IPAddress getDns2Ip() const;
  [[nodiscard]] IPAddress getGatewayIp() const;
  [[nodiscard]] String getStrLocalIp();
  [[nodiscard]] String getStrSubnet();
  [[nodiscard]] String getStrDns1Ip();
  [[nodiscard]] String getStrDns2Ip();
  [[nodiscard]] String getStrGatewayIp();
  [[nodiscard]] String getWifiSsid();
  [[nodiscard]] String getWifiPassword();
  [[nodiscard]] String getSensorId();
  [[nodiscard]] String getConfigName();
  [[nodiscard]] String getHttpConfigUrl();
  [[nodiscard]] String getMqttServer();
  [[nodiscard]] uint16_t getMqttPort() const;
  [[nodiscard]] String getMqttUsername();
  [[nodiscard]] String getMqttPassword();
  [[nodiscard]] String getMqttTopic();
  [[nodiscard]] String getCaCertificate();

  void setEthernetProperties(const Ethernet_Properties_t& ethernetProperties);
  void setEthernetProperties(IPAddress localIp, IPAddress subnet, IPAddress dns1Ip, IPAddress dns2Ip, IPAddress gatewayIp);
  bool setEthernetProperties(const char* strLocalIp, const char* strSubnet, const char* strDns1Ip, const char* strDns2Ip, const char* strGatewayIp);
  bool setLocalIp(const char* strLocalIp);
  bool setSubnet(const char* strSubnet);
  bool setDns1Ip(const char* strDns1Ip);
  bool setDns2Ip(const char* strDns2Ip);
  bool setGatewayIp(const char* strGatewayIp);
  void setWifiSsid(const char* s);
  void setWifiPassword(const char* s);
  void setSensorId(const char* s);
  void setConfigName(const char* s);
  void setHttpConfigURL(const char* s);
  void setMqttServer(const char* s);
  void setMqttPort(uint16_t p);
  void setMqttUsername(const char* s);
  void setMqttPassword(const char* s);
  void setMqttTopic(const char* s);
  void setCaCertificate(const char* s);

  bool isDhcp() const;
  bool isSavedData() const;
  bool saveProperties();
  bool loadProperties();

  // Convenience: reflect SensorId into the BLE advertised name suffix (call after setSensorId)
  void refreshAdvertisedName() const;

private:
  /***** BLE UUIDs (keep constant across devices; devices differ by MAC/name) *****/
  static constexpr auto SVC_UUID   = "f86e3d3a-1b1e-48a6-9b8a-0d7f0a3bfa10";
  static constexpr auto CTRL_UUID  = "f86e3d3a-1b1e-48a6-9b8a-0d7f0a3bfa11";
  static constexpr auto DATA_UUID  = "f86e3d3a-1b1e-48a6-9b8a-0d7f0a3bfa12";
  static constexpr auto STAT_UUID  = "f86e3d3a-1b1e-48a6-9b8a-0d7f0a3bfa13";

  uint8_t  savedData_; // 1 for true and 0 for false
  /**
   * 
   */
  uint32_t eepromWrites_;
  Ethernet_Properties_t ethProps_;
  bool isDhcp_;

  /***** Fixed-size storage *****/
  char strLocalIp_[16]     = {0};
  char strSubnet_[16]      = {0};
  char strDns1Ip_[16]      = {0};
  char strDns2Ip_[16]      = {0};
  char strGatewayIp_[16]   = {0};
  char wifiSsid_[33]       = {0};
  char wifiPassword_[65]   = {0};
  char sensorId_[37]       = {0};
  char configName_[37]     = {0};
  char httpConfigUrl_[1025]= {0};
  char mqttServer_[257]    = {0};
  uint16_t mqttPort_       = 1883;
  char mqttUsername_[25]   = {0};
  char mqttPassword_[1025] = {0};
  char mqttTopic_[1025]    = {0};
  char caCertificate_[3072]= {0};

  /***** BLE objects *****/
  NimBLEServer*          server_   = nullptr;
  NimBLEService*         svc_      = nullptr;
  NimBLECharacteristic*  ctrl_     = nullptr;
  NimBLECharacteristic*  data_     = nullptr;
  NimBLECharacteristic*  status_   = nullptr;
  NimBLEAdvertising*     adv_      = nullptr;

  /***** Assembly state for chunked JSON *****/
  bool collecting_         = false;
  std::size_t expectedLen_      = 0;
  std::string rxBuffer_;

  /***** Helpers *****/
  void safeCopy(char* dst, std::size_t dstSize, const char* src) const;
  static bool looksLikeIPv4(const char* s);
  void notifyStatus(const char* msg) const;
  bool applyConfigFromJSON(const char* json, std::size_t len, String& err);
  void generateAllIpStrings();
  static String generateIpString(IPAddress ipAddress);

  /***** Callbacks *****/
  class ServerCallbacks : public NimBLEServerCallbacks {
  public:
    explicit ServerCallbacks(BleConfigService* self) : self_(self) {}
    void onConnect(NimBLEServer* pServer) const;
    void onConnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) const;
    void onDisconnect(NimBLEServer* pServer) const;
   #ifdef HAS_NIMBLE_CONNINFO
    void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override;
    void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override;
   #endif
private:
    BleConfigService* self_;  
  };

  class CtrlCallbacks : public NimBLECharacteristicCallbacks {
  public:
    explicit CtrlCallbacks(BleConfigService* self) : self_(self) {}
    void onWrite(const NimBLECharacteristic* c) const;                           // older/common
  #ifdef HAS_NIMBLE_CONNINFO
    void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& info) override;     // newer
  #endif
  private:
    BleConfigService* self_;    
  };

  class DataCallbacks : public NimBLECharacteristicCallbacks {
  public:
    explicit DataCallbacks(BleConfigService* self) : self_(self) {}
    void onWrite(const NimBLECharacteristic* c) const;                           // older/common
  #ifdef HAS_NIMBLE_CONNINFO
    void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& info) override;     // newer
  #endif
  private:
    BleConfigService* self_;    
  };

  // inside class BleConfigService (private section)
  friend class ServerCallbacks;
  friend class CtrlCallbacks;
  friend class DataCallbacks;

  // Hold instances so their lifetime matches the service
  ServerCallbacks serverCallbacks_{this};
  CtrlCallbacks   ctrlCallbacks_{this};
  DataCallbacks   dataCallbacks_{this};
};

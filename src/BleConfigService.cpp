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

#include "BleConfigService.h"
#include <Arduino_JSON.h>
#include <EEPROM.h>
#include <utility>

/***** Constructor *****/
BleConfigService::BleConfigService(const char* sensorId) {
  if (std::strlen(sensorId) > 0) safeCopy(sensorId_, sizeof(sensorId_), sensorId); 
  else safeCopy(sensorId_, sizeof(sensorId_), "ESP32-SENSOR");
}

/***** Public API *****/
void BleConfigService::beginBle() {
  // If already initialized the just return
  if (NimBLEDevice::isInitialized()) return;

  // Init NimBLE
  NimBLEDevice::init(sensorId_);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  NimBLEDevice::setMTU(247);

  server_ = NimBLEDevice::createServer();
  static BleConfigService::ServerCallbacks serverCbs(this);
  server_->setCallbacks(&serverCbs);

  svc_ = server_->createService(SVC_UUID);

  ctrl_ = svc_->createCharacteristic(CTRL_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::READ);
  static BleConfigService::CtrlCallbacks ctrlCbs(this);
  ctrl_->setCallbacks(&ctrlCbs);

  data_ = svc_->createCharacteristic(DATA_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  static BleConfigService::DataCallbacks dataCbs(this);
  data_->setCallbacks(&dataCbs);

  status_ = svc_->createCharacteristic(STAT_UUID, NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ);
  status_->setValue("READY");

  svc_->start();

  adv_ = NimBLEDevice::getAdvertising();
  adv_->addServiceUUID(SVC_UUID);
  adv_->enableScanResponse(true);
  adv_->setMinInterval(0x06);
  adv_->setMaxInterval(0x12);
  adv_->start();
  //NimBLEDevice::startAdvertising();
}

void BleConfigService::stopBle() {
  NimBLEDevice::deinit(true);
}

void BleConfigService::refreshAdvertisedName() const {
  if (std::strlen(sensorId_) > 0) {
    NimBLEDevice::setDeviceName(sensorId_);
    NimBLEDevice::init(sensorId_); // refresh GAP name
    // Restart advertising to push new name
    NimBLEDevice::stopAdvertising();
    NimBLEDevice::startAdvertising();
  }
}

/***** Helpers *****/
void BleConfigService::safeCopy(char* dst, const std::size_t dstSize, const char* src) const {
  if (!dst || !dstSize) return;
  if (!src) { dst[0] = '\0'; return; }
  std::strncpy(dst, src, dstSize - 1);
  dst[dstSize - 1] = '\0';
}

bool BleConfigService::looksLikeIPv4(const char* s) {
  if (!s || !*s) return false;
  int dots = 0;
  int seg = 0;
  for (const char* p = s; *p; ++p) {
    if (*p == '.') {
      dots++;
      if (seg < 0 || seg > 255) return false;
      seg = 0;
      continue;
    }
    if (*p < '0' || *p > '9') return false;
    seg = seg * 10 + (*p - '0');
    if (seg > 255) return false;
  }
  return (dots == 3) && (seg >= 0 && seg <= 255);
}

void BleConfigService::notifyStatus(const char* msg) const {
  if (status_) {
    status_->setValue(reinterpret_cast<const uint8_t*>(msg), std::strlen(msg));
    // ReSharper disable once CppExpressionWithoutSideEffects
    status_->notify();
  }
}

bool BleConfigService::applyConfigFromJSON(const char* json, std::size_t len, String& err) {
  // 4096 is usually enough; raise if you send very large strings in multiple fields
  JSONVar jsonConfig = JSON.parse(json);

  const char* strLocalIp     = jsonConfig["localIp"].length()       > 0 ? jsonConfig["localIp"]       : "";
  const char* strSubnet      = jsonConfig["subnet"].length()        > 0 ? jsonConfig["subnet"]        : "";
  const char* strDns1Ip      = jsonConfig["dns1Ip"].length()        > 0 ? jsonConfig["dns1Ip"]        : "";
  const char* strDns2Ip      = jsonConfig["dns2Ip"].length()        > 0 ? jsonConfig["dns2Ip"]        : "";
  const char* strGatewayIp   = jsonConfig["gatewayIp"].length()     > 0 ? jsonConfig["gatewayIp"]     : "";
  const char* wifiSsid       = jsonConfig["wifiSsid"].length()      > 0 ? jsonConfig["wifiSsid"]      : "";
  const char* wifiPassword   = jsonConfig["wifiPassword"].length()  > 0 ? jsonConfig["wifiPassword"]  : "";
  const char* sensorId       = jsonConfig["sensorId"].length()      > 0 ? jsonConfig["sensorId"]      : "";
  const char* configName     = jsonConfig["configName"].length()    > 0 ? jsonConfig["configName"]    : "";
  const char* httpConfigURL  = jsonConfig["httpConfigURL"].length() > 0 ? jsonConfig["httpConfigURL"] : "";
  const char* mqttServer     = jsonConfig["mqttServer"].length()    > 0 ? jsonConfig["mqttServer"]    : "";
  const uint16_t    mqttPort   = jsonConfig["mqttPort"].length()       > 0    ? static_cast<uint16_t>(jsonConfig["mqttPort"]) : 1883;
  const char* mqttUsername   = jsonConfig["mqttUsername"].length()  > 0 ? jsonConfig["mqttUsername"]  : "";
  const char* mqttPassword   = jsonConfig["mqttPassword"].length()  > 0 ? jsonConfig["mqttPassword"]  : "";
  const char* mqttTopic      = jsonConfig["mqttTopic"].length()     > 0 ? jsonConfig["mqttTopic"]     : "/";
  const char* caCertificate  = jsonConfig["caCertificate"].length() > 0 ? jsonConfig["caCertificate"] : "";

  // quick validations
  auto tooLongTooShort = [&](const char* v, std::size_t cap, const char* name)->bool {
    if (std::strlen(v) >= cap) { err = String(name) + " too long"; return true; }
    if (std::strcmp(v , "") == 0) { err = String(name) + " is not set"; return true; }
    return false;
  };

  if (std::strcmp(strLocalIp, "") != 0) {  // If there is an IP address we need all the other IP params
    if (!looksLikeIPv4(strLocalIp))   { err = "strLocalIp invalid IPv4"; return false; }
    if (!looksLikeIPv4(strSubnet))    { err = "strSubnet invalid IPv4"; return false; }
    if (!looksLikeIPv4(strDns1Ip))    { err = "strDns1Ip invalid IPv4"; return false; }
    if (!looksLikeIPv4(strDns2Ip))    { err = "strDns2Ip invalid IPv4"; return false; }
    if (!looksLikeIPv4(strGatewayIp)) { err = "strGatewayIp invalid IPv4"; return false; }
  }

  if (tooLongTooShort(wifiSsid,       sizeof(wifiSsid_),       "wifiSsid"))       return false;
  if (tooLongTooShort(wifiPassword,   sizeof(wifiPassword_),   "wifiPassword"))   return false;
  if (tooLongTooShort(sensorId,       sizeof(sensorId_),       "sensorId"))       return false;
  if (tooLongTooShort(configName,     sizeof(configName_),     "configName"))     return false;
  if (tooLongTooShort(httpConfigURL,  sizeof(httpConfigUrl_),  "httpConfigURL"))  return false;
  if (tooLongTooShort(mqttServer,     sizeof(mqttServer_),     "mqttServer"))     return false;
  if (tooLongTooShort(mqttUsername,   sizeof(mqttUsername_),   "mqttUsername"))   return false;
  if (tooLongTooShort(mqttPassword,   sizeof(mqttPassword_),   "mqttPassword"))   return false;
  if (tooLongTooShort(mqttTopic,      sizeof(mqttTopic_),      "mqttTopic"))      return false;
  if (tooLongTooShort(caCertificate,  sizeof(caCertificate_),  "caCertificate"))  return false;

  // apply
  safeCopy(strLocalIp_,     sizeof(strLocalIp_),     strLocalIp);
  safeCopy(strSubnet_,      sizeof(strSubnet_),      strSubnet);
  safeCopy(strDns1Ip_,      sizeof(strDns1Ip_),      strDns1Ip);
  safeCopy(strDns2Ip_,      sizeof(strDns2Ip_),      strDns2Ip);
  safeCopy(strGatewayIp_,   sizeof(strGatewayIp_),   strGatewayIp);
  safeCopy(wifiSsid_,       sizeof(wifiSsid_),       wifiSsid);
  safeCopy(wifiPassword_,   sizeof(wifiPassword_),   wifiPassword);
  safeCopy(sensorId_,       sizeof(sensorId_),       sensorId);
  safeCopy(configName_,     sizeof(configName_),     configName);
  safeCopy(httpConfigUrl_,  sizeof(httpConfigUrl_),  httpConfigURL);
  safeCopy(mqttServer_,     sizeof(mqttServer_),     mqttServer);
  mqttPort_ = mqttPort;
  safeCopy(mqttUsername_,   sizeof(mqttUsername_),   mqttUsername);
  safeCopy(mqttPassword_,   sizeof(mqttPassword_),   mqttPassword);
  safeCopy(mqttTopic_,      sizeof(mqttTopic_),      mqttTopic);
  safeCopy(caCertificate_,  sizeof(caCertificate_),  caCertificate);

  // Optionally refresh advertised name to include sensorId
  refreshAdvertisedName();
  return true;
}

/***** Server callbacks *****/
void BleConfigService::ServerCallbacks::onConnect(NimBLEServer* pServer) const {
  self_->notifyStatus("CONNECTED");
}
void BleConfigService::ServerCallbacks::onConnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) const {
  onConnect(pServer);
}
void BleConfigService::ServerCallbacks::onDisconnect(NimBLEServer* pServer) const {
  self_->collecting_ = false;
  self_->expectedLen_ = 0;
  self_->rxBuffer_.clear();
  self_->notifyStatus("DISCONNECTED");
  NimBLEDevice::startAdvertising();
}
#ifdef HAS_NIMBLE_CONNINFO
void BleConfigService::ServerCallbacks::onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo)
{
  onConnect(pServer);
}
void BleConfigService::ServerCallbacks::onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason)
{
  onDisconnect(pServer);
}
#endif

/***** Ctrl characteristic *****/
void BleConfigService::CtrlCallbacks::onWrite(const NimBLECharacteristic* c) const {
  const std::string v = c->getValue();
  if (v.empty()) return;

  if (v.starts_with("BEGIN ")) {
    const std::size_t len = std::strtoul(v.c_str() + 6, nullptr, 10);  // NOLINT(clang-diagnostic-shorten-64-to-32)
    if (len == 0 || len > 100 * 1024) { self_->notifyStatus("ERR:BAD_LEN"); return; }
    self_->collecting_ = true;
    self_->expectedLen_ = len;
    self_->rxBuffer_.clear();
    self_->rxBuffer_.reserve(len);
    self_->notifyStatus("OK:BEGIN");
    return;
  }

  if (v == "ABORT") {
    self_->collecting_ = false;
    self_->expectedLen_ = 0;
    self_->rxBuffer_.clear();
    self_->notifyStatus("OK:ABORT");
    return;
  }

  if (v == "COMMIT") {
    if (!self_->collecting_) { self_->notifyStatus("ERR:NO_BEGIN"); return; }
    if (self_->rxBuffer_.size() != self_->expectedLen_) {
      self_->notifyStatus("ERR:LEN_MISMATCH"); return;
    }
    if (String err; !self_->applyConfigFromJSON(self_->rxBuffer_.c_str(), self_->rxBuffer_.size(), err)) {
      const String msg = "ERR:" + err;
      self_->notifyStatus(msg.c_str());
      return;
    }
    self_->saveProperties();
    self_->collecting_ = false;
    self_->expectedLen_ = 0;
    self_->rxBuffer_.clear();
    self_->notifyStatus("OK:COMMIT");
    return;
  }

  if (v == "SHOW") {
    char msg[513];
    if (snprintf(msg, sizeof(msg), "SSID=%s MQTT=%s:%u Topic=%s", self_->wifiSsid_, self_->mqttServer_, self_->mqttPort_, self_->mqttTopic_) < 0) {
      Serial.println("ERROR: Cannot create MQTT info string.");
      return;
    }
    self_->notifyStatus(msg);
    return;
  }

  self_->notifyStatus("ERR:UNKNOWN_CMD");
}

#ifdef HAS_NIMBLE_CONNINFO
void BleConfigService::CtrlCallbacks::onWrite(NimBLECharacteristic* c, NimBLEConnInfo& info) {
    onWrite(c);
}
#endif

/***** Data characteristic *****/
void BleConfigService::DataCallbacks::onWrite(const NimBLECharacteristic* c) const {
  if (!self_->collecting_) { self_->notifyStatus("ERR:WRITE_NO_BEGIN"); return; }
  std::string chunk = c->getValue();
  if (chunk.empty()) return;

  if (self_->rxBuffer_.size() + chunk.size() > self_->expectedLen_) {
    self_->notifyStatus("ERR:TOO_MUCH_DATA");
    self_->collecting_ = false;
    self_->rxBuffer_.clear();
    self_->expectedLen_ = 0;
    return;
  }

  self_->rxBuffer_.append(chunk);

  // Optional progress ping roughly every 1KB
  if ((self_->rxBuffer_.size() % 1024) < chunk.size()) {
    char msg[40];
    if (snprintf(msg, sizeof(msg), "PROG:%u/%u", self_->rxBuffer_.size(), self_->expectedLen_) < 0) {
      Serial.println("ERROR: Cannot create info string.");
      return;
    }
    self_->notifyStatus(msg);
  }
}

#ifdef HAS_NIMBLE_CONNINFO
void BleConfigService::DataCallbacks::onWrite(NimBLECharacteristic* c, NimBLEConnInfo& info) {
    onWrite(c);
}
#endif

void BleConfigService::clearEEPROM() {
  Serial.println("DEBUG: Clearing EEPROM");
  Serial.print("       EEPROM_DATASIZE = "); Serial.println(EEPROM_DATASIZE);
  Serial.print("       EEPROM_MAXSIZE  = "); Serial.println(EEPROM.length());
  for (int i = 0; std::cmp_less(i, EEPROM_DATASIZE); i++) {
    EEPROM.write(i, 0xFF);
  }
}

uint32_t BleConfigService::getEepromWrites() const { return eepromWrites_; }
Ethernet_Properties_t BleConfigService::getEthernetProperties() const { return ethProps_; }
IPAddress BleConfigService::getLocalIp() const {
  return {ethProps_.localIp[0], ethProps_.localIp[1], ethProps_.localIp[2], ethProps_.localIp[3]};
}
IPAddress BleConfigService::getSubnet() const {
  return {ethProps_.subnet[0], ethProps_.subnet[1], ethProps_.subnet[2], ethProps_.subnet[3]};
}
IPAddress BleConfigService::getDns1Ip() const {
  return {ethProps_.dns1Ip[0], ethProps_.dns1Ip[1], ethProps_.dns1Ip[2], ethProps_.dns1Ip[3]};
}
IPAddress BleConfigService::getDns2Ip() const {
  return {ethProps_.dns2Ip[0], ethProps_.dns2Ip[1], ethProps_.dns2Ip[2], ethProps_.dns2Ip[3]};
}
IPAddress BleConfigService::getGatewayIp() const {
  return {ethProps_.gatewayIp[0], ethProps_.gatewayIp[1], ethProps_.gatewayIp[2], ethProps_.gatewayIp[3]};
}
String BleConfigService::getStrLocalIp()     { return {strLocalIp_}; }
String BleConfigService::getStrSubnet()      { return {strSubnet_}; }
String BleConfigService::getStrDns1Ip()      { return {strDns1Ip_}; }
String BleConfigService::getStrDns2Ip()      { return {strDns2Ip_}; }
String BleConfigService::getStrGatewayIp()   { return {strGatewayIp_}; }
String BleConfigService::getWifiSsid()       { return {wifiSsid_}; }
String BleConfigService::getWifiPassword()   { return {wifiPassword_}; }
String BleConfigService::getSensorId()       { return {sensorId_}; }
String BleConfigService::getConfigName()     { return {configName_}; }
String BleConfigService::getHttpConfigUrl()  { return {httpConfigUrl_}; }
String BleConfigService::getMqttServer()     { return {mqttServer_}; }
uint16_t BleConfigService::getMqttPort() const { return mqttPort_; }
String BleConfigService::getMqttUsername()   { return {mqttUsername_}; }
String BleConfigService::getMqttPassword()   { return {mqttPassword_}; }
String BleConfigService::getMqttTopic()      { return {mqttTopic_}; }
String BleConfigService::getCaCertificate()  { return {caCertificate_}; }

void BleConfigService::setEthernetProperties(const Ethernet_Properties_t& ethernetProperties) { ethProps_ = ethernetProperties; }
void BleConfigService::setWifiSsid(const char* s) { safeCopy(wifiSsid_, sizeof(wifiSsid_), s); }
void BleConfigService::setWifiPassword(const char* s) { safeCopy(wifiPassword_, sizeof(wifiPassword_), s); }
void BleConfigService::setSensorId(const char* s) { safeCopy(sensorId_, sizeof(sensorId_), s); }
void BleConfigService::setConfigName(const char* s) { safeCopy(configName_, sizeof(configName_), s); }
void BleConfigService::setHttpConfigURL(const char* s) { safeCopy(httpConfigUrl_, sizeof(httpConfigUrl_), s); }
void BleConfigService::setMqttServer(const char* s) { safeCopy(mqttServer_, sizeof(mqttServer_), s); }
void BleConfigService::setMqttPort(uint16_t p) { mqttPort_ = p; }
void BleConfigService::setMqttUsername(const char* s) { safeCopy(mqttUsername_, sizeof(mqttUsername_), s); }
void BleConfigService::setMqttPassword(const char* s) { safeCopy(mqttPassword_, sizeof(mqttPassword_), s); }
void BleConfigService::setMqttTopic(const char* s) { safeCopy(mqttTopic_, sizeof(mqttTopic_), s); }
void BleConfigService::setCaCertificate(const char* s) { safeCopy(caCertificate_, sizeof(caCertificate_), s); }
bool BleConfigService::isDhcp() const { return isDhcp_; };
bool BleConfigService::isSavedData() const { return savedData_ == 0x01; };

void BleConfigService::setEthernetProperties(IPAddress localIp, IPAddress subnet, IPAddress dns1Ip, IPAddress dns2Ip, IPAddress gatewayIp) {
  ethProps_.localIp = { localIp[0], localIp[1], localIp[2], localIp[3] };
  ethProps_.subnet = { subnet[0], subnet[1], subnet[2], subnet[3] };
  ethProps_.dns1Ip = { dns1Ip[0], dns1Ip[1], dns1Ip[2], dns1Ip[3] };
  ethProps_.dns2Ip = { dns2Ip[0], dns2Ip[1], dns2Ip[2], dns2Ip[3] };
  ethProps_.gatewayIp = { gatewayIp[0], gatewayIp[1], gatewayIp[2], gatewayIp[3] };
}

// Returns false if any of the strings are not valid IP addresses.
bool BleConfigService::setEthernetProperties(const char* strLocalIp, const char* strSubnet, const char* strDns1Ip, const char* strDns2Ip, const char* strGatewayIp) {
  IPAddress localIp; 
  IPAddress subnet;
  IPAddress dns1Ip; 
  IPAddress dns2Ip; 
  IPAddress gatewayIp;
  if (!localIp.fromString(strLocalIp)) return false;
  if (!subnet.fromString(strSubnet)) return false;
  if (!dns1Ip.fromString(strDns1Ip)) return false;
  if (!dns2Ip.fromString(strDns2Ip)) return false;
  if (!gatewayIp.fromString(strGatewayIp)) return false;

  setEthernetProperties(localIp, subnet, dns1Ip, dns2Ip, gatewayIp);
  return true;
}

bool BleConfigService::setLocalIp(const char* strLocalIp) {
  IPAddress localIp;
  if (!localIp.fromString(strLocalIp)) return false; 
  ethProps_.localIp = { localIp[0], localIp[1], localIp[2], localIp[3] };
  safeCopy(strLocalIp_, sizeof(strLocalIp_), strLocalIp);
  return true;
}

bool BleConfigService::setSubnet(const char* strSubnet) {
  IPAddress subnet;
  if (!subnet.fromString(strSubnet)) return false;
  ethProps_.subnet = { subnet[0], subnet[1], subnet[2], subnet[3] };
  safeCopy(strSubnet_, sizeof(strSubnet_), strSubnet);
  return true;
}

bool BleConfigService::setDns1Ip(const char* strDns1Ip) {
  IPAddress dns1Ip;
  if (!dns1Ip.fromString(strDns1Ip)) return false;
  ethProps_.dns1Ip = { dns1Ip[0], dns1Ip[1], dns1Ip[2], dns1Ip[3] };
  safeCopy(strDns1Ip_, sizeof(strDns1Ip_), strDns1Ip);  
  return true;
}

bool BleConfigService::setDns2Ip(const char* strDns2Ip) {
  IPAddress dns2Ip;
  if (!dns2Ip.fromString(strDns2Ip)) return false;
  ethProps_.dns2Ip = { dns2Ip[0], dns2Ip[1], dns2Ip[2], dns2Ip[3] };
  safeCopy(strDns2Ip_, sizeof(strDns2Ip_), strDns2Ip);  
  return true;
}

bool BleConfigService::setGatewayIp(const char* strGatewayIp) {
  IPAddress gatewayIp;
  if (!gatewayIp.fromString(strGatewayIp)) return false;
  ethProps_.gatewayIp = { gatewayIp[0], gatewayIp[1], gatewayIp[2], gatewayIp[3] };
  safeCopy(strGatewayIp_, sizeof(strGatewayIp_), strGatewayIp);
  return true;
}

bool BleConfigService::saveProperties() { 
  if (savedData_ != 0x01) {
    // This is the first time this application has run, and we need set the "saved data" indicator bit in the EEPROM and same the default properties.
    savedData_ = 0x01;  
    EEPROM.put(EERPOM_SAVED_ADDR, savedData_);
  }

  eepromWrites_++;
  EEPROM.put(EERPOM_WRITES_ADDR, eepromWrites_);
  EEPROM.put(EEPROM_ETHERNET_ADDR, (const Ethernet_Properties_t)ethProps_);
  EEPROM.put(EEPROM_WIFISSID_ADDR, wifiSsid_);
  EEPROM.put(EEPROM_WIFIPASS_ADDR, wifiPassword_);
  EEPROM.put(EEPROM_SENSORID_ADDR, sensorId_);
  EEPROM.put(EEPROM_CONFIGNAME_ADDR, configName_);
  EEPROM.put(EEPROM_CONFIGURL_ADDR, httpConfigUrl_);
  EEPROM.put(EEPROM_MQTTSERVER_ADDR, mqttServer_);
  EEPROM.put(EEPROM_MQTTPORT_ADDR, mqttPort_);
  EEPROM.put(EEPROM_MQTTUSER_ADDR, mqttUsername_);
  EEPROM.put(EEPROM_MQTTPASS_ADDR, mqttPassword_);
  EEPROM.put(EEPROM_MQTTTOPIC_ADDR, mqttTopic_);
  EEPROM.put(EEPROM_CACERT_ADDR, caCertificate_);

  return true;
}

bool BleConfigService::loadProperties() {
  EEPROM.get(EERPOM_SAVED_ADDR, savedData_);
  if (savedData_ != 0x01) {
    // This is the first time this application has run, so don't get the values.  
    return false;
  }

  EEPROM.get(EERPOM_WRITES_ADDR, eepromWrites_);
  EEPROM.get(EEPROM_ETHERNET_ADDR, ethProps_);
  // If the IP value is 0 then use DHCP.
  if (ethProps_.localIp[0] == 0x00) isDhcp_ = true;
  generateAllIpStrings();
  EEPROM.get(EEPROM_WIFISSID_ADDR, wifiSsid_);
  EEPROM.get(EEPROM_WIFIPASS_ADDR, wifiPassword_);
  EEPROM.get(EEPROM_SENSORID_ADDR, sensorId_);
  EEPROM.get(EEPROM_CONFIGNAME_ADDR, configName_);
  EEPROM.get(EEPROM_CONFIGURL_ADDR, httpConfigUrl_);
  EEPROM.get(EEPROM_MQTTSERVER_ADDR, mqttServer_);
  EEPROM.get(EEPROM_MQTTPORT_ADDR, mqttPort_);
  EEPROM.get(EEPROM_MQTTUSER_ADDR, mqttUsername_);
  EEPROM.get(EEPROM_MQTTPASS_ADDR, mqttPassword_);
  EEPROM.get(EEPROM_MQTTTOPIC_ADDR, mqttTopic_);
  EEPROM.get(EEPROM_CACERT_ADDR, caCertificate_);

  return true;
}

String BleConfigService::generateIpString(IPAddress ipAddress) {
  char strIpAddress[16] = "";
  if (sprintf(strIpAddress, "%u.%u.%u.%u", ipAddress[0], ipAddress[1], ipAddress[2], ipAddress[3]) < 0) {
    Serial.println("ERROR: Cannot convert IP address to string.");
    return "";
  }
  return {strIpAddress};
}

void BleConfigService::generateAllIpStrings() {
  if (sprintf(strLocalIp_, "%u.%u.%u.%u", ethProps_.localIp[0], ethProps_.localIp[1], ethProps_.localIp[2], ethProps_.localIp[3]) < 0) {
    Serial.println("ERROR: Cannot convert IP address to string.");
  }
  if (sprintf(strSubnet_, "%u.%u.%u.%u", ethProps_.subnet[0], ethProps_.subnet[1], ethProps_.subnet[2], ethProps_.subnet[3]) < 0) {
    Serial.println("ERROR: Cannot convert IP address to string.");
  }
  if (sprintf(strDns1Ip_, "%u.%u.%u.%u", ethProps_.dns1Ip[0], ethProps_.dns1Ip[1], ethProps_.dns1Ip[2], ethProps_.dns1Ip[3]) < 0) {
    Serial.println("ERROR: Cannot convert IP address to string.");
  }
  if (sprintf(strDns2Ip_, "%u.%u.%u.%u", ethProps_.dns2Ip[0], ethProps_.dns2Ip[1], ethProps_.dns2Ip[2], ethProps_.dns2Ip[3]) < 0) {
    Serial.println("ERROR: Cannot convert IP address to string.");
  }
  if (sprintf(strGatewayIp_, "%u.%u.%u.%u", ethProps_.gatewayIp[0], ethProps_.gatewayIp[1], ethProps_.gatewayIp[2], ethProps_.gatewayIp[3]) < 0) {
    Serial.println("ERROR: Cannot convert IP address to string.");
  }
}
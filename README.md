# EnvDataMqtt_ESP

## PURPOSE
This project provides firmware for the **Seeed XIAO ESP32-C3**, enabling it to collect environmental telemetry and publish the data securely to an MQTT broker. Network and MQTT configuration parameters can be updated **over Bluetooth Low Energy (BLE)** without reflashing the firmware, making it practical for headless IoT sensors and field deployments.

## DESCRIPTION
The core application (`EnvDataMqtt_ESP.ino`) manages Wi-Fi connectivity, MQTT session handling, and telemetry publishing. A dedicated configuration module (`BleConfigService.h/.cpp`) uses the NimBLE stack to expose a BLE service where settings such as Wi-Fi SSID/password, MQTT broker address/port, topic, username/password, and root CA certificate can be provisioned. Configuration is sent as a JSON payload in chunks, committed with a control command, and persisted in EEPROM, ensuring reliable operation across reboots.  

On startup, the device loads saved settings, connects to the configured Wi-Fi network, and begins publishing sensor data to the MQTT broker. If needed, settings can be reset or updated through any BLE-capable client (e.g., nRF Connect on a smartphone or computer). The firmware includes input validation, safe fixed-size storage, and connection recovery mechanisms, providing a secure and flexible workflow for deploying IoT devices that can be reconfigured on-site without code changes.

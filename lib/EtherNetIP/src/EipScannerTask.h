// Background EtherNet/IP scanner task (ESP-IDF only, Kconfig-gated).
//
// Retries TCP connect with exponential backoff; does NOT reference
// MqttBridge::EthernetLink. Uses the default netif established by MQTT bring-up.

#ifndef ETHERNET_IP_EIP_SCANNER_TASK_H
#define ETHERNET_IP_EIP_SCANNER_TASK_H

namespace eip {

// Start the scanner FreeRTOS task. No-op when CONFIG_EIP_SCANNER_ENABLED is off.
void startScannerTask();

}  // namespace eip

#endif  // ETHERNET_IP_EIP_SCANNER_TASK_H

// Background EtherNet/IP scanner task (ESP-IDF only, Kconfig-gated).
//
// Waits for ILinkStatus (W5500 PHY link up), then runs Class 1 lifecycle.
// X and/or Z share one UDP 2222 via EipMultiScanner (demux by connection ID).
// Theta (HCS01) uses the single-axis EipScanner path.
// On SENDOK / connect failure, hard-resets the W5500 chip before retrying;
// after repeated recover failures, esp_restart().

#ifndef ETHERNET_IP_EIP_SCANNER_TASK_H
#define ETHERNET_IP_EIP_SCANNER_TASK_H

#include "EipTransport.h"

class W5500;

namespace w5500 { class W5500Hal; }

namespace eip {

class EipProcessImage;

// Start the scanner FreeRTOS task. No-op when CONFIG_EIP_SCANNER_ENABLED is off.
// Pass nullptr for an unused axis image. `chip` is used for runtime recover().
void startScannerTask(W5500& chip, w5500::W5500Hal& hal, ILinkStatus& link,
                      EipProcessImage* image_x = nullptr,
                      EipProcessImage* image_z = nullptr,
                      EipProcessImage* image_theta = nullptr);

// Dump Class 1 latency ring stats (exchange / O->T / cycle / cmd-to-StartMotion).
void dumpClass1TimingStats();

}  // namespace eip

#endif  // ETHERNET_IP_EIP_SCANNER_TASK_H

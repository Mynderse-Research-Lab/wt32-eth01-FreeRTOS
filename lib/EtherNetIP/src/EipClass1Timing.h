// Host-testable Class 1 cycle helpers (no FreeRTOS / ESP-IDF).

#ifndef ETHERNET_IP_EIP_CLASS1_TIMING_H
#define ETHERNET_IP_EIP_CLASS1_TIMING_H

#include <cstdint>

namespace eip {

// Delay after exchangeOnce so the cycle period tracks granted RPI instead of
// stretching toward ~2x RPI when send+recv already consumed wall time.
inline uint32_t rpiRemainderMs(uint32_t rpi_ms, uint32_t elapsed_ms) {
  if (rpi_ms == 0) return 1;
  if (elapsed_ms >= rpi_ms) return 0;
  return rpi_ms - elapsed_ms;
}

// Soft-miss policy: allow a few T->O misses while O->T keeps running before
// tearing down dual Class 1 (avoids cascading E602 on a single dropped UDP).
inline bool shouldTeardownAfterInputMisses(uint32_t consecutive_misses,
                                           uint32_t max_misses = 3) {
  return consecutive_misses >= max_misses;
}

enum class ExchangeStatus : uint8_t {
  kOk = 0,
  kOutputSendFailed = 1,  // O->T / SENDOK — escalate to chip recover
  kInputMiss = 2,         // T->O partial/timeout — soft-retry at task layer
};

}  // namespace eip

#endif  // ETHERNET_IP_EIP_CLASS1_TIMING_H

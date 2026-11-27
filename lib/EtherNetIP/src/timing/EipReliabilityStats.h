// Runtime reliability counters for Class 1 / W5500 (host-testable).
#ifndef ETHERNET_IP_EIP_RELIABILITY_STATS_H
#define ETHERNET_IP_EIP_RELIABILITY_STATS_H

#include <cstdint>

namespace eip {

struct ReliabilitySnapshot {
  uint32_t soft_miss = 0;
  uint32_t sendok_fail = 0;
  uint32_t chip_recover = 0;
  uint32_t reconnect = 0;
};

class ReliabilityStats {
 public:
  void reset() { snap_ = {}; }
  void noteSoftMiss() { ++snap_.soft_miss; }
  void noteSendOkFail() { ++snap_.sendok_fail; }
  void noteChipRecover() { ++snap_.chip_recover; }
  void noteReconnect() { ++snap_.reconnect; }
  ReliabilitySnapshot snapshot() const { return snap_; }

 private:
  ReliabilitySnapshot snap_{};
};

ReliabilityStats& reliabilityStats();

}  // namespace eip

#endif  // ETHERNET_IP_EIP_RELIABILITY_STATS_H

/**
 * @file EipClass1TimingStats.h
 * @brief Ring-buffer Class 1 latency samples (host-testable, no ESP-IDF).
 *
 * Captures exchange / O->T send / T->O drain / cycle / cmd-to-StartMotion wall
 * times for the L1+L2+L3 < 500 us go/no-go gate.
 */

#ifndef ETHERNET_IP_EIP_CLASS1_TIMING_STATS_H
#define ETHERNET_IP_EIP_CLASS1_TIMING_STATS_H

#include <cstddef>
#include <cstdint>

namespace eip {

inline constexpr uint32_t kClass1LatencyTargetUs = 500;
inline constexpr size_t kClass1TimingRingSize = 128;

struct Class1TimingSnapshot {
  uint32_t count = 0;
  uint32_t min_us = 0;
  uint32_t max_us = 0;
  uint32_t p50_us = 0;
  uint32_t p99_us = 0;
};

class Class1TimingStats {
 public:
  void reset();

  void recordExchangeUs(uint32_t us);
  void recordOtSendUs(uint32_t us);
  void recordToDrainUs(uint32_t us);
  void recordCycleUs(uint32_t us);
  void recordCmdToStartUs(uint32_t us);

  // Pacing health: an overrun means xTaskDelayUntil did not block, so the
  // cycle was not on the RPI grid. Both must stay near zero for a
  // deterministic cadence.
  void notePaceOverrun() { ++pace_overrun_; }
  void notePaceYield() { ++pace_yield_; }
  uint32_t paceOverrunCount() const { return pace_overrun_; }
  uint32_t paceYieldCount() const { return pace_yield_; }

  // Absolute fast-path: axis notes wall time when StartMotion is published.
  void noteAbsoluteStartMotionPublished(int64_t now_us);
  // Scanner notes when an O->T assembly carries StartMotion; closes the span.
  void noteOtAssemblySent(const uint8_t* assembly, size_t len, int64_t now_us);

  Class1TimingSnapshot exchange() const;
  Class1TimingSnapshot otSend() const;
  Class1TimingSnapshot toDrain() const;
  Class1TimingSnapshot cycle() const;
  Class1TimingSnapshot cmdToStart() const;

  uint32_t sampleCount() const { return exchange_count_; }

 private:
  struct Ring {
    uint32_t samples[kClass1TimingRingSize] = {};
    size_t next = 0;
    size_t filled = 0;
    uint32_t total = 0;

    void push(uint32_t us);
    Class1TimingSnapshot snapshot() const;
  };

  Ring exchange_;
  Ring ot_send_;
  Ring to_drain_;
  Ring cycle_;
  Ring cmd_to_start_;
  uint32_t exchange_count_ = 0;
  uint32_t pace_overrun_ = 0;
  uint32_t pace_yield_ = 0;

  bool start_pending_ = false;
  int64_t start_published_us_ = 0;
};

Class1TimingStats& class1TimingStats();

// Monotonic microseconds for firmware (esp_timer) or host (steady_clock).
int64_t class1NowUs();

}  // namespace eip

#endif  // ETHERNET_IP_EIP_CLASS1_TIMING_STATS_H

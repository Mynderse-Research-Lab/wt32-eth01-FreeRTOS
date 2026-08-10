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

// Microsecond remainder for esp_timer-paced Class 1 (preferred over ms ticks).
inline uint32_t rpiRemainderUs(uint32_t rpi_us, uint32_t elapsed_us) {
  if (rpi_us == 0) return 1000;
  if (elapsed_us >= rpi_us) return 0;
  return rpi_us - elapsed_us;
}

// Whole FreeRTOS ticks per Class 1 period (minimum 1). Used with
// xTaskDelayUntil so the scanner always blocks and IDLE can feed the TWDT.
inline uint32_t class1PaceTicks(uint32_t rpi_us, uint32_t tick_us = 1000) {
  if (tick_us == 0) tick_us = 1000;
  if (rpi_us == 0) return 1;
  const uint32_t ticks = rpi_us / tick_us;
  return (ticks == 0) ? 1u : ticks;
}

// Sub-tick remainder after class1PaceTicks. Zero when RPI is tick-aligned or
// when RPI was clamped up to one full tick (rpi_us < tick_us).
inline uint32_t class1RpiFractionUs(uint32_t rpi_us, uint32_t tick_us = 1000) {
  if (tick_us == 0) tick_us = 1000;
  if (rpi_us == 0 || rpi_us < tick_us) return 0;
  return rpi_us % tick_us;
}

// Soft-miss policy: allow a few T->O misses while O->T keeps running before
// tearing down dual Class 1 (avoids cascading E602 on a single dropped UDP).
inline bool shouldTeardownAfterInputMisses(uint32_t consecutive_misses,
                                           uint32_t max_misses = 3) {
  return consecutive_misses >= max_misses;
}

// Produce-then-drain policy: tear down when feedback age exceeds N * RPI
// (one RPI of phase lag is normal; more means the drive stopped answering).
inline bool shouldTeardownAfterStaleUs(uint32_t age_us, uint32_t rpi_us,
                                       uint32_t max_rpi_multiples = 3) {
  if (rpi_us == 0) return age_us > 0;
  const uint64_t limit =
      static_cast<uint64_t>(rpi_us) * static_cast<uint64_t>(max_rpi_multiples);
  return static_cast<uint64_t>(age_us) >= limit;
}

// Arm the cyclic stale gate only after Class 1 has been pumping for N RPIs
// and at least one O->T has been sent. FO→first-exchange gap must not count.
// Callers should reseed last_feedback when the gate first latches so the
// subsequent stale window starts *after* grace (not age-from-cyclic-start).
inline bool class1StaleGateArmed(uint32_t since_cyclic_start_us, uint32_t rpi_us,
                                 bool ot_sent_since_cyclic,
                                 uint32_t max_rpi_multiples = 3) {
  if (!ot_sent_since_cyclic) return false;
  if (rpi_us == 0) return since_cyclic_start_us > 0;
  const uint64_t limit =
      static_cast<uint64_t>(rpi_us) * static_cast<uint64_t>(max_rpi_multiples);
  return static_cast<uint64_t>(since_cyclic_start_us) >= limit;
}

/// Rate-limit soft-miss WARN logs. Returns true when a WARN should be emitted
/// (first miss or ≥ min_interval_ms since last WARN). Caller should ESP_LOGD
/// when this returns false.
inline bool shouldWarnSoftMiss(uint32_t& last_warn_ms, uint32_t now_ms,
                               uint32_t min_interval_ms = 1000) {
  if (last_warn_ms == 0 ||
      (now_ms - last_warn_ms) >= min_interval_ms) {
    last_warn_ms = (now_ms == 0) ? 1u : now_ms;
    return true;
  }
  return false;
}

enum class ExchangeStatus : uint8_t {
  kOk = 0,
  kOutputSendFailed = 1,  // O->T / SENDOK — escalate to chip recover
  kInputMiss = 2,         // T->O partial/timeout — soft-retry at task layer
};

}  // namespace eip

#endif  // ETHERNET_IP_EIP_CLASS1_TIMING_H

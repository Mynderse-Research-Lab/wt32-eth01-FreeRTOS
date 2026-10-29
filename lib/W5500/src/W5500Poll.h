// Host-testable µs-deadline busy poll with yield (no 1 ms sleep per spin).
#ifndef W5500_POLL_H
#define W5500_POLL_H

#include <cstdint>
#include <functional>

#ifdef ESP_PLATFORM
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#else
#include <chrono>
#include <thread>
#endif

namespace w5500 {

inline int64_t pollNowUs() {
#ifdef ESP_PLATFORM
  return esp_timer_get_time();
#else
  using clock = std::chrono::steady_clock;
  return std::chrono::duration_cast<std::chrono::microseconds>(
             clock::now().time_since_epoch())
      .count();
#endif
}

/// Above this timeout, yield between checks so long TCP waits do not starve
/// other tasks. At FreeRTOS 1000 Hz, taskYIELD() often costs ~1 ms — fatal for
/// Class 1 SENDOK / UDP recv (dual-axis O->T was ~3 ms from yields alone).
constexpr uint32_t kBusyPollYieldAboveUs = 50000;  // 50 ms

/// Spin-check until `check()` is true or `timeout_us` elapses.
/// Short waits (Class 1) spin tightly; long waits yield between checks.
/// Returns true if check succeeded before timeout.
inline bool busyPollUs(uint32_t timeout_us, const std::function<bool()>& check) {
  const int64_t deadline = pollNowUs() + static_cast<int64_t>(timeout_us);
  const bool allow_yield = timeout_us > kBusyPollYieldAboveUs;
  while (pollNowUs() < deadline) {
    if (check()) return true;
#ifdef ESP_PLATFORM
    if (allow_yield) {
      taskYIELD();
    }
#else
    if (allow_yield) {
      std::this_thread::sleep_for(std::chrono::microseconds(50));
    }
#endif
  }
  return check();
}

/// Always spin tightly (no yield). Use for Class 1 SENDOK / CR under SPI lock.
inline bool busyPollUsTight(uint32_t timeout_us,
                            const std::function<bool()>& check) {
  const int64_t deadline = pollNowUs() + static_cast<int64_t>(timeout_us);
  while (pollNowUs() < deadline) {
    if (check()) return true;
  }
  return check();
}

inline bool busyPollMs(uint32_t timeout_ms, const std::function<bool()>& check) {
  const uint32_t us =
      (timeout_ms > (UINT32_MAX / 1000u)) ? UINT32_MAX : (timeout_ms * 1000u);
  return busyPollUs(us, check);
}

/// Block the calling task for about `ms` (vTaskDelay on target, so tasks of
/// any priority can run; taskYIELD only lets equal/higher priority run).
inline void pollSleepMs(uint32_t ms) {
#ifdef ESP_PLATFORM
  const TickType_t ticks = pdMS_TO_TICKS(ms);
  vTaskDelay(ticks > 0 ? ticks : 1);
#else
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
#endif
}

/// Slow-wait poll: check, then sleep `interval_ms`, until `check()` is true or
/// `timeout_ms` of wall time elapses. `check` receives the real elapsed
/// milliseconds (for diagnostics). Returns true on success. Use for TCP
/// connect/ARP, never for Class 1 paths.
inline bool sleepPollMs(uint32_t timeout_ms, uint32_t interval_ms,
                        const std::function<bool(uint32_t elapsed_ms)>& check) {
  const int64_t start = pollNowUs();
  const int64_t deadline = start + static_cast<int64_t>(timeout_ms) * 1000;
  while (true) {
    const int64_t now = pollNowUs();
    const uint32_t elapsed_ms = static_cast<uint32_t>((now - start) / 1000);
    if (check(elapsed_ms)) return true;
    if (now >= deadline) return false;
    pollSleepMs(interval_ms);
  }
}

// TCP connect: Sn_SR poll interval (one FreeRTOS tick at 1000 Hz).
constexpr uint32_t kConnectPollIntervalMs = 1;

// SENDOK waits: UDP Class 1 must fail fast; TCP FO can wait longer.
constexpr uint32_t kUdpSendOkTimeoutUs = 2000;    // 2 ms tight (was 20 ms + yields)
// First UDP datagram to a new DIPR (3-axis Class 1 on one socket) may ARP.
// 2 ms was aborting SEND before SENDOK/TIMEOUT, then chip-recovering X/Z.
constexpr uint32_t kUdpDestChangeSendOkTimeoutUs = 10000;
constexpr uint32_t kTcpSendOkTimeoutUs = 50000;   // 50 ms
// Sn_CR RECV completes in microseconds. busyPollMs(100) allowed yield
// (timeout > kBusyPollYieldAboveUs) and cost ~1 ms per datagram at 1000 Hz.
constexpr uint32_t kUdpRecvCmdTimeoutUs = 2000;

}  // namespace w5500

#endif  // W5500_POLL_H

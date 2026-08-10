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

// SENDOK waits: UDP Class 1 must fail fast; TCP FO can wait longer.
constexpr uint32_t kUdpSendOkTimeoutUs = 2000;    // 2 ms tight (was 20 ms + yields)
constexpr uint32_t kTcpSendOkTimeoutUs = 500000;  // 500 ms

}  // namespace w5500

#endif  // W5500_POLL_H

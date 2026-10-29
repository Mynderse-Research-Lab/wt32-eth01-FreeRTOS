#pragma once

#include <stdint.h>

namespace Gantry {

// Default GantryUpdate period. Timeouts below are multiples of this 10 ms quantum
// so a 10 ms loop keeps the previous durations. A faster loop spends the same
// microseconds and does not shorten a home wait or a move timeout.
inline uint32_t& controlPeriodUs() {
  static uint32_t us = 10000;
  return us;
}

inline void setControlPeriodUs(uint32_t us) {
  if (us < 1000u) {
    us = 1000u;
  }
  if (us > 20000u) {
    us = 20000u;
  }
  controlPeriodUs() = us;
}

// `ticks` is the historical count at 100 Hz. 50 ticks is 500 ms.
inline uint32_t legacyTicksUs(uint32_t ticks) { return ticks * 10000u; }

inline void spendUs(uint32_t& budget) {
  const uint32_t step = controlPeriodUs();
  budget = budget > step ? budget - step : 0u;
}

inline void accrueUs(uint32_t& elapsed, uint32_t cap_us) {
  const uint32_t step = controlPeriodUs();
  if (elapsed >= cap_us) {
    return;
  }
  const uint32_t room = cap_us - elapsed;
  elapsed += step < room ? step : room;
}

}  // namespace Gantry

/**
 * @file GantryEipRotaryAxisInternal.h
 * @brief Phase timings and the ready-state label shared by the
 *        GantryEipRotaryAxis files.
 *
 * Private to this library; include it only from those files.
 */
#pragma once

#include "GantryControlPeriod.h"
#include "GantryRotaryPhases.h"
#include "GantryUtils.h"
#include "Hcs01ControlStatus.h"

#if defined(ESP_PLATFORM)
#include "esp_log.h"
#include "esp_timer.h"
#else
#include "esp_timer.h"
#define ESP_LOGW(tag, fmt, ...) ((void)0)
#define ESP_LOGI(tag, fmt, ...) ((void)0)
#endif

namespace Gantry {

namespace {
constexpr uint8_t kMovePreloadTicks = RotaryPhase::kPreloadTicks;
constexpr uint8_t kMoveReleaseHaltTicks = RotaryPhase::kReleaseHaltTicks;
constexpr uint8_t kMoveStartTicks = RotaryPhase::kStartAckTicks;
constexpr uint8_t kClearFaultTicks = 10;     // hold P-0-4077 bit5 (C0500)
constexpr uint16_t kWaitAfTicks = 100;       // 1 s @ 100 Hz
constexpr uint8_t kArrivalStableTicks = 3;   // ~30 ms @ 100 Hz at target
// Exact drive arrival matches HCS01 S-0-0057 (0.1000 deg) with reached=1.
constexpr float kArrivalEpsDeg = Constants::EIP_THETA_SETTLED_TOL_DEG;
// Stiction fallback: HCS01 in_standstill asserts even during slow 30 deg/s
// bring-up moves, so require truly stationary feedback (<= 0.015 deg/tick)
// for 50 ms inside 0.30 deg to break +290 ms S-0-0104=1.00 integrator stalls
// without prematurely halting slow calibration moves.
constexpr float kStictionArrivalEpsDeg = 0.30f;
constexpr float kStictionStepEpsDeg = 0.015f;
constexpr uint8_t kStictionStableTicks = 5;  // ~50 ms @ 100 Hz stationary

const char* readyLabel(const eip::hcs01::Hcs01StatusWord& st) {
  using eip::hcs01::ReadyForOperation;
  if (st.ready == ReadyForOperation::kInOperation) {
    return st.not_following_command ? "AH" : "AF";
  }
  if (st.ready == ReadyForOperation::kReadyAb) return "Ab";
  if (st.ready == ReadyForOperation::kReadyBb) return "bb";
  return "AS";
}
}  // namespace
}  // namespace Gantry

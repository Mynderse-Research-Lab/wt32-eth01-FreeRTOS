/**
 * @file GantryEipLinearAxisInternal.h
 * @brief Phase timings shared by the GantryEipLinearAxis files.
 *
 * Private to this library; include it only from those files.
 */
#pragma once

#include "GantryLinearPhases.h"

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
const char* kEipAxisTag = "EipAxis";

constexpr uint8_t kArmServoLowTicks = 4;
constexpr uint8_t kArmServoSettleTicks = 40;
constexpr uint8_t kHomePreloadTicks = 4;
constexpr uint8_t kHomeStartTicks = 5;
constexpr uint16_t kHomeWaitTicks = 50;   // 0.5 s @ 100 Hz
// Absolute PTP matches Home34: preload Position (SM=0), then StartMotion edge.
// Fast-path SM=1 in the same packet as a new Position was dropped by Kinetix
// (Z 130→30 sat until the 60 s timeout; retry after StopMotion ran).
constexpr uint8_t kMovePreloadTicks = LinearPhase::kPreloadTicks;
constexpr uint8_t kMoveStartMotionTicks = LinearPhase::kStartMotionTicks;
// A queued Absolute stays on the wire with StartMotion=0 at least this long
// (3 RPI) before its edge, for the same reason as kMovePreloadTicks.
constexpr int64_t kQueueToEdgeUs = 6000;
constexpr uint8_t kStopPulseTicks = 5;
constexpr uint8_t kStopStableTicks = 20;   // 200 ms @ 100 Hz
constexpr uint16_t kStopTimeoutTicks = 100;  // 1 s @ 100 Hz — outlasts a powered decel
constexpr int32_t kDefaultAccelRef = 1000;  // 100.0 RPM/s
// 0.8 RPM: must be below ~1.5 RPM @ 1 mm/s X creep or Stop "completes" while coasting.
constexpr int32_t kNearStopSpeedRef = 8;
constexpr double kStopStableMmPerTick = 0.005;
constexpr int8_t kHomingMethodDefineCurrent = 34;
constexpr double kArrivalBandMm = 0.5;
constexpr uint8_t kArrivalStableTicks = LinearPhase::kArrivalStableTicks;  // before finishMoveHold
constexpr uint16_t kMoveTimeoutTicks = 1500;  // 15 s @ 100 Hz
}  // namespace
}  // namespace Gantry

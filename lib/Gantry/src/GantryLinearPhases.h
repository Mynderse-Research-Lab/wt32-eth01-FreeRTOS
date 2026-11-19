/**
 * @file GantryLinearPhases.h
 * @brief Kinetix linear move phases, shared by the axis and the pick budget.
 *
 * An Absolute preloads Position with StartMotion=0, raises StartMotion, and is
 * done once it has held inside the arrival band. Ticks are 10 ms.
 */
#pragma once

#include <cstdint>

#include "GantryControlPeriod.h"

namespace Gantry {
namespace LinearPhase {

inline constexpr uint8_t kPreloadTicks = 2;
inline constexpr uint8_t kStartMotionTicks = 5;
inline constexpr uint8_t kArrivalStableTicks = 3;
/// From moveToMm() until the StartMotion edge.
inline constexpr float kPreloadS = static_cast<float>(kPreloadTicks) * kLegacyTickS;
/// Held in the arrival band before the axis reports idle.
inline constexpr float kArrivalSettleS =
    static_cast<float>(kArrivalStableTicks) * kLegacyTickS;

}  // namespace LinearPhase
}  // namespace Gantry

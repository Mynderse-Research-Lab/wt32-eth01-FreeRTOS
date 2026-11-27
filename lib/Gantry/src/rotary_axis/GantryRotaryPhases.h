/**
 * @file GantryRotaryPhases.h
 * @brief HCS01 theta move start phases, shared by the axis and the pick budget.
 *
 * A theta Absolute latches the target while in AH, releases halt and waits for
 * AF, then toggles the command bit and waits for its ack. Ticks are 10 ms.
 */
#pragma once

#include <cstdint>

#include "GantryControlPeriod.h"

namespace Gantry {
namespace RotaryPhase {

inline constexpr uint8_t kPreloadTicks = 4;
inline constexpr uint8_t kReleaseHaltTicks = 10;
inline constexpr uint8_t kStartAckTicks = 4;
inline constexpr float kTickS = kLegacyTickS;
/// Worst case from moveToDeg() until the drive runs the profile.
inline constexpr float kStartMaxS =
    static_cast<float>(kPreloadTicks + kReleaseHaltTicks + kStartAckTicks) * kTickS;

}  // namespace RotaryPhase
}  // namespace Gantry

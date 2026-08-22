/**
 * @file GantryEndEffectorPolicy.h
 * @brief Digital gripper level and pin-domain routing (host-testable).
 */
#pragma once

#include "GantryLimitSwitchPolicy.h"

#include <cstdint>

namespace Gantry {
namespace EndEffectorPolicy {

inline uint8_t outputLevel(bool active, bool active_high) {
    return (active == active_high) ? 1u : 0u;
}

enum class PinDomain { kInvalid, kMcp, kDirectGpio };

inline PinDomain classifyPin(int pin) {
    if (LimitPolicy::isMcpLogicalPin(pin)) {
        return PinDomain::kMcp;
    }
    if (LimitPolicy::resolveDirectGpioPin(pin) >= 0) {
        return PinDomain::kDirectGpio;
    }
    return PinDomain::kInvalid;
}

}  // namespace EndEffectorPolicy
}  // namespace Gantry

/**
 * @file GantryLimitSwitchPolicy.h
 * @brief Pin-encoding and N-sample debounce for GantryLimitSwitch (host-testable).
 */
#pragma once

#include <cstdint>

#ifndef GPIO_DIRECT_PIN_BASE
#define GPIO_DIRECT_PIN_BASE 0x10
#endif
#ifndef GPIO_EXPANDER_DIRECT_FLAG
#define GPIO_EXPANDER_DIRECT_FLAG 0x100
#endif
#ifndef GPIO_EXPANDER_DIRECT_MASK
#define GPIO_EXPANDER_DIRECT_MASK 0x0FF
#endif

namespace Gantry {
namespace LimitPolicy {

inline bool isEncodedDirectPin(int pin) {
    return (pin & GPIO_EXPANDER_DIRECT_FLAG) != 0;
}

inline bool isMcpLogicalPin(int pin) {
    return pin >= 0 && pin < GPIO_DIRECT_PIN_BASE && !isEncodedDirectPin(pin);
}

inline int resolveDirectGpioPin(int pin) {
    if (pin < 0) {
        return -1;
    }
    if (isEncodedDirectPin(pin)) {
        return pin & GPIO_EXPANDER_DIRECT_MASK;
    }
    if (pin >= GPIO_DIRECT_PIN_BASE) {
        return pin;
    }
    return -1;
}

inline uint8_t coerceDebounceCycles(uint8_t n) { return (n == 0) ? 1 : n; }

inline bool rawLevelIsActive(uint8_t level, bool active_low) {
    return active_low ? (level == 0) : (level != 0);
}

struct Debounce {
    bool sample_state = false;
    bool stable_state = false;
    uint8_t stable_count = 0;
};

inline void forceSample(Debounce& d, bool raw_active, uint8_t cycles) {
    d.sample_state = raw_active;
    d.stable_state = raw_active;
    d.stable_count = cycles;
}

inline void debounceSample(Debounce& d, bool raw_active, uint8_t cycles) {
    if (raw_active == d.sample_state) {
        if (d.stable_count < cycles) {
            d.stable_count++;
        }
    } else {
        d.sample_state = raw_active;
        d.stable_count = 1;
    }
    if (d.stable_count >= cycles) {
        d.stable_state = d.sample_state;
    }
}

}  // namespace LimitPolicy
}  // namespace Gantry

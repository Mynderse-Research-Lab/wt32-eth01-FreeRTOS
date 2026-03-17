/**
 * @file GantryEndEffector.cpp
 * @brief End-effector implementation
 * @version 1.0.0
 */

#include "GantryEndEffector.h"
#include "gpio_expander.h"

namespace Gantry {

namespace {
bool isEncodedDirectPin(int pin) {
    return (pin & GPIO_EXPANDER_DIRECT_FLAG) != 0;
}

bool isMcpLogicalPin(int pin) {
    return pin >= 0 && pin < GPIO_DIRECT_PIN_BASE && !isEncodedDirectPin(pin);
}

int resolveDirectGpioPin(int pin) {
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
}  // namespace

GantryEndEffector::GantryEndEffector()
    : pin_(-1), activeHigh_(true), configured_(false), active_(false) {}

void GantryEndEffector::configurePin(int pin, bool activeHigh) {
    pin_ = pin;
    activeHigh_ = activeHigh;
    configured_ = (pin_ >= 0);
}

bool GantryEndEffector::begin() {
    if (!configured_) {
        return false;
    }
    if (isMcpLogicalPin(pin_)) {
        if (gpio_expander_set_direction(pin_, true) != ESP_OK) {
            return false;
        }
    } else {
    const int gpioPin = resolveDirectGpioPin(pin_);
    if (gpioPin < 0) {
        return false;
    }
    pinMode(gpioPin, OUTPUT);
    }
    setActive(false);
    return true;
}

void GantryEndEffector::setActive(bool active) {
    if (!configured_) {
        return;
    }
    const uint8_t level = (active == activeHigh_) ? 1 : 0;
    if (isMcpLogicalPin(pin_)) {
        gpio_expander_write(pin_, level);
    } else {
        const int gpioPin = resolveDirectGpioPin(pin_);
        if (gpioPin < 0) {
            return;
        }
        digitalWrite(gpioPin, level ? HIGH : LOW);
    }
    active_ = active;
}

} // namespace Gantry

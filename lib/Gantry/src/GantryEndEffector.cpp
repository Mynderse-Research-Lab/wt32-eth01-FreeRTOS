/**
 * @file GantryEndEffector.cpp
 * @brief End-effector implementation
 * @version 1.0.0
 */

#include "GantryEndEffector.h"
#include "GantryEndEffectorPolicy.h"
#include "GantryLimitSwitchPolicy.h"
#include "gpio_expander.h"
#include "driver/gpio.h"

namespace Gantry {

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
    if (EndEffectorPolicy::classifyPin(pin_) == EndEffectorPolicy::PinDomain::kMcp) {
        if (gpio_expander_set_direction(pin_, true) != ESP_OK) {
            return false;
        }
    } else {
        const int gpioPin = LimitPolicy::resolveDirectGpioPin(pin_);
        if (gpioPin < 0) {
            return false;
        }
        gpio_config_t io_conf = {};
        io_conf.pin_bit_mask = (1ULL << gpioPin);
        io_conf.mode         = GPIO_MODE_OUTPUT;
        io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.intr_type    = GPIO_INTR_DISABLE;
        if (gpio_config(&io_conf) != ESP_OK) {
            return false;
        }
    }
    setActive(false);
    return true;
}

bool GantryEndEffector::setActive(bool active) {
    if (!configured_) {
        return false;
    }
    const uint8_t level = EndEffectorPolicy::outputLevel(active, activeHigh_);
    if (EndEffectorPolicy::classifyPin(pin_) == EndEffectorPolicy::PinDomain::kMcp) {
        if (gpio_expander_write(pin_, level) != ESP_OK) {
            return false;
        }
    } else {
        const int gpioPin = LimitPolicy::resolveDirectGpioPin(pin_);
        if (gpioPin < 0) {
            return false;
        }
        if (gpio_set_level((gpio_num_t)gpioPin, level) != ESP_OK) {
            return false;
        }
    }
    active_ = active;
    return true;
}

} // namespace Gantry

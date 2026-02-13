#include "GantryLimitSwitch.h"

#include "gpio_expander.h"

namespace Gantry {

GantryLimitSwitch::GantryLimitSwitch()
    : pin_(-1), activeLow_(true), enablePullup_(true), debounceCycles_(3),
      sampleState_(false), stableState_(false), stableCount_(0), initialized_(false) {}

void GantryLimitSwitch::configure(int pin, bool activeLow, bool enablePullup,
                                  uint8_t debounceCycles) {
    pin_ = pin;
    activeLow_ = activeLow;
    enablePullup_ = enablePullup;
    debounceCycles_ = (debounceCycles == 0) ? 1 : debounceCycles;
    sampleState_ = false;
    stableState_ = false;
    stableCount_ = 0;
    initialized_ = false;
}

bool GantryLimitSwitch::begin() {
    if (pin_ < 0) {
        return false;
    }

    // Input pin for switch signal (LOW/HIGH interpretation handled by activeLow_).
    gpio_expander_set_direction((uint8_t)pin_, false);
    if (enablePullup_) {
        // Most limit wiring in this project uses active-low switches with pullups.
        gpio_expander_set_pullup((uint8_t)pin_, true);
    }

    update(true);
    initialized_ = true;
    return true;
}

void GantryLimitSwitch::update(bool force) {
    if (pin_ < 0) {
        return;
    }

    // Convert electrical level to logical "active" state.
    bool rawActive = activeLow_ ? (gpio_expander_read((uint8_t)pin_) == 0)
                                : (gpio_expander_read((uint8_t)pin_) != 0);

    if (force) {
        sampleState_ = rawActive;
        stableState_ = rawActive;
        stableCount_ = debounceCycles_;
        return;
    }

    // Debounce policy: accept a state change only after N consecutive samples.
    if (rawActive == sampleState_) {
        if (stableCount_ < debounceCycles_) {
            stableCount_++;
        }
    } else {
        sampleState_ = rawActive;
        stableCount_ = 1;
    }

    if (stableCount_ >= debounceCycles_) {
        stableState_ = sampleState_;
    }
}

bool GantryLimitSwitch::isConfigured() const {
    return pin_ >= 0;
}

bool GantryLimitSwitch::isActive() const {
    return stableState_;
}

int GantryLimitSwitch::getPin() const {
    return pin_;
}

} // namespace Gantry

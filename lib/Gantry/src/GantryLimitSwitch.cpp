#include "GantryLimitSwitch.h"

#include "gpio_expander.h"
#include "driver/gpio.h"

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

uint8_t readPinLevel(int pin) {
    if (isMcpLogicalPin(pin)) {
        return gpio_expander_read(pin);
    }
    int gpioPin = resolveDirectGpioPin(pin);
    if (gpioPin < 0) {
        return 0;
    }
    return (uint8_t)gpio_get_level((gpio_num_t)gpioPin);
}
} // namespace

GantryLimitSwitch::GantryLimitSwitch()
    : pin_(-1), activeLow_(true), enablePullup_(true), debounceCycles_(6),
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
    if (isMcpLogicalPin(pin_)) {
        gpio_expander_set_direction(pin_, false);
        if (enablePullup_) {
            // Most limit wiring in this project uses active-low switches with pullups.
            gpio_expander_set_pullup(pin_, true);
        }
    } else {
        const int gpioPin = resolveDirectGpioPin(pin_);
        if (gpioPin < 0) {
            return false;
        }
        gpio_config_t io_conf = {};
        io_conf.pin_bit_mask = (1ULL << gpioPin);
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pull_up_en = enablePullup_ ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.intr_type = GPIO_INTR_DISABLE;
        if (gpio_config(&io_conf) != ESP_OK) {
            return false;
        }
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
    bool rawActive = activeLow_ ? (readPinLevel(pin_) == 0)
                                : (readPinLevel(pin_) != 0);

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

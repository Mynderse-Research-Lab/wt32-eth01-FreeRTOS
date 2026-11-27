#include "GantryLimitSwitch.h"
#include "GantryLimitSwitchPolicy.h"

#include "driver/gpio.h"
#include "gpio_expander.h"

namespace Gantry {
namespace {

uint8_t readPinLevel(int pin) {
    if (LimitPolicy::isMcpLogicalPin(pin)) {
        return gpio_expander_read(pin);
    }
    int gpioPin = LimitPolicy::resolveDirectGpioPin(pin);
    if (gpioPin < 0) {
        return 0;
    }
    return (uint8_t)gpio_get_level((gpio_num_t)gpioPin);
}

}  // namespace

GantryLimitSwitch::GantryLimitSwitch()
    : source_(Source::kGpio),
      pin_(-1), activeLow_(true), enablePullup_(true), debounceCycles_(6),
      sampleState_(false), stableState_(false), stableCount_(0), initialized_(false),
      externalActive_(false) {}

void GantryLimitSwitch::configure(int pin, bool activeLow, bool enablePullup,
                                  uint8_t debounceCycles) {
    source_ = Source::kGpio;
    pin_ = pin;
    activeLow_ = activeLow;
    enablePullup_ = enablePullup;
    debounceCycles_ = LimitPolicy::coerceDebounceCycles(debounceCycles);
    sampleState_ = false;
    stableState_ = false;
    stableCount_ = 0;
    initialized_ = false;
    externalActive_ = false;
}

bool GantryLimitSwitch::begin() {
    if (source_ == Source::kDriveManaged) {
        initialized_ = true;
        return true;
    }
    if (pin_ < 0) {
        return false;
    }

    // Input pin for switch signal (LOW/HIGH interpretation handled by activeLow_).
    if (LimitPolicy::isMcpLogicalPin(pin_)) {
        gpio_expander_set_direction(pin_, false);
        if (enablePullup_) {
            // Most limit wiring in this project uses active-low switches with pullups.
            gpio_expander_set_pullup(pin_, true);
        }
    } else {
        const int gpioPin = LimitPolicy::resolveDirectGpioPin(pin_);
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
    // Drive-managed switches are updated via setExternalActive(), not GPIO polling.
    if (source_ == Source::kDriveManaged) {
        return;
    }
    if (pin_ < 0) {
        return;
    }

    bool rawActive = LimitPolicy::rawLevelIsActive(readPinLevel(pin_), activeLow_);

    if (force) {
        LimitPolicy::Debounce d{sampleState_, stableState_, stableCount_};
        LimitPolicy::forceSample(d, rawActive, debounceCycles_);
        sampleState_ = d.sample_state;
        stableState_ = d.stable_state;
        stableCount_ = d.stable_count;
        return;
    }

    LimitPolicy::Debounce d{sampleState_, stableState_, stableCount_};
    LimitPolicy::debounceSample(d, rawActive, debounceCycles_);
    sampleState_ = d.sample_state;
    stableState_ = d.stable_state;
    stableCount_ = d.stable_count;
}

bool GantryLimitSwitch::isConfigured() const {
    if (source_ == Source::kDriveManaged) {
        return initialized_;
    }
    return pin_ >= 0;
}

bool GantryLimitSwitch::isActive() const {
    if (source_ == Source::kDriveManaged) {
        return externalActive_;
    }
    return stableState_;
}

int GantryLimitSwitch::getPin() const {
    return pin_;
}

} // namespace Gantry

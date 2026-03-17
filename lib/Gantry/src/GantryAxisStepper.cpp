/**
 * @file GantryAxisStepper.cpp
 * @brief Step/dir stepper axis implementation
 * @version 1.0.0
 */

#include "GantryAxisStepper.h"
#include <cmath>
#include "gpio_expander.h"
#include "driver/gpio.h"

namespace Gantry {

namespace {
inline bool isValidPin(int pin) {
    return pin >= 0 && pin <= 255;
}

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

inline bool configureOutputPin(int pin) {
    if (!isValidPin(pin)) {
        return false;
    }
    if (isMcpLogicalPin(pin)) {
        return gpio_expander_set_direction(pin, true) == ESP_OK;
    }
    int gpioPin = resolveDirectGpioPin(pin);
    if (gpioPin < 0) {
        return false;
    }
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << gpioPin);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    return gpio_config(&io_conf) == ESP_OK;
}

inline void writePinLevel(int pin, bool high) {
    if (!isValidPin(pin)) {
        return;
    }
    if (isMcpLogicalPin(pin)) {
        (void)gpio_expander_write(pin, high ? 1 : 0);
        return;
    }
    int gpioPin = resolveDirectGpioPin(pin);
    if (gpioPin >= 0) {
        (void)gpio_set_level((gpio_num_t)gpioPin, high ? 1 : 0);
    }
}
}  // namespace

GantryAxisStepper::GantryAxisStepper()
    : stepPin_(-1),
      dirPin_(-1),
      enablePin_(-1),
      invertDir_(false),
      enableActiveLow_(true),
      configured_(false),
      enabled_(false),
      stepsPerMm_(1.0f),
      minMm_(0.0f),
      maxMm_(0.0f),
      maxSpeedMmPerS_(50.0f),
      accelMmPerS2_(200.0f),
      decelMmPerS2_(200.0f),
      stepPulseWidthUs_(2),
      currentSteps_(0),
      targetSteps_(0),
      moving_(false),
      currentSpeedStepsPerS_(0.0f),
      maxSpeedStepsPerS_(0.0f),
      accelStepsPerS2_(0.0f),
      decelStepsPerS2_(0.0f),
      stepAccumulator_(0.0f),
      lastUpdateUs_(0),
      stepDirectGpio_(-1),
      stepUseLedc_(false),
      lastLedcFreqHz_(0) {}

void GantryAxisStepper::configurePins(int stepPin, int dirPin, int enablePin,
                                      bool invertDir, bool enableActiveLow) {
    stepPin_ = stepPin;
    dirPin_ = dirPin;
    enablePin_ = enablePin;
    invertDir_ = invertDir;
    enableActiveLow_ = enableActiveLow;
    configured_ = (stepPin_ >= 0 && dirPin_ >= 0);
}

void GantryAxisStepper::setStepsPerMm(float stepsPerMm) {
    if (stepsPerMm > 0.0f) {
        stepsPerMm_ = stepsPerMm;
    }
}

void GantryAxisStepper::setLimits(float minMm, float maxMm) {
    minMm_ = minMm;
    maxMm_ = maxMm;
}

void GantryAxisStepper::setMotionLimits(float maxSpeedMmPerS,
                                        float accelMmPerS2,
                                        float decelMmPerS2) {
    if (maxSpeedMmPerS > 0.0f) {
        maxSpeedMmPerS_ = maxSpeedMmPerS;
    }
    if (accelMmPerS2 > 0.0f) {
        accelMmPerS2_ = accelMmPerS2;
    }
    if (decelMmPerS2 > 0.0f) {
        decelMmPerS2_ = decelMmPerS2;
    }
}

void GantryAxisStepper::setStepPulseWidthUs(uint16_t pulseWidthUs) {
    if (pulseWidthUs > 0) {
        stepPulseWidthUs_ = pulseWidthUs;
    }
}

bool GantryAxisStepper::begin() {
    if (!configured_) {
        return false;
    }

    if (!configureOutputPin(stepPin_)) {
        return false;
    }
    if (!configureOutputPin(dirPin_)) {
        return false;
    }
    stepDirectGpio_ = resolveDirectGpioPin(stepPin_);
    stepUseLedc_ = (stepDirectGpio_ >= 0);

    if (stepUseLedc_) {
        // Direct pulse pin uses LEDC hardware pulse train generation.
        if (!ledcAttach(static_cast<uint8_t>(stepDirectGpio_), 1000, 1)) {
            return false;
        }
        ledcWrite(static_cast<uint8_t>(stepDirectGpio_), 0);
        lastLedcFreqHz_ = 1000;
    } else {
        writePinLevel(stepPin_, false);
    }

    if (enablePin_ >= 0) {
        if (!configureOutputPin(enablePin_)) {
            return false;
        }
        disable();
    }

    return true;
}

void GantryAxisStepper::enable() {
    if (enablePin_ >= 0) {
        writePinLevel(enablePin_, !enableActiveLow_);
    }
    enabled_ = true;
}

void GantryAxisStepper::disable() {
    if (enablePin_ >= 0) {
        writePinLevel(enablePin_, enableActiveLow_);
    }
    enabled_ = false;
    if (stepUseLedc_ && stepDirectGpio_ >= 0) {
        ledcWrite(static_cast<uint8_t>(stepDirectGpio_), 0);
    }
    stop();
}

bool GantryAxisStepper::moveToMm(float targetMm,
                                 float speedMmPerS,
                                 float accelMmPerS2,
                                 float decelMmPerS2) {
    if (!configured_) {
        return false;
    }

    if (!withinLimits(targetMm)) {
        return false;
    }

    targetMm = clampToLimits(targetMm);

    targetSteps_ = (int32_t)lroundf(targetMm * stepsPerMm_);
    maxSpeedStepsPerS_ =
        (speedMmPerS > 0.0f ? speedMmPerS : maxSpeedMmPerS_) * stepsPerMm_;
    accelStepsPerS2_ =
        (accelMmPerS2 > 0.0f ? accelMmPerS2 : accelMmPerS2_) * stepsPerMm_;
    decelStepsPerS2_ =
        (decelMmPerS2 > 0.0f ? decelMmPerS2 : decelMmPerS2_) * stepsPerMm_;

    if (maxSpeedStepsPerS_ <= 0.0f || accelStepsPerS2_ <= 0.0f ||
        decelStepsPerS2_ <= 0.0f) {
        return false;
    }

    moving_ = (targetSteps_ != currentSteps_);
    currentSpeedStepsPerS_ = 0.0f;
    stepAccumulator_ = 0.0f;
    lastUpdateUs_ = micros();
    return true;
}

void GantryAxisStepper::stop() {
    moving_ = false;
    currentSpeedStepsPerS_ = 0.0f;
    stepAccumulator_ = 0.0f;
    if (stepUseLedc_ && stepDirectGpio_ >= 0) {
        ledcWrite(static_cast<uint8_t>(stepDirectGpio_), 0);
    }
}

void GantryAxisStepper::update() {
    if (!moving_ || !enabled_) {
        return;
    }

    uint32_t nowUs = micros();
    uint32_t elapsedUs = nowUs - lastUpdateUs_;
    if (elapsedUs == 0) {
        return;
    }
    lastUpdateUs_ = nowUs;

    float dt = static_cast<float>(elapsedUs) / 1000000.0f;
    int32_t remainingSteps = targetSteps_ - currentSteps_;
    if (remainingSteps == 0) {
        moving_ = false;
        currentSpeedStepsPerS_ = 0.0f;
        return;
    }

    bool dirPositive = remainingSteps > 0;
    bool dirLevel = dirPositive ^ invertDir_;
    writePinLevel(dirPin_, dirLevel);

    float remaining = fabsf(static_cast<float>(remainingSteps));
    float decelDistance = (currentSpeedStepsPerS_ * currentSpeedStepsPerS_) /
                          (2.0f * decelStepsPerS2_);

    if (remaining <= decelDistance) {
        currentSpeedStepsPerS_ -= decelStepsPerS2_ * dt;
    } else {
        currentSpeedStepsPerS_ += accelStepsPerS2_ * dt;
    }

    if (currentSpeedStepsPerS_ < 0.0f) {
        currentSpeedStepsPerS_ = 0.0f;
    }
    if (currentSpeedStepsPerS_ > maxSpeedStepsPerS_) {
        currentSpeedStepsPerS_ = maxSpeedStepsPerS_;
    }

    stepAccumulator_ += currentSpeedStepsPerS_ * dt;

    if (stepUseLedc_ && stepDirectGpio_ >= 0) {
        uint32_t freqHz = static_cast<uint32_t>(currentSpeedStepsPerS_ + 0.5f);
        if (freqHz == 0) {
            ledcWrite(static_cast<uint8_t>(stepDirectGpio_), 0);
        } else {
            if (freqHz != lastLedcFreqHz_) {
                (void)ledcChangeFrequency(static_cast<uint8_t>(stepDirectGpio_), freqHz, 1);
                lastLedcFreqHz_ = freqHz;
            }
            // 1-bit LEDC duty: 1 = 50% pulse train.
            ledcWrite(static_cast<uint8_t>(stepDirectGpio_), 1);
        }
    }

    int32_t maxStepsThisUpdate = 200;
    while (stepAccumulator_ >= 1.0f && maxStepsThisUpdate-- > 0) {
        if (!stepUseLedc_) {
            stepOnce(dirPositive);
        }
        currentSteps_ += dirPositive ? 1 : -1;
        stepAccumulator_ -= 1.0f;

        if (currentSteps_ == targetSteps_) {
            moving_ = false;
            currentSpeedStepsPerS_ = 0.0f;
            stepAccumulator_ = 0.0f;
            if (stepUseLedc_ && stepDirectGpio_ >= 0) {
                ledcWrite(static_cast<uint8_t>(stepDirectGpio_), 0);
            }
            break;
        }
    }
}

float GantryAxisStepper::getCurrentMm() const {
    return static_cast<float>(currentSteps_) / stepsPerMm_;
}

float GantryAxisStepper::getTargetMm() const {
    return static_cast<float>(targetSteps_) / stepsPerMm_;
}

void GantryAxisStepper::stepOnce(bool dirPositive) {
    bool dirLevel = dirPositive ^ invertDir_;
    writePinLevel(dirPin_, dirLevel);
    writePinLevel(stepPin_, true);
    delayMicroseconds(stepPulseWidthUs_);
    writePinLevel(stepPin_, false);
}

bool GantryAxisStepper::withinLimits(float targetMm) const {
    if (minMm_ == 0.0f && maxMm_ == 0.0f) {
        return true;
    }
    return targetMm >= minMm_ && targetMm <= maxMm_;
}

float GantryAxisStepper::clampToLimits(float targetMm) const {
    if (minMm_ == 0.0f && maxMm_ == 0.0f) {
        return targetMm;
    }
    if (targetMm < minMm_) {
        return minMm_;
    }
    if (targetMm > maxMm_) {
        return maxMm_;
    }
    return targetMm;
}

} // namespace Gantry

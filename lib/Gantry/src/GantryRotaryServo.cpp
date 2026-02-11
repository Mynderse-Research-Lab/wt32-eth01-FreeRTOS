/**
 * @file GantryRotaryServo.cpp
 * @brief PWM servo implementation
 * @version 1.0.0
 */

#include "GantryRotaryServo.h"

namespace Gantry {

namespace {
constexpr uint32_t kServoFreqHz = 50;
constexpr uint8_t kPwmResolutionBits = 16;
constexpr uint32_t kServoPeriodUs = 1000000UL / kServoFreqHz;
} // namespace

GantryRotaryServo::GantryRotaryServo()
    : pwmPin_(-1),
      pwmChannel_(0),
      configured_(false),
      active_(false),
      minDeg_(-90.0f),
      maxDeg_(90.0f),
      minPulseUs_(1000),
      maxPulseUs_(2000),
      currentDeg_(0.0f)
#if !defined(ARDUINO_ARCH_ESP32)
      ,
      servoAttached_(false)
#endif
{
}

void GantryRotaryServo::configurePin(int pwmPin, int pwmChannel) {
    pwmPin_ = pwmPin;
    pwmChannel_ = pwmChannel;
    configured_ = (pwmPin_ >= 0);
}

void GantryRotaryServo::setAngleRange(float minDeg, float maxDeg) {
    minDeg_ = minDeg;
    maxDeg_ = maxDeg;
}

void GantryRotaryServo::setPulseRange(uint16_t minPulseUs, uint16_t maxPulseUs) {
    if (minPulseUs < maxPulseUs) {
        minPulseUs_ = minPulseUs;
        maxPulseUs_ = maxPulseUs;
    }
}

bool GantryRotaryServo::begin() {
    if (!configured_) {
        return false;
    }

#if defined(ARDUINO_ARCH_ESP32)
    active_ = ledcAttach(static_cast<uint8_t>(pwmPin_), kServoFreqHz, kPwmResolutionBits);
    if (!active_) {
        return false;
    }
    moveToDeg(currentDeg_);
    return true;
#else
    servoAttached_ = servo_.attach(pwmPin_, minPulseUs_, maxPulseUs_);
    if (servoAttached_) {
        servo_.write(currentDeg_);
    }
    active_ = servoAttached_;
    return servoAttached_;
#endif
}

void GantryRotaryServo::moveToDeg(float angleDeg) {
    if (!configured_) {
        return;
    }

    if (angleDeg < minDeg_) {
        angleDeg = minDeg_;
    } else if (angleDeg > maxDeg_) {
        angleDeg = maxDeg_;
    }
    currentDeg_ = angleDeg;

    float spanDeg = maxDeg_ - minDeg_;
    float spanPulse = static_cast<float>(maxPulseUs_ - minPulseUs_);
    float fraction = (spanDeg == 0.0f) ? 0.0f : (angleDeg - minDeg_) / spanDeg;
    uint16_t pulseUs =
        static_cast<uint16_t>(minPulseUs_ + (spanPulse * fraction));
    writePulseUs(pulseUs);
}

void GantryRotaryServo::writePulseUs(uint16_t pulseUs) {
#if defined(ARDUINO_ARCH_ESP32)
    uint32_t maxDuty = (1UL << kPwmResolutionBits) - 1;
    uint32_t duty =
        static_cast<uint32_t>((static_cast<float>(pulseUs) / kServoPeriodUs) *
                              static_cast<float>(maxDuty));
    if (duty > maxDuty) {
        duty = maxDuty;
    }
    ledcWrite(pwmChannel_, duty);
#else
    if (servoAttached_) {
        servo_.writeMicroseconds(pulseUs);
    }
#endif
}

} // namespace Gantry

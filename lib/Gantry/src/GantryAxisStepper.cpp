/**
 * @file GantryAxisStepper.cpp
 * @brief Step/dir stepper axis implementation
 * @version 1.0.0
 */

#include "GantryAxisStepper.h"
#include <cmath>

namespace Gantry {

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
      lastUpdateUs_(0) {}

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

    pinMode(stepPin_, OUTPUT);
    pinMode(dirPin_, OUTPUT);
    digitalWrite(stepPin_, LOW);

    if (enablePin_ >= 0) {
        pinMode(enablePin_, OUTPUT);
        disable();
    }

    return true;
}

void GantryAxisStepper::enable() {
    if (enablePin_ >= 0) {
        digitalWrite(enablePin_, enableActiveLow_ ? LOW : HIGH);
    }
    enabled_ = true;
}

void GantryAxisStepper::disable() {
    if (enablePin_ >= 0) {
        digitalWrite(enablePin_, enableActiveLow_ ? HIGH : LOW);
    }
    enabled_ = false;
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
    int32_t maxStepsThisUpdate = 200;
    while (stepAccumulator_ >= 1.0f && maxStepsThisUpdate-- > 0) {
        stepOnce(dirPositive);
        currentSteps_ += dirPositive ? 1 : -1;
        stepAccumulator_ -= 1.0f;

        if (currentSteps_ == targetSteps_) {
            moving_ = false;
            currentSpeedStepsPerS_ = 0.0f;
            stepAccumulator_ = 0.0f;
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
    digitalWrite(dirPin_, dirLevel ? HIGH : LOW);
    digitalWrite(stepPin_, HIGH);
    delayMicroseconds(stepPulseWidthUs_);
    digitalWrite(stepPin_, LOW);
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

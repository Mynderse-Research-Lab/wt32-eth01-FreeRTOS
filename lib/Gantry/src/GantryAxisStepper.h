/**
 * @file GantryAxisStepper.h
 * @brief Simple step/dir stepper axis driver for gantry Y-axis
 * @version 1.0.0
 */

#ifndef GANTRY_AXIS_STEPPER_H
#define GANTRY_AXIS_STEPPER_H

#include <Arduino.h>
#include <stdint.h>

namespace Gantry {

/**
 * @class GantryAxisStepper
 * @brief Lightweight step/dir stepper controller with accel/decel
 *
 * Note: This is a cooperative, update()-driven controller intended for
 * low/medium speeds without a hardware timer ISR.
 */
class GantryAxisStepper {
public:
    GantryAxisStepper();

    void configurePins(int stepPin, int dirPin, int enablePin = -1,
                       bool invertDir = false, bool enableActiveLow = true);
    void setStepsPerMm(float stepsPerMm);
    void setLimits(float minMm, float maxMm);
    void setMotionLimits(float maxSpeedMmPerS,
                         float accelMmPerS2,
                         float decelMmPerS2);
    void setStepPulseWidthUs(uint16_t pulseWidthUs);

    bool begin();
    void enable();
    void disable();

    bool isConfigured() const { return configured_; }
    bool isEnabled() const { return enabled_; }
    bool isBusy() const { return moving_; }

    bool moveToMm(float targetMm,
                  float speedMmPerS = 0.0f,
                  float accelMmPerS2 = 0.0f,
                  float decelMmPerS2 = 0.0f);
    void stop();
    void update();

    float getCurrentMm() const;
    float getTargetMm() const;
    int32_t getCurrentSteps() const { return currentSteps_; }

private:
    void stepOnce(bool dirPositive);
    bool withinLimits(float targetMm) const;
    float clampToLimits(float targetMm) const;

    int stepPin_;
    int dirPin_;
    int enablePin_;
    bool invertDir_;
    bool enableActiveLow_;
    bool configured_;
    bool enabled_;

    float stepsPerMm_;
    float minMm_;
    float maxMm_;

    float maxSpeedMmPerS_;
    float accelMmPerS2_;
    float decelMmPerS2_;
    uint16_t stepPulseWidthUs_;

    int32_t currentSteps_;
    int32_t targetSteps_;
    bool moving_;

    float currentSpeedStepsPerS_;
    float maxSpeedStepsPerS_;
    float accelStepsPerS2_;
    float decelStepsPerS2_;
    float stepAccumulator_;
    uint32_t lastUpdateUs_;
};

} // namespace Gantry

#endif // GANTRY_AXIS_STEPPER_H

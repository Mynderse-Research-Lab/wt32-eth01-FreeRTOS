/**
 * @file GantryPulseMotorRotaryAxis.cpp
 */

#include "GantryPulseMotorRotaryAxis.h"
#include <cmath>
#include <esp_log.h>

namespace Gantry {

static const char* TAG = "GantryRotAxis";

GantryPulseMotorRotaryAxis::GantryPulseMotorRotaryAxis(
    const PulseMotor::DriverConfig& drv,
    const PulseMotor::DrivetrainConfig& dt)
  : driver_(drv),
    drivetrain_(dt),
    pulses_per_deg_(PulseMotor::pulsesPerDeg(dt)),
    min_deg_(-180.0f),
    max_deg_(180.0f),
    target_deg_(0.0f) {
    if (pulses_per_deg_ <= 0.0) {
        ESP_LOGW(TAG,
                 "Non-rotary drivetrain or zero-scale config: ppd=%.6f (type=%d)",
                 pulses_per_deg_, (int)dt.type);
    }
}

bool GantryPulseMotorRotaryAxis::begin()              { return driver_.initialize(); }
bool GantryPulseMotorRotaryAxis::enable()             { return driver_.enable(); }
bool GantryPulseMotorRotaryAxis::disable()            { return driver_.disable(); }
bool GantryPulseMotorRotaryAxis::isEnabled() const    { return driver_.isEnabled(); }
bool GantryPulseMotorRotaryAxis::isMotionActive() const { return driver_.isMotionActive(); }
bool GantryPulseMotorRotaryAxis::isAlarmActive() const  { return driver_.getStatus().alarm_active; }
bool GantryPulseMotorRotaryAxis::clearAlarm()         { return driver_.clearAlarm(); }
double GantryPulseMotorRotaryAxis::pulsesPerDeg() const { return pulses_per_deg_; }

void GantryPulseMotorRotaryAxis::setAngleRange(float min_deg, float max_deg) {
    if (min_deg < max_deg) {
        min_deg_ = min_deg;
        max_deg_ = max_deg;
    }
}

bool GantryPulseMotorRotaryAxis::moveToDeg(float target_deg, float speed_deg_per_s,
                                           float accel_deg_per_s2, float decel_deg_per_s2) {
    if (pulses_per_deg_ <= 0.0) return false;

    // Clamp to configured soft-limit range.
    if (target_deg < min_deg_) target_deg = min_deg_;
    if (target_deg > max_deg_) target_deg = max_deg_;

    const int32_t current_counts = driver_.getConfig().enable_encoder_feedback
        ? driver_.getEncoderPosition()
        : (int32_t)driver_.getPosition();
    const int32_t target_counts = (int32_t)lroundf(target_deg * (float)pulses_per_deg_);
    const int32_t delta_counts  = target_counts - current_counts;
    target_deg_ = target_deg;
    if (delta_counts == 0) return true;

    uint32_t speed_pps = (uint32_t)lroundf(speed_deg_per_s * (float)pulses_per_deg_);
    uint32_t accel_pps = (accel_deg_per_s2 > 0.0f)
        ? (uint32_t)lroundf(accel_deg_per_s2 * (float)pulses_per_deg_)
        : (speed_pps / 2);
    uint32_t decel_pps = (decel_deg_per_s2 > 0.0f)
        ? (uint32_t)lroundf(decel_deg_per_s2 * (float)pulses_per_deg_)
        : (speed_pps / 2);
    if (speed_pps == 0) speed_pps = 1;
    if (accel_pps == 0) accel_pps = 1;
    if (decel_pps == 0) decel_pps = 1;

    return driver_.moveRelative((int64_t)delta_counts, speed_pps, accel_pps, decel_pps);
}

bool  GantryPulseMotorRotaryAxis::stopMotion()        { return driver_.stopMotion(0); }
float GantryPulseMotorRotaryAxis::getTargetDeg() const { return target_deg_; }

float GantryPulseMotorRotaryAxis::getCurrentDeg() const {
    if (pulses_per_deg_ <= 0.0) return 0.0f;
    const int32_t counts = driver_.getConfig().enable_encoder_feedback
        ? driver_.getEncoderPosition()
        : (int32_t)driver_.getPosition();
    return (float)((double)counts / pulses_per_deg_);
}

bool GantryPulseMotorRotaryAxis::isBusy() const { return driver_.isMotionActive(); }

void GantryPulseMotorRotaryAxis::update() {
    (void)TAG;
}

} // namespace Gantry

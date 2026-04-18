/**
 * @file GantryPulseMotorLinearAxis.cpp
 */

#include "GantryPulseMotorLinearAxis.h"
#include <cmath>
#include <esp_log.h>

namespace Gantry {

static const char* TAG = "GantryLinAxis";

GantryPulseMotorLinearAxis::GantryPulseMotorLinearAxis(
    const PulseMotor::DriverConfig& drv,
    const PulseMotor::DrivetrainConfig& dt)
  : driver_(drv),
    drivetrain_(dt),
    pulses_per_mm_(PulseMotor::pulsesPerMm(dt)),
    target_mm_(0.0f) {
    if (pulses_per_mm_ <= 0.0) {
        ESP_LOGW(TAG,
                 "Non-linear drivetrain or zero-scale config: ppm=%.6f (type=%d)",
                 pulses_per_mm_, (int)dt.type);
    }
}

bool GantryPulseMotorLinearAxis::begin()              { return driver_.initialize(); }
bool GantryPulseMotorLinearAxis::enable()             { return driver_.enable(); }
bool GantryPulseMotorLinearAxis::disable()            { return driver_.disable(); }
bool GantryPulseMotorLinearAxis::isEnabled() const    { return driver_.isEnabled(); }
bool GantryPulseMotorLinearAxis::isMotionActive() const { return driver_.isMotionActive(); }
bool GantryPulseMotorLinearAxis::isAlarmActive() const  { return driver_.getStatus().alarm_active; }
bool GantryPulseMotorLinearAxis::clearAlarm()         { return driver_.clearAlarm(); }

double GantryPulseMotorLinearAxis::pulsesPerMm() const { return pulses_per_mm_; }

bool GantryPulseMotorLinearAxis::isEncoderFeedbackEnabled() const {
    return driver_.getConfig().enable_encoder_feedback;
}

uint32_t GantryPulseMotorLinearAxis::homingSpeedPps() const {
    return driver_.getConfig().homing_speed_pps;
}

bool GantryPulseMotorLinearAxis::moveToMm(float target_mm, float speed_mm_per_s,
                                          float accel_mm_per_s2, float decel_mm_per_s2) {
    if (pulses_per_mm_ <= 0.0) return false;

    const int32_t current_counts = isEncoderFeedbackEnabled()
        ? driver_.getEncoderPosition()
        : (int32_t)driver_.getPosition();
    const int32_t target_counts = (int32_t)lroundf(target_mm * (float)pulses_per_mm_);
    const int32_t delta_counts  = target_counts - current_counts;
    if (delta_counts == 0) {
        target_mm_ = target_mm;
        return true;
    }

    uint32_t speed_pps = (uint32_t)lroundf(speed_mm_per_s * (float)pulses_per_mm_);
    uint32_t accel_pps = (accel_mm_per_s2 > 0.0f)
        ? (uint32_t)lroundf(accel_mm_per_s2 * (float)pulses_per_mm_)
        : (speed_pps / 2);
    uint32_t decel_pps = (decel_mm_per_s2 > 0.0f)
        ? (uint32_t)lroundf(decel_mm_per_s2 * (float)pulses_per_mm_)
        : (speed_pps / 2);
    if (speed_pps == 0) speed_pps = 1;
    if (accel_pps == 0) accel_pps = 1;
    if (decel_pps == 0) decel_pps = 1;

    target_mm_ = target_mm;
    return driver_.moveRelative((int64_t)delta_counts, speed_pps, accel_pps, decel_pps);
}

bool GantryPulseMotorLinearAxis::moveRelativeMm(float delta_mm, float speed_mm_per_s,
                                                float accel_mm_per_s2, float decel_mm_per_s2) {
    if (pulses_per_mm_ <= 0.0) return false;
    const int64_t delta_counts = (int64_t)llroundf(delta_mm * (float)pulses_per_mm_);
    if (delta_counts == 0) return true;

    uint32_t speed_pps = (uint32_t)lroundf(speed_mm_per_s * (float)pulses_per_mm_);
    uint32_t accel_pps = (accel_mm_per_s2 > 0.0f)
        ? (uint32_t)lroundf(accel_mm_per_s2 * (float)pulses_per_mm_)
        : (speed_pps / 2);
    uint32_t decel_pps = (decel_mm_per_s2 > 0.0f)
        ? (uint32_t)lroundf(decel_mm_per_s2 * (float)pulses_per_mm_)
        : (speed_pps / 2);
    if (speed_pps == 0) speed_pps = 1;
    if (accel_pps == 0) accel_pps = 1;
    if (decel_pps == 0) decel_pps = 1;

    target_mm_ = getCurrentMm() + delta_mm;
    return driver_.moveRelative(delta_counts, speed_pps, accel_pps, decel_pps);
}

bool  GantryPulseMotorLinearAxis::stopMotion()       { return driver_.stopMotion(0); }
float GantryPulseMotorLinearAxis::getTargetMm() const { return target_mm_; }

float GantryPulseMotorLinearAxis::getCurrentMm() const {
    if (pulses_per_mm_ <= 0.0) return 0.0f;
    const int32_t counts = isEncoderFeedbackEnabled()
        ? driver_.getEncoderPosition()
        : (int32_t)driver_.getPosition();
    return (float)((double)counts / pulses_per_mm_);
}

bool GantryPulseMotorLinearAxis::isBusy() const { return driver_.isMotionActive(); }

bool GantryPulseMotorLinearAxis::moveToPulses(uint32_t target_pulses, uint32_t speed_pps,
                                              uint32_t accel_pps2, uint32_t decel_pps2) {
    return driver_.moveToPosition(target_pulses, speed_pps, accel_pps2, decel_pps2);
}

uint32_t GantryPulseMotorLinearAxis::getCurrentPulses() const { return driver_.getPosition(); }
int32_t  GantryPulseMotorLinearAxis::getEncoderPulses() const { return driver_.getEncoderPosition(); }
void     GantryPulseMotorLinearAxis::setCurrentPulses(uint32_t pos) { driver_.setPosition(pos); }

void GantryPulseMotorLinearAxis::update() {
    // Driver runs on esp_timer; no periodic action needed at the axis level.
    (void)TAG;
}

} // namespace Gantry

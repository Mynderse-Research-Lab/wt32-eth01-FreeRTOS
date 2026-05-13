/**
 * @file GantryPulseMotorRotaryAxis.cpp
 */

#include "GantryPulseMotorRotaryAxis.h"
#include <cmath>
#include <esp_log.h>
#include <esp_timer.h>

namespace Gantry {

static const char* TAG = "GantryRotAxis";

static inline uint32_t rot_axis_millis() {
    return (uint32_t)(esp_timer_get_time() / 1000LL);
}

GantryPulseMotorRotaryAxis::GantryPulseMotorRotaryAxis(
    const PulseMotor::DriverConfig& drv,
    const PulseMotor::DrivetrainConfig& dt)
  : driver_(drv),
    drivetrain_(dt),
    pulses_per_deg_(PulseMotor::pulsesPerDeg(dt)),
    min_deg_(-180.0f),
    max_deg_(180.0f),
    target_deg_(0.0f),
    log_tag_(""),
    log_rate_hz_(0),
    last_log_ms_(0),
    was_active_(false) {
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

void GantryPulseMotorRotaryAxis::setLogTag(const char* tag) {
    log_tag_ = (tag != nullptr) ? tag : "";
}

void GantryPulseMotorRotaryAxis::setLogRateHz(uint32_t hz) {
    log_rate_hz_ = hz;
    last_log_ms_ = 0;
}

void GantryPulseMotorRotaryAxis::update() {
    // The driver's motion profile runs on its own esp_timer; this hook is
    // used purely to emit per-axis MOVE log lines. State transitions fire
    // unconditionally; the periodic MOVE line is gated by log_rate_hz_.
    const bool active = driver_.isMotionActive();
    const float cur_deg    = getCurrentDeg();
    const float target_deg = target_deg_;

    if (active != was_active_) {
        if (active) {
            ESP_LOGI(TAG, "MOVE START: axis=%s pos=%.3f deg target=%.3f deg",
                     log_tag_, cur_deg, target_deg);
            last_log_ms_ = rot_axis_millis();
        } else {
            ESP_LOGI(TAG, "MOVE END: axis=%s pos=%.3f deg", log_tag_, cur_deg);
        }
        was_active_ = active;
    }

    if (active && log_rate_hz_ > 0) {
        const uint32_t now_ms = rot_axis_millis();
        uint32_t period_ms = 1000u / log_rate_hz_;
        if (period_ms == 0) {
            period_ms = 1;
        }
        if (last_log_ms_ == 0 || (now_ms - last_log_ms_) >= period_ms) {
            ESP_LOGI(TAG, "MOVE: axis=%s pos=%.3f deg target=%.3f deg",
                     log_tag_, cur_deg, target_deg);
            last_log_ms_ = now_ms;
        }
    }
}

} // namespace Gantry

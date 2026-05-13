/**
 * @file GantryPulseMotorLinearAxis.cpp
 */

#include "GantryPulseMotorLinearAxis.h"
#include <cmath>
#include <esp_log.h>
#include <esp_timer.h>

namespace Gantry {

static const char* TAG = "GantryLinAxis";

static inline uint32_t lin_axis_millis() {
    return (uint32_t)(esp_timer_get_time() / 1000LL);
}

GantryPulseMotorLinearAxis::GantryPulseMotorLinearAxis(
    const PulseMotor::DriverConfig& drv,
    const PulseMotor::DrivetrainConfig& dt)
  : driver_(drv),
    drivetrain_(dt),
    pulses_per_mm_(PulseMotor::pulsesPerMm(dt)),
    target_mm_(0.0f),
    log_tag_(""),
    log_rate_hz_(0),
    last_log_ms_(0),
    was_active_(false) {
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

void GantryPulseMotorLinearAxis::setLogTag(const char* tag) {
    log_tag_ = (tag != nullptr) ? tag : "";
}

void GantryPulseMotorLinearAxis::setLogRateHz(uint32_t hz) {
    log_rate_hz_ = hz;
    last_log_ms_ = 0;
}

void GantryPulseMotorLinearAxis::update() {
    // The driver's motion profile runs on its own esp_timer; this hook is
    // used purely to emit per-axis MOVE log lines. State transitions fire
    // unconditionally; the periodic MOVE line is gated by log_rate_hz_.
    const bool active = driver_.isMotionActive();
    const float cur_mm    = getCurrentMm();
    const float target_mm = target_mm_;

    if (active != was_active_) {
        if (active) {
            ESP_LOGI(TAG, "MOVE START: axis=%s pos=%.3f mm target=%.3f mm",
                     log_tag_, cur_mm, target_mm);
            last_log_ms_ = lin_axis_millis();
        } else {
            ESP_LOGI(TAG, "MOVE END: axis=%s pos=%.3f mm", log_tag_, cur_mm);
        }
        was_active_ = active;
    }

    if (active && log_rate_hz_ > 0) {
        const uint32_t now_ms = lin_axis_millis();
        uint32_t period_ms = 1000u / log_rate_hz_;
        if (period_ms == 0) {
            period_ms = 1;
        }
        if (last_log_ms_ == 0 || (now_ms - last_log_ms_) >= period_ms) {
            ESP_LOGI(TAG, "MOVE: axis=%s pos=%.3f mm target=%.3f mm",
                     log_tag_, cur_mm, target_mm);
            last_log_ms_ = now_ms;
        }
    }
}

} // namespace Gantry

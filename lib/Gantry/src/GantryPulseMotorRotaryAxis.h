/**
 * @file GantryPulseMotorRotaryAxis.h
 * @brief Rotary axis backed by PulseMotor::PulseMotorDriver.
 *
 * Works with DRIVETRAIN_ROTARY_DIRECT drivetrains. Converts deg <-> pulses
 * using DrivetrainConfig.
 */

#ifndef GANTRY_PULSE_MOTOR_ROTARY_AXIS_H
#define GANTRY_PULSE_MOTOR_ROTARY_AXIS_H

#include "GantryRotaryAxis.h"
#include "PulseMotor.h"

namespace Gantry {

class GantryPulseMotorRotaryAxis : public GantryRotaryAxis {
public:
    GantryPulseMotorRotaryAxis(const PulseMotor::DriverConfig& drv,
                               const PulseMotor::DrivetrainConfig& dt);

    bool begin() override;
    bool enable() override;
    bool disable() override;
    bool isEnabled() const override;

    bool  moveToDeg(float target_deg, float speed_deg_per_s,
                    float accel_deg_per_s2, float decel_deg_per_s2) override;
    bool  stopMotion() override;
    float getCurrentDeg() const override;
    float getTargetDeg() const override;
    bool  isBusy() const override;
    bool  isMotionActive() const override;

    bool isAlarmActive() const override;
    bool clearAlarm() override;

    double pulsesPerDeg() const override;
    void   update() override;

    void setAngleRange(float min_deg, float max_deg) override;

    void setLogTag(const char* tag) override;
    void setLogRateHz(uint32_t hz) override;

    PulseMotor::PulseMotorDriver&       driver()       { return driver_; }
    const PulseMotor::PulseMotorDriver& driver() const { return driver_; }

private:
    PulseMotor::PulseMotorDriver driver_;
    PulseMotor::DrivetrainConfig drivetrain_;
    double pulses_per_deg_;
    float  min_deg_;
    float  max_deg_;
    float  target_deg_;

    // Motion logging state (driven from update()).
    const char* log_tag_;
    uint32_t    log_rate_hz_;
    uint32_t    last_log_ms_;
    bool        was_active_;
};

} // namespace Gantry

#endif // GANTRY_PULSE_MOTOR_ROTARY_AXIS_H

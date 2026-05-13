/**
 * @file GantryPulseMotorLinearAxis.h
 * @brief Linear axis backed by PulseMotor::PulseMotorDriver.
 *
 * Works for any linear drivetrain (BALLSCREW, BELT, RACKPINION). The
 * conversion from mm to encoder pulses comes from the DrivetrainConfig
 * supplied at construction time.
 */

#ifndef GANTRY_PULSE_MOTOR_LINEAR_AXIS_H
#define GANTRY_PULSE_MOTOR_LINEAR_AXIS_H

#include "GantryLinearAxis.h"
#include "PulseMotor.h"

namespace Gantry {

class GantryPulseMotorLinearAxis : public GantryLinearAxis {
public:
    GantryPulseMotorLinearAxis(const PulseMotor::DriverConfig& drv,
                               const PulseMotor::DrivetrainConfig& dt);

    bool begin() override;
    bool enable() override;
    bool disable() override;
    bool isEnabled() const override;

    bool  moveToMm(float target_mm, float speed_mm_per_s,
                   float accel_mm_per_s2, float decel_mm_per_s2) override;
    bool  moveRelativeMm(float delta_mm, float speed_mm_per_s,
                         float accel_mm_per_s2, float decel_mm_per_s2) override;
    bool  stopMotion() override;
    float getCurrentMm() const override;
    float getTargetMm() const override;
    bool  isBusy() const override;

    bool     moveToPulses(uint32_t target_pulses, uint32_t speed_pps,
                          uint32_t accel_pps2, uint32_t decel_pps2) override;
    uint32_t getCurrentPulses() const override;
    int32_t  getEncoderPulses() const override;
    void     setCurrentPulses(uint32_t pos) override;
    bool     isMotionActive() const override;

    bool isAlarmActive() const override;
    bool clearAlarm() override;

    double pulsesPerMm() const override;
    bool   isEncoderFeedbackEnabled() const override;

    void update() override;

    uint32_t homingSpeedPps() const override;

    void setLogTag(const char* tag) override;
    void setLogRateHz(uint32_t hz) override;

    /// @brief Direct access to the underlying driver. Gantry-internal use only
    ///        (homing, travel measurement, alarm monitoring details).
    PulseMotor::PulseMotorDriver&       driver()       { return driver_; }
    const PulseMotor::PulseMotorDriver& driver() const { return driver_; }

private:
    PulseMotor::PulseMotorDriver  driver_;
    PulseMotor::DrivetrainConfig  drivetrain_;
    double pulses_per_mm_;
    float  target_mm_;

    // Motion logging state (driven from update()).
    const char* log_tag_;
    uint32_t    log_rate_hz_;
    uint32_t    last_log_ms_;
    bool        was_active_;
};

} // namespace Gantry

#endif // GANTRY_PULSE_MOTOR_LINEAR_AXIS_H

/**
 * @file GantryLinearAxis.h
 * @brief Abstract interface for a linear (mm-domain) gantry axis.
 *
 * Implemented by concrete classes (e.g. GantryPulseMotorLinearAxis) that back
 * this interface with a specific driver + drivetrain combination. The Gantry
 * class holds its X and Y axes as std::unique_ptr<GantryLinearAxis> so the
 * axis topology can be chosen per axis at construction time without any
 * compile-time flags.
 */

#ifndef GANTRY_LINEAR_AXIS_H
#define GANTRY_LINEAR_AXIS_H

#include <stdint.h>

namespace Gantry {

/**
 * @class GantryLinearAxis
 * @brief Linear axis interface (unit: mm). Pulse-domain accessors are exposed
 *        for the homing/calibration code paths which must run in driver
 *        counts rather than engineering units.
 */
class GantryLinearAxis {
public:
    virtual ~GantryLinearAxis() = default;

    // ---------- Lifecycle ----------
    virtual bool begin()                  = 0;
    virtual bool enable()                 = 0;
    virtual bool disable()                = 0;
    virtual bool isEnabled() const        = 0;

    // ---------- mm-domain motion ----------
    virtual bool  moveToMm(float target_mm, float speed_mm_per_s,
                           float accel_mm_per_s2, float decel_mm_per_s2) = 0;
    virtual bool  moveRelativeMm(float delta_mm, float speed_mm_per_s,
                                 float accel_mm_per_s2, float decel_mm_per_s2) = 0;
    virtual bool  stopMotion()            = 0;
    virtual float getCurrentMm() const    = 0;
    virtual float getTargetMm() const     = 0;
    virtual bool  isBusy() const          = 0;

    // ---------- Pulse-domain accessors (for homing / calibration) ----------
    virtual bool     moveToPulses(uint32_t target_pulses, uint32_t speed_pps,
                                  uint32_t accel_pps2, uint32_t decel_pps2) = 0;
    virtual uint32_t getCurrentPulses() const        = 0;
    virtual int32_t  getEncoderPulses() const        = 0;
    virtual void     setCurrentPulses(uint32_t pos)  = 0;
    virtual bool     isMotionActive() const          = 0;

    // ---------- Faults ----------
    virtual bool isAlarmActive() const = 0;
    virtual bool clearAlarm()          = 0;

    // ---------- Scaling ----------
    virtual double pulsesPerMm() const = 0;
    virtual bool   isEncoderFeedbackEnabled() const = 0;

    // ---------- Periodic ----------
    virtual void update() = 0;

    // ---------- Homing speed (used by Gantry::home / calibrate) ----------
    virtual uint32_t homingSpeedPps() const = 0;
};

} // namespace Gantry

#endif // GANTRY_LINEAR_AXIS_H

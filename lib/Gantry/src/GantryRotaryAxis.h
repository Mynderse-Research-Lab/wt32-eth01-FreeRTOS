/**
 * @file GantryRotaryAxis.h
 * @brief Abstract interface for a rotary (degree-domain) gantry axis.
 */

#ifndef GANTRY_ROTARY_AXIS_H
#define GANTRY_ROTARY_AXIS_H

#include <stdint.h>

namespace Gantry {

/**
 * @class GantryRotaryAxis
 * @brief Rotary axis interface (unit: degrees).
 */
class GantryRotaryAxis {
public:
    virtual ~GantryRotaryAxis() = default;

    virtual bool begin()           = 0;
    virtual bool enable()          = 0;
    virtual bool disable()         = 0;
    virtual bool isEnabled() const = 0;

    virtual bool  moveToDeg(float target_deg, float speed_deg_per_s,
                            float accel_deg_per_s2, float decel_deg_per_s2) = 0;
    virtual bool  stopMotion()         = 0;
    virtual float getCurrentDeg() const = 0;
    virtual float getTargetDeg() const  = 0;
    virtual bool  isBusy() const        = 0;
    virtual bool  isMotionActive() const = 0;

    virtual bool isAlarmActive() const = 0;
    virtual bool clearAlarm()          = 0;

    virtual double pulsesPerDeg() const = 0;

    virtual void update() = 0;

    virtual void setAngleRange(float min_deg, float max_deg) = 0;
};

} // namespace Gantry

#endif // GANTRY_ROTARY_AXIS_H

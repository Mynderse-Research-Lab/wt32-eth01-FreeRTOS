/**
 * @file GantryRotaryAxis.h
 * @brief Abstract interface for a rotary (degree-domain) gantry axis.
 */

#ifndef GANTRY_ROTARY_AXIS_H
#define GANTRY_ROTARY_AXIS_H

#include <stddef.h>
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
    /// EIP: fill buf with HCS01 diag summary. Default false / empty.
    virtual bool getDriveAlarmSummary(char* buf, size_t n) const {
        if (buf != nullptr && n > 0) {
            buf[0] = '\0';
        }
        return false;
    }
    /// One-line CIP/T→O / ready / c1err / diag. Default empty.
    virtual bool formatCipStatus(char* buf, size_t n) const {
        if (buf != nullptr && n > 0) {
            buf[0] = '\0';
        }
        return false;
    }

    virtual double pulsesPerDeg() const = 0;

    /// Runtime PUU/deg (console `puu t`). Re-home after changing scale.
    virtual bool setPuuPerDeg(double puu_per_deg) {
        (void)puu_per_deg;
        return false;
    }

    /// Capture current encoder as joint 0 (HIPERFACE absolute; no X31 seek).
    /// On the EIP rotary this aligns joint to drive abs when |S-0-0051| is
    /// small (C0300 done); otherwise it offsets and shrinks thetalim.
    virtual bool captureSoftHome() = 0;
    /// True when a Class 1 actual assembly has been received.
    virtual bool hasLiveFeedback() const = 0;
    /// Drive S-0-0051 in degrees (no firmware origin offset).
    virtual float getDriveAbsDeg() const { return getCurrentDeg(); }
    virtual bool isDriveOriginAligned() const { return true; }
    virtual float getMinDeg() const { return -180.0f; }
    virtual float getMaxDeg() const { return 180.0f; }

    virtual void update() = 0;

    virtual void setAngleRange(float min_deg, float max_deg) = 0;

    // ---------- Motion logging ----------
    /// @brief Set the short human-readable axis tag used in MOVE log lines.
    ///        Caller must keep the pointer valid for the axis lifetime.
    virtual void setLogTag(const char* tag) = 0;
    /// @brief Periodic MOVE log rate while motion is active.
    ///        0 disables periodic output; START/END events always fire.
    virtual void setLogRateHz(uint32_t hz) = 0;
};

} // namespace Gantry

#endif // GANTRY_ROTARY_AXIS_H

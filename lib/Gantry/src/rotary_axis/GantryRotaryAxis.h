/**
 * @file GantryRotaryAxis.h
 * @brief Abstract interface for a rotary (degree-domain) gantry axis.
 */

#ifndef GANTRY_ROTARY_AXIS_H
#define GANTRY_ROTARY_AXIS_H

#include "GantryDriveRef.h"

#include <stddef.h>
#include <stdint.h>

namespace Gantry {

/// One drive feedback read for the 1 kHz theta trace.
struct RotaryTraceSample {
    double deg = 0.0;        // actual position, axis degrees
    float vel_deg_s = 0.0f;  // actual velocity
    uint16_t status = 0;     // drive status word
};

/// How the axis ended its last move.
enum class RotaryMoveEnd : uint8_t {
    kNone = 0,      // still running
    kExact = 1,     // in the position window with the drive's reached bit
    kStiction = 2,  // stationary inside the wider stiction window
    kTimeout = 3,
    kAbort = 4,     // stopMotion or link loss
};

/// The last commanded move, as the axis sent it. seq changes on every move.
struct RotaryMoveStamp {
    uint32_t seq = 0;
    int64_t cmd_us = 0;    // move commanded (target preloaded)
    int64_t start_us = 0;  // start edge sent to the drive; 0 = not yet
    int64_t done_us = 0;   // axis ended the move; 0 = not yet
    RotaryMoveEnd end = RotaryMoveEnd::kNone;
    float start_deg = 0.0f;
    float target_deg = 0.0f;
    float vel_deg_s = 0.0f;  // after the drive caps
    float acc_deg_s2 = 0.0f;
    float dec_deg_s2 = 0.0f;
};

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
    /// Cyclic drive link up. While false, angle reads are not live.
    virtual bool isLinkOnline() const  = 0;
    /// Drive reports its servo on (in operation). False after a trip until 'enable'.
    virtual bool isServoActive() const = 0;
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
    /// Live HCS01 in-reference. Default unknown (host fakes override).
    virtual DrivePositionRef getDrivePositionRef() const {
        return DrivePositionRef::kUnknown;
    }
    /// Drive S-0-0051 in degrees (no firmware origin offset).
    virtual float getDriveAbsDeg() const { return getCurrentDeg(); }
    virtual bool isDriveOriginAligned() const { return true; }
    virtual float getMinDeg() const { return -180.0f; }
    virtual float getMaxDeg() const { return 180.0f; }

    virtual void update() = 0;

    virtual void setAngleRange(float min_deg, float max_deg) = 0;

    // ---------- Trace (lock-free, any task) ----------
    /// Live feedback for the 1 kHz theta trace. False when not available.
    virtual bool sampleTrace(RotaryTraceSample& out) const {
        (void)out;
        return false;
    }
    /// Last move stamp. False when the axis does not stamp moves.
    virtual bool getMoveStamp(RotaryMoveStamp& out) const {
        (void)out;
        return false;
    }

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

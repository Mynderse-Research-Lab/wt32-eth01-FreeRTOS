/**
 * @file GantryTypes.h
 * @brief Error, status, motion-state and waypoint types for the gantry.
 */
#ifndef GANTRY_TYPES_H
#define GANTRY_TYPES_H

#include "GantryConfig.h"

#include <cstddef>
#include <cstdint>

namespace Gantry {

enum class GantryError {
    OK,
    NOT_INITIALIZED,
    MOTOR_NOT_ENABLED,
    NOT_CALIBRATED,
    ALREADY_MOVING,
    INVALID_POSITION,
    INVALID_PARAMETER,
    TIMEOUT,
    LIMIT_SWITCH_FAILED,
    CALIBRATION_FAILED,
    CONVERSION_ERROR
};

enum class HomingStatus {
    IDLE,
    IN_PROGRESS,
    COMPLETE,
    FAILED
};

struct GantryStatus {
    int32_t currentX_mm;
    int32_t currentZ_mm;
    int32_t currentTheta_deg;
    int32_t targetX_mm;
    int32_t targetZ_mm;
    int32_t targetTheta_deg;
    bool isBusy;
    bool xMoving;
    bool zMoving;
    bool thetaMoving;
    bool initialized;
    bool enabled;
    bool gripperActive;
    bool alarmActive;
    int32_t axisLength_mm;
    int32_t workspaceOriginOffset_mm;
    uint32_t lastUpdate_ms;
};

    // Sequential / path motion state machine. +Z = down (toward belt):
    //   Z_DESCENDING  - Z-alone increasing joint Z (toward belt / A014).
    //   Z_RETRACTING  - Z-alone decreasing joint Z (toward A015 / retract).
    //   X_MOVING      - X-alone (Z in SAFE_Z band).
    //   XZ_MOVING     - in-band X+Z, or retract with X unlocked once in-band.
    enum class MotionState {
        IDLE,
        Z_DESCENDING,
        GRIPPER_ACTUATING,
        Z_RETRACTING,
        X_MOVING,
        XZ_MOVING,
        THETA_MOVING
    };
    // Full bring-up SM (Z- A015-clear = 0 → X home/cal → X park →
    // Z+ stroke → return to band ceiling). When active, single-axis
    // eipLimitPhase_ is not used.
    enum class BringUpPhase : uint8_t {
        kIdle = 0,
        kZMinusSeek,
        kZMinusCreep,
        kZMinusSettle,
        kZEnterBand,
        kXHomeSeek,
        kXHomeCreep,
        kXHomeSettle,
        kXCalSeek,
        kXCalCreep,
        kXCalSettle,
        kXPark,
        kThetaOrient,
        kZPlusSeek,
        kZPlusCreep,
        kZPlusSettle,
        kZReturnBand,
    };

// ============================================================================
// WAYPOINT AND TRAJECTORY PLANNING (unchanged)
// ============================================================================

struct Waypoint {
    EndEffectorPose pose;
    uint32_t speed_mm_per_s;
    uint32_t speed_deg_per_s;
    uint32_t acceleration_mm_per_s2;
    uint32_t deceleration_mm_per_s2;

    Waypoint()
      : speed_mm_per_s(50), speed_deg_per_s(30),
        acceleration_mm_per_s2(0), deceleration_mm_per_s2(0) {}

    Waypoint(const EndEffectorPose& p)
      : pose(p), speed_mm_per_s(50), speed_deg_per_s(30),
        acceleration_mm_per_s2(0), deceleration_mm_per_s2(0) {}
};

template<size_t MAX_WAYPOINTS = 16>
class WaypointQueue {
private:
    Waypoint waypoints_[MAX_WAYPOINTS];
    size_t head_  = 0;
    size_t tail_  = 0;
    size_t count_ = 0;

public:
    bool push(const Waypoint& wp) {
        if (count_ >= MAX_WAYPOINTS) return false;
        waypoints_[tail_] = wp;
        tail_ = (tail_ + 1) % MAX_WAYPOINTS;
        count_++;
        return true;
    }
    bool pop(Waypoint& wp) {
        if (count_ == 0) return false;
        wp = waypoints_[head_];
        head_ = (head_ + 1) % MAX_WAYPOINTS;
        count_--;
        return true;
    }
    size_t size() const { return count_; }
    bool   empty() const { return count_ == 0; }
    bool   full() const { return count_ >= MAX_WAYPOINTS; }
    void   clear() { head_ = 0; tail_ = 0; count_ = 0; }
};

} // namespace Gantry

#endif  // GANTRY_TYPES_H

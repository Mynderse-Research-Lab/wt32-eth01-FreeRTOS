/**
 * @file Gantry.h
 * @brief Multi-axis gantry control system for ESP32
 * @version 2.0.0
 *
 * All three motion axes (X, Y, Theta) are driven by the generic PulseMotor
 * library. Drivetrain topology is chosen PER AXIS via a DrivetrainType enum,
 * so any axis can independently be a ballscrew, a belt, a rack-and-pinion, or
 * a direct-drive rotary.
 *
 * Verified hardware layout:
 *   - X: Allen-Bradley Kinetix 5100 + SCHUNK Beta 100-ZRS belt actuator.
 *   - Y: Allen-Bradley Kinetix 5100 + SCHUNK Beta 80-SRS ballscrew actuator.
 *   - Theta: Custom pulse-train driver + SCHUNK ERD 04-40-D-H-N rotary module.
 *   - End effector: SCHUNK KGG 100-80 pneumatic parallel gripper (digital).
 */

#ifndef GANTRY_H
#define GANTRY_H

#include "GantryConfig.h"
#include "GantryKinematics.h"
#include "GantryTrajectory.h"
#include "GantryLinearAxis.h"
#include "GantryRotaryAxis.h"
#include "GantryEndEffector.h"
#include "GantryLimitSwitch.h"
#include "GantryUtils.h"
#include "PulseMotor.h"
#include <memory>
#include <cstdint>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace Gantry {

struct EndEffectorPose;

enum class GantryError {
    OK,
    NOT_INITIALIZED,
    MOTOR_NOT_ENABLED,
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
    int32_t currentY_mm;
    int32_t currentTheta_deg;
    int32_t targetX_mm;
    int32_t targetY_mm;
    int32_t targetTheta_deg;
    bool isBusy;
    bool xMoving;
    bool yMoving;
    bool thetaMoving;
    bool initialized;
    bool enabled;
    bool gripperActive;
    bool alarmActive;
    int32_t axisLength_mm;
    int32_t workspaceOriginOffset_mm;
    uint32_t lastUpdate_ms;
};

/**
 * @class Gantry
 * @brief Multi-axis gantry control.
 *
 * Construction takes three (DriverConfig, DrivetrainConfig) pairs plus a
 * gripper pin. The Gantry factories the appropriate concrete linear/rotary
 * axis classes based on each axis's DrivetrainType.
 */
class Gantry {
public:
    /**
     * @brief Construct with three per-axis driver/drivetrain pairs.
     *
     * X and Y axes must be configured with a linear drivetrain
     * (BALLSCREW / BELT / RACKPINION). Theta must be configured with
     * DRIVETRAIN_ROTARY_DIRECT.
     */
    Gantry(const PulseMotor::DriverConfig&     xDrv,
           const PulseMotor::DrivetrainConfig& xDt,
           const PulseMotor::DriverConfig&     yDrv,
           const PulseMotor::DrivetrainConfig& yDt,
           const PulseMotor::DriverConfig&     tDrv,
           const PulseMotor::DrivetrainConfig& tDt,
           int gripperPin);

    // ---------- Lifecycle ----------
    bool begin();
    void enable();
    void disable();

    // ---------- Configuration ----------
    void setLimitPins(int xMinPin, int xMaxPin);
    void setYAxisLimits(float minMm, float maxMm);
    void setThetaLimits(float minDeg, float maxDeg);
    void setJointLimits(float xMin, float xMax,
                        float yMin, float yMax,
                        float thetaMin, float thetaMax);
    void setEndEffectorPin(int pin, bool activeHigh = true);
    void setSafeYHeight(float safeHeight_mm);

    // ---------- Motion ----------
    void home();
    int  calibrate();
    void requestAbort();
    bool isAbortRequested() const;

    void moveTo(int32_t x, int32_t y, int32_t theta, uint32_t speed);

    GantryError moveTo(const JointConfig& joint,
                       uint32_t speed_mm_per_s        = 50,
                       uint32_t speed_deg_per_s       = 30,
                       uint32_t acceleration_mm_per_s2 = 0,
                       uint32_t deceleration_mm_per_s2 = 0);

    GantryError moveTo(const EndEffectorPose& pose,
                       uint32_t speed_mm_per_s        = 50,
                       uint32_t speed_deg_per_s       = 30,
                       uint32_t acceleration_mm_per_s2 = 0,
                       uint32_t deceleration_mm_per_s2 = 0);

    bool isBusy() const;
    bool isEnabled() const;
    void update();

    // ---------- Gripper ----------
    void grip(bool active);

    // ---------- Position accessors ----------
    int     getXEncoder() const;
    int     getXEncoderRaw() const;
    int32_t getXCommandedPulses() const;
    float   getXCommandedMm() const;
    float   getXEncoderMm() const;
    int     getCurrentY() const;
    int     getCurrentTheta() const;

    // ---------- Diagnostics ----------
    bool isAlarmActive() const;
    bool clearAlarm();
    void setHomingSpeed(uint32_t speed_pps);

    // ---------- Kinematics ----------
    EndEffectorPose forwardKinematics(const JointConfig& joint) const;
    JointConfig     inverseKinematics(const EndEffectorPose& pose) const;
    JointConfig     getCurrentJointConfig() const;
    JointConfig     getTargetJointConfig() const;
    EndEffectorPose getCurrentEndEffectorPose() const;
    EndEffectorPose getTargetEndEffectorPose() const;

    // ---------- Legacy helpers (kept for console/tests) ----------
    void  setStepsPerRevolution(float steps_per_rev);
    float getStepsPerRevolution() const { return stepsPerRev_; }

    /// @brief X-axis pulses-per-mm (used by console diagnostics).
    float getPulsesPerMm() const;

private:
    std::unique_ptr<GantryLinearAxis> axisX_;
    std::unique_ptr<GantryLinearAxis> axisY_;
    std::unique_ptr<GantryRotaryAxis> axisTheta_;
    GantryEndEffector endEffector_;

    int  gripperPin_;
    int  xMinPin_;
    int  xMaxPin_;
    GantryLimitSwitch xMinSwitch_;
    GantryLimitSwitch xMaxSwitch_;

    bool initialized_;
    bool enabled_;
    bool abortRequested_;
    bool homingInProgress_;
    bool calibrationInProgress_;
    bool gripperActive_;

    // Position tracking (mirrored from axis wrappers for cheap access)
    float   currentX_mm_;
    int32_t currentY_;
    int32_t currentTheta_;
    int32_t targetY_;
    int32_t targetTheta_;
    int32_t axisLength_;

    GantryConfig         config_;
    KinematicParameters  kinematicParams_;
    float                stepsPerRev_;

    // Sequential motion state machine
    enum class MotionState {
        IDLE,
        Y_DESCENDING,
        GRIPPER_ACTUATING,
        Y_RETRACTING,
        X_MOVING,
        THETA_MOVING
    };
    MotionState motionState_;
    float       targetX_mm_;
    float       targetY_mm_;
    float       targetTheta_deg_;
    float       safeYHeight_mm_;
    uint32_t    speed_mm_per_s_;
    uint32_t    speed_deg_per_s_;
    uint32_t    acceleration_mm_per_s2_;
    uint32_t    deceleration_mm_per_s2_;
    bool        gripperTargetState_;
    uint32_t    gripperActuateStart_ms_;
    uint32_t    gripperActuateDurationMs_;
    uint32_t    lastXPositionCounts_;

    // Helpers
    float   pulsesToMm(int32_t pulses) const;
    int32_t mmToPulses(float mm) const;

    void startSequentialMotion();
    void processSequentialMotion();
    void startXAxisMotion();

    uint32_t getHomingSpeed() const;
    bool     moveYAxisTo(float targetY, float speed, float accel, float decel);
    void     updateAxisPositions();
    void     stopAllMotion();
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

#endif // GANTRY_H

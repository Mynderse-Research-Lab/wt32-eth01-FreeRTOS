/**
 * @file Gantry.h
 * @brief Multi-axis gantry control system for ESP32
 * @version 2.0.0
 *
 * This class provides a complete gantry control system with three
 * pulse-train axes (X, Y, Theta), driven by PulseMotor::PulseMotorDriver
 * instances. Per-axis mm/deg <-> pulse conversion is sourced from an
 * explicit PulseMotor::DrivetrainConfig for each axis.
 */

#ifndef GANTRY_H
#define GANTRY_H

#include "GantryConfig.h"
#include "GantryKinematics.h"
#include "GantryTrajectory.h"
#include "GantryEndEffector.h"
#include "GantryLimitSwitch.h"
#include "GantryUtils.h"
#include "PulseMotor.h"
#include <Arduino.h>

#if defined(ARDUINO_ARCH_ESP32)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#endif

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
 * @brief Three-axis pulse-train gantry controller.
 *
 * Mechanical layout:
 * - X axis: linear (belt or ballscrew)
 * - Y axis: linear (belt or ballscrew)
 * - Theta axis: rotary direct-drive
 *
 * Every axis is commanded through PulseMotor::PulseMotorDriver. Unit
 * conversion (mm or deg -> pulses) is driven by the per-axis
 * PulseMotor::DrivetrainConfig supplied via setXDrivetrain() /
 * setYDrivetrain() / setThetaDrivetrain().
 */
class Gantry {
public:
    Gantry(const PulseMotor::DriverConfig &xConfig, int gripperPin);
    Gantry(const PulseMotor::DriverConfig &xConfig,
           const PulseMotor::DriverConfig &yConfig,
           int gripperPin);
    Gantry(const PulseMotor::DriverConfig &xConfig,
           const PulseMotor::DriverConfig &yConfig,
           const PulseMotor::DriverConfig &thetaConfig,
           int gripperPin);

    bool begin();
    void enable();
    void disable();

    /** @brief Configure X-axis soft limit switches (MCP pins). */
    void setLimitPins(int xMinPin, int xMaxPin);

    /** @brief Install the per-axis drivetrain (scaling + travel envelope source). */
    void setXDrivetrain(const PulseMotor::DrivetrainConfig &cfg);
    void setYDrivetrain(const PulseMotor::DrivetrainConfig &cfg);
    void setThetaDrivetrain(const PulseMotor::DrivetrainConfig &cfg);

    /** @brief Joint-space soft travel limits used by validation. */
    void setYAxisLimits(float minMm, float maxMm);
    void setThetaLimits(float minDeg, float maxDeg);
    void setJointLimits(float xMin, float xMax,
                        float yMin, float yMax,
                        float thetaMin, float thetaMax);

    /** @brief Override end-effector pin configuration (optional). */
    void setEndEffectorPin(int pin, bool activeHigh = true);

    /** @brief Minimum Y height above which horizontal travel is allowed. */
    void setSafeYHeight(float safeHeight_mm);

    /** @brief Home the X axis against its MIN limit switch. */
    void home();

    /** @brief Measure X-axis travel length (MIN -> MAX). */
    int calibrate();

    void requestAbort();
    bool isAbortRequested() const;

    /**
     * @brief Simple move command.
     * @param x Target X position (mm).
     * @param y Target Y position (mm).
     * @param theta Target Theta angle (deg).
     * @param speed_mm_per_s Commanded linear speed, shared by X and Y (mm/s).
     */
    void moveTo(int32_t x, int32_t y, int32_t theta, uint32_t speed_mm_per_s);

    GantryError moveTo(const JointConfig& joint,
                      uint32_t speed_mm_per_s = 50,
                      uint32_t speed_deg_per_s = 30,
                      uint32_t acceleration_mm_per_s2 = 0,
                      uint32_t deceleration_mm_per_s2 = 0);

    GantryError moveTo(const EndEffectorPose& pose,
                      uint32_t speed_mm_per_s = 50,
                      uint32_t speed_deg_per_s = 30,
                      uint32_t acceleration_mm_per_s2 = 0,
                      uint32_t deceleration_mm_per_s2 = 0);

    bool isBusy() const;
    bool isEnabled() const;
    void update();

    void grip(bool active);

    int getXEncoder() const;
    int getXEncoderRaw() const;
    int32_t getXCommandedPulses() const;
    float getXCommandedMm() const;
    float getXEncoderMm() const;

    int getCurrentY() const;
    int getCurrentTheta() const;

    bool isAlarmActive() const;
    bool clearAlarm();

    void setHomingSpeed(uint32_t speed_pps);

    EndEffectorPose forwardKinematics(const JointConfig& joint) const;
    JointConfig inverseKinematics(const EndEffectorPose& pose) const;
    JointConfig getCurrentJointConfig() const;
    JointConfig getTargetJointConfig() const;
    EndEffectorPose getCurrentEndEffectorPose() const;
    EndEffectorPose getTargetEndEffectorPose() const;

    /** @brief Per-axis scaling accessors (derived from DrivetrainConfig). */
    double xPulsesPerMm() const;
    double yPulsesPerMm() const;
    double thetaPulsesPerDeg() const;

private:
    PulseMotor::PulseMotorDriver axisX_;
    PulseMotor::PulseMotorDriver axisY_;
    PulseMotor::PulseMotorDriver axisTheta_;

    bool yConfigured_;
    bool yUseEncoder_;
    bool thetaConfigured_;
    bool thetaUseEncoder_;

    PulseMotor::DrivetrainConfig xDrivetrain_;
    PulseMotor::DrivetrainConfig yDrivetrain_;
    PulseMotor::DrivetrainConfig thetaDrivetrain_;

    GantryEndEffector endEffector_;
    int gripperPin_;
    int xMinPin_;
    int xMaxPin_;
    GantryLimitSwitch xMinSwitch_;
    GantryLimitSwitch xMaxSwitch_;

    bool initialized_;
    bool enabled_;
    bool abortRequested_;
    bool homingInProgress_;
    bool calibrationInProgress_;
    bool gripperActive_;

    // Position tracking
    float currentX_mm_;
    int32_t currentY_;
    int32_t currentTheta_;
    int32_t targetY_;
    int32_t targetTheta_;
    int32_t axisLength_;

    // Configuration
    GantryConfig config_;
    KinematicParameters kinematicParams_;

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
    float targetX_mm_;
    float targetY_mm_;
    float targetTheta_deg_;
    float safeYHeight_mm_;
    uint32_t speed_mm_per_s_;
    uint32_t speed_deg_per_s_;
    uint32_t acceleration_mm_per_s2_;
    uint32_t deceleration_mm_per_s2_;
    bool gripperTargetState_;
    uint32_t gripperActuateStart_ms_;
    uint32_t lastXPositionCounts_;

    // Unit conversion helpers (per axis, backed by DrivetrainConfig)
    float xPulsesToMm(int32_t pulses) const;
    int32_t xMmToPulses(float mm) const;
    float yPulsesToMm(int32_t pulses) const;
    int32_t yMmToPulses(float mm) const;
    float thetaPulsesToDeg(int32_t pulses) const;
    int32_t thetaDegToPulses(float deg) const;

    // Sequential motion helpers
    void startSequentialMotion();
    void processSequentialMotion();
    void startXAxisMotion();

    // Common helper functions
    float getCurrentYPosition() const;
    uint32_t getHomingSpeed() const;
    bool moveYAxisTo(float targetY, float speed, float accel, float decel);
    bool isYAxisConfigured() const;
    bool isYAxisBusy() const;
    float getCurrentYFromServoMm() const;
    void updateAxisPositions();
    void stopAllMotion();
};

// ============================================================================
// WAYPOINT AND TRAJECTORY PLANNING
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
    size_t head_ = 0;
    size_t tail_ = 0;
    size_t count_ = 0;

public:
    bool push(const Waypoint& wp) {
        if (count_ >= MAX_WAYPOINTS) {
            return false;
        }
        waypoints_[tail_] = wp;
        tail_ = (tail_ + 1) % MAX_WAYPOINTS;
        count_++;
        return true;
    }

    bool pop(Waypoint& wp) {
        if (count_ == 0) {
            return false;
        }
        wp = waypoints_[head_];
        head_ = (head_ + 1) % MAX_WAYPOINTS;
        count_--;
        return true;
    }

    size_t size() const { return count_; }
    bool empty() const { return count_ == 0; }
    bool full() const { return count_ >= MAX_WAYPOINTS; }
    void clear() {
        head_ = 0;
        tail_ = 0;
        count_ = 0;
    }
};

} // namespace Gantry

#endif // GANTRY_H

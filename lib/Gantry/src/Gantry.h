/**
 * @file Gantry.h
 * @brief Multi-axis gantry control system for ESP32
 * @version 1.0.0
 * 
 * This class provides a complete gantry control system with:
 * - X-axis: Real hardware servo driver (SDF08NK8X)
 * - Y-axis: Step/dir stepper axis or optional SDF08NK8X servo
 * - Theta axis: PWM servo (inline rotary)
 * - End-effector: Digital output control
 * - Workspace coordinate system
 * - Homing and calibration
 * - Integrated with GantryConfig, GantryKinematics, GantryTrajectory modules
 */

#ifndef GANTRY_H
#define GANTRY_H

#include "GantryConfig.h"
#include "GantryKinematics.h"
#include "GantryTrajectory.h"
#include "GantryAxisStepper.h"
#include "GantryRotaryServo.h"
#include "GantryEndEffector.h"
#include "GantryLimitSwitch.h"
#include "GantryUtils.h"
#include "SDF08NK8X.h"
#include <Arduino.h>

#if defined(ARDUINO_ARCH_ESP32)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#endif

namespace Gantry {

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================

struct EndEffectorPose;  // Forward declaration

/**
 * @enum GantryError
 * @brief Error codes for Gantry operations
 */
enum class GantryError {
    OK,                      // Operation successful
    NOT_INITIALIZED,         // Gantry not initialized
    MOTOR_NOT_ENABLED,       // Motor not enabled
    ALREADY_MOVING,          // Motion already in progress
    INVALID_POSITION,        // Position out of valid range
    INVALID_PARAMETER,       // Invalid parameter (speed, etc.)
    TIMEOUT,                 // Operation timed out
    LIMIT_SWITCH_FAILED,     // Limit switch not triggered
    CALIBRATION_FAILED,      // Calibration failed
    CONVERSION_ERROR         // Unit conversion error
};

/**
 * @enum HomingStatus
 * @brief Status of homing operation
 */
enum class HomingStatus {
    IDLE,        // Not homing
    IN_PROGRESS, // Homing in progress
    COMPLETE,    // Homing completed successfully
    FAILED       // Homing failed
};

/**
 * @struct GantryStatus
 * @brief Complete status snapshot of gantry system
 */
struct GantryStatus {
    // Position (current)
    int32_t currentX_mm;        // Current X position in mm
    int32_t currentY_mm;        // Current Y position in mm
    int32_t currentTheta_deg;   // Current Theta angle in degrees
    
    // Target positions
    int32_t targetX_mm;         // Target X position in mm
    int32_t targetY_mm;         // Target Y position in mm
    int32_t targetTheta_deg;    // Target Theta angle in degrees
    
    // Motion state
    bool isBusy;                // True if any axis is moving
    bool xMoving;               // True if X axis is moving
    bool yMoving;               // True if Y axis is moving
    bool thetaMoving;           // True if Theta axis is moving (simulated)
    
    // System state
    bool initialized;           // True if initialized
    bool enabled;               // True if motor enabled
    bool gripperActive;         // True if gripper closed
    bool alarmActive;           // True if alarm condition exists
    
    // Configuration
    int32_t axisLength_mm;      // X-axis length in mm (after calibration)
    int32_t workspaceOriginOffset_mm; // Workspace origin offset in mm
    
    // Timestamp
    uint32_t lastUpdate_ms;     // Timestamp of status update
};

/**
 * @class Gantry
 * @brief Multi-axis gantry control system
 * 
 * Mechanical Layout:
 * - X-axis: Horizontal axis (SDF08NK8X servo driver, ball-screw)
 * - Y-axis: Vertical axis (stepper step/dir or SDF08NK8X servo)
 * - Theta: Rotary axis (PWM servo)
 */
class Gantry {
public:
    /**
     * @brief Construct a new Gantry object
     * @param xConfig Configuration for the X-axis Servo Driver
     * @param gripperPin GPIO pin for the gripper
     */
    Gantry(const BergerdaServo::DriverConfig &xConfig, int gripperPin);
    Gantry(const BergerdaServo::DriverConfig &xConfig,
           const BergerdaServo::DriverConfig &yConfig,
           int gripperPin);
    
    /**
     * @brief Initialize the Gantry system
     * @return bool true if successful, false otherwise
     */
    bool begin();
    
    /**
     * @brief Enable all axes
     */
    void enable();
    
    /**
     * @brief Disable all axes
     */
    void disable();
    
    /**
     * @brief Set limit switch pins
     * @param xMinPin Minimum limit pin (home)
     * @param xMaxPin Maximum limit pin (end)
     */
    void setLimitPins(int xMinPin, int xMaxPin);

    /**
     * @brief Configure Y-axis stepper pins
     */
    void setYAxisPins(int stepPin, int dirPin, int enablePin = -1,
                      bool invertDir = false, bool enableActiveLow = true);

    /**
     * @brief Set Y-axis steps-per-mm conversion
     */
    void setYAxisStepsPerMm(float stepsPerMm);

    /**
     * @brief Set Y-axis travel limits
     */
    void setYAxisLimits(float minMm, float maxMm);

    /**
     * @brief Set Y-axis motion limits
     */
    void setYAxisMotionLimits(float maxSpeedMmPerS,
                              float accelMmPerS2,
                              float decelMmPerS2);

    /**
     * @brief Configure theta servo PWM pin and channel
     */
    void setThetaServo(int pwmPin, int pwmChannel = 0);

    /**
     * @brief Set theta angular limits (degrees)
     */
    void setThetaLimits(float minDeg, float maxDeg);

    /**
     * @brief Set theta servo pulse width range (microseconds)
     */
    void setThetaPulseRange(uint16_t minPulseUs, uint16_t maxPulseUs);

    /**
     * @brief Set joint-space validation limits used by move commands
     */
    void setJointLimits(float xMin, float xMax,
                        float yMin, float yMax,
                        float thetaMin, float thetaMax);

    /**
     * @brief Configure end-effector pin (overrides constructor pin)
     */
    void setEndEffectorPin(int pin, bool activeHigh = true);
    
    /**
     * @brief Set safe Y height for X-axis travel (default: 150mm)
     * @param safeHeight_mm Safe height in mm
     */
    void setSafeYHeight(float safeHeight_mm);
    
    /**
     * @brief Home the X-axis
     */
    void home();
    
    /**
     * @brief Calibrate the X-axis length
     * @return int Axis length in mm, or 0 on failure
     */
    int calibrate();

    /**
     * @brief Request immediate abort of ongoing homing/calibration/motion
     */
    void requestAbort();

    /**
     * @brief Check whether an abort has been requested
     */
    bool isAbortRequested() const;
    
    /**
     * @brief Move to target position
     * @param x Target X position (mm)
     * @param y Target Y position (mm)
     * @param theta Target Theta angle (degrees)
     * @param speed Speed in pulses per second (for X axis)
     */
    void moveTo(int32_t x, int32_t y, int32_t theta, uint32_t speed);
    
    /**
     * @brief Move to joint configuration
     * @param joint Target joint configuration
     * @param speed_mm_per_s Motion speed in mm/s (for X and Y axes)
     * @param speed_deg_per_s Motion speed in deg/s (for Theta axis)
     * @param acceleration_mm_per_s2 Acceleration in mm/s² (0 = use default)
     * @param deceleration_mm_per_s2 Deceleration in mm/s² (0 = use default)
     * @return GantryError code
     */
    GantryError moveTo(const JointConfig& joint,
                      uint32_t speed_mm_per_s = 50,
                      uint32_t speed_deg_per_s = 30,
                      uint32_t acceleration_mm_per_s2 = 0,
                      uint32_t deceleration_mm_per_s2 = 0);
    
    /**
     * @brief Move to end-effector pose
     * @param pose Target end-effector pose in workspace coordinates
     * @param speed_mm_per_s Motion speed in mm/s (for X and Y axes)
     * @param speed_deg_per_s Motion speed in deg/s (for Theta axis)
     * @param acceleration_mm_per_s2 Acceleration in mm/s² (0 = use default)
     * @param deceleration_mm_per_s2 Deceleration in mm/s² (0 = use default)
     * @return GantryError code
     */
    GantryError moveTo(const EndEffectorPose& pose,
                      uint32_t speed_mm_per_s = 50,
                      uint32_t speed_deg_per_s = 30,
                      uint32_t acceleration_mm_per_s2 = 0,
                      uint32_t deceleration_mm_per_s2 = 0);
    
    /**
     * @brief Check if gantry is busy (moving)
     * @return bool true if busy
     */
    bool isBusy() const;

    /**
     * @brief Check if gantry motor control is enabled
     * @return bool true if enabled
     */
    bool isEnabled() const;
    
    /**
     * @brief Update function - call frequently in loop
     */
    void update();
    
    /**
     * @brief Control the gripper
     * @param active true to close, false to open
     */
    void grip(bool active);
    
    /**
     * @brief Get X-axis encoder position
     * @return int Encoder position (pulses)
     */
    int getXEncoder() const;

    /**
     * @brief Get raw X-axis encoder feedback (always encoder counter)
     * @return int Encoder position (pulses)
     */
    int getXEncoderRaw() const;

    /**
     * @brief Get X-axis commanded/driver position
     * @return int Position (pulses)
     */
    int32_t getXCommandedPulses() const;

    /**
     * @brief Get X-axis commanded/driver position in mm
     * @return float Position (mm)
     */
    float getXCommandedMm() const;

    /**
     * @brief Get X-axis raw encoder position in mm
     * @return float Position (mm)
     */
    float getXEncoderMm() const;
    
    /**
     * @brief Get current Y position
     * @return int Current Y position (mm)
     */
    int getCurrentY() const;
    
    /**
     * @brief Get current Theta angle
     * @return int Current Theta angle (degrees)
     */
    int getCurrentTheta() const;
    
    /**
     * @brief Check if alarm condition is active
     * @return bool true if alarm is active, false otherwise
     */
    bool isAlarmActive() const;

    /**
     * @brief Send alarm reset pulse (ARST) to configured drive(s)
     * @return bool true if at least one reset pulse was sent
     */
    bool clearAlarm();
    
    /**
     * @brief Set homing speed for X-axis
     * @param speed_pps Homing speed in pulses per second
     */
    void setHomingSpeed(uint32_t speed_pps);

    // ============================================================================
    // ENHANCED KINEMATICS API (Phase 1.2)
    // ============================================================================
    
    /**
     * @brief Forward kinematics: Joint space -> Workspace
     * @param joint Joint configuration in joint space
     * @return End-effector pose in workspace coordinates
     */
    EndEffectorPose forwardKinematics(const JointConfig& joint) const;
    
    /**
     * @brief Inverse kinematics: Workspace -> Joint space
     * @param pose Desired end-effector pose in workspace coordinates
     * @return Required joint configuration
     */
    JointConfig inverseKinematics(const EndEffectorPose& pose) const;
    
    /**
     * @brief Get current joint configuration
     * @return Current joint positions in joint space
     */
    JointConfig getCurrentJointConfig() const;
    
    /**
     * @brief Get target joint configuration
     * @return Target joint positions in joint space
     */
    JointConfig getTargetJointConfig() const;
    
    /**
     * @brief Get current end-effector pose
     * @return Current end-effector pose in workspace coordinates
     */
    EndEffectorPose getCurrentEndEffectorPose() const;
    
    /**
     * @brief Get target end-effector pose
     * @return Target end-effector pose in workspace coordinates
     */
    EndEffectorPose getTargetEndEffectorPose() const;

private:
    BergerdaServo::ServoDriver axisX_;
    BergerdaServo::ServoDriver axisYServo_;
    bool yServoConfigured_;
    bool yServoUseEncoder_;
    GantryAxisStepper axisY_;
    GantryRotaryServo axisTheta_;
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
    float stepsPerRev_;  // Steps per motor revolution (default: 6000)
    
    // Sequential motion state machine
    enum class MotionState {
        IDLE,              // No motion in progress
        Y_DESCENDING,      // Y-axis moving down to target
        GRIPPER_ACTUATING, // Gripper opening/closing
        Y_RETRACTING,      // Y-axis retracting to safe height
        X_MOVING,          // X-axis moving to target
        THETA_MOVING       // Theta axis moving (can happen anytime)
    };
    
    MotionState motionState_;
    float targetX_mm_;
    float targetY_mm_;
    float targetTheta_deg_;
    float safeYHeight_mm_;  // Safe Y height for X-axis travel
    uint32_t speed_mm_per_s_;
    uint32_t speed_deg_per_s_;
    uint32_t acceleration_mm_per_s2_;
    uint32_t deceleration_mm_per_s2_;
    bool gripperTargetState_;  // Target gripper state (grab/release)
    uint32_t gripperActuateStart_ms_;  // Timestamp when gripper started actuating
    uint32_t lastXPositionCounts_;
    
    // Helper methods for unit conversion (private)
    float pulsesToMm(int32_t pulses) const;
    int32_t mmToPulses(float mm) const;
    
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

public:
    // Kinematics configuration accessors (public for external calculations)
    void setStepsPerRevolution(float steps_per_rev);
    float getStepsPerRevolution() const { return stepsPerRev_; }
    float getPulsesPerMm() const;
};

// ============================================================================
// WAYPOINT AND TRAJECTORY PLANNING (Phase 2.1)
// ============================================================================

/**
 * @struct Waypoint
 * @brief Waypoint for trajectory planning
 * 
 * Represents a point in a trajectory with associated motion parameters.
 * Uses EndEffectorPose for workspace coordinates.
 */
struct Waypoint {
    EndEffectorPose pose;                    // Target pose in workspace coordinates
    uint32_t speed_mm_per_s;                 // Speed for this segment (mm/s)
    uint32_t speed_deg_per_s;                // Speed for theta (deg/s)
    uint32_t acceleration_mm_per_s2;         // Acceleration (mm/s², 0 = use default)
    uint32_t deceleration_mm_per_s2;         // Deceleration (mm/s², 0 = use default)
    
    /**
     * @brief Default constructor
     */
    Waypoint() 
        : speed_mm_per_s(50), speed_deg_per_s(30),
          acceleration_mm_per_s2(0), deceleration_mm_per_s2(0) {}
    
    /**
     * @brief Constructor with pose
     * @param p End-effector pose
     */
    Waypoint(const EndEffectorPose& p) 
        : pose(p), speed_mm_per_s(50), speed_deg_per_s(30),
          acceleration_mm_per_s2(0), deceleration_mm_per_s2(0) {}
};

/**
 * @class WaypointQueue
 * @brief Circular buffer queue for waypoints
 * @tparam MAX_WAYPOINTS Maximum number of waypoints (default: 16)
 * 
 * Thread-safe queue for managing trajectory waypoints.
 * Uses a circular buffer implementation for efficient storage.
 */
template<size_t MAX_WAYPOINTS = 16>
class WaypointQueue {
private:
    Waypoint waypoints_[MAX_WAYPOINTS];
    size_t head_ = 0;
    size_t tail_ = 0;
    size_t count_ = 0;
    
public:
    /**
     * @brief Add waypoint to queue
     * @param wp Waypoint to add
     * @return true if added, false if queue is full
     */
    bool push(const Waypoint& wp) {
        if (count_ >= MAX_WAYPOINTS) {
            return false;  // Queue is full
        }
        
        waypoints_[tail_] = wp;
        tail_ = (tail_ + 1) % MAX_WAYPOINTS;
        count_++;
        return true;
    }
    
    /**
     * @brief Remove waypoint from queue
     * @param wp Reference to store popped waypoint
     * @return true if waypoint was popped, false if queue is empty
     */
    bool pop(Waypoint& wp) {
        if (count_ == 0) {
            return false;  // Queue is empty
        }
        
        wp = waypoints_[head_];
        head_ = (head_ + 1) % MAX_WAYPOINTS;
        count_--;
        return true;
    }
    
    /**
     * @brief Get number of waypoints in queue
     * @return Number of waypoints
     */
    size_t size() const { return count_; }
    
    /**
     * @brief Check if queue is empty
     * @return true if empty
     */
    bool empty() const { return count_ == 0; }
    
    /**
     * @brief Check if queue is full
     * @return true if full
     */
    bool full() const { return count_ >= MAX_WAYPOINTS; }
    
    /**
     * @brief Clear all waypoints from queue
     */
    void clear() {
        head_ = 0;
        tail_ = 0;
        count_ = 0;
    }
};

} // namespace Gantry

#endif // GANTRY_H

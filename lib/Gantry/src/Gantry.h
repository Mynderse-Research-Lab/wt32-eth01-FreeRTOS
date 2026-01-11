/**
 * @file Gantry.h
 * @brief Multi-axis gantry control system for ESP32
 * @version 1.0.0
 * 
 * This class provides a complete gantry control system with:
 * - X-axis: Real hardware servo driver (SDF08NK8X)
 * - Y/Theta axes: Simulated with trapezoidal velocity profiles
 * - Workspace coordinate system
 * - Homing and calibration
 * - Integrated with GantryConfig, GantryKinematics, GantryTrajectory modules
 */

#ifndef GANTRY_H
#define GANTRY_H

#include "GantryConfig.h"
#include "GantryKinematics.h"
#include "GantryTrajectory.h"
#include "SDF08NK8X.h"
#include <Arduino.h>

#if defined(ARDUINO_ARCH_ESP32)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#endif

namespace Gantry {

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
    bool yMoving;               // True if Y axis is moving (simulated)
    bool thetaMoving;           // True if Theta axis is moving (simulated)
    
    // System state
    bool initialized;           // True if initialized
    bool enabled;               // True if motor enabled
    bool gripperActive;         // True if gripper closed
    
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
 * - Y-axis: Vertical axis (simulated)
 * - Theta: Rotary axis (simulated)
 */
class Gantry {
public:
    /**
     * @brief Construct a new Gantry object
     * @param xConfig Configuration for the X-axis Servo Driver
     * @param gripperPin GPIO pin for the gripper
     */
    Gantry(const BergerdaServo::DriverConfig &xConfig, int gripperPin);
    
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
     * @brief Home the X-axis
     */
    void home();
    
    /**
     * @brief Calibrate the X-axis length
     * @return int Axis length in mm, or 0 on failure
     */
    int calibrate();
    
    /**
     * @brief Move to target position
     * @param x Target X position (mm)
     * @param y Target Y position (mm)
     * @param theta Target Theta angle (degrees)
     * @param speed Speed in pulses per second (for X axis)
     */
    void moveTo(int32_t x, int32_t y, int32_t theta, uint32_t speed);
    
    /**
     * @brief Check if gantry is busy (moving)
     * @return bool true if busy
     */
    bool isBusy() const;
    
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
     * @brief Get current Y position
     * @return int Current Y position (mm)
     */
    int getCurrentY() const;
    
    /**
     * @brief Get current Theta angle
     * @return int Current Theta angle (degrees)
     */
    int getCurrentTheta() const;

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
    int gripperPin_;
    int xMinPin_;
    int xMaxPin_;
    
    bool initialized_;
    bool enabled_;
    bool gripperActive_;
    
    // Position tracking
    int32_t currentY_;
    int32_t currentTheta_;
    int32_t targetY_;
    int32_t targetTheta_;
    int32_t axisLength_;
    
    // Configuration
    GantryConfig config_;
    KinematicParameters kinematicParams_;
    
    // Helper methods for unit conversion
    float pulsesToMm(int32_t pulses) const;
    int32_t mmToPulses(float mm) const;
};

} // namespace Gantry

#endif // GANTRY_H

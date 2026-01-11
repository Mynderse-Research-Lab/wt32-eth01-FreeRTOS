/**
 * @file Gantry.cpp
 * @brief Implementation of Gantry class
 * @version 1.0.0
 */

#include "Gantry.h"
#include <cmath>

namespace Gantry {

// ============================================================================
// CONSTRUCTOR
// ============================================================================

Gantry::Gantry(const BergerdaServo::DriverConfig &xConfig, int gripperPin)
    : axisX_(xConfig), gripperPin_(gripperPin), xMinPin_(-1), xMaxPin_(-1),
      initialized_(false), enabled_(false), gripperActive_(false),
      currentY_(0), currentTheta_(0), targetY_(0), targetTheta_(0),
      axisLength_(0), config_(), kinematicParams_() {
}

// ============================================================================
// INITIALIZATION
// ============================================================================

bool Gantry::begin() {
    if (initialized_) {
        return true;
    }
    
    // Initialize gripper pin
    if (gripperPin_ >= 0) {
        pinMode(gripperPin_, OUTPUT);
        digitalWrite(gripperPin_, LOW);
        gripperActive_ = false;
    }
    
    // Configure limit switch pins if set
    if (xMinPin_ >= 0) {
        pinMode(xMinPin_, INPUT_PULLUP);
    }
    if (xMaxPin_ >= 0) {
        pinMode(xMaxPin_, INPUT_PULLUP);
    }
    
    // Initialize X-axis servo driver
    if (!axisX_.initialize()) {
        return false;
    }
    
    initialized_ = true;
    return true;
}

// ============================================================================
// ENABLE/DISABLE
// ============================================================================

void Gantry::enable() {
    if (!initialized_) {
        return;
    }
    axisX_.enable();
    enabled_ = true;
}

void Gantry::disable() {
    if (!initialized_) {
        return;
    }
    if (isBusy()) {
        axisX_.stopMotion(0);
    }
    axisX_.disable();
    enabled_ = false;
}

// ============================================================================
// CONFIGURATION
// ============================================================================

void Gantry::setLimitPins(int xMinPin, int xMaxPin) {
    xMinPin_ = xMinPin;
    xMaxPin_ = xMaxPin;
}

// ============================================================================
// MOTION CONTROL
// ============================================================================

void Gantry::moveTo(int32_t x, int32_t y, int32_t theta, uint32_t speed) {
    if (!initialized_ || !enabled_) {
        return;
    }
    
    // Validate inputs
    if (speed == 0) {
        speed = 5000; // Default speed
    }
    
    // For now, store target positions
    // X-axis motion will be implemented using servo driver
    // Y and Theta are simulated (stub implementation)
    currentY_ = y;
    currentTheta_ = theta;
    
    // TODO: Implement actual motion control with X-axis servo driver
    // This is a minimal implementation to maintain API compatibility
}

GantryError Gantry::moveTo(const JointConfig& joint,
                           uint32_t speed_mm_per_s,
                           uint32_t speed_deg_per_s,
                           uint32_t acceleration_mm_per_s2,
                           uint32_t deceleration_mm_per_s2) {
    if (!initialized_) {
        return GantryError::NOT_INITIALIZED;
    }
    if (!enabled_) {
        return GantryError::MOTOR_NOT_ENABLED;
    }
    if (isBusy()) {
        return GantryError::ALREADY_MOVING;
    }
    
    // Validate joint limits
    if (!Kinematics::validate(joint, config_.limits)) {
        return GantryError::INVALID_POSITION;
    }
    
    // Store target positions
    targetY_ = (int32_t)joint.y;
    targetTheta_ = (int32_t)joint.theta;
    
    // Convert speed: mm/s to pulses/s for X-axis
    uint32_t speed_pps = (uint32_t)(speed_mm_per_s * 100.0f); // Approximate conversion
    
    // Call existing moveTo method
    moveTo((int32_t)joint.x, (int32_t)joint.y, (int32_t)joint.theta, speed_pps);
    
    return GantryError::OK;
}

GantryError Gantry::moveTo(const EndEffectorPose& pose,
                           uint32_t speed_mm_per_s,
                           uint32_t speed_deg_per_s,
                           uint32_t acceleration_mm_per_s2,
                           uint32_t deceleration_mm_per_s2) {
    if (!initialized_) {
        return GantryError::NOT_INITIALIZED;
    }
    if (!enabled_) {
        return GantryError::MOTOR_NOT_ENABLED;
    }
    if (isBusy()) {
        return GantryError::ALREADY_MOVING;
    }
    
    // Use inverse kinematics to get joint configuration
    JointConfig joint = inverseKinematics(pose);
    
    // Validate joint limits
    if (!Kinematics::validate(joint, config_.limits)) {
        return GantryError::INVALID_POSITION;
    }
    
    // Call moveTo with joint configuration
    return moveTo(joint, speed_mm_per_s, speed_deg_per_s, acceleration_mm_per_s2, deceleration_mm_per_s2);
}

bool Gantry::isBusy() const {
    if (!initialized_) {
        return false;
    }
    return axisX_.isMotionActive();
}

void Gantry::update() {
    if (!initialized_) {
        return;
    }
    
    // Update X-axis position from encoder
    // Y and Theta are simulated, so no update needed for now
    // TODO: Implement trajectory following for Y/Theta axes
}

// ============================================================================
// HOMING AND CALIBRATION
// ============================================================================

void Gantry::home() {
    if (!initialized_ || !enabled_) {
        return;
    }
    
    if (xMinPin_ < 0) {
        return; // No limit pin configured
    }
    
    // TODO: Implement homing sequence
    // This is a stub implementation
}

int Gantry::calibrate() {
    if (!initialized_ || !enabled_) {
        return 0;
    }
    
    if (xMaxPin_ < 0) {
        return 0; // No max limit pin configured
    }
    
    // TODO: Implement calibration sequence
    // This is a stub implementation
    return 0;
}

// ============================================================================
// GRIPPER CONTROL
// ============================================================================

void Gantry::grip(bool active) {
    if (!initialized_) {
        return;
    }
    
    if (gripperPin_ >= 0) {
        digitalWrite(gripperPin_, active ? HIGH : LOW);
        gripperActive_ = active;
    }
}

// ============================================================================
// ACCESSORS
// ============================================================================

int Gantry::getXEncoder() const {
    if (!initialized_) {
        return 0;
    }
    return axisX_.getEncoderPosition();
}

int Gantry::getCurrentY() const {
    return currentY_;
}

int Gantry::getCurrentTheta() const {
    return currentTheta_;
}

// ============================================================================
// ENHANCED KINEMATICS API (Phase 1.2)
// ============================================================================

EndEffectorPose Gantry::forwardKinematics(const JointConfig& joint) const {
    return Kinematics::forward(joint, kinematicParams_);
}

JointConfig Gantry::inverseKinematics(const EndEffectorPose& pose) const {
    return Kinematics::inverse(pose, kinematicParams_);
}

JointConfig Gantry::getCurrentJointConfig() const {
    JointConfig joint;
    
    // Get X position from encoder and convert to mm
    int32_t xPulses = getXEncoder();
    joint.x = pulsesToMm(xPulses);
    
    // Y and Theta are stored directly in mm/degrees
    joint.y = (float)currentY_;
    joint.theta = (float)currentTheta_;
    
    return joint;
}

JointConfig Gantry::getTargetJointConfig() const {
    JointConfig joint;
    
    // Target X position (for now, use current - will be updated when moveTo is implemented)
    int32_t xPulses = getXEncoder();
    joint.x = pulsesToMm(xPulses);
    
    // Y and Theta targets
    joint.y = (float)targetY_;
    joint.theta = (float)targetTheta_;
    
    return joint;
}

EndEffectorPose Gantry::getCurrentEndEffectorPose() const {
    JointConfig joint = getCurrentJointConfig();
    return forwardKinematics(joint);
}

EndEffectorPose Gantry::getTargetEndEffectorPose() const {
    JointConfig joint = getTargetJointConfig();
    return forwardKinematics(joint);
}

// ============================================================================
// HELPER METHODS
// ============================================================================

float Gantry::pulsesToMm(int32_t pulses) const {
    // Convert encoder pulses to mm
    // For a ball screw: mm = pulses * (pitch_mm / encoder_ppr)
    // Using default kinematic parameter for ball screw pitch (40mm)
    // Note: encoder_ppr should come from axisX_ config, but using default for now
    // This will need to be updated when encoder configuration is available
    const float defaultEncoderPPR = 2500.0f; // Default encoder PPR
    const float pitch = kinematicParams_.x_axis_ball_screw_pitch_mm;
    return (float)pulses * (pitch / defaultEncoderPPR);
}

int32_t Gantry::mmToPulses(float mm) const {
    // Convert mm to encoder pulses
    const float defaultEncoderPPR = 2500.0f; // Default encoder PPR
    const float pitch = kinematicParams_.x_axis_ball_screw_pitch_mm;
    return (int32_t)(mm * (defaultEncoderPPR / pitch));
}

} // namespace Gantry

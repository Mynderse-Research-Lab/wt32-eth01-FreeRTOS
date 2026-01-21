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
      axisLength_(0), config_(), kinematicParams_(),
      stepsPerRev_(6000.0f) {  // Default: 6000 steps/rev (common servo)
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
    
    // Configure limit switch pins in driver config BEFORE initialization
    // This allows the driver to handle limit switch debouncing and safety
    if (xMinPin_ >= 0 && xMaxPin_ >= 0) {
        // Get mutable reference to driver config
        BergerdaServo::DriverConfig& xConfig = 
            const_cast<BergerdaServo::DriverConfig&>(axisX_.getConfig());
        
        // Set limit pins in driver config
        xConfig.limit_min_pin = xMinPin_;
        xConfig.limit_max_pin = xMaxPin_;
        
        // Configure limit switch pins (driver will also configure them during initialize)
        pinMode(xMinPin_, INPUT_PULLUP);
        pinMode(xMaxPin_, INPUT_PULLUP);
    }
    
    // Initialize X-axis servo driver (will configure limit switches if pins are set)
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
    
    // Store target positions for Y and Theta (simulated)
    targetY_ = y;
    targetTheta_ = theta;
    currentY_ = y;  // Simulated - instant update
    currentTheta_ = theta;  // Simulated - instant update
    
    // Calculate X-axis movement in pulses
    int32_t currentX_pulses = axisX_.getPosition();
    int32_t targetX_pulses = mmToPulses((float)x);
    int32_t deltaX = targetX_pulses - currentX_pulses;
    
    // Check limit switches before movement
    axisX_.updateLimitDebounce(true);
    if (deltaX > 0 && axisX_.getLimitMaxDebounced()) {
        return; // MAX limit active - cannot move forward
    }
    if (deltaX < 0 && axisX_.getLimitMinDebounced()) {
        return; // MIN limit active - cannot move backward
    }
    
    // Move X-axis using driver (if delta is non-zero)
    if (deltaX != 0) {
        uint32_t accel = speed / 2;
        uint32_t decel = speed / 2;
        axisX_.moveRelative(deltaX, speed, accel, decel);
    }
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
    
    // Check for alarm condition
    if (axisX_.isAlarmActive()) {
        return GantryError::TIMEOUT; // Or could add GantryError::ALARM
    }
    
    // Validate joint limits
    if (!Kinematics::validate(joint, config_.limits)) {
        return GantryError::INVALID_POSITION;
    }
    
    // Update limit debouncing before movement check
    axisX_.updateLimitDebounce(true);
    
    // Get current X position and calculate delta
    int32_t currentX_pulses = axisX_.getPosition();
    int32_t targetX_pulses = mmToPulses(joint.x);
    int32_t deltaX = targetX_pulses - currentX_pulses;
    
    // Check limit switches before allowing movement
    // Driver will also check, but explicit check provides better error reporting
    if (deltaX > 0 && axisX_.getLimitMaxDebounced()) {
        // Trying to move forward but MAX limit is active
        return GantryError::LIMIT_SWITCH_FAILED;
    }
    if (deltaX < 0 && axisX_.getLimitMinDebounced()) {
        // Trying to move backward but MIN limit is active
        return GantryError::LIMIT_SWITCH_FAILED;
    }
    
    // Store target positions
    targetY_ = (int32_t)joint.y;
    targetTheta_ = (int32_t)joint.theta;
    
    // Convert speed: mm/s to pulses/s for X-axis
    uint32_t speed_pps = (uint32_t)(speed_mm_per_s * 100.0f); // Approximate conversion
    
    // Convert acceleration/deceleration if provided
    uint32_t accel_pps = 0;
    uint32_t decel_pps = 0;
    if (acceleration_mm_per_s2 > 0) {
        accel_pps = (uint32_t)(acceleration_mm_per_s2 * 100.0f);
    }
    if (deceleration_mm_per_s2 > 0) {
        decel_pps = (uint32_t)(deceleration_mm_per_s2 * 100.0f);
    }
    
    // Use default if not specified
    if (accel_pps == 0) accel_pps = speed_pps / 2;
    if (decel_pps == 0) decel_pps = speed_pps / 2;
    
    // Move X-axis using driver (will also check limits internally)
    if (!axisX_.moveRelative(deltaX, speed_pps, accel_pps, decel_pps)) {
        return GantryError::INVALID_PARAMETER;
    }
    
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
    // Check if X-axis is moving OR homing
    return axisX_.isMotionActive() || axisX_.isHoming();
}

void Gantry::update() {
    if (!initialized_) {
        return;
    }
    
    // CRITICAL: Update limit switch debouncing every loop
    // This prevents false triggers from mechanical switch bounce
    axisX_.updateLimitDebounce();
    
    // Check for alarm condition
    if (axisX_.isAlarmActive()) {
        // Handle alarm - stop motion and disable
        if (isBusy()) {
            axisX_.stopMotion(0);
        }
        enabled_ = false;
        // Note: Could add alarm callback or status flag here
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
    
    // Update limit debouncing to get current state
    axisX_.updateLimitDebounce(true);
    
    // Check if already at home (MIN limit active)
    if (axisX_.getLimitMinDebounced()) {
        // Already at home - set position to 0
        axisX_.setPosition(0);
        return;
    }
    
    // Use driver's built-in homing sequence
    // Moves negative until MIN limit switch is triggered
    // Uses config.homing_speed_pps (default 6000 pps)
    uint32_t homing_speed = axisX_.getConfig().homing_speed_pps;
    if (homing_speed == 0) {
        homing_speed = 6000; // Default fallback
    }
    
    // Start homing (non-blocking)
    // Position will be set to 0 automatically when MIN limit is reached
    axisX_.startHoming(homing_speed);
}

int Gantry::calibrate() {
    if (!initialized_ || !enabled_) {
        return 0;
    }
    
    if (xMinPin_ < 0 || xMaxPin_ < 0) {
        return 0; // Both limit pins required for calibration
    }
    
    // Ensure we're at MIN limit (home) first
    axisX_.updateLimitDebounce(true);
    if (!axisX_.getLimitMinDebounced()) {
        // Not at home - home first
        home();
        
        // Wait for homing to complete (with timeout)
        unsigned long start_ms = millis();
        while (axisX_.isHoming() && (millis() - start_ms) < 30000) {
            axisX_.updateLimitDebounce();
            delay(10);
        }
        
        if (axisX_.isHoming()) {
            return 0; // Homing timeout
        }
    }
    
    // Now measure travel distance from MIN to MAX
    uint32_t speed = axisX_.getConfig().homing_speed_pps;
    if (speed == 0) {
        speed = 6000; // Default fallback
    }
    
    // Start travel measurement (non-blocking)
    // This will move from MIN to MAX and measure distance
    if (!axisX_.measureTravelDistance(speed, speed, speed, 90000)) {
        return 0; // Failed to start measurement
    }
    
    // Wait for measurement to complete (with timeout)
    unsigned long start_ms = millis();
    while (!axisX_.hasTravelDistance() && (millis() - start_ms) < 90000) {
        axisX_.updateLimitDebounce();
        delay(10);
        
        // Check if measurement failed (alarm, etc.)
        if (axisX_.isAlarmActive()) {
            return 0; // Alarm during measurement
        }
    }
    
    if (!axisX_.hasTravelDistance()) {
        return 0; // Measurement timeout
    }
    
    // Get travel distance in pulses
    uint32_t travel_pulses = axisX_.getTravelDistance();
    
    // Convert to mm and store
    axisLength_ = pulsesToMm(travel_pulses);
    
    return axisLength_;
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
// ALARM MONITORING
// ============================================================================

bool Gantry::isAlarmActive() const {
    if (!initialized_) {
        return false;
    }
    return axisX_.isAlarmActive();
}

void Gantry::setHomingSpeed(uint32_t speed_pps) {
    if (initialized_) {
        BergerdaServo::DriverConfig& config = 
            const_cast<BergerdaServo::DriverConfig&>(axisX_.getConfig());
        config.homing_speed_pps = speed_pps;
    }
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

void Gantry::setStepsPerRevolution(float steps_per_rev) {
    if (steps_per_rev > 0) {
        stepsPerRev_ = steps_per_rev;
    }
}

float Gantry::getPulsesPerMm() const {
    // Calculate pulses per mm based on motor steps and ball screw pitch
    // Formula: pulses_per_mm = steps_per_rev / pitch_mm
    // Example: 6000 steps/rev / 40mm = 150 pulses/mm
    const float pitch = kinematicParams_.x_axis_ball_screw_pitch_mm;
    if (pitch <= 0) return 150.0f;  // Default fallback
    return stepsPerRev_ / pitch;
}

float Gantry::pulsesToMm(int32_t pulses) const {
    // Convert motor pulses to mm
    // Formula: mm = pulses / pulses_per_mm
    // For ball screw: mm = pulses * (pitch_mm / steps_per_rev)
    // Example: 150 pulses * (40mm / 6000) = 1mm
    float pulsesPerMm = getPulsesPerMm();
    if (pulsesPerMm <= 0) return 0.0f;
    return (float)pulses / pulsesPerMm;
}

int32_t Gantry::mmToPulses(float mm) const {
    // Convert mm to motor pulses
    // Formula: pulses = mm * pulses_per_mm
    // Example: 1mm * 150 = 150 pulses
    return (int32_t)(mm * getPulsesPerMm());
}

} // namespace Gantry

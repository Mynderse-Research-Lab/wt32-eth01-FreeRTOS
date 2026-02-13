/**
 * @file GANTRY_INTEGRATION_EXAMPLE.cpp
 * @brief Example implementation showing integration of driver library features
 * 
 * This file shows how to integrate the new SDF08NK8X driver features into Gantry.
 * Copy relevant sections into Gantry.cpp and adapt as needed.
 */

#include "Gantry.h"

namespace Gantry {

// ============================================================================
// EXAMPLE 1: Enhanced begin() with limit switch configuration
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
    if (xMinPin_ >= 0 && xMaxPin_ >= 0) {
        // Get mutable reference to driver config
        BergerdaServo::DriverConfig& xConfig = 
            const_cast<BergerdaServo::DriverConfig&>(axisX_.getConfig());
        
        // Set limit pins
        xConfig.limit_min_pin = xMinPin_;
        xConfig.limit_max_pin = xMaxPin_;
        
        // Configure limit switch pins (driver will also configure them)
        pinMode(xMinPin_, INPUT_PULLUP);
        pinMode(xMaxPin_, INPUT_PULLUP);
        
        // Optional: Configure GPIO interrupts for immediate detection
        // Note: Requires careful ISR design - see Example 6
    }
    
    // Initialize X-axis servo driver (will configure limit switches)
    if (!axisX_.initialize()) {
        return false;
    }
    
    initialized_ = true;
    return true;
}

// ============================================================================
// EXAMPLE 2: Implement home() using driver API
// ============================================================================

void Gantry::home() {
    if (!initialized_ || !enabled_) {
        return;
    }
    
    if (xMinPin_ < 0) {
        return; // No limit pin configured
    }
    
    // Check if already at home (MIN limit active)
    axisX_.updateLimitDebounce(true);
    if (axisX_.getLimitMinDebounced()) {
        // Already at home - set position to 0
        axisX_.setPosition(0);
        return;
    }
    
    // Use driver's built-in homing
    // Uses config.homing_speed_pps (default 6000 pps)
    uint32_t homing_speed = axisX_.getConfig().homing_speed_pps;
    if (homing_speed == 0) {
        homing_speed = 6000; // Default fallback
    }
    
    axisX_.startHoming(homing_speed);
    
    // Homing is non-blocking - check isHoming() in update()
    // Position will be set to 0 automatically when MIN limit is reached
}

// ============================================================================
// EXAMPLE 3: Implement calibrate() using travel measurement
// ============================================================================

int Gantry::calibrate() {
    if (!initialized_ || !enabled_) {
        return 0;
    }
    
    if (xMinPin_ < 0 || xMaxPin_ < 0) {
        return 0; // Both limit pins required for calibration
    }
    
    // Ensure we're at MIN limit (home)
    axisX_.updateLimitDebounce(true);
    if (!axisX_.getLimitMinDebounced()) {
        // Not at home - home first
        home();
        
        // Wait for homing to complete (with timeout)
        unsigned long start_ms = millis();
        while (axisX_.isHoming() && (millis() - start_ms) < 35000) {
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
    
    // Wait for measurement to complete (check in update() or poll here)
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
// EXAMPLE 4: Enhanced update() with debouncing
// ============================================================================

void Gantry::update() {
    if (!initialized_) {
        return;
    }
    
    // CRITICAL: Update limit switch debouncing every loop
    axisX_.updateLimitDebounce();
    
    // Check for alarm condition
    if (axisX_.isAlarmActive()) {
        // Handle alarm - stop motion and disable
        if (isBusy()) {
            axisX_.stopMotion(0);
        }
        enabled_ = false;
        // Could add alarm callback or status flag here
    }
    
    // Update X-axis position from encoder
    // Y and Theta are simulated, so no update needed for now
    // TODO: Implement trajectory following for Y/Theta axes
    
    // Check homing status (if homing was started)
    if (axisX_.isHoming()) {
        // Homing in progress - driver handles it automatically
        // Position will be set to 0 when MIN limit is reached
    }
}

// ============================================================================
// EXAMPLE 5: Enhanced moveTo() with limit safety checks
// ============================================================================

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
    
    // Check for alarm
    if (axisX_.isAlarmActive()) {
        return GantryError::TIMEOUT; // Or add GantryError::ALARM
    }
    
    // Validate joint limits
    if (!Kinematics::validate(joint, config_.limits)) {
        return GantryError::INVALID_POSITION;
    }
    
    // Update limit debouncing before movement check
    axisX_.updateLimitDebounce(true);
    
    // Get current X position
    int32_t currentX_pulses = axisX_.getPosition();
    int32_t targetX_pulses = mmToPulses(joint.x);
    int32_t deltaX = targetX_pulses - currentX_pulses;
    
    // Check limit switches before allowing movement
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
    
    // Move X-axis using driver
    if (!axisX_.moveRelative(deltaX, speed_pps, accel_pps, decel_pps)) {
        return GantryError::INVALID_PARAMETER;
    }
    
    return GantryError::OK;
}

// ============================================================================
// EXAMPLE 6: GPIO Interrupt Support (Advanced)
// ============================================================================

// Note: This requires careful design due to ISR context limitations
// Option A: Static callback with instance pointer

static Gantry* gantry_instance = nullptr;

void IRAM_ATTR onLimitMinIrq() {
    if (gantry_instance) {
        gantry_instance->axisX_.notifyLimitIrq();
    }
}

void IRAM_ATTR onLimitMaxIrq() {
    if (gantry_instance) {
        gantry_instance->axisX_.notifyLimitIrq();
    }
}

bool Gantry::begin() {
    // ... existing initialization ...
    
    // Set instance pointer for ISR callbacks
    gantry_instance = this;
    
    // Configure GPIO interrupts
    if (xMinPin_ >= 0) {
        attachInterrupt(digitalPinToInterrupt(xMinPin_), onLimitMinIrq, CHANGE);
    }
    if (xMaxPin_ >= 0) {
        attachInterrupt(digitalPinToInterrupt(xMaxPin_), onLimitMaxIrq, CHANGE);
    }
    
    // ... rest of initialization ...
}

// Option B: Lambda with capture (may not work in ISR context)
// Better to use Option A with static callback

// ============================================================================
// EXAMPLE 7: Enhanced isBusy() to include homing status
// ============================================================================

bool Gantry::isBusy() const {
    if (!initialized_) {
        return false;
    }
    // Check if X-axis is moving OR homing
    return axisX_.isMotionActive() || axisX_.isHoming();
}

// ============================================================================
// EXAMPLE 8: Add alarm monitoring method
// ============================================================================

bool Gantry::isAlarmActive() const {
    if (!initialized_) {
        return false;
    }
    return axisX_.isAlarmActive();
}

// ============================================================================
// EXAMPLE 9: Add homing speed configuration
// ============================================================================

void Gantry::setHomingSpeed(uint32_t speed_pps) {
    if (initialized_) {
        BergerdaServo::DriverConfig& config = 
            const_cast<BergerdaServo::DriverConfig&>(axisX_.getConfig());
        config.homing_speed_pps = speed_pps;
    }
}

// ============================================================================
// EXAMPLE 10: Enhanced GantryStatus with alarm
// ============================================================================

// In Gantry.h, add to GantryStatus struct:
/*
struct GantryStatus {
    // ... existing fields ...
    bool alarmActive;  // Add this field
};
*/

// In getStatus() or similar method:
GantryStatus Gantry::getStatus() const {
    GantryStatus status;
    // ... populate existing fields ...
    status.alarmActive = isAlarmActive();
    return status;
}

} // namespace Gantry

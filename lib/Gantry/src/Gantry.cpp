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
      currentY_(0), currentTheta_(0), axisLength_(0) {
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

} // namespace Gantry

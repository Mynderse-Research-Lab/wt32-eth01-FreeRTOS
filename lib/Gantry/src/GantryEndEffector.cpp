/**
 * @file GantryEndEffector.cpp
 * @brief End-effector implementation
 * @version 1.0.0
 */

#include "GantryEndEffector.h"

namespace Gantry {

GantryEndEffector::GantryEndEffector()
    : pin_(-1), activeHigh_(true), configured_(false), active_(false) {}

void GantryEndEffector::configurePin(int pin, bool activeHigh) {
    pin_ = pin;
    activeHigh_ = activeHigh;
    configured_ = (pin_ >= 0);
}

bool GantryEndEffector::begin() {
    if (!configured_) {
        return false;
    }
    pinMode(pin_, OUTPUT);
    setActive(false);
    return true;
}

void GantryEndEffector::setActive(bool active) {
    if (!configured_) {
        return;
    }
    digitalWrite(pin_, active == activeHigh_ ? HIGH : LOW);
    active_ = active;
}

} // namespace Gantry

/**
 * @file GantryEndEffector.h
 * @brief Simple digital end-effector (gripper) control
 * @version 1.0.0
 */

#ifndef GANTRY_END_EFFECTOR_H
#define GANTRY_END_EFFECTOR_H

#include <Arduino.h>

namespace Gantry {

class GantryEndEffector {
public:
    GantryEndEffector();

    void configurePin(int pin, bool activeHigh = true);
    bool begin();
    bool isConfigured() const { return configured_; }

    void setActive(bool active);
    bool isActive() const { return active_; }
    int getPin() const { return pin_; }

private:
    int pin_;
    bool activeHigh_;
    bool configured_;
    bool active_;
};

} // namespace Gantry

#endif // GANTRY_END_EFFECTOR_H

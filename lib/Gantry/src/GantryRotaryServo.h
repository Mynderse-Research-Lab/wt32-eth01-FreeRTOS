/**
 * @file GantryRotaryServo.h
 * @brief Simple PWM servo driver for inline rotary axis
 * @version 1.0.0
 */

#ifndef GANTRY_ROTARY_SERVO_H
#define GANTRY_ROTARY_SERVO_H

#include <Arduino.h>
#include <stdint.h>

#if !defined(ARDUINO_ARCH_ESP32)
#include <Servo.h>
#endif

namespace Gantry {

/**
 * @class GantryRotaryServo
 * @brief PWM servo control with configurable pulse/angle ranges
 */
class GantryRotaryServo {
public:
    GantryRotaryServo();

    void configurePin(int pwmPin, int pwmChannel = 0);
    void setAngleRange(float minDeg, float maxDeg);
    void setPulseRange(uint16_t minPulseUs, uint16_t maxPulseUs);
    bool begin();

    bool isConfigured() const { return configured_; }
    bool isActive() const { return active_; }

    bool moveToDeg(float angleDeg);
    float getCurrentDeg() const { return currentDeg_; }

private:
    void writePulseUs(uint16_t pulseUs);

    int pwmPin_;
    int pwmChannel_;
    bool configured_;
    bool active_;

    float minDeg_;
    float maxDeg_;
    uint16_t minPulseUs_;
    uint16_t maxPulseUs_;
    float currentDeg_;

#if !defined(ARDUINO_ARCH_ESP32)
    Servo servo_;
    bool servoAttached_;
#endif
};

} // namespace Gantry

#endif // GANTRY_ROTARY_SERVO_H

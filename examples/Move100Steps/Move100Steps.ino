/**
 * @file Move100Steps.ino
 * @brief Example: Move a pulse+direction motor 100 steps using internal
 *        step counting (no encoder feedback).
 *
 * Demonstrates:
 * 1. Driver initialization (no encoder feedback)
 * 2. Motor enable
 * 3. +100-pulse move
 * 4. Waiting for motion complete
 * 5. -100-pulse move back to start
 *
 * Works against any pulse+direction driver (verified: Bergerda SDF08NK8X,
 * Allen-Bradley Kinetix 5100, custom pulse-train driver for SCHUNK ERD
 * 04-40-D-H-N).
 *
 * Hardware Requirements:
 * - WT32-ETH01 or ESP32 development board
 * - Pulse+direction motor driver + motor
 *
 * Pin Connections (WT32-ETH01):
 * - GPIO 2  -> Driver PULSE input
 * - GPIO 4  -> Driver DIR input
 * - GPIO 12 -> Driver ENABLE / SON input
 * - GPIO 14 -> Limit switch MIN (active LOW)
 * - GPIO 32 -> Limit switch MAX (active LOW)
 */

#include "PulseMotor.h"

using namespace PulseMotor;

#define PIN_PULSE     2
#define PIN_DIR       4
#define PIN_ENABLE   12
#define PIN_LIMIT_MIN 14
#define PIN_LIMIT_MAX 32

DriverConfig driverConfig;
PulseMotorDriver* driver = nullptr;

const uint32_t STEPS_TO_MOVE = 100;
const uint32_t MAX_SPEED     = 5000;
const uint32_t ACCELERATION  = 2000;
const uint32_t DECELERATION  = 2000;

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n========================================");
    Serial.println("Move 100 Steps Example (PulseMotor)");
    Serial.println("========================================\n");

    driverConfig.pulse_pin               = PIN_PULSE;
    driverConfig.dir_pin                 = PIN_DIR;
    driverConfig.enable_pin              = PIN_ENABLE;
    driverConfig.enable_encoder_feedback = false;
    driverConfig.pulse_mode              = PulseMode::PULSE_DIRECTION;

    pinMode(PIN_LIMIT_MIN, INPUT_PULLUP);
    pinMode(PIN_LIMIT_MAX, INPUT_PULLUP);

    driver = new PulseMotorDriver(driverConfig);

    Serial.println("Initializing driver...");
    if (!driver->initialize()) {
        Serial.println("ERROR: Driver initialization failed!");
        return;
    }
    Serial.println("Driver initialized");

    Serial.println("Enabling motor...");
    if (!driver->enable()) {
        Serial.println("ERROR: Failed to enable motor!");
        return;
    }
    Serial.println("Motor enabled");

    uint32_t initialPosition = driver->getPosition();
    bool limitMin = !digitalRead(PIN_LIMIT_MIN);
    bool limitMax = !digitalRead(PIN_LIMIT_MAX);
    Serial.printf("\nInitial Position: %u steps\n", initialPosition);
    Serial.printf("Limit Switches: MIN=%s MAX=%s\n",
                  limitMin ? "ACTIVE" : "open",
                  limitMax ? "ACTIVE" : "open");
    Serial.println("\nReady to move 100 steps...\n");
}

void loop() {
    if (driver == nullptr || !driver->isEnabled()) {
        Serial.println("ERROR: Driver not initialized or motor not enabled!");
        delay(5000);
        return;
    }

    if (driver->isMotionActive()) {
        delay(10);
        return;
    }

    uint32_t currentPos = driver->getPosition();
    bool limitMin = !digitalRead(PIN_LIMIT_MIN);
    bool limitMax = !digitalRead(PIN_LIMIT_MAX);

    Serial.println("----------------------------------------");
    Serial.printf("Current Position: %u steps\n", currentPos);
    Serial.printf("Limit Switches: MIN=%s MAX=%s\n",
                  limitMin ? "ACTIVE" : "open",
                  limitMax ? "ACTIVE" : "open");

    if (limitMax) {
        Serial.println("MAX limit active - skipping forward move");
        delay(2000);
        return;
    }

    Serial.printf("\nMoving +%u steps...\n", STEPS_TO_MOVE);
    if (driver->moveRelative(STEPS_TO_MOVE, MAX_SPEED, ACCELERATION, DECELERATION)) {
        while (driver->isMotionActive()) {
            delay(10);
        }
        Serial.printf("Final Position: %u steps\n", driver->getPosition());
    } else {
        Serial.println("Forward move rejected");
    }

    delay(2000);

    currentPos = driver->getPosition();
    limitMin = !digitalRead(PIN_LIMIT_MIN);
    if (limitMin) {
        Serial.println("MIN limit active - skipping return move");
        delay(2000);
        return;
    }

    Serial.printf("\nMoving -%u steps (back to start)...\n", STEPS_TO_MOVE);
    if (driver->moveRelative(-(int64_t)STEPS_TO_MOVE, MAX_SPEED, ACCELERATION, DECELERATION)) {
        while (driver->isMotionActive()) {
            if (!digitalRead(PIN_LIMIT_MIN)) {
                Serial.println("\nMIN limit triggered during motion. Stopping.");
                driver->stopMotion(DECELERATION);
                delay(500);
                break;
            }
            delay(10);
        }
        Serial.printf("Final Position: %u steps\n", driver->getPosition());
    } else {
        Serial.println("Return move rejected");
    }

    delay(3000);
}

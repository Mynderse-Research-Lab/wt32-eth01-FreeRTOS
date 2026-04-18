/**
 * @file BasicDriverTest.ino
 * @brief Very basic test program for the PulseMotor driver library.
 *
 * Verifies end-to-end initialization, enable, status read, and a short move
 * against any pulse+direction motor driver (verified: Bergerda SDF08NK8X,
 * Allen-Bradley Kinetix 5100, custom pulse-train driver for SCHUNK
 * ERD 04-40-D-H-N).
 *
 * Hardware Requirements:
 * - WT32-ETH01 or ESP32
 * - One pulse+direction motor driver
 * - Motor (optional for the basic tests; required for the move test)
 *
 * Pin Connections:
 * - GPIO 2  -> Driver PULSE input
 * - GPIO 4  -> Driver DIR input
 * - GPIO 12 -> Driver ENABLE / SON input
 * - GPIO 14 -> Limit switch MIN (Home position, active LOW)
 * - GPIO 32 -> Limit switch MAX (End position, active LOW)
 */

#include <PulseMotor.h>

using namespace PulseMotor;

#define PIN_PULSE     2
#define PIN_DIR       4
#define PIN_ENABLE   12
#define PIN_LIMIT_MIN 14
#define PIN_LIMIT_MAX 32

DriverConfig config;
PulseMotorDriver* driver = nullptr;

void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("\n========================================");
    Serial.println("Basic PulseMotor Library Test");
    Serial.println("========================================\n");

    // Named pin fields - no slot-index encoding.
    config.pulse_pin               = PIN_PULSE;
    config.dir_pin                 = PIN_DIR;
    config.enable_pin              = PIN_ENABLE;
    config.enable_encoder_feedback = false;
    config.pulse_mode              = PulseMode::PULSE_DIRECTION;

    pinMode(PIN_LIMIT_MIN, INPUT_PULLUP);
    pinMode(PIN_LIMIT_MAX, INPUT_PULLUP);

    driver = new PulseMotorDriver(config);

    Serial.println("Test 1: Driver Initialization");
    Serial.println("-------------------------------");
    if (driver->initialize()) {
        Serial.println("PASS: Driver initialized successfully");
    } else {
        Serial.println("FAIL: Driver initialization failed");
        Serial.println("      Check pin connections and try again.");
        return;
    }

    Serial.println("\nTest 2: Motor Enable");
    Serial.println("-------------------------------");
    if (driver->enable()) {
        Serial.println("PASS: Motor enabled successfully");
    } else {
        Serial.println("FAIL: Motor enable failed");
        Serial.println("      Check if motor is connected and driver is powered.");
        return;
    }

    Serial.println("\nTest 3: Status Check");
    Serial.println("-------------------------------");
    DriveStatus status = driver->getStatus();
    Serial.printf("  Servo Enabled: %s\n", status.servo_enabled ? "YES" : "NO");
    Serial.printf("  Motion Active: %s\n", driver->isMotionActive() ? "YES" : "NO");
    Serial.printf("  Current Position: %u steps\n", driver->getPosition());
    Serial.printf("  Current Speed: %u pps\n", driver->getSpeed());
    Serial.println("PASS: Status read successfully");

    Serial.println("\nTest 4: Enable State Check");
    Serial.println("-------------------------------");
    Serial.println(driver->isEnabled() ? "PASS: Driver reports enabled"
                                       : "FAIL: Driver should be enabled but is not");

    Serial.println("\nTest 5: Limit Switch Reading");
    Serial.println("-------------------------------");
    bool limitMin = !digitalRead(PIN_LIMIT_MIN);
    bool limitMax = !digitalRead(PIN_LIMIT_MAX);
    Serial.printf("  Limit MIN (Home): %s\n", limitMin ? "ACTIVE (LOW)" : "open (HIGH)");
    Serial.printf("  Limit MAX (End):  %s\n", limitMax ? "ACTIVE (LOW)" : "open (HIGH)");

    Serial.println("\nTest 6: Short Forward Move");
    Serial.println("-------------------------------");
    if (limitMax) {
        Serial.println("Skipping move - MAX limit active");
    } else {
        Serial.println("Moving 3000 pulses forward...");
        if (driver->moveRelative(3000, 1000, 500, 500)) {
            Serial.println("Move command accepted");
            unsigned long startTime = millis();
            while (driver->isMotionActive()) {
                if (!digitalRead(PIN_LIMIT_MAX) || !digitalRead(PIN_LIMIT_MIN)) {
                    Serial.println("\nLIMIT SWITCH TRIGGERED - stopping motor.");
                    driver->stopMotion(50000);
                    delay(500);
                    break;
                }
                delay(2);
                if (millis() - startTime > 5000) {
                    Serial.println("Motion taking longer than expected");
                    break;
                }
            }
            Serial.printf("Final Position: %u pulses\n", driver->getPosition());
        } else {
            Serial.println("Move command rejected");
        }
    }

    Serial.println("\n========================================");
    Serial.println("Basic PulseMotor tests complete.");
    Serial.println("========================================");
}

void loop() {
    static unsigned long lastCheck = 0;
    if (millis() - lastCheck >= 5000) {
        if (driver && driver->isEnabled()) {
            DriveStatus status = driver->getStatus();
            bool limitMin = !digitalRead(PIN_LIMIT_MIN);
            bool limitMax = !digitalRead(PIN_LIMIT_MAX);
            Serial.printf("[Status] Position: %u | Speed: %u | Motion: %s | Limits: MIN=%s MAX=%s\n",
                          status.current_position, status.current_speed,
                          driver->isMotionActive() ? "YES" : "NO",
                          limitMin ? "ACTIVE" : "open",
                          limitMax ? "ACTIVE" : "open");
        }
        lastCheck = millis();
    }
    delay(100);
}

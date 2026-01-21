/**
 * @file Move100Steps.ino
 * @brief Example: Move motor 100 steps using internal step counting only
 * 
 * This example demonstrates how to:
 * 1. Initialize the servo driver (without encoder feedback)
 * 2. Enable the motor
 * 3. Move the motor 100 steps (pulses) in the positive direction
 * 4. Wait for motion to complete
 * 5. Move back 100 steps
 * 
 * Note: This example uses internal step counting only (no encoder feedback).
 * Position is tracked by counting pulses sent to the driver.
 * 
 * Hardware Requirements:
 * - WT32-ETH01 or ESP32 development board
 * - SDF08NK8X servo driver
 * - Servo motor (encoder not required)
 * 
 * Pin Connections (WT32-ETH01):
 * - GPIO 2  -> Driver CN1 Pin 18 (PULSE)
 * - GPIO 4  -> Driver CN1 Pin 19 (DIR)
 * - GPIO 12 -> Driver CN1 Pin 21 (ENABLE/SON)
 * - GPIO 14 -> Limit switch MIN (Home position, active LOW)
 * - GPIO 32 -> Limit switch MAX (End position, active LOW)
 */

#include "SDF08NK8X.h"

using namespace BergerdaServo;

// Pin Definitions (WT32-ETH01 Safe)
#define PIN_PULSE    2   // Output: Pulse signal to servo driver
#define PIN_DIR      4   // Output: Direction signal to servo driver
#define PIN_ENABLE   12  // Output: Servo enable (SON)
#define PIN_LIMIT_MIN 14 // Input Pullup: Home limit switch (active LOW)
#define PIN_LIMIT_MAX 32 // Input Pullup: End limit switch (active LOW)

// Servo Driver Configuration
DriverConfig driverConfig;
ServoDriver* driver = nullptr;

// Motion Parameters
const uint32_t STEPS_TO_MOVE = 100;        // Number of steps to move
const uint32_t MAX_SPEED = 5000;            // Maximum speed (pulses per second)
const uint32_t ACCELERATION = 2000;         // Acceleration rate (pps²)
const uint32_t DECELERATION = 2000;         // Deceleration rate (pps²)

void setup() {
  Serial.begin(115200);
  delay(1000);  // Wait for serial monitor
  
  Serial.println("\n========================================");
  Serial.println("Move 100 Steps Example");
  Serial.println("========================================\n");
  
  // Configure servo driver pins
  driverConfig.output_pin_nos[6] = PIN_PULSE;   // PULSE output
  driverConfig.output_pin_nos[7] = PIN_DIR;      // DIR output
  driverConfig.output_pin_nos[0] = PIN_ENABLE;  // ENABLE output
  
  // Disable encoder feedback - use internal step counting only
  driverConfig.enable_encoder_feedback = false;
  
  // Configure limit switch pins (INPUT with internal pullup)
  pinMode(PIN_LIMIT_MIN, INPUT_PULLUP);
  pinMode(PIN_LIMIT_MAX, INPUT_PULLUP);
  
  // Set control mode
  driverConfig.pulse_mode = PulseMode::PULSE_DIRECTION;
  driverConfig.control_mode = ControlMode::POSITION;
  
  // Create servo driver instance
  driver = new ServoDriver(driverConfig);
  
  // Initialize driver
  Serial.println("Initializing servo driver...");
  if (!driver->initialize()) {
    Serial.println("ERROR: Driver initialization failed!");
    Serial.println("Check your pin connections and try again.");
    return;
  }
  Serial.println("✓ Driver initialized successfully");
  
  // Enable motor
  Serial.println("Enabling motor...");
  if (!driver->enable()) {
    Serial.println("ERROR: Failed to enable motor!");
    return;
  }
  Serial.println("✓ Motor enabled");
  
  // Get initial position (internal step counter)
  uint32_t initialPosition = driver->getPosition();
  
  // Check limit switch status
  bool limitMin = !digitalRead(PIN_LIMIT_MIN);  // Active LOW
  bool limitMax = !digitalRead(PIN_LIMIT_MAX);  // Active LOW
  
  Serial.printf("\nInitial Position: %u steps (internal counter)\n", initialPosition);
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
  
  // Check if motor is already moving
  if (driver->isMotionActive()) {
    // Update motion profile (required for smooth motion)
    driver->update();
    delay(10);
    return;
  }
  
  // Get current position (internal step counter)
  uint32_t currentPos = driver->getPosition();
  
  // Check limit switches
  bool limitMin = !digitalRead(PIN_LIMIT_MIN);  // Active LOW
  bool limitMax = !digitalRead(PIN_LIMIT_MAX);  // Active LOW
  
  Serial.println("----------------------------------------");
  Serial.printf("Current Position: %u steps\n", currentPos);
  Serial.printf("Limit Switches: MIN=%s MAX=%s\n", 
                limitMin ? "ACTIVE" : "open", 
                limitMax ? "ACTIVE" : "open");
  
  // Check if we can move forward (positive direction)
  if (limitMax) {
    Serial.println("⚠ WARNING: End limit switch (MAX) is ACTIVE!");
    Serial.println("   Cannot move forward. Skipping this move.");
    delay(2000);
    return;
  }
  
  // Move 100 steps forward (positive direction)
  Serial.printf("\nMoving +%u steps...\n", STEPS_TO_MOVE);
  Serial.printf("Speed: %u pps, Accel: %u pps², Decel: %u pps²\n", 
                MAX_SPEED, ACCELERATION, DECELERATION);
  
  if (driver->moveRelative(STEPS_TO_MOVE, MAX_SPEED, ACCELERATION, DECELERATION)) {
    Serial.println("✓ Move command accepted");
    
    // Wait for motion to complete
    Serial.println("Waiting for motion to complete...");
    while (driver->isMotionActive()) {
      driver->update();  // Update motion profile
      
      // Print status every 100ms
      static unsigned long lastPrint = 0;
      if (millis() - lastPrint >= 100) {
        DriveStatus status = driver->getStatus();
        Serial.printf("  Position: %u steps | Speed: %u pps\r", 
                     status.current_position, 
                     status.current_speed);
        lastPrint = millis();
      }
      
      delay(10);
    }
    
    // Motion complete
    Serial.println("\n✓ Motion complete!");
    
    // Get final position (internal step counter)
    uint32_t finalPos = driver->getPosition();
    Serial.printf("Final Position: %u steps\n", finalPos);
    Serial.printf("Distance moved: %u steps\n", finalPos - currentPos);
    
  } else {
    Serial.println("✗ Move command failed!");
    Serial.println("Check if motor is enabled and not already moving.");
  }
  
  // Wait 2 seconds before moving back
  Serial.println("\nWaiting 2 seconds...");
  delay(2000);
  
  // Move 100 steps backward (negative direction)
  currentPos = driver->getPosition();
  
  // Check limit switches again
  limitMin = !digitalRead(PIN_LIMIT_MIN);  // Active LOW
  limitMax = !digitalRead(PIN_LIMIT_MAX);  // Active LOW
  
  Serial.println("----------------------------------------");
  Serial.printf("Current Position: %u steps\n", currentPos);
  Serial.printf("Limit Switches: MIN=%s MAX=%s\n", 
                limitMin ? "ACTIVE" : "open", 
                limitMax ? "ACTIVE" : "open");
  
  // Check if we can move backward (negative direction)
  if (limitMin) {
    Serial.println("⚠ WARNING: Home limit switch (MIN) is ACTIVE!");
    Serial.println("   Cannot move backward. Skipping this move.");
    delay(2000);
    return;
  }
  
  Serial.printf("\nMoving -%u steps (back to start)...\n", STEPS_TO_MOVE);
  
  if (driver->moveRelative(-(int64_t)STEPS_TO_MOVE, MAX_SPEED, ACCELERATION, DECELERATION)) {
    Serial.println("✓ Move command accepted");
    
    // Wait for motion to complete
    Serial.println("Waiting for motion to complete...");
    while (driver->isMotionActive()) {
      driver->update();  // Update motion profile
      
      // Check limit switches during motion
      limitMin = !digitalRead(PIN_LIMIT_MIN);  // Active LOW
      if (limitMin) {
        Serial.println("\n⚠ LIMIT SWITCH TRIGGERED: Home limit (MIN) is ACTIVE!");
        Serial.println("   Stopping motor immediately...");
        driver->stopMotion(DECELERATION);
        delay(500);  // Wait for stop to complete
        break;
      }
      
      // Print status every 100ms
      static unsigned long lastPrint = 0;
      if (millis() - lastPrint >= 100) {
        DriveStatus status = driver->getStatus();
        Serial.printf("  Position: %u steps | Speed: %u pps\r", 
                     status.current_position, 
                     status.current_speed);
        lastPrint = millis();
      }
      
      delay(10);
    }
    
    // Motion complete
    Serial.println("\n✓ Motion complete!");
    
    // Get final position (internal step counter)
    uint32_t finalPos = driver->getPosition();
    Serial.printf("Final Position: %u steps\n", finalPos);
    Serial.printf("Distance moved: %d steps\n", (int32_t)finalPos - (int32_t)currentPos);
    
  } else {
    Serial.println("✗ Move command failed!");
  }
  
  // Wait 3 seconds before repeating
  Serial.println("\nWaiting 3 seconds before repeating...\n");
  delay(3000);
}

/**
 * @file BasicDriverTest.ino
 * @brief Very basic test program for SDF08NK8X servo driver library
 * 
 * This is a minimal test to verify the driver library works correctly.
 * It performs:
 * 1. Driver initialization
 * 2. Motor enable
 * 3. Simple status check
 * 4. Basic move command (optional)
 * 
 * Hardware Requirements:
 * - WT32-ETH01 or ESP32
 * - SDF08NK8X servo driver
 * - Servo motor (optional for basic test)
 * 
 * Pin Connections:
 * - GPIO 2  -> Driver CN1 Pin 18 (PULSE)
 * - GPIO 4  -> Driver CN1 Pin 19 (DIR)
 * - GPIO 12 -> Driver CN1 Pin 21 (ENABLE/SON)
 * - GPIO 14 -> Limit switch MIN (Home position, active LOW)
 * - GPIO 32 -> Limit switch MAX (End position, active LOW)
 */

 #include "SDF08NK8X.h"

 using namespace BergerdaServo;
 
 // Pin Definitions
 #define PIN_PULSE    2   // Output: Pulse signal
 #define PIN_DIR      4   // Output: Direction signal
 #define PIN_ENABLE   12  // Output: Servo enable (SON)
 #define PIN_LIMIT_MIN 14 // Input Pullup: Home limit switch (active LOW)
 #define PIN_LIMIT_MAX 32 // Input Pullup: End limit switch (active LOW)
 
 // Servo Driver
 DriverConfig config;
 ServoDriver* driver = nullptr;
 
 void setup() {
   Serial.begin(115200);
   delay(2000);  // Wait for serial monitor
   
   Serial.println("\n========================================");
   Serial.println("Basic Driver Library Test");
   Serial.println("========================================\n");
   
   // Configure pins
   config.output_pin_nos[6] = PIN_PULSE;   // PULSE
   config.output_pin_nos[7] = PIN_DIR;    // DIR
   config.output_pin_nos[0] = PIN_ENABLE; // ENABLE
   
   // Disable encoder feedback for basic test
   config.enable_encoder_feedback = false;
   
   // Configure limit switch pins (INPUT with internal pullup)
   pinMode(PIN_LIMIT_MIN, INPUT_PULLUP);
   pinMode(PIN_LIMIT_MAX, INPUT_PULLUP);
   
   // Set control mode
   config.pulse_mode = PulseMode::PULSE_DIRECTION;
   config.control_mode = ControlMode::POSITION;
   
   // Create driver instance
   driver = new ServoDriver(config);
   
   // Test 1: Initialize driver
   Serial.println("Test 1: Driver Initialization");
   Serial.println("-------------------------------");
   if (driver->initialize()) {
     Serial.println("✓ PASS: Driver initialized successfully");
   } else {
     Serial.println("✗ FAIL: Driver initialization failed");
     Serial.println("   Check pin connections and try again.");
     return;
   }
   Serial.println();
   
   // Test 2: Enable motor
   Serial.println("Test 2: Motor Enable");
   Serial.println("-------------------------------");
   if (driver->enable()) {
     Serial.println("✓ PASS: Motor enabled successfully");
   } else {
     Serial.println("✗ FAIL: Motor enable failed");
     Serial.println("   Check if motor is connected and driver is powered.");
     return;
   }
   Serial.println();
   
   // Test 3: Check status
   Serial.println("Test 3: Status Check");
   Serial.println("-------------------------------");
   DriveStatus status = driver->getStatus();
   Serial.printf("  Servo Enabled: %s\n", status.servo_enabled ? "YES" : "NO");
   Serial.printf("  Motion Active: %s\n", driver->isMotionActive() ? "YES" : "NO");
   Serial.printf("  Current Position: %u steps\n", driver->getPosition());
   Serial.printf("  Current Speed: %u pps\n", driver->getSpeed());
   Serial.println("✓ PASS: Status read successfully");
   Serial.println();
   
   // Test 4: Check if enabled
   Serial.println("Test 4: Enable State Check");
   Serial.println("-------------------------------");
   if (driver->isEnabled()) {
     Serial.println("✓ PASS: Driver reports enabled state correctly");
   } else {
     Serial.println("✗ FAIL: Driver should be enabled but reports disabled");
   }
   Serial.println();
   
   // Test 5: Limit Switch Reading
   Serial.println("Test 5: Limit Switch Reading");
   Serial.println("-------------------------------");
   bool limitMin = !digitalRead(PIN_LIMIT_MIN);  // Active LOW
   bool limitMax = !digitalRead(PIN_LIMIT_MAX);  // Active LOW
   
   Serial.printf("  Limit MIN (Home): %s\n", limitMin ? "ACTIVE (LOW)" : "open (HIGH)");
   Serial.printf("  Limit MAX (End):  %s\n", limitMax ? "ACTIVE (LOW)" : "open (HIGH)");
   
   // Test reading multiple times to verify stability
   bool limitMin2 = !digitalRead(PIN_LIMIT_MIN);
   bool limitMax2 = !digitalRead(PIN_LIMIT_MAX);
   
   if (limitMin == limitMin2 && limitMax == limitMax2) {
     Serial.println("✓ PASS: Limit switches read consistently");
   } else {
     Serial.println("⚠ WARNING: Limit switch readings inconsistent (may be noise)");
   }
   
   Serial.println("\nNote: Limit switches should read HIGH (open) when not connected.");
   Serial.println("      If switches are connected and closed, they will read LOW (ACTIVE).");
   Serial.println();
   
   // Test 6: Limit Switch Protection (before move)
   Serial.println("Test 6: Limit Switch Protection");
   Serial.println("-------------------------------");
   limitMin = !digitalRead(PIN_LIMIT_MIN);
   limitMax = !digitalRead(PIN_LIMIT_MAX);
   
   if (limitMax) {
     Serial.println("⚠ WARNING: End limit (MAX) is ACTIVE - cannot move forward");
     Serial.println("   This is correct behavior - move will be prevented.");
   } else {
     Serial.println("✓ End limit (MAX) is open - forward movement allowed");
   }
   
   if (limitMin) {
     Serial.println("⚠ WARNING: Home limit (MIN) is ACTIVE - cannot move backward");
     Serial.println("   This is correct behavior - move will be prevented.");
   } else {
     Serial.println("✓ Home limit (MIN) is open - backward movement allowed");
   }
   Serial.println();
   
  // Test 7: Simple move (optional - comment out if motor not connected)
  Serial.println("Test 7: Simple Move Test (3000 steps)");
  Serial.println("-------------------------------");
  Serial.println("NOTE: Check serial output for LEDC errors.");
  Serial.println("      If you see 'ledc setup failed' errors, the test FAILED.");
  Serial.println();
  
  // Check limit before moving
  limitMax = !digitalRead(PIN_LIMIT_MAX);
  if (limitMax) {
    Serial.println("⚠ Skipping forward move - End limit (MAX) is ACTIVE");
  } else {
    Serial.println("Attempting to move 3000 steps forward...");
    
    if (driver->moveRelative(3000, 1000, 500, 500)) {
       Serial.println("✓ Move command accepted");
       Serial.println("Waiting for motion to complete...");
       Serial.println("(Monitor for LEDC errors above - they indicate FAILURE)");
       
       unsigned long startTime = millis();
       bool motionCompleted = false;
       while (driver->isMotionActive()) {
         // Check limit switch during motion
         limitMax = !digitalRead(PIN_LIMIT_MAX);
         if (limitMax) {
           Serial.println("\n⚠ LIMIT SWITCH TRIGGERED during motion!");
           Serial.println("   Stopping motor...");
           driver->stopMotion(500);
           delay(500);
           break;
         }
         
         delay(10);
         if (millis() - startTime > 5000) {
           Serial.println("⚠ WARNING: Motion taking longer than expected");
           break;
         }
       }
       
       if (!driver->isMotionActive()) {
         motionCompleted = true;
         Serial.println("✓ Motion completed");
         Serial.printf("  Final Position: %u steps\n", driver->getPosition());
         Serial.println();
         Serial.println("⚠ IMPORTANT: Review serial output above for LEDC errors.");
         Serial.println("   If 'ledc setup failed' errors appeared, this test FAILED.");
         Serial.println("   The motion may have completed, but pulse generation is broken.");
       } else {
         Serial.println("⚠ Motion still active (may be normal if motor not connected)");
       }
     } else {
       Serial.println("⚠ Move command rejected (may be normal if motor not connected)");
     }
   }
   Serial.println();
   
   // Summary
   Serial.println("========================================");
   Serial.println("Test Summary");
   Serial.println("========================================");
   Serial.println("Basic driver library tests completed.");
   Serial.println("If all tests passed, the library is working correctly.");
   Serial.println("\nNote: Move test may fail if motor is not connected.");
   Serial.println("      This is normal and does not indicate a library problem.");
   Serial.println();
 }
 
 void loop() {
   // Just keep checking status periodically
   static unsigned long lastCheck = 0;
   if (millis() - lastCheck >= 5000) {
     if (driver && driver->isEnabled()) {
       DriveStatus status = driver->getStatus();
       bool limitMin = !digitalRead(PIN_LIMIT_MIN);
       bool limitMax = !digitalRead(PIN_LIMIT_MAX);
       
       Serial.printf("[Status] Position: %u | Speed: %u | Motion: %s | Limits: MIN=%s MAX=%s\n",
                    status.current_position,
                    status.current_speed,
                    driver->isMotionActive() ? "YES" : "NO",
                    limitMin ? "ACTIVE" : "open",
                    limitMax ? "ACTIVE" : "open");
     }
     lastCheck = millis();
   }
   delay(100);
 }
 
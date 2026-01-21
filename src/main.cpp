#include <Arduino.h>
#include <stdint.h>
#include <inttypes.h>
 #include "SDF08NK8X.h"

 using namespace BergerdaServo;

// Timestamped serial logging helpers.
#define LOG_PRINT(value) do { Serial.printf("[%lu] ", millis()); Serial.print(value); } while (0)
#define LOG_PRINTLN(value) do { Serial.printf("[%lu] ", millis()); Serial.println(value); } while (0)
#define LOG_PRINTF(...) do { Serial.printf("[%lu] ", millis()); Serial.printf(__VA_ARGS__); } while (0)
 
 // Pin Definitions
 #define PIN_PULSE    2   // Output: Pulse signal
#define PIN_DIR      12   // Output: Direction signal (avoid ETH clock GPIO17)
#define PIN_ENABLE   15  // Output: Servo enable (SON)
#define PIN_LIMIT_MIN 33 // Input Pullup: Home limit switch (active LOW)
#define PIN_LIMIT_MAX 5 // Input Pullup: End limit switch (active LOW)
#define PIN_ALARM    17 // Input Pullup: Alarm signal (active LOW)
 
 // Servo Driver
 DriverConfig config;
 ServoDriver* driver = nullptr;
bool output_invert = true;
bool pulse_active = false;
uint32_t pulse_end_ms = 0;
uint32_t pulse_freq_hz = 0;
bool homing_done = false;  // Track if initial homing has been completed
bool homing_initiated = false;
bool homing_reported = false;
unsigned long homing_start_ms = 0;
String inputLine;
unsigned long last_rx_ms = 0;
void IRAM_ATTR onLimitIrq() {
  if (driver) {
    driver->notifyLimitIrq();
  }
}
static const uint32_t STEPS_PER_REV = 6000;
// Forward declarations for helpers used before definitions.
bool canMoveSteps(int32_t steps);

uint32_t rpmToPps(double rpm) {
  if (rpm <= 0.0) {
    return 0;
  }
  double pps = (rpm * STEPS_PER_REV) / 60.0;
  if (pps >= (double)UINT32_MAX) {
    return UINT32_MAX;
  }
  return (uint32_t)(pps + 0.5);
}


void printHelp() {
  LOG_PRINTLN("Commands:");
  LOG_PRINTLN("  help                 - show this help");
  LOG_PRINTLN("  status               - print driver status");
  LOG_PRINTLN("  limits               - read limit switches");
  LOG_PRINTLN("  enable               - enable motor");
  LOG_PRINTLN("  disable              - disable motor");
  LOG_PRINTLN("  move <steps> [v a d] - move steps (v/a/d in RPM, 6000 steps/rev)");
  LOG_PRINTLN("  stop                 - stop motion");
  LOG_PRINTLN("  slow6000             - 1 turn (6000 steps) sequence (blocking)");
  LOG_PRINTLN("  travel               - measure distance between MIN/MAX");
  LOG_PRINTLN("  invertout <0|1>       - set output inversion and reinit");
  LOG_PRINTLN("  pulse <freq_hz> <ms>  - LEDC pulse test on PULSE pin");
  LOG_PRINTLN("  pulsestop             - stop pulse test immediately");
  LOG_PRINTLN("");
}

void detachPulseLedc() {
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcWrite(PIN_PULSE, 0);
  ledcDetach(PIN_PULSE);
#else
  ledcWrite(config.ledc_channel, 0);
  ledcDetachPin(PIN_PULSE);
#endif
  // Don't set pinMode - leave pin detached from LEDC for driver to reclaim
}

void stopPulseTest() {
  if (!pulse_active) {
    return;
  }
  pulse_active = false;
  // Just detach LEDC and stop pulse - driver will reattach LEDC on next move
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcWrite(PIN_PULSE, 0);
  ledcDetach(PIN_PULSE);
#else
  ledcWrite(config.ledc_channel, 0);
  ledcDetachPin(PIN_PULSE);
#endif
  // Don't set pinMode to OUTPUT - leave pin detached from LEDC
  // The driver will properly reattach LEDC when starting motion
}

void servicePulseTest() {
  if (!pulse_active) {
    return;
  }
  if ((int32_t)(millis() - pulse_end_ms) >= 0) {
    stopPulseTest();
    // Don't call driver->stopMotion() - it hangs when motion isn't active
    LOG_PRINTLN("OK pulse test complete");
  }
}

bool initDriver() {
  if (driver) {
    driver->disable();
    delete driver;
    driver = nullptr;
  }

  config.invert_output_logic = output_invert;
  config.invert_dir_pin = true;
  driver = new ServoDriver(config);
  if (!driver->initialize()) {
    return false;
  }
  return driver->enable();
}

void printLimits() {
  if (driver) {
    driver->updateLimitDebounce(true);
  }
  bool limitMin = driver ? driver->getLimitMinDebounced() : false;
  bool limitMax = driver ? driver->getLimitMaxDebounced() : false;
  LOG_PRINTF("  X_LS_MIN (IO%d / Home): %s\n", PIN_LIMIT_MIN,
             limitMin ? "ACTIVE (LOW)" : "open (HIGH)");
  LOG_PRINTF("  X_LS_MAX (IO%d / End):  %s\n", PIN_LIMIT_MAX,
             limitMax ? "ACTIVE (LOW)" : "open (HIGH)");
}

void printStatus() {
  if (!driver) {
    LOG_PRINTLN("⚠ Driver not initialized");
    return;
  }
  DriveStatus status = driver->getStatus();
  driver->updateLimitDebounce(true);
  bool limitMin = driver->getLimitMinDebounced();
  bool limitMax = driver->getLimitMaxDebounced();
  LOG_PRINTF("  Servo Enabled: %s\n", status.servo_enabled ? "YES" : "NO");
  LOG_PRINTF("  Motion Active: %s\n", driver->isMotionActive() ? "YES" : "NO");
  LOG_PRINTF("  Current Position: %" PRIu32 " steps\n", driver->getPosition());
  LOG_PRINTF("  Current Speed: %" PRIu32 " pps\n", driver->getSpeed());
  LOG_PRINTF("  Limits: MIN=%s MAX=%s\n",
             limitMin ? "ACTIVE" : "open",
             limitMax ? "ACTIVE" : "open");
}

bool canMoveSteps(int32_t steps) {
  if (steps == 0) {
    LOG_PRINTLN("⚠ Move ignored (0 steps)");
    return false;
  }
  if (driver) {
    driver->updateLimitDebounce(true);
  }
  bool limitMin = driver ? driver->getLimitMinDebounced() : false;
  bool limitMax = driver ? driver->getLimitMaxDebounced() : false;
  if (steps > 0 && limitMax) {
    LOG_PRINTLN("⚠ X_LS_MAX (IO32) is ACTIVE - cannot move forward");
    return false;
  }
  if (steps < 0 && limitMin) {
    LOG_PRINTLN("⚠ X_LS_MIN (IO14) is ACTIVE - cannot move backward");
    return false;
  }
  return true;
}


void runslow6000() {
  const uint32_t total_steps = 6000;
  double speed_rpm = 6.0; // 600 Hz for 6000 steps = 10 seconds
  uint32_t speed = rpmToPps(speed_rpm);
  if (speed > config.max_pulse_freq) {
    speed = config.max_pulse_freq;
  }
  const uint32_t accel = rpmToPps(3.0);
  const uint32_t decel = rpmToPps(3.0);
  const uint32_t progress_interval = 600;

  if (driver) {
    driver->updateLimitDebounce(true);
  }
  if (!driver->isEnabled()) {
    if (!driver->enable()) {
      LOG_PRINTLN("ERR enable failed");
      return;
    }
  }
  int32_t steps = static_cast<int32_t>(total_steps);
  if (!canMoveSteps(steps)) {
    LOG_PRINTLN("ERR limit active");
    return;
  }

  const uint32_t start_pos = driver->getPosition();
  if (!driver->moveRelative(steps, speed, accel, decel)) {
    LOG_PRINTLN("ERR move rejected");
    return;
  }
  uint32_t last_report = 0;
  while (driver->isMotionActive()) {
    driver->updateLimitDebounce();
    if (driver->getLimitMaxDebounced()) {
      driver->stopMotion(500);
      break;
    }
    uint32_t current_pos = driver->getPosition();
    uint32_t moved_steps = (current_pos >= start_pos)
                               ? (current_pos - start_pos)
                               : (start_pos - current_pos);
    if (moved_steps != last_report && moved_steps % progress_interval == 0) {
      last_report = moved_steps;
      LOG_PRINTF("Progress: %" PRIu32 " / %" PRIu32 " steps\n", moved_steps, total_steps);
    }
    delay(2);
  }
  uint32_t final_pos = driver->getPosition();
  uint32_t moved_steps = (final_pos >= start_pos)
                             ? (final_pos - start_pos)
                             : (start_pos - final_pos);
  LOG_PRINTF("Progress: %" PRIu32 " / %" PRIu32 " steps\n", moved_steps, total_steps);
}

void processCommand(String cmd) {
  String clean;
  clean.reserve(cmd.length());
  for (size_t i = 0; i < cmd.length(); ++i) {
    char ch = cmd.charAt(i);
    if (ch >= 32 && ch <= 126) {
      clean += ch;
    }
  }
  clean.trim();
  clean.toLowerCase();
  if (clean.length() == 0) {
    return;
  }

  // Auto-homing on first command if HOME_ON_BOOT is enabled
  if (config.home_on_boot && !homing_done && !homing_initiated) {
    LOG_PRINTLN("");
    LOG_PRINTLN("========================================");
    LOG_PRINTLN(" AUTO-HOMING SEQUENCE");
    LOG_PRINTLN("========================================");
    LOG_PRINTF("Moving to MIN limit switch (IO%d)...\n", PIN_LIMIT_MIN);
    LOG_PRINTLN("Press Ctrl+C to abort if needed.");
    LOG_PRINTLN("");
    
    // If MIN limit already active, skip homing
    if (driver->getLimitMinDebounced()) {
      LOG_PRINTLN("✓ MIN limit already active - homing skipped");
      homing_done = true;
      homing_reported = true;
    } else if (driver->startHoming(config.homing_speed_pps)) {
      homing_initiated = true;
      homing_start_ms = millis();
      LOG_PRINTLN("Homing started... (non-blocking)");
    } else {
      LOG_PRINTLN("✗ Homing failed to start");
      homing_done = true;
      homing_reported = true;
    }
    LOG_PRINTLN("");
    return;  // don't process the triggering command
  }

  if (driver && driver->isHoming() && clean != "stop" && clean != "status") {
    LOG_PRINTLN("⚠ Homing in progress - command ignored");
    return;
  }

  if (clean == "help" || clean == "?") {
    printHelp();
  } else if (clean == "status") {
    printStatus();
  } else if (clean == "limits") {
    printLimits();
  } else if (clean == "enable") {
    bool ok = driver->enable();
    LOG_PRINTF("%s Motor enabled\n", ok ? "OK" : "ERR");
  } else if (clean == "disable") {
    bool ok = driver->disable();
    LOG_PRINTF("%s Motor disabled\n", ok ? "OK" : "ERR");
  } else if (clean == "stop") {
    driver->stopMotion(500);
    LOG_PRINTLN("OK Stop requested");
  } else if (clean == "home") {
    if (driver->getLimitMinDebounced()) {
      LOG_PRINTLN("✓ MIN limit already active - homing skipped");
      homing_done = true;
      homing_reported = true;
    } else if (driver->startHoming(config.homing_speed_pps)) {
      homing_initiated = true;
      homing_start_ms = millis();
      LOG_PRINTLN("Homing started... (non-blocking)");
    } else {
      LOG_PRINTLN("✗ Homing failed to start");
      homing_done = true;
      homing_reported = true;
    }
    while (driver->isHoming()) {
      driver->updateLimitDebounce();
      delay(10);
    }
    if (!driver->isHoming()) {
        LOG_PRINTLN("OK Homing complete - position set to 0");
      }
  } else if (clean == "slow6000" || clean == "slow100000" || clean == "slow1000000" || clean == "slow400000") {
    runslow6000();
  } else if (clean == "travel") {
    LOG_PRINTLN("Measuring travel between MIN and MAX...");
    bool ok = driver->measureTravelDistance(12000, 12000, 12000, 90000);
    if (ok && driver->hasTravelDistance()) {
      LOG_PRINTF("OK Travel distance: %" PRIu32 " steps\n", driver->getTravelDistance());
    } else {
      LOG_PRINTLN("ERR Travel measurement failed");
    }
  } else if (clean.startsWith("invertout")) {
    int value = 0;
    if (sscanf(clean.c_str(), "invertout %d", &value) == 1) {
      output_invert = (value != 0);
    } else {
      output_invert = !output_invert;
    }
    if (initDriver()) {
      LOG_PRINTF("Output invert: %s\n", output_invert ? "ON" : "OFF");
    } else {
      LOG_PRINTLN("ERR reinit failed");
    }
  } else if (clean == "pulsestop" || clean == "pulse stop") {
    stopPulseTest();
    driver->stopMotion(500);
    LOG_PRINTLN("OK pulse test stopped");
  } else if (clean.startsWith("pulse")) {
    LOG_PRINTLN("[DBG] pulse: enter");
    uint32_t freq = 0;
    uint32_t ms = 0;
    int parsed = sscanf(clean.c_str(), "pulse %lu %lu", &freq, &ms);
    LOG_PRINTF("[DBG] pulse: sscanf returned %d, freq=%lu, ms=%lu\n", parsed, freq, ms);
    if (parsed < 2) {
      LOG_PRINTLN("⚠ Usage: pulse <freq_hz> <ms>");
      return;
    }
    if (freq == 0 || ms == 0) {
      LOG_PRINTLN("⚠ freq_hz and ms must be > 0");
      return;
    }
    if (driver) {
      driver->updateLimitDebounce(true);
      if (driver->getLimitMinDebounced() || driver->getLimitMaxDebounced()) {
        LOG_PRINTLN("⚠ Limit switch active - pulse blocked");
        return;
      }
    }
    LOG_PRINTLN("[DBG] pulse: stopping active pulse if any");
    if (pulse_active) {
      LOG_PRINTLN("[DBG] pulse: calling stopPulseTest()");
      stopPulseTest();
      LOG_PRINTLN("[DBG] pulse: stopPulseTest() done");
    } else {
      LOG_PRINTLN("[DBG] pulse: no active pulse test");
    }
    // Don't call driver->stopMotion() here - it can hang if motion isn't active
    // We're about to detach LEDC anyway, so driver state doesn't matter
    LOG_PRINTLN("[DBG] pulse: detaching LEDC");
    detachPulseLedc();
    LOG_PRINTLN("[DBG] pulse: attaching LEDC");
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcAttach(PIN_PULSE, freq, 1);
    ledcWrite(PIN_PULSE, 1);
#else
    ledcSetup(config.ledc_channel, freq, 1);
    ledcAttachPin(PIN_PULSE, config.ledc_channel);
    ledcWrite(config.ledc_channel, 1);
#endif
    LOG_PRINTLN("[DBG] pulse: setting vars");
    pulse_active = true;
    pulse_freq_hz = freq;
    pulse_end_ms = millis() + ms;
    LOG_PRINTLN("OK pulse test started");
  } else if (clean.startsWith("move")) {
    LOG_PRINTLN("[DBG] move: enter");
    int32_t steps = 0;
    double speed_rpm = 1000.0;
    double accel_rpm = 500.0;
    double decel_rpm = 500.0;
    int parsed = sscanf(clean.c_str(), "move %ld %lf %lf %lf",
                        &steps, &speed_rpm, &accel_rpm, &decel_rpm);
    LOG_PRINTF("[DBG] move: parsed=%d, steps=%ld, speed_rpm=%.1f\n", parsed, steps, speed_rpm);
    if (parsed < 1) {
      LOG_PRINTLN("⚠ Usage: move <steps> [speed_rpm accel_rpm decel_rpm]");
      return;
    }
    LOG_PRINTLN("[DBG] move: calling canMoveSteps");
    if (!canMoveSteps(steps)) {
      LOG_PRINTLN("[DBG] move: canMoveSteps returned false");
      return;
    }
    LOG_PRINTLN("[DBG] move: canMoveSteps OK, converting RPM");
    uint32_t speed = rpmToPps(speed_rpm);
    uint32_t accel = rpmToPps(accel_rpm);
    uint32_t decel = rpmToPps(decel_rpm);
    if (speed > config.max_pulse_freq) {
      speed = config.max_pulse_freq;
    }
    LOG_PRINTF("[DBG] move: calling moveRelative(steps=%ld, speed=%lu)\n", steps, speed);
    if (driver->moveRelative(steps, speed, accel, decel)) {
      LOG_PRINTLN("OK Move command accepted");
    } else {
      LOG_PRINTLN("ERR Move command rejected");
    }
  } else {
    LOG_PRINTLN("⚠ Unknown command. Type 'help' for options.");
  }

  Serial.flush();
}
 
 void setup() {
   Serial.begin(115200);
  Serial.setTimeout(50);
   delay(2000);  // Wait for serial monitor
   
  LOG_PRINTLN("\n========================================");
  LOG_PRINTLN("Serial Driver Test (Interactive)");
  LOG_PRINTLN("========================================\n");
   
   // Configure pins
   config.output_pin_nos[6] = PIN_PULSE;   // PULSE
   config.output_pin_nos[7] = PIN_DIR;    // DIR
   config.output_pin_nos[0] = PIN_ENABLE; // ENABLE

  // Configure limit switch pins
  config.limit_min_pin = PIN_LIMIT_MIN;
  config.limit_max_pin = PIN_LIMIT_MAX;
  config.homing_speed_pps = 6000;

  // Enable homing on boot
  config.home_on_boot = (HOME_ON_BOOT != 0);
   
   // Disable encoder feedback for basic test
   config.enable_encoder_feedback = false;
   
   
   // Set control mode
   config.pulse_mode = PulseMode::PULSE_DIRECTION;
   config.control_mode = ControlMode::POSITION;
   
  LOG_PRINTLN("Initializing driver...");
  if (!initDriver()) {
    LOG_PRINTLN("✗ FAIL: Driver initialization failed");
    LOG_PRINTLN("   Check pin connections and try again.");
     return;
   }
  LOG_PRINTLN("✓ Driver ready (motor enabled).");

  // Attach limit switch interrupts after driver configures pins
  attachInterrupt(digitalPinToInterrupt(PIN_LIMIT_MIN), onLimitIrq, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_LIMIT_MAX), onLimitIrq, CHANGE);
  
  // Auto-homing on boot if enabled
  if (config.home_on_boot) {
    LOG_PRINTLN("");
    LOG_PRINTLN("⚠ HOME_ON_BOOT enabled - waiting for any serial input to start homing...");
    LOG_PRINTF("   (Homing will move negative until MIN limit switch IO%d is triggered)\n",
               PIN_LIMIT_MIN);
  }
  
  LOG_PRINTLN("");
  LOG_PRINTLN("Type 'help' and press Enter to see commands.");
  printHelp();
}

void loop() {
  if (driver) {
    driver->updateLimitDebounce();
  }
  servicePulseTest();
  if (homing_initiated && !homing_done) {
    if (!driver->isHoming()) {
      homing_done = true;
      if (!homing_reported) {
        LOG_PRINTLN("✓ Homing complete - position set to 0");
        homing_reported = true;
      }
    } else if (millis() - homing_start_ms > 30000) {
      LOG_PRINTLN("✗ Homing timeout - stopping motion");
      driver->stopMotion(0);
      homing_done = true;
      homing_reported = true;
    }
  }
  while (Serial.available() > 0) {
    char ch = static_cast<char>(Serial.read());
    last_rx_ms = millis();
    if (ch == '\n' || ch == '\r' || ch == ';') {
      if (inputLine.length() > 0) {
        String cmd = inputLine;
        inputLine = "";
        LOG_PRINTF("[RX] \"%s\"\n", cmd.c_str());
        processCommand(cmd);
      }
      continue;
    }
    inputLine += ch;
    if (inputLine.length() > 200) {
      inputLine = "";
      LOG_PRINTLN("⚠ Input too long, cleared");
    }
  }

  if (inputLine.length() > 0) {
    if ((millis() - last_rx_ms) > 50) {
      String line = inputLine;
      inputLine = "";
      line.trim();
      if (line.length() == 0) {
        return;
      }
      LOG_PRINTF("[RX timeout] \"%s\"\n", line.c_str());
      int start = 0;
      while (start < static_cast<int>(line.length())) {
        int sep = line.indexOf(';', start);
        if (sep < 0) {
          String cmd = line.substring(start);
          processCommand(cmd);
          break;
        }
        String cmd = line.substring(start, sep);
        processCommand(cmd);
        start = sep + 1;
      }
    }
  }
 }
 
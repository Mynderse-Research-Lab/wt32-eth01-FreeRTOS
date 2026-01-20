#include <Arduino.h>
#include <stdint.h>
#include "SDF08NK8X.h"

using namespace BergerdaServo;

// Timestamped serial logging helpers.
#define LOG_PRINT(value) do { Serial.printf("[%lu] ", millis()); Serial.print(value); } while (0)
#define LOG_PRINTLN(value) do { Serial.printf("[%lu] ", millis()); Serial.println(value); } while (0)
#define LOG_PRINTF(...) do { Serial.printf("[%lu] ", millis()); Serial.printf(__VA_ARGS__); } while (0)

// Pin Definitions
#define PIN_PULSE    2   // Output: Pulse signal
#define PIN_DIR       17  // Output: Direction signal
#define PIN_ENABLE   15  // Output: Servo enable (SON)
#define PIN_LIMIT_MIN 14 // Input Pullup: Home limit switch (active LOW)
#define PIN_LIMIT_MAX 32 // Input Pullup: End limit switch (active LOW)

// Servo Driver
DriverConfig config;
ServoDriver* driver = nullptr;
String inputLine;
unsigned long last_rx_ms = 0;
uint8_t limit_min_sample = 0;
uint8_t limit_max_sample = 0;
uint8_t limit_min_stable = 0;
uint8_t limit_max_stable = 0;
bool limit_min_state = false;
bool limit_max_state = false;
unsigned long last_limit_sample_ms = 0;
static const uint8_t LIMIT_DEBOUNCE_CYCLES = 10;
static const uint32_t STEPS_PER_REV = 6000;
static bool invert_direction = false;

// Forward declarations for helpers used before definitions.
void updateLimitDebounce();
bool getLimitMinDebounced();
bool getLimitMaxDebounced();
bool canMoveSteps(int32_t steps);

struct SoftwareMoveState {
  bool active = false;
  bool direction = true;
  uint32_t total_steps = 0;
  uint32_t moved_steps = 0;
  uint32_t accel_steps = 0;
  uint32_t decel_steps = 0;
  double accel_rate = 0.0;
  double decel_rate = 0.0;
  double max_speed = 0.0;
  uint32_t high_us = 100;
  uint32_t next_pulse_us = 0;
  uint32_t pulse_high_start_us = 0;
  bool pulse_high = false;
  uint32_t progress_interval = 0;
  uint32_t last_report = 0;
  int pulse_pin = -1;
  int dir_pin = -1;
};

SoftwareMoveState soft_move;

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

void writeLogicalPin(int pin, bool logical_state) {
  if (pin == config.output_pin_nos[7] && invert_direction) {
    logical_state = !logical_state;
  }
  bool physical_state = config.invert_output_logic ? !logical_state : logical_state;
  digitalWrite(pin, physical_state ? HIGH : LOW);
}
int32_t applyDirectionInvert(int32_t steps) {
  return invert_direction ? -steps : steps;
}

uint32_t getLedcMinPps() {
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  return 5000;
#else
  return 40000;
#endif
}

bool startSoftwareMove(int32_t steps, uint32_t max_pps,
                       uint32_t accel_pps_per_s, uint32_t decel_pps_per_s,
                       uint32_t progress_interval) {
  if (soft_move.active || steps == 0 || max_pps == 0) {
    return false;
  }
  if (!canMoveSteps(steps)) {
    return false;
  }

  soft_move.active = true;
  soft_move.direction = steps > 0;
  soft_move.total_steps = (steps >= 0) ? (uint32_t)steps : (uint32_t)(-steps);
  soft_move.moved_steps = 0;
  soft_move.accel_rate = (accel_pps_per_s > 0) ? (double)accel_pps_per_s : 0.0;
  soft_move.decel_rate = (decel_pps_per_s > 0) ? (double)decel_pps_per_s : 0.0;
  soft_move.max_speed = (double)max_pps;
  soft_move.progress_interval = progress_interval;
  soft_move.last_report = 0;
  soft_move.pulse_high = false;
  soft_move.high_us = 100;
  soft_move.pulse_pin = config.output_pin_nos[6];
  soft_move.dir_pin = config.output_pin_nos[7];
  if (soft_move.pulse_pin < 0 || soft_move.dir_pin < 0) {
    soft_move.active = false;
    return false;
  }

  soft_move.accel_steps = 0;
  soft_move.decel_steps = 0;
  if (soft_move.accel_rate > 0.0) {
    soft_move.accel_steps = (uint32_t)((soft_move.max_speed * soft_move.max_speed) /
                                       (2.0 * soft_move.accel_rate));
  }
  if (soft_move.decel_rate > 0.0) {
    soft_move.decel_steps = (uint32_t)((soft_move.max_speed * soft_move.max_speed) /
                                       (2.0 * soft_move.decel_rate));
  }
  if (soft_move.accel_steps + soft_move.decel_steps > soft_move.total_steps) {
    soft_move.accel_steps = soft_move.total_steps / 2;
    soft_move.decel_steps = soft_move.total_steps - soft_move.accel_steps;
    if (soft_move.accel_rate > 0.0) {
      soft_move.max_speed = sqrt(2.0 * soft_move.accel_rate *
                                 (double)soft_move.accel_steps);
    }
  }

  // Detach LEDC so we can bit-bang the pulse pin at low frequency.
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcDetach(soft_move.pulse_pin);
#else
  ledcDetachPin(soft_move.pulse_pin);
#endif
  pinMode(soft_move.pulse_pin, OUTPUT);
  writeLogicalPin(soft_move.dir_pin, soft_move.direction);
  delayMicroseconds(10);
  soft_move.next_pulse_us = micros();
  return true;
}
void stopSoftwareMove() {
  if (!soft_move.active) {
    return;
  }
  soft_move.active = false;
  soft_move.pulse_high = false;
  writeLogicalPin(soft_move.dir_pin, false);
  writeLogicalPin(soft_move.pulse_pin, false);
  delayMicroseconds(1);

  // Restore LEDC for normal driver operations.
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcAttach(soft_move.pulse_pin, 1000, config.ledc_resolution);
  ledcWrite(soft_move.pulse_pin, 0);
#else
  ledcSetup(config.ledc_channel, 1000, config.ledc_resolution);
  ledcAttachPin(soft_move.pulse_pin, config.ledc_channel);
  ledcWrite(config.ledc_channel, 0);
#endif
}
void serviceSoftwareMove() {
  if (!soft_move.active) {
    return;
  }

  updateLimitDebounce();
  if ((soft_move.direction && getLimitMaxDebounced()) ||
      (!soft_move.direction && getLimitMinDebounced())) {
    stopSoftwareMove();
    return;
  }

  uint32_t now_us = micros();

  if (soft_move.pulse_high) {
    if ((uint32_t)(now_us - soft_move.pulse_high_start_us) >= soft_move.high_us) {
      writeLogicalPin(soft_move.pulse_pin, false);
      soft_move.pulse_high = false;
      soft_move.moved_steps++;

      if (soft_move.progress_interval > 0 &&
          soft_move.moved_steps % soft_move.progress_interval == 0) {
        LOG_PRINTF("Progress: %u / %u steps\n",
                   soft_move.moved_steps, soft_move.total_steps);
      }

      if (soft_move.moved_steps >= soft_move.total_steps) {
        LOG_PRINTF("Progress: %u / %u steps\n",
                   soft_move.moved_steps, soft_move.total_steps);
        stopSoftwareMove();
        return;
      }

      double pps = soft_move.max_speed;
      if (soft_move.accel_rate > 0.0 &&
          soft_move.moved_steps < soft_move.accel_steps) {
        pps = sqrt(2.0 * soft_move.accel_rate *
                   (double)soft_move.moved_steps);
        if (pps < 1.0) {
          pps = 1.0;
        }
      } else if (soft_move.decel_rate > 0.0 &&
                 soft_move.moved_steps >=
                     (soft_move.total_steps - soft_move.decel_steps)) {
        uint32_t remaining = soft_move.total_steps - soft_move.moved_steps;
        pps = sqrt(2.0 * soft_move.decel_rate * (double)remaining);
        if (pps < 1.0) {
          pps = 1.0;
        }
      }

      uint32_t period_us = (uint32_t)(1000000.0 / pps);
      if (period_us < soft_move.high_us + 1) {
        period_us = soft_move.high_us + 1;
      }
      soft_move.next_pulse_us = now_us + (period_us - soft_move.high_us);
    }
    return;
  }

  if ((int32_t)(now_us - soft_move.next_pulse_us) >= 0) {
    writeLogicalPin(soft_move.pulse_pin, true);
    soft_move.pulse_high = true;
    soft_move.pulse_high_start_us = now_us;
  }
}

void updateLimitDebounce() {
  const unsigned long now = millis();
  if (now - last_limit_sample_ms < 1) {
    return;
  }
  last_limit_sample_ms = now;

  bool min_now = !digitalRead(PIN_LIMIT_MIN);
  bool max_now = !digitalRead(PIN_LIMIT_MAX);

  if (min_now == limit_min_sample) {
    if (limit_min_stable < LIMIT_DEBOUNCE_CYCLES) {
      limit_min_stable++;
      if (limit_min_stable == LIMIT_DEBOUNCE_CYCLES) {
        limit_min_state = min_now;
      }
    }
  } else {
    limit_min_sample = min_now;
    limit_min_stable = 1;
  }

  if (max_now == limit_max_sample) {
    if (limit_max_stable < LIMIT_DEBOUNCE_CYCLES) {
      limit_max_stable++;
      if (limit_max_stable == LIMIT_DEBOUNCE_CYCLES) {
        limit_max_state = max_now;
      }
    }
  } else {
    limit_max_sample = max_now;
    limit_max_stable = 1;
  }
}

bool getLimitMinDebounced() { return limit_min_state; }
bool getLimitMaxDebounced() { return limit_max_state; }

void printHelp() {
  LOG_PRINTLN("Commands:");
  LOG_PRINTLN("  help                 - show this help");
  LOG_PRINTLN("  status               - print driver status");
  LOG_PRINTLN("  limits               - read limit switches");
  LOG_PRINTLN("  enable               - enable motor");
  LOG_PRINTLN("  disable              - disable motor");
  LOG_PRINTLN("  move <steps> [v a d] - move steps (v/a/d in RPM, 6000 steps/rev)");
  LOG_PRINTLN("  wait                 - wait for motion to complete");
  LOG_PRINTLN("  stop                 - stop motion");
  LOG_PRINTLN("  slow6000             - 1 turn (6000 steps) sequence (blocking)");
  LOG_PRINTLN("  dirinvert <0|1>       - invert direction for move/slow");
  LOG_PRINTLN("");
}

void printLimits() {
  updateLimitDebounce();
  bool limitMin = getLimitMinDebounced();
  bool limitMax = getLimitMaxDebounced();
  LOG_PRINTF("  X_LS_MIN (IO14 / Home): %s\n", limitMin ? "ACTIVE (LOW)" : "open (HIGH)");
  LOG_PRINTF("  X_LS_MAX (IO32 / End):  %s\n", limitMax ? "ACTIVE (LOW)" : "open (HIGH)");
}

void printStatus() {
  if (!driver) {
    LOG_PRINTLN("⚠ Driver not initialized");
    return;
  }
  DriveStatus status = driver->getStatus();
  updateLimitDebounce();
  bool limitMin = getLimitMinDebounced();
  bool limitMax = getLimitMaxDebounced();
  LOG_PRINTF("  Servo Enabled: %s\n", status.servo_enabled ? "YES" : "NO");
  LOG_PRINTF("  Motion Active: %s\n", driver->isMotionActive() ? "YES" : "NO");
  LOG_PRINTF("  Current Position: %u steps\n", driver->getPosition());
  LOG_PRINTF("  Current Speed: %u pps\n", driver->getSpeed());
  LOG_PRINTF("  Limits: MIN=%s MAX=%s\n",
             limitMin ? "ACTIVE" : "open",
             limitMax ? "ACTIVE" : "open");
}

bool canMoveSteps(int32_t steps) {
  if (steps == 0) {
    LOG_PRINTLN("⚠ Move ignored (0 steps)");
    return false;
  }
  updateLimitDebounce();
  bool limitMin = getLimitMinDebounced();
  bool limitMax = getLimitMaxDebounced();
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

void waitForMotion() {
  if (!driver) {
    return;
  }
  LOG_PRINTF("Motion Active: %s\n", driver->isMotionActive() ? "YES" : "NO");
}

void runslow6000() {
  const uint32_t total_steps = 6000;
  double speed_rpm = 6.0; // 600 Hz for 6000 steps = 10 seconds
  uint32_t pps = rpmToPps(speed_rpm);
  if (pps == 0) {
    return;
  }
  const uint32_t progress_interval = 600;

  updateLimitDebounce();
  if (!driver->isEnabled()) {
    if (!driver->enable()) {
      LOG_PRINTLN("ERR enable failed");
      return;
    }
  }
  int32_t steps = applyDirectionInvert(static_cast<int32_t>(total_steps));
  if (!canMoveSteps(steps)) {
    LOG_PRINTLN("ERR limit active");
    return;
  }

  startSoftwareMove(steps, pps,
                    pps / 2, pps / 2, progress_interval);
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
  } else if (clean == "wait") {
    waitForMotion();
  } else if (clean == "stop") {
    driver->stopMotion(500);
    stopSoftwareMove();
    LOG_PRINTLN("OK Stop requested");
  } else if (clean == "slow6000" || clean == "slow100000" || clean == "slow1000000" || clean == "slow400000") {
    runslow6000();
  } else if (clean.startsWith("dirinvert")) {
    int value = 0;
    if (sscanf(clean.c_str(), "dirinvert %d", &value) == 1) {
      invert_direction = (value != 0);
    } else {
      invert_direction = !invert_direction;
    }
    LOG_PRINTF("Direction invert: %s\n", invert_direction ? "ON" : "OFF");
  } else if (clean.startsWith("move")) {
    int32_t steps = 0;
    double speed_rpm = 1000.0;
    double accel_rpm = 500.0;
    double decel_rpm = 500.0;
    int parsed = sscanf(clean.c_str(), "move %ld %lf %lf %lf",
                        &steps, &speed_rpm, &accel_rpm, &decel_rpm);
    if (parsed < 1) {
      LOG_PRINTLN("⚠ Usage: move <steps> [speed_rpm accel_rpm decel_rpm]");
      return;
    }
    steps = applyDirectionInvert(steps);
    if (!canMoveSteps(steps)) {
      return;
    }
    uint32_t speed = rpmToPps(speed_rpm);
    uint32_t accel = rpmToPps(accel_rpm);
    uint32_t decel = rpmToPps(decel_rpm);
    if (speed > config.max_pulse_freq) {
      speed = config.max_pulse_freq;
    }

    if (speed < getLedcMinPps()) {
      bool ok = startSoftwareMove(steps, speed, accel, decel, 0);
      LOG_PRINTLN(ok ? "OK Move command accepted" : "ERR Move command rejected");
    } else {
      if (driver->moveRelative(steps, speed, accel, decel)) {
        LOG_PRINTLN("OK Move command accepted");
      } else {
        LOG_PRINTLN("ERR Move command rejected");
      }
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

  // Disable encoder feedback for basic test
  config.enable_encoder_feedback = false;

  // Configure limit switch pins (INPUT with internal pullup)
  pinMode(PIN_LIMIT_MIN, INPUT_PULLUP);
  pinMode(PIN_LIMIT_MAX, INPUT_PULLUP);
  limit_min_sample = !digitalRead(PIN_LIMIT_MIN);
  limit_max_sample = !digitalRead(PIN_LIMIT_MAX);
  limit_min_state = limit_min_sample;
  limit_max_state = limit_max_sample;
  limit_min_stable = 1;
  limit_max_stable = 1;

  // Set control mode
  config.pulse_mode = PulseMode::PULSE_DIRECTION;
  config.control_mode = ControlMode::POSITION;

  // Create driver instance
  driver = new ServoDriver(config);
  LOG_PRINTLN("Initializing driver...");
  if (!driver->initialize()) {
    LOG_PRINTLN("✗ FAIL: Driver initialization failed");
    LOG_PRINTLN("   Check pin connections and try again.");
    return;
  }
  if (!driver->enable()) {
    LOG_PRINTLN("✗ FAIL: Motor enable failed");
    LOG_PRINTLN("   Check if motor is connected and driver is powered.");
    return;
  }
  LOG_PRINTLN("✓ Driver ready.");
  LOG_PRINTLN("");
  LOG_PRINTLN("Type 'help' and press Enter to see commands.");
  printHelp();
}

void loop() {
  updateLimitDebounce();
  serviceSoftwareMove();
  bool line_complete = false;
  while (Serial.available() > 0) {
    char ch = static_cast<char>(Serial.read());
    last_rx_ms = millis();
    if (ch == '\n' || ch == '\r') {
      line_complete = true;
    } else {
      inputLine += ch;
      if (inputLine.length() > 200) {
        inputLine = "";
        LOG_PRINTLN("⚠ Input too long, cleared");
      }
    }
  }

  if (inputLine.length() > 0) {
    if (line_complete || (millis() - last_rx_ms) > 50) {
      String line = inputLine;
      inputLine = "";
      line.trim();
      if (line.length() == 0) {
        return;
      }
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

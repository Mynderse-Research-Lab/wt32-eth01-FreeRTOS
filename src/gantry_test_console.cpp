#include "gantry_test_console.h"

#include "basic_tests.h"
#include "freertos/task.h"
#include "gantry_app_constants.h"
#include "gpio_expander.h"
#include "esp_log.h"

#include <ctype.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

extern "C" int getchar(void);

namespace {
static const char *TAG = "GantryConsole";

struct ControlStateSnapshot {
  bool enabled;
  bool busy;
  bool alarm;
  bool min_limit_active;
  bool max_limit_active;
  int raw_alarm_level;
};

struct ControlDebounceState {
  bool initialized;
  ControlStateSnapshot stable;
  ControlStateSnapshot pending;
  uint8_t enabled_cnt;
  uint8_t busy_cnt;
  uint8_t alarm_cnt;
  uint8_t min_limit_cnt;
  uint8_t max_limit_cnt;
  uint8_t raw_alarm_cnt;
};

static constexpr uint8_t kFlipDebounceSamples = 12;       // 12ms stable at 1ms monitor tick
static constexpr uint8_t kRawAlarmDebounceSamples = 12;   // 12ms stable at 1ms monitor tick
static ControlDebounceState g_controlDebounce = {
    false, {false, false, false, false, false, -1}, {false, false, false, false, false, -1},
    0, 0, 0, 0, 0, 0};
static bool g_homeCompletedThisSession = false;
static bool g_calibratedThisSession = false;
static bool g_calibrationInProgress = false;
static uint32_t g_moveSpeedMmPerS = 50;
static uint32_t g_moveSpeedDegPerS = 30;
static uint32_t g_moveAccelMmPerS2 = 0;
static uint32_t g_moveDecelMmPerS2 = 0;
static uint32_t g_lastLiveMotionLogMs = 0;
static bool g_liveMotionWasBusy = false;

void logLiveMotionState(const GantryTestConsoleConfig *cfg) {
  if (cfg == nullptr || cfg->gantry == nullptr) {
    return;
  }

  const bool busy = cfg->gantry->isBusy();
  const uint32_t nowMs = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
  constexpr uint32_t kLiveLogIntervalMs = 100;

  if (!busy) {
    g_liveMotionWasBusy = false;
    g_lastLiveMotionLogMs = 0;
    return;
  }

  // While motion is active, emit a periodic dual-source X report:
  // - x_cmd: commanded/driver position
  // - x_enc: encoder feedback position
  if (!g_liveMotionWasBusy || (nowMs - g_lastLiveMotionLogMs) >= kLiveLogIntervalMs) {
    const Gantry::JointConfig joint = cfg->gantry->getCurrentJointConfig();
    const float xCmdMm = cfg->gantry->getXCommandedMm();
    const float xEncMm = cfg->gantry->getXEncoderMm();
    ESP_LOGI(TAG, "LIVE POS: x_cmd=%.2f mm, x_enc=%.2f mm, y=%.2f mm, theta=%.2f deg",
             xCmdMm, xEncMm, joint.y, joint.theta);
    g_lastLiveMotionLogMs = nowMs;
    g_liveMotionWasBusy = true;
  }
}

void calibrationTask(void *param) {
  auto *cfg = static_cast<GantryTestConsoleConfig *>(param);
  if (cfg == nullptr || cfg->gantry == nullptr) {
    g_calibrationInProgress = false;
    vTaskDelete(nullptr);
    return;
  }

  const bool hadCalibration = g_calibratedThisSession;
  ESP_LOGI(TAG, "Calibration task: started");
  int len = cfg->gantry->calibrate();
  if (len > 0) {
    cfg->gantry->setJointLimits(GANTRY_X_MIN_MM, (float)len,
                                GANTRY_Y_MIN_MM, GANTRY_Y_MAX_MM,
                                GANTRY_THETA_MIN_DEG, GANTRY_THETA_MAX_DEG);
    g_calibratedThisSession = true;
    ESP_LOGI(TAG, "OK Calibrated length: %d mm", len);
    ESP_LOGI(TAG, "OK X joint max updated from calibration: %.1f mm", (float)len);
  } else if (cfg->gantry->isAbortRequested()) {
    g_calibratedThisSession = hadCalibration;
    ESP_LOGI(TAG, "Calibration aborted by stop request");
  } else {
    g_calibratedThisSession = hadCalibration;
    ESP_LOGI(TAG, "ERROR: Calibration failed");
    if (hadCalibration) {
      ESP_LOGI(TAG, "Keeping previous calibrated X max (no successful recalibration)");
    }
  }

  g_calibrationInProgress = false;
  vTaskDelete(nullptr);
}

void monitorControlVariableFlips(const GantryTestConsoleConfig *cfg) {
  if (cfg == nullptr || cfg->gantry == nullptr) {
    return;
  }

  ControlStateSnapshot cur = {};
  cur.enabled = cfg->gantry->isEnabled();
  cur.busy = cfg->gantry->isBusy();
  cur.alarm = cfg->gantry->isAlarmActive();
  cur.min_limit_active = (gpio_expander_read(cfg->limit_min_pin) == 0);
  cur.max_limit_active = (gpio_expander_read(cfg->limit_max_pin) == 0);
  cur.raw_alarm_level = (cfg->x_alarm_pin >= 0) ? gpio_expander_read(cfg->x_alarm_pin) : -1;

  if (!g_controlDebounce.initialized) {
    g_controlDebounce.initialized = true;
    g_controlDebounce.stable = cur;
    g_controlDebounce.pending = cur;
    ESP_LOGI(TAG,
             "CTRL INIT: enabled=%d busy=%d alarm=%d min=%d max=%d raw_alm=%d",
             (int)cur.enabled, (int)cur.busy, (int)cur.alarm,
             (int)cur.min_limit_active, (int)cur.max_limit_active,
             cur.raw_alarm_level);
    return;
  }

  auto debounceBool = [](bool sample, bool &stable, bool &pending, uint8_t &cnt,
                         uint8_t requiredSamples, const char *name) {
    if (sample == stable) {
      cnt = 0;
      pending = sample;
      return;
    }
    if (sample != pending) {
      pending = sample;
      cnt = 1;
      return;
    }
    if (cnt < 255) {
      cnt++;
    }
    if (cnt >= requiredSamples) {
      ESP_LOGI(TAG, "CTRL FLIP: %s %d -> %d", name, (int)stable, (int)sample);
      stable = sample;
      cnt = 0;
    }
  };

  auto debounceInt = [](int sample, int &stable, int &pending, uint8_t &cnt,
                        uint8_t requiredSamples, const char *name) {
    if (sample == stable) {
      cnt = 0;
      pending = sample;
      return;
    }
    if (sample != pending) {
      pending = sample;
      cnt = 1;
      return;
    }
    if (cnt < 255) {
      cnt++;
    }
    if (cnt >= requiredSamples) {
      ESP_LOGI(TAG, "CTRL FLIP: %s %d -> %d", name, stable, sample);
      stable = sample;
      cnt = 0;
    }
  };

  debounceBool(cur.enabled, g_controlDebounce.stable.enabled,
               g_controlDebounce.pending.enabled, g_controlDebounce.enabled_cnt,
               kFlipDebounceSamples, "motor_enabled");
  debounceBool(cur.busy, g_controlDebounce.stable.busy,
               g_controlDebounce.pending.busy, g_controlDebounce.busy_cnt,
               kFlipDebounceSamples, "busy");
  debounceBool(cur.alarm, g_controlDebounce.stable.alarm,
               g_controlDebounce.pending.alarm, g_controlDebounce.alarm_cnt,
               kFlipDebounceSamples, "alarm");
  debounceBool(cur.min_limit_active, g_controlDebounce.stable.min_limit_active,
               g_controlDebounce.pending.min_limit_active,
               g_controlDebounce.min_limit_cnt, kFlipDebounceSamples,
               "min_limit");
  debounceBool(cur.max_limit_active, g_controlDebounce.stable.max_limit_active,
               g_controlDebounce.pending.max_limit_active,
               g_controlDebounce.max_limit_cnt, kFlipDebounceSamples,
               "max_limit");
  debounceInt(cur.raw_alarm_level, g_controlDebounce.stable.raw_alarm_level,
              g_controlDebounce.pending.raw_alarm_level,
              g_controlDebounce.raw_alarm_cnt, kRawAlarmDebounceSamples,
              "raw_alarm_pin_level");
}

void printStatus(Gantry::Gantry *gantry) {
  if (gantry == nullptr) {
    ESP_LOGI(TAG, "ERROR: Gantry not initialized");
    return;
  }

  ESP_LOGI(TAG, "=== Gantry Status ===");
  Gantry::JointConfig current = gantry->getCurrentJointConfig();
  ESP_LOGI(TAG, "X Position: %.1f mm", current.x);
  ESP_LOGI(TAG, "X Encoder : %d pulses", gantry->getXEncoder());
  ESP_LOGI(TAG, "Y Position: %d mm", gantry->getCurrentY());
  ESP_LOGI(TAG, "Theta: %d deg", gantry->getCurrentTheta());
  ESP_LOGI(TAG, "Motor Enabled: %s", gantry->isEnabled() ? "Yes" : "No");
  ESP_LOGI(TAG, "Busy: %s", gantry->isBusy() ? "Yes" : "No");
  ESP_LOGI(TAG, "Alarm: %s", gantry->isAlarmActive() ? "Yes" : "No");
  ESP_LOGI(TAG, "Motion Profile: speed=%lu mm/s, theta=%lu deg/s, accel=%lu mm/s2, decel=%lu mm/s2",
           (unsigned long)g_moveSpeedMmPerS, (unsigned long)g_moveSpeedDegPerS,
           (unsigned long)g_moveAccelMmPerS2, (unsigned long)g_moveDecelMmPerS2);

  ESP_LOGI(TAG, "Joint Config: x=%.1f y=%.1f theta=%.1f", current.x, current.y, current.theta);

  Gantry::EndEffectorPose pose = gantry->getCurrentEndEffectorPose();
  ESP_LOGI(TAG, "End-Effector: x=%.1f y=%.1f z=%.1f theta=%.1f", pose.x, pose.y, pose.z,
           pose.theta);
}

void printLimits(const GantryTestConsoleConfig *cfg) {
  if (cfg == nullptr || cfg->gantry == nullptr) {
    ESP_LOGI(TAG, "ERROR: Gantry not initialized");
    return;
  }

  ESP_LOGI(TAG, "=== Limit Switches ===");
  uint8_t limit_min = gpio_expander_read(cfg->limit_min_pin);
  uint8_t limit_max = gpio_expander_read(cfg->limit_max_pin);
  if (cfg->use_mcp23s17) {
    ESP_LOGI(TAG, "X_LS_MIN (MCP23S17 PA%d / Home): %s", cfg->limit_min_pin,
             limit_min == 0 ? "ACTIVE (LOW)" : "open (HIGH)");
    ESP_LOGI(TAG, "X_LS_MAX (MCP23S17 PA%d / End):  %s", cfg->limit_max_pin,
             limit_max == 0 ? "ACTIVE (LOW)" : "open (HIGH)");
  } else {
    ESP_LOGI(TAG, "X_LS_MIN (GPIO %d / Home): %s", cfg->limit_min_pin,
             limit_min == 0 ? "ACTIVE (LOW)" : "open (HIGH)");
    ESP_LOGI(TAG, "X_LS_MAX (GPIO %d / End):  %s", cfg->limit_max_pin,
             limit_max == 0 ? "ACTIVE (LOW)" : "open (HIGH)");
  }
}

void printActivePins(const GantryTestConsoleConfig *cfg) {
  if (cfg == nullptr) {
    ESP_LOGI(TAG, "ERROR: Pin configuration not available");
    return;
  }

  ESP_LOGI(TAG, "=== Active Pin Configuration ===");
  ESP_LOGI(TAG, "Mode: %s", cfg->use_mcp23s17 ? "MCP23S17 IO Expander" : "Direct WT32 GPIO (temporary)");

  ESP_LOGI(TAG, "X Pulse      : %d", cfg->x_pulse_pin);
  ESP_LOGI(TAG, "X Dir        : %d", cfg->x_dir_pin);
  ESP_LOGI(TAG, "X Enable     : %d", cfg->x_enable_pin);
  ESP_LOGI(TAG, "X Alarm In   : %d", cfg->x_alarm_pin);
  if (cfg->x_alarm_reset_pin >= 0) {
    ESP_LOGI(TAG, "X Alarm Reset: %d", cfg->x_alarm_reset_pin);
  } else {
    ESP_LOGI(TAG, "X Alarm Reset: disabled");
  }
  ESP_LOGI(TAG, "X Encoder A  : %d", cfg->x_encoder_a_pin);
  ESP_LOGI(TAG, "X Encoder B  : %d", cfg->x_encoder_b_pin);
  ESP_LOGI(TAG, "Theta PWM    : %d", cfg->theta_pwm_pin);

  if (cfg->use_mcp23s17) {
    ESP_LOGI(TAG, "X Min Limit  : MCP P%d", cfg->limit_min_pin);
    ESP_LOGI(TAG, "X Max Limit  : MCP P%d", cfg->limit_max_pin);
  } else {
    ESP_LOGI(TAG, "X Min Limit  : GPIO %d", cfg->limit_min_pin);
    ESP_LOGI(TAG, "X Max Limit  : GPIO %d", cfg->limit_max_pin);
  }

  ESP_LOGI(TAG, "========================================");
}

void processCommand(const GantryTestConsoleConfig *cfg, const char *cmd) {
  if (cfg == nullptr || cmd == nullptr || strlen(cmd) == 0) {
    return;
  }

  char cmdLower[256];
  strncpy(cmdLower, cmd, sizeof(cmdLower) - 1);
  cmdLower[sizeof(cmdLower) - 1] = '\0';
  for (int i = 0; cmdLower[i]; i++) {
    cmdLower[i] = static_cast<char>(tolower(cmdLower[i]));
  }

  if (cfg->gantry == nullptr && strcmp(cmdLower, "help") != 0) {
    ESP_LOGI(TAG, "ERROR: Gantry not initialized");
    return;
  }

  if (strcmp(cmdLower, "help") == 0 || strcmp(cmdLower, "?") == 0) {
    gantryTestPrintHelp();
  } else if (strcmp(cmdLower, "status") == 0) {
    printStatus(cfg->gantry);
  } else if (strcmp(cmdLower, "limits") == 0) {
    printLimits(cfg);
  } else if (strcmp(cmdLower, "pins") == 0) {
    printActivePins(cfg);
  } else if (strcmp(cmdLower, "enable") == 0) {
    cfg->gantry->enable();
    if (cfg->gantry->isEnabled()) {
      ESP_LOGI(TAG, "OK Motors enabled");
    } else {
      ESP_LOGE(TAG, "ERROR: Motor enable failed (check alarm/driver state)");
    }
  } else if (strcmp(cmdLower, "disable") == 0) {
    cfg->gantry->disable();
    ESP_LOGI(TAG, "OK Motors disabled");
  } else if (strcmp(cmdLower, "home") == 0) {
    if (!cfg->gantry->isEnabled()) {
      ESP_LOGE(TAG, "ERROR: Motors not enabled");
      return;
    }
    ESP_LOGI(TAG, "Starting homing sequence...");

    // Snapshot pre-home MIN state to detect "already home" no-motion case.
    bool minWasActive = (gpio_expander_read(cfg->limit_min_pin) == 0);
    cfg->gantry->home();
    vTaskDelay(pdMS_TO_TICKS(20));

    // If homing did not even start, do not report success.
    if (!cfg->gantry->isBusy()) {
      bool minIsActive = (gpio_expander_read(cfg->limit_min_pin) == 0);
      if (minWasActive || minIsActive) {
        g_homeCompletedThisSession = true;
        ESP_LOGI(TAG, "OK Homing skipped: already at MIN/home switch");
      } else {
        ESP_LOGE(TAG, "ERROR: Homing did not start");
        ESP_LOGI(TAG, "Check motor enable, alarm status, and limit switch wiring");
      }
      return;
    }
    g_homeCompletedThisSession = true;
    ESP_LOGI(TAG, "OK Homing started (use 'stop' to abort, 'status' to monitor)");
  } else if (strcmp(cmdLower, "calibrate") == 0) {
    if (!g_homeCompletedThisSession) {
      ESP_LOGE(TAG, "ERROR: Run 'home' first after startup");
      return;
    }
    if (g_calibrationInProgress) {
      ESP_LOGI(TAG, "Calibration is already in progress");
      return;
    }
    g_calibrationInProgress = true;
    g_calibratedThisSession = false;
    BaseType_t taskOk = xTaskCreatePinnedToCore(
        calibrationTask, "CalibrateTask", 4096, (void *)cfg, 2, nullptr, CONSOLE_TASK_CORE);
    if (taskOk != pdPASS) {
      g_calibrationInProgress = false;
      ESP_LOGE(TAG, "ERROR: Failed to start calibration task");
    } else {
      ESP_LOGI(TAG, "Calibration started (use 'stop' to abort)");
    }
  } else if (strncmp(cmdLower, "speed", 5) == 0) {
    int speedMm = -1;
    int speedDeg = -1;
    int parsed = sscanf(cmd, "speed %d %d", &speedMm, &speedDeg);
    if (parsed < 1 || speedMm <= 0) {
      ESP_LOGI(TAG, "ERROR: Usage: speed <mm_per_s> [deg_per_s]");
      return;
    }
    g_moveSpeedMmPerS = (uint32_t)speedMm;
    if (parsed >= 2 && speedDeg > 0) {
      g_moveSpeedDegPerS = (uint32_t)speedDeg;
    }
    ESP_LOGI(TAG, "OK Speed updated: %lu mm/s, theta=%lu deg/s",
             (unsigned long)g_moveSpeedMmPerS, (unsigned long)g_moveSpeedDegPerS);
  } else if (strncmp(cmdLower, "accel", 5) == 0) {
    int accel = -1;
    int decel = -1;
    int parsed = sscanf(cmd, "accel %d %d", &accel, &decel);
    if (parsed < 1 || accel < 0) {
      ESP_LOGI(TAG, "ERROR: Usage: accel <mm_per_s2> [decel_mm_per_s2]");
      return;
    }
    g_moveAccelMmPerS2 = (uint32_t)accel;
    if (parsed >= 2 && decel >= 0) {
      g_moveDecelMmPerS2 = (uint32_t)decel;
    } else {
      g_moveDecelMmPerS2 = (uint32_t)accel;
    }
    ESP_LOGI(TAG, "OK Accel updated: accel=%lu mm/s2, decel=%lu mm/s2",
             (unsigned long)g_moveAccelMmPerS2, (unsigned long)g_moveDecelMmPerS2);
  } else if (strncmp(cmdLower, "move", 4) == 0) {
    if (!g_homeCompletedThisSession || !g_calibratedThisSession) {
      ESP_LOGE(TAG, "ERROR: Move blocked. Run 'home' then 'calibrate' after every startup.");
      return;
    }
    float x = 0.0f;
    float y = 0.0f;
    float theta = 0.0f;
    int parsed = sscanf(cmd, "move %f %f %f", &x, &y, &theta);
    if (parsed < 3) {
      ESP_LOGI(TAG, "ERROR: Usage: move <x_mm> <y_mm> <theta_deg>");
      return;
    }

    Gantry::JointConfig target;
    target.x = x;
    target.y = y;
    target.theta = theta;

    ESP_LOGI(TAG, "Moving to: x=%.1f y=%.1f theta=%.1f", x, y, theta);
    Gantry::GantryError result = cfg->gantry->moveTo(
        target, g_moveSpeedMmPerS, g_moveSpeedDegPerS, g_moveAccelMmPerS2, g_moveDecelMmPerS2);
    if (result == Gantry::GantryError::OK) {
      ESP_LOGI(TAG, "OK Move started (use 'stop' to abort, 'status' to monitor)");
    } else {
      ESP_LOGE(TAG, "ERROR: Move failed: %d", (int)result);
    }
  } else if (strncmp(cmdLower, "grip", 4) == 0) {
    int value = 0;
    int parsed = sscanf(cmd, "grip %d", &value);
    if (parsed < 1) {
      ESP_LOGI(TAG, "ERROR: Usage: grip <0|1>");
      return;
    }
    cfg->gantry->grip(value != 0);
    ESP_LOGI(TAG, "OK Gripper %s", value ? "closed" : "opened");
  } else if (strcmp(cmdLower, "stop") == 0) {
    cfg->gantry->requestAbort();
    cfg->gantry->disable();
    ESP_LOGI(TAG, "OK Stop requested (motors disabled immediately)");
    if (g_calibrationInProgress) {
      ESP_LOGI(TAG, "Calibration abort requested");
    }
  } else if (strcmp(cmdLower, "alarmreset") == 0 || strcmp(cmdLower, "arst") == 0) {
    if (cfg->gantry->clearAlarm()) {
      ESP_LOGI(TAG, "OK Alarm reset pulse sent");
    } else {
      ESP_LOGE(TAG, "ERROR: Alarm reset failed (ARST pin may be disabled)");
    }
  } else if (strcmp(cmdLower, "selftest") == 0) {
    BasicTestSummary result = runBasicTests();
    ESP_LOGI(TAG, "Selftest complete: passed=%d failed=%d", result.passed, result.failed);
  } else {
    ESP_LOGE(TAG, "ERROR: Unknown command: %s", cmd);
    ESP_LOGI(TAG, "Type 'help' for available commands");
  }
}
}  // namespace

void gantryTestPrintHelp() {
  ESP_LOGI(TAG, "========================================");
  ESP_LOGI(TAG, "Gantry Library Example - Commands:");
  ESP_LOGI(TAG, "========================================");
  ESP_LOGI(TAG, "  help                 - show this help");
  ESP_LOGI(TAG, "  status               - print gantry status");
  ESP_LOGI(TAG, "  limits               - read limit switches");
  ESP_LOGI(TAG, "  pins                 - print active pin configuration");
  ESP_LOGI(TAG, "  enable               - enable motors");
  ESP_LOGI(TAG, "  disable              - disable motors");
  ESP_LOGI(TAG, "  home                 - home X-axis");
  ESP_LOGI(TAG, "  calibrate            - calibrate X-axis and set X max from result");
  ESP_LOGI(TAG, "  speed <mm/s> [deg/s] - set move speed");
  ESP_LOGI(TAG, "  accel <a> [d]        - set accel/decel (mm/s2)");
  ESP_LOGI(TAG, "  move <x> <y> <t>     - move to position (requires home+calibrate this startup)");
  ESP_LOGI(TAG, "  grip <0|1>           - control gripper (0=open, 1=close)");
  ESP_LOGI(TAG, "  stop                 - stop all motion");
  ESP_LOGI(TAG, "  alarmreset | arst    - pulse alarm reset output (ARST)");
  ESP_LOGI(TAG, "  selftest             - run basic math/config tests");
  ESP_LOGI(TAG, "");
}

void gantryTestConsoleTask(void *param) {
  auto *cfg = static_cast<GantryTestConsoleConfig *>(param);

  char inputLine[256];
  size_t inputIndex = 0;

  ESP_LOGI(TAG, "Serial task started");
  ESP_LOGI(TAG, "Type 'help' for commands");

  while (1) {
    monitorControlVariableFlips(cfg);
    logLiveMotionState(cfg);

    int c = getchar();
    if (c >= 0) {
      if (c == '\n' || c == '\r' || c == ';') {
        if (inputIndex > 0) {
          inputLine[inputIndex] = '\0';
          ESP_LOGI(TAG, "[RX] %s", inputLine);
          processCommand(cfg, inputLine);
          inputIndex = 0;
        }
      } else if (c >= 32 && c <= 126 && inputIndex < sizeof(inputLine) - 1) {
        inputLine[inputIndex++] = static_cast<char>(c);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

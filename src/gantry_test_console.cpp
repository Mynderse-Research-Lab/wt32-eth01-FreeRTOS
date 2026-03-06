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
enum class LinearUnitMode { MM = 0, INCH = 1 };
static uint32_t g_moveSpeedMmPerS = 50;
static uint32_t g_moveSpeedDegPerS = 30;
static uint32_t g_moveAccelMmPerS2 = 0;
static uint32_t g_moveDecelMmPerS2 = 0;
static bool g_motionProfileRangeLimitEnabled = true;
static LinearUnitMode g_linearUnitMode = LinearUnitMode::MM;
static uint32_t g_lastLiveMotionLogMs = 0;
static bool g_liveMotionWasBusy = false;

static constexpr float kMmPerInch = 25.4f;
static constexpr uint32_t kMinSpeedMmPerS = 1;
static constexpr uint32_t kMaxSpeedMmPerS = 500;
static constexpr uint32_t kMinAccelMmPerS2 = 100;
static constexpr uint32_t kMaxAccelMmPerS2 = 900;

uint32_t applyRangeLimitU32(uint32_t value, uint32_t minValue, uint32_t maxValue, bool enabled) {
  if (!enabled) {
    return value;
  }
  if (value < minValue) {
    return minValue;
  }
  if (value > maxValue) {
    return maxValue;
  }
  return value;
}

const char *getLinearUnitLabel() {
  return (g_linearUnitMode == LinearUnitMode::INCH) ? "in" : "mm";
}

float convertMmToSelected(float valueMm) {
  if (g_linearUnitMode == LinearUnitMode::INCH) {
    return valueMm / kMmPerInch;
  }
  return valueMm;
}

float convertSelectedToMm(float valueSelected) {
  if (g_linearUnitMode == LinearUnitMode::INCH) {
    return valueSelected * kMmPerInch;
  }
  return valueSelected;
}

void logLiveMotionState(const GantryTestConsoleConfig *cfg) {
  if (cfg == nullptr || cfg->gantry == nullptr) {
    return;
  }

  const bool busy = cfg->gantry->isBusy();
  const uint32_t nowMs = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
  constexpr uint32_t kLiveLogIntervalBusyMs = 100;
  constexpr uint32_t kLiveLogIntervalIdleMs = 5000;
  const uint32_t intervalMs = busy ? kLiveLogIntervalBusyMs : kLiveLogIntervalIdleMs;
  const bool stateChanged = (busy != g_liveMotionWasBusy);

  // While motion is active, emit a periodic dual-source X report:
  // - x_cmd: commanded/driver position
  // - x_enc: encoder feedback position
  if (stateChanged || g_lastLiveMotionLogMs == 0 ||
      (nowMs - g_lastLiveMotionLogMs) >= intervalMs) {
    const Gantry::JointConfig joint = cfg->gantry->getCurrentJointConfig();
    const float xCmdMm = cfg->gantry->getXCommandedMm();
    const float xEncMm = cfg->gantry->getXEncoderMm();
    const float xCmdDisp = convertMmToSelected(xCmdMm);
    const float xEncDisp = convertMmToSelected(xEncMm);
    const float yDisp = convertMmToSelected(joint.y);
    ESP_LOGI(TAG, "LIVE POS: x_cmd=%.2f %s, x_enc=%.2f %s, y=%.2f %s, theta=%.2f deg",
             xCmdDisp, getLinearUnitLabel(), xEncDisp, getLinearUnitLabel(),
             yDisp, getLinearUnitLabel(), joint.theta);
    g_lastLiveMotionLogMs = nowMs;
  }
  g_liveMotionWasBusy = busy;
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
  const float xDisp = convertMmToSelected(current.x);
  const float yDisp = convertMmToSelected((float)gantry->getCurrentY());
  ESP_LOGI(TAG, "X Position: %.3f %s", xDisp, getLinearUnitLabel());
  ESP_LOGI(TAG, "X Encoder : %d pulses", gantry->getXEncoder());
  ESP_LOGI(TAG, "Y Position: %.3f %s", yDisp, getLinearUnitLabel());
  ESP_LOGI(TAG, "Theta: %d deg", gantry->getCurrentTheta());
  ESP_LOGI(TAG, "Motor Enabled: %s", gantry->isEnabled() ? "Yes" : "No");
  ESP_LOGI(TAG, "Busy: %s", gantry->isBusy() ? "Yes" : "No");
  ESP_LOGI(TAG, "Alarm: %s", gantry->isAlarmActive() ? "Yes" : "No");
  ESP_LOGI(TAG, "Motion Profile: speed=%.3f %s/s, theta=%lu deg/s, accel=%.3f %s/s2, decel=%.3f %s/s2",
           convertMmToSelected((float)g_moveSpeedMmPerS), getLinearUnitLabel(),
           (unsigned long)g_moveSpeedDegPerS,
           convertMmToSelected((float)g_moveAccelMmPerS2), getLinearUnitLabel(),
           convertMmToSelected((float)g_moveDecelMmPerS2), getLinearUnitLabel());
  ESP_LOGI(TAG, "Range Limits: %s (speed:%.3f-%.3f %s/s, accel/decel:%.3f-%.3f %s/s2)",
           g_motionProfileRangeLimitEnabled ? "ENABLED" : "DISABLED",
           convertMmToSelected((float)kMinSpeedMmPerS), convertMmToSelected((float)kMaxSpeedMmPerS),
           getLinearUnitLabel(),
           convertMmToSelected((float)kMinAccelMmPerS2), convertMmToSelected((float)kMaxAccelMmPerS2),
           getLinearUnitLabel());
  ESP_LOGI(TAG, "Units: linear=%s (internal mm)", getLinearUnitLabel());

  ESP_LOGI(TAG, "Joint Config: x=%.3f %s, y=%.3f %s, theta=%.1f deg",
           convertMmToSelected(current.x), getLinearUnitLabel(),
           convertMmToSelected(current.y), getLinearUnitLabel(), current.theta);

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
  ESP_LOGI(TAG, "X_LS_MIN (MCP23S17 PA%d / Home): %s", cfg->limit_min_pin,
           limit_min == 0 ? "ACTIVE (LOW)" : "open (HIGH)");
  ESP_LOGI(TAG, "X_LS_MAX (MCP23S17 PA%d / End):  %s", cfg->limit_max_pin,
           limit_max == 0 ? "ACTIVE (LOW)" : "open (HIGH)");
}

void printActivePins(const GantryTestConsoleConfig *cfg) {
  if (cfg == nullptr) {
    ESP_LOGI(TAG, "ERROR: Pin configuration not available");
    return;
  }

  ESP_LOGI(TAG, "=== Active Pin Configuration ===");
  ESP_LOGI(TAG, "Mode: MCP23S17 IO Expander");

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

  ESP_LOGI(TAG, "X Min Limit  : MCP P%d", cfg->limit_min_pin);
  ESP_LOGI(TAG, "X Max Limit  : MCP P%d", cfg->limit_max_pin);

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
    const uint32_t requestedSpeedMm = (uint32_t)convertSelectedToMm((float)speedMm);
    const uint32_t clampedSpeedMm =
        applyRangeLimitU32(requestedSpeedMm, kMinSpeedMmPerS, kMaxSpeedMmPerS,
                           g_motionProfileRangeLimitEnabled);
    g_moveSpeedMmPerS = clampedSpeedMm;
    if (parsed >= 2 && speedDeg > 0) {
      g_moveSpeedDegPerS = (uint32_t)speedDeg;
    }
    if (g_motionProfileRangeLimitEnabled && clampedSpeedMm != requestedSpeedMm) {
      ESP_LOGW(TAG, "Speed clamped: requested=%.3f %s/s -> applied=%.3f %s/s",
               convertMmToSelected((float)requestedSpeedMm), getLinearUnitLabel(),
               convertMmToSelected((float)clampedSpeedMm), getLinearUnitLabel());
    }
    ESP_LOGI(TAG, "OK Speed updated: %.3f %s/s, theta=%lu deg/s",
             convertMmToSelected((float)g_moveSpeedMmPerS), getLinearUnitLabel(),
             (unsigned long)g_moveSpeedDegPerS);
  } else if (strncmp(cmdLower, "accel", 5) == 0) {
    int accel = -1;
    int decel = -1;
    int parsed = sscanf(cmd, "accel %d %d", &accel, &decel);
    if (parsed < 1 || accel <= 0) {
      ESP_LOGI(TAG, "ERROR: Usage: accel <mm_per_s2> [decel_mm_per_s2] (values must be > 0)");
      return;
    }
    if (parsed >= 2 && decel <= 0) {
      ESP_LOGI(TAG, "ERROR: Decel must be > 0");
      return;
    }
    const uint32_t requestedAccel = (uint32_t)convertSelectedToMm((float)accel);
    const uint32_t requestedDecel =
        (parsed >= 2 && decel > 0) ? (uint32_t)convertSelectedToMm((float)decel)
                                   : (uint32_t)convertSelectedToMm((float)accel);
    const uint32_t clampedAccel =
        applyRangeLimitU32(requestedAccel, kMinAccelMmPerS2, kMaxAccelMmPerS2,
                           g_motionProfileRangeLimitEnabled);
    const uint32_t clampedDecel =
        applyRangeLimitU32(requestedDecel, kMinAccelMmPerS2, kMaxAccelMmPerS2,
                           g_motionProfileRangeLimitEnabled);

    g_moveAccelMmPerS2 = clampedAccel;
    if (parsed >= 2 && decel >= 0) {
      g_moveDecelMmPerS2 = clampedDecel;
    } else {
      g_moveDecelMmPerS2 = clampedAccel;
    }
    if (g_motionProfileRangeLimitEnabled &&
        (clampedAccel != requestedAccel || clampedDecel != requestedDecel)) {
      ESP_LOGW(TAG, "Accel/decel clamped: requested=(%.3f,%.3f) %s/s2 -> applied=(%.3f,%.3f) %s/s2",
               convertMmToSelected((float)requestedAccel), convertMmToSelected((float)requestedDecel),
               getLinearUnitLabel(),
               convertMmToSelected((float)clampedAccel), convertMmToSelected((float)clampedDecel),
               getLinearUnitLabel());
    }
    ESP_LOGI(TAG, "OK Accel updated: accel=%.3f %s/s2, decel=%.3f %s/s2",
             convertMmToSelected((float)g_moveAccelMmPerS2), getLinearUnitLabel(),
             convertMmToSelected((float)g_moveDecelMmPerS2), getLinearUnitLabel());
  } else if (strncmp(cmdLower, "units", 5) == 0) {
    char unitStr[16] = {0};
    int parsed = sscanf(cmd, "units %15s", unitStr);
    if (parsed < 1) {
      ESP_LOGI(TAG, "ERROR: Usage: units <mm|in>");
      return;
    }
    for (int i = 0; unitStr[i]; i++) {
      unitStr[i] = static_cast<char>(tolower(unitStr[i]));
    }
    if (strcmp(unitStr, "mm") == 0) {
      g_linearUnitMode = LinearUnitMode::MM;
    } else if (strcmp(unitStr, "in") == 0 || strcmp(unitStr, "inch") == 0 || strcmp(unitStr, "inches") == 0) {
      g_linearUnitMode = LinearUnitMode::INCH;
    } else {
      ESP_LOGI(TAG, "ERROR: Usage: units <mm|in>");
      return;
    }
    ESP_LOGI(TAG, "OK Linear units set to %s (internal storage remains mm)", getLinearUnitLabel());
  } else if (strncmp(cmdLower, "rangelimit", 10) == 0) {
    int enabled = -1;
    int parsed = sscanf(cmd, "rangelimit %d", &enabled);
    if (parsed < 1 || (enabled != 0 && enabled != 1)) {
      ESP_LOGI(TAG, "ERROR: Usage: rangelimit <0|1>");
      return;
    }
    g_motionProfileRangeLimitEnabled = (enabled == 1);
    ESP_LOGI(TAG, "OK Range limits %s", g_motionProfileRangeLimitEnabled ? "ENABLED" : "DISABLED");
    ESP_LOGI(TAG, "Configured ranges: speed=%lu..%lu mm/s, accel/decel=%lu..%lu mm/s2",
             (unsigned long)kMinSpeedMmPerS, (unsigned long)kMaxSpeedMmPerS,
             (unsigned long)kMinAccelMmPerS2, (unsigned long)kMaxAccelMmPerS2);
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
      ESP_LOGI(TAG, "ERROR: Usage: move <x_%s> <y_%s> <theta_deg>",
               getLinearUnitLabel(), getLinearUnitLabel());
      return;
    }

    Gantry::JointConfig target;
    target.x = convertSelectedToMm(x);
    target.y = convertSelectedToMm(y);
    target.theta = theta;

    ESP_LOGI(TAG, "Moving to: x=%.3f %s, y=%.3f %s, theta=%.1f deg",
             x, getLinearUnitLabel(), y, getLinearUnitLabel(), theta);
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
  ESP_LOGI(TAG, "  units <mm|in>        - set linear input/output units");
  ESP_LOGI(TAG, "  speed <v> [deg/s]    - set move speed (v in selected linear units/s)");
  ESP_LOGI(TAG, "  accel <a> [d]        - set accel/decel (>0, selected linear units/s2)");
  ESP_LOGI(TAG, "  rangelimit <0|1>     - enable/disable speed+accel/decel range clamps");
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

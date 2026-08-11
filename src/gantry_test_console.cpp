#include "gantry_test_console.h"

#if CONFIG_GANTRY_SELFTEST
#include "basic_tests.h"
#endif
#include "EipScannerTask.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "ethernet_app_config.h"
#include "gantry_app_constants.h"
#include "axis_drivetrain_params.h"
#include "gpio_expander.h"
#include "MCP23S17.h"
#include "esp_log.h"
#include "driver/gpio.h"

#ifndef MCP_DEBUG_CMDS
#define MCP_DEBUG_CMDS 1
#endif

#include <ctype.h>
#include <cstdarg>
#include <cmath>
#include <cstdio>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

extern "C" int getchar(void);

namespace {
static const char *TAG = "GantryConsole";

SemaphoreHandle_t g_consoleCmdMutex = nullptr;

GantryConsoleReplyFn g_replyFn = nullptr;
void *g_replyCtx = nullptr;
GantryConsoleReplyFn g_sessionFn = nullptr;
void *g_sessionCtx = nullptr;
vprintf_like_t g_prevVprintf = nullptr;
bool g_teeInstalled = false;

int consoleTeeVprintf(const char *fmt, va_list args);

/** ESP-IDF log letter after optional ANSI color (E/W/I/D/V). */
char espLogLetter(const char *s) {
  while (*s == '\033') {
    ++s;
    if (*s == '[') {
      ++s;
      while (*s != '\0' && *s != 'm') {
        ++s;
      }
      if (*s == 'm') {
        ++s;
      }
    }
  }
  while (*s == ' ' || *s == '\t') {
    ++s;
  }
  const char c = *s;
  if (c == 'E' || c == 'W' || c == 'I' || c == 'D' || c == 'V') {
    return c;
  }
  return 0;
}

int espLogSeverity(char letter) {
  switch (letter) {
    case 'E':
      return CONSOLE_TCP_LOG_SEV_ERR;
    case 'W':
      return CONSOLE_TCP_LOG_SEV_WARN;
    case 'I':
      return CONSOLE_TCP_LOG_SEV_INFO;
    case 'D':
    case 'V':
      return CONSOLE_TCP_LOG_SEV_DBUG;
    default:
      return CONSOLE_TCP_LOG_SEV_INFO;
  }
}

bool lanLogAllowsLine(const char *line) {
  if (!CONSOLE_TCP_LOG_ENABLE) {
    return false;
  }
  return espLogSeverity(espLogLetter(line)) <= CONSOLE_TCP_LOG_LEVEL;
}

void installTeeIfNeeded() {
  if (g_teeInstalled) {
    return;
  }
  g_prevVprintf = esp_log_set_vprintf(consoleTeeVprintf);
  g_teeInstalled = true;
}

void uninstallTeeIfIdle() {
  if (!g_teeInstalled) {
    return;
  }
  if (g_sessionFn != nullptr || g_replyFn != nullptr) {
    return;
  }
  esp_log_set_vprintf(g_prevVprintf != nullptr ? g_prevVprintf : vprintf);
  g_prevVprintf = nullptr;
  g_teeInstalled = false;
}

int consoleTeeVprintf(const char *fmt, va_list args) {
  char buf[512];
  va_list copy;
  va_copy(copy, args);
  int n = vsnprintf(buf, sizeof(buf), fmt, copy);
  va_end(copy);
  if (n > 0) {
    buf[sizeof(buf) - 1] = '\0';
    if (g_sessionFn != nullptr) {
      if (lanLogAllowsLine(buf)) {
        g_sessionFn(g_sessionCtx, buf);
      }
    } else if (g_replyFn != nullptr) {
      g_replyFn(g_replyCtx, buf);
    }
  }
  if (g_prevVprintf != nullptr) {
    return g_prevVprintf(fmt, args);
  }
  return vprintf(fmt, args);
}

void ensureConsoleMutex() {
  if (g_consoleCmdMutex == nullptr) {
    g_consoleCmdMutex = xSemaphoreCreateMutex();
  }
}

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
static bool g_homeZCompletedThisSession = false;
static bool g_calibratedZThisSession = false;
static bool g_calibrationInProgress = false;
enum class LinearUnitMode { MM = 0, INCH = 1 };
static uint32_t g_moveSpeedMmPerS = GANTRY_DEFAULT_SPEED_MM_PER_S;
static uint32_t g_moveSpeedDegPerS = GANTRY_DEFAULT_SPEED_DEG_PER_S;
static uint32_t g_moveAccelMmPerS2 = GANTRY_DEFAULT_ACCEL_MM_PER_S2;
static uint32_t g_moveDecelMmPerS2 = GANTRY_DEFAULT_DECEL_MM_PER_S2;
static bool g_motionProfileRangeLimitEnabled = true;
static LinearUnitMode g_linearUnitMode = LinearUnitMode::MM;
// LIVE POS periodic logger frequency control.
// 0 Hz disables periodic output (default).
static uint32_t g_livePosFrequencyHz = 0;
// Per-axis MOVE log periodic rate (Hz). 0 disables periodic output; the
// start/end transition events from the axis classes always fire.
static uint32_t g_axisLogFrequencyHz = 0;
static uint32_t g_lastLiveMotionLogMs = 0;
static bool g_liveMotionWasBusy = false;

bool limitsWired(const GantryTestConsoleConfig *cfg) {
  return cfg != nullptr && cfg->limit_switches_active &&
         cfg->limit_min_pin >= 0 && cfg->limit_max_pin >= 0;
}

// GPIO pin limits or EIP drive-managed (A014/A015) external switches.
bool physicalLimitsAvailable(const GantryTestConsoleConfig *cfg) {
  if (limitsWired(cfg)) return true;
  return cfg != nullptr && cfg->gantry != nullptr &&
         cfg->gantry->driveManagedLimitsEnabled();
}

static constexpr float kMmPerInch = 25.4f;
static constexpr uint32_t kMinSpeedMmPerS = 1;
static constexpr uint32_t kMinAccelMmPerS2 = 100;

// Shared 2-D path envelope: console speed/accel/decel are the RESULTANT along
// the X-Z path. Protective clamp: never exceed Z ballscrew critical-RPM speed.
uint32_t maxLinearSpeedMmPerS() {
  uint32_t pathCap = GANTRY_PATH_MAX_SPEED_MM_PER_S;
  const double crit = AXIS_Z_SPEED_CAP_FROM_CRITICAL_RPM_MM_PER_S;
  if (crit > 0.0 && crit < static_cast<double>(pathCap)) {
    pathCap = static_cast<uint32_t>(crit);
  }
  return pathCap;
}

uint32_t maxLinearAccelMmPerS2() {
  return GANTRY_PATH_MAX_ACCEL_MM_PER_S2;
}

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
  if (g_livePosFrequencyHz == 0) {
    g_liveMotionWasBusy = busy;
    return;
  }
  const uint32_t nowMs = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
  uint32_t intervalMs = 1000u / g_livePosFrequencyHz;
  if (intervalMs == 0) {
    intervalMs = 1;
  }
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
    const float zDisp = convertMmToSelected(joint.z);
    ESP_LOGI(TAG, "LIVE POS: x_cmd=%.2f %s, x_enc=%.2f %s, z=%.2f %s, theta=%.2f deg",
             xCmdDisp, getLinearUnitLabel(), xEncDisp, getLinearUnitLabel(),
             zDisp, getLinearUnitLabel(), joint.theta);
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
    const float zMax =
        cfg->gantry->zAxisLengthMm() > 0
            ? static_cast<float>(cfg->gantry->zAxisLengthMm())
            : AXIS_Z_HARD_LIMIT_MAX_MM;
    cfg->gantry->setJointLimits(AXIS_X_HARD_LIMIT_MIN_MM, (float)len,
                                AXIS_Z_HARD_LIMIT_MIN_MM, zMax,
                                AXIS_THETA_HARD_LIMIT_MIN_DEG, AXIS_THETA_HARD_LIMIT_MAX_DEG);
    g_calibratedThisSession = true;
    ESP_LOGI(TAG, "OK Calibrated length: %d mm", len);
    ESP_LOGI(TAG, "OK X joint max updated from calibration: %.1f mm", (float)len);
  } else if (cfg->gantry->isAbortRequested()) {
    g_calibratedThisSession = hadCalibration;
    ESP_LOGI(TAG, "Calibration aborted by stop request");
  } else {
    g_calibratedThisSession = hadCalibration;
    ESP_LOGE(TAG, "Calibration failed");
    if (hadCalibration) {
      ESP_LOGI(TAG, "Keeping previous calibrated X max (no successful recalibration)");
    }
  }

  g_calibrationInProgress = false;
  vTaskDelete(nullptr);
}

void zCalibrationTask(void *param) {
  auto *cfg = static_cast<GantryTestConsoleConfig *>(param);
  if (cfg == nullptr || cfg->gantry == nullptr) {
    g_calibrationInProgress = false;
    vTaskDelete(nullptr);
    return;
  }

  const bool hadCalibration = g_calibratedZThisSession;
  ESP_LOGI(TAG, "Z calibration task: started");
  int len = cfg->gantry->calibrateZ();
  if (len > 0) {
    g_calibratedZThisSession = true;
    ESP_LOGI(TAG, "OK Z Calibrated length: %d mm", len);
    ESP_LOGI(TAG, "OK Z joint max updated from calibration: %.1f mm", (float)len);
  } else if (cfg->gantry->isAbortRequested()) {
    g_calibratedZThisSession = hadCalibration;
    ESP_LOGI(TAG, "Z calibration aborted by stop request");
  } else {
    g_calibratedZThisSession = hadCalibration;
    ESP_LOGE(TAG, "Z calibration failed");
    if (hadCalibration) {
      ESP_LOGI(TAG, "Keeping previous calibrated Z max (no successful recalibration)");
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
  if (limitsWired(cfg)) {
    cur.min_limit_active = (gpio_expander_read(cfg->limit_min_pin) == 0);
    cur.max_limit_active = (gpio_expander_read(cfg->limit_max_pin) == 0);
  } else {
    cur.min_limit_active = false;
    cur.max_limit_active = false;
  }
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
    ESP_LOGE(TAG, "Gantry not initialized");
    return;
  }

  ESP_LOGI(TAG, "=== Gantry Status ===");
  Gantry::JointConfig current = gantry->getCurrentJointConfig();
  const float xDisp = convertMmToSelected(current.x);
  const float zDisp = convertMmToSelected((float)gantry->getCurrentZ());
  ESP_LOGI(TAG, "X Position: %.3f %s", xDisp, getLinearUnitLabel());
  ESP_LOGI(TAG, "X Encoder : %d pulses", gantry->getXEncoder());
  ESP_LOGI(TAG, "Z Position: %.3f %s (+Z = up; belt = 0)", zDisp, getLinearUnitLabel());
  ESP_LOGI(TAG, "Theta: %d deg", gantry->getCurrentTheta());
  ESP_LOGI(TAG, "Motor Enabled: %s", gantry->isEnabled() ? "Yes" : "No");
  ESP_LOGI(TAG, "Busy: %s", gantry->isBusy() ? "Yes" : "No");
  ESP_LOGI(TAG, "Alarm: %s", gantry->isAlarmActive() ? "Yes" : "No");
  {
    char xSum[192] = {};
    char zSum[192] = {};
    const bool xTrip = gantry->getXDriveAlarmSummary(xSum, sizeof(xSum));
    const bool zTrip = gantry->getZDriveAlarmSummary(zSum, sizeof(zSum));
    if (xTrip && xSum[0] && strcmp(xSum, "clear") != 0) {
      ESP_LOGW(TAG, "X drive: %s", xSum);
    }
    if (zTrip && zSum[0] && strcmp(zSum, "clear") != 0) {
      ESP_LOGW(TAG, "Z drive: %s", zSum);
    }
  }
  ESP_LOGI(TAG, "2-D Path Profile: speed=%.3f %s/s, theta=%lu deg/s, accel=%.3f %s/s2, decel=%.3f %s/s2",
           convertMmToSelected((float)g_moveSpeedMmPerS), getLinearUnitLabel(),
           (unsigned long)g_moveSpeedDegPerS,
           convertMmToSelected((float)g_moveAccelMmPerS2), getLinearUnitLabel(),
           convertMmToSelected((float)g_moveDecelMmPerS2), getLinearUnitLabel());
  ESP_LOGI(TAG, "Path Range Limits: %s (speed:%.3f-%.3f %s/s, accel/decel:%.3f-%.3f %s/s2)",
           g_motionProfileRangeLimitEnabled ? "ENABLED" : "DISABLED",
           convertMmToSelected((float)kMinSpeedMmPerS),
           convertMmToSelected((float)maxLinearSpeedMmPerS()),
           getLinearUnitLabel(),
           convertMmToSelected((float)kMinAccelMmPerS2),
           convertMmToSelected((float)maxLinearAccelMmPerS2()),
           getLinearUnitLabel());
  ESP_LOGI(TAG, "Units: linear=%s (internal mm)", getLinearUnitLabel());
  ESP_LOGI(TAG, "PUU scale: X=%.3f PUU/mm  Z=%.3f PUU/mm (use puuinfo / puucal)",
           gantry->getPulsesPerMm(), gantry->getZPulsesPerMm());

  ESP_LOGI(TAG, "Joint Config: x=%.3f %s, z=%.3f %s, theta=%.1f deg",
           convertMmToSelected(current.x), getLinearUnitLabel(),
           convertMmToSelected(current.z), getLinearUnitLabel(), current.theta);

  Gantry::EndEffectorPose pose = gantry->getCurrentEndEffectorPose();
  ESP_LOGI(TAG, "End-Effector: x=%.1f y=%.1f z=%.1f theta=%.1f", pose.x, pose.y, pose.z,
           pose.theta);
}

void printPuuInfo(Gantry::Gantry *gantry) {
  if (gantry == nullptr) {
    ESP_LOGE(TAG, "Gantry not initialized");
    return;
  }

  const float xPpm = gantry->getPulsesPerMm();
  const float zPpm = gantry->getZPulsesPerMm();
  const float xMm = gantry->getXEncoderMm();
  const float zMm = gantry->getZEncoderMm();
  const Gantry::JointConfig target = gantry->getTargetJointConfig();

  ESP_LOGI(TAG, "=== PUU / mm scaling ===");
  ESP_LOGI(TAG, "X: scale=%.4f PUU/mm  actual=%.3f mm (%d PUU)  target=%.3f mm  lead=%.1f mm/rev",
           xPpm, xMm, gantry->getXEncoderRaw(), target.x, AXIS_X_LEAD_MM_PER_REV);
  ESP_LOGI(TAG, "Z: scale=%.4f PUU/mm  actual=%.3f mm (%ld PUU)  target=%.3f mm  lead=%.1f mm/rev",
           zPpm, zMm, static_cast<long>(gantry->getZEncoderPulses()), target.z,
           AXIS_Z_LEAD_MM_PER_REV);
  ESP_LOGI(TAG, "Calibrate: move a known delta, measure travel, then:");
  ESP_LOGI(TAG, "  puucal <x|z> <commanded_mm> <measured_mm>");
  ESP_LOGI(TAG, "Suggested Kconfig: EIP_AXIS_X_PUU_PER_MM / EIP_AXIS_Z_PUU_PER_MM");
  ESP_LOGI(TAG, "Formula: new = current * (commanded / measured)");
}

void runPuuCalCommand(Gantry::Gantry *gantry, const char *cmd) {
  if (gantry == nullptr) {
    ESP_LOGE(TAG, "Gantry not initialized");
    return;
  }

  char axis = '\0';
  float commanded = 0.0f;
  float measured = 0.0f;
  if (sscanf(cmd, "puucal %c %f %f", &axis, &commanded, &measured) != 3) {
    ESP_LOGE(TAG, "Usage: puucal <x|z> <commanded_mm> <measured_mm>");
    return;
  }
  axis = static_cast<char>(tolower(static_cast<unsigned char>(axis)));
  if (axis != 'x' && axis != 'z') {
    ESP_LOGE(TAG, "Axis must be x or z");
    return;
  }
  if (commanded <= 0.0f || measured <= 0.0f) {
    ESP_LOGE(TAG, "commanded_mm and measured_mm must be > 0");
    return;
  }

  const float current =
      (axis == 'x') ? gantry->getPulsesPerMm() : gantry->getZPulsesPerMm();
  if (current <= 0.0f) {
    ESP_LOGE(TAG, "No PUU/mm scale available for axis %c", axis);
    return;
  }

  const double suggested =
      static_cast<double>(current) *
      (static_cast<double>(commanded) / static_cast<double>(measured));
  const float errPct =
      (measured - commanded) / commanded * 100.0f;

  ESP_LOGI(TAG, "=== PUU calibration (%c) ===", axis);
  ESP_LOGI(TAG, "Commanded=%.3f mm  Measured=%.3f mm  error=%+.2f%%",
           commanded, measured, errPct);
  ESP_LOGI(TAG, "Current scale=%.4f PUU/mm", current);
  ESP_LOGI(TAG, "Suggested scale=%.4f PUU/mm", suggested);
  if (axis == 'x') {
    ESP_LOGI(TAG, "Set CONFIG_EIP_AXIS_X_PUU_PER_MM=%.4f then rebuild/flash",
             suggested);
  } else {
    ESP_LOGI(TAG, "Set CONFIG_EIP_AXIS_Z_PUU_PER_MM=%.4f then rebuild/flash",
             suggested);
  }
  ESP_LOGI(TAG, "Target tol: X +/-0.08 mm (SCHUNK), Z +/-0.03 mm (first pass +/-0.5 mm OK)");
}

void printLimits(const GantryTestConsoleConfig *cfg) {
  if (cfg == nullptr || cfg->gantry == nullptr) {
    ESP_LOGE(TAG, "Gantry not initialized");
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
    ESP_LOGE(TAG, "Pin configuration not available");
    return;
  }

  ESP_LOGI(TAG, "=== Active Pins + Current States ===");

  uint8_t iodira = 0xFF, iodirb = 0xFF, gppua = 0x00, gppub = 0x00, gpioa = 0x00,
          gpiob = 0x00, olata = 0x00, olatb = 0x00;
  bool mcpRegsValid = false;
  mcp23s17_handle_t h = gpio_expander_get_mcp_handle();
  if (h != nullptr) {
    if (mcp23s17_debug_read_register(h, 0x00, &iodira) == ESP_OK &&
        mcp23s17_debug_read_register(h, 0x01, &iodirb) == ESP_OK &&
        mcp23s17_debug_read_register(h, 0x0C, &gppua) == ESP_OK &&
        mcp23s17_debug_read_register(h, 0x0D, &gppub) == ESP_OK &&
        mcp23s17_debug_read_register(h, 0x12, &gpioa) == ESP_OK &&
        mcp23s17_debug_read_register(h, 0x13, &gpiob) == ESP_OK &&
        mcp23s17_debug_read_register(h, 0x14, &olata) == ESP_OK &&
        mcp23s17_debug_read_register(h, 0x15, &olatb) == ESP_OK) {
      mcpRegsValid = true;
    }
  }

  auto printMcpPin = [&](const char *name, int pin) {
    if (pin < 0) {
      ESP_LOGI(TAG, "%-12s: disabled", name);
      return;
    }
    if (pin > 15) {
      ESP_LOGI(TAG, "%-12s: not MCP (pin=%d)", name, pin);
      return;
    }
    const uint8_t bit = (uint8_t)(pin & 0x07);
    if (!mcpRegsValid) {
      ESP_LOGI(TAG, "%-12s: MCP P%d (state unavailable)", name, pin);
      return;
    }
    const bool portA = (pin < 8);
    const uint8_t iodir = portA ? iodira : iodirb;
    const uint8_t gppu = portA ? gppua : gppub;
    const uint8_t gpio = portA ? gpioa : gpiob;
    const uint8_t olat = portA ? olata : olatb;
    const int dir = (iodir >> bit) & 0x1;
    const int pull = (gppu >> bit) & 0x1;
    const int level = (gpio >> bit) & 0x1;
    const int latch = (olat >> bit) & 0x1;
    ESP_LOGI(TAG, "%-12s: MCP P%d dir=%s pull=%d gpio=%d olat=%d", name, pin,
             dir ? "IN" : "OUT", pull, level, latch);
  };

  auto printDirectPin = [&](const char *name, int pin) {
    if (pin < 0) {
      ESP_LOGI(TAG, "%-12s: disabled", name);
      return;
    }
    const int level = gpio_get_level((gpio_num_t)pin);
    ESP_LOGI(TAG, "%-12s: GPIO %d level=%d", name, pin, level);
  };

  ESP_LOGI(TAG, "--- MCP23S17 pins ---");
  printMcpPin("X Dir", cfg->x_dir_pin);
  printMcpPin("X Enable", cfg->x_enable_pin);
  printMcpPin("X Alarm In", cfg->x_alarm_pin);
  printMcpPin("X Alarm Rst", cfg->x_alarm_reset_pin);
  printMcpPin("Z Alarm In", cfg->z_alarm_pin);
  printMcpPin("Z Alarm Rst", cfg->z_alarm_reset_pin);
  if (cfg->use_mcp23s17) {
    printMcpPin("X Min Limit", cfg->limit_min_pin);
    printMcpPin("X Max Limit", cfg->limit_max_pin);
  }

  ESP_LOGI(TAG, "--- ESP32 pins ---");
  printDirectPin("X Pulse", cfg->x_pulse_pin);
  printDirectPin("Z Pulse", cfg->z_pulse_pin);
  printDirectPin("Theta Pulse", cfg->theta_pulse_pin);
  printDirectPin("X Encoder A", cfg->x_encoder_a_pin);
  printDirectPin("X Encoder B", cfg->x_encoder_b_pin);
  printDirectPin("Z Encoder A", cfg->z_encoder_a_pin);
  printDirectPin("Z Encoder B", cfg->z_encoder_b_pin);
  if (!cfg->use_mcp23s17) {
    printDirectPin("X Min Limit", cfg->limit_min_pin);
    printDirectPin("X Max Limit", cfg->limit_max_pin);
  }

  ESP_LOGI(TAG, "LEDC: X ch %d, Z ch %d, Theta ch %d", cfg->x_pulse_ledc_channel,
           cfg->z_pulse_ledc_channel, cfg->theta_pulse_ledc_channel);
  ESP_LOGI(TAG, "========================================");
}

#if MCP_DEBUG_CMDS
void runMcpPinModeCommand(const char *cmd) {
  int pin = -1;
  char mode[16] = {0};
  if (sscanf(cmd, "mcp_pin_mode %d %15s", &pin, mode) < 2) {
    ESP_LOGE(TAG, "Usage: mcp_pin_mode <0..15> <inpu|in|out0|out1>");
    return;
  }
  if (pin < 0 || pin > 15) {
    ESP_LOGE(TAG, "Pin must be in range 0..15 for MCP23S17");
    return;
  }

  for (int i = 0; mode[i]; ++i) {
    mode[i] = static_cast<char>(tolower(mode[i]));
  }

  const uint8_t pin_u8 = static_cast<uint8_t>(pin);
  auto logImmediateMcpState = [pin]() {
    mcp23s17_handle_t h = gpio_expander_get_mcp_handle();
    if (h == nullptr) {
      ESP_LOGE(TAG, "MCP handle not available for immediate readback");
      return;
    }

    const bool isPortA = (pin < 8);
    const int bit = pin & 0x07;
    const uint8_t reg_iodir = isPortA ? 0x00 : 0x01;
    const uint8_t reg_gppu = isPortA ? 0x0C : 0x0D;
    const uint8_t reg_gpio = isPortA ? 0x12 : 0x13;
    const uint8_t reg_olat = isPortA ? 0x14 : 0x15;
    uint8_t iodir = 0, gppu = 0, olat = 0, gpio = 0;

    ESP_ERROR_CHECK(mcp23s17_debug_read_register(h, reg_iodir, &iodir));
    ESP_ERROR_CHECK(mcp23s17_debug_read_register(h, reg_gppu, &gppu));
    ESP_ERROR_CHECK(mcp23s17_debug_read_register(h, reg_olat, &olat));
    ESP_ERROR_CHECK(mcp23s17_debug_read_register(h, reg_gpio, &gpio));

    ESP_LOGI(TAG,
             "MCP immediate pin %d: dir=%d(1=input) pullup=%d olat=%d gpio=%d [IODIR=0x%02X GPPU=0x%02X OLAT=0x%02X GPIO=0x%02X]",
             pin, (iodir >> bit) & 0x1, (gppu >> bit) & 0x1, (olat >> bit) & 0x1,
             (gpio >> bit) & 0x1, iodir, gppu, olat, gpio);
  };

  if (strcmp(mode, "inpu") == 0 || strcmp(mode, "in") == 0) {
    ESP_ERROR_CHECK(gpio_expander_set_direction(pin_u8, false));
    ESP_ERROR_CHECK(gpio_expander_set_pullup(pin_u8, true));
    vTaskDelay(pdMS_TO_TICKS(2));
    const int level = gpio_expander_read(pin_u8);
    ESP_LOGI(TAG, "MCP pin %d -> INPUT_PULLUP, level=%d", pin, level);
    logImmediateMcpState();
    return;
  }
  if (strcmp(mode, "out0") == 0) {
    ESP_ERROR_CHECK(gpio_expander_set_direction(pin_u8, true));
    ESP_ERROR_CHECK(gpio_expander_write(pin_u8, 0));
    vTaskDelay(pdMS_TO_TICKS(2));
    const int level = gpio_expander_read(pin_u8);
    ESP_LOGI(TAG, "MCP pin %d -> OUTPUT LOW, level=%d", pin, level);
    logImmediateMcpState();
    return;
  }
  if (strcmp(mode, "out1") == 0) {
    ESP_ERROR_CHECK(gpio_expander_set_direction(pin_u8, true));
    ESP_ERROR_CHECK(gpio_expander_write(pin_u8, 1));
    vTaskDelay(pdMS_TO_TICKS(2));
    const int level = gpio_expander_read(pin_u8);
    ESP_LOGI(TAG, "MCP pin %d -> OUTPUT HIGH, level=%d", pin, level);
    logImmediateMcpState();
    return;
  }

  ESP_LOGE(TAG, "Invalid mode '%s'. Use inpu|in|out0|out1", mode);
}

void runMcpDumpCommand(const char *cmd) {
  char portChar = 0;
  if (sscanf(cmd, "mcp_dump %c", &portChar) < 1) {
    ESP_LOGE(TAG, "Usage: mcp_dump <a|b>");
    return;
  }

  const char port = static_cast<char>(tolower(static_cast<unsigned char>(portChar)));
  if (port != 'a' && port != 'b') {
    ESP_LOGE(TAG, "Port must be 'a' or 'b'");
    return;
  }

  mcp23s17_handle_t h = gpio_expander_get_mcp_handle();
  if (h == nullptr) {
    ESP_LOGE(TAG, "MCP handle not available");
    return;
  }

  uint8_t iocon = 0, iodir = 0, gppu = 0, olat = 0, gpio = 0;
  const uint8_t reg_iodir = (port == 'a') ? 0x00 : 0x01;
  const uint8_t reg_gppu = (port == 'a') ? 0x0C : 0x0D;
  const uint8_t reg_gpio = (port == 'a') ? 0x12 : 0x13;
  const uint8_t reg_olat = (port == 'a') ? 0x14 : 0x15;

  ESP_ERROR_CHECK(mcp23s17_debug_read_register(h, 0x0A, &iocon));
  ESP_ERROR_CHECK(mcp23s17_debug_read_register(h, reg_iodir, &iodir));
  ESP_ERROR_CHECK(mcp23s17_debug_read_register(h, reg_gppu, &gppu));
  ESP_ERROR_CHECK(mcp23s17_debug_read_register(h, reg_olat, &olat));
  ESP_ERROR_CHECK(mcp23s17_debug_read_register(h, reg_gpio, &gpio));

  ESP_LOGI(TAG, "MCP dump port %c: IOCON=0x%02X IODIR%c=0x%02X GPPU%c=0x%02X OLAT%c=0x%02X GPIO%c=0x%02X",
           (char)toupper((unsigned char)port), iocon,
           (char)toupper((unsigned char)port), iodir,
           (char)toupper((unsigned char)port), gppu,
           (char)toupper((unsigned char)port), olat,
           (char)toupper((unsigned char)port), gpio);

  if (port == 'b') {
    const int pb4_dir = (iodir >> 4) & 0x1;
    const int pb4_pull = (gppu >> 4) & 0x1;
    const int pb4_olat = (olat >> 4) & 0x1;
    const int pb4_gpio = (gpio >> 4) & 0x1;
    ESP_LOGI(TAG, "PB4 bits: dir=%d(1=input) pullup=%d olat=%d gpio=%d",
             pb4_dir, pb4_pull, pb4_olat, pb4_gpio);
  }
}

void runMcpRegCommand(const char *cmd) {
  char action[8] = {0};
  char regStr[16] = {0};
  char valueStr[16] = {0};
  const int parsed = sscanf(cmd, "mcp_reg %7s %15s %15s", action, regStr, valueStr);
  if (parsed < 2) {
    ESP_LOGE(TAG, "Usage: mcp_reg <r|w> <reg> [value]");
    return;
  }

  for (int i = 0; action[i]; ++i) {
    action[i] = static_cast<char>(tolower(static_cast<unsigned char>(action[i])));
  }

  mcp23s17_handle_t h = gpio_expander_get_mcp_handle();
  if (h == nullptr) {
    ESP_LOGE(TAG, "MCP handle not available");
    return;
  }

  char *endPtr = nullptr;
  const unsigned long regUl = strtoul(regStr, &endPtr, 0);
  if (endPtr == regStr || *endPtr != '\0' || regUl > 0x1F) {
    ESP_LOGE(TAG, "Invalid reg '%s' (expected 0x00..0x1F)", regStr);
    return;
  }
  const uint8_t reg = static_cast<uint8_t>(regUl);

  if (strcmp(action, "r") == 0) {
    uint8_t value = 0;
    ESP_ERROR_CHECK(mcp23s17_debug_read_register(h, reg, &value));
    ESP_LOGI(TAG, "MCP reg[0x%02X] = 0x%02X", reg, value);
    return;
  }

  if (strcmp(action, "w") == 0) {
    if (parsed < 3) {
      ESP_LOGE(TAG, "Usage: mcp_reg w <reg> <value>");
      return;
    }
    endPtr = nullptr;
    const unsigned long valueUl = strtoul(valueStr, &endPtr, 0);
    if (endPtr == valueStr || *endPtr != '\0' || valueUl > 0xFF) {
      ESP_LOGE(TAG, "Invalid value '%s' (expected 0x00..0xFF)", valueStr);
      return;
    }
    const uint8_t value = static_cast<uint8_t>(valueUl);
    ESP_ERROR_CHECK(mcp23s17_debug_write_register(h, reg, value));

    uint8_t verify = 0;
    ESP_ERROR_CHECK(mcp23s17_debug_read_register(h, reg, &verify));
    ESP_LOGI(TAG, "MCP reg[0x%02X] <= 0x%02X, readback=0x%02X", reg, value, verify);
    return;
  }

  ESP_LOGE(TAG, "Invalid action '%s'. Use r or w", action);
}

void runFieldDoutCommand(const char *cmd) {
  int ch = -1;
  int level = -1;
  if (sscanf(cmd, "field_dout %d %d", &ch, &level) < 2) {
    ESP_LOGE(TAG, "Usage: field_dout <0..3> <0|1>");
    return;
  }
  if (ch < 0 || ch >= FIELD_24V_DOUT_COUNT || (level != 0 && level != 1)) {
    ESP_LOGE(TAG, "Usage: field_dout <0..3> <0|1>");
    return;
  }
  if (field_dout_set(static_cast<unsigned>(ch), level != 0) != ESP_OK) {
    ESP_LOGE(TAG, "field_dout failed (MCP not ready?)");
    return;
  }
  ESP_LOGI(TAG, "FIELD_DOUT%d = %d (MCP PA%d)", ch, level, MCP_FIELD_DOUT0 + ch);
}

void runFieldDinCommand(const char *cmd) {
  (void)cmd;
  if (gpio_expander_get_mcp_handle() == nullptr) {
    ESP_LOGE(TAG, "MCP not initialized");
    return;
  }
  for (unsigned i = 0; i < FIELD_24V_DIN_COUNT; ++i) {
    ESP_LOGI(TAG, "FIELD_DIN%d = %d (MCP PA%d)", i, field_din_get(i) ? 1 : 0,
             MCP_FIELD_DIN0 + static_cast<int>(i));
  }
  ESP_LOGI(TAG, "ENC A/B/PUSH/KO = %d %d %d %d  TFT_CS(PB2)=%d  W5500_RST(PB7)=%d",
           gpio_expander_read(MCP_UI_ENC_A), gpio_expander_read(MCP_UI_ENC_B),
           gpio_expander_read(MCP_UI_ENC_PUSH), gpio_expander_read(MCP_UI_ENC_KO),
           gpio_expander_read(MCP_TFT_CS), gpio_expander_read(MCP_W5500_RST));
}
#endif  // MCP_DEBUG_CMDS

void runGpioDriveCommand(const char *cmd) {
  int gpio = -1;
  int value = -1;
  if (sscanf(cmd, "gpio_drive %d %d", &gpio, &value) < 2) {
    ESP_LOGE(TAG, "Usage: gpio_drive <gpio_num> <0|1>");
    return;
  }
  if (gpio < 0 || gpio >= 40 || (value != 0 && value != 1)) {
    ESP_LOGE(TAG, "Invalid args. gpio=0..39, value=0|1");
    return;
  }

  gpio_config_t io_conf = {};
  io_conf.pin_bit_mask = (1ULL << gpio);
  io_conf.mode = GPIO_MODE_INPUT_OUTPUT;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.intr_type = GPIO_INTR_DISABLE;
  ESP_ERROR_CHECK(gpio_config(&io_conf));
  ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)gpio, value));
  vTaskDelay(pdMS_TO_TICKS(2));
  int level = gpio_get_level((gpio_num_t)gpio);
  ESP_LOGI(TAG, "GPIO %d driven to %d, readback=%d", gpio, value, level);
}

// ---- Per-axis homing / calibration dispatchers ---------------------------
// Tokenize `home` / `calibrate` args (default X). Soft-home / soft-calibrate
// only when neither GPIO limits nor EIP drive-managed switches are available.

enum class AxisToken { X, Z, THETA };

bool parseAxisToken(const char *tok, AxisToken &out) {
  if (tok == nullptr) return false;
  if (strcmp(tok, "x") == 0) {
    out = AxisToken::X;
    return true;
  }
  if (strcmp(tok, "z") == 0) {
    out = AxisToken::Z;
    return true;
  }
  if (strcmp(tok, "t") == 0 || strcmp(tok, "theta") == 0) {
    out = AxisToken::THETA;
    return true;
  }
  return false;
}

// EIP soft-home / soft-calibrate when limit_switches_active=false.

void runSoftHomeX(const GantryTestConsoleConfig *cfg) {
  cfg->gantry->softHomeJointDatum();
  g_homeCompletedThisSession = true;
  g_homeZCompletedThisSession = true;
  ESP_LOGI(TAG,
           "OK soft-home (X+Z). Joint datum = current drive positions "
           "(X %.3f mm / %d PUU joint, Z %.3f mm). "
           "`move` targets are relative to this datum - not absolute drive PUU.",
           cfg->gantry->getXEncoderMm(), cfg->gantry->getXEncoderRaw(),
           cfg->gantry->getCurrentZ());
}

void runSoftCalibrateX(const GantryTestConsoleConfig *cfg) {
  const float xMax = AXIS_X_HARD_LIMIT_MAX_MM;
  cfg->gantry->setJointLimits(AXIS_X_HARD_LIMIT_MIN_MM, xMax,
                              AXIS_Z_HARD_LIMIT_MIN_MM, AXIS_Z_HARD_LIMIT_MAX_MM,
                              AXIS_THETA_HARD_LIMIT_MIN_DEG,
                              AXIS_THETA_HARD_LIMIT_MAX_DEG);
  g_calibratedThisSession = true;
  ESP_LOGI(TAG,
           "OK X soft-calibrate (no limit switches). Joint envelope X=%.1f..%.1f mm "
           "from SCHUNK datasheet. Run puucal after measured moves.",
           AXIS_X_HARD_LIMIT_MIN_MM, xMax);
}

void runHomeXSequence(const GantryTestConsoleConfig *cfg) {
  if (!cfg->gantry->isEnabled()) {
    ESP_LOGE(TAG, "ERROR: Motors not enabled");
    return;
  }

  if (cfg->gantry->driveManagedLimitsEnabled() &&
      !cfg->gantry->zInTraverseBand()) {
    ESP_LOGE(TAG,
             "ERROR: home x blocked — Z above SAFE_Z band ceiling (%.1f mm). "
             "Lower Z into the band from Z-/A015 first, or run 'calibrate all'.",
             (double)cfg->gantry->traverseClearanceZMm());
    return;
  }

  if (!physicalLimitsAvailable(cfg)) {
    ESP_LOGI(TAG, "Starting soft-home (X) - limit switches not wired (EIP mode)");
    runSoftHomeX(cfg);
    return;
  }

  const bool driveManaged = cfg->gantry->driveManagedLimitsEnabled();
  ESP_LOGI(TAG, "Starting homing sequence (X)%s...",
           driveManaged
               ? " (EIP: seek -X A014/PL -> creep clear -> joint zero)"
               : "");

  bool minWasActive = false;
  if (limitsWired(cfg)) {
    minWasActive = (gpio_expander_read(cfg->limit_min_pin) == 0);
  }
  cfg->gantry->homeX();
  vTaskDelay(pdMS_TO_TICKS(20));

  if (!cfg->gantry->isBusy()) {
    bool minIsActive = false;
    bool maxIsActive = false;
    if (limitsWired(cfg)) {
      minIsActive = (gpio_expander_read(cfg->limit_min_pin) == 0);
      maxIsActive = (gpio_expander_read(cfg->limit_max_pin) == 0);
    }
    bool alarmActive = cfg->gantry->isAlarmActive();
    if (minWasActive || minIsActive ||
        (driveManaged && !alarmActive)) {
      // Drive-managed: homeX may finish immediately if already on A014, or
      // the Absolute sweep may not yet show busy within 20 ms - accept and
      // let the operator watch status / calibrate.
      g_homeCompletedThisSession = true;
      if (driveManaged && !minWasActive && !minIsActive) {
        ESP_LOGI(TAG,
                 "OK X drive-managed home accepted (sweep or already at A014); "
                 "wait for busy=0 / status");
      } else {
        ESP_LOGI(TAG, "OK X homing skipped: already at MIN/home switch");
      }
      ESP_LOGI(TAG, "Home gate state: alarm=%d min=%d max=%d drive_managed=%d",
               alarmActive ? 1 : 0, minIsActive ? 1 : 0, maxIsActive ? 1 : 0,
               driveManaged ? 1 : 0);
    } else {
      ESP_LOGE(TAG, "ERROR: X homing did not start");
      ESP_LOGI(TAG, "Home gate state: alarm=%d min=%d max=%d drive_managed=%d",
               alarmActive ? 1 : 0, minIsActive ? 1 : 0, maxIsActive ? 1 : 0,
               driveManaged ? 1 : 0);
      ESP_LOGI(TAG, "Check motor enable, alarm status, and limit switch wiring");
    }
    return;
  }
  g_homeCompletedThisSession = true;
  ESP_LOGI(TAG, "OK X homing started (use 'stop' to abort, 'status' to monitor)");
}

void runHomeZSequence(const GantryTestConsoleConfig *cfg) {
  if (!cfg->gantry->isEnabled()) {
    ESP_LOGE(TAG, "ERROR: Motors not enabled");
    return;
  }

  if (!cfg->gantry->driveManagedLimitsEnabled()) {
    ESP_LOGE(TAG,
             "ERROR: Z home requires EIP drive-managed limits "
             "(GPIO Z switches not integrated)");
    return;
  }

  if (cfg->gantry->isBusy()) {
    ESP_LOGE(TAG, "ERROR: Gantry busy — finish or 'stop' before 'home z'");
    return;
  }

  ESP_LOGI(TAG,
           "Starting homing sequence (Z) (EIP: seek -Z A015/NL -> creep clear -> "
           "joint zero)...");

  cfg->gantry->homeZ();
  vTaskDelay(pdMS_TO_TICKS(20));

  if (!cfg->gantry->isBusy()) {
    const bool alarmActive = cfg->gantry->isAlarmActive();
    if (!alarmActive) {
      g_homeZCompletedThisSession = true;
      ESP_LOGI(TAG,
               "OK Z drive-managed home accepted (sweep or already at A015); "
               "wait for busy=0 / status");
    } else {
      ESP_LOGE(TAG, "ERROR: Z homing did not start (alarm active)");
    }
    return;
  }
  g_homeZCompletedThisSession = true;
  ESP_LOGI(TAG, "OK Z homing started (use 'stop' to abort, 'status' to monitor)");
}

void waitGantryIdle(const GantryTestConsoleConfig *cfg, uint32_t timeout_ms) {
  const TickType_t start = xTaskGetTickCount();
  while (cfg->gantry->isBusy()) {
    if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(timeout_ms)) {
      ESP_LOGW(TAG, "wait for idle timed out after %lu ms",
               (unsigned long)timeout_ms);
      return;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void runEipBringUpSequence(const GantryTestConsoleConfig *cfg) {
  if (!cfg->gantry->isEnabled()) {
    ESP_LOGE(TAG, "ERROR: Motors not enabled");
    return;
  }
  if (!cfg->gantry->driveManagedLimitsEnabled()) {
    ESP_LOGE(TAG, "ERROR: Bring-up requires EIP drive-managed limits");
    return;
  }
  if (cfg->gantry->isBusy() || g_calibrationInProgress) {
    ESP_LOGE(TAG, "ERROR: Gantry busy — finish or 'stop' before bring-up");
    return;
  }
  g_calibrationInProgress = true;
  g_calibratedThisSession = false;
  g_calibratedZThisSession = false;
  g_homeCompletedThisSession = false;
  g_homeZCompletedThisSession = false;

  if (!cfg->gantry->startEipBringUp()) {
    g_calibrationInProgress = false;
    ESP_LOGE(TAG, "ERROR: Bring-up did not start");
    return;
  }

  ESP_LOGI(TAG,
           "EIP bring-up started: Z- (A015) -> X home/cal -> X=%.1f -> Z+ (A014) "
           "-> SAFE_Z ceiling (use 'stop' to abort, 'status' to monitor)",
           (double)GANTRY_CAL_X_PARK_MM);

  const TickType_t start = xTaskGetTickCount();
  while (cfg->gantry->eipBringUpInProgress() || cfg->gantry->isBusy()) {
    if ((xTaskGetTickCount() - start) >
        pdMS_TO_TICKS(Gantry::Constants::TRAVEL_MEASUREMENT_TIMEOUT_MS * 4)) {
      ESP_LOGE(TAG, "ERROR: Bring-up timed out");
      cfg->gantry->requestAbort();
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
  g_calibrationInProgress = false;

  const int xLen = static_cast<int>(cfg->gantry->xAxisLengthMm());
  const int zLen = static_cast<int>(cfg->gantry->zAxisLengthMm());
  if (xLen > 0 && zLen > 0) {
    g_homeCompletedThisSession = true;
    g_homeZCompletedThisSession = true;
    g_calibratedThisSession = true;
    g_calibratedZThisSession = true;
    cfg->gantry->setJointLimits(AXIS_X_HARD_LIMIT_MIN_MM, (float)xLen,
                                AXIS_Z_HARD_LIMIT_MIN_MM, (float)zLen,
                                AXIS_THETA_HARD_LIMIT_MIN_DEG,
                                AXIS_THETA_HARD_LIMIT_MAX_DEG);
    ESP_LOGI(TAG,
             "OK Bring-up complete: X stroke=%d mm, Z stroke=%d mm, "
             "SAFE_Z ceiling=%.1f mm (z_min + %.1f)",
             xLen, zLen, (double)cfg->gantry->traverseClearanceZMm(),
             (double)GANTRY_SAFE_Z_HEIGHT_MM);
  } else {
    ESP_LOGE(TAG, "ERROR: Bring-up finished without valid X/Z stroke");
  }
}

void runHomeForAxis(const GantryTestConsoleConfig *cfg, AxisToken axis) {
  switch (axis) {
    case AxisToken::X:
      runHomeXSequence(cfg);
      break;
    case AxisToken::Z:
      runHomeZSequence(cfg);
      break;
    case AxisToken::THETA:
      cfg->gantry->homeTheta();
      ESP_LOGI(TAG, "OK Theta home accepted (stub - not yet wired)");
      break;
  }
}

void runCalibrateForAxis(const GantryTestConsoleConfig *cfg, AxisToken axis) {
  switch (axis) {
    case AxisToken::X: {
      if (!g_homeCompletedThisSession) {
        ESP_LOGE(TAG, "ERROR: Run 'home x' first after startup");
        return;
      }
      if (cfg->gantry->driveManagedLimitsEnabled() &&
          !cfg->gantry->zInTraverseBand()) {
        ESP_LOGE(TAG,
                 "ERROR: calibrate x blocked — Z above SAFE_Z band ceiling "
                 "(%.1f mm). Lower Z into the band from Z-/A015 first, or run "
                 "'calibrate all'.",
                 (double)cfg->gantry->traverseClearanceZMm());
        return;
      }
      if (!physicalLimitsAvailable(cfg)) {
        runSoftCalibrateX(cfg);
        break;
      }
      if (g_calibrationInProgress) {
        ESP_LOGI(TAG, "Calibration is already in progress");
        return;
      }
      g_calibrationInProgress = true;
      g_calibratedThisSession = false;
      BaseType_t taskOk = xTaskCreatePinnedToCore(
          calibrationTask, "CalibrateTask", 4096, (void *)cfg, 2, nullptr,
          CONSOLE_TASK_CORE);
      if (taskOk != pdPASS) {
        g_calibrationInProgress = false;
        ESP_LOGE(TAG, "ERROR: Failed to start X calibration task");
      } else {
        ESP_LOGI(TAG, "X calibration started%s (use 'stop' to abort)",
                 cfg->gantry->driveManagedLimitsEnabled()
                     ? " (EIP: seek +X A015/NL -> creep clear -> joint max)"
                     : "");
      }
      break;
    }
    case AxisToken::Z: {
      if (!g_homeZCompletedThisSession) {
        ESP_LOGE(TAG, "ERROR: Run 'home z' first after startup");
        return;
      }
      if (!cfg->gantry->driveManagedLimitsEnabled()) {
        ESP_LOGE(TAG,
                 "ERROR: Z calibrate requires EIP drive-managed limits");
        return;
      }
      if (g_calibrationInProgress) {
        ESP_LOGI(TAG, "Calibration is already in progress");
        return;
      }
      if (cfg->gantry->isBusy()) {
        ESP_LOGE(TAG, "ERROR: Gantry busy — finish or 'stop' before 'calibrate z'");
        return;
      }
      g_calibrationInProgress = true;
      g_calibratedZThisSession = false;
      BaseType_t taskOk = xTaskCreatePinnedToCore(
          zCalibrationTask, "ZCalibrateTask", 4096, (void *)cfg, 2, nullptr,
          CONSOLE_TASK_CORE);
      if (taskOk != pdPASS) {
        g_calibrationInProgress = false;
        ESP_LOGE(TAG, "ERROR: Failed to start Z calibration task");
      } else {
        ESP_LOGI(TAG,
                 "Z calibration started (EIP: seek +Z A014/PL -> creep clear -> "
                 "joint max) (use 'stop' to abort)");
      }
      break;
    }
    case AxisToken::THETA:
      cfg->gantry->calibrateTheta();
      ESP_LOGI(TAG, "OK Theta calibrate accepted (stub - not yet wired)");
      break;
  }
}

// Parses tokens after the leading command word and dispatches per axis.
// `cmd` is the lowercased full command line (e.g. "home x z" or "calibrate").
// `leading` is the word to skip ("home" or "calibrate").
// `runFn` is called for each parsed axis in the order they appear.
void dispatchPerAxisCommand(const GantryTestConsoleConfig *cfg, const char *cmd,
                            const char *leading,
                            void (*runFn)(const GantryTestConsoleConfig *,
                                          AxisToken)) {
  char buf[256];
  strncpy(buf, cmd, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';

  char *saveptr = nullptr;
  char *first = strtok_r(buf, " \t", &saveptr);
  if (first == nullptr) {
    return;
  }
  (void)first;  // skip the leading command word ("home" / "calibrate")

  bool sawAnyToken = false;
  bool ranX = false;
  bool ranZ = false;
  bool ranT = false;

  for (char *tok = strtok_r(nullptr, " \t", &saveptr); tok != nullptr;
       tok = strtok_r(nullptr, " \t", &saveptr)) {
    sawAnyToken = true;
    if (strcmp(tok, "all") == 0) {
      if (strcmp(leading, "calibrate") == 0 &&
          cfg->gantry->driveManagedLimitsEnabled()) {
        // Preferred full sequence: Z- -> X home/cal -> X park -> Z+ -> SAFE_Z.
        runEipBringUpSequence(cfg);
        ranX = true;
        ranZ = true;
        ranT = true;
        continue;
      }
      // home all / non-EIP calibrate all: Z before X so SAFE_Z interlock can pass.
      if (!ranZ) {
        runFn(cfg, AxisToken::Z);
        ranZ = true;
        waitGantryIdle(cfg, Gantry::Constants::TRAVEL_MEASUREMENT_TIMEOUT_MS);
        while (g_calibrationInProgress) {
          vTaskDelay(pdMS_TO_TICKS(50));
        }
      }
      if (!ranX) {
        if (strcmp(leading, "home") == 0 &&
            cfg->gantry->driveManagedLimitsEnabled() &&
            !cfg->gantry->zInTraverseBand()) {
          ESP_LOGE(TAG,
                   "ERROR: home all — Z is above SAFE_Z band ceiling (%.1f mm); "
                   "X home skipped. Lower Z into the band from Z-/A015, or run "
                   "'calibrate all'.",
                   (double)cfg->gantry->traverseClearanceZMm());
        } else {
          runFn(cfg, AxisToken::X);
        }
        ranX = true;
        waitGantryIdle(cfg, Gantry::Constants::TRAVEL_MEASUREMENT_TIMEOUT_MS);
        while (g_calibrationInProgress) {
          vTaskDelay(pdMS_TO_TICKS(50));
        }
      }
      if (!ranT) {
        runFn(cfg, AxisToken::THETA);
        ranT = true;
      }
      continue;
    }
    AxisToken axis;
    if (!parseAxisToken(tok, axis)) {
      ESP_LOGE(TAG, "ERROR: Unknown %s axis '%s'; use x|z|t|all", leading, tok);
      continue;
    }
    if (axis == AxisToken::X && ranX) continue;
    if (axis == AxisToken::Z && ranZ) continue;
    if (axis == AxisToken::THETA && ranT) continue;
    runFn(cfg, axis);
    if (axis == AxisToken::X) ranX = true;
    if (axis == AxisToken::Z) ranZ = true;
    if (axis == AxisToken::THETA) ranT = true;
  }

  if (!sawAnyToken) {
    // No tokens supplied -> default to X (back-compat with bare `home` /
    // `calibrate`). When tokens were supplied but all were invalid, the
    // per-token error message has already been logged and we deliberately
    // do NOT fall through to running X by accident.
    runFn(cfg, AxisToken::X);
  }
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
    ESP_LOGE(TAG, "Gantry not initialized");
    return;
  }

  if (strcmp(cmdLower, "help") == 0 || strcmp(cmdLower, "?") == 0) {
    gantryTestPrintHelp();
#if MCP_DEBUG_CMDS
  } else if (strncmp(cmdLower, "mcp_pin_mode", 12) == 0) {
    runMcpPinModeCommand(cmd);
  } else if (strncmp(cmdLower, "mcp_dump", 8) == 0) {
    runMcpDumpCommand(cmd);
  } else if (strncmp(cmdLower, "mcp_reg", 7) == 0) {
    runMcpRegCommand(cmd);
  } else if (strncmp(cmdLower, "field_dout", 10) == 0) {
    runFieldDoutCommand(cmd);
  } else if (strncmp(cmdLower, "field_din", 9) == 0) {
    runFieldDinCommand(cmd);
#endif  // MCP_DEBUG_CMDS
  } else if (strncmp(cmdLower, "gpio_drive", 10) == 0) {
    runGpioDriveCommand(cmd);
  } else if (strcmp(cmdLower, "status") == 0) {
    printStatus(cfg->gantry);
  } else if (strcmp(cmdLower, "faults") == 0 || strcmp(cmdLower, "alarms") == 0) {
    char xSum[192] = {};
    char zSum[192] = {};
    ESP_LOGI(TAG, "=== Drive faults / warnings ===");
    if (cfg->gantry->getXDriveAlarmSummary(xSum, sizeof(xSum))) {
      ESP_LOGI(TAG, "X: %s", xSum[0] ? xSum : "clear");
    } else {
      ESP_LOGI(TAG, "X: (no EIP summary)");
    }
    if (cfg->gantry->getZDriveAlarmSummary(zSum, sizeof(zSum))) {
      ESP_LOGI(TAG, "Z: %s", zSum[0] ? zSum : "clear");
    } else {
      ESP_LOGI(TAG, "Z: (no EIP summary)");
    }
    cfg->gantry->logDriveAlarmSummaries();
  } else if (strcmp(cmdLower, "puuinfo") == 0) {
    printPuuInfo(cfg->gantry);
  } else if (strncmp(cmdLower, "puucal", 6) == 0) {
    runPuuCalCommand(cfg->gantry, cmd);
  } else if (strcmp(cmdLower, "limits") == 0) {
    printLimits(cfg);
  } else if (strcmp(cmdLower, "pins") == 0) {
    printActivePins(cfg);
  } else if (strcmp(cmdLower, "enable") == 0) {
    cfg->gantry->enable();
    if (cfg->gantry->isEnabled()) {
      ESP_LOGI(TAG, "OK Motors enabled (settling for Active - wait for 'Servo arm complete')");
    } else {
      ESP_LOGE(TAG, "ERROR: Motor enable failed (check alarm/driver state)");
    }
  } else if (strcmp(cmdLower, "disable") == 0) {
    cfg->gantry->disable();
    ESP_LOGI(TAG, "OK Motors disabled");
  } else if (strncmp(cmdLower, "home", 4) == 0 &&
             (cmdLower[4] == '\0' || cmdLower[4] == ' ' || cmdLower[4] == '\t')) {
    dispatchPerAxisCommand(cfg, cmdLower, "home", runHomeForAxis);
  } else if (strncmp(cmdLower, "calibrate", 9) == 0 &&
             (cmdLower[9] == '\0' || cmdLower[9] == ' ' || cmdLower[9] == '\t')) {
    dispatchPerAxisCommand(cfg, cmdLower, "calibrate", runCalibrateForAxis);
  } else if (strncmp(cmdLower, "speed", 5) == 0) {
    int speedMm = -1;
    int speedDeg = -1;
    int parsed = sscanf(cmd, "speed %d %d", &speedMm, &speedDeg);
    if (parsed < 1 || speedMm <= 0) {
      ESP_LOGE(TAG, "Usage: speed <mm_per_s> [deg_per_s]");
      return;
    }
    const uint32_t requestedSpeedMm = (uint32_t)convertSelectedToMm((float)speedMm);
    const uint32_t clampedSpeedMm =
        applyRangeLimitU32(requestedSpeedMm, kMinSpeedMmPerS, maxLinearSpeedMmPerS(),
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
    ESP_LOGI(TAG, "OK Path speed updated: %.3f %s/s (resultant), theta=%lu deg/s",
             convertMmToSelected((float)g_moveSpeedMmPerS), getLinearUnitLabel(),
             (unsigned long)g_moveSpeedDegPerS);
  } else if (strncmp(cmdLower, "accel", 5) == 0) {
    int accel = -1;
    int decel = -1;
    int parsed = sscanf(cmd, "accel %d %d", &accel, &decel);
    if (parsed < 1 || accel <= 0) {
      ESP_LOGE(TAG, "Usage: accel <mm_per_s2> [decel_mm_per_s2] (values must be > 0)");
      return;
    }
    if (parsed >= 2 && decel <= 0) {
      ESP_LOGE(TAG, "Decel must be > 0");
      return;
    }
    const uint32_t requestedAccel = (uint32_t)convertSelectedToMm((float)accel);
    const uint32_t requestedDecel =
        (parsed >= 2 && decel > 0) ? (uint32_t)convertSelectedToMm((float)decel)
                                   : (uint32_t)convertSelectedToMm((float)accel);
    const uint32_t clampedAccel =
        applyRangeLimitU32(requestedAccel, kMinAccelMmPerS2, maxLinearAccelMmPerS2(),
                           g_motionProfileRangeLimitEnabled);
    const uint32_t clampedDecel =
        applyRangeLimitU32(requestedDecel, kMinAccelMmPerS2, maxLinearAccelMmPerS2(),
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
    ESP_LOGI(TAG, "OK Path accel updated: accel=%.3f %s/s2, decel=%.3f %s/s2 (resultant)",
             convertMmToSelected((float)g_moveAccelMmPerS2), getLinearUnitLabel(),
             convertMmToSelected((float)g_moveDecelMmPerS2), getLinearUnitLabel());
  } else if (strncmp(cmdLower, "units", 5) == 0) {
    char unitStr[16] = {0};
    int parsed = sscanf(cmd, "units %15s", unitStr);
    if (parsed < 1) {
      ESP_LOGE(TAG, "Usage: units <mm|in>");
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
      ESP_LOGE(TAG, "Usage: units <mm|in>");
      return;
    }
    ESP_LOGI(TAG, "OK Linear units set to %s (internal storage remains mm)", getLinearUnitLabel());
  } else if (strncmp(cmdLower, "rangelimit", 10) == 0) {
    int enabled = -1;
    int parsed = sscanf(cmd, "rangelimit %d", &enabled);
    if (parsed < 1 || (enabled != 0 && enabled != 1)) {
      ESP_LOGE(TAG, "Usage: rangelimit <0|1>");
      return;
    }
    g_motionProfileRangeLimitEnabled = (enabled == 1);
    ESP_LOGI(TAG, "OK Range limits %s", g_motionProfileRangeLimitEnabled ? "ENABLED" : "DISABLED");
    ESP_LOGI(TAG, "Configured path ranges: speed=%lu..%lu mm/s, accel/decel=%lu..%lu mm/s2",
             (unsigned long)kMinSpeedMmPerS, (unsigned long)maxLinearSpeedMmPerS(),
             (unsigned long)kMinAccelMmPerS2, (unsigned long)maxLinearAccelMmPerS2());
  } else if (strncmp(cmdLower, "livepos", 7) == 0) {
    int hz = -1;
    int parsed = sscanf(cmd, "livepos %d", &hz);
    if (parsed < 1) {
      if (g_livePosFrequencyHz == 0) {
        ESP_LOGI(TAG, "LIVE POS periodic logging: OFF");
      } else {
        uint32_t periodMs = 1000u / g_livePosFrequencyHz;
        if (periodMs == 0) {
          periodMs = 1;
        }
        ESP_LOGI(TAG, "LIVE POS periodic logging: %lu Hz (~%lu ms period)",
                 (unsigned long)g_livePosFrequencyHz, (unsigned long)periodMs);
      }
      return;
    }
    if (hz < 0) {
      ESP_LOGE(TAG, "Usage: livepos <hz> (0=off)");
      return;
    }
    g_livePosFrequencyHz = (uint32_t)hz;
    g_lastLiveMotionLogMs = 0;
    if (g_livePosFrequencyHz == 0) {
      ESP_LOGI(TAG, "OK LIVE POS periodic logging disabled");
    } else {
      uint32_t periodMs = 1000u / g_livePosFrequencyHz;
      if (periodMs == 0) {
        periodMs = 1;
      }
      ESP_LOGI(TAG, "OK LIVE POS frequency set to %lu Hz (~%lu ms period)",
               (unsigned long)g_livePosFrequencyHz, (unsigned long)periodMs);
    }
  } else if (strncmp(cmdLower, "axislog", 7) == 0) {
    int hz = -1;
    int parsed = sscanf(cmd, "axislog %d", &hz);
    if (parsed < 1) {
      if (g_axisLogFrequencyHz == 0) {
        ESP_LOGI(TAG, "Axis MOVE periodic logging: OFF (START/END events always fire)");
      } else {
        uint32_t periodMs = 1000u / g_axisLogFrequencyHz;
        if (periodMs == 0) {
          periodMs = 1;
        }
        ESP_LOGI(TAG, "Axis MOVE periodic logging: %lu Hz (~%lu ms period)",
                 (unsigned long)g_axisLogFrequencyHz, (unsigned long)periodMs);
      }
      return;
    }
    if (hz < 0) {
      ESP_LOGE(TAG, "Usage: axislog <hz> (0=off; START/END events always fire)");
      return;
    }
    g_axisLogFrequencyHz = (uint32_t)hz;
    cfg->gantry->setAxisLogRateHz(g_axisLogFrequencyHz);
    if (g_axisLogFrequencyHz == 0) {
      ESP_LOGI(TAG, "OK Axis MOVE periodic logging disabled (START/END still on)");
    } else {
      uint32_t periodMs = 1000u / g_axisLogFrequencyHz;
      if (periodMs == 0) {
        periodMs = 1;
      }
      ESP_LOGI(TAG, "OK Axis MOVE frequency set to %lu Hz (~%lu ms period)",
               (unsigned long)g_axisLogFrequencyHz, (unsigned long)periodMs);
    }
  } else if (strncmp(cmdLower, "move", 4) == 0) {
    float x = 0.0f;
    float z = 0.0f;
    float theta = 0.0f;
    int parsed = sscanf(cmd, "move %f %f %f", &x, &z, &theta);
    if (parsed < 3) {
      ESP_LOGE(TAG, "Usage: move <x_%s> <z_%s> <theta_deg>",
               getLinearUnitLabel(), getLinearUnitLabel());
      return;
    }

    Gantry::JointConfig target;
    target.x = convertSelectedToMm(x);
    target.z = convertSelectedToMm(z);
    target.theta = theta;

    // Per-axis session gates: only require home/cal for axes that move.
    // Lets Z-only jog after home z / calibrate z before X is brought up
    // (SAFE_Z band is enforced in Gantry path planning, not here).
    constexpr float kHoldEpsMm = 0.5f;
    const Gantry::JointConfig cur = cfg->gantry->getCurrentJointConfig();
    const bool moveX = fabsf(target.x - cur.x) > kHoldEpsMm;
    const bool moveZ = fabsf(target.z - cur.z) > kHoldEpsMm;
    if (moveX && (!g_homeCompletedThisSession || !g_calibratedThisSession)) {
      ESP_LOGE(TAG,
               "ERROR: X move blocked. Run 'home x' then 'calibrate x' "
               "(or 'calibrate all'). Z-only: stay at current X.");
      return;
    }
    if (moveZ && (!g_homeZCompletedThisSession || !g_calibratedZThisSession)) {
      ESP_LOGE(TAG,
               "ERROR: Z move blocked. Run 'home z' then 'calibrate z' "
               "(or 'calibrate all').");
      return;
    }
    if (!moveX && !moveZ) {
      // Theta-only or no-op still needs at least one axis brought up once.
      if ((!g_homeCompletedThisSession || !g_calibratedThisSession) &&
          (!g_homeZCompletedThisSession || !g_calibratedZThisSession)) {
        ESP_LOGE(TAG,
                 "ERROR: Move blocked. Run home+calibrate for X and/or Z "
                 "after startup (or 'calibrate all').");
        return;
      }
    }

    ESP_LOGI(TAG, "Moving to: x=%.3f %s, z=%.3f %s, theta=%.1f deg",
             x, getLinearUnitLabel(), z, getLinearUnitLabel(), theta);
    Gantry::GantryError result = cfg->gantry->moveTo(
        target, g_moveSpeedMmPerS, g_moveSpeedDegPerS, g_moveAccelMmPerS2, g_moveDecelMmPerS2);
    if (result == Gantry::GantryError::OK) {
      vTaskDelay(pdMS_TO_TICKS(20));
      if (cfg->gantry->isBusy()) {
        ESP_LOGI(TAG, "OK Move started (use 'stop' to abort, 'status' to monitor)");
      } else {
        ESP_LOGE(TAG, "ERROR: Move command accepted but motion did not start");
        ESP_LOGI(TAG, "Check alarm/limits and commanded-vs-encoder X position in status logs");
      }
    } else {
      const char *errName = "UNKNOWN";
      switch (result) {
        case Gantry::GantryError::NOT_INITIALIZED: errName = "NOT_INITIALIZED"; break;
        case Gantry::GantryError::MOTOR_NOT_ENABLED: errName = "MOTOR_NOT_ENABLED"; break;
        case Gantry::GantryError::ALREADY_MOVING: errName = "ALREADY_MOVING"; break;
        case Gantry::GantryError::INVALID_POSITION: errName = "INVALID_POSITION"; break;
        case Gantry::GantryError::INVALID_PARAMETER: errName = "INVALID_PARAMETER"; break;
        case Gantry::GantryError::TIMEOUT: errName = "TIMEOUT (alarm?)"; break;
        default: break;
      }
      ESP_LOGE(TAG, "ERROR: Move failed: %d (%s)", (int)result, errName);
      if (result == Gantry::GantryError::ALREADY_MOVING) {
        ESP_LOGI(TAG, "Gantry reports busy - wait for motion to finish, or 'stop' then 'enable'");
      }
    }
  } else if (strncmp(cmdLower, "grip", 4) == 0) {
    int value = 0;
    int parsed = sscanf(cmd, "grip %d", &value);
    if (parsed < 1) {
      ESP_LOGE(TAG, "Usage: grip <0|1>");
      return;
    }
    cfg->gantry->grip(value != 0);
    ESP_LOGI(TAG, "OK Gripper %s", value ? "closed" : "opened");
  } else if (strcmp(cmdLower, "eiptiming") == 0) {
    eip::dumpClass1TimingStats();
  } else if (strcmp(cmdLower, "stop") == 0) {
    // Stop motion while servos stay enabled, then disable. Do not reverse
    // that order: disable-only left sticky CIP; re-enable ClearCip StopMotion
    // then raised A603.
    cfg->gantry->requestAbort();
    cfg->gantry->disable();
    ESP_LOGI(TAG, "OK Stop requested (motion stopped, motors disabled)");
    if (g_calibrationInProgress) {
      ESP_LOGI(TAG, "Calibration abort requested");
    }
  } else if (strcmp(cmdLower, "alarmreset") == 0 || strcmp(cmdLower, "arst") == 0) {
    if (cfg->gantry->clearAlarm()) {
      ESP_LOGI(TAG, "OK Alarm reset pulse sent (clears Fault/Warning latch e.g. A603)");
      char xSum[192] = {};
      char zSum[192] = {};
      (void)cfg->gantry->getXDriveAlarmSummary(xSum, sizeof(xSum));
      (void)cfg->gantry->getZDriveAlarmSummary(zSum, sizeof(zSum));
      ESP_LOGI(TAG, "After reset - X: %s | Z: %s",
               xSum[0] ? xSum : "?", zSum[0] ? zSum : "?");
    } else {
      ESP_LOGE(TAG, "ERROR: Alarm reset failed (ARST pin may be disabled)");
    }
#if CONFIG_GANTRY_SELFTEST
  } else if (strcmp(cmdLower, "selftest") == 0) {
    BasicTestSummary result = runBasicTests();
    ESP_LOGI(TAG, "Selftest complete: passed=%d failed=%d", result.passed, result.failed);
#endif
  } else {
    ESP_LOGE(TAG, "ERROR: Unknown command: %s", cmd);
    ESP_LOGI(TAG, "Type 'help' for available commands");
  }
}
}  // namespace

void gantryConsoleAttachLogSink(GantryConsoleReplyFn fn, void *ctx) {
  ensureConsoleMutex();
  if (g_consoleCmdMutex != nullptr) {
    xSemaphoreTake(g_consoleCmdMutex, portMAX_DELAY);
  }
  g_sessionFn = fn;
  g_sessionCtx = ctx;
  if (fn != nullptr) {
    installTeeIfNeeded();
  } else {
    uninstallTeeIfIdle();
  }
  if (g_consoleCmdMutex != nullptr) {
    xSemaphoreGive(g_consoleCmdMutex);
  }
}

void gantryConsoleDetachLogSink(void) {
  gantryConsoleAttachLogSink(nullptr, nullptr);
}

void gantryConsoleProcessLine(const GantryTestConsoleConfig *cfg, const char *line,
                              GantryConsoleReplyFn reply_fn, void *reply_ctx) {
  ensureConsoleMutex();
  if (g_consoleCmdMutex != nullptr) {
    xSemaphoreTake(g_consoleCmdMutex, portMAX_DELAY);
  }

  const bool use_temp_tee = (reply_fn != nullptr && g_sessionFn == nullptr);
  g_replyFn = reply_fn;
  g_replyCtx = reply_ctx;
  if (use_temp_tee) {
    installTeeIfNeeded();
  }

  processCommand(cfg, line);

  if (use_temp_tee) {
    g_replyFn = nullptr;
    g_replyCtx = nullptr;
    uninstallTeeIfIdle();
  } else {
    g_replyFn = nullptr;
    g_replyCtx = nullptr;
  }

  if (g_consoleCmdMutex != nullptr) {
    xSemaphoreGive(g_consoleCmdMutex);
  }
}

void gantryTestPrintHelp() {
  ESP_LOGI(TAG, "========================================");
  ESP_LOGI(TAG, "Gantry Library Example - Commands:");
  ESP_LOGI(TAG, "========================================");
  ESP_LOGI(TAG, "  help                 - show this help");
  ESP_LOGI(TAG, "  status               - print gantry status");
  ESP_LOGI(TAG, "  faults | alarms      - decode X/Z Kinetix FaultCode/WarningCode (e.g. A603)");
  ESP_LOGI(TAG, "  puuinfo              - print X/Z PUU/mm scale and positions");
  ESP_LOGI(TAG, "  eiptiming            - dump Class 1 latency p50/p99 (exchange/ot/cycle/cmd2start)");
  ESP_LOGI(TAG, "  puucal <x|z> c m     - suggest new PUU/mm from commanded vs measured mm");
  ESP_LOGI(TAG, "  limits               - read limit switches");
  ESP_LOGI(TAG, "  pins                 - print active pin configuration");
#if MCP_DEBUG_CMDS
  ESP_LOGI(TAG, "  mcp_pin_mode p m     - force MCP pin mode (m=inpu|in|out0|out1)");
  ESP_LOGI(TAG, "  mcp_dump <a|b>       - dump MCP IOCON/dir/pullup/olat/gpio");
  ESP_LOGI(TAG, "  mcp_reg <r|w> r [v]  - raw MCP register read/write (hex/dec)");
  ESP_LOGI(TAG, "  field_dout <0..3> v  - set Field 24 V DOUT (0=gripper PA0)");
  ESP_LOGI(TAG, "  field_din            - read Field DIN + encoder + W5500 INT");
#endif  // MCP_DEBUG_CMDS
  ESP_LOGI(TAG, "  gpio_drive g v       - drive direct ESP32 GPIO g to v (0|1)");
  ESP_LOGI(TAG, "  enable               - enable motors");
  ESP_LOGI(TAG, "  disable              - disable motors");
  ESP_LOGI(TAG, "  home [x|z|t|all]     - home (EIP: seek A014/PL; all=Z then X; "
                "X needs SAFE_Z band)");
  ESP_LOGI(TAG, "  calibrate [x|z|t|all] - calibrate; 'all' = EIP bring-up "
                "(Z-/A015 -> X home/cal -> X=%.0f -> Z+/A014 -> SAFE_Z)",
           (double)GANTRY_CAL_X_PARK_MM);
  ESP_LOGI(TAG, "  units <mm|in>        - set linear input/output units");
  ESP_LOGI(TAG, "  speed <v> [deg/s]    - set 2-D path speed (resultant; v in selected linear units/s)");
  ESP_LOGI(TAG, "  accel <a> [d]        - set 2-D path accel/decel (resultant; >0, selected linear units/s2)");
  ESP_LOGI(TAG, "  rangelimit <0|1>     - enable/disable path speed+accel/decel range clamps");
  ESP_LOGI(TAG, "  livepos <hz>         - set LIVE POS periodic rate (0=off, default off)");
  ESP_LOGI(TAG, "  axislog <hz>         - set per-axis MOVE periodic rate (0=off; START/END always on)");
  ESP_LOGI(TAG, "  move <x> <z> <t>     - move to (x_linear, z_linear, theta_deg); +Z=up, z=joint datum");
  ESP_LOGI(TAG, "  grip <0|1>           - control gripper (0=open, 1=close)");
  ESP_LOGI(TAG, "  stop                 - stop all motion");
  ESP_LOGI(TAG, "  alarmreset | arst    - pulse alarm reset output (ARST)");
#if CONFIG_GANTRY_SELFTEST
  ESP_LOGI(TAG, "  selftest             - run basic math/config tests");
#endif
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
          gantryConsoleProcessLine(cfg, inputLine);
          inputIndex = 0;
        }
      } else if (c >= 32 && c <= 126 && inputIndex < sizeof(inputLine) - 1) {
        inputLine[inputIndex++] = static_cast<char>(c);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

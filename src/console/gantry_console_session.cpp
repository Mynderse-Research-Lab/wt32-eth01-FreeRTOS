/**
 * @file gantry_console_session.cpp
 * @brief Gantry test console: session state, motion profile, workspace latch and the idle monitor.
 *
 * One of the gantry_console_*.cpp files that make up the test console;
 * gantry_console_internal.h declares what they share.
 */

#include "gantry_console_internal.h"

namespace GantryConsoleImpl {

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
std::atomic<bool> g_homeCompletedThisSession{false};
std::atomic<bool> g_calibratedThisSession{false};
std::atomic<bool> g_homeZCompletedThisSession{false};
std::atomic<bool> g_calibratedZThisSession{false};
std::atomic<bool> g_homeThetaThisSession{false};
std::atomic<bool> g_calibratedThetaThisSession{false};
// 'home t' started its return to 0 deg; resolved by resolveThetaHome().
std::atomic<bool> g_thetaHomePending{false};
std::atomic<bool> g_calibrationInProgress{false};
std::atomic<bool> g_testCycleInProgress{false};
static bool g_wsCalEdgeInited = false;
static bool g_wsCalEdgeLast = false;
// Older images stored the workspace latch here and restored it at boot.
static constexpr char kWsCalNvsNs[] = "gantry_cfg";
static constexpr char kWsCalNvsKey[] = "ws_cal";
// Console move profile: the stored profile, until 'speed' or 'accel' changes it
// for this session.
uint32_t g_moveSpeedMmPerS = 0;
uint32_t g_moveSpeedDegPerS = 0;
uint32_t g_moveAccelMmPerS2 = 0;
uint32_t g_moveDecelMmPerS2 = 0;
uint32_t g_moveAccelDegPerS2 = 0;
uint32_t g_moveDecelDegPerS2 = 0;
bool g_motionProfileRangeLimitEnabled = true;
LinearUnitMode g_linearUnitMode = LinearUnitMode::MM;
// LIVE POS periodic logger frequency control.
// 0 Hz disables periodic output (default).
uint32_t g_livePosFrequencyHz = 0;
uint32_t g_lastLiveMotionLogMs = 0;
static bool g_liveMotionWasBusy = false;

const Config::AppConfigData &stored() {
  return Config::AppConfig::instance().data();
}

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

// Shared 2-D path envelope: console speed/accel/decel are the RESULTANT along
// the X-Z path. Protective clamp: never exceed Z ballscrew critical-RPM speed.
uint32_t maxLinearSpeedMmPerS() {
  // Effective path speed is operator-set and may exceed 1 m/s. The Z
  // ballscrew critical-RPM figure stays in the drivetrain header for
  // commissioning; it is not a hard ceiling on this setting.
  return static_cast<uint32_t>(stored().gantry_path_max_speed_mm_per_s);
}

uint32_t maxLinearAccelMmPerS2() {
  return static_cast<uint32_t>(stored().gantry_path_max_accel_mm_per_s2);
}

uint32_t maxThetaSpeedDegPerS() {
  if (stored().axis_theta_max_speed_deg_per_s <= 0.0f) {
    return 3600u;
  }
  return static_cast<uint32_t>(stored().axis_theta_max_speed_deg_per_s);
}

uint32_t maxThetaAccelDegPerS2() {
  if (stored().axis_theta_accel_deg_per_s2 <= 0.0f) {
    return 18000u;
  }
  return static_cast<uint32_t>(stored().axis_theta_accel_deg_per_s2);
}

uint32_t maxThetaDecelDegPerS2() {
  if (stored().axis_theta_decel_deg_per_s2 <= 0.0f) {
    return 18000u;
  }
  return static_cast<uint32_t>(stored().axis_theta_decel_deg_per_s2);
}

const char *getLinearUnitLabel() {
  return GantryConsole::linearUnitLabel(g_linearUnitMode);
}

float convertMmToSelected(float valueMm) {
  return GantryConsole::convertMmToSelected(valueMm, g_linearUnitMode);
}

float convertSelectedToMm(float valueSelected) {
  return GantryConsole::convertSelectedToMm(valueSelected, g_linearUnitMode);
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

static void clearSessionHomeCalFlags() {
  g_homeCompletedThisSession = false;
  g_calibratedThisSession = false;
  g_homeZCompletedThisSession = false;
  g_calibratedZThisSession = false;
  g_homeThetaThisSession = false;
  g_calibratedThetaThisSession = false;
  g_thetaHomePending = false;
}

// 'home t' returns before theta reaches 0 deg. Count it as homed only once
// theta is idle at 0; a faulted or timed-out return leaves it unhomed.
// Called at every theta gate and from the idle monitor, so it does not
// depend on the UART task.
void resolveThetaHome(const GantryTestConsoleConfig *cfg) {
    if (!g_thetaHomePending || cfg->gantry->isBusy()) {
        return;
    }
    if (!g_thetaHomePending.exchange(false)) {
        return;
    }
    if (cfg->gantry->thetaSettledAt(0.0f)) {
        g_homeThetaThisSession = true;
        ESP_LOGI(TAG, "OK Theta home complete at %.3f deg",
                 (double)cfg->gantry->getCurrentThetaDeg());
        return;
    }
    char diag[96] = {};
    (void)cfg->gantry->getThetaDriveAlarmSummary(diag, sizeof(diag));
    ESP_LOGE(TAG,
             "ERROR: Theta home did not reach 0 deg (at %.3f deg; %s). "
             "Run 'alarmreset', then 'home t' again",
             (double)cfg->gantry->getCurrentThetaDeg(), diag[0] != '\0' ? diag : "no diag");
}

static void noteWorkspaceCalEdge(Gantry::Gantry *gantry);

void maybeLatchWorkspaceCalibrated(Gantry::Gantry *gantry) {
  if (gantry == nullptr) {
    return;
  }
  if ((gantry->xAxisLengthMm() > 0 && gantry->zAxisLengthMm() > 0) ||
      (g_calibratedThisSession && g_calibratedZThisSession)) {
    gantry->setWorkspaceCalibrated(true);
    noteWorkspaceCalEdge(gantry);
  }
}

// When a drive loses position the latch drops; the session home/cal gates go with it.
static void noteWorkspaceCalEdge(Gantry::Gantry *gantry) {
    if (gantry == nullptr) {
        return;
    }
    const bool now = gantry->isWorkspaceCalibrated();
    if (!g_wsCalEdgeInited) {
        g_wsCalEdgeInited = true;
        g_wsCalEdgeLast = now;
        return;
    }
    if (now == g_wsCalEdgeLast) {
        return;
    }
    g_wsCalEdgeLast = now;
    if (!now) {
        clearSessionHomeCalFlags();
    }
}

// The X/Z joint zero lives in RAM: after an ESP reboot the axes read raw drive
// position (hundreds of mm off), so the workspace latch must not survive one.
// Boot starts uncalibrated; run 'calibrate all'. The stale key is erased so a
// rollback to an older image cannot restore it either.
void startWorkspaceUncalibrated(Gantry::Gantry *gantry) {
    if (gantry == nullptr) {
        return;
    }
    nvs_handle_t handle = 0;
    if (nvs_open(kWsCalNvsNs, NVS_READWRITE, &handle) == ESP_OK) {
        if (nvs_erase_key(handle, kWsCalNvsKey) == ESP_OK) {
            (void)nvs_commit(handle);
            ESP_LOGW(TAG, "Dropped stored workspace calibration; run 'calibrate all' after boot");
        }
        nvs_close(handle);
    }
    g_wsCalEdgeInited = true;
    g_wsCalEdgeLast = gantry->isWorkspaceCalibrated();
}

void printWorkspaceCalibrated(Gantry::Gantry *gantry) {
  if (gantry == nullptr) {
    ESP_LOGE(TAG, "Gantry not initialized");
    return;
  }
  ESP_LOGI(TAG, "Workspace calibrated: %s",
           gantry->isWorkspaceCalibrated() ? "Yes" : "No");
}

void monitorControlVariableFlips(const GantryTestConsoleConfig *cfg) {
  if (cfg == nullptr || cfg->gantry == nullptr) {
    return;
  }
  resolveThetaHome(cfg);

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
    noteWorkspaceCalEdge(cfg->gantry);
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
  noteWorkspaceCalEdge(cfg->gantry);
}

}  // namespace GantryConsoleImpl

/**
 * @file gantry_console_bringup.cpp
 * @brief Gantry test console: EtherNet/IP bring-up sequence and per-axis command dispatch.
 *
 * One of the gantry_console_*.cpp files that make up the test console;
 * gantry_console_internal.h declares what they share.
 */

#include "gantry_console_internal.h"

namespace GantryConsoleImpl {

// Backstop for the bring-up wait on the theta return to 0 deg.
static constexpr uint32_t kThetaHomeWaitMs = 120000;

bool runEipBringUpSequence(const GantryTestConsoleConfig *cfg) {
  if (!cfg->gantry->isEnabled()) {
    ESP_LOGE(TAG, "ERROR: Motors not enabled");
    return false;
  }
  if (!cfg->gantry->driveManagedLimitsEnabled()) {
    ESP_LOGE(TAG, "ERROR: Bring-up requires EIP drive-managed limits");
    return false;
  }
  if (cfg->gantry->isBusy() || g_calibrationInProgress) {
    ESP_LOGE(TAG, "ERROR: Gantry busy — finish or 'stop' before bring-up");
    return false;
  }
  g_calibrationInProgress = true;
  g_calibratedThisSession = false;
  g_calibratedZThisSession = false;
  g_calibratedThetaThisSession = false;
  g_homeCompletedThisSession = false;
  g_homeZCompletedThisSession = false;
  g_homeThetaThisSession = false;
  g_thetaHomePending = false;

  if (!cfg->gantry->startEipBringUp()) {
    g_calibrationInProgress = false;
    ESP_LOGE(TAG, "ERROR: Bring-up did not start");
    return false;
  }

  ESP_LOGI(TAG,
           "EIP bring-up started: Z- (A015 switch-clear = 0) -> X home/cal -> "
           "X=%.1f -> Z+ (A014) -> SAFE_Z ceiling (use 'stop' to abort, "
           "'status' to monitor)",
           (double)GANTRY_CAL_X_PARK_MM);

  const TickType_t start = xTaskGetTickCount();
  while (cfg->gantry->eipBringUpInProgress() || cfg->gantry->isBusy()) {
    if (cfg->gantry->isAbortRequested()) {
      ESP_LOGE(TAG, "ERROR: Bring-up aborted");
      break;
    }
    if ((xTaskGetTickCount() - start) >
        pdMS_TO_TICKS(Gantry::Constants::BRINGUP_SEQUENCE_TIMEOUT_MS)) {
      ESP_LOGE(TAG, "ERROR: Bring-up timed out");
      cfg->gantry->requestAbort();
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
  g_calibrationInProgress = false;

  if (cfg->gantry->isAbortRequested()) {
    return false;
  }

  const int xLen = static_cast<int>(cfg->gantry->xAxisLengthMm());
  const int zLen = static_cast<int>(cfg->gantry->zAxisLengthMm());
  if (xLen > 0 && zLen > 0) {
    g_homeCompletedThisSession = true;
    g_homeZCompletedThisSession = true;
    g_calibratedThisSession = true;
    g_calibratedZThisSession = true;
    cfg->gantry->setJointLimits(stored().axis_x_hard_limit_min_mm, (float)xLen,
                                stored().axis_z_hard_limit_min_mm, (float)zLen,
                                stored().axis_theta_hard_limit_min_deg,
                                stored().axis_theta_hard_limit_max_deg);
    if (cfg->gantry->hasThetaAxis()) {
        bool thetaOk = cfg->gantry->homeTheta();
        if (thetaOk) {
            // The axis times out its own move (thetaMoveTimeoutUs); this cap
            // is only a backstop.
            const TickType_t th_start = xTaskGetTickCount();
            while (cfg->gantry->isBusy()) {
                if ((xTaskGetTickCount() - th_start) > pdMS_TO_TICKS(kThetaHomeWaitMs)) {
                    ESP_LOGW(TAG, "Bring-up: timeout waiting for theta to settle at 0 deg");
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(50));
            }
            thetaOk = cfg->gantry->thetaSettledAt(0.0f) && cfg->gantry->calibrateTheta() > 0;
        }
        if (thetaOk) {
            g_homeThetaThisSession = true;
            g_calibratedThetaThisSession = true;
            ESP_LOGI(TAG,
                     "Bring-up: theta origin %s at 0 deg (drive_abs=%.3f, joint_lim=%.1f..%.1f)",
                     cfg->gantry->isThetaDriveOriginAligned() ? "ALIGNED" : "OFFSET",
                     (double)cfg->gantry->getThetaDriveAbsDeg(),
                     (double)cfg->gantry->getJointLimits().theta_min,
                     (double)cfg->gantry->getJointLimits().theta_max);
        } else {
            ESP_LOGW(TAG,
                     "Bring-up: X/Z OK; theta home/cal failed at %.3f deg "
                     "(alarm? CIP .23 / Class 1 / encoder) — X/Z gates stay set",
                     (double)cfg->gantry->getCurrentThetaDeg());
        }
    }
    ESP_LOGI(TAG,
             "OK Bring-up complete: X stroke=%d mm, Z stroke=%d mm, "
             "SAFE_Z ceiling=%.1f mm (z_min + %.1f)",
             xLen, zLen, (double)cfg->gantry->traverseClearanceZMm(),
             (double)stored().gantry_safe_z_height_mm);
    maybeLatchWorkspaceCalibrated(cfg->gantry);
    return true;
  }
  ESP_LOGE(TAG, "ERROR: Bring-up finished without valid X/Z stroke");
  return false;
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

}  // namespace GantryConsoleImpl

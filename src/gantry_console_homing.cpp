/**
 * @file gantry_console_homing.cpp
 * @brief Gantry test console: home and calibrate sequences per axis.
 *
 * One of the gantry_console_*.cpp files that make up the test console;
 * gantry_console_internal.h declares what they share.
 */

#include "gantry_console_internal.h"

namespace GantryConsoleImpl {

static void calibrationTask(void *param) {
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
            : stored().axis_z_hard_limit_max_mm;
    cfg->gantry->setJointLimits(stored().axis_x_hard_limit_min_mm, (float)len,
                                stored().axis_z_hard_limit_min_mm, zMax,
                                stored().axis_theta_hard_limit_min_deg,
                                stored().axis_theta_hard_limit_max_deg);
    g_calibratedThisSession = true;
    ESP_LOGI(TAG, "OK Calibrated length: %d mm", len);
    ESP_LOGI(TAG, "OK X joint max updated from calibration: %.1f mm", (float)len);
    maybeLatchWorkspaceCalibrated(cfg->gantry);
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

static void zCalibrationTask(void *param) {
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
    maybeLatchWorkspaceCalibrated(cfg->gantry);
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

// ---- Per-axis homing / calibration dispatchers ---------------------------
// Tokenize `home` / `calibrate` args (default X). Soft-home / soft-calibrate
// only when neither GPIO limits nor EIP drive-managed switches are available.

// EIP soft-home / soft-calibrate when limit_switches_active=false.

static void runSoftHomeX(const GantryTestConsoleConfig *cfg) {
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

static void runSoftCalibrateX(const GantryTestConsoleConfig *cfg) {
  const float xMax = stored().axis_x_hard_limit_max_mm;
  cfg->gantry->setJointLimits(stored().axis_x_hard_limit_min_mm, xMax,
                              stored().axis_z_hard_limit_min_mm,
                              stored().axis_z_hard_limit_max_mm,
                              stored().axis_theta_hard_limit_min_deg,
                              stored().axis_theta_hard_limit_max_deg);
  g_calibratedThisSession = true;
  ESP_LOGI(TAG,
           "OK X soft-calibrate (no limit switches). Joint envelope X=%.1f..%.1f mm "
           "from SCHUNK datasheet. Run puucal after measured moves.",
           stored().axis_x_hard_limit_min_mm, xMax);
  maybeLatchWorkspaceCalibrated(cfg->gantry);
}

static void runHomeXSequence(const GantryTestConsoleConfig *cfg) {
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

static void runHomeZSequence(const GantryTestConsoleConfig *cfg) {
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

void runHomeForAxis(const GantryTestConsoleConfig *cfg, AxisToken axis) {
  switch (axis) {
    case AxisToken::X:
      runHomeXSequence(cfg);
      break;
    case AxisToken::Z:
      runHomeZSequence(cfg);
      break;
    case AxisToken::THETA:
      if (!cfg->gantry->hasThetaAxis()) {
        ESP_LOGE(TAG, "ERROR: Theta axis not compiled in (CONFIG_EIP_AXIS_THETA)");
        return;
      }
      if (!cfg->gantry->isEnabled()) {
        ESP_LOGE(TAG, "ERROR: Motors not enabled");
        return;
      }
      if (cfg->gantry->homeTheta()) {
        g_homeThetaThisSession = false;
        g_thetaHomePending = true;
        ESP_LOGI(TAG, "Theta origin captured (HIPERFACE; no X31). %s",
                 cfg->gantry->isThetaDriveOriginAligned()
                     ? "ALIGNED: joint = drive abs"
                     : "OFFSET: thetalim shrunk to remaining drive travel; "
                       "C0300 at this pose then 'home t' for full envelope");
        ESP_LOGI(TAG, "Theta returning to 0 deg (use 'status' to monitor)");
        resolveThetaHome(cfg);
      } else {
        ESP_LOGE(TAG, "ERROR: Theta home failed (enable, Class 1 on CIP .23, clear alarm)");
      }
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
      if (!cfg->gantry->hasThetaAxis()) {
        ESP_LOGE(TAG, "ERROR: Theta axis not compiled in (CONFIG_EIP_AXIS_THETA)");
        return;
      }
      resolveThetaHome(cfg);
      if (g_thetaHomePending) {
        ESP_LOGE(TAG, "ERROR: Theta home still returning to 0 deg — wait for busy=0");
        return;
      }
      if (!g_homeThetaThisSession) {
        ESP_LOGE(TAG, "ERROR: Run 'home t' first after startup");
        return;
      }
      if (!cfg->gantry->isEnabled()) {
        ESP_LOGE(TAG, "ERROR: Motors not enabled");
        return;
      }
      {
        const int span = cfg->gantry->calibrateTheta();
        if (span > 0) {
          g_calibratedThetaThisSession = true;
          ESP_LOGI(TAG,
                   "OK Theta calibrated: envelope %.1f .. %.1f deg (span %d); "
                   "drive_abs=%.3f aligned=%d",
                   (double)cfg->gantry->getJointLimits().theta_min,
                   (double)cfg->gantry->getJointLimits().theta_max, span,
                   (double)cfg->gantry->getThetaDriveAbsDeg(),
                   cfg->gantry->isThetaDriveOriginAligned() ? 1 : 0);
        } else {
          ESP_LOGE(TAG, "ERROR: Theta calibrate failed");
        }
      }
      break;
  }
}

}  // namespace GantryConsoleImpl

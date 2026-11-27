/**
 * @file gantry_console_cmd_motion.cpp
 * @brief Gantry test console: enable, stop, faults, move, grip and test-cycle commands.
 *
 * One of the gantry_console_*.cpp files that make up the test console;
 * gantry_console_internal.h declares what they share.
 */

#include "gantry_console_internal.h"

namespace GantryConsoleImpl {

void cmdFaults(const GantryTestConsoleConfig *cfg, const char *, const char *) {
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
  char tSum[192] = {};
  if (cfg->gantry->hasThetaAxis()) {
    if (cfg->gantry->getThetaDriveAlarmSummary(tSum, sizeof(tSum))) {
      ESP_LOGI(TAG, "T: %s", tSum[0] ? tSum : "clear");
    } else {
      ESP_LOGI(TAG, "T: %s", tSum[0] ? tSum : "(no T->O)");
    }
    if (!cfg->gantry->hasThetaLiveFeedback()) {
      ESP_LOGW(TAG, "T: Class 1 down — cyclic C0500 cannot reach the drive; "
                    "use 'py tools/hcs01_eng.py c0500'");
    }
  }
  cfg->gantry->logDriveAlarmSummaries();
}

void cmdEnable(const GantryTestConsoleConfig *cfg, const char *, const char *) {
  cfg->gantry->enable();
  if (cfg->gantry->isEnabled()) {
    ESP_LOGI(TAG, "OK Motors enabled (settling for Active - wait for 'Servo arm complete')");
  } else {
    ESP_LOGE(TAG, "ERROR: Motor enable failed (check alarm/driver state)");
  }
}

void cmdDisable(const GantryTestConsoleConfig *cfg, const char *, const char *) {
  cfg->gantry->disable();
  ESP_LOGI(TAG, "OK Motors disabled");
}

void cmdTestCycle(const GantryTestConsoleConfig *cfg, const char *, const char *) {
  if (g_testCycleInProgress || g_calibrationInProgress ||
      cfg->gantry->isBusy()) {
    ESP_LOGE(TAG, "ERROR: Gantry busy — finish or 'stop' before test_cycle");
    return;
  }
  if (!cfg->gantry->driveManagedLimitsEnabled()) {
    ESP_LOGE(TAG, "ERROR: test_cycle requires EIP drive-managed limits");
    return;
  }
  g_testCycleInProgress = true;
  BaseType_t created = xTaskCreatePinnedToCore(
      testCycleTask, "TestCycle", 8192, (void *)cfg, 2, nullptr,
      CONSOLE_TASK_CORE);
  if (created != pdPASS) {
    g_testCycleInProgress = false;
    ESP_LOGE(TAG, "ERROR: test_cycle task create failed");
  } else {
    ESP_LOGI(TAG,
             "OK test_cycle started (use 'stop' to abort, 'status' to monitor)");
  }
}

void cmdTestThetaPath(const GantryTestConsoleConfig *cfg, const char *, const char *) {
  if (g_testCycleInProgress || g_calibrationInProgress ||
      cfg->gantry->isBusy()) {
    ESP_LOGE(TAG, "ERROR: Gantry busy — finish or 'stop' before test_theta_path");
    return;
  }
  g_testCycleInProgress = true;
  BaseType_t created = xTaskCreatePinnedToCore(
      testThetaPathTask, "TestThetaPath", 8192, (void *)cfg, 2, nullptr,
      CONSOLE_TASK_CORE);
  if (created != pdPASS) {
    g_testCycleInProgress = false;
    ESP_LOGE(TAG, "ERROR: test_theta_path task create failed");
  } else {
    ESP_LOGI(TAG,
             "OK test_theta_path started (use 'stop' to abort, 'status' to "
             "monitor)");
  }
}

void cmdMove(const GantryTestConsoleConfig *cfg, const char *, const char *cmdLower) {
  const GantryConsole::MoveParse parsed = GantryConsole::parseMoveCommand(cmdLower);
  if (!parsed.ok) {
    ESP_LOGE(TAG, "Usage: move <x_%s> <z_%s> <theta_deg>",
             getLinearUnitLabel(), getLinearUnitLabel());
    return;
  }
  const float x = parsed.x;
  const float z = parsed.z;
  const float theta = parsed.theta;

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
  constexpr float kHoldEpsDeg = 0.1f;
  const bool moveT = fabsf(target.theta - cur.theta) > kHoldEpsDeg;
  if (!cfg->gantry->isWorkspaceCalibrated()) {
    ESP_LOGE(TAG,
             "ERROR: Move blocked — workspace not calibrated. Run "
             "'calibrate all' (query with 'calibrated'). Flag clears on every "
             "ESP reboot (OTA, power, crash) and when a drive loses position "
             "(motor 24 V / encoder ref).");
    return;
  }
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
  if (moveT) {
    if (!cfg->gantry->hasThetaAxis()) {
      ESP_LOGE(TAG, "ERROR: Theta move requested but axis not compiled in");
      return;
    }
    resolveThetaHome(cfg);
    if (!g_homeThetaThisSession || !g_calibratedThetaThisSession) {
      ESP_LOGE(TAG,
               "ERROR: Theta move blocked. Run 'home t' then 'calibrate t' "
               "(or 'calibrate all' after theta Class 1 is up).");
      return;
    }
    if (!cfg->gantry->zInTraverseBand()) {
      // Not a rejection: the path planner defers theta into the in-band
      // segment (or logs [INTERLOCK] if it cannot).
      ESP_LOGI(TAG,
               "Theta deferred until Z is in the SAFE_Z band (ceiling "
               "%.1f mm); the path rotates once Z retracts.",
               (double)cfg->gantry->traverseClearanceZMm());
    }
  }
  if (!moveX && !moveZ && !moveT) {
    ESP_LOGI(TAG, "Move is a no-op (already at target)");
    return;
  }

  ESP_LOGI(TAG, "Moving to: x=%.3f %s, z=%.3f %s, theta=%.1f deg",
           x, getLinearUnitLabel(), z, getLinearUnitLabel(), theta);
  Gantry::GantryError result = cfg->gantry->moveTo(
      target, g_moveSpeedMmPerS, g_moveSpeedDegPerS, g_moveAccelMmPerS2,
      g_moveDecelMmPerS2, g_moveAccelDegPerS2, g_moveDecelDegPerS2);
  if (result == Gantry::GantryError::OK) {
    vTaskDelay(pdMS_TO_TICKS(20));
    if (cfg->gantry->isBusy()) {
      ESP_LOGI(TAG, "OK Move started (use 'stop' to abort, 'status' to monitor)");
    } else {
      const Gantry::JointConfig after = cfg->gantry->getCurrentJointConfig();
      const bool at_xz = fabsf(after.x - target.x) <= 1.5f &&
                         fabsf(after.z - target.z) <= 1.5f;
      const bool at_th = fabsf(after.theta - target.theta) <= 1.0f;
      if (at_xz && at_th) {
        ESP_LOGI(TAG, "OK Move complete");
      } else {
        ESP_LOGE(TAG, "ERROR: Move command accepted but motion did not start");
        ESP_LOGI(TAG, "Check alarm/limits and commanded-vs-encoder X position in status logs");
      }
    }
  } else {
    ESP_LOGE(TAG, "ERROR: Move failed: %d (%s)", (int)result,
             gantryErrorName(result));
    if (result == Gantry::GantryError::ALREADY_MOVING) {
      ESP_LOGI(TAG, "Gantry reports busy - wait for motion to finish, or 'stop' then 'enable'");
    }
  }
}

void cmdGrip(const GantryTestConsoleConfig *cfg, const char *cmd, const char *) {
  int value = 0;
  int parsed = sscanf(cmd, "grip %d", &value);
  if (parsed < 1) {
    ESP_LOGE(TAG, "Usage: grip <0|1>");
    return;
  }
  cfg->gantry->grip(value != 0);
  ESP_LOGI(TAG, "OK Gripper %s", value ? "closed" : "opened");
}

void cmdStop(const GantryTestConsoleConfig *cfg, const char *, const char *) {
  // Stop motion while servos stay enabled, then disable. Do not reverse
  // that order: disable-only left sticky CIP; re-enable ClearCip StopMotion
  // then raised A603.
  cfg->gantry->requestAbort();
  cfg->gantry->disable();
  ESP_LOGI(TAG, "OK Stop requested (motion stopped, motors disabled)");
  if (g_calibrationInProgress) {
    ESP_LOGI(TAG, "Calibration abort requested");
  }
  if (g_testCycleInProgress) {
    ESP_LOGI(TAG, "test_cycle abort requested");
  }
}

void cmdAlarmReset(const GantryTestConsoleConfig *cfg, const char *, const char *) {
  if (cfg->gantry->clearAlarm()) {
    ESP_LOGI(TAG, "OK Alarm reset pulse sent (clears Fault/Warning latch e.g. A603)");
    char xSum[192] = {};
    char zSum[192] = {};
    char tSum[192] = {};
    (void)cfg->gantry->getXDriveAlarmSummary(xSum, sizeof(xSum));
    (void)cfg->gantry->getZDriveAlarmSummary(zSum, sizeof(zSum));
    (void)cfg->gantry->getThetaDriveAlarmSummary(tSum, sizeof(tSum));
    ESP_LOGI(TAG, "After reset - X: %s | Z: %s | T: %s",
             xSum[0] ? xSum : "?", zSum[0] ? zSum : "?",
             tSum[0] ? tSum : (cfg->gantry->hasThetaAxis() ? "?" : "n/a"));
    if (cfg->gantry->hasThetaAxis() && !cfg->gantry->hasThetaLiveFeedback()) {
      ESP_LOGW(TAG, "Theta Class 1 down — arst bit5 did not reach HCS01; "
                    "use 'py tools/hcs01_eng.py c0500'");
    }
  } else {
    ESP_LOGE(TAG, "ERROR: Alarm reset failed (ARST pin may be disabled)");
  }
}

}  // namespace GantryConsoleImpl

/**
 * @file gantry_console_commands.cpp
 * @brief Gantry test console: command dispatch.
 *
 * One of the gantry_console_*.cpp files that make up the test console;
 * gantry_console_internal.h declares what they share.
 */

#include "gantry_console_internal.h"

namespace GantryConsoleImpl {

// Per-axis MOVE log periodic rate (Hz). 0 disables periodic output; the
// start/end transition events from the axis classes always fire.
static uint32_t g_axisLogFrequencyHz = 0;

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

  const bool cfg_cmd = cmdTokenIs(cmd, "cfg");
  if (cfg->gantry == nullptr && strcmp(cmdLower, "help") != 0 &&
      strcmp(cmdLower, "?") != 0 && !cfg_cmd) {
    ESP_LOGE(TAG, "Gantry not initialized");
    return;
  }

  if (strcmp(cmdLower, "help") == 0 || strcmp(cmdLower, "?") == 0) {
    gantryTestPrintHelp();
  } else if (cfg_cmd) {
    runCfgCommand(cfg, cmd);
  } else if (g_testCycleInProgress &&
             strcmp(cmdLower, "stop") != 0 &&
             strcmp(cmdLower, "status") != 0 &&
             strcmp(cmdLower, "calibrated") != 0 &&
             strcmp(cmdLower, "faults") != 0 &&
             strcmp(cmdLower, "alarms") != 0 &&
             strcmp(cmdLower, "eiptiming") != 0 &&
             strcmp(cmdLower, "gaptrace") != 0 &&
             strcmp(cmdLower, "bootlog") != 0 &&
             strncmp(cmdLower, "ethlink", 7) != 0 &&
             strcmp(cmdLower, "limits") != 0 &&
             strcmp(cmdLower, "puuinfo") != 0 &&
             strncmp(cmdLower, "puucal", 6) != 0 &&
             strncmp(cmdLower, "puu ", 4) != 0 &&
             strncmp(cmdLower, "thetalim", 8) != 0 &&
             strncmp(cmdLower, "speed", 5) != 0 &&
             strncmp(cmdLower, "accel", 5) != 0 &&
             strncmp(cmdLower, "rangelimit", 10) != 0 &&
             strncmp(cmdLower, "livepos", 7) != 0 &&
             strncmp(cmdLower, "axislog", 7) != 0) {
    ESP_LOGE(TAG,
             "ERROR: test_cycle/test_theta_path running — 'stop' to abort, "
             "'status' to monitor");
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
  } else if (strncmp(cmdLower, "ota", 3) == 0) {
    gantryOtaPrintStatus();
  } else if (strcmp(cmdLower, "status") == 0) {
    printStatus(cfg->gantry);
  } else if (strcmp(cmdLower, "calibrated") == 0) {
    printWorkspaceCalibrated(cfg->gantry);
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
  } else if (strcmp(cmdLower, "puuinfo") == 0) {
    printPuuInfo(cfg->gantry);
  } else if (strncmp(cmdLower, "puucal", 6) == 0) {
    runPuuCalCommand(cfg->gantry, cmd);
  } else if (strncmp(cmdLower, "puu ", 4) == 0) {
    runPuuSetCommand(cfg->gantry, cmd);
  } else if (strncmp(cmdLower, "thetalim", 8) == 0) {
    runThetaLimCommand(cfg->gantry, cmd);
  } else if (strncmp(cmdLower, "autotune", 8) == 0) {
    runAutotuneCommand(cfg, cmd);
  } else if (strncmp(cmdLower, "ethlink", 7) == 0) {
    runEthLinkCommand(cfg, cmdLower);
  } else if (strncmp(cmdLower, "drv", 3) == 0 && (cmdLower[3] == '\0' || cmdLower[3] == ' ')) {
    runDrvCommand(cfg, cmd);
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
  } else if (strcmp(cmdLower, "test_cycle") == 0) {
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
  } else if (strcmp(cmdLower, "test_theta_path") == 0) {
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
  } else if (strncmp(cmdLower, "calibrate", 9) == 0 &&
             (cmdLower[9] == '\0' || cmdLower[9] == ' ' || cmdLower[9] == '\t')) {
    dispatchPerAxisCommand(cfg, cmdLower, "calibrate", runCalibrateForAxis);
  } else if (strncmp(cmdLower, "speed", 5) == 0) {
    const GantryConsole::SpeedParse parsed = GantryConsole::parseSpeedCommand(cmdLower);
    if (!parsed.ok) {
      ESP_LOGE(TAG, "Usage: speed <mm_per_s> [deg_per_s]");
      return;
    }
    const uint32_t requestedSpeedMm = (uint32_t)convertSelectedToMm((float)parsed.speed_mm);
    const uint32_t clampedSpeedMm =
        applyRangeLimitU32(requestedSpeedMm, kMinSpeedMmPerS, maxLinearSpeedMmPerS(),
                           g_motionProfileRangeLimitEnabled);
    g_moveSpeedMmPerS = clampedSpeedMm;
    if (parsed.has_deg) {
      const uint32_t requestedDeg = (uint32_t)parsed.speed_deg;
      const uint32_t clampedDeg =
          applyRangeLimitU32(requestedDeg, 1u, maxThetaSpeedDegPerS(),
                             g_motionProfileRangeLimitEnabled);
      g_moveSpeedDegPerS = clampedDeg;
      if (g_motionProfileRangeLimitEnabled && clampedDeg != requestedDeg) {
        ESP_LOGW(TAG, "Theta speed clamped: requested=%lu deg/s -> applied=%lu deg/s",
                 (unsigned long)requestedDeg, (unsigned long)clampedDeg);
      }
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
    const GantryConsole::AccelParse parsed = GantryConsole::parseAccelCommand(cmdLower);
    if (!parsed.ok) {
      ESP_LOGE(TAG,
               "Usage: accel <mm_per_s2> [decel_mm] [deg_per_s2] [decel_deg] (values must be > 0)");
      return;
    }
    const int accel = parsed.accel;
    const int decel = parsed.decel;
    const int accelDeg = parsed.accel_deg;
    const int decelDeg = parsed.decel_deg;
    const uint32_t requestedAccel = (uint32_t)convertSelectedToMm((float)accel);
    const uint32_t requestedDecel =
        (parsed.n >= 2 && decel > 0) ? (uint32_t)convertSelectedToMm((float)decel)
                                   : (uint32_t)convertSelectedToMm((float)accel);
    const uint32_t clampedAccel =
        applyRangeLimitU32(requestedAccel, kMinAccelMmPerS2, maxLinearAccelMmPerS2(),
                           g_motionProfileRangeLimitEnabled);
    const uint32_t clampedDecel =
        applyRangeLimitU32(requestedDecel, kMinAccelMmPerS2, maxLinearAccelMmPerS2(),
                           g_motionProfileRangeLimitEnabled);

    g_moveAccelMmPerS2 = clampedAccel;
    g_moveDecelMmPerS2 = (parsed.n >= 2 && decel >= 0) ? clampedDecel : clampedAccel;
    if (parsed.n >= 3) {
      const uint32_t reqA = (uint32_t)accelDeg;
      const uint32_t reqD =
          (parsed.n >= 4 && decelDeg > 0) ? (uint32_t)decelDeg : reqA;
      const uint32_t clA =
          applyRangeLimitU32(reqA, kMinAccelDegPerS2, maxThetaAccelDegPerS2(),
                             g_motionProfileRangeLimitEnabled);
      const uint32_t clD =
          applyRangeLimitU32(reqD, kMinAccelDegPerS2, maxThetaAccelDegPerS2(),
                             g_motionProfileRangeLimitEnabled);
      g_moveAccelDegPerS2 = clA;
      g_moveDecelDegPerS2 = clD;
      if (g_motionProfileRangeLimitEnabled && (clA != reqA || clD != reqD)) {
        ESP_LOGW(TAG,
                 "Theta accel/decel clamped: requested=(%lu,%lu) -> applied=(%lu,%lu) deg/s2",
                 (unsigned long)reqA, (unsigned long)reqD,
                 (unsigned long)clA, (unsigned long)clD);
      }
    }
    if (g_motionProfileRangeLimitEnabled &&
        (clampedAccel != requestedAccel || clampedDecel != requestedDecel)) {
      ESP_LOGW(TAG, "Accel/decel clamped: requested=(%.3f,%.3f) %s/s2 -> applied=(%.3f,%.3f) %s/s2",
               convertMmToSelected((float)requestedAccel), convertMmToSelected((float)requestedDecel),
               getLinearUnitLabel(),
               convertMmToSelected((float)clampedAccel), convertMmToSelected((float)clampedDecel),
               getLinearUnitLabel());
    }
    ESP_LOGI(TAG,
             "OK Path accel: %.3f / %.3f %s/s2 (resultant); theta %lu / %lu deg/s2",
             convertMmToSelected((float)g_moveAccelMmPerS2),
             convertMmToSelected((float)g_moveDecelMmPerS2), getLinearUnitLabel(),
             (unsigned long)g_moveAccelDegPerS2,
             (unsigned long)g_moveDecelDegPerS2);
  } else if (strncmp(cmdLower, "units", 5) == 0) {
    char unitStr[16] = {0};
    int parsed = sscanf(cmdLower, "units %15s", unitStr);
    if (parsed < 1) {
      ESP_LOGE(TAG, "Usage: units <mm|in>");
      return;
    }
    if (!GantryConsole::parseLinearUnit(unitStr, g_linearUnitMode)) {
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
    ESP_LOGI(TAG, "Configured theta ranges: speed=1..%lu deg/s, accel/decel=%lu..%lu deg/s2",
             (unsigned long)maxThetaSpeedDegPerS(),
             (unsigned long)kMinAccelDegPerS2,
             (unsigned long)maxThetaAccelDegPerS2());
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
  } else if (strcmp(cmdLower, "boottime") == 0) {
    const Network::EthernetBootTimes bt = Network::ethernetBootTimes();
    ESP_LOGI(TAG, "boottime reset=%d (ms since app start, -1 = not reached): eth_start=%lld "
                  "eth_started=%lld phy_link_up=%lld got_ip=%lld",
             static_cast<int>(esp_reset_reason()),
             bt.start_us < 0 ? -1LL : static_cast<long long>(bt.start_us / 1000),
             bt.eth_started_us < 0 ? -1LL : static_cast<long long>(bt.eth_started_us / 1000),
             bt.link_up_us < 0 ? -1LL : static_cast<long long>(bt.link_up_us / 1000),
             bt.got_ip_us < 0 ? -1LL : static_cast<long long>(bt.got_ip_us / 1000));
  } else if (strncmp(cmdLower, "flashstall ", 11) == 0) {
    char mode[8] = {};
    int writes = 96;
    const int got = sscanf(cmdLower + 11, "%7s %d", mode, &writes);
    const bool erase = strcmp(mode, "erase") == 0;
    char line[200];
    if (got < 1 || (!erase && strcmp(mode, "write") != 0)) {
      ESP_LOGE(TAG, "usage: flashstall erase|write [n]");
      return;
    }
    if (gantryOtaFlashStallTest(cfg->gantry, erase, writes, line, sizeof(line)) != ESP_OK) {
      ESP_LOGE(TAG, "ERROR: %s", line);
      return;
    }
    ESP_LOGI(TAG, "%s", line);
  } else if (strcmp(cmdLower, "netstat") == 0 || strcmp(cmdLower, "netstat reset") == 0) {
    char line[200];
    if (!pickSchedulerNetStats(strcmp(cmdLower, "netstat reset") == 0, line, sizeof(line))) {
      ESP_LOGE(TAG, "ERROR: %s", line);
      return;
    }
    ESP_LOGI(TAG, "%s", line);
  } else if (strncmp(cmdLower, "vismark", 7) == 0) {
    const char* arg = cmdLower + 7;
    while (*arg == ' ') {
      ++arg;
    }
    char line[160];
    const bool on = strcmp(arg, "on") == 0;
    if ((!on && strcmp(arg, "off") != 0) || !pickSchedulerVisMark(on, line, sizeof(line))) {
      ESP_LOGE(TAG, "ERROR: %s", (on || strcmp(arg, "off") == 0) ? line : "usage: vismark on|off");
      return;
    }
    ESP_LOGI(TAG, "%s", line);
  } else if (strncmp(cmdLower, "gripmark", 8) == 0) {
    // Ahead of "grip", whose prefix test would also take "gripmark".
    const char* arg = cmdLower + 8;
    while (*arg == ' ') {
      ++arg;
    }
    char line[160];
    const bool on = strcmp(arg, "on") == 0;
    if ((!on && strcmp(arg, "off") != 0) || !pickSchedulerGripMark(on, line, sizeof(line))) {
      ESP_LOGE(TAG, "ERROR: %s", (on || strcmp(arg, "off") == 0) ? line : "usage: gripmark on|off");
      return;
    }
    ESP_LOGI(TAG, "%s", line);
  } else if (strncmp(cmdLower, "grip", 4) == 0) {
    int value = 0;
    int parsed = sscanf(cmd, "grip %d", &value);
    if (parsed < 1) {
      ESP_LOGE(TAG, "Usage: grip <0|1>");
      return;
    }
    cfg->gantry->grip(value != 0);
    ESP_LOGI(TAG, "OK Gripper %s", value ? "closed" : "opened");
  } else if (strncmp(cmdLower, "picksim", 7) == 0) {
    float x_mm = 0.0f;
    float y_mm = 0.0f;
    float theta_deg = 0.0f;
    int battery_class = 0;
    unsigned item_id = 0;
    float width_mm = 0.0f;
    float length_mm = 0.0f;
    float height_from_belt_mm = 0.0f;
    const int n = sscanf(cmdLower, "picksim %f %f %f %d %u %f %f %f", &x_mm, &y_mm, &theta_deg,
                         &battery_class, &item_id, &width_mm, &length_mm, &height_from_belt_mm);
    if (n != 8) {
      ESP_LOGE(TAG,
               "Usage: picksim <x> <y> <theta> <class 1-13> <item_id> <width_mm> <length_mm> <height_from_belt_mm>");
      return;
    }
    if (!pickSchedulerInject(x_mm, y_mm, theta_deg, static_cast<uint8_t>(battery_class), item_id,
                             width_mm, length_mm, height_from_belt_mm)) {
      ESP_LOGE(TAG, "ERROR: picksim not queued");
    }
  } else if (strncmp(cmdLower, "pickplan", 8) == 0) {
    float x_mm = 0.0f;
    float y_mm = 0.0f;
    float theta_deg = 0.0f;
    int battery_class = 0;
    unsigned item_id = 0;
    float width_mm = 0.0f;
    float length_mm = 0.0f;
    float height_from_belt_mm = 0.0f;
    const int n = sscanf(cmdLower, "pickplan %f %f %f %d %u %f %f %f", &x_mm, &y_mm, &theta_deg,
                         &battery_class, &item_id, &width_mm, &length_mm, &height_from_belt_mm);
    if (n != 8) {
      ESP_LOGE(TAG,
               "Usage: pickplan <x> <y> <theta> <class 1-13> <item_id> <width_mm> <length_mm> <height_from_belt_mm>");
      return;
    }
    char plan[512];
    if (!pickSchedulerPlan(x_mm, y_mm, theta_deg, static_cast<uint8_t>(battery_class), item_id,
                           plan, sizeof(plan), width_mm, length_mm, height_from_belt_mm)) {
      ESP_LOGE(TAG, "ERROR: %s", plan);
      return;
    }
    ESP_LOGI(TAG, "pickplan %s", plan);
  } else if (strncmp(cmdLower, "pickcover", 9) == 0) {
    int cls = 1;
    if (sscanf(cmdLower, "pickcover %d", &cls) != 1) {
      cls = 1;
    }
    char line[384];
    if (!pickSchedulerCover(static_cast<uint8_t>(cls), line, sizeof(line))) {
      ESP_LOGE(TAG, "ERROR: %s", line);
      return;
    }
    ESP_LOGI(TAG, "pickcover %s", line);
  } else if (strncmp(cmdLower, "pickcal", 7) == 0) {
    const char* args = cmdLower + 7;
    while (*args == ' ') {
      ++args;
    }
    char line[384];
    if (!pickSchedulerCalibrate(args, line, sizeof(line))) {
      ESP_LOGE(TAG, "ERROR: %s", line);
      return;
    }
    ESP_LOGI(TAG, "%s", line);
  } else if (strncmp(cmdLower, "beltsim", 7) == 0) {
    float mm_s = 0.0f;
    if (sscanf(cmdLower, "beltsim %f", &mm_s) != 1) {
      ESP_LOGE(TAG, "Usage: beltsim <mm_s>  (0 = off)");
      return;
    }
    char line[160];
    if (!pickSchedulerBeltSim(mm_s, line, sizeof(line))) {
      ESP_LOGE(TAG, "ERROR: %s", line);
      return;
    }
    ESP_LOGI(TAG, "%s", line);
  } else if (strncmp(cmdLower, "ztrace", 6) == 0) {
    const char* args = cmdLower + 6;
    while (*args == ' ') {
      ++args;
    }
    char line[512];
    if (!pickSchedulerTrace(args, line, sizeof(line))) {
      ESP_LOGE(TAG, "ERROR: %s", line);
      return;
    }
    ESP_LOGI(TAG, "%s", line);
  } else if (strncmp(cmdLower, "thtrace", 7) == 0) {
    const char* args = cmdLower + 7;
    while (*args == ' ') {
      ++args;
    }
    char line[768];
    if (!thetaTraceCommand(cfg->gantry, args, line, sizeof(line))) {
      ESP_LOGE(TAG, "ERROR: %s", line);
      return;
    }
    ESP_LOGI(TAG, "%s", line);
  } else if (strncmp(cmdLower, "beltcal", 7) == 0) {
    const char* arg = cmdLower + 7;
    while (*arg == ' ') {
      ++arg;
    }
    char line[384];
    if (!pickSchedulerBeltCal(arg, line, sizeof(line))) {
      ESP_LOGE(TAG, "ERROR: %s", line);
      return;
    }
    ESP_LOGI(TAG, "%s", line);
  } else if (strcmp(cmdLower, "netcon") == 0) {
    char line[480];
    gantryNetConsoleStats(line, sizeof(line));
    ESP_LOGI(TAG, "%s", line);
  } else if (strcmp(cmdLower, "eiptiming") == 0) {
    eip::dumpClass1TimingStats();
  } else if (strcmp(cmdLower, "gaptrace") == 0) {
    eip::printLastClass1GapReport();
  } else if (strcmp(cmdLower, "bootlog") == 0) {
    ESP_LOGI(TAG, "=== boot log (%u bytes captured from reset) ===",
             static_cast<unsigned>(bootLogRingBytes()));
    bootLogRingForEach(
        [](const char *line, void *) { ESP_LOGI(TAG, "boot| %s", line); }, nullptr);
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
    if (g_testCycleInProgress) {
      ESP_LOGI(TAG, "test_cycle abort requested");
    }
  } else if (strcmp(cmdLower, "alarmreset") == 0 || strcmp(cmdLower, "arst") == 0) {
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

}  // namespace GantryConsoleImpl

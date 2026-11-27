/**
 * @file gantry_console_status.cpp
 * @brief Gantry test console: status, PUU, limit and pin printouts and their set commands.
 *
 * One of the gantry_console_*.cpp files that make up the test console;
 * gantry_console_internal.h declares what they share.
 */

#include "gantry_console_internal.h"

namespace GantryConsoleImpl {

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
  ESP_LOGI(TAG, "Z Position: %.3f %s (+Z = down / toward belt; 0 = A015 retract)", zDisp, getLinearUnitLabel());
  ESP_LOGI(TAG, "Theta: %.2f deg%s", gantry->getCurrentThetaDeg(),
           gantry->hasThetaAxis() ? "" : " (axis not compiled in)");
  ESP_LOGI(TAG, "Motor Enabled: %s", gantry->isEnabled() ? "Yes" : "No");
  ESP_LOGI(TAG, "Unsaved parameters: %d", Config::AppConfig::instance().pendingChanges());
  ESP_LOGI(TAG, "Busy: %s", gantry->isBusy() ? "Yes" : "No");
  ESP_LOGI(TAG, "Alarm: %s", gantry->isAlarmActive() ? "Yes" : "No");
  ESP_LOGI(TAG, "Workspace calibrated: %s",
           gantry->isWorkspaceCalibrated() ? "Yes" : "No");
  ESP_LOGI(TAG, "Uptime: %lu s (last reset: %s)",
           static_cast<unsigned long>(esp_timer_get_time() / 1000000LL), gantryResetReasonName());
  // Never-used stack of this console task (UART or TCP): a low value warns
  // before a long command like 'calibrate all' overflows it.
  ESP_LOGI(TAG, "Console stack free (min): %lu of %d bytes",
           static_cast<unsigned long>(uxTaskGetStackHighWaterMark(nullptr)), CONSOLE_TASK_STACK);
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
  ESP_LOGI(TAG, "2-D Path Profile: speed=%.3f %s/s, accel=%.3f %s/s2, decel=%.3f %s/s2",
           convertMmToSelected((float)g_moveSpeedMmPerS), getLinearUnitLabel(),
           convertMmToSelected((float)g_moveAccelMmPerS2), getLinearUnitLabel(),
           convertMmToSelected((float)g_moveDecelMmPerS2), getLinearUnitLabel());
  ESP_LOGI(TAG, "Theta Profile: speed=%lu deg/s, accel=%lu deg/s2, decel=%lu deg/s2",
           (unsigned long)g_moveSpeedDegPerS,
           (unsigned long)g_moveAccelDegPerS2,
           (unsigned long)g_moveDecelDegPerS2);
  ESP_LOGI(TAG, "Path Range Limits: %s (speed:%.3f-%.3f %s/s, accel/decel:%.3f-%.3f %s/s2)",
           g_motionProfileRangeLimitEnabled ? "ENABLED" : "DISABLED",
           convertMmToSelected((float)kMinSpeedMmPerS),
           convertMmToSelected((float)maxLinearSpeedMmPerS()),
           getLinearUnitLabel(),
           convertMmToSelected((float)kMinAccelMmPerS2),
           convertMmToSelected((float)maxLinearAccelMmPerS2()),
           getLinearUnitLabel());
  ESP_LOGI(TAG, "Theta Range Limits: speed=1-%lu deg/s, accel/decel=%lu-%lu deg/s2",
           (unsigned long)maxThetaSpeedDegPerS(),
           (unsigned long)kMinAccelDegPerS2,
           (unsigned long)maxThetaAccelDegPerS2());
  {
    const Gantry::JointLimits lim = gantry->getJointLimits();
    ESP_LOGI(TAG, "Theta joint limits: %.1f .. %.1f deg (thetalim)",
             (double)lim.theta_min, (double)lim.theta_max);
    ESP_LOGI(TAG, "Theta drive abs: %.3f deg (S-0-0051) origin %s",
             (double)gantry->getThetaDriveAbsDeg(),
             gantry->isThetaDriveOriginAligned() ? "ALIGNED joint=drive"
                                                 : "OFFSET (run C0300 at home pose, then home t)");
    char cip[192] = {};
    if (gantry->getThetaCipStatus(cip, sizeof(cip)) && cip[0]) {
      ESP_LOGI(TAG, "Theta CIP: %s", cip);
    }
  }
  {
    // Dropout diagnostics: periodic W5500 register read-back (W5500Diag).
    const w5500::SpiCheckStats sc = w5500::spiCheckStats();
    ESP_LOGI(TAG, "W5500 SPI check: %lu reads, bad ver/mac/ip=%lu/%lu/%lu, bad rx hdr=%lu",
             (unsigned long)sc.checks, (unsigned long)sc.version_bad,
             (unsigned long)sc.mac_bad, (unsigned long)sc.ip_bad,
             (unsigned long)sc.rx_hdr_bad);
  }
  ESP_LOGI(TAG, "Units: linear=%s (internal mm)", getLinearUnitLabel());
  ESP_LOGI(TAG, "PUU scale: X=%.3f PUU/mm  Z=%.3f PUU/mm  T=%.3f PUU/deg (puuinfo / puu t / puucal)",
           gantry->getPulsesPerMm(), gantry->getZPulsesPerMm(),
           gantry->getThetaPulsesPerDeg());

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
           xPpm, xMm, gantry->getXEncoderRaw(), target.x, stored().axis_x_lead_mm_per_rev);
  ESP_LOGI(TAG, "Z: scale=%.4f PUU/mm  actual=%.3f mm (%ld PUU)  target=%.3f mm  lead=%.1f mm/rev",
           zPpm, zMm, static_cast<long>(gantry->getZEncoderPulses()), target.z,
           stored().axis_z_lead_mm_per_rev);
  if (gantry->hasThetaAxis()) {
    const float tPpd = gantry->getThetaPulsesPerDeg();
    ESP_LOGI(TAG,
             "Theta: scale=%.4f PUU/deg  actual=%.3f deg  target=%.3f deg  "
             "(HIPERFACE abs; confirm with a known-degree move)",
             tPpd, gantry->getCurrentThetaDeg(), target.theta);
  } else {
    ESP_LOGI(TAG, "Theta: axis not compiled in (CONFIG_EIP_AXIS_THETA)");
  }
  ESP_LOGI(TAG, "Calibrate: move a known delta, measure travel, then:");
  ESP_LOGI(TAG, "  puucal <x|z|t> c m   - suggest new PUU/mm or PUU/deg from commanded vs measured");
  ESP_LOGI(TAG, "Suggested Kconfig: EIP_AXIS_X_PUU_PER_MM / EIP_AXIS_Z_PUU_PER_MM / EIP_AXIS_THETA_PUU_PER_DEG");
  ESP_LOGI(TAG, "Formula: new = current * (commanded / measured)");
}

void runPuuCalCommand(Gantry::Gantry *gantry, const char *cmd) {
  if (gantry == nullptr) {
    ESP_LOGE(TAG, "Gantry not initialized");
    return;
  }

  const GantryConsole::PuuCalParse parsed = GantryConsole::parsePuuCalCommand(cmd);
  if (!parsed.ok) {
    ESP_LOGE(TAG, "Usage: puucal <x|z|t> <commanded> <measured>");
    return;
  }
  const char axis = parsed.axis;
  const float commanded = parsed.commanded;
  const float measured = parsed.measured;

  if (axis == 't') {
    if (!gantry->hasThetaAxis()) {
      ESP_LOGE(TAG, "Theta axis not compiled in");
      return;
    }
    const float current = gantry->getThetaPulsesPerDeg();
    double suggested = 0.0;
    if (!GantryConsole::suggestPuuScale(static_cast<double>(current), commanded,
                                        measured, suggested)) {
      ESP_LOGE(TAG, "No PUU/deg scale available for theta");
      return;
    }
    const float errPct = (measured - commanded) / commanded * 100.0f;
    ESP_LOGI(TAG, "=== PUU calibration (t) ===");
    ESP_LOGI(TAG, "Commanded=%.3f deg  Measured=%.3f deg  error=%+.2f%%",
             commanded, measured, errPct);
    ESP_LOGI(TAG, "Current scale=%.4f PUU/deg", current);
    ESP_LOGI(TAG, "Suggested scale=%.4f PUU/deg", suggested);
    if (gantry->setThetaPuuPerDeg(suggested)) {
      ESP_LOGI(TAG, "Applied live. Re-run 'home t'. Persist with CONFIG_EIP_AXIS_THETA_PUU_PER_DEG=%.4f",
               suggested);
    } else {
      ESP_LOGE(TAG, "Failed to apply PUU/deg");
    }
    return;
  }

  const float current =
      (axis == 'x') ? gantry->getPulsesPerMm() : gantry->getZPulsesPerMm();
  double suggested = 0.0;
  if (!GantryConsole::suggestPuuScale(static_cast<double>(current), commanded,
                                      measured, suggested)) {
    ESP_LOGE(TAG, "No PUU/mm scale available for axis %c", axis);
    return;
  }
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

void runPuuSetCommand(Gantry::Gantry *gantry, const char *cmd) {
  if (gantry == nullptr) {
    ESP_LOGE(TAG, "Gantry not initialized");
    return;
  }
  char axis = '\0';
  double scale = 0.0;
  if (sscanf(cmd, "puu %c %lf", &axis, &scale) != 2) {
    ESP_LOGE(TAG, "Usage: puu t <puu_per_deg>");
    return;
  }
  axis = static_cast<char>(tolower(static_cast<unsigned char>(axis)));
  if (axis != 't') {
    ESP_LOGE(TAG, "Live PUU set is theta-only (X/Z stay Kconfig + puucal suggest)");
    return;
  }
  if (!gantry->hasThetaAxis()) {
    ESP_LOGE(TAG, "Theta axis not compiled in");
    return;
  }
  if (scale <= 0.0) {
    ESP_LOGE(TAG, "PUU/deg must be > 0");
    return;
  }
  if (!gantry->setThetaPuuPerDeg(scale)) {
    ESP_LOGE(TAG, "Failed to apply PUU/deg");
    return;
  }
  ESP_LOGI(TAG, "OK Theta scale=%.4f PUU/deg. Re-run 'home t'. Persist CONFIG_EIP_AXIS_THETA_PUU_PER_DEG=%.4f",
           scale, scale);
}

void runThetaLimCommand(Gantry::Gantry *gantry, const char *cmd) {
  if (gantry == nullptr) {
    ESP_LOGE(TAG, "Gantry not initialized");
    return;
  }
  const GantryConsole::ThetaLimParse parsed = GantryConsole::parseThetaLimCommand(cmd);
  if (!parsed.ok) {
    ESP_LOGE(TAG, "Usage: thetalim <min_deg> <max_deg>");
    return;
  }
  const float minDeg = parsed.min_deg;
  const float maxDeg = parsed.max_deg;
  gantry->setThetaLimits(minDeg, maxDeg);
  const Gantry::JointLimits applied = gantry->getJointLimits();
  if (applied.theta_min != minDeg || applied.theta_max != maxDeg) {
    ESP_LOGW(TAG,
             "thetalim clamped to captured envelope %.1f .. %.1f (requested "
             "%.1f .. %.1f) — C0300 + home t ALIGNED to widen",
             (double)applied.theta_min, (double)applied.theta_max,
             (double)minDeg, (double)maxDeg);
  }
  ESP_LOGI(TAG, "OK Theta joint limits %.1f .. %.1f deg",
           (double)applied.theta_min, (double)applied.theta_max);
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

}  // namespace GantryConsoleImpl

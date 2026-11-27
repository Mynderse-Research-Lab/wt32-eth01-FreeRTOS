/**
 * @file gantry_console_cmd_profile.cpp
 * @brief Gantry test console: speed, accel, units, range-limit and logging-rate commands.
 *
 * One of the gantry_console_*.cpp files that make up the test console;
 * gantry_console_internal.h declares what they share.
 */

#include "gantry_console_internal.h"

namespace GantryConsoleImpl {

// Per-axis MOVE log periodic rate (Hz). 0 disables periodic output; the
// start/end transition events from the axis classes always fire.
static uint32_t g_axisLogFrequencyHz = 0;

void cmdSpeed(const GantryTestConsoleConfig *, const char *, const char *cmdLower) {
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
}

void cmdAccel(const GantryTestConsoleConfig *, const char *, const char *cmdLower) {
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
}

void cmdUnits(const GantryTestConsoleConfig *, const char *, const char *cmdLower) {
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
}

void cmdRangeLimit(const GantryTestConsoleConfig *, const char *cmd, const char *) {
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
}

void cmdLivePos(const GantryTestConsoleConfig *, const char *cmd, const char *) {
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
}

void cmdAxisLog(const GantryTestConsoleConfig *cfg, const char *cmd, const char *) {
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
}

}  // namespace GantryConsoleImpl

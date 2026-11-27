/**
 * @file GantryEipLinearAxisAlarm.cpp
 * @brief Alarm state and drive-trip logging for GantryEipLinearAxis.
 *
 * One of the GantryEipLinearAxis files; GantryEipLinearAxisInternal.h
 * holds what they share.
 */

#include "GantryEipLinearAxis.h"
#include "GantryEipLinearAxisInternal.h"
#include "GantryControlPeriod.h"
#include "KinetixFaultCodes.h"

#include <cmath>
#include <cstdio>

namespace Gantry {

bool GantryEipLinearAxis::isAlarmActive() const {
  eip::k5100::InputAssembly154 fb;
  if (!readFeedback(fb)) return false;
  return fb.fault || fb.connection_faulted;
}

bool GantryEipLinearAxis::clearAlarm() {
  motion_commanded_ = false;
  move_phase_ = MovePhase::kIdle;
  move_phase_us_ = 0;

  eip::k5100::InputAssembly154 fb;
  if (readFeedback(fb)) {
    command_position_puu_ = fb.actual_position;
    target_mm_ = puuToMm(toJointPuu(command_position_puu_));
  }

  eip::k5100::OutputAssembly104 cmd = buildSettleCommand(enabled_);
  cmd.fault_reset = true;
  fault_reset_pulse_us_ = Gantry::legacyTicksUs(3);
  return publishCommand(cmd);
}

bool GantryEipLinearAxis::getDriveAlarmSummary(char* buf, size_t n) const {
  eip::k5100::InputAssembly154 fb;
  if (!readFeedback(fb)) {
    if (buf && n) std::snprintf(buf, n, "no feedback");
    return false;
  }
  (void)eip::k5100::formatDriveTripSummary(
      buf, n, fb.fault, fb.warning_present, fb.connection_faulted, fb.fault_code,
      fb.warning_code);
  return true;
}

uint16_t GantryEipLinearAxis::getDriveFaultCode() const {
  eip::k5100::InputAssembly154 fb;
  if (!readFeedback(fb)) return 0;
  return fb.fault_code;
}

uint16_t GantryEipLinearAxis::getDriveWarningCode() const {
  eip::k5100::InputAssembly154 fb;
  if (!readFeedback(fb)) return 0;
  return fb.warning_code;
}

bool GantryEipLinearAxis::isA014WarningActive() const {
  eip::k5100::InputAssembly154 fb;
  if (!readFeedback(fb)) return false;
  return fb.warning_present && eip::k5100::isWarningA014(fb.warning_code);
}

bool GantryEipLinearAxis::isA015WarningActive() const {
  eip::k5100::InputAssembly154 fb;
  if (!readFeedback(fb)) return false;
  return fb.warning_present && eip::k5100::isWarningA015(fb.warning_code);
}

void GantryEipLinearAxis::logAlarmEdge(const eip::k5100::InputAssembly154& fb) {
  const bool fault_now = fb.fault || fb.connection_faulted;
  const bool warn_now = fb.warning_present;
  const bool changed =
      (fault_now != last_fault_latched_) || (warn_now != last_warning_latched_) ||
      (fb.fault_code != last_fault_code_) || (fb.warning_code != last_warning_code_);
  if (!changed) return;

  last_fault_latched_ = fault_now;
  last_warning_latched_ = warn_now;
  last_fault_code_ = fb.fault_code;
  last_warning_code_ = fb.warning_code;

  char summary[192];
  eip::k5100::formatDriveTripSummary(summary, sizeof(summary), fb.fault,
                                     fb.warning_present, fb.connection_faulted,
                                     fb.fault_code, fb.warning_code);
  const char* tag = (log_tag_ && log_tag_[0]) ? log_tag_ : kEipAxisTag;
  if (fault_now || warn_now || fb.connection_faulted) {
    ESP_LOGW(tag, "Drive trip: %s", summary);
    if (eip::k5100::isWarningA603(fb.warning_code)) {
      ESP_LOGW(tag,
               "A603 hint: need Active; Absolute moves use OM=1 TM=2 after "
               "Home34 (see lib/Gantry/REFERENCE.md)");
    }
    eip::k5100::DriveCodeInfo info{};
    char disp[8];
    if (fb.fault_code &&
        eip::k5100::lookupDriveCode(fb.fault_code, 'E', disp, sizeof(disp), info)) {
      ESP_LOGW(tag, "  %s: %s", info.display, info.hint);
    }
    if (fb.warning_code &&
        eip::k5100::lookupDriveCode(fb.warning_code, 'A', disp, sizeof(disp),
                                    info)) {
      ESP_LOGW(tag, "  %s: %s", info.display, info.hint);
    }
  } else {
    ESP_LOGI(tag, "Drive trip cleared");
  }
}

}  // namespace Gantry

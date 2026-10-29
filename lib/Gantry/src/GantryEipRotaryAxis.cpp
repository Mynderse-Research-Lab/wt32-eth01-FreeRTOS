/**
 * @file GantryEipRotaryAxis.cpp
 */

#include "GantryEipRotaryAxis.h"
#include "GantryControlPeriod.h"

#include "axis_drivetrain_params.h"
#include "GantryRotaryPhases.h"
#include "GantryThetaOrigin.h"
#include "GantryUtils.h"
#include "Hcs01Assembly.h"
#include "Hcs01ControlStatus.h"

#include <cmath>
#include <cstdio>

#if defined(ESP_PLATFORM)
#include "esp_log.h"
#include "esp_timer.h"
#else
#include "esp_timer.h"
#define ESP_LOGW(tag, fmt, ...) ((void)0)
#define ESP_LOGI(tag, fmt, ...) ((void)0)
#endif

namespace Gantry {

namespace {
constexpr uint8_t kMovePreloadTicks = RotaryPhase::kPreloadTicks;
constexpr uint8_t kMoveReleaseHaltTicks = RotaryPhase::kReleaseHaltTicks;
constexpr uint8_t kMoveStartTicks = RotaryPhase::kStartAckTicks;
constexpr uint8_t kClearFaultTicks = 10;     // hold P-0-4077 bit5 (C0500)
constexpr uint16_t kWaitAfTicks = 100;       // 1 s @ 100 Hz
constexpr uint8_t kArrivalStableTicks = 3;   // ~30 ms @ 100 Hz at target
// Theta positioning tolerance: 0.01 deg
constexpr float kArrivalEpsDeg = 0.01f;

const char* readyLabel(const eip::hcs01::Hcs01StatusWord& st) {
  using eip::hcs01::ReadyForOperation;
  if (st.ready == ReadyForOperation::kInOperation) {
    return st.not_following_command ? "AH" : "AF";
  }
  if (st.ready == ReadyForOperation::kReadyAb) return "Ab";
  if (st.ready == ReadyForOperation::kReadyBb) return "bb";
  return "AS";
}
}  // namespace

GantryEipRotaryAxis::GantryEipRotaryAxis(eip::EipProcessImage& image,
                                         const EipRotaryAxisConfig& cfg)
    : image_(image),
      config_(cfg),
      initialized_(false),
      enabled_(false),
      stop_requested_(false),
      motion_active_(false),
      target_deg_(0.0f),
      min_deg_(-180.0f),
      max_deg_(180.0f),
      envelope_min_deg_(-180.0f),
      envelope_max_deg_(180.0f),
      origin_aligned_(false),
      command_accept_level_(false),
      log_tag_(""),
      log_rate_hz_(0),
      last_axislog_us_(0),
      move_phase_(MovePhase::kIdle),
      arm_phase_(ArmPhase::kIdle),
      move_phase_us_(0),
      arm_phase_us_(0),
      cmd_pos_puu_(0),
      cmd_vel_puu_(0),
      cmd_acc_puu_(0),
      cmd_dec_puu_(0),
      stuck_log_div_(0),
      start_deg_(0.0f),
      in_pos_armed_(false),
      arrival_stable_us_(0),
      run_us_(0) {}

const char* GantryEipRotaryAxis::tag() const {
  return (log_tag_ != nullptr && log_tag_[0] != '\0') ? log_tag_ : "Theta";
}

bool GantryEipRotaryAxis::begin() {
  initialized_ = true;
  eip::hcs01::Hcs01PositioningCommand idle;
  idle.control = eip::hcs01::Hcs01ControlWord::makeDriveOff();
  return publishCommand(idle);
}

bool GantryEipRotaryAxis::enable() {
  if (!initialized_) return false;
  enabled_ = true;
  stop_requested_ = false;
  motion_active_ = false;
  move_phase_ = MovePhase::kIdle;
  move_phase_us_ = 0;
  in_pos_armed_ = false;
  arrival_stable_us_ = 0;
  run_us_ = 0;
  // F4009 latches when bits 13/14/15 fall together. Pulse C0500 (bit5)
  // before Drive ON. Armed idle is AH (ready=3 + Drive Halt), not AF.
  arm_phase_ = ArmPhase::kClearFaults;
  arm_phase_us_ = legacyTicksUs(kClearFaultTicks);
  return publishClearFaults();
}

bool GantryEipRotaryAxis::disable() {
  enabled_ = false;
  stop_requested_ = false;
  motion_active_ = false;
  move_phase_ = MovePhase::kIdle;
  in_pos_armed_ = false;
  arrival_stable_us_ = 0;
  run_us_ = 0;
  arm_phase_ = ArmPhase::kIdle;
  arm_phase_us_ = 0;
  eip::hcs01::Hcs01PositioningCommand cmd;
  cmd.control = eip::hcs01::Hcs01ControlWord::makeDriveOff();
  cmd.control.command_value_accept = command_accept_level_;
  // Keep S-0-0282 at the current encoder. Default 0 is abs 0 deg, not joint 0.
  cmd.positioning_command_value = feedbackAbsPuu();
  cmd.positioning_velocity = config_.default_velocity_puu;
  cmd.positioning_acceleration = config_.default_accel_puu;
  cmd.positioning_deceleration = config_.default_decel_puu;
  cmd_pos_puu_ = cmd.positioning_command_value;
  return publishCommand(cmd);
}

bool GantryEipRotaryAxis::isEnabled() const { return enabled_; }

bool GantryEipRotaryAxis::isServoActive() const {
    eip::hcs01::Hcs01PositioningActual fb;
    return image_.isOnline() && readFeedback(fb) &&
           fb.status.ready == eip::hcs01::ReadyForOperation::kInOperation;
}

int32_t GantryEipRotaryAxis::degToPuu(float deg) const {
  return static_cast<int32_t>(llround(static_cast<double>(deg) * config_.puu_per_deg));
}

float GantryEipRotaryAxis::puuToDeg(int32_t puu) const {
  if (config_.puu_per_deg <= 0.0) return 0.0f;
  return static_cast<float>(static_cast<double>(puu) / config_.puu_per_deg);
}

int32_t GantryEipRotaryAxis::feedbackAbsPuu() const {
  eip::hcs01::Hcs01PositioningActual fb;
  if (readFeedback(fb)) return fb.position_feedback;
  return cmd_pos_puu_;
}

bool GantryEipRotaryAxis::publishCommand(
    const eip::hcs01::Hcs01PositioningCommand& cmd) {
  image_.setCommand(cmd.serialize());
  return true;
}

bool GantryEipRotaryAxis::readFeedback(
    eip::hcs01::Hcs01PositioningActual& out) const {
  eip::Bytes fb;
  if (!image_.getFeedback(fb)) return false;
  return out.deserialize(fb);
}

bool GantryEipRotaryAxis::publishHold() {
  eip::hcs01::Hcs01PositioningCommand cmd;
  cmd.control = eip::hcs01::Hcs01ControlWord::makeDriveHalt();
  cmd.control.command_value_accept = command_accept_level_;
  cmd.positioning_command_value = feedbackAbsPuu();
  cmd.positioning_velocity = config_.default_velocity_puu;
  cmd.positioning_acceleration = config_.default_accel_puu;
  cmd.positioning_deceleration = config_.default_decel_puu;
  cmd_pos_puu_ = cmd.positioning_command_value;
  return publishCommand(cmd);
}

bool GantryEipRotaryAxis::publishClearFaults() {
  eip::hcs01::Hcs01PositioningCommand cmd;
  cmd.control = eip::hcs01::Hcs01ControlWord::makeDriveOff();
  cmd.control.clear_errors = true;
  cmd.control.command_value_accept = command_accept_level_;
  cmd.positioning_command_value = feedbackAbsPuu();
  cmd.positioning_velocity = config_.default_velocity_puu;
  cmd.positioning_acceleration = config_.default_accel_puu;
  cmd.positioning_deceleration = config_.default_decel_puu;
  cmd_pos_puu_ = cmd.positioning_command_value;
  return publishCommand(cmd);
}

void GantryEipRotaryAxis::logAfStatus(
    const char* why, const eip::hcs01::Hcs01PositioningActual& fb) const {
  ESP_LOGI(tag(),
           "[THETA] %s st=0x%04X ready=%u (%s) in_ref=%d not_follow=%d "
           "c1err=%d diag=0x%08lX",
           why, fb.status.encode(),
           static_cast<unsigned>(fb.status.ready),
           readyLabel(fb.status),
           fb.status.in_reference ? 1 : 0,
           fb.status.not_following_command ? 1 : 0,
           fb.status.class1_error ? 1 : 0,
           static_cast<unsigned long>(fb.diagnostic_message));
}

eip::hcs01::Hcs01PositioningCommand GantryEipRotaryAxis::buildMoveCommand(
    bool start_edge) const {
  eip::hcs01::Hcs01PositioningCommand cmd;
  cmd.control = start_edge ? eip::hcs01::Hcs01ControlWord::makeDriveEnable()
                           : eip::hcs01::Hcs01ControlWord::makeDriveHalt();
  cmd.control.command_value_accept = command_accept_level_;
  cmd.positioning_command_value = cmd_pos_puu_;
  cmd.positioning_velocity = cmd_vel_puu_;
  cmd.positioning_acceleration = cmd_acc_puu_;
  cmd.positioning_deceleration = cmd_dec_puu_;
  return cmd;
}

bool GantryEipRotaryAxis::publishMove(bool start_edge) {
  return publishCommand(buildMoveCommand(start_edge));
}

void GantryEipRotaryAxis::finishMoveHold() {
  motion_active_ = false;
  move_phase_ = MovePhase::kIdle;
  arrival_stable_us_ = 0;
  run_us_ = 0;
  (void)publishHold();
}

bool GantryEipRotaryAxis::moveToDeg(float target_deg, float speed_deg_per_s,
                                    float accel_deg_per_s2,
                                    float decel_deg_per_s2) {
    if (!initialized_ || !enabled_ || config_.puu_per_deg <= 0.0) {
        return false;
    }
    if (!image_.isOnline()) {
        return false;
    }

    if (target_deg < min_deg_) {
        target_deg = min_deg_;
    }
    if (target_deg > max_deg_) {
        target_deg = max_deg_;
    }
    target_deg_ = target_deg;
    start_deg_ = getCurrentDeg();
    stop_requested_ = false;
    stuck_log_div_ = 0;
    in_pos_armed_ = false;
    arrival_stable_us_ = 0;
    run_us_ = 0;
    arm_phase_ = ArmPhase::kHolding;
    arm_phase_us_ = 0;

    cmd_pos_puu_ = degToPuu(target_deg);
    cmd_vel_puu_ = speed_deg_per_s > 0.0f
                       ? hcs01VelocityUnits(speed_deg_per_s)
                       : config_.default_velocity_puu;
    cmd_acc_puu_ = accel_deg_per_s2 > 0.0f ? hcs01AccelUnits(accel_deg_per_s2)
                                           : config_.default_accel_puu;
    cmd_dec_puu_ = decel_deg_per_s2 > 0.0f ? hcs01AccelUnits(decel_deg_per_s2)
                                           : config_.default_decel_puu;
    const double eff_speed_deg_s =
        static_cast<double>(cmd_vel_puu_) / kHcs01VelocityUnitsPerDegS;
    move_timeout_us_ = thetaMoveTimeoutUs(target_deg - start_deg_, eff_speed_deg_s);

    // Preload S-0-0282 while AH (Drive Halt). Next ticks: halt 0→1 (AF),
    // then bit0 toggle. Combining those edges dropped the drive to Ab.
    // Phase before motion_active so a concurrent update cannot see kIdle+active
    // and treat sticky command_value_reached as arrival at the previous pose.
    move_phase_ = MovePhase::kPreload;
    move_phase_us_ = legacyTicksUs(kMovePreloadTicks);
    motion_active_ = true;
    ESP_LOGI(tag(),
             "[THETA] START current=%.2f target=%.2f cmd_puu=%ld vel=%ld acc=%ld dec=%ld "
             "timeout=%lu ms (scale=%.1f PUU/deg)",
             static_cast<double>(start_deg_), static_cast<double>(target_deg),
             static_cast<long>(cmd_pos_puu_), static_cast<long>(cmd_vel_puu_),
             static_cast<long>(cmd_acc_puu_), static_cast<long>(cmd_dec_puu_),
             static_cast<unsigned long>(move_timeout_us_ / 1000u), config_.puu_per_deg);
    eip::hcs01::Hcs01PositioningActual fb;
    if (readFeedback(fb)) {
        logAfStatus("move-start", fb);
        if (fb.status.ready != eip::hcs01::ReadyForOperation::kInOperation) {
            ESP_LOGW(tag(),
                     "[THETA] not AH/AF (ready=%u, want 3) — Drive ON not accepted",
                     static_cast<unsigned>(fb.status.ready));
        }
    }
    return publishMove(/*start_edge=*/false);
}

bool GantryEipRotaryAxis::stopMotion() {
  stop_requested_ = true;
  motion_active_ = false;
  move_phase_ = MovePhase::kIdle;
  eip::hcs01::Hcs01PositioningCommand cmd;
  cmd.control = eip::hcs01::Hcs01ControlWord::makeDriveHalt();
  cmd.control.command_value_accept = command_accept_level_;
  cmd.positioning_command_value = feedbackAbsPuu();
  cmd.positioning_velocity = cmd_vel_puu_ != 0 ? cmd_vel_puu_
                                               : config_.default_velocity_puu;
  cmd.positioning_acceleration = cmd_acc_puu_ != 0 ? cmd_acc_puu_
                                                   : config_.default_accel_puu;
  cmd.positioning_deceleration = cmd_dec_puu_ != 0 ? cmd_dec_puu_
                                                   : config_.default_decel_puu;
  return publishCommand(cmd);
}

float GantryEipRotaryAxis::getCurrentDeg() const {
  eip::hcs01::Hcs01PositioningActual fb;
  if (!readFeedback(fb)) return 0.0f;
  return puuToDeg(fb.position_feedback);
}

float GantryEipRotaryAxis::getTargetDeg() const { return target_deg_; }

bool GantryEipRotaryAxis::isBusy() const {
  // Sticky P-0-4078 bit4 (command_value_reached) stays 1 after the previous
  // target. Consecutive in-range moves must stay busy until the NEW pose
  // arrives — same pattern as GantryEipLinearAxis::motion_commanded_.
  return motion_active_ && !stop_requested_;
}

bool GantryEipRotaryAxis::isMotionActive() const { return isBusy(); }

bool GantryEipRotaryAxis::isAlarmActive() const {
  eip::hcs01::Hcs01PositioningActual fb;
  if (!readFeedback(fb)) return false;
  if (fb.status.class1_error) return true;
  const uint32_t code = fb.diagnostic_message & 0xFFFFu;
  return code == 0x4009u || code == 0x4005u;
}

bool GantryEipRotaryAxis::clearAlarm() {
  motion_active_ = false;
  move_phase_ = MovePhase::kIdle;
  move_phase_us_ = 0;
  in_pos_armed_ = false;
  arrival_stable_us_ = 0;
  run_us_ = 0;
  arm_phase_ = ArmPhase::kClearFaults;
  arm_phase_us_ = legacyTicksUs(kClearFaultTicks);
  return publishClearFaults();
}

bool GantryEipRotaryAxis::getDriveAlarmSummary(char* buf, size_t n) const {
  if (buf == nullptr || n == 0) return false;
  eip::hcs01::Hcs01PositioningActual fb;
  if (!readFeedback(fb)) {
    std::snprintf(buf, n, "no T->O (CIP .23 down)");
    return false;
  }
  const uint32_t diag = fb.diagnostic_message;
  const uint32_t code = diag & 0xFFFFu;
  const char* name = "";
  if (code == 0x2057u) name = " F2057";
  else if (code == 0x4009u) name = " F4009";
  else if (code == 0x4005u) name = " E4005";
  else if (code == 0x2174u) name = " F2174";
  else if (code == 0x2028u) name = " F2028";
  if (!fb.status.class1_error && name[0] == '\0' &&
      code != 0x4009u && code != 0x4005u) {
    std::snprintf(buf, n, "clear");
    return true;
  }
  std::snprintf(buf, n, "c1err=%d diag=0x%08lX%s",
                fb.status.class1_error ? 1 : 0,
                static_cast<unsigned long>(diag), name);
  return true;
}

bool GantryEipRotaryAxis::formatCipStatus(char* buf, size_t n) const {
  if (buf == nullptr || n == 0) return false;
  eip::hcs01::Hcs01PositioningActual fb;
  if (!readFeedback(fb)) {
    std::snprintf(buf, n,
                  "T->O STALE (CIP .23 Class 1 down) — HTTP c0500 if faulted");
    return true;
  }
  std::snprintf(buf, n,
                "T->O live in_ref=%d ready=%s c1err=%d diag=0x%08lX",
                fb.status.in_reference ? 1 : 0, readyLabel(fb.status),
                fb.status.class1_error ? 1 : 0,
                static_cast<unsigned long>(fb.diagnostic_message));
  return true;
}

double GantryEipRotaryAxis::pulsesPerDeg() const { return config_.puu_per_deg; }

bool GantryEipRotaryAxis::setPuuPerDeg(double puu_per_deg) {
  if (puu_per_deg <= 0.0) return false;
  config_.puu_per_deg = puu_per_deg;
  return true;
}

bool GantryEipRotaryAxis::captureSoftHome() {
  eip::hcs01::Hcs01PositioningActual fb;
  if (!readFeedback(fb)) return false;
  
  const float abs_deg = puuToDeg(fb.position_feedback);
  
  // Permanent physical alignment: joint 0 is drive 0.
  // We assume the drive has been zeroed (C0300) so abs_deg=0 is the desired mechanical zero.
  // If it hasn't, the user is still commanding relative to drive_abs=0.
  // Because the end-effector uses a pass-through, we don't need to prevent wrapping.
  origin_aligned_ = true;
  
  // The drive limit is typically configured for +/- 36000 in this continuous mode.
  envelope_min_deg_ = -36000.0f;
  envelope_max_deg_ = 36000.0f;
  min_deg_ = envelope_min_deg_;
  max_deg_ = envelope_max_deg_;
  
  target_deg_ = abs_deg; // Will be immediately overwritten by homeTheta() calling moveToTheta(0.0)
  motion_active_ = false;
  stop_requested_ = false;
  move_phase_ = MovePhase::kIdle;
  in_pos_armed_ = false;
  arrival_stable_us_ = 0;
  run_us_ = 0;
  
  ESP_LOGI(tag(),
           "[THETA] origin ALIGNED (continuous slip-ring). drive_abs=%.3f joint_lim=%.2f..%.2f",
           static_cast<double>(abs_deg),
           static_cast<double>(min_deg_), static_cast<double>(max_deg_));
  return true;
}

float GantryEipRotaryAxis::getDriveAbsDeg() const {
  return puuToDeg(feedbackAbsPuu());
}

bool GantryEipRotaryAxis::isDriveOriginAligned() const {
  return origin_aligned_;
}

bool GantryEipRotaryAxis::hasLiveFeedback() const {
  eip::hcs01::Hcs01PositioningActual fb;
  return readFeedback(fb);
}

void GantryEipRotaryAxis::update() {
  if (image_.feedbackFresh()) {
    image_.consumeFeedbackFresh();
  }

  if (!image_.isOnline()) {
    if (motion_active_ || arm_phase_ != ArmPhase::kIdle) {
      const char* tag = (log_tag_ && log_tag_[0]) ? log_tag_ : "EipRotary";
      ESP_LOGW(tag, "EIP connection offline — aborting motion and disarming");
      motion_active_ = false;
      stop_requested_ = false;
      move_phase_ = MovePhase::kIdle;
      arm_phase_ = ArmPhase::kIdle;
    }
    return;
  }

  if (arm_phase_ == ArmPhase::kClearFaults) {
    (void)publishClearFaults();
    if (arm_phase_us_ > 0) spendUs(arm_phase_us_);
    if (arm_phase_us_ == 0) {
      if (!enabled_) {
        arm_phase_ = ArmPhase::kIdle;
        eip::hcs01::Hcs01PositioningCommand cmd;
        cmd.control = eip::hcs01::Hcs01ControlWord::makeDriveOff();
        cmd.control.command_value_accept = command_accept_level_;
        cmd.positioning_command_value = feedbackAbsPuu();
        cmd.positioning_velocity = config_.default_velocity_puu;
        cmd.positioning_acceleration = config_.default_accel_puu;
        cmd.positioning_deceleration = config_.default_decel_puu;
        cmd_pos_puu_ = cmd.positioning_command_value;
        (void)publishCommand(cmd);
        return;
      }
      arm_phase_ = ArmPhase::kWaitAf;
      arm_phase_us_ = legacyTicksUs(kWaitAfTicks);
      (void)publishHold();
    }
    return;
  }

  if (arm_phase_ == ArmPhase::kWaitAf) {
    (void)publishHold();
    if (arm_phase_us_ > 0) spendUs(arm_phase_us_);
    eip::hcs01::Hcs01PositioningActual fb;
    const bool have_fb = readFeedback(fb);
    if (have_fb && fb.status.ready == eip::hcs01::ReadyForOperation::kInOperation) {
      logAfStatus("Servo arm complete", fb);
      arm_phase_ = ArmPhase::kHolding;
      arm_phase_us_ = 0;
      return;
    }
    if (arm_phase_us_ == 0) {
      if (have_fb) {
        logAfStatus("AH timeout — still not ready=3", fb);
      } else {
        ESP_LOGW(tag(), "[THETA] AH timeout — no T->O feedback");
      }
      arm_phase_ = ArmPhase::kHolding;
    }
    return;
  }

  if (move_phase_ == MovePhase::kPreload) {
    if (move_phase_us_ > 0) spendUs(move_phase_us_);
    (void)publishMove(/*start_edge=*/false);
    if (move_phase_us_ == 0) {
      move_phase_ = MovePhase::kReleaseHalt;
      move_phase_us_ = legacyTicksUs(kMoveReleaseHaltTicks);
      (void)publishMove(/*start_edge=*/true);
    }
    return;
  }

  if (move_phase_ == MovePhase::kReleaseHalt) {
    (void)publishMove(/*start_edge=*/true);
    if (move_phase_us_ > 0) spendUs(move_phase_us_);
    bool following = false;
    eip::hcs01::Hcs01PositioningActual fb;
    if (readFeedback(fb)) {
      following = !fb.status.not_following_command &&
                  fb.status.ready == eip::hcs01::ReadyForOperation::kInOperation;
    }
    if (following || move_phase_us_ == 0) {
      if (!following) {
        ESP_LOGW(tag(), "[THETA] release-halt timeout waiting for AF");
      }
      move_phase_ = MovePhase::kStart;
      move_phase_us_ = legacyTicksUs(kMoveStartTicks);
      command_accept_level_ = !command_accept_level_;
      (void)publishMove(/*start_edge=*/true);
    }
    return;
  }

  if (move_phase_ == MovePhase::kStart) {
    (void)publishMove(/*start_edge=*/true);
    if (move_phase_us_ > 0) spendUs(move_phase_us_);
    bool acked = false;
    eip::hcs01::Hcs01PositioningActual fb;
    if (readFeedback(fb)) {
      acked = (fb.status.command_value_ack == command_accept_level_);
    }
    if (acked || move_phase_us_ == 0) {
      move_phase_ = MovePhase::kRun;
      run_us_ = 0;
      arrival_stable_us_ = 0;
    }
    return;
  }

  if (!motion_active_ || stop_requested_ || move_phase_ != MovePhase::kRun) {
    return;
  }

  (void)publishMove(/*start_edge=*/true);

  eip::hcs01::Hcs01PositioningActual fb;
  if (!readFeedback(fb)) return;
  const float now_deg = puuToDeg(fb.position_feedback);
  if (axisLogDue(log_rate_hz_, esp_timer_get_time(), &last_axislog_us_)) {
    ESP_LOGI(tag(),
             "[THETA] MOVE joint=%.3f drive=%.3f target=%.3f st=0x%04X "
             "ready=%s c1err=%d",
             static_cast<double>(now_deg),
             static_cast<double>(puuToDeg(fb.position_feedback)),
             static_cast<double>(target_deg_), fb.status.encode(),
             readyLabel(fb.status), fb.status.class1_error ? 1 : 0);
  }

  // Bit4 stays 1 at the previous target until the new S-0-0282 is accepted.
  // Arm only after the drive leaves that in-pos (bit4 fall or motion).
  if (!fb.status.command_value_reached ||
      std::fabs(now_deg - start_deg_) > kArrivalEpsDeg) {
    in_pos_armed_ = true;
  }
  const bool at_target = std::fabs(now_deg - target_deg_) <= kArrivalEpsDeg &&
                         (fb.status.command_value_reached ||
                          fb.status.in_standstill);
  if (in_pos_armed_ && at_target) {
    accrueUs(arrival_stable_us_, legacyTicksUs(kArrivalStableTicks));
    if (arrival_stable_us_ >= legacyTicksUs(kArrivalStableTicks)) {
      ESP_LOGI(tag(), "[THETA] END current=%.3f", static_cast<double>(now_deg));
      finishMoveHold();
      return;
    }
  } else {
    arrival_stable_us_ = 0;
  }

  accrueUs(run_us_, move_timeout_us_);
  if (run_us_ >= move_timeout_us_) {
    ESP_LOGW(tag(),
             "[THETA] move timeout (current=%.3f target=%.3f) — halt",
             static_cast<double>(now_deg), static_cast<double>(target_deg_));
    finishMoveHold();
    return;
  }

  ++stuck_log_div_;
  if (stuck_log_div_ >= 100) {
    stuck_log_div_ = 0;
    ESP_LOGW(tag(),
             "[THETA] waiting in-pos (current=%.3f target=%.3f st=0x%04X "
             "reached=%d standstill=%d not_follow=%d ready=%u (%s) "
             "diag=0x%08lX)",
             static_cast<double>(now_deg),
             static_cast<double>(target_deg_),
             fb.status.encode(), fb.status.command_value_reached ? 1 : 0,
             fb.status.in_standstill ? 1 : 0,
             fb.status.not_following_command ? 1 : 0,
             static_cast<unsigned>(fb.status.ready),
             readyLabel(fb.status),
             static_cast<unsigned long>(fb.diagnostic_message));
  }
}

void GantryEipRotaryAxis::setAngleRange(float min_deg, float max_deg) {
  if (min_deg >= max_deg) {
    return;
  }
  if (min_deg < envelope_min_deg_) {
    min_deg = envelope_min_deg_;
  }
  if (max_deg > envelope_max_deg_) {
    max_deg = envelope_max_deg_;
  }
  if (min_deg < max_deg) {
    min_deg_ = min_deg;
    max_deg_ = max_deg;
  }
}

void GantryEipRotaryAxis::setLogTag(const char* tag) {
  log_tag_ = (tag != nullptr) ? tag : "";
}

void GantryEipRotaryAxis::setLogRateHz(uint32_t hz) {
  log_rate_hz_ = hz;
  last_axislog_us_ = 0;
}

DrivePositionRef GantryEipRotaryAxis::getDrivePositionRef() const {
  if (!image_.isOnline()) {
    return DrivePositionRef::kUnknown;
  }
  eip::hcs01::Hcs01PositioningActual fb;
  if (!readFeedback(fb)) {
    return DrivePositionRef::kUnknown;
  }
  return fb.status.in_reference ? DrivePositionRef::kHomed
                                : DrivePositionRef::kLost;
}

}  // namespace Gantry

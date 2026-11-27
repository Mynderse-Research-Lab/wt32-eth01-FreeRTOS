/**
 * @file GantryEipLinearAxis.cpp
 * @brief Position Absolute PTP over Kinetix 5100 assemblies 104/154.
 *
 * See lib/Gantry/REFERENCE.md and tools/eip_position_abs.py.
 */

#include "GantryEipLinearAxis.h"
#include "GantryEipLinearAxisInternal.h"

#include "EipClass1TimingStats.h"
#include "GantryControlPeriod.h"
#include "GantryLinearPhases.h"
#include "GantryUtils.h"
#include "Kinetix5100Assembly.h"
#include "KinetixFaultCodes.h"

#include <cmath>
#include <cstdio>


namespace Gantry {


GantryEipLinearAxis::GantryEipLinearAxis(eip::EipProcessImage& image,
                                         const EipLinearAxisConfig& cfg)
    : image_(image),
      config_(cfg),
      initialized_(false),
      enabled_(false),
      stop_requested_(false),
      target_mm_(0.0f),
      zero_puu_(0),
      command_position_puu_(0),
      log_tag_(""),
      log_rate_hz_(0),
      last_axislog_us_(0),
      last_fault_latched_(false),
      last_warning_latched_(false),
      last_fault_code_(0),
      last_warning_code_(0),
      motion_commanded_(false),
      arm_phase_(ArmPhase::kIdle),
      arm_phase_us_(0),
      a603_clear_attempts_(0),
      home_wait_us_(0),
      move_phase_(MovePhase::kIdle),
      move_phase_us_(0),
      fault_reset_pulse_us_(0),
      last_speed_mm_s_(50.0f),
      last_accel_mm_s2_(0.0f),
      last_decel_mm_s2_(0.0f),
      move_speed_ref_(0),
      run_us_(0),
      stop_stable_us_(0),
      arrival_stable_us_(0),
      last_stop_sample_puu_(0),
      stop_sample_valid_(false),
      limit_min_sw_(nullptr),
      limit_max_sw_(nullptr),
      ignore_a014_abort_(false),
      ignore_a015_abort_(false) {}

bool GantryEipLinearAxis::begin() {
  initialized_ = true;
  eip::k5100::OutputAssembly104 idle;
  idle.servo_on = false;
  idle.travel_mode = 10;
  idle.torque_ramp_time = 1000;
  return publishCommand(idle);
}

bool GantryEipLinearAxis::enable() {
  if (!initialized_) return false;
  enabled_ = true;
  stop_requested_ = false;
  motion_commanded_ = false;
  move_phase_ = MovePhase::kIdle;
  move_phase_us_ = 0;

  eip::k5100::InputAssembly154 fb;
  if (readFeedback(fb)) {
    command_position_puu_ = fb.actual_position;
    target_mm_ = puuToMm(toJointPuu(command_position_puu_));
  }

  arm_phase_ = ArmPhase::kServoLow;
  arm_phase_us_ = Gantry::legacyTicksUs(kArmServoLowTicks);
  a603_clear_attempts_ = 0;
  return publishCommand(buildSettleCommand(/*servo_on=*/false));
}

bool GantryEipLinearAxis::disable() {
  enabled_ = false;
  stop_requested_ = false;
  motion_commanded_ = false;
  move_phase_ = MovePhase::kIdle;
  move_phase_us_ = 0;
  arm_phase_ = ArmPhase::kIdle;
  arm_phase_us_ = 0;
  eip::k5100::OutputAssembly104 cmd;
  cmd.servo_off = true;
  cmd.travel_mode = 10;
  cmd.torque_ramp_time = 1000;
  return publishCommand(cmd);
}

bool GantryEipLinearAxis::isEnabled() const { return enabled_; }

bool GantryEipLinearAxis::isServoActive() const {
    eip::k5100::InputAssembly154 fb;
    return image_.isOnline() && readFeedback(fb) && fb.active;
}

int32_t GantryEipLinearAxis::mmToPuu(float mm) const {
  return static_cast<int32_t>(
      llround(static_cast<double>(mm) * signedPuuPerMm()));
}

float GantryEipLinearAxis::puuToMm(int32_t puu) const {
  const double spp = signedPuuPerMm();
  if (std::fabs(spp) < 1e-9) return 0.0f;
  return static_cast<float>(static_cast<double>(puu) / spp);
}

double GantryEipLinearAxis::signedPuuPerMm() const {
  return config_.invert_direction ? -config_.puu_per_mm : config_.puu_per_mm;
}

double GantryEipLinearAxis::absPuuPerMm() const {
  return std::fabs(config_.puu_per_mm);
}

int32_t GantryEipLinearAxis::toJointPuu(int32_t abs_puu) const {
  return abs_puu - zero_puu_;
}

int32_t GantryEipLinearAxis::toAbsPuu(int32_t joint_puu) const {
  return joint_puu + zero_puu_;
}

bool GantryEipLinearAxis::publishCommand(const eip::k5100::OutputAssembly104& cmd) {
  image_.setCommand(cmd.serialize());
  return true;
}

bool GantryEipLinearAxis::readFeedback(eip::k5100::InputAssembly154& out) const {
  eip::Bytes fb;
  if (!image_.getFeedback(fb)) return false;
  return out.deserialize(fb);
}

bool GantryEipLinearAxis::moveToMm(float target_mm, float speed_mm_per_s,
                                   float accel_mm_per_s2, float decel_mm_per_s2) {
  return startAbsoluteMm(target_mm, speed_mm_per_s, accel_mm_per_s2, decel_mm_per_s2,
                         false);
}

bool GantryEipLinearAxis::moveToMmHeld(float target_mm, float speed_mm_per_s,
                                      float accel_mm_per_s2, float decel_mm_per_s2) {
  return startAbsoluteMm(target_mm, speed_mm_per_s, accel_mm_per_s2, decel_mm_per_s2,
                         true);
}

bool GantryEipLinearAxis::releaseStartMotion() {
  if (move_phase_ == MovePhase::kRun && queued_ == QueuedMove::kReady) {
    // Edge for the queued Absolute. Override and overlap stay 0, so the drive
    // runs it after the move in flight completes (2198-UM004D, Consecutive
    // Position Commands) instead of replacing or blending it.
    if (eip::class1NowUs() - queued_at_us_ < kQueueToEdgeUs) {
      return false;
    }
    queued_ = QueuedMove::kNone;
  } else if (move_phase_ != MovePhase::kPreloadHold) {
    return false;
  }
  move_phase_ = MovePhase::kStart;
  move_phase_us_ = Gantry::legacyTicksUs(kMoveStartMotionTicks);
  eip::class1TimingStats().noteAbsoluteStartMotionPublished(eip::class1NowUs());
  republishMoveCommand(/*start_motion_edge=*/true);
  return true;
}

bool GantryEipLinearAxis::isStartHeld() const {
  if (move_phase_ == MovePhase::kPreloadHold) {
    return true;
  }
  return queued_ != QueuedMove::kNone &&
         (move_phase_ == MovePhase::kStart || move_phase_ == MovePhase::kRun);
}

bool GantryEipLinearAxis::queueHeldMm(float target_mm, float speed_mm_per_s,
                                      float accel_mm_per_s2, float decel_mm_per_s2) {
  if (!enabled_ || stop_requested_ || queued_ != QueuedMove::kNone ||
      absPuuPerMm() <= 0.0) {
    return false;
  }
  if (move_phase_ != MovePhase::kStart && move_phase_ != MovePhase::kRun) {
    return false;
  }
  queued_target_mm_ = target_mm;
  queued_speed_mm_s_ = speed_mm_per_s > 0.0f ? speed_mm_per_s : 50.0f;
  queued_accel_mm_s2_ = accel_mm_per_s2;
  queued_decel_mm_s2_ = decel_mm_per_s2 > 0.0f ? decel_mm_per_s2 : accel_mm_per_s2;
  queued_ = QueuedMove::kPending;
  // During kStart the current edge is still high; the swap waits for kRun.
  if (move_phase_ == MovePhase::kRun) {
    applyQueuedMove();
  }
  return true;
}

void GantryEipLinearAxis::applyQueuedMove() {
  noteMoveDirection(getCurrentMm(), queued_target_mm_);
  target_mm_ = queued_target_mm_;
  command_position_puu_ = toAbsPuu(mmToPuu(queued_target_mm_));
  last_speed_mm_s_ = queued_speed_mm_s_;
  last_accel_mm_s2_ = queued_accel_mm_s2_;
  last_decel_mm_s2_ = queued_decel_mm_s2_;
  move_speed_ref_ = static_cast<int32_t>(llround(
      static_cast<double>(last_speed_mm_s_) * config_.speed_ref_per_mm_s));
  if (move_speed_ref_ < 1) move_speed_ref_ = 1;
  // Arrival now means the queued target; the move in flight runs on in the drive.
  arrival_stable_us_ = 0;
  queued_ = QueuedMove::kReady;
  queued_at_us_ = eip::class1NowUs();
  republishMoveCommand(/*start_motion_edge=*/false);
}

// A drive allows motion away from an active limit switch (Kinetix 5100
// 2198-UM004D, ch. 15, DI functions 0x22/0x23), but it raises the A014 or
// A015 warning only once the axis moves. A move that starts on a switch
// would otherwise see that warning as a fresh trip and abort (G-A13), so a
// switch behind the move never aborts it. Which switch sits at which joint
// end follows joint_min_warning_a015_ (X: A014 = min; Z: A015 = min).
void GantryEipLinearAxis::noteMoveDirection(float from_mm, float to_mm) {
    const bool toward_max = to_mm > from_mm;
    const bool toward_min = to_mm < from_mm;
    const bool a014_at_min = !joint_min_warning_a015_;
    away_from_a014_ = a014_at_min ? toward_max : toward_min;
    away_from_a015_ = a014_at_min ? toward_min : toward_max;
}

bool GantryEipLinearAxis::startAbsoluteMm(float target_mm, float speed_mm_per_s,
                                          float accel_mm_per_s2, float decel_mm_per_s2,
                                          bool hold_start) {
  if (!initialized_ || !enabled_ || absPuuPerMm() <= 0.0) return false;
  if (!image_.isOnline()) return false;
  if (arm_phase_ != ArmPhase::kIdle && arm_phase_ != ArmPhase::kHolding) {
    const char* tag = (log_tag_ && log_tag_[0]) ? log_tag_ : kEipAxisTag;
    ESP_LOGW(tag, "move rejected: servo still arming/homing (wait for Active)");
    return false;
  }

  eip::k5100::InputAssembly154 fb;
  if (!readFeedback(fb)) return false;
  if (!fb.active || !fb.ready || fb.fault || fb.connection_faulted ||
      fb.isLikelyLimitStop()) {
    const char* tag = (log_tag_ && log_tag_[0]) ? log_tag_ : kEipAxisTag;
    ESP_LOGW(tag,
             "move rejected: active=%d ready=%d fault=%d limit=%d",
             (int)fb.active, (int)fb.ready, (int)fb.fault,
             (int)fb.isLikelyLimitStop());
    return false;
  }

  // Escape/creep: if already on a limit warning, suppress abort for that code
  // until it clears (precision home/cal clear-edge).
  ignore_a014_abort_ =
      fb.warning_present && eip::k5100::isWarningA014(fb.warning_code);
  ignore_a015_abort_ =
      fb.warning_present && eip::k5100::isWarningA015(fb.warning_code);

  target_mm_ = target_mm;
  command_position_puu_ = toAbsPuu(mmToPuu(target_mm));
  stop_requested_ = false;
  last_speed_mm_s_ = speed_mm_per_s > 0.0f ? speed_mm_per_s : 50.0f;
  last_accel_mm_s2_ = accel_mm_per_s2;
  last_decel_mm_s2_ = decel_mm_per_s2 > 0.0f ? decel_mm_per_s2 : accel_mm_per_s2;

  move_speed_ref_ = static_cast<int32_t>(llround(
      static_cast<double>(last_speed_mm_s_) * config_.speed_ref_per_mm_s));
  if (move_speed_ref_ < 1) move_speed_ref_ = 1;

  const float here = getCurrentMm();
  noteMoveDirection(here, target_mm);
  if (std::fabs(here - target_mm) <= 0.05f) {
    motion_commanded_ = false;
    move_phase_ = MovePhase::kIdle;
    return publishCommand(buildHoldCommand());
  }

  motion_commanded_ = true;
  queued_ = QueuedMove::kNone;
  // Preload the new Absolute with StartMotion=0; the rising edge is kStart.
  // kPreloadHold waits for releaseStartMotion() so a strike can fire that edge
  // on a timer instead of after a fixed tick count.
  move_phase_ = hold_start ? MovePhase::kPreloadHold : MovePhase::kPreload;
  move_phase_us_ = hold_start ? 0 : Gantry::legacyTicksUs(kMovePreloadTicks);
  run_us_ = 0;
  arrival_stable_us_ = 0;
  return publishCommand(buildMoveCommand(/*start_motion_edge=*/false));
}

bool GantryEipLinearAxis::moveRelativeMm(float delta_mm, float speed_mm_per_s,
                                         float accel_mm_per_s2,
                                         float decel_mm_per_s2) {
  return moveToMm(getCurrentMm() + delta_mm, speed_mm_per_s, accel_mm_per_s2,
                  decel_mm_per_s2);
}

bool GantryEipLinearAxis::stopMotion() {
  stop_requested_ = true;
  eip::k5100::InputAssembly154 fb;
  if (readFeedback(fb)) {
    command_position_puu_ = fb.actual_position;
    target_mm_ = puuToMm(toJointPuu(command_position_puu_));
  }
  motion_commanded_ = true;
  enterAbortStop();
  return true;
}

float GantryEipLinearAxis::getCurrentMm() const {
  eip::k5100::InputAssembly154 fb;
  if (!readFeedback(fb)) return 0.0f;
  return puuToMm(toJointPuu(fb.actual_position));
}

float GantryEipLinearAxis::getTargetMm() const { return target_mm_; }

bool GantryEipLinearAxis::isBusy() const { return motion_commanded_; }

bool GantryEipLinearAxis::moveToPulses(uint32_t target_pulses, uint32_t speed_pps,
                                       uint32_t accel_pps2, uint32_t decel_pps2) {
  if (absPuuPerMm() <= 0.0) return false;
  const float target_mm =
      static_cast<float>(static_cast<double>(target_pulses) / absPuuPerMm());
  const float speed_mm_s =
      static_cast<float>(static_cast<double>(speed_pps) / absPuuPerMm());
  const float accel_mm_s2 =
      static_cast<float>(static_cast<double>(accel_pps2) / absPuuPerMm());
  const float decel_mm_s2 =
      static_cast<float>(static_cast<double>(decel_pps2) / absPuuPerMm());
  return moveToMm(target_mm, speed_mm_s, accel_mm_s2, decel_mm_s2);
}

uint32_t GantryEipLinearAxis::getCurrentPulses() const {
  eip::k5100::InputAssembly154 fb;
  if (!readFeedback(fb)) return 0;
  return static_cast<uint32_t>(toJointPuu(fb.actual_position));
}

int32_t GantryEipLinearAxis::getEncoderPulses() const {
  return static_cast<int32_t>(getCurrentPulses());
}

void GantryEipLinearAxis::setCurrentPulses(uint32_t pos) {
  const int32_t joint = static_cast<int32_t>(pos);
  eip::k5100::InputAssembly154 fb;
  if (readFeedback(fb)) {
    zero_puu_ = fb.actual_position - joint;
    command_position_puu_ = fb.actual_position;
  } else {
    zero_puu_ = command_position_puu_ - joint;
    command_position_puu_ = toAbsPuu(joint);
  }
  target_mm_ = puuToMm(joint);
}

bool GantryEipLinearAxis::isMotionActive() const { return isBusy(); }

double GantryEipLinearAxis::pulsesPerMm() const { return absPuuPerMm(); }

bool GantryEipLinearAxis::isEncoderFeedbackEnabled() const { return true; }

void GantryEipLinearAxis::update() {
  if (image_.feedbackFresh()) {
    image_.consumeFeedbackFresh();
  }
  
  if (!image_.isOnline()) {
    if (motion_commanded_ || arm_phase_ != ArmPhase::kIdle) {
      const char* tag = (log_tag_ && log_tag_[0]) ? log_tag_ : kEipAxisTag;
      ESP_LOGW(tag, "EIP connection offline — aborting motion and disarming");
      motion_commanded_ = false;
      stop_requested_ = false;
      move_phase_ = MovePhase::kIdle;
      arm_phase_ = ArmPhase::kIdle;
    }
    return;
  }

  eip::k5100::InputAssembly154 fb;
  if (!readFeedback(fb)) return;
  logAlarmEdge(fb);

  // Drive-managed endstops mirrored onto soft switches.
  // Default (X): A014/PL -> joint min, A015/NL -> joint max.
  // Z (joint_min_warning_a015_): A015 -> min, A014 -> max.
  const bool a014 =
      fb.warning_present && eip::k5100::isWarningA014(fb.warning_code);
  const bool a015 =
      fb.warning_present && eip::k5100::isWarningA015(fb.warning_code);
  const bool overtravel = fb.isLikelyLimitStop() || a014 || a015;
  const bool at_min = joint_min_warning_a015_ ? a015 : a014;
  const bool at_max = joint_min_warning_a015_ ? a014 : a015;
  if (limit_min_sw_ != nullptr) {
    limit_min_sw_->setExternalActive(at_min || (overtravel && !at_max));
  }
  if (limit_max_sw_ != nullptr) {
    limit_max_sw_->setExternalActive(at_max || (overtravel && !at_min));
  }

  advanceArmPhase();

  if (fault_reset_pulse_us_ > 0) {
    Gantry::spendUs(fault_reset_pulse_us_);
    if (fault_reset_pulse_us_ == 0) {
      eip::k5100::OutputAssembly104 cmd = buildSettleCommand(enabled_);
      cmd.fault_reset = false;
      (void)publishCommand(cmd);
    }
  }

  if (motion_commanded_) {
    advanceMovePhase(fb);
    if (motion_commanded_ &&
        axisLogDue(log_rate_hz_, esp_timer_get_time(), &last_axislog_us_)) {
      const char* tag = (log_tag_ && log_tag_[0]) ? log_tag_ : kEipAxisTag;
      ESP_LOGI(tag, "MOVE mm=%.3f target=%.3f", (double)getCurrentMm(),
               (double)target_mm_);
    }
  }
}

void GantryEipLinearAxis::attachLimitSwitches(GantryLimitSwitch* min_sw,
                                              GantryLimitSwitch* max_sw) {
  limit_min_sw_ = min_sw;
  limit_max_sw_ = max_sw;
}

void GantryEipLinearAxis::setJointMinWarningA015(bool enable) {
  joint_min_warning_a015_ = enable;
}

uint32_t GantryEipLinearAxis::homingSpeedPps() const {
  constexpr double kSafeHomingRpm = 120.0;
  constexpr double kRpmToRps = 1.0 / 60.0;
  const double linearSpeedMmPerS =
      config_.lead_mm_per_rev * kSafeHomingRpm * kRpmToRps;
  return static_cast<uint32_t>(linearSpeedMmPerS * absPuuPerMm());
}

void GantryEipLinearAxis::setLogTag(const char* tag) { log_tag_ = tag ? tag : ""; }

void GantryEipLinearAxis::setLogRateHz(uint32_t hz) {
  log_rate_hz_ = hz;
  last_axislog_us_ = 0;
}

DrivePositionRef GantryEipLinearAxis::getDrivePositionRef() const {
  if (!image_.isOnline()) {
    return DrivePositionRef::kUnknown;
  }
  eip::k5100::InputAssembly154 fb{};
  if (!readFeedback(fb)) {
    return DrivePositionRef::kUnknown;
  }
  return fb.homed_status ? DrivePositionRef::kHomed : DrivePositionRef::kLost;
}

}  // namespace Gantry

/**
 * @file GantryEipRotaryAxis.cpp
 */

#include "GantryEipRotaryAxis.h"

#include "axis_drivetrain_params.h"
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
constexpr uint8_t kMovePreloadTicks = 4;     // ~40 ms @ 100 Hz — latch S-0-0282 while AH
constexpr uint8_t kMoveReleaseHaltTicks = 4;  // bit13 0→1 (AH→AF) before bit0
constexpr uint8_t kMoveStartTicks = 4;
constexpr uint8_t kClearFaultTicks = 10;     // hold P-0-4077 bit5 (C0500)
constexpr uint16_t kWaitAfTicks = 300;       // 3 s @ 100 Hz

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
      zero_puu_(0),
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
      move_phase_ticks_(0),
      arm_phase_ticks_(0),
      cmd_pos_puu_(0),
      cmd_vel_puu_(0),
      cmd_acc_puu_(0),
      cmd_dec_puu_(0),
      stuck_log_div_(0) {}

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
  move_phase_ticks_ = 0;
  // F4009 latches when bits 13/14/15 fall together. Pulse C0500 (bit5)
  // before Drive ON. Armed idle is AH (ready=3 + Drive Halt), not AF.
  arm_phase_ = ArmPhase::kClearFaults;
  arm_phase_ticks_ = kClearFaultTicks;
  return publishClearFaults();
}

bool GantryEipRotaryAxis::disable() {
  enabled_ = false;
  stop_requested_ = false;
  motion_active_ = false;
  move_phase_ = MovePhase::kIdle;
  arm_phase_ = ArmPhase::kIdle;
  arm_phase_ticks_ = 0;
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

bool GantryEipRotaryAxis::moveToDeg(float target_deg, float speed_deg_per_s,
                                    float accel_deg_per_s2,
                                    float decel_deg_per_s2) {
  if (!initialized_ || !enabled_ || config_.puu_per_deg <= 0.0) return false;
  if (!image_.isOnline()) return false;

  if (target_deg < min_deg_) target_deg = min_deg_;
  if (target_deg > max_deg_) target_deg = max_deg_;
  target_deg_ = target_deg;
  stop_requested_ = false;
  motion_active_ = true;
  stuck_log_div_ = 0;
  arm_phase_ = ArmPhase::kHolding;
  arm_phase_ticks_ = 0;

  // S-0-0282 is configured for -36000..+36000 deg (non-modulo scaling
  // type 0x0002). Preserve the continuous absolute frame; wrapping a
  // soft-home-relative target at +/-180 could command the long way around.
  cmd_pos_puu_ = zero_puu_ + degToPuu(target_deg);
  if (speed_deg_per_s > 0.0f) {
    cmd_vel_puu_ =
        static_cast<int32_t>(llround(speed_deg_per_s * config_.puu_per_deg));
  } else {
    cmd_vel_puu_ = config_.default_velocity_puu;
  }
  if (accel_deg_per_s2 > 0.0f) {
    cmd_acc_puu_ =
        static_cast<int32_t>(llround(accel_deg_per_s2 * config_.puu_per_deg));
  } else {
    cmd_acc_puu_ = config_.default_accel_puu;
  }
  if (decel_deg_per_s2 > 0.0f) {
    cmd_dec_puu_ =
        static_cast<int32_t>(llround(decel_deg_per_s2 * config_.puu_per_deg));
  } else {
    cmd_dec_puu_ = config_.default_decel_puu;
  }

  // Preload S-0-0282 while AH (Drive Halt). Next ticks: halt 0→1 (AF),
  // then bit0 toggle. Combining those edges dropped the drive to Ab.
  move_phase_ = MovePhase::kPreload;
  move_phase_ticks_ = kMovePreloadTicks;
  ESP_LOGI(tag(),
           "[THETA] START current=%.2f target=%.2f cmd_puu=%ld vel=%ld "
           "(scale=%.1f PUU/deg)",
           static_cast<double>(getCurrentDeg()), static_cast<double>(target_deg),
           static_cast<long>(cmd_pos_puu_), static_cast<long>(cmd_vel_puu_),
           config_.puu_per_deg);
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
  return puuToDeg(fb.position_feedback - zero_puu_);
}

float GantryEipRotaryAxis::getTargetDeg() const { return target_deg_; }

bool GantryEipRotaryAxis::isBusy() const {
  if (stop_requested_ || !motion_active_) return false;
  if (move_phase_ == MovePhase::kPreload ||
      move_phase_ == MovePhase::kReleaseHalt ||
      move_phase_ == MovePhase::kStart) {
    return true;
  }
  eip::hcs01::Hcs01PositioningActual fb;
  if (!readFeedback(fb)) return true;
  return !fb.status.command_value_reached;
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
  move_phase_ticks_ = 0;
  arm_phase_ = ArmPhase::kClearFaults;
  arm_phase_ticks_ = kClearFaultTicks;
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
  const ThetaOrigin::Plan plan = ThetaOrigin::plan(
      abs_deg, AXIS_THETA_DRIVE_ABS_MIN_DEG, AXIS_THETA_DRIVE_ABS_MAX_DEG,
      AXIS_THETA_HARD_LIMIT_MIN_DEG, AXIS_THETA_HARD_LIMIT_MAX_DEG);
  if (!plan.ok) {
    ESP_LOGW(tag(),
             "[THETA] origin plan empty: drive_abs=%.3f drive=%.1f..%.1f",
             static_cast<double>(abs_deg),
             static_cast<double>(AXIS_THETA_DRIVE_ABS_MIN_DEG),
             static_cast<double>(AXIS_THETA_DRIVE_ABS_MAX_DEG));
    return false;
  }
  origin_aligned_ = plan.aligned;
  zero_puu_ = plan.aligned ? 0 : fb.position_feedback;
  envelope_min_deg_ = plan.joint_min_deg;
  envelope_max_deg_ = plan.joint_max_deg;
  min_deg_ = envelope_min_deg_;
  max_deg_ = envelope_max_deg_;
  target_deg_ = 0.0f;
  motion_active_ = false;
  stop_requested_ = false;
  move_phase_ = MovePhase::kIdle;
  ESP_LOGI(tag(),
           "[THETA] origin %s drive_abs=%.3f zero_puu=%ld joint_lim=%.2f..%.2f",
           plan.aligned ? "ALIGNED (joint=drive)" : "OFFSET (C0300 needed)",
           static_cast<double>(abs_deg), static_cast<long>(zero_puu_),
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

  if (arm_phase_ == ArmPhase::kClearFaults) {
    (void)publishClearFaults();
    if (arm_phase_ticks_ > 0) --arm_phase_ticks_;
    if (arm_phase_ticks_ == 0) {
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
      arm_phase_ticks_ = kWaitAfTicks;
      (void)publishHold();
    }
    return;
  }

  if (arm_phase_ == ArmPhase::kWaitAf) {
    (void)publishHold();
    if (arm_phase_ticks_ > 0) --arm_phase_ticks_;
    eip::hcs01::Hcs01PositioningActual fb;
    const bool have_fb = readFeedback(fb);
    if (have_fb && fb.status.ready == eip::hcs01::ReadyForOperation::kInOperation) {
      logAfStatus("Servo arm complete", fb);
      arm_phase_ = ArmPhase::kHolding;
      arm_phase_ticks_ = 0;
      return;
    }
    if (arm_phase_ticks_ == 0) {
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
    if (move_phase_ticks_ > 0) --move_phase_ticks_;
    (void)publishMove(/*start_edge=*/false);
    if (move_phase_ticks_ == 0) {
      move_phase_ = MovePhase::kReleaseHalt;
      move_phase_ticks_ = kMoveReleaseHaltTicks;
      (void)publishMove(/*start_edge=*/true);
    }
    return;
  }

  if (move_phase_ == MovePhase::kReleaseHalt) {
    if (move_phase_ticks_ > 0) --move_phase_ticks_;
    (void)publishMove(/*start_edge=*/true);
    bool following = false;
    eip::hcs01::Hcs01PositioningActual fb;
    if (readFeedback(fb)) {
      following = !fb.status.not_following_command &&
                  fb.status.ready == eip::hcs01::ReadyForOperation::kInOperation;
    }
    if (move_phase_ticks_ == 0 || following) {
      move_phase_ = MovePhase::kStart;
      move_phase_ticks_ = kMoveStartTicks;
      command_accept_level_ = !command_accept_level_;
      (void)publishMove(/*start_edge=*/true);
    }
    return;
  }

  if (move_phase_ == MovePhase::kStart) {
    if (move_phase_ticks_ > 0) --move_phase_ticks_;
    (void)publishMove(/*start_edge=*/true);
    if (move_phase_ticks_ == 0) {
      move_phase_ = MovePhase::kRun;
    }
    return;
  }

  if (!motion_active_ || stop_requested_ || move_phase_ != MovePhase::kRun) {
    return;
  }

  (void)publishMove(/*start_edge=*/true);

  eip::hcs01::Hcs01PositioningActual fb;
  if (!readFeedback(fb)) return;
  if (axisLogDue(log_rate_hz_, esp_timer_get_time(), &last_axislog_us_)) {
    ESP_LOGI(tag(),
             "[THETA] MOVE joint=%.3f drive=%.3f target=%.3f st=0x%04X "
             "ready=%s c1err=%d",
             static_cast<double>(puuToDeg(fb.position_feedback - zero_puu_)),
             static_cast<double>(puuToDeg(fb.position_feedback)),
             static_cast<double>(target_deg_), fb.status.encode(),
             readyLabel(fb.status), fb.status.class1_error ? 1 : 0);
  }
  if (fb.status.command_value_reached) {
    ESP_LOGI(tag(), "[THETA] END current=%.2f",
             static_cast<double>(puuToDeg(fb.position_feedback - zero_puu_)));
    motion_active_ = false;
    move_phase_ = MovePhase::kIdle;
    (void)publishHold();
    return;
  }

  ++stuck_log_div_;
  if (stuck_log_div_ >= 100) {
    stuck_log_div_ = 0;
    ESP_LOGW(tag(),
             "[THETA] waiting in-pos (current=%.2f target=%.2f st=0x%04X "
             "reached=%d standstill=%d not_follow=%d ready=%u (%s) "
             "diag=0x%08lX)",
             static_cast<double>(puuToDeg(fb.position_feedback - zero_puu_)),
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

}  // namespace Gantry

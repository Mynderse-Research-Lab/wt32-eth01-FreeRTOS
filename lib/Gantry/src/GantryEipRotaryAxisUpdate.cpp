/**
 * @file GantryEipRotaryAxisUpdate.cpp
 * @brief GantryEipRotaryAxis::update and the move trace stamps.
 *
 * One of the GantryEipRotaryAxis files; GantryEipRotaryAxisInternal.h
 * holds what they share.
 */

#include "GantryEipRotaryAxis.h"
#include "GantryEipRotaryAxisInternal.h"

#include <cmath>

namespace Gantry {

void GantryEipRotaryAxis::update() {
  if (image_.feedbackFresh()) {
    image_.consumeFeedbackFresh();
  }

  if (!image_.isOnline()) {
    if (motion_active_ || arm_phase_ != ArmPhase::kIdle) {
      const char* tag = (log_tag_ && log_tag_[0]) ? log_tag_ : "EipRotary";
      ESP_LOGW(tag, "EIP connection offline — aborting motion and disarming");
      if (motion_active_) {
        stampEnd(RotaryMoveEnd::kAbort);
      }
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
      stamp_start_us_.store(esp_timer_get_time(), std::memory_order_release);
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
      stiction_stable_us_ = 0;
      prev_deg_ = getCurrentDeg();
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
  const float err_deg = std::fabs(now_deg - target_deg_);
  const float step_deg = std::fabs(now_deg - prev_deg_);
  prev_deg_ = now_deg;

  const bool at_target_exact =
      err_deg <= kArrivalEpsDeg && fb.status.command_value_reached;
  const bool at_target_stiction =
      err_deg <= kStictionArrivalEpsDeg && fb.status.in_standstill &&
      step_deg <= kStictionStepEpsDeg;

  if (in_pos_armed_ && at_target_exact) {
    accrueUs(arrival_stable_us_, legacyTicksUs(kArrivalStableTicks));
    if (arrival_stable_us_ >= legacyTicksUs(kArrivalStableTicks)) {
      ESP_LOGI(tag(), "[THETA] END current=%.3f", static_cast<double>(now_deg));
      finishMoveHold(RotaryMoveEnd::kExact);
      return;
    }
  } else {
    arrival_stable_us_ = 0;
  }

  if (in_pos_armed_ && at_target_stiction) {
    accrueUs(stiction_stable_us_, legacyTicksUs(kStictionStableTicks));
    if (stiction_stable_us_ >= legacyTicksUs(kStictionStableTicks)) {
      ESP_LOGI(tag(), "[THETA] END current=%.3f", static_cast<double>(now_deg));
      finishMoveHold(RotaryMoveEnd::kStiction);
      return;
    }
  } else {
    stiction_stable_us_ = 0;
  }

  accrueUs(run_us_, move_timeout_us_);
  if (run_us_ >= move_timeout_us_) {
    ESP_LOGW(tag(),
             "[THETA] move timeout (current=%.3f target=%.3f) — halt",
             static_cast<double>(now_deg), static_cast<double>(target_deg_));
    finishMoveHold(RotaryMoveEnd::kTimeout);
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

// S-0-0040 is read with the S-0-0259 scaling (S-0-0044..0046, 0.0001 rpm).
// Assumed from the shared velocity scaling; check against the position slope.
bool GantryEipRotaryAxis::sampleTrace(RotaryTraceSample& out) const {
  if (!image_.isOnline() || config_.puu_per_deg <= 0.0) return false;
  eip::hcs01::Hcs01PositioningActual fb;
  if (!readFeedback(fb)) return false;
  out.deg = static_cast<double>(fb.position_feedback) / config_.puu_per_deg;
  out.vel_deg_s = static_cast<float>(static_cast<double>(fb.velocity_feedback) /
                                     kHcs01VelocityUnitsPerDegS);
  out.status = fb.status.encode();
  return true;
}

bool GantryEipRotaryAxis::getMoveStamp(RotaryMoveStamp& out) const {
  for (int tries = 0; tries < 8; ++tries) {
    const uint32_t s1 = stamp_seq_.load(std::memory_order_acquire);
    if ((s1 & 1u) != 0) continue;
    out.cmd_us = stamp_cmd_us_.load(std::memory_order_relaxed);
    out.start_us = stamp_start_us_.load(std::memory_order_relaxed);
    out.done_us = stamp_done_us_.load(std::memory_order_relaxed);
    out.end = static_cast<RotaryMoveEnd>(stamp_end_.load(std::memory_order_relaxed));
    out.start_deg = stamp_start_deg_.load(std::memory_order_relaxed);
    out.target_deg = stamp_target_deg_.load(std::memory_order_relaxed);
    out.vel_deg_s = stamp_vel_deg_s_.load(std::memory_order_relaxed);
    out.acc_deg_s2 = stamp_acc_deg_s2_.load(std::memory_order_relaxed);
    out.dec_deg_s2 = stamp_dec_deg_s2_.load(std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_acquire);
    if (stamp_seq_.load(std::memory_order_relaxed) == s1) {
      out.seq = s1 / 2u;
      return true;
    }
  }
  return false;
}

// Stamps the move as sent: speed and accel after the drive-unit caps.
void GantryEipRotaryAxis::stampMove() {
  constexpr double kAccelUnitsPerDegS2 = 1000.0 * 3.141592653589793 / 180.0;
  const uint32_t seq = stamp_seq_.load(std::memory_order_relaxed);
  stamp_seq_.store(seq + 1u, std::memory_order_relaxed);
  std::atomic_thread_fence(std::memory_order_release);
  stamp_cmd_us_.store(esp_timer_get_time(), std::memory_order_relaxed);
  stamp_start_us_.store(0, std::memory_order_relaxed);
  stamp_done_us_.store(0, std::memory_order_relaxed);
  stamp_end_.store(static_cast<uint8_t>(RotaryMoveEnd::kNone), std::memory_order_relaxed);
  stamp_start_deg_.store(start_deg_, std::memory_order_relaxed);
  stamp_target_deg_.store(target_deg_, std::memory_order_relaxed);
  stamp_vel_deg_s_.store(
      static_cast<float>(static_cast<double>(cmd_vel_puu_) / kHcs01VelocityUnitsPerDegS),
      std::memory_order_relaxed);
  stamp_acc_deg_s2_.store(
      static_cast<float>(static_cast<double>(cmd_acc_puu_) / kAccelUnitsPerDegS2),
      std::memory_order_relaxed);
  stamp_dec_deg_s2_.store(
      static_cast<float>(static_cast<double>(cmd_dec_puu_) / kAccelUnitsPerDegS2),
      std::memory_order_relaxed);
  stamp_seq_.store(seq + 2u, std::memory_order_release);
}

void GantryEipRotaryAxis::stampEnd(RotaryMoveEnd end) {
  if (stamp_seq_.load(std::memory_order_acquire) == 0 ||
      stamp_done_us_.load(std::memory_order_acquire) != 0) {
    return;
  }
  stamp_end_.store(static_cast<uint8_t>(end), std::memory_order_relaxed);
  stamp_done_us_.store(esp_timer_get_time(), std::memory_order_release);
}

}  // namespace Gantry

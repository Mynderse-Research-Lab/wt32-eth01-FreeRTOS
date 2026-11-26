/**
 * @file GantryEipLinearAxisPhases.cpp
 * @brief Arm and move phase machine for GantryEipLinearAxis.
 *
 * One of the GantryEipLinearAxis files; GantryEipLinearAxisInternal.h
 * holds what they share.
 */

#include "GantryEipLinearAxis.h"
#include "GantryEipLinearAxisInternal.h"
#include "EipClass1TimingStats.h"
#include "GantryControlPeriod.h"
#include "KinetixFaultCodes.h"

#include <cmath>
#include <cstdio>

namespace Gantry {

eip::k5100::OutputAssembly104 GantryEipLinearAxis::buildSettleCommand(
    bool servo_on) const {
  eip::k5100::OutputAssembly104 cmd;
  cmd.operating_mode =
      static_cast<int8_t>(eip::k5100::OperatingMode::kNotSpecified);
  cmd.servo_on = servo_on;
  cmd.travel_mode = 10;
  cmd.torque_ramp_time = 1000;
  cmd.position_reference = command_position_puu_;
  return cmd;
}

eip::k5100::OutputAssembly104 GantryEipLinearAxis::buildHomeCommand(
    bool start_motion_edge) const {
  eip::k5100::OutputAssembly104 cmd;
  cmd.operating_mode = static_cast<int8_t>(eip::k5100::OperatingMode::kHome);
  cmd.servo_on = true;
  cmd.travel_mode = 2;
  cmd.homing_method = kHomingMethodDefineCurrent;
  cmd.home_return_speed = 60;  // 6.0 RPM
  cmd.speed_reference = 600;   // 60.0 RPM (method 34 typically no travel)
  cmd.accel_reference = kDefaultAccelRef;
  cmd.decel_reference = kDefaultAccelRef;
  cmd.position_reference = command_position_puu_;
  cmd.torque_ramp_time = 1000;
  cmd.start_motion = start_motion_edge;
  return cmd;
}

eip::k5100::OutputAssembly104 GantryEipLinearAxis::buildHoldCommand() const {
  // Post-move hold: settle image (OM=0 TM=10) — golden sequence step 5, docs §6.
  // Only published once the axis is verified stopped, never mid-profile.
  return buildSettleCommand(enabled_);
}

eip::k5100::OutputAssembly104 GantryEipLinearAxis::buildStopCommand(
    bool stop_motion) const {
  // Abort must differ from the in-flight move image by the StopMotion bit alone.
  // The settle image (OM=0 TM=10, speed/accel/decel all 0) A603s when published
  // against a running Absolute: it swaps OperatingMode and TravelMode mid-profile
  // and asks for a decel-to-stop with a zero decel ramp. The drive rejects it and
  // keeps executing the old profile, which is why aborted creeps ran on forever.
  eip::k5100::OutputAssembly104 cmd = buildMoveCommand(/*start_motion_edge=*/false);
  cmd.servo_on = enabled_;
  cmd.stop_motion = stop_motion;
  return cmd;
}

eip::k5100::OutputAssembly104 GantryEipLinearAxis::buildMoveCommand(
    bool start_motion_edge) const {
  eip::k5100::OutputAssembly104 cmd;
  cmd.operating_mode =
      static_cast<int8_t>(eip::k5100::OperatingMode::kPosition);
  cmd.servo_on = true;
  cmd.travel_mode = 2;
  cmd.non_cyclic_move_type =
      static_cast<int8_t>(eip::k5100::NonCyclicMoveType::kAbsolute);
  cmd.torque_ramp_time = 1000;
  cmd.start_motion = start_motion_edge;
  cmd.speed_reference = move_speed_ref_ > 0 ? move_speed_ref_ : 1;
  cmd.position_reference = command_position_puu_;

  if (last_accel_mm_s2_ > 0.0f) {
    cmd.accel_reference = static_cast<int32_t>(llround(
        static_cast<double>(last_accel_mm_s2_) * config_.accel_ref_per_mm_s2));
  } else {
    cmd.accel_reference = kDefaultAccelRef;
  }
  if (last_decel_mm_s2_ > 0.0f) {
    cmd.decel_reference = static_cast<int32_t>(llround(
        static_cast<double>(last_decel_mm_s2_) * config_.decel_ref_per_mm_s2));
  } else {
    cmd.decel_reference = kDefaultAccelRef;
  }
  if (cmd.accel_reference < 1) cmd.accel_reference = kDefaultAccelRef;
  if (cmd.decel_reference < 1) cmd.decel_reference = kDefaultAccelRef;
  // EDS accel/decel floor ~45.8 RPM/s (458 in 0.1 units) — clamp if tiny.
  constexpr int32_t kMinAccelRef = 458;
  if (cmd.accel_reference < kMinAccelRef) cmd.accel_reference = kMinAccelRef;
  if (cmd.decel_reference < kMinAccelRef) cmd.decel_reference = kMinAccelRef;
  return cmd;
}

void GantryEipLinearAxis::republishMoveCommand(bool start_motion_edge) {
  (void)publishCommand(buildMoveCommand(start_motion_edge));
}

bool GantryEipLinearAxis::inPositionBand(
    const eip::k5100::InputAssembly154& fb) const {
  const int32_t err = fb.actual_position - command_position_puu_;
  const double rem_mm =
      (absPuuPerMm() > 0.0)
          ? (std::fabs(static_cast<double>(err)) / absPuuPerMm())
          : 1e9;
  return rem_mm <= kArrivalBandMm;
}

void GantryEipLinearAxis::finishMoveHold() {
  motion_commanded_ = false;
  stop_requested_ = false;
  queued_ = QueuedMove::kNone;
  move_phase_ = MovePhase::kIdle;
  move_phase_us_ = 0;
  run_us_ = 0;
  stop_stable_us_ = 0;
  arrival_stable_us_ = 0;
  stop_sample_valid_ = false;
  // Keep command_position at the Absolute target (do not snap — preserves
  // puuinfo target vs actual for calibration).
  (void)publishCommand(buildHoldCommand());
}

void GantryEipLinearAxis::enterAbortStop() {
  move_phase_ = MovePhase::kStopping;
  move_phase_us_ = Gantry::legacyTicksUs(kStopPulseTicks);
  run_us_ = 0;  // stop-elapsed timeout counter while Stopping
  stop_stable_us_ = 0;
  stop_sample_valid_ = false;
  (void)publishCommand(buildStopCommand(true));
}

void GantryEipLinearAxis::advanceArmPhase() {
  if (arm_phase_ == ArmPhase::kIdle || !enabled_) return;

  if (!motion_commanded_ && arm_phase_ == ArmPhase::kHolding) {
    eip::k5100::InputAssembly154 fb;
    if (readFeedback(fb)) {
      command_position_puu_ = fb.actual_position;
      target_mm_ = puuToMm(toJointPuu(command_position_puu_));
    }
  }

  eip::k5100::InputAssembly154 fb{};
  const bool have_fb = readFeedback(fb);

  if ((arm_phase_ == ArmPhase::kServoSettle ||
       arm_phase_ == ArmPhase::kHolding ||
       arm_phase_ == ArmPhase::kHomeWait) &&
      have_fb && fb.warning_present &&
      eip::k5100::isWarningA603(fb.warning_code) &&
      a603_clear_attempts_ < 2) {
    ++a603_clear_attempts_;
    eip::k5100::OutputAssembly104 clr = buildSettleCommand(true);
    clr.fault_reset = true;
    fault_reset_pulse_us_ = Gantry::legacyTicksUs(3);
    (void)publishCommand(clr);
    return;
  }

  if (arm_phase_us_ > 0) {
    Gantry::spendUs(arm_phase_us_);
  }

  if (arm_phase_ == ArmPhase::kServoLow) {
    if (arm_phase_us_ > 0) return;
    arm_phase_ = ArmPhase::kServoSettle;
    arm_phase_us_ = Gantry::legacyTicksUs(kArmServoSettleTicks);
    (void)publishCommand(buildSettleCommand(/*servo_on=*/true));
    return;
  }

  if (arm_phase_ == ArmPhase::kServoSettle) {
    if (fault_reset_pulse_us_ == 0) {
      (void)publishCommand(buildSettleCommand(/*servo_on=*/true));
    }
    const bool active_ok =
        have_fb && fb.active && fb.ready && !fb.fault &&
        !fb.connection_faulted &&
        !(fb.warning_present && eip::k5100::isWarningA603(fb.warning_code));
    if (active_ok || arm_phase_us_ == 0) {
      const char* tag = (log_tag_ && log_tag_[0]) ? log_tag_ : kEipAxisTag;
      if (have_fb && fb.active) {
        ESP_LOGI(tag, "Servo arm complete: active=1 ready=%d pos=%ld PUU",
                 (int)fb.ready, (long)fb.actual_position);
        command_position_puu_ = fb.actual_position;
        if (fb.homed_status) {
          arm_phase_ = ArmPhase::kHolding;
          arm_phase_us_ = 0;
          (void)publishCommand(buildSettleCommand(true));
        } else {
          arm_phase_ = ArmPhase::kHomePreload;
          arm_phase_us_ = Gantry::legacyTicksUs(kHomePreloadTicks);
          (void)publishCommand(buildHomeCommand(false));
        }
      } else if (have_fb) {
        ESP_LOGW(tag,
                 "Servo arm timeout: active=0 ready=%d — check STO/SON; "
                 "A603 clears=%u",
                 (int)fb.ready, (unsigned)a603_clear_attempts_);
        arm_phase_ = ArmPhase::kHolding;
        arm_phase_us_ = 0;
        (void)publishCommand(buildSettleCommand(true));
      }
    }
    return;
  }

  if (arm_phase_ == ArmPhase::kHomePreload) {
    (void)publishCommand(buildHomeCommand(false));
    if (arm_phase_us_ == 0) {
      arm_phase_ = ArmPhase::kHomeStart;
      arm_phase_us_ = Gantry::legacyTicksUs(kHomeStartTicks);
      (void)publishCommand(buildHomeCommand(true));
    }
    return;
  }

  if (arm_phase_ == ArmPhase::kHomeStart) {
    (void)publishCommand(buildHomeCommand(arm_phase_us_ > 0));
    if (arm_phase_us_ == 0) {
      arm_phase_ = ArmPhase::kHomeWait;
      home_wait_us_ = Gantry::legacyTicksUs(kHomeWaitTicks);
      (void)publishCommand(buildHomeCommand(false));
    }
    return;
  }

  if (arm_phase_ == ArmPhase::kHomeWait) {
    (void)publishCommand(buildHomeCommand(false));
    if (home_wait_us_ > 0) Gantry::spendUs(home_wait_us_);
    if (have_fb && fb.homed_status) {
      // Method 34 may redefine drive origin — resync joint zero to current.
      zero_puu_ = fb.actual_position;
      command_position_puu_ = fb.actual_position;
      target_mm_ = 0.0f;
      const char* tag = (log_tag_ && log_tag_[0]) ? log_tag_ : kEipAxisTag;
      ESP_LOGI(tag, "Home34 complete: HomedStatus=1 pos=%ld (joint zero synced)",
               (long)fb.actual_position);
      arm_phase_ = ArmPhase::kHolding;
      (void)publishCommand(buildSettleCommand(true));
    } else if (home_wait_us_ == 0) {
      const char* tag = (log_tag_ && log_tag_[0]) ? log_tag_ : kEipAxisTag;
      ESP_LOGW(tag, "Home34 timeout — Absolute moves may raise E237");
      arm_phase_ = ArmPhase::kHolding;
      (void)publishCommand(buildSettleCommand(true));
    }
    return;
  }
}

void GantryEipLinearAxis::advanceMovePhase(
    const eip::k5100::InputAssembly154& fb) {
  if (move_phase_ == MovePhase::kStopping) {
    const int32_t pos_lim = static_cast<int32_t>(
        llround(absPuuPerMm() * kStopStableMmPerTick));
    const int32_t lim = (pos_lim < 1) ? 1 : pos_lim;
    if (!stop_sample_valid_) {
      last_stop_sample_puu_ = fb.actual_position;
      stop_sample_valid_ = true;
      stop_stable_us_ = 0;
    } else {
      const int32_t d = fb.actual_position - last_stop_sample_puu_;
      last_stop_sample_puu_ = fb.actual_position;
      if (d <= lim && d >= -lim) {
        Gantry::accrueUs(stop_stable_us_, Gantry::legacyTicksUs(kStopStableTicks));
      } else {
        stop_stable_us_ = 0;
      }
    }
    if (move_phase_us_ > 0) Gantry::spendUs(move_phase_us_);
    Gantry::accrueUs(run_us_, Gantry::legacyTicksUs(kStopTimeoutTicks));
    // Pulse StopMotion for kStopPulseTicks, then keep settle; do NOT finish
    // when the pulse ends — early Hold while Absolute still coasts drifts the
    // axis with firmware busy=0 (seen after clear-edge home).
    (void)publishCommand(buildStopCommand(move_phase_us_ > 0));
    const bool position_stable = (stop_stable_us_ >= Gantry::legacyTicksUs(kStopStableTicks));
    const bool speed_low =
        fb.isMotionStopped() ||
        (fb.actual_speed <= kNearStopSpeedRef &&
         fb.actual_speed >= -kNearStopSpeedRef);
    if (position_stable && speed_low) {
      command_position_puu_ = fb.actual_position;
      target_mm_ = puuToMm(toJointPuu(command_position_puu_));
      finishMoveHold();
      return;
    }
    if (run_us_ >= Gantry::legacyTicksUs(kStopTimeoutTicks)) {
      const char* tag = (log_tag_ && log_tag_[0]) ? log_tag_ : kEipAxisTag;
      ESP_LOGW(tag, "StopMotion timeout — forcing hold (pos still moving?)");
      command_position_puu_ = fb.actual_position;
      target_mm_ = puuToMm(toJointPuu(command_position_puu_));
      finishMoveHold();
    }
    return;
  }

  if (move_phase_ == MovePhase::kPreloadHold) {
    republishMoveCommand(/*start_motion_edge=*/false);
    return;
  }

  if (move_phase_us_ > 0) Gantry::spendUs(move_phase_us_);

  if (move_phase_ == MovePhase::kPreload) {
    republishMoveCommand(/*start_motion_edge=*/false);
    if (move_phase_us_ == 0) {
      move_phase_ = MovePhase::kStart;
      move_phase_us_ = Gantry::legacyTicksUs(kMoveStartMotionTicks);
      eip::class1TimingStats().noteAbsoluteStartMotionPublished(
          eip::class1NowUs());
      republishMoveCommand(/*start_motion_edge=*/true);
    }
    return;
  }

  if (move_phase_ == MovePhase::kStart) {
    republishMoveCommand(/*start_motion_edge=*/move_phase_us_ > 0);
    if (move_phase_us_ == 0) {
      move_phase_ = MovePhase::kRun;
      run_us_ = 0;
      if (queued_ == QueuedMove::kPending) {
        applyQueuedMove();
      } else {
        republishMoveCommand(/*start_motion_edge=*/false);
      }
    }
    return;
  }

  if (move_phase_ == MovePhase::kRun && !stop_requested_) {
    const bool a014 =
        fb.warning_present && eip::k5100::isWarningA014(fb.warning_code);
    const bool a015 =
        fb.warning_present && eip::k5100::isWarningA015(fb.warning_code);
    if (!a014) ignore_a014_abort_ = false;
    if (!a015) ignore_a015_abort_ = false;
    const bool trip_a014 = a014 && !ignore_a014_abort_;
    const bool trip_a015 = a015 && !ignore_a015_abort_;
    if (fb.isLikelyLimitStop() || fb.hasDriveFault() || trip_a014 || trip_a015) {
      const char* tag = (log_tag_ && log_tag_[0]) ? log_tag_ : kEipAxisTag;
      ESP_LOGW(tag,
               "Absolute move abort: drive fault/limit (fault=%d stopped=%d "
               "A014=%d A015=%d)",
               (int)fb.fault, (int)fb.stopped, (int)trip_a014, (int)trip_a015);
      enterAbortStop();
      return;
    }
    Gantry::accrueUs(run_us_, Gantry::legacyTicksUs(kMoveTimeoutTicks));
    republishMoveCommand(/*start_motion_edge=*/false);

    const bool at_ref = fb.at_reference && inPositionBand(fb);
    const bool band_stopped =
        inPositionBand(fb) &&
        (fb.isMotionStopped() ||
         (fb.actual_speed <= kNearStopSpeedRef &&
          fb.actual_speed >= -kNearStopSpeedRef));
    if (at_ref || band_stopped) {
      Gantry::accrueUs(arrival_stable_us_, Gantry::legacyTicksUs(kArrivalStableTicks));
      if (arrival_stable_us_ >= Gantry::legacyTicksUs(kArrivalStableTicks)) {
        finishMoveHold();
        return;
      }
    } else {
      arrival_stable_us_ = 0;
    }
    if (run_us_ >= Gantry::legacyTicksUs(kMoveTimeoutTicks)) {
      const char* tag = (log_tag_ && log_tag_[0]) ? log_tag_ : kEipAxisTag;
      ESP_LOGW(tag, "Absolute move timeout — aborting with StopMotion");
      enterAbortStop();
    }
  }
}

}  // namespace Gantry

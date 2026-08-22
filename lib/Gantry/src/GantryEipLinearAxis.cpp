/**
 * @file GantryEipLinearAxis.cpp
 * @brief Position Absolute PTP over Kinetix 5100 assemblies 104/154.
 *
 * See docs/LOW_LEVEL_GANTRY_CONTROL.md and tools/eip_position_abs.py.
 */

#include "GantryEipLinearAxis.h"

#include "EipClass1TimingStats.h"
#include "GantryUtils.h"
#include "Kinetix5100Assembly.h"
#include "KinetixFaultCodes.h"

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
const char* kEipAxisTag = "EipAxis";

constexpr uint8_t kArmServoLowTicks = 4;
constexpr uint8_t kArmServoSettleTicks = 40;
constexpr uint8_t kHomePreloadTicks = 4;
constexpr uint8_t kHomeStartTicks = 5;
constexpr uint16_t kHomeWaitTicks = 200;  // ~2 s @ 100 Hz
// Absolute PTP matches Home34: preload Position (SM=0), then StartMotion edge.
// Fast-path SM=1 in the same packet as a new Position was dropped by Kinetix
// (Z 130→30 sat until the 60 s timeout; retry after StopMotion ran).
constexpr uint8_t kMovePreloadTicks = 4;
constexpr uint8_t kMoveStartMotionTicks = 5;
constexpr uint8_t kStopPulseTicks = 5;
constexpr uint8_t kStopStableTicks = 20;   // 200 ms @ 100 Hz
constexpr uint16_t kStopTimeoutTicks = 300;  // ~3 s @ 100 Hz — never end Stop early
constexpr int32_t kDefaultAccelRef = 1000;  // 100.0 RPM/s
// 0.8 RPM: must be below ~1.5 RPM @ 1 mm/s X creep or Stop "completes" while coasting.
constexpr int32_t kNearStopSpeedRef = 8;
constexpr double kStopStableMmPerTick = 0.005;
constexpr int8_t kHomingMethodDefineCurrent = 34;
constexpr double kArrivalBandMm = 0.5;
constexpr uint8_t kArrivalStableTicks = 3;  // ~30 ms @ 100 Hz before finishMoveHold
constexpr uint16_t kMoveTimeoutTicks = 6000;  // ~60 s @ 100 Hz
}  // namespace

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
      arm_phase_ticks_(0),
      a603_clear_attempts_(0),
      home_wait_ticks_(0),
      move_phase_(MovePhase::kIdle),
      move_phase_ticks_(0),
      fault_reset_pulse_ticks_(0),
      last_speed_mm_s_(50.0f),
      last_accel_mm_s2_(0.0f),
      last_decel_mm_s2_(0.0f),
      move_speed_ref_(0),
      run_ticks_(0),
      stop_stable_ticks_(0),
      arrival_stable_ticks_(0),
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
  move_phase_ticks_ = 0;

  eip::k5100::InputAssembly154 fb;
  if (readFeedback(fb)) {
    command_position_puu_ = fb.actual_position;
    target_mm_ = puuToMm(toJointPuu(command_position_puu_));
  }

  arm_phase_ = ArmPhase::kServoLow;
  arm_phase_ticks_ = kArmServoLowTicks;
  a603_clear_attempts_ = 0;
  return publishCommand(buildSettleCommand(/*servo_on=*/false));
}

bool GantryEipLinearAxis::disable() {
  enabled_ = false;
  stop_requested_ = false;
  motion_commanded_ = false;
  move_phase_ = MovePhase::kIdle;
  move_phase_ticks_ = 0;
  arm_phase_ = ArmPhase::kIdle;
  arm_phase_ticks_ = 0;
  eip::k5100::OutputAssembly104 cmd;
  cmd.servo_off = true;
  cmd.travel_mode = 10;
  cmd.torque_ramp_time = 1000;
  return publishCommand(cmd);
}

bool GantryEipLinearAxis::isEnabled() const { return enabled_; }

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
  move_phase_ = MovePhase::kIdle;
  move_phase_ticks_ = 0;
  run_ticks_ = 0;
  stop_stable_ticks_ = 0;
  arrival_stable_ticks_ = 0;
  stop_sample_valid_ = false;
  // Keep command_position at the Absolute target (do not snap — preserves
  // puuinfo target vs actual for calibration).
  (void)publishCommand(buildHoldCommand());
}

void GantryEipLinearAxis::enterAbortStop() {
  move_phase_ = MovePhase::kStopping;
  move_phase_ticks_ = kStopPulseTicks;
  run_ticks_ = 0;  // stop-elapsed timeout counter while Stopping
  stop_stable_ticks_ = 0;
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
    fault_reset_pulse_ticks_ = 3;
    (void)publishCommand(clr);
    return;
  }

  if (arm_phase_ticks_ > 0) {
    --arm_phase_ticks_;
  }

  if (arm_phase_ == ArmPhase::kServoLow) {
    if (arm_phase_ticks_ > 0) return;
    arm_phase_ = ArmPhase::kServoSettle;
    arm_phase_ticks_ = kArmServoSettleTicks;
    (void)publishCommand(buildSettleCommand(/*servo_on=*/true));
    return;
  }

  if (arm_phase_ == ArmPhase::kServoSettle) {
    if (fault_reset_pulse_ticks_ == 0) {
      (void)publishCommand(buildSettleCommand(/*servo_on=*/true));
    }
    const bool active_ok =
        have_fb && fb.active && fb.ready && !fb.fault &&
        !fb.connection_faulted &&
        !(fb.warning_present && eip::k5100::isWarningA603(fb.warning_code));
    if (active_ok || arm_phase_ticks_ == 0) {
      const char* tag = (log_tag_ && log_tag_[0]) ? log_tag_ : kEipAxisTag;
      if (have_fb && fb.active) {
        ESP_LOGI(tag, "Servo arm complete: active=1 ready=%d pos=%ld PUU",
                 (int)fb.ready, (long)fb.actual_position);
        command_position_puu_ = fb.actual_position;
        if (fb.homed_status) {
          arm_phase_ = ArmPhase::kHolding;
          arm_phase_ticks_ = 0;
          (void)publishCommand(buildSettleCommand(true));
        } else {
          arm_phase_ = ArmPhase::kHomePreload;
          arm_phase_ticks_ = kHomePreloadTicks;
          (void)publishCommand(buildHomeCommand(false));
        }
      } else if (have_fb) {
        ESP_LOGW(tag,
                 "Servo arm timeout: active=0 ready=%d — check STO/SON; "
                 "A603 clears=%u",
                 (int)fb.ready, (unsigned)a603_clear_attempts_);
        arm_phase_ = ArmPhase::kHolding;
        arm_phase_ticks_ = 0;
        (void)publishCommand(buildSettleCommand(true));
      }
    }
    return;
  }

  if (arm_phase_ == ArmPhase::kHomePreload) {
    (void)publishCommand(buildHomeCommand(false));
    if (arm_phase_ticks_ == 0) {
      arm_phase_ = ArmPhase::kHomeStart;
      arm_phase_ticks_ = kHomeStartTicks;
      (void)publishCommand(buildHomeCommand(true));
    }
    return;
  }

  if (arm_phase_ == ArmPhase::kHomeStart) {
    (void)publishCommand(buildHomeCommand(arm_phase_ticks_ > 0));
    if (arm_phase_ticks_ == 0) {
      arm_phase_ = ArmPhase::kHomeWait;
      home_wait_ticks_ = kHomeWaitTicks;
      (void)publishCommand(buildHomeCommand(false));
    }
    return;
  }

  if (arm_phase_ == ArmPhase::kHomeWait) {
    (void)publishCommand(buildHomeCommand(false));
    if (home_wait_ticks_ > 0) --home_wait_ticks_;
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
    } else if (home_wait_ticks_ == 0) {
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
      stop_stable_ticks_ = 0;
    } else {
      const int32_t d = fb.actual_position - last_stop_sample_puu_;
      last_stop_sample_puu_ = fb.actual_position;
      if (d <= lim && d >= -lim) {
        if (stop_stable_ticks_ < 255) ++stop_stable_ticks_;
      } else {
        stop_stable_ticks_ = 0;
      }
    }
    if (move_phase_ticks_ > 0) --move_phase_ticks_;
    if (run_ticks_ < 0xFFFF) ++run_ticks_;
    // Pulse StopMotion for kStopPulseTicks, then keep settle; do NOT finish
    // when the pulse ends — early Hold while Absolute still coasts drifts the
    // axis with firmware busy=0 (seen after clear-edge home).
    (void)publishCommand(buildStopCommand(move_phase_ticks_ > 0));
    const bool position_stable = (stop_stable_ticks_ >= kStopStableTicks);
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
    if (run_ticks_ >= kStopTimeoutTicks) {
      const char* tag = (log_tag_ && log_tag_[0]) ? log_tag_ : kEipAxisTag;
      ESP_LOGW(tag, "StopMotion timeout — forcing hold (pos still moving?)");
      command_position_puu_ = fb.actual_position;
      target_mm_ = puuToMm(toJointPuu(command_position_puu_));
      finishMoveHold();
    }
    return;
  }

  if (move_phase_ticks_ > 0) --move_phase_ticks_;

  if (move_phase_ == MovePhase::kPreload) {
    republishMoveCommand(/*start_motion_edge=*/false);
    if (move_phase_ticks_ == 0) {
      move_phase_ = MovePhase::kStart;
      move_phase_ticks_ = kMoveStartMotionTicks;
      eip::class1TimingStats().noteAbsoluteStartMotionPublished(
          eip::class1NowUs());
      republishMoveCommand(/*start_motion_edge=*/true);
    }
    return;
  }

  if (move_phase_ == MovePhase::kStart) {
    republishMoveCommand(/*start_motion_edge=*/move_phase_ticks_ > 0);
    if (move_phase_ticks_ == 0) {
      move_phase_ = MovePhase::kRun;
      run_ticks_ = 0;
      republishMoveCommand(/*start_motion_edge=*/false);
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
    if (run_ticks_ < 0xFFFF) ++run_ticks_;
    republishMoveCommand(/*start_motion_edge=*/false);

    const bool at_ref = fb.at_reference && inPositionBand(fb);
    const bool band_stopped =
        inPositionBand(fb) &&
        (fb.isMotionStopped() ||
         (fb.actual_speed <= kNearStopSpeedRef &&
          fb.actual_speed >= -kNearStopSpeedRef));
    if (at_ref || band_stopped) {
      if (arrival_stable_ticks_ < 255) ++arrival_stable_ticks_;
      if (arrival_stable_ticks_ >= kArrivalStableTicks) {
        finishMoveHold();
        return;
      }
    } else {
      arrival_stable_ticks_ = 0;
    }
    if (run_ticks_ >= kMoveTimeoutTicks) {
      const char* tag = (log_tag_ && log_tag_[0]) ? log_tag_ : kEipAxisTag;
      ESP_LOGW(tag, "Absolute move timeout — aborting with StopMotion");
      enterAbortStop();
    }
  }
}

bool GantryEipLinearAxis::moveToMm(float target_mm, float speed_mm_per_s,
                                   float accel_mm_per_s2, float decel_mm_per_s2) {
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
  if (std::fabs(here - target_mm) <= 0.05f) {
    motion_commanded_ = false;
    move_phase_ = MovePhase::kIdle;
    return publishCommand(buildHoldCommand());
  }

  motion_commanded_ = true;
  // Preload the new Absolute with StartMotion=0; the rising edge is kStart.
  move_phase_ = MovePhase::kPreload;
  move_phase_ticks_ = kMovePreloadTicks;
  run_ticks_ = 0;
  arrival_stable_ticks_ = 0;
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

bool GantryEipLinearAxis::isAlarmActive() const {
  eip::k5100::InputAssembly154 fb;
  if (!readFeedback(fb)) return false;
  return fb.fault || fb.connection_faulted;
}

bool GantryEipLinearAxis::clearAlarm() {
  motion_commanded_ = false;
  move_phase_ = MovePhase::kIdle;
  move_phase_ticks_ = 0;

  eip::k5100::InputAssembly154 fb;
  if (readFeedback(fb)) {
    command_position_puu_ = fb.actual_position;
    target_mm_ = puuToMm(toJointPuu(command_position_puu_));
  }

  eip::k5100::OutputAssembly104 cmd = buildSettleCommand(enabled_);
  cmd.fault_reset = true;
  fault_reset_pulse_ticks_ = 3;
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
               "Home34 (see docs/LOW_LEVEL_GANTRY_CONTROL.md)");
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

  if (fault_reset_pulse_ticks_ > 0) {
    --fault_reset_pulse_ticks_;
    if (fault_reset_pulse_ticks_ == 0) {
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

}  // namespace Gantry

/**
 * @file pick_scheduler.cpp
 * @brief On-the-fly side-grip pick scheduler. Joint moves only; the strike is
 *        timer-fired and re-timed on every conveyor frame.
 */

#include "pick_scheduler.h"

#include "pick_intercept.h"
#include "AppConfig.h"
#include "axis_drivetrain_params.h"
#include "gantry_app_constants.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include <atomic>
#include <cstdio>
#include <cstring>

static const char *TAG = "PickScheduler";

constexpr uint8_t kCmdStart = 1;
constexpr uint8_t kCmdStop = 2;
constexpr uint8_t kCmdPause = 3;
constexpr uint8_t kCmdClearFault = 4;
constexpr uint8_t kCmdPark = 5;
constexpr uint8_t kCmdSetGripper = 6;

constexpr uint8_t kStrikeZ = 1;
constexpr uint8_t kStrikeGrip = 2;
constexpr uint8_t kStrikeRetime = 4;

// Timer bits earlier than this before their (re-timed) target are stale.
constexpr int64_t kEarlyUs = 1000;
constexpr int64_t kPreloadDeadlineUs = 20000;
constexpr int64_t kGripDeadlineUs = 30000;

struct CellCmd {
  uint8_t command_id;
  uint8_t param_u8;
};

struct VisionItem {
  L2VisionDetectPayload detect;
  int64_t receipt_us;
  int64_t latency_us;  // < 0: cv_vis_ms
  uint32_t epoch;
  bool capture_valid;
  bool simulated;
};

struct PickConfig {
  PickIntercept::FingerGeom fingers;
  PickIntercept::CellGeom cells[PickIntercept::kClassCount];
  float jaw_override_s[PickIntercept::kClassCount] = {};
  PickIntercept::MotionLimits limits;
  float valve_s = 0.0f;
  float guard_s = 0.0f;
  float close_s = 0.0f;
  bool dropin = false;
  float col_x_min = 0.0f;
  float z_belt = 0.0f;
  float z_safe = 0.0f;
  float z_pick1 = 0.0f;
  float stroke = 0.0f;
  float jaw_offset = 0.0f;
  float x_off = 0.0f;
  float cam_y = 0.0f;
  float pick_y = 0.0f;
};

struct ZPlan {
  float grip = 0.0f;
  float hover = 0.0f;
  float top = 0.0f;
  float top_s = 0.0f;
  float tail_s = 0.0f;
};

struct PickEval {
  PickIntercept::StrikeSchedule strike;
  PickIntercept::StrikeTiming timing;
  Gantry::JointConfig approach;
  Gantry::JointConfig hover;
  Gantry::JointConfig pick;
  ZPlan z;
  float jaw_world_deg = 0.0f;
  float approach_deg_s = PickIntercept::kApproachSlowDegS;
  double s_arrive_mm = 0.0;
  bool permanent = false;
  const char* reason = "ok";
};

struct StrikeCtl {
  PickIntercept::StrikeTiming timing;
  float jaw_world_deg = 0.0f;
  uint8_t battery_class = 1;
  bool z_pending = false;
  bool grip_pending = false;
  int64_t z_at_us = 0;
  int64_t grip_at_us = 0;
  int64_t arrive_us = 0;
  float contact_mm_s = 0.0f;
};

static QueueHandle_t s_cmd_queue = nullptr;
static QueueHandle_t s_vision_queue = nullptr;
static PickIntercept::BeltClock s_belt;
static PickIntercept::PartTracker s_parts;
static PickIntercept::TrackedPart s_active;
static bool s_active_valid = false;
static portMUX_TYPE s_belt_mux = portMUX_INITIALIZER_UNLOCKED;
static float s_last_cycle_ms = 0.0f;
static int64_t s_status_us = 0;
static int64_t s_hold_log_us = 0;
static std::atomic<bool> s_halt_motion{false};
static bool s_cell_run = false;
static bool s_cell_paused = false;
static Gantry::Gantry* s_gantry = nullptr;
static std::atomic<bool> s_cell_in_jaws{false};

static TaskHandle_t s_strike_task = nullptr;
static esp_timer_handle_t s_z_timer = nullptr;
static esp_timer_handle_t s_grip_timer = nullptr;
static std::atomic<bool> s_strike_armed{false};
static std::atomic<uint8_t> s_strike_bits{0};
static std::atomic<int64_t> s_z_fired_us{0};
static std::atomic<int64_t> s_grip_fired_us{0};
static std::atomic<const char*> s_strike_abort{nullptr};
static StrikeCtl s_ctl;
static portMUX_TYPE s_strike_mux = portMUX_INITIALIZER_UNLOCKED;

static double s_cam_mark_mm = 0.0;
static bool s_cam_marked = false;
static float s_clamped_pick_z = -1.0f;

static PickIntercept::BeltClock copyBelt() {
  portENTER_CRITICAL(&s_belt_mux);
  const PickIntercept::BeltClock belt = s_belt;
  portEXIT_CRITICAL(&s_belt_mux);
  return belt;
}

static StrikeCtl copyCtl() {
  portENTER_CRITICAL(&s_strike_mux);
  const StrikeCtl ctl = s_ctl;
  portEXIT_CRITICAL(&s_strike_mux);
  return ctl;
}

static void strikeNotify(uint8_t bit) {
  s_strike_bits.fetch_or(bit, std::memory_order_acq_rel);
  if (s_strike_task != nullptr) {
    xTaskNotifyGive(s_strike_task);
  }
}

static void zTimerCb(void*) { strikeNotify(kStrikeZ); }
static void gripTimerCb(void*) { strikeNotify(kStrikeGrip); }

static void stopTimers() {
  if (s_z_timer != nullptr) {
    esp_timer_stop(s_z_timer);
  }
  if (s_grip_timer != nullptr) {
    esp_timer_stop(s_grip_timer);
  }
}

static void disarmStrike() {
  s_strike_armed.store(false, std::memory_order_release);
  stopTimers();
  portENTER_CRITICAL(&s_strike_mux);
  s_ctl.z_pending = false;
  s_ctl.grip_pending = false;
  portEXIT_CRITICAL(&s_strike_mux);
  s_strike_bits.store(0, std::memory_order_release);
}

static void strikeAbort(const char* why) {
  const char* none = nullptr;
  s_strike_abort.compare_exchange_strong(none, why, std::memory_order_acq_rel);
  disarmStrike();
}

static void armOne(esp_timer_handle_t timer, int64_t fire_us, uint8_t bit) {
  const int64_t delay = fire_us - esp_timer_get_time();
  if (timer == nullptr || delay <= 1000) {
    strikeNotify(bit);
    return;
  }
  esp_timer_stop(timer);
  esp_timer_start_once(timer, static_cast<uint64_t>(delay));
}

static void armStrike(const PickIntercept::StrikeTiming& timing, float jaw_world_deg,
                      uint8_t battery_class, bool z_pending, int64_t z_at, int64_t grip_at,
                      int64_t arrive_at) {
  disarmStrike();
  s_strike_abort.store(nullptr, std::memory_order_release);
  s_z_fired_us.store(0, std::memory_order_release);
  s_grip_fired_us.store(0, std::memory_order_release);
  portENTER_CRITICAL(&s_strike_mux);
  s_ctl = StrikeCtl{};
  s_ctl.timing = timing;
  s_ctl.jaw_world_deg = jaw_world_deg;
  s_ctl.battery_class = battery_class;
  s_ctl.z_pending = z_pending;
  s_ctl.grip_pending = true;
  s_ctl.z_at_us = z_at;
  s_ctl.grip_at_us = grip_at;
  s_ctl.arrive_us = arrive_at;
  portEXIT_CRITICAL(&s_strike_mux);
  s_strike_armed.store(true, std::memory_order_release);
  if (z_pending) {
    armOne(s_z_timer, z_at, kStrikeZ);
  }
  armOne(s_grip_timer, grip_at, kStrikeGrip);
}

static void retimeArmed(int64_t now) {
  StrikeCtl ctl = copyCtl();
  if (!ctl.z_pending && !ctl.grip_pending) {
    return;
  }
  const PickIntercept::Retime r = PickIntercept::retimePick(ctl.timing, copyBelt(), now);
  if (!r.ok) {
    strikeAbort(r.reason);
    return;
  }
  portENTER_CRITICAL(&s_strike_mux);
  s_ctl.arrive_us = r.arrive_us;
  s_ctl.contact_mm_s = r.contact_mm_s;
  if (s_ctl.z_pending) {
    s_ctl.z_at_us = r.release_us;
  }
  if (s_ctl.grip_pending) {
    s_ctl.grip_at_us = r.grip_us;
  }
  ctl = s_ctl;
  portEXIT_CRITICAL(&s_strike_mux);
  if (ctl.z_pending) {
    armOne(s_z_timer, ctl.z_at_us, kStrikeZ);
  }
  if (ctl.grip_pending) {
    armOne(s_grip_timer, ctl.grip_at_us, kStrikeGrip);
  }
}

static bool takeDue(bool is_z, int64_t now) {
  bool due = false;
  portENTER_CRITICAL(&s_strike_mux);
  if (is_z) {
    due = s_ctl.z_pending && now + kEarlyUs >= s_ctl.z_at_us;
    if (due) {
      s_ctl.z_pending = false;
    }
  } else {
    due = s_ctl.grip_pending && now + kEarlyUs >= s_ctl.grip_at_us;
    if (due) {
      s_ctl.grip_pending = false;
    }
  }
  portEXIT_CRITICAL(&s_strike_mux);
  return due;
}

static void strikeTask(void*) {
  while (true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    for (;;) {
      const uint8_t bits = s_strike_bits.exchange(0, std::memory_order_acq_rel);
      if (bits == 0) {
        break;
      }
      if (!s_strike_armed.load(std::memory_order_acquire) || s_gantry == nullptr) {
        continue;
      }
      const int64_t now = esp_timer_get_time();
      if ((bits & kStrikeRetime) != 0) {
        retimeArmed(now);
      } else if ((bits & (kStrikeZ | kStrikeGrip)) != 0) {
        // No frame since the last retime: the belt may have gone stale or stopped.
        const PickIntercept::Retime r =
            PickIntercept::retimePick(copyCtl().timing, copyBelt(), now);
        if (!r.ok) {
          strikeAbort(r.reason);
          continue;
        }
      }
      if ((bits & kStrikeZ) != 0 && takeDue(true, now)) {
        if (s_gantry->releasePreloadedMotion()) {
          s_z_fired_us.store(now, std::memory_order_release);
        } else {
          strikeAbort("Z release failed");
        }
      }
      if ((bits & kStrikeGrip) != 0 && takeDue(false, now)) {
        if (s_gantry->grip(true)) {
          s_cell_in_jaws.store(true, std::memory_order_release);
          s_grip_fired_us.store(esp_timer_get_time(), std::memory_order_release);
        } else {
          strikeAbort("grip write failed");
        }
      }
    }
  }
}

static void noteBelt(const PickIntercept::ConveyorFrame& frame) {
  portENTER_CRITICAL(&s_belt_mux);
  const PickIntercept::BeltNote note = PickIntercept::noteConveyor(s_belt, frame);
  if (note == PickIntercept::BeltNote::Restart) {
    PickIntercept::trackClear(s_parts);
  }
  portEXIT_CRITICAL(&s_belt_mux);
  if (note == PickIntercept::BeltNote::Restart) {
    ESP_LOGW(TAG, "conveyor clock restarted — queued parts dropped");
  } else if (note == PickIntercept::BeltNote::Fallback) {
    ESP_LOGW(TAG, "conveyor displacement_m is flat while moving — integrating speed");
  }
  if (s_strike_armed.load(std::memory_order_acquire)) {
    strikeNotify(kStrikeRetime);
  }
}

static void clearParts() {
  portENTER_CRITICAL(&s_belt_mux);
  PickIntercept::trackClear(s_parts);
  s_active_valid = false;
  portEXIT_CRITICAL(&s_belt_mux);
}

static uint32_t u32(float v) {
  if (v < 1.0f) {
    return 1;
  }
  if (v > 100000.0f) {
    return 100000;
  }
  return static_cast<uint32_t>(v + 0.5f);
}

static float clampPickZ(float configured, float stroke) {
  if (configured <= stroke) {
    return configured;
  }
  float z = PickIntercept::kBenchPickZMm;
  if (z > stroke) {
    z = stroke;
  }
  if (s_clamped_pick_z != configured) {
    s_clamped_pick_z = configured;
    ESP_LOGW(TAG, "cv_z_pick_mm %.1f is past the %.1f mm stroke; using %.1f",
             static_cast<double>(configured), static_cast<double>(stroke),
             static_cast<double>(z));
  }
  return z;
}

static PickConfig pickConfig(const Gantry::JointLimits& lim) {
  const auto& c = Config::AppConfig::instance().data();
  PickConfig p;
  p.fingers.open_mm = c.pick_finger_open_mm;
  p.fingers.thick_mm = c.pick_finger_thick_mm;
  p.fingers.width_mm = c.pick_finger_width_mm;
  p.fingers.jaw_mm_s = c.pick_jaw_speed_mm_s;
  for (int i = 0; i < PickIntercept::kClassCount; ++i) {
    p.cells[i].length_mm = c.pick_cell_len_mm[i];
    p.cells[i].height_mm = c.pick_cell_h_mm[i];
    p.jaw_override_s[i] = static_cast<float>(c.pick_jaw_ms[i]) * 0.001f;
  }
  // Stored pk_spd_mm_s / pk_th_spd are the old single profile. The approach
  // is the slow pair unless evaluate() raises it for a later part.
  const auto slow = PickIntercept::approachPair(false);
  p.limits.speed = slow.x_mm_s;
  p.limits.accel = static_cast<float>(c.pick_accel_mm_s2 < 1 ? 1 : c.pick_accel_mm_s2);
  p.limits.decel = p.limits.accel;
  p.limits.theta_cap = slow.theta_deg_s;
  if (c.axis_theta_max_speed_deg_per_s > 0.0f &&
      c.axis_theta_max_speed_deg_per_s < p.limits.theta_cap) {
    p.limits.theta_cap = c.axis_theta_max_speed_deg_per_s;
  }
  p.limits.theta_accel = c.pick_theta_accel_deg_s2;
  p.limits.theta_decel = c.pick_theta_accel_deg_s2;
  p.limits.z_cap = static_cast<float>(AXIS_Z_SPEED_CAP_FROM_CRITICAL_RPM_MM_PER_S);
  if (p.limits.z_cap < 1.0f) {
    p.limits.z_cap = PickIntercept::kZCriticalMmS;
  }
  p.limits.theta_start_frac = 0.0f;
  p.valve_s = static_cast<float>(c.pick_valve_ms) * 0.001f;
  p.guard_s = static_cast<float>(c.pick_guard_ms) * 0.001f;
  p.close_s = static_cast<float>(c.gantry_gripper_close_time_ms) * 0.001f;
  p.dropin = c.pick_dropin_enable;
  p.col_x_min = c.gantry_conveyor_collision_x_min_mm;
  p.z_belt = c.gantry_conveyor_collision_z_min_mm;
  p.z_safe = c.gantry_safe_z_height_mm;
  p.stroke = (lim.z_max > 1.0f && lim.z_max < PickIntercept::kMeasuredStrokeMm)
                 ? lim.z_max
                 : PickIntercept::kMeasuredStrokeMm;
  p.z_pick1 = clampPickZ(c.conveyor_z_pick_joint_mm, p.stroke);
  p.jaw_offset = c.pick_jaw_offset_deg;
  p.x_off = c.conveyor_x_across_to_gantry_x_offset_mm;
  p.cam_y = c.conveyor_y_cam_mm;
  p.pick_y = c.conveyor_y_pick_mm;
  return p;
}

// Grasp is half the height down from the object top, capped at 25.4 mm below
// that top. The hover clears the tallest tracked cell unless the fingers
// already hover at hover_at_mm.
static ZPlan zPlan(const PickConfig& p, float object_h_mm, float tallest_mm,
                   float hover_at_mm = -1.0f) {
  ZPlan z;
  z.grip = PickIntercept::graspJointZMm(p.z_belt, object_h_mm);
  z.top = p.z_belt - object_h_mm;
  z.hover = p.z_belt - tallest_mm - PickIntercept::kHoverClearanceMm;
  if (hover_at_mm >= 0.0f) {
    z.hover = hover_at_mm;
  }
  if (z.hover < p.z_safe) {
    z.hover = p.z_safe;
  }
  const float v = p.limits.z_cap > 1.0f ? p.limits.z_cap : p.limits.speed;
  z.top_s = PickIntercept::kTickS +
            PickIntercept::zToTopS(z.hover, z.top, z.grip, v, p.limits.accel, p.limits.decel);
  z.tail_s = PickIntercept::kTickS +
             PickIntercept::profileSeconds(z.grip - z.hover, v, p.limits.accel, p.limits.decel);
  return z;
}

static float partHeightMm(const PickConfig& p, const PickIntercept::TrackedPart& part) {
  if (part.height_from_belt_mm > 0.0f) {
    return part.height_from_belt_mm;
  }
  return p.cells[PickIntercept::classIndex(part.battery_class)].height_mm;
}

static float tallestMm(const PickConfig& p, const PickIntercept::TrackedPart* others, int n,
                       const PickIntercept::TrackedPart& self) {
  float tallest = partHeightMm(p, self);
  for (int i = 0; i < n; ++i) {
    const float h = partHeightMm(p, others[i]);
    if (h > tallest) {
      tallest = h;
    }
  }
  return tallest;
}

static PickIntercept::StrikeTiming strikeTiming(const PickConfig& p, int cls,
                                                const PickIntercept::GripGeometry& grip,
                                                const ZPlan& z, uint32_t epoch,
                                                double s_arrive_mm) {
  PickIntercept::StrikeTiming t;
  t.mode = grip.mode;
  t.epoch = epoch;
  t.s_arrive_mm = s_arrive_mm;
  t.side_gap_mm = grip.side_gap_mm;
  t.along_factor = grip.along_factor;
  t.jaw_mm_s = p.fingers.jaw_mm_s;
  t.jaw_override_s = p.jaw_override_s[cls];
  t.valve_s = p.valve_s;
  t.guard_s = p.guard_s;
  t.z_top_s = z.top_s;
  t.z_tail_s = z.tail_s;
  return t;
}

static void markLate(PickEval& eval, const char* why) {
  if (eval.strike.feasible) {
    eval.strike.feasible = false;
    eval.reason = why;
  }
}

// frozen_joint: jaw joint already committed (Z below the band); the pose is
// then the hover / depth and the budget starts there.
static PickEval evaluate(const PickConfig& p, const Gantry::JointConfig& pose,
                         const Gantry::JointLimits& lim, const PickIntercept::TrackedPart& part,
                         const PickIntercept::BeltClock& belt, int64_t now_us,
                         const PickIntercept::TrackedPart* others, int n_others,
                         const float* frozen_joint) {
  PickEval eval;
  const int cls = PickIntercept::classIndex(part.battery_class);
  eval.s_arrive_mm = part.s_arrive_mm;
  if (const char* size_bad =
          PickIntercept::visionSizeReject(part.width_mm, part.length_mm, part.height_from_belt_mm,
                                          p.fingers.open_mm)) {
    eval.strike.reason = size_bad;
    eval.reason = size_bad;
    eval.permanent = true;
    return eval;
  }
  const PickIntercept::CellGeom cell =
      PickIntercept::applyVisionSize(p.cells[cls], part.width_mm, part.length_mm,
                                    part.height_from_belt_mm);
  const bool vision_width = part.width_mm > 0.0f;

  float jaw_joint = 0.0f;
  if (frozen_joint != nullptr) {
    jaw_joint = *frozen_joint;
  } else {
    const PickIntercept::JawChoice choice = PickIntercept::chooseSideJaw(
        pose.theta, part.theta_deg, p.jaw_offset, lim.theta_min, lim.theta_max);
    if (!choice.ok) {
      eval.strike.reason = "theta range";
      eval.reason = "theta range";
      eval.permanent = true;
      return eval;
    }
    jaw_joint = Gantry::Path::commandThetaDeg(pose.theta, choice.joint_deg);
  }
  eval.jaw_world_deg = jaw_joint - p.jaw_offset;

  PickIntercept::Occupant occ[PickIntercept::kTrackerCap];
  int n_occ = 0;
  for (int i = 0; i < n_others && n_occ < PickIntercept::kTrackerCap; ++i) {
    if (part.token != 0 && others[i].token == part.token) {
      continue;
    }
    occ[n_occ].dx_mm = others[i].x_across_mm - part.x_across_mm;
    occ[n_occ].remain_mm = others[i].s_arrive_mm - PickIntercept::beltPositionMm(belt, now_us);
    occ[n_occ].cell_deg = others[i].theta_deg;
    occ[n_occ].cell = PickIntercept::applyVisionSize(
        p.cells[PickIntercept::classIndex(others[i].battery_class)], others[i].width_mm,
        others[i].length_mm, others[i].height_from_belt_mm);
    ++n_occ;
  }

  eval.z = zPlan(p, partHeightMm(p, part), tallestMm(p, others, n_others, part),
                 frozen_joint != nullptr ? pose.z : -1.0f);
  const float tcp_x = p.x_off + part.x_across_mm;
  eval.approach = s_gantry->inverseKinematics(Gantry::EndEffectorPose(
      tcp_x, 0.0f, p.z_safe + GANTRY_Z_DATUM_OFFSET_ABOVE_BED_MM, jaw_joint));
  eval.hover = s_gantry->inverseKinematics(Gantry::EndEffectorPose(
      tcp_x, 0.0f, eval.z.hover + GANTRY_Z_DATUM_OFFSET_ABOVE_BED_MM, jaw_joint));
  eval.pick = s_gantry->inverseKinematics(Gantry::EndEffectorPose(
      tcp_x, 0.0f, eval.z.grip + GANTRY_Z_DATUM_OFFSET_ABOVE_BED_MM, jaw_joint));

  const PickIntercept::GripGeometry grip =
      PickIntercept::classifyClearance(part.theta_deg, eval.jaw_world_deg, cell, p.fingers, p.dropin);
  eval.timing = strikeTiming(p, cls, grip, eval.z, belt.epoch, part.s_arrive_mm);
  if (vision_width) {
    eval.timing.jaw_override_s = 0.0f;
  }

  const bool dropin = grip.mode == PickIntercept::Clearance::LateZ;
  PickIntercept::MotionLimits motion = p.limits;
  const auto budgetAt = [&](const PickIntercept::MotionLimits& m) {
    if (frozen_joint != nullptr) {
      return PickIntercept::budgetPick(pose.x, pose.z, pose.theta, pose.x, pose.z, pose.z,
                                       eval.pick.z, pose.theta, m);
    }
    return PickIntercept::budgetPick(pose.x, pose.z, pose.theta, eval.approach.x, eval.approach.z,
                                     dropin ? eval.hover.z : eval.approach.z, eval.pick.z,
                                     eval.approach.theta, m);
  };
  PickIntercept::MotionBudget budget = budgetAt(motion);
  eval.approach_deg_s = motion.theta_cap;

  const double s_now = PickIntercept::beltPositionMm(belt, now_us);
  const float arrive_s = PickIntercept::secondsToPlane(
      static_cast<float>(part.s_arrive_mm - s_now), belt.speed_mm_s);
  eval.strike = PickIntercept::planPick(arrive_s, belt.speed_mm_s, budget, grip, eval.timing);
  eval.reason = eval.strike.reason;
  if (eval.strike.feasible && frozen_joint == nullptr) {
    const float retract_s =
        PickIntercept::profileSeconds(std::fabs(eval.pick.z - p.z_safe), motion.z_cap, motion.accel,
                                      motion.decel) +
        PickIntercept::kPreloadS + PickIntercept::kSettleS;
    const float free_s = arrive_s + p.close_s + retract_s;
    bool later = false;
    for (int i = 0; i < n_others; ++i) {
      if (part.token != 0 && others[i].token == part.token) {
        continue;
      }
      const float other_arrive = PickIntercept::secondsToPlane(
          static_cast<float>(others[i].s_arrive_mm - s_now), belt.speed_mm_s);
      const PickIntercept::JawChoice next_jaw = PickIntercept::chooseSideJaw(
          jaw_joint, others[i].theta_deg, p.jaw_offset, lim.theta_min, lim.theta_max);
      const float dth = next_jaw.ok ? next_jaw.joint_deg - jaw_joint : 180.0f;
      const float dx = (p.x_off + others[i].x_across_mm) - eval.approach.x;
      const float next_approach = PickIntercept::slowApproachSeconds(
          dx, dth, motion.accel, motion.decel, motion.theta_accel, motion.theta_decel);
      if (PickIntercept::laterPartNeedsFastApproach(free_s, other_arrive, next_approach)) {
        later = true;
        break;
      }
    }
    if (later) {
      const auto fast = PickIntercept::approachPair(true);
      motion.speed = fast.x_mm_s;
      motion.theta_cap = fast.theta_deg_s;
      const float axis_th = Config::AppConfig::instance().data().axis_theta_max_speed_deg_per_s;
      if (axis_th > 0.0f && axis_th < motion.theta_cap) {
        motion.theta_cap = axis_th;
      }
      budget = budgetAt(motion);
      eval.approach_deg_s = motion.theta_cap;
      eval.strike = PickIntercept::planPick(arrive_s, belt.speed_mm_s, budget, grip, eval.timing);
      eval.reason = eval.strike.reason;
    }
  }
  if (!eval.strike.feasible && grip.mode == PickIntercept::Clearance::Skip) {
    eval.permanent = true;
  } else if (!eval.strike.feasible &&
             std::strcmp(eval.strike.reason, "too fast for this angle") == 0) {
    eval.permanent = true;
  }

  const Gantry::JointConfig& lowest = eval.pick;
  if (lowest.z > p.stroke + 0.05f || lowest.z < lim.z_min || eval.z.grip > p.z_belt) {
    eval.strike.feasible = false;
    eval.reason = "pick Z unreachable";
    eval.permanent = true;
  } else if (PickIntercept::inConveyorCollision(lowest.x, lowest.z, p.col_x_min, p.z_belt) ||
             PickIntercept::inConveyorCollision(eval.approach.x, eval.approach.z, p.col_x_min,
                                                p.z_belt)) {
    eval.strike.feasible = false;
    eval.reason = "collision";
    eval.permanent = true;
  }
  if (!eval.strike.feasible) {
    return eval;
  }

  const float t_up = arrive_s + p.close_s + budget.z_motion_s;
  if (dropin) {
    const float t_below = eval.strike.z_start_s + eval.z.top_s;
    const float free_s = PickIntercept::fingersFreeFromS(occ, n_occ, eval.jaw_world_deg,
                                                         p.fingers, belt.speed_mm_s, t_below, t_up);
    if (free_s > t_below + 1.0e-3f) {
      markLate(eval, "neighbour");
    }
  } else {
    const float free_s = PickIntercept::fingersFreeFromS(
        occ, n_occ, eval.jaw_world_deg, p.fingers, belt.speed_mm_s, eval.strike.z_start_s, t_up);
    if (free_s > eval.strike.z_start_s) {
      eval.strike.z_start_s = free_s;
      eval.strike.z_done_s = free_s + eval.strike.z_motion_s;
      eval.strike.slack_s = arrive_s - p.guard_s - eval.strike.z_done_s;
      if (eval.strike.slack_s < 0.0f) {
        markLate(eval, "neighbour");
      }
    }
  }
  return eval;
}

static void formatEval(const PickEval& eval, char* out, size_t n) {
  std::snprintf(out, n,
                "%s jaw %.1f arrive %.3fs prepos %.3fs z_start %.3fs z_done %.3fs grip %.3fs "
                "slack %.3fs window %.3fs xgap %.1fmm side %.1fmm contact %.0fmm/s vz %.0f "
                "joint %.1f %.1f %.1f %s",
                PickIntercept::clearanceName(eval.strike.mode),
                static_cast<double>(eval.jaw_world_deg),
                static_cast<double>(eval.strike.arrive_s),
                static_cast<double>(eval.strike.approach_s),
                static_cast<double>(eval.strike.z_start_s),
                static_cast<double>(eval.strike.z_done_s),
                static_cast<double>(eval.strike.grip_s),
                static_cast<double>(eval.strike.slack_s),
                static_cast<double>(eval.strike.window_s >= PickIntercept::kWaitS
                                        ? -1.0f
                                        : eval.strike.window_s),
                static_cast<double>(eval.strike.x_clearance_mm),
                static_cast<double>(eval.strike.side_gap_mm),
                static_cast<double>(eval.strike.contact_mm_s),
                static_cast<double>(eval.strike.descend_speed_mm_s),
                static_cast<double>(eval.pick.x), static_cast<double>(eval.pick.z),
                static_cast<double>(eval.pick.theta), eval.reason);
}

static int copyTracked(PickIntercept::TrackedPart* out) {
  portENTER_CRITICAL(&s_belt_mux);
  const int n = PickIntercept::trackCopy(s_parts, out, PickIntercept::kTrackerCap);
  portEXIT_CRITICAL(&s_belt_mux);
  return n;
}

static bool copyActive(PickIntercept::TrackedPart* out) {
  portENTER_CRITICAL(&s_belt_mux);
  const bool valid = s_active_valid;
  if (valid) {
    *out = s_active;
  }
  portEXIT_CRITICAL(&s_belt_mux);
  return valid;
}

static float dedupeMm() {
  const float d = Config::AppConfig::instance().data().pick_dedupe_mm;
  return d < 5.0f ? 40.0f : d;
}

// A repeat detection of the cell being picked refines its timing; the jaw is
// frozen once armed, so an angle that no longer fits aborts the strike.
static bool refreshActive(const PickIntercept::TrackedPart& part, const PickConfig& p) {
  PickIntercept::TrackedPart active;
  bool hit = false;
  portENTER_CRITICAL(&s_belt_mux);
  if (s_active_valid &&
      PickIntercept::sameCell(part.item_id, part.x_across_mm, part.s_arrive_mm, s_active.item_id,
                              s_active.x_across_mm, s_active.s_arrive_mm, dedupeMm())) {
    s_active.s_capture_mm = part.s_capture_mm;
    s_active.s_arrive_mm = part.s_arrive_mm;
    s_active.x_across_mm = part.x_across_mm;
    s_active.theta_deg = part.theta_deg;
    s_active.width_mm = part.width_mm;
    s_active.length_mm = part.length_mm;
    s_active.height_from_belt_mm = part.height_from_belt_mm;
    active = s_active;
    hit = true;
  }
  portEXIT_CRITICAL(&s_belt_mux);
  if (!hit) {
    return false;
  }
  if (!s_strike_armed.load(std::memory_order_acquire)) {
    return true;
  }
  const StrikeCtl ctl = copyCtl();
  const int cls = PickIntercept::classIndex(ctl.battery_class);
  const PickIntercept::GripGeometry g = PickIntercept::classifyClearance(
      active.theta_deg, ctl.jaw_world_deg, p.cells[cls], p.fingers, p.dropin);
  if (g.mode == PickIntercept::Clearance::Skip ||
      (ctl.timing.mode == PickIntercept::Clearance::EarlyZ &&
       g.mode != PickIntercept::Clearance::EarlyZ)) {
    strikeAbort("refined angle");
    return true;
  }
  portENTER_CRITICAL(&s_strike_mux);
  s_ctl.timing.s_arrive_mm = active.s_arrive_mm;
  s_ctl.timing.side_gap_mm = g.side_gap_mm;
  s_ctl.timing.along_factor = g.along_factor;
  portEXIT_CRITICAL(&s_strike_mux);
  strikeNotify(kStrikeRetime);
  return true;
}

static void acceptDetect(const VisionItem& item) {
  if (s_gantry == nullptr) {
    return;
  }
  const char* source = item.simulated ? "picksim" : "vision";
  const L2VisionDetectPayload& detect = item.detect;
  const PickIntercept::BeltClock belt = copyBelt();
  if (!belt.valid) {
    return;
  }
  const auto& c = Config::AppConfig::instance().data();
  const int64_t latency_us = item.latency_us >= 0
                                 ? item.latency_us
                                 : static_cast<int64_t>(c.pick_vision_latency_ms) * 1000;
  const double s_capture = PickIntercept::capturePositionMm(
      belt, item.receipt_us, latency_us, item.capture_valid, detect.capture_displacement_m);
  PickIntercept::TrackedPart part;
  part.item_id = detect.item_id;
  part.epoch = item.epoch;
  part.x_across_mm = detect.x_across_mm;
  part.y_bat_mm = detect.y_bat_mm;
  part.theta_deg = detect.theta_deg;
  part.width_mm = detect.width_mm;
  part.length_mm = detect.length_mm;
  part.height_from_belt_mm = detect.height_from_belt_mm;
  part.battery_class = detect.battery_class == 0 ? 1 : detect.battery_class;
  part.s_capture_mm = s_capture;
  part.s_arrive_mm = PickIntercept::arriveBeltMm(s_capture, c.conveyor_y_pick_mm,
                                                 c.conveyor_y_cam_mm, detect.y_bat_mm);

  const Gantry::JointLimits lim = s_gantry->getJointLimits();
  const PickConfig p = pickConfig(lim);
  if (part.epoch == belt.epoch && refreshActive(part, p)) {
    return;
  }

  const int64_t now = esp_timer_get_time();
  PickIntercept::TrackedPart others[PickIntercept::kTrackerCap];
  const int n = copyTracked(others);
  const bool busy = s_gantry->isBusy() || s_active_valid;
  const PickEval eval = evaluate(p, s_gantry->getCurrentJointConfig(), lim, part, belt, now,
                                 others, n, nullptr);
  const bool time_ok = !eval.permanent && eval.strike.arrive_s > 0.0f &&
                       (busy || eval.strike.feasible);
  const double s_now = PickIntercept::beltPositionMm(belt, now);
  portENTER_CRITICAL(&s_belt_mux);
  const PickIntercept::TrackAdd added =
      PickIntercept::trackAdd(s_parts, part, s_now, dedupeMm(), time_ok, belt.epoch);
  portEXIT_CRITICAL(&s_belt_mux);
  if (added == PickIntercept::TrackAdd::Added) {
    ESP_LOGI(TAG, "%s queued item=%lu x=%.1f y=%.1f th=%.1f %s jaw %.1f arrive %.3fs",
             source, static_cast<unsigned long>(detect.item_id),
             static_cast<double>(detect.x_across_mm), static_cast<double>(detect.y_bat_mm),
             static_cast<double>(detect.theta_deg), PickIntercept::clearanceName(eval.strike.mode),
             static_cast<double>(eval.jaw_world_deg), static_cast<double>(eval.strike.arrive_s));
    return;
  }
  if (added == PickIntercept::TrackAdd::Duplicate || added == PickIntercept::TrackAdd::Taken) {
    return;
  }
  ESP_LOGW(TAG, "%s rejected item=%lu (%s)", source, static_cast<unsigned long>(detect.item_id),
           added == PickIntercept::TrackAdd::Outside ? "outside window"
           : added == PickIntercept::TrackAdd::Full  ? "tracker full"
           : added == PickIntercept::TrackAdd::Epoch ? "conveyor restart"
                                                     : eval.reason);
}

static void processVision() {
  if (s_vision_queue == nullptr) {
    return;
  }
  VisionItem item;
  while (xQueueReceive(s_vision_queue, &item, 0) == pdTRUE) {
    acceptDetect(item);
  }
}

static bool queueVision(const VisionItem& item) {
  if (s_vision_queue == nullptr || xQueueSend(s_vision_queue, &item, 0) != pdTRUE) {
    return false;
  }
  return true;
}

static uint32_t beltEpoch() {
  portENTER_CRITICAL(&s_belt_mux);
  const uint32_t epoch = s_belt.epoch;
  portEXIT_CRITICAL(&s_belt_mux);
  return epoch;
}

bool pickSchedulerInject(float x_across_mm, float y_bat_mm, float theta_deg, float width_mm,
                         float length_mm, float height_from_belt_mm) {
  if (!s_cell_run || s_cell_paused) {
    ESP_LOGW(TAG, "picksim ignored: cell is not running");
    return false;
  }
  if (!copyBelt().valid) {
    ESP_LOGW(TAG, "picksim ignored: no conveyor clock yet");
    return false;
  }
  VisionItem item = {};
  item.detect.x_across_mm = x_across_mm;
  item.detect.y_bat_mm = y_bat_mm;
  item.detect.theta_deg = theta_deg;
  item.detect.width_mm = width_mm;
  item.detect.length_mm = length_mm;
  item.detect.height_from_belt_mm = height_from_belt_mm;
  item.detect.battery_class = 1;
  item.receipt_us = esp_timer_get_time();
  item.latency_us = 0;
  item.epoch = beltEpoch();
  item.simulated = true;
  if (!queueVision(item)) {
    ESP_LOGW(TAG, "picksim ignored: vision queue full");
    return false;
  }
  return true;
}

bool pickSchedulerPlan(float x_across_mm, float y_bat_mm, float theta_deg, char* out,
                       size_t out_len, float width_mm, float length_mm,
                       float height_from_belt_mm) {
  if (out == nullptr || out_len == 0) {
    return false;
  }
  out[0] = '\0';
  if (s_gantry == nullptr) {
    std::snprintf(out, out_len, "pick planner not running");
    return false;
  }
  const PickIntercept::BeltClock belt = copyBelt();
  if (!belt.valid) {
    std::snprintf(out, out_len, "no conveyor clock yet");
    return false;
  }
  const auto& c = Config::AppConfig::instance().data();
  const int64_t now = esp_timer_get_time();
  PickIntercept::TrackedPart part;
  part.x_across_mm = x_across_mm;
  part.y_bat_mm = y_bat_mm;
  part.theta_deg = theta_deg;
  part.width_mm = width_mm;
  part.length_mm = length_mm;
  part.height_from_belt_mm = height_from_belt_mm;
  part.epoch = belt.epoch;
  part.s_capture_mm = PickIntercept::beltPositionMm(belt, now);
  part.s_arrive_mm = PickIntercept::arriveBeltMm(part.s_capture_mm, c.conveyor_y_pick_mm,
                                                 c.conveyor_y_cam_mm, y_bat_mm);
  const Gantry::JointLimits lim = s_gantry->getJointLimits();
  PickIntercept::TrackedPart others[PickIntercept::kTrackerCap];
  const int n = copyTracked(others);
  const PickEval eval = evaluate(pickConfig(lim), s_gantry->getCurrentJointConfig(), lim, part,
                                 belt, now, others, n, nullptr);
  formatEval(eval, out, out_len);
  return true;
}

bool pickSchedulerCover(uint8_t battery_class, char* out, size_t out_len) {
  if (out == nullptr || out_len == 0) {
    return false;
  }
  out[0] = '\0';
  if (s_gantry == nullptr) {
    std::snprintf(out, out_len, "pick planner not running");
    return false;
  }
  const PickIntercept::BeltClock belt = copyBelt();
  const auto& c = Config::AppConfig::instance().data();
  const float v = belt.valid ? belt.speed_mm_s : c.pick_default_belt_speed_mm_s;
  const Gantry::JointLimits lim = s_gantry->getJointLimits();
  const PickConfig p = pickConfig(lim);
  const int cls = PickIntercept::classIndex(battery_class);
  const ZPlan z = zPlan(p, p.cells[cls].height_mm, p.cells[cls].height_mm);
  size_t used = static_cast<size_t>(std::snprintf(
      out, out_len, "v=%.0f cls%d P=pick S=skip:",
      static_cast<double>(v), cls + 1));
  for (int deg = -90; deg < 90 && used + 8 < out_len; deg += 5) {
    const float cell_deg = static_cast<float>(deg);
    const PickIntercept::JawChoice jaw =
        PickIntercept::chooseSideJaw(0.0f, cell_deg, p.jaw_offset, lim.theta_min, lim.theta_max);
    char mode = 'S';
    if (jaw.ok) {
      const PickIntercept::GripGeometry g = PickIntercept::classifyClearance(
          cell_deg, jaw.joint_deg - p.jaw_offset, p.cells[cls], p.fingers, p.dropin);
      if (g.mode != PickIntercept::Clearance::Skip) {
        const PickIntercept::StrikeLeads leads =
            PickIntercept::strikeLeads(strikeTiming(p, cls, g, z, 0, 0.0), v);
        mode = leads.ok ? 'P' : 'S';
      }
    }
    used += static_cast<size_t>(std::snprintf(out + used, out_len - used, " %d:%c", deg, mode));
  }
  return true;
}

bool pickSchedulerCalibrate(const char* args, char* out, size_t out_len) {
  if (args == nullptr || out == nullptr || out_len == 0) {
    return false;
  }
  out[0] = '\0';
  auto& app = Config::AppConfig::instance();
  bool reboot = false;
  if (std::strncmp(args, "valve ", 6) == 0) {
    int ms = 0;
    if (std::sscanf(args, "valve %d", &ms) != 1 ||
        !app.setParamInt("cv_valve_ms", ms, reboot)) {
      std::snprintf(out, out_len, "usage: pickcal valve <ms>");
      return false;
    }
    std::snprintf(out, out_len, "valve delay %d ms", ms);
    return true;
  }
  if (std::strncmp(args, "jaw ", 4) == 0) {
    int cls = 0;
    int ms = 0;
    const char* keys[4] = {"cv_jaw1_ms", "cv_jaw2_ms", "cv_jaw3_ms", "cv_jaw4_ms"};
    if (std::sscanf(args, "jaw %d %d", &cls, &ms) != 2 || cls < 1 || cls > 4 ||
        !app.setParamInt(keys[cls - 1], ms, reboot)) {
      std::snprintf(out, out_len, "usage: pickcal jaw <class 1-4> <ms, 0 = from jaw travel>");
      return false;
    }
    std::snprintf(out, out_len, "class %d jaw contact %d ms%s", cls, ms,
                  ms == 0 ? " (derived from jaw travel)" : "");
    return true;
  }
  if (std::strncmp(args, "jawoff ", 7) == 0) {
    float deg = 0.0f;
    if (std::sscanf(args, "jawoff %f", &deg) != 1 ||
        !app.setParamFloat("cv_jaw_deg", deg, reboot)) {
      std::snprintf(out, out_len, "usage: pickcal jawoff <deg>");
      return false;
    }
    std::snprintf(out, out_len, "jaw offset %.1f deg", static_cast<double>(deg));
    return true;
  }
  if (std::strcmp(args, "cam mark") == 0) {
    const PickIntercept::BeltClock belt = copyBelt();
    if (!belt.valid) {
      std::snprintf(out, out_len, "no conveyor clock yet");
      return false;
    }
    s_cam_mark_mm = PickIntercept::beltPositionMm(belt, esp_timer_get_time());
    s_cam_marked = true;
    std::snprintf(out, out_len, "camera mark at belt %.1f mm — advance until the part is under the jaws, then pickcal cam apply",
                  s_cam_mark_mm);
    return true;
  }
  if (std::strcmp(args, "cam apply") == 0) {
    if (!s_cam_marked) {
      std::snprintf(out, out_len, "pickcal cam mark first");
      return false;
    }
    const PickIntercept::BeltClock belt = copyBelt();
    if (!belt.valid) {
      std::snprintf(out, out_len, "no conveyor clock yet");
      return false;
    }
    const double now_mm = PickIntercept::beltPositionMm(belt, esp_timer_get_time());
    const double travel = now_mm - s_cam_mark_mm;
    if (travel < 50.0) {
      std::snprintf(out, out_len, "belt only moved %.1f mm", travel);
      return false;
    }
    const float y_pick = app.data().conveyor_y_pick_mm;
    const float y_cam = y_pick - static_cast<float>(travel);
    if (!app.setParamFloat("cv_cam_y_mm", y_cam, reboot)) {
      std::snprintf(out, out_len, "camera Y %.1f rejected", static_cast<double>(y_cam));
      return false;
    }
    s_cam_marked = false;
    std::snprintf(out, out_len,
                  "camera Y %.1f mm (%0.1f mm of belt). TCP Y offset is %.1f mm; set cv_pick_y_mm if the jaws are not on the gantry plane",
                  static_cast<double>(y_cam), travel,
                  static_cast<double>(app.data().gantry_z_axis_y_offset_mm));
    return true;
  }
  std::snprintf(out, out_len, "usage: pickcal valve|jaw|jawoff|cam mark|cam apply");
  return false;
}

static void publishGantryStatus(PickSchedulerTaskConfig *cfg) {
  if (cfg == nullptr || cfg->net_l2 == nullptr || !cfg->net_l2->isReady()) {
    return;
  }
  const int64_t now_us = esp_timer_get_time();
  if (now_us - s_status_us < 500000) {
    return;
  }
  s_status_us = now_us;

  const Gantry::JointConfig joints = cfg->gantry->getCurrentJointConfig();
  uint8_t motion = 0;
  uint16_t fault = 0;
  if (cfg->gantry->isAlarmActive()) {
    motion = 4;
    fault = 1;
  } else if (cfg->gantry->eipBringUpInProgress()) {
    motion = 1;
  } else if (cfg->gantry->isBusy()) {
    motion = cfg->gantry->isGripperActive() ? 3 : 2;
  }
  cfg->net_l2->sendGantryStatus(
      motion, 0, joints.x, joints.z, joints.theta, s_last_cycle_ms, fault);
}

static bool running() {
  return !s_halt_motion.load() && s_cell_run;
}

static void idleSlice(PickSchedulerTaskConfig* cfg, int ms) {
  vTaskDelay(pdMS_TO_TICKS(ms));
  processVision();
  publishGantryStatus(cfg);
}

static bool delayMs(PickSchedulerTaskConfig* cfg, int ms) {
  int left = ms;
  while (left > 0) {
    if (!running()) {
      return false;
    }
    const int slice = left > 20 ? 20 : left;
    idleSlice(cfg, slice);
    left -= slice;
  }
  return running();
}

static bool waitMotion(PickSchedulerTaskConfig *cfg) {
  while (cfg->gantry->isBusy()) {
    if (!running()) {
      disarmStrike();
      cfg->gantry->requestAbort();
      return false;
    }
    idleSlice(cfg, 10);
  }
  return running();
}

static bool moveJoint(PickSchedulerTaskConfig* cfg, const Gantry::JointConfig& joint,
                      float speed, float accel, float decel, float theta_speed,
                      float theta_accel, float theta_start_frac) {
  if (cfg->gantry->isAlarmActive()) {
    ESP_LOGW(TAG, "axis alarm — move skipped");
    return false;
  }
  const auto err = cfg->gantry->moveTo(joint, u32(speed), u32(theta_speed), u32(accel),
                                       u32(decel), u32(theta_accel), u32(theta_accel),
                                       theta_start_frac);
  if (err != Gantry::GantryError::OK) {
    ESP_LOGW(TAG, "joint move rejected (%d) x=%.2f z=%.2f th=%.2f", static_cast<int>(err),
             static_cast<double>(joint.x), static_cast<double>(joint.z),
             static_cast<double>(joint.theta));
    return false;
  }
  if (!waitMotion(cfg)) {
    return false;
  }
  const Gantry::JointConfig now = cfg->gantry->getCurrentJointConfig();
  if (!PickIntercept::jointsArrived(now.x, now.z, now.theta, joint.x, joint.z, joint.theta)) {
    ESP_LOGW(TAG, "move ended at x=%.2f z=%.2f th=%.2f, want %.2f %.2f %.2f",
             static_cast<double>(now.x), static_cast<double>(now.z), static_cast<double>(now.theta),
             static_cast<double>(joint.x), static_cast<double>(joint.z),
             static_cast<double>(joint.theta));
    return false;
  }
  return true;
}

static bool moveApproach(PickSchedulerTaskConfig* cfg, const Gantry::JointConfig& joint,
                         const PickConfig& p, float x_mm_s, float theta_deg_s) {
  return moveJoint(cfg, joint, x_mm_s, p.limits.accel, p.limits.decel, theta_deg_s,
                   p.limits.theta_accel, p.limits.theta_start_frac);
}

static bool moveZ(PickSchedulerTaskConfig* cfg, const Gantry::JointConfig& joint,
                  const PickConfig& p, float z_mm_s) {
  return moveJoint(cfg, joint, z_mm_s, p.limits.accel, p.limits.decel, p.limits.theta_cap,
                   p.limits.theta_accel, 0.0f);
}

static bool retract(PickSchedulerTaskConfig* cfg, const PickConfig& p) {
  const auto err = cfg->gantry->retractToSafeZ(u32(p.limits.z_cap), u32(p.limits.accel),
                                               u32(p.limits.decel));
  if (err != Gantry::GantryError::OK) {
    ESP_LOGE(TAG, "Z retract refused (%d)", static_cast<int>(err));
    return false;
  }
  while (cfg->gantry->isBusy()) {
    idleSlice(cfg, 10);
  }
  return cfg->gantry->zInTraverseBand();
}

static void abortStrike(PickSchedulerTaskConfig* cfg, const PickConfig& p, const char* why) {
  disarmStrike();
  const bool gripped = s_cell_in_jaws.load(std::memory_order_acquire);
  ESP_LOGW(TAG, "strike aborted (%s) — retracting Z%s", why,
           gripped ? " with the jaws closed" : "");
  cfg->gantry->requestAbort();
  if (!retract(cfg, p)) {
    ESP_LOGE(TAG, "Z is not in the SAFE_Z band after abort");
  }
}

static const char* strikeAbortReason() {
  return s_strike_abort.load(std::memory_order_acquire);
}

static bool dropHeld(PickSchedulerTaskConfig* cfg, const PickConfig& p, float approach_mm_s,
                     float approach_deg_s) {
  const auto& c = Config::AppConfig::instance().data();
  const Gantry::JointConfig here = cfg->gantry->getCurrentJointConfig();
  const float theta = c.pick_drop_any_theta ? here.theta : c.gantry_cal_theta_park_deg;
  const Gantry::JointConfig drop_safe(c.gantry_cal_x_park_mm, c.gantry_safe_z_height_mm, theta);
  const Gantry::JointConfig drop(c.gantry_cal_x_park_mm, c.gantry_cal_z_park_mm, theta);
  if (PickIntercept::inConveyorCollision(drop.x, drop.z, c.gantry_conveyor_collision_x_min_mm,
                                         c.gantry_conveyor_collision_z_min_mm)) {
    ESP_LOGW(TAG, "bin drop violates the conveyor envelope; gripper stays closed");
    return false;
  }
  if (!retract(cfg, p) ||
      !moveApproach(cfg, drop_safe, p, approach_mm_s, approach_deg_s) ||
      !moveZ(cfg, drop, p, p.limits.z_cap)) {
    ESP_LOGW(TAG, "transfer failed; gripper stays closed");
    return false;
  }
  ESP_LOGI(TAG, "At bin drop X=%.2f mm Z=%.2f mm theta=%.2f deg. Opening gripper.",
           static_cast<double>(drop.x), static_cast<double>(drop.z),
           static_cast<double>(drop.theta));
  if (!cfg->gantry->grip(false)) {
    ESP_LOGE(TAG, "gripper open write failed at the bin");
    return false;
  }
  s_cell_in_jaws.store(false, std::memory_order_release);
  return delayMs(cfg, c.gantry_gripper_open_time_ms);
}

static bool takeNextPart(PickIntercept::TrackedPart* out) {
  const PickIntercept::BeltClock belt = copyBelt();
  const int64_t now = esp_timer_get_time();
  if (!belt.valid || PickIntercept::beltStale(belt, now)) {
    return false;
  }
  portENTER_CRITICAL(&s_belt_mux);
  const double s_now = PickIntercept::beltPositionMm(s_belt, now);
  PickIntercept::trackDropDownstream(s_parts, s_now);
  PickIntercept::TrackedPart copy[PickIntercept::kTrackerCap];
  const int n = PickIntercept::trackCopy(s_parts, copy, PickIntercept::kTrackerCap);
  portEXIT_CRITICAL(&s_belt_mux);

  const Gantry::JointLimits lim = s_gantry->getJointLimits();
  const PickConfig p = pickConfig(lim);
  const Gantry::JointConfig pose = s_gantry->getCurrentJointConfig();
  const PickIntercept::TrackedPart* best = nullptr;
  for (int i = 0; i < n; ++i) {
    if (copy[i].epoch != belt.epoch) {
      continue;
    }
    const PickEval eval = evaluate(p, pose, lim, copy[i], belt, now, copy, n, nullptr);
    if (!eval.strike.feasible) {
      if (eval.permanent || eval.strike.arrive_s <= 0.0f ||
          std::strcmp(eval.reason, "late") == 0) {
        portENTER_CRITICAL(&s_belt_mux);
        PickIntercept::trackRemove(s_parts, copy[i].token);
        portEXIT_CRITICAL(&s_belt_mux);
        ESP_LOGW(TAG, "pick item=%lu dropped (%s)", static_cast<unsigned long>(copy[i].item_id),
                 eval.reason);
      }
      continue;
    }
    if (best == nullptr || copy[i].s_arrive_mm < best->s_arrive_mm) {
      best = &copy[i];
    }
  }
  if (best == nullptr) {
    return false;
  }
  *out = *best;
  portENTER_CRITICAL(&s_belt_mux);
  PickIntercept::trackRemove(s_parts, best->token);
  PickIntercept::trackNoteTaken(s_parts, *best);
  s_active = *best;
  s_active_valid = true;
  portEXIT_CRITICAL(&s_belt_mux);
  return true;
}

static PickEval evaluateActive(const PickConfig& p, const Gantry::JointLimits& lim,
                               const PickIntercept::TrackedPart& fallback,
                               const float* frozen_joint) {
  PickIntercept::TrackedPart part = fallback;
  (void)copyActive(&part);
  PickIntercept::TrackedPart others[PickIntercept::kTrackerCap];
  const int n = copyTracked(others);
  return evaluate(p, s_gantry->getCurrentJointConfig(), lim, part, copyBelt(),
                  esp_timer_get_time(), others, n, frozen_joint);
}

// Waits for the strike to finish its Z move. False on abort, halt, or a held
// preload that was not released in time.
static bool waitStrikeZ(PickSchedulerTaskConfig* cfg, bool held) {
  while (cfg->gantry->isBusy()) {
    if (!running()) {
      disarmStrike();
      cfg->gantry->requestAbort();
      return false;
    }
    if (strikeAbortReason() != nullptr) {
      return false;
    }
    if (held && cfg->gantry->isPreloadHeld()) {
      const StrikeCtl ctl = copyCtl();
      if (esp_timer_get_time() > ctl.z_at_us + kPreloadDeadlineUs) {
        strikeAbort("preload deadline");
        return false;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(2));
    processVision();
  }
  return running() && strikeAbortReason() == nullptr;
}

static void runPick(PickSchedulerTaskConfig* cfg, const PickIntercept::TrackedPart& part) {
  const auto& c = Config::AppConfig::instance().data();
  const int64_t started = esp_timer_get_time();
  const Gantry::JointLimits lim = cfg->gantry->getJointLimits();
  const PickConfig p = pickConfig(lim);

  PickEval eval = evaluateActive(p, lim, part, nullptr);
  if (!eval.strike.feasible) {
    ESP_LOGW(TAG, "pick item=%lu skipped (%s)", static_cast<unsigned long>(part.item_id),
             eval.reason);
    return;
  }
  char plan[360];
  formatEval(eval, plan, sizeof(plan));
  ESP_LOGI(TAG, "pick item=%lu %s", static_cast<unsigned long>(part.item_id), plan);

  if (!retract(cfg, p) ||
      !moveApproach(cfg, eval.approach, p, eval.strike.approach_speed_mm_s, eval.approach_deg_s)) {
    return;
  }

  eval = evaluateActive(p, lim, part, nullptr);
  if (eval.strike.feasible) {
    const Gantry::JointConfig here = cfg->gantry->getCurrentJointConfig();
    if (std::fabs(here.theta - eval.approach.theta) > PickIntercept::kArriveEpsDeg) {
      if (!moveApproach(cfg, eval.approach, p, eval.strike.approach_speed_mm_s,
                        eval.approach_deg_s)) {
        return;
      }
      eval = evaluateActive(p, lim, part, nullptr);
    }
  }
  if (!eval.strike.feasible) {
    ESP_LOGW(TAG, "pick item=%lu missed during pre-position (%s)",
             static_cast<unsigned long>(part.item_id), eval.reason);
    return;
  }

  const bool dropin = eval.strike.mode == PickIntercept::Clearance::LateZ;
  const float jaw_joint = cfg->gantry->getCurrentJointConfig().theta;
  if (dropin) {
    if (!moveZ(cfg, eval.hover, p, eval.strike.descend_speed_mm_s)) {
      (void)retract(cfg, p);
      return;
    }
    eval = evaluateActive(p, lim, part, &jaw_joint);
    if (!eval.strike.feasible) {
      abortStrike(cfg, p, eval.reason);
      return;
    }
  } else {
    const int64_t z_go = esp_timer_get_time() +
                         static_cast<int64_t>((eval.strike.z_start_s - PickIntercept::kPreloadS) *
                                              1.0e6f);
    while (esp_timer_get_time() < z_go) {
      if (!running()) {
        return;
      }
      idleSlice(cfg, 2);
    }
    eval = evaluateActive(p, lim, part, &jaw_joint);
    if (!eval.strike.feasible) {
      ESP_LOGW(TAG, "pick item=%lu missed before Z (%s)",
               static_cast<unsigned long>(part.item_id), eval.reason);
      return;
    }
  }

  const int64_t t0 = esp_timer_get_time();
  const int64_t z_at = t0 + static_cast<int64_t>(eval.strike.z_start_s * 1.0e6f);
  const int64_t grip_at = t0 + static_cast<int64_t>(eval.strike.grip_s * 1.0e6f);
  const int64_t arrive_at = t0 + static_cast<int64_t>(eval.strike.arrive_s * 1.0e6f);
  const Gantry::JointConfig here = cfg->gantry->getCurrentJointConfig();
  const Gantry::JointConfig strike(here.x, eval.pick.z, here.theta);

  bool held = false;
  if (dropin) {
    const auto pre = cfg->gantry->preloadJointMove(
        strike, u32(eval.strike.descend_speed_mm_s), u32(p.limits.accel), u32(p.limits.decel));
    held = pre == Gantry::GantryError::OK && cfg->gantry->isPreloadHeld();
    if (!held) {
      abortStrike(cfg, p, "Z preload did not hold StartMotion");
      return;
    }
  }
  armStrike(eval.timing, eval.jaw_world_deg, part.battery_class, dropin, z_at, grip_at, arrive_at);
  if (!dropin) {
    const auto err = cfg->gantry->moveTo(strike, u32(eval.strike.descend_speed_mm_s),
                                         u32(p.limits.theta_cap), u32(p.limits.accel),
                                         u32(p.limits.decel), u32(p.limits.theta_accel),
                                         u32(p.limits.theta_accel), 0.0f);
    if (err != Gantry::GantryError::OK) {
      abortStrike(cfg, p, "Z move rejected");
      return;
    }
  }

  if (!waitStrikeZ(cfg, held)) {
    const char* why = strikeAbortReason();
    if (why != nullptr) {
      abortStrike(cfg, p, why);
    }
    return;
  }
  const int64_t z_depth_us = esp_timer_get_time();
  const Gantry::JointConfig at = cfg->gantry->getCurrentJointConfig();
  if (!PickIntercept::jointsArrived(at.x, at.z, at.theta, strike.x, strike.z, strike.theta)) {
    abortStrike(cfg, p, "Z ended off target");
    return;
  }
  StrikeCtl ctl = copyCtl();
  if (z_depth_us > ctl.arrive_us - static_cast<int64_t>(p.guard_s * 1.0e6f)) {
    abortStrike(cfg, p, "Z late");
    return;
  }

  while (s_grip_fired_us.load(std::memory_order_acquire) == 0) {
    ctl = copyCtl();
    if (!running()) {
      disarmStrike();
      return;
    }
    if (strikeAbortReason() != nullptr) {
      abortStrike(cfg, p, strikeAbortReason());
      return;
    }
    if (esp_timer_get_time() > ctl.grip_at_us + kGripDeadlineUs) {
      abortStrike(cfg, p, "grip did not fire");
      return;
    }
    vTaskDelay(pdMS_TO_TICKS(2));
    processVision();
  }
  const int64_t grip_us = s_grip_fired_us.load(std::memory_order_acquire);
  const int64_t z_us = s_z_fired_us.load(std::memory_order_acquire);
  ctl = copyCtl();
  disarmStrike();
  const int64_t jaws_done = grip_us + static_cast<int64_t>(c.gantry_gripper_close_time_ms) * 1000;
  while (esp_timer_get_time() < jaws_done) {
    if (!delayMs(cfg, 10)) {
      return;
    }
  }
  ESP_LOGI(TAG,
           "pick actual item=%lu %s z_release_err_ms=%.0f z_depth_before_arrive_ms=%.0f "
           "grip_err_ms=%.0f xgap %.1fmm side %.1fmm contact %.0fmm/s",
           static_cast<unsigned long>(part.item_id), PickIntercept::clearanceName(eval.strike.mode),
           (z_us == 0 || !dropin) ? 0.0 : static_cast<double>(z_us - z_at) / 1000.0,
           static_cast<double>(ctl.arrive_us - z_depth_us) / 1000.0,
           static_cast<double>(grip_us - grip_at) / 1000.0,
           static_cast<double>(eval.strike.x_clearance_mm),
           static_cast<double>(eval.strike.side_gap_mm),
           static_cast<double>(ctl.contact_mm_s));

  portENTER_CRITICAL(&s_belt_mux);
  s_active_valid = false;
  portEXIT_CRITICAL(&s_belt_mux);
  if (!dropHeld(cfg, p, eval.strike.approach_speed_mm_s, eval.approach_deg_s)) {
    return;
  }
  s_last_cycle_ms = static_cast<float>(esp_timer_get_time() - started) / 1000.0f;
  s_status_us = 0;
  publishGantryStatus(cfg);
}

static void applyCellCommand(PickSchedulerTaskConfig *cfg, const CellCmd &cmd) {
  switch (cmd.command_id) {
    case kCmdStart: {
      clearParts();
      s_cell_paused = false;
      s_cell_run = true;
      s_halt_motion.store(false);
      cfg->gantry->enable();
      ESP_LOGI(TAG, "CELL_COMMAND START — servos %s",
               cfg->gantry->isEnabled() ? "enabled" : "not enabled");
      const Gantry::JointLimits lim = cfg->gantry->getJointLimits();
      if (lim.theta_max - lim.theta_min < PickIntercept::kMinThetaSpanDeg) {
        ESP_LOGW(TAG, "theta span %.1f deg is under 180 — some cell angles cannot be gripped",
                 static_cast<double>(lim.theta_max - lim.theta_min));
      }
      break;
    }
    case kCmdStop:
      s_cell_run = false;
      s_cell_paused = false;
      s_halt_motion.store(true);
      disarmStrike();
      clearParts();
      cfg->gantry->requestAbort();
      cfg->gantry->disable();
      s_halt_motion.store(false);
      ESP_LOGI(TAG, "CELL_COMMAND STOP — motion stopped, motors disabled");
      break;
    case kCmdPause:
      s_cell_paused = true;
      ESP_LOGI(TAG, "CELL_COMMAND PAUSE — new picks held");
      break;
    case kCmdClearFault:
      s_halt_motion.store(true);
      if (cfg->gantry->clearAlarm()) {
        ESP_LOGI(TAG, "CELL_COMMAND CLEAR_FAULT — alarm reset sent");
      } else {
        ESP_LOGW(TAG, "CELL_COMMAND CLEAR_FAULT — alarm reset failed");
      }
      s_halt_motion.store(false);
      break;
    case kCmdPark: {
      const auto &cdata = Config::AppConfig::instance().data();
      const Gantry::JointConfig park(cdata.gantry_cal_x_park_mm, cdata.gantry_safe_z_height_mm,
                                    cdata.gantry_cal_theta_park_deg);
      const float speed = static_cast<float>(cdata.gantry_default_speed_mm_per_s);
      const float accel = static_cast<float>(cdata.gantry_path_max_accel_mm_per_s2);
      const float decel = static_cast<float>(cdata.gantry_path_max_decel_mm_per_s2);
      if (!moveJoint(cfg, park, speed, accel, decel, cdata.axis_theta_max_speed_deg_per_s,
                     cdata.axis_theta_accel_deg_per_s2, -1.0f)) {
        ESP_LOGW(TAG, "CELL_COMMAND PARK rejected");
        break;
      }
      ESP_LOGI(TAG, "CELL_COMMAND PARK — X=%.1f mm Z=%.1f mm theta=%.1f deg",
               static_cast<double>(park.x), static_cast<double>(park.z),
               static_cast<double>(park.theta));
      break;
    }
    case kCmdSetGripper: {
      const bool close = cmd.param_u8 != 0;
      if (cfg->gantry->grip(close)) {
        s_cell_in_jaws.store(close, std::memory_order_release);
        ESP_LOGI(TAG, "CELL_COMMAND SET_GRIPPER %s", close ? "close" : "open");
      } else {
        ESP_LOGW(TAG, "CELL_COMMAND SET_GRIPPER %s failed", close ? "close" : "open");
      }
      break;
    }
    default:
      ESP_LOGW(TAG, "CELL_COMMAND ignored id=%u", static_cast<unsigned>(cmd.command_id));
      break;
  }
  s_status_us = 0;
  publishGantryStatus(cfg);
}

static void drainCellCommands(PickSchedulerTaskConfig *cfg) {
  if (s_cmd_queue == nullptr) {
    return;
  }
  CellCmd cmd = {};
  while (xQueueReceive(s_cmd_queue, &cmd, 0) == pdTRUE) {
    applyCellCommand(cfg, cmd);
  }
}

// Staging: belt middle at SAFE_Z. Theta 0° puts the jaw axis along +Y, so the
// next side-grip jaw is at most 90° away.
static void goToStaging(PickSchedulerTaskConfig* cfg) {
  const auto& c = Config::AppConfig::instance().data();
  const Gantry::JointLimits lim = cfg->gantry->getJointLimits();
  float theta = 0.0f;
  if (theta < lim.theta_min) {
    theta = lim.theta_min;
  } else if (theta > lim.theta_max) {
    theta = lim.theta_max;
  }
  const Gantry::JointConfig stage(c.pick_stage_x_mm, c.gantry_safe_z_height_mm, theta);
  const Gantry::JointConfig here = cfg->gantry->getCurrentJointConfig();
  if (PickIntercept::jointsArrived(here.x, here.z, here.theta, stage.x, stage.z, stage.theta)) {
    return;
  }
  const PickConfig stage_cfg = pickConfig(lim);
  if (!retract(cfg, stage_cfg) ||
      !moveApproach(cfg, stage, stage_cfg, stage_cfg.limits.speed, stage_cfg.limits.theta_cap)) {
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void pickSchedulerTask(void *param) {
  auto *cfg = static_cast<PickSchedulerTaskConfig *>(param);
  if (cfg == nullptr || cfg->gantry == nullptr) {
    ESP_LOGE(TAG, "Invalid pick scheduler config");
    vTaskDelete(nullptr);
    return;
  }
  s_gantry = cfg->gantry;

  s_cmd_queue = xQueueCreate(8, sizeof(CellCmd));
  s_vision_queue = xQueueCreate(16, sizeof(VisionItem));
  if (s_cmd_queue == nullptr || s_vision_queue == nullptr) {
    ESP_LOGE(TAG, "Failed to create scheduler queues");
    vTaskDelete(nullptr);
    return;
  }

  const BaseType_t strike_ok = xTaskCreatePinnedToCore(
      strikeTask, "PickStrike", PICK_STRIKE_TASK_STACK, nullptr, PICK_STRIKE_TASK_PRIORITY,
      &s_strike_task, PICK_STRIKE_TASK_CORE);
  esp_timer_create_args_t z_args = {};
  z_args.callback = &zTimerCb;
  z_args.name = "pick_z";
  esp_timer_create_args_t g_args = {};
  g_args.callback = &gripTimerCb;
  g_args.name = "pick_grip";
  const bool timers_ok = strike_ok == pdPASS &&
                         esp_timer_create(&z_args, &s_z_timer) == ESP_OK &&
                         esp_timer_create(&g_args, &s_grip_timer) == ESP_OK;
  if (!timers_ok) {
    ESP_LOGE(TAG, "strike timer unavailable — picks will not run");
  }

  if (cfg->net_l2 != nullptr) {
    cfg->net_l2->onVisionDetect([](const L2VisionDetectPayload &payload, const L2CellHeader &) {
      VisionItem item = {};
      item.detect = payload;
      item.receipt_us = esp_timer_get_time();
      item.latency_us = -1;
      item.epoch = beltEpoch();
      item.capture_valid = (payload.reserved[0] & kVisionCaptureDispValid) != 0;
      (void)queueVision(item);
    });
    cfg->net_l2->onCellCommand([](const L2CellCommandPayload &payload, const L2CellHeader &hdr) {
      if (hdr.sender_id != static_cast<uint8_t>(CellNodeId::SUPERVISOR) ||
          s_cmd_queue == nullptr) {
        return;
      }
      if (payload.command_id == kCmdStop || payload.command_id == kCmdClearFault) {
        s_halt_motion.store(true);
        disarmStrike();
      }
      const CellCmd cmd = {payload.command_id, payload.param_u8};
      if (xQueueSend(s_cmd_queue, &cmd, 0) != pdTRUE) {
        ESP_LOGW(TAG, "CELL_COMMAND queue full, id=%u",
                 static_cast<unsigned>(payload.command_id));
      }
    });
    cfg->net_l2->onConveyorSpeed([](const L2ConveyorSpeedPayload &payload, const L2CellHeader &hdr) {
      PickIntercept::ConveyorFrame frame;
      frame.gantry_us = esp_timer_get_time();
      frame.timestamp_us = hdr.timestamp_us_low;
      frame.speed_mm_s = payload.speed_mm_s;
      frame.displacement_m = payload.displacement_m;
      frame.sequence = hdr.sequence;
      noteBelt(frame);
    });
  }

  ESP_LOGI(TAG, "Pick scheduler started; picks run after CELL_COMMAND START");

  while (true) {
    drainCellCommands(cfg);
    processVision();
    publishGantryStatus(cfg);

    if (!s_cell_run || s_cell_paused || !timers_ok) {
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }
    if (cfg->gantry->isAlarmActive()) {
      const int64_t now = esp_timer_get_time();
      if (now - s_hold_log_us > 2000000) {
        s_hold_log_us = now;
        ESP_LOGW(TAG, "axis alarm — picks held");
      }
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }
    if (s_cell_in_jaws.load(std::memory_order_acquire) || cfg->gantry->isGripperActive()) {
      const PickConfig p = pickConfig(cfg->gantry->getJointLimits());
      if (!dropHeld(cfg, p, p.limits.speed, p.limits.theta_cap)) {
        vTaskDelay(pdMS_TO_TICKS(1000));
      }
      continue;
    }
    const PickIntercept::BeltClock belt = copyBelt();
    if (PickIntercept::beltStale(belt, esp_timer_get_time())) {
      const int64_t now = esp_timer_get_time();
      if (belt.valid && now - s_hold_log_us > 2000000) {
        s_hold_log_us = now;
        ESP_LOGW(TAG, "conveyor frames stale — picks held");
      }
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    PickIntercept::TrackedPart part;
    if (!takeNextPart(&part)) {
      goToStaging(cfg);
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }
    if (!running() || s_cell_paused) {
      clearParts();
      continue;
    }
    runPick(cfg, part);
    portENTER_CRITICAL(&s_belt_mux);
    s_active_valid = false;
    portEXIT_CRITICAL(&s_belt_mux);
  }
}

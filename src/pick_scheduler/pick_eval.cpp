/**
 * @file pick_eval.cpp
 * @brief Pick scheduler: pick configuration, Z and strike plans, and pick evaluation.
 *
 * One of the pick_*.cpp files that make up the pick scheduler;
 * pick_scheduler_internal.h declares what they share.
 */

#include "pick_scheduler_internal.h"

namespace PickSchedulerImpl {

uint32_t u32(float v) {
  if (v < 1.0f) {
    return 1;
  }
  if (v > 100000.0f) {
    return 100000;
  }
  return static_cast<uint32_t>(v + 0.5f);
}

PickConfig pickConfig(const Gantry::JointLimits& lim) {
  const auto& c = Config::AppConfig::instance().data();
  PickConfig p;
  p.fingers.open_mm = c.pick_finger_open_mm;
  p.fingers.thick_mm = c.pick_finger_thick_mm;
  p.fingers.width_mm = c.pick_finger_width_mm;
  p.fingers.jaw_mm_s = c.pick_jaw_speed_mm_s;
  for (int i = 0; i < PickIntercept::kClassCount; ++i) {
    p.jaw_override_s[i] = static_cast<float>(c.pick_jaw_ms[i]) * 0.001f;
  }
  // Picks run on the global gantry motion profile (live). The X approach uses
  // the default speed unless a later part needs the speed ceiling; theta is
  // capped by the axis maximum.
  p.limits.speed = static_cast<float>(c.gantry_default_speed_mm_per_s);
  p.limits.fast_speed = static_cast<float>(c.gantry_path_max_speed_mm_per_s);
  p.limits.accel = static_cast<float>(c.gantry_default_accel_mm_per_s2);
  p.limits.decel = static_cast<float>(c.gantry_default_decel_mm_per_s2);
  p.limits.z_accel = c.axis_z_accel_mm_per_s2;
  p.limits.z_decel = c.axis_z_decel_mm_per_s2;
  p.limits.theta_cap = static_cast<float>(c.gantry_default_speed_deg_per_s);
  if (c.axis_theta_max_speed_deg_per_s > 0.0f &&
      c.axis_theta_max_speed_deg_per_s < p.limits.theta_cap) {
    p.limits.theta_cap = c.axis_theta_max_speed_deg_per_s;
  }
  p.limits.theta_accel = static_cast<float>(c.gantry_default_accel_deg_per_s2);
  p.limits.theta_decel = static_cast<float>(c.gantry_default_decel_deg_per_s2);
  p.limits.z_cap = AXIS_Z_PICK_SPEED_FRACTION * s_gantry->zSpeedCapMmS();
  p.limits.theta_start_frac = 0.0f;
  p.limits.theta_full_speed = true;
  p.valve_s = static_cast<float>(c.pick_valve_ms) * 0.001f;
  p.guard_s = static_cast<float>(c.pick_guard_ms) * 0.001f;
  p.close_s = static_cast<float>(c.gantry_gripper_close_time_ms) * 0.001f;
  p.valve_on_s = static_cast<float>(c.pick_valve_on_ms) * 0.001f;
  p.z_plunge_dead_s = static_cast<float>(c.pick_z_plunge_dead_ms) * 0.001f;
  p.z_retract_dead_s = static_cast<float>(c.pick_z_retract_dead_ms) * 0.001f;
  p.off_plunge_ms = static_cast<int>(c.pick_offset_plunge_ms);
  p.off_close_ms = static_cast<int>(c.pick_offset_close_ms);
  p.off_retract_ms = static_cast<int>(c.pick_offset_retract_ms);
  p.dropin = c.pick_dropin_enable;
  p.col_x_min = c.gantry_conveyor_collision_x_min_mm;
  p.z_belt = c.gantry_conveyor_collision_z_min_mm;
  p.z_safe = s_gantry->traverseClearanceZMm();
  // Z stroke measured by this session's calibration (switch-limited, so the
  // gripper just reaches the belt). Before calibration, picks are blocked
  // anyway; the joint limit is the fallback.
  const int32_t z_len_mm = s_gantry->zAxisLengthMm();
  p.stroke = z_len_mm > 0 ? static_cast<float>(z_len_mm) : lim.z_max;
  if (lim.z_max > 1.0f && lim.z_max < p.stroke) {
      p.stroke = lim.z_max;
  }
  p.jaw_offset = c.pick_jaw_offset_deg;
  p.x_off = c.conveyor_x_across_to_gantry_x_offset_mm;
  p.cam_y = c.conveyor_y_cam_mm;
  p.pick_y = c.conveyor_y_pick_mm;
  return p;
}

float zAccel(const PickConfig& p) {
  return PickIntercept::effectiveZAccel(p.limits.accel, p.limits.z_accel);
}

float zDecel(const PickConfig& p) {
  return PickIntercept::effectiveZAccel(p.limits.decel, p.limits.z_decel);
}

// Grasp: a cell under 30 mm tall is gripped 3 mm above the belt; a taller one
// half its height down from the object top, capped at 25.4 mm below that top.
// The hover clears the tallest tracked cell unless the fingers already hover
// at hover_at_mm.
ZPlan zPlan(const PickConfig& p, float object_h_mm, float tallest_mm,
                   float hover_at_mm) {
  ZPlan z;
  if (object_h_mm > 0.0f && object_h_mm < 30.0f) {
    z.grip = p.z_belt - 3.0f;
  } else {
    z.grip = PickIntercept::graspJointZMm(p.z_belt, object_h_mm);
  }
  z.top = p.z_belt - object_h_mm;
  z.hover = p.z_belt - tallest_mm - PickIntercept::kHoverClearanceMm;
  if (hover_at_mm >= 0.0f) {
    z.hover = hover_at_mm;
  }
  if (z.hover < p.z_safe) {
    z.hover = p.z_safe;
  }
  const float v = p.limits.z_cap > 1.0f ? p.limits.z_cap : p.limits.speed;
  const float za = zAccel(p);
  const float zd = zDecel(p);
  z.top_s = PickIntercept::kTickS +
            PickIntercept::zToTopS(z.hover, z.top, z.grip, v, za, zd);
  z.tail_s = PickIntercept::kTickS +
             PickIntercept::profileSeconds(z.grip - z.hover, v, za, zd);
  return z;
}

// Simulation scheduler timing (strike_plan.h). Geometry and Z limits come
// from the firmware config; the pneumatic jaw model keeps the sim constants.
StrikePlan::Plan strikePlan(const PickConfig& p, const Gantry::JointLimits& lim,
                                   float width_mm, float height_mm, float theta_deg,
                                   float belt_mm_s, float z_grasp_mm) {
    StrikePlan::Inputs in;
    in.width_mm = width_mm;
    in.height_mm = height_mm;
    in.theta_deg = theta_deg;
    in.belt_mm_s = belt_mm_s;
    in.z_belt_mm = p.z_belt;
    in.z_ceiling_mm = p.z_safe;
    in.z_min_mm = lim.z_min;
    in.z_grasp_mm = z_grasp_mm;
    in.pad_half_open_mm = 0.5 * p.fingers.open_mm;
    in.vz_mm_s = p.limits.z_cap;
    in.az_mm_s2 = zAccel(p);
    in.dz_mm_s2 = zDecel(p);
    in.valve_delay_s = p.valve_on_s;
    in.z_plunge_dead_s = p.z_plunge_dead_s;
    in.z_retract_dead_s = p.z_retract_dead_s;
    // fin_spd_mm_s: 210 mm/s = SCHUNK KGG 100-80 datasheet, 40 mm per jaw in
    // 0.19 s at 6 bar (movement only; valve and hose time are not included).
    in.jaw_speed_mm_s = p.fingers.jaw_mm_s;
    in.plunge_offset_ms = p.off_plunge_ms;
    in.close_offset_ms = p.off_close_ms;
    in.retract_offset_ms = p.off_retract_ms;
    return StrikePlan::plan(in);
}

// Strike schedule (seconds from now) for a strike fixed to arrival.
static PickIntercept::StrikeSchedule simSchedule(float arrive_s, float belt_mm_s,
                                                 const PickIntercept::MotionBudget& budget,
                                                 const PickIntercept::GripGeometry& grip,
                                                 const StrikePlan::Plan& sim) {
    PickIntercept::StrikeSchedule s;
    s.mode = grip.mode;
    s.arrive_s = arrive_s;
    s.approach_s = budget.approach_s;
    s.x_clearance_mm = grip.x_clearance_mm;
    s.side_gap_mm = grip.side_gap_mm;
    s.approach_speed_mm_s = budget.approach_speed_mm_s;
    s.descend_speed_mm_s = budget.descend_speed_mm_s;
    s.window_s = PickIntercept::kWaitS;
    if (!sim.ok) {
        s.reason = sim.reason;
        return s;
    }
    s.z_motion_s = static_cast<float>(sim.tau_plunge_s + sim.z_plunge_dead_s);
    s.z_start_s = arrive_s + static_cast<float>(sim.t_z_s);
    s.z_done_s = s.z_start_s + s.z_motion_s;
    s.grip_s = arrive_s + static_cast<float>(sim.t_close_s);
    // Preloading Z holds StartMotion=0 without moving, so it needs kPreloadS
    // only (approach_s already includes arrival settle for the pre-position).
    s.slack_s = s.z_start_s - (budget.approach_s + PickIntercept::kPreloadS);
    if (belt_mm_s <= 0.0f || arrive_s >= PickIntercept::kWaitS) {
        s.reason = "belt stopped";
        return s;
    }
    if (arrive_s <= 0.0f) {
        s.reason = "missed";
        return s;
    }
    if (budget.approach_s >= PickIntercept::kWaitS || s.slack_s < 0.0f) {
        s.reason = "late";
        return s;
    }
    s.feasible = true;
    return s;
}

static float partHeightMm(const PickIntercept::TrackedPart& part) {
  return part.height_from_belt_mm;
}

static float tallestMm(const PickIntercept::TrackedPart* others, int n,
                       const PickIntercept::TrackedPart& self) {
  float tallest = partHeightMm(self);
  for (int i = 0; i < n; ++i) {
    const float h = partHeightMm(others[i]);
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
PickEval evaluate(const PickConfig& p, const Gantry::JointConfig& pose,
                         const Gantry::JointLimits& lim, const PickIntercept::TrackedPart& part,
                         const PickIntercept::BeltClock& belt, int64_t now_us,
                         const PickIntercept::TrackedPart* others, int n_others,
                         const float* frozen_joint) {
  PickEval eval;
  eval.s_arrive_mm = part.s_arrive_mm;
  if (part.item_id == 0) {
    eval.strike.reason = "item id";
    eval.reason = "item id";
    eval.permanent = true;
    return eval;
  }
  if (const char* bad = PickIntercept::classPickReject(part.battery_class)) {
    eval.strike.reason = bad;
    eval.reason = bad;
    eval.permanent = true;
    return eval;
  }
  const int cls = PickIntercept::classIndex(part.battery_class);
  if (const char* size_bad =
          PickIntercept::visionSizeReject(part.width_mm, part.length_mm, part.height_from_belt_mm,
                                          p.fingers.open_mm)) {
    eval.strike.reason = size_bad;
    eval.reason = size_bad;
    eval.permanent = true;
    return eval;
  }
  const PickIntercept::GripAxes axes =
      PickIntercept::gripAcrossWidth(part.width_mm, part.length_mm, part.theta_deg);
  const PickIntercept::CellGeom cell =
      PickIntercept::cellFromVision(axes.width_mm, axes.length_mm, part.height_from_belt_mm);

  float jaw_joint = 0.0f;
  if (frozen_joint != nullptr) {
    jaw_joint = *frozen_joint;
  } else {
    const PickIntercept::JawChoice choice = PickIntercept::chooseSideJaw(
        pose.theta, axes.theta_deg, p.jaw_offset, lim.theta_min, lim.theta_max);
    if (!choice.ok) {
      eval.strike.reason = "theta range";
      eval.reason = "theta range";
      eval.permanent = true;
      return eval;
    }
    jaw_joint = Gantry::Path::commandThetaDeg(pose.theta, choice.joint_deg);
  }
  eval.jaw_world_deg = Gantry::thetaPoseFromJointDeg(jaw_joint);

  PickIntercept::Occupant occ[PickIntercept::kTrackerCap];
  int n_occ = 0;
  for (int i = 0; i < n_others && n_occ < PickIntercept::kTrackerCap; ++i) {
    if (part.token != 0 && others[i].token == part.token) {
      continue;
    }
    const PickIntercept::GripAxes other_grip = PickIntercept::gripAcrossWidth(
        others[i].width_mm, others[i].length_mm, others[i].theta_deg);
    occ[n_occ].dx_mm = others[i].x_across_mm - part.x_across_mm;
    occ[n_occ].remain_mm = others[i].s_arrive_mm - PickIntercept::beltPositionMm(belt, now_us);
    occ[n_occ].cell_deg = other_grip.theta_deg;
    occ[n_occ].cell = PickIntercept::cellFromVision(other_grip.width_mm, other_grip.length_mm,
                                                   others[i].height_from_belt_mm);
    ++n_occ;
  }

  eval.z = zPlan(p, partHeightMm(part), tallestMm(others, n_others, part),
                 frozen_joint != nullptr ? pose.z : -1.0f);
  const float tcp_x = p.x_off + part.x_across_mm;
  const float joint_x = s_gantry != nullptr
                            ? s_gantry->inverseKinematics(Gantry::EndEffectorPose(tcp_x, 0.0f, 0.0f, 0.0f)).x
                            : tcp_x;
  eval.pick = Gantry::JointConfig(joint_x, eval.z.grip, jaw_joint);

  // Lane and side gap are kept for the log and the neighbour test.
  const PickIntercept::GripGeometry grip = PickIntercept::classifyClearance(
      axes.theta_deg, PickIntercept::closeAxisDeg(eval.jaw_world_deg), cell, p.fingers, p.dropin);
  eval.timing = strikeTiming(p, cls, grip, eval.z, belt.epoch, part.s_arrive_mm);

  // Simulation scheduler: hover, plunge, close and retract from the vision size.
  // The pre-position is the hover itself, inside the SAFE_Z band.
  eval.sim = strikePlan(p, lim, PickIntercept::beltCrossMm(cell), cell.height_mm, axes.theta_deg,
                        belt.speed_mm_s, eval.z.grip);
  const float hover_z = eval.sim.ok ? static_cast<float>(eval.sim.z_hover_mm) : p.z_safe;
  eval.hover = Gantry::JointConfig(joint_x, hover_z, jaw_joint);
  eval.approach = eval.hover;

  PickIntercept::MotionLimits motion = p.limits;
  if (part.approach_fast && frozen_joint == nullptr) {
      motion.speed = motion.fast_speed;
  }
  const PickIntercept::MotionBudget budget =
      frozen_joint != nullptr
          ? PickIntercept::budgetPick(pose.x, pose.z, pose.theta, pose.x, pose.z, pose.z,
                                      eval.pick.z, pose.theta, motion)
          : PickIntercept::budgetPick(pose.x, pose.z, pose.theta, eval.approach.x,
                                      eval.approach.z, eval.approach.z, eval.pick.z,
                                      eval.approach.theta, motion);
  eval.approach_deg_s = motion.theta_cap;

  const double s_now = PickIntercept::beltPositionMm(belt, now_us);
  const float arrive_s = PickIntercept::secondsToPlane(
      static_cast<float>(part.s_arrive_mm - s_now), belt.speed_mm_s);
  eval.strike = simSchedule(arrive_s, belt.speed_mm_s, budget, grip, eval.sim);
  eval.reason = eval.strike.reason;
  if (!eval.sim.ok && belt.speed_mm_s > 0.0f) {
      // Size, jaw fit or Z limits: the same detection never becomes pickable.
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

  // Neighbours must stay out of the finger lanes from the plunge until Z is up.
  const float retract_s =
      PickIntercept::profileSeconds(std::fabs(eval.pick.z - eval.hover.z), p.limits.z_cap,
                                    zAccel(p), zDecel(p)) +
      PickIntercept::kPreloadS + PickIntercept::kSettleS;
  const float t_up = arrive_s + static_cast<float>(eval.sim.t_retract_s) + retract_s;
  const float free_s = PickIntercept::fingersFreeFromS(
      occ, n_occ, PickIntercept::closeAxisDeg(eval.jaw_world_deg), p.fingers, belt.speed_mm_s,
      eval.strike.z_start_s, t_up);
  if (free_s > eval.strike.z_start_s + 1.0e-3f) {
      markLate(eval, "neighbour");
  }
  return eval;
}

void formatEval(const PickEval& eval, char* out, size_t n) {
  std::snprintf(out, n,
                "%s jaw %.1f arrive %.3fs prepos %.3fs z_start %.3fs z_done %.3fs grip %.3fs "
                "slack %.3fs window %.3fs xgap %.1fmm side %.1fmm contact %.0fmm/s vz %.0f "
                "joint %.1f %.1f %.1f %s | sim z/close/retract %.1f/%.1f/%.1f ms "
                "dwell %.0f ms nom %.1f/%.1f/%.1f hover %.1f%s",
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
                static_cast<double>(eval.pick.theta), eval.reason,
                eval.sim.t_z_s * 1000.0, eval.sim.t_close_s * 1000.0, eval.sim.t_retract_s * 1000.0,
                eval.sim.dwell_s * 1000.0, eval.sim.t_z_nominal_s * 1000.0,
                eval.sim.t_close_nominal_s * 1000.0, eval.sim.t_retract_nominal_s * 1000.0,
                eval.sim.z_hover_mm,
                eval.sim.clamped ? " clamped" : "");
}

}  // namespace PickSchedulerImpl

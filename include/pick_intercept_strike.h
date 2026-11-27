/**
 * @file pick_intercept_strike.h
 * @brief Strike leads and the motion budget.
 */
#ifndef PICK_INTERCEPT_STRIKE_H
#define PICK_INTERCEPT_STRIKE_H

#include "pick_intercept_clearance.h"

namespace PickIntercept {

// ---------------------------------------------------------------------------
// Strike geometry and timing
// ---------------------------------------------------------------------------

enum class Clearance : uint8_t {
  EarlyZ,
  LateZ,
  Skip,
};

inline const char* clearanceName(Clearance mode) {
  if (mode == Clearance::EarlyZ || mode == Clearance::LateZ) {
    return "pick";
  }
  return "skip";
}

// Everything the strike controller needs to re-time from a new belt state.
struct StrikeTiming {
  Clearance mode = Clearance::Skip;
  uint32_t epoch = 0;
  double s_arrive_mm = 0.0;
  float side_gap_mm = 0.0f;
  float along_factor = 0.0f;
  float jaw_mm_s = 0.0f;
  float jaw_override_s = 0.0f;
  float valve_s = 0.0f;
  float guard_s = 0.0f;
  float z_top_s = 0.0f;
  float z_tail_s = 0.0f;
};

struct StrikeLeads {
  bool ok = false;
  float window_s = 0.0f;
  float close_s = 0.0f;
  float grip_lead_s = 0.0f;
  float release_lead_s = 0.0f;
  float contact_mm_s = 0.0f;
  const char* reason = "ok";
};

// Provisional until bench calibration. With the jaws open, the object's face
// meets the downstream finger this long before the center reaches the jaw
// plane. A jaw perpendicular to the belt (along factor 0) or a zero gap has
// no earlier belt contact; the commanded jaw close still meets the faces at
// center arrival.
inline float beltContactLeadS(float side_gap_mm, float belt_mm_s, float along_factor) {
  const float along = belt_mm_s * along_factor;
  if (!(side_gap_mm > 0.0f) || !(along > 0.0f)) {
    return 0.0f;
  }
  return side_gap_mm / along;
}

// Contact of the closing jaws is at arrival. The jaws close across g at jaw
// speed; if the belt carries the cell along the jaw faster than that, the
// downstream finger meets the cell first. Late Z is released so the fingertips
// are at z_g one guard before the earliest of those contacts, and only after
// the upstream finger is clear. The lead, hover clearance, lane margin, and
// guard are provisional.
inline StrikeLeads strikeLeads(const StrikeTiming& t, float belt_mm_s) {
  StrikeLeads l;
  const float along = belt_mm_s * t.along_factor;
  float travel = t.jaw_override_s;
  if (travel <= 0.0f) {
    travel = t.jaw_mm_s > 0.0f ? t.side_gap_mm / t.jaw_mm_s : 0.0f;
  }
  l.close_s = travel;
  if (along > 0.0f && t.side_gap_mm > 0.0f && t.side_gap_mm / along < l.close_s) {
    l.close_s = t.side_gap_mm / along;
  }
  l.grip_lead_s = t.valve_s + l.close_s;
  l.contact_mm_s = along > t.jaw_mm_s ? along + t.jaw_mm_s : t.jaw_mm_s;
  if (!(t.side_gap_mm > 0.0f)) {
    l.window_s = 0.0f;
  } else if (along > 0.0f) {
    l.window_s = 2.0f * t.side_gap_mm / along;
  } else {
    l.window_s = kWaitS;
  }
  if (t.side_gap_mm < 0.0f) {
    l.reason = "cell wider than jaws";
    return l;
  }
  if (t.side_gap_mm == 0.0f) {
    l.close_s = 0.0f;
    l.grip_lead_s = t.valve_s;
  }
  if (t.mode == Clearance::LateZ) {
    const float need = 2.0f * (t.z_tail_s - t.z_top_s + t.guard_s);
    if (l.window_s < need) {
      l.reason = "too fast for this angle";
      return l;
    }
    l.release_lead_s = t.guard_s + t.z_tail_s;
  } else if (t.mode != Clearance::EarlyZ) {
    l.reason = "no pick";
    return l;
  }
  l.ok = true;
  return l;
}

struct GripGeometry {
  Clearance mode = Clearance::Skip;
  float x_clearance_mm = 0.0f;
  float side_gap_mm = 0.0f;
  float along_factor = 0.0f;
  const char* reason = "ok";
};

// Early Z only when the X-projection misses both finger lanes. g = 0 does not
// force that: a full-width cell sitting on a finger has no hole to drop through.
inline GripGeometry classifyClearance(float cell_deg, float jaw_deg, const CellGeom& cell,
                                      const FingerGeom& f, bool late_z_enabled) {
  GripGeometry g;
  g.x_clearance_mm = xLaneClearanceMm(cell_deg, jaw_deg, cell, f);
  g.side_gap_mm = sideGapMm(cell_deg, jaw_deg, cell, f);
  g.along_factor = alongFactor(jaw_deg);
  if (g.side_gap_mm < 0.0f) {
    g.reason = "cell wider than jaws";
  } else if (g.x_clearance_mm >= kLaneMarginMm) {
    g.mode = Clearance::EarlyZ;
  } else if (!late_z_enabled) {
    g.reason = "late Z off";
  } else {
    g.mode = Clearance::LateZ;
  }
  return g;
}

// Filled from the global gantry settings (pickConfig in pick_eval.cpp).
// `speed` is the normal approach; `fast_speed` is used only when a later
// tracked part would otherwise be missed. Z uses `z_cap` / `z_accel` / `z_decel`.
struct MotionLimits {
  float speed = 0.0f;
  float fast_speed = 0.0f;
  float accel = 0.0f;
  float decel = 0.0f;
  float z_accel = 0.0f;
  float z_decel = 0.0f;
  float theta_cap = 0.0f;
  float theta_accel = 0.0f;
  float theta_decel = 0.0f;
  float z_cap = 0.0f;
  float theta_start_frac = 0.0f;
  bool theta_full_speed = false;
};

// Effective Z accel/decel honoring optional per-axis Z cap (<= 0 disables cap).
inline float effectiveZAccel(float path_accel, float z_accel_cap) {
  if (!(z_accel_cap > 0.0f)) {
    return path_accel;
  }
  if (!(path_accel > 0.0f) || z_accel_cap < path_accel) {
    return z_accel_cap;
  }
  return path_accel;
}

// True when finishing this cycle at `free_after_slow_s` and then the later
// part's own slow approach would pass that part's arrival.
inline bool laterPartNeedsFastApproach(float free_after_slow_s, float next_arrive_s,
                                       float next_slow_approach_s) {
  if (!(next_arrive_s > 0.0f) || next_arrive_s >= kWaitS) {
    return false;
  }
  return free_after_slow_s + next_slow_approach_s > next_arrive_s;
}

// Later of the X trapezoid and the theta move at one approach pair.
inline float approachSeconds(float distance_mm, float delta_theta_deg, float x_mm_s,
                             float theta_deg_s, float accel, float decel, float theta_accel,
                             float theta_decel) {
  const float t_x = profileSeconds(std::fabs(distance_mm), x_mm_s, accel, decel) + kPreloadS +
                    kSettleS;
  float t_th = 0.0f;
  if (std::fabs(delta_theta_deg) >= Gantry::Path::kThetaCommandMinDeg) {
    t_th = kTickS + kThetaStartS +
           profileSeconds(std::fabs(delta_theta_deg), theta_deg_s, theta_accel, theta_decel) +
           kSettleS;
  }
  return t_x > t_th ? t_x : t_th;
}

// Time from contact until the fingers are back in the SAFE_Z band at the bin,
// ready for the next approach. Transfer runs at the normal approach speed.
struct PostBinCycle {
  float grip_dwell_s = 0.0f;
  float retract_s = 0.0f;
  float transfer_s = 0.0f;
  float descend_s = 0.0f;
  float open_dwell_s = 0.0f;
  float bin_retract_s = 0.0f;
  float total_s = 0.0f;
};

inline PostBinCycle postBinCycle(float pick_z_mm, float pick_x_mm, float pick_theta_deg,
                                 float bin_x_mm, float z_safe_mm, float bin_z_mm,
                                 float bin_theta_deg, float grip_dwell_s, float open_dwell_s,
                                 const MotionLimits& m) {
  const float z_cap = m.z_cap;
  const float accel = m.accel;
  const float decel = m.decel;
  const float z_accel = effectiveZAccel(m.accel, m.z_accel);
  const float z_decel = effectiveZAccel(m.decel, m.z_decel);
  PostBinCycle cycle;
  cycle.grip_dwell_s = grip_dwell_s;
  cycle.retract_s = profileSeconds(std::fabs(pick_z_mm - z_safe_mm), z_cap, z_accel, z_decel) +
                    kPreloadS + kSettleS;
  cycle.transfer_s = approachSeconds(bin_x_mm - pick_x_mm, bin_theta_deg - pick_theta_deg, m.speed,
                                     m.theta_cap, accel, decel, m.theta_accel, m.theta_decel);
  cycle.descend_s = profileSeconds(std::fabs(bin_z_mm - z_safe_mm), z_cap, z_accel, z_decel) +
                    kPreloadS + kSettleS;
  cycle.open_dwell_s = open_dwell_s;
  cycle.bin_retract_s = cycle.descend_s;
  cycle.total_s = cycle.grip_dwell_s + cycle.retract_s + cycle.transfer_s + cycle.descend_s +
                  cycle.open_dwell_s + cycle.bin_retract_s;
  return cycle;
}

enum class FollowerPace : uint8_t { Slow, Fast, Skip };

// The follower is timed from the post-bin pose. Slow when that pair arrives,
// fast only when the slow pair does not, and a skip when neither does.
inline FollowerPace followerPace(float ready_s, float follower_arrive_s, float slow_approach_s,
                                 float fast_approach_s) {
  if (!(follower_arrive_s > 0.0f) || follower_arrive_s >= kWaitS) {
    return FollowerPace::Skip;
  }
  if (ready_s + slow_approach_s <= follower_arrive_s) {
    return FollowerPace::Slow;
  }
  if (ready_s + fast_approach_s <= follower_arrive_s) {
    return FollowerPace::Fast;
  }
  return FollowerPace::Skip;
}

struct MotionBudget {
  float approach_s = 0.0f;
  float linear_s = 0.0f;
  float theta_s = 0.0f;
  float hover_s = 0.0f;
  float z_motion_s = 0.0f;
  float approach_speed_mm_s = 0.0f;
  float descend_speed_mm_s = 0.0f;
  bool z_limited = false;
};

inline float segmentWallSeconds(float x0, float z0, const Gantry::Path::PathSegment& seg,
                                float ceiling, float speed, float accel, float decel,
                                float z_cap, float z_accel = 0.0f, float z_decel = 0.0f) {
  const float dx = seg.x_mm - x0;
  const float dz = seg.z_mm - z0;
  const float ax = std::fabs(dx);
  const float az = std::fabs(dz);
  const float z_speed = z_cap > 1.0f ? z_cap : speed;
  const float za = effectiveZAccel(accel, z_accel);
  const float zd = effectiveZAccel(decel, z_decel);
  const bool deferred = seg.move_x && !Gantry::Path::zInTraverseBand(z0, ceiling);

  if (!seg.move_x && !seg.move_z) {
    return 0.0f;
  }
  if (deferred && seg.move_z) {
    const float t_z = profileSeconds(az, z_speed, za, zd) + kPreloadS;
    const float to_band = std::fabs(z0 - ceiling);
    const float t_enter =
        timeAlongProfile(to_band, az, z_speed, za, zd) + kPreloadS;
    const float t_x = profileSeconds(ax, speed, accel, decel) + kPreloadS;
    const float done = t_z > (t_enter + t_x) ? t_z : (t_enter + t_x);
    return done + kSettleS;
  }
  if (!seg.move_x) {
    return profileSeconds(az, z_speed, za, zd) + kPreloadS + kSettleS;
  }
  if (!seg.move_z) {
    return profileSeconds(ax, speed, accel, decel) + kPreloadS + kSettleS;
  }
  const float path = std::hypot(ax, az);
  const float v = Gantry::Path::zCappedPathSpeed(speed, dx, dz, z_cap);
  const float a = Gantry::Path::zCappedPathAccel(accel, dx, dz, z_accel);
  const float d = Gantry::Path::zCappedPathAccel(decel, dx, dz, z_decel);
  return profileSeconds(path, v, a, d) + kPreloadS + kSettleS;
}

// Pre-position from (x0, z0, theta0) to (x1, z_safe, theta1), then an optional
// Z-only hover, then the descend to z_pick. Theta runs from the start of the
// in-band segment (Gantry theta start fraction) plus the rotary start phases,
// over the linear change; the pre-position ends when both X and theta are done.
inline MotionBudget budgetPick(float x0, float z0, float theta0, float x1, float z_safe,
                               float z_hover, float z_pick, float theta1,
                               const MotionLimits& m) {
  if (std::fabs(x1 - x0) <= kArriveEpsMm) {
    x1 = x0;
  }
  if (std::fabs(z_safe - z0) <= kArriveEpsMm) {
    z_safe = z0;
  }
  if (std::fabs(theta1 - theta0) <= kArriveEpsDeg) {
    theta1 = theta0;
  }
  MotionBudget b;
  b.approach_speed_mm_s = m.speed;

  Gantry::Path::PathSegment segs[3];
  const size_t n = Gantry::Path::planSegments(x0, z0, x1, z_safe, z_safe, segs);
  float t = 0.0f;
  float cx = x0;
  float cz = z0;
  size_t win = n;
  float win_len = 0.0f;
  float win_start = 0.0f;
  float win_speed = m.speed;
  bool win_x = false;
  const float za = effectiveZAccel(m.accel, m.z_accel);
  const float zd = effectiveZAccel(m.decel, m.z_decel);
  for (size_t i = 0; i < n; ++i) {
    const float dx = segs[i].x_mm - cx;
    const float dz = segs[i].z_mm - cz;
    const float len = std::hypot(dx, dz);
    const bool deferred = segs[i].move_x && !Gantry::Path::zInTraverseBand(cz, z_safe);
    float start = t;
    if (deferred && segs[i].move_z) {
      const float z_speed = m.z_cap > 1.0f ? m.z_cap : m.speed;
      start += timeAlongProfile(std::fabs(cz - z_safe), std::fabs(dz), z_speed, za, zd) +
               kPreloadS;
    }
    if (segs[i].move_z &&
        Gantry::Path::zCappedPathSpeed(m.speed, segs[i].move_x ? dx : 0.0f, dz, m.z_cap) + 0.5f <
            m.speed) {
      b.z_limited = true;
    }
    if (Gantry::Path::zInTraverseBand(segs[i].z_mm, z_safe) && (!win_x || segs[i].move_x)) {
      win = i;
      win_len = len;
      win_start = start;
      win_x = segs[i].move_x;
      win_speed = (segs[i].move_x && segs[i].move_z && !deferred)
                      ? Gantry::Path::zCappedPathSpeed(m.speed, dx, dz, m.z_cap)
                      : m.speed;
    }
    t += segmentWallSeconds(cx, cz, segs[i], z_safe, m.speed, m.accel, m.decel, m.z_cap,
                            m.z_accel, m.z_decel);
    cx = segs[i].x_mm;
    cz = segs[i].z_mm;
  }
  b.linear_s = t;

  const float dth = std::fabs(theta1 - theta0);
  float theta_end = 0.0f;
  if (dth >= Gantry::Path::kThetaCommandMinDeg) {
    float v_th = m.theta_cap;
    float start = 0.0f;
    if (win < n && win_len > Gantry::Path::kAxisEpsMm) {
      if (!m.theta_full_speed) {
        const Gantry::Path::ThetaWindow w =
            Gantry::Path::planThetaWindow(win_len, m.speed, dth, m.theta_cap, m.theta_start_frac);
        if (w.runnable) {
          v_th = w.speed_deg_per_s;
        }
      }
      start = win_start + timeAlongProfile(m.theta_start_frac * win_len, win_len, win_speed,
                                           m.accel, m.decel);
    }
    theta_end = start + kTickS + kThetaStartS +
                profileSeconds(dth, v_th, m.theta_accel, m.theta_decel);
    b.theta_s = theta_end;
    if (theta_end + kSettleS > t) {
      t = theta_end + kSettleS;
    }
  }
  b.approach_s = t;

  b.descend_speed_mm_s = m.z_cap > 1.0f ? m.z_cap : m.speed;
  if (b.descend_speed_mm_s + 0.5f < m.speed) {
    b.z_limited = true;
  }
  float z_from = z_safe;
  if (z_hover > z_safe + kArriveEpsMm) {
    b.hover_s = profileSeconds(z_hover - z_safe, b.descend_speed_mm_s, za, zd) +
                kPreloadS + kSettleS;
    b.approach_s += b.hover_s;
    z_from = z_hover;
  }
  b.z_motion_s = profileSeconds(std::fabs(z_pick - z_from), b.descend_speed_mm_s, za, zd);
  return b;
}


}  // namespace PickIntercept

#endif  // PICK_INTERCEPT_STRIKE_H

/**
 * @file coordinated_grasp.h
 * @brief Joint Z-release and jaw-close schedule. Host-tested against the paper grid.
 *
 * The analytic roots and the Python timestep sampler do not share this file.
 * Finger width is the close-axis pad size. Finger length runs along the pads.
 */
#ifndef COORDINATED_GRASP_H
#define COORDINATED_GRASP_H

#include <cmath>
#include <cstdint>

namespace CoordinatedGrasp {

constexpr double kOpenMm = 80.0;
constexpr double kAOpen = 40.0;
constexpr double kCloseS = 0.190;
constexpr double kQdot = 40.0 / kCloseS;
constexpr double kValveOnS = 0.008;
constexpr double kValveOffS = 0.017;
constexpr double kFingerWidthMm = 10.0;
constexpr double kFingerLengthMm = 50.0;
constexpr double kBeltZMm = 115.0;
constexpr double kSafeZMm = 35.7;
constexpr double kHoverClearMm = 3.0;
constexpr double kVzMmS = 800.0;
constexpr double kAccMmS2 = 3000.0;
constexpr double kStrokeMm = 105.5;
constexpr double kGraspCapMm = 25.4;
constexpr double kDragLimitMm = 10.0;
constexpr double kDragTightenMm = 1.0;
constexpr double kPreGraspGapMm = 0.5;
constexpr double kZUncertaintyS = 0.005;
constexpr double kCloseUncertaintyS = 0.002;

enum class Status : int {
  AnalyticClear = 0,
  Tighten = 1,
  DragExceeds = 2,
  ContactAssisted = 3,
  OutsideLimit = 4,
  InvalidInput = 5,
};

inline bool executable(Status status) {
  return status == Status::AnalyticClear || status == Status::Tighten;
}

struct Plan {
  Status status = Status::InvalidInput;
  double drag_mm = 0.0;
  double robust_s = 0.0;
  double t_close_s = 0.0;
  double t_z_s = 0.0;
  double t_jaw_s = 0.0;
  double t_grasp_s = 0.0;
  double t_first_s = 0.0;
  double t_second_s = 0.0;
};

struct Body {
  double length_mm = 0.0;
  double width_mm = 0.0;
  double height_mm = 0.0;
  double theta_deg = 0.0;
  double speed_mm_s = 0.0;
  double finger_width_mm = kFingerWidthMm;
  double finger_length_mm = kFingerLengthMm;
};

inline double jawA(double t, double tm) {
  if (t <= tm) {
    return kAOpen;
  }
  const double q = kQdot * (t - tm);
  if (q >= 40.0) {
    return 0.0;
  }
  return kAOpen - q;
}

inline double profileSeconds(double distance, double speed, double accel, double decel) {
  if (distance <= 0.0) {
    return 0.0;
  }
  if (speed <= 0.0 || accel <= 0.0 || decel <= 0.0) {
    return 1.0e6;
  }
  const double ramp = (speed * speed) * (0.5 / accel + 0.5 / decel);
  if (distance >= ramp) {
    return (distance - ramp) / speed + speed / accel + speed / decel;
  }
  const double peak = std::sqrt(2.0 * distance / (1.0 / accel + 1.0 / decel));
  return peak / accel + peak / decel;
}

inline double timeAlong(double distance, double length, double speed, double accel, double decel) {
  if (distance <= 0.0 || length <= 0.0) {
    return 0.0;
  }
  if (distance >= length) {
    return profileSeconds(length, speed, accel, decel);
  }
  if (speed <= 0.0 || accel <= 0.0 || decel <= 0.0) {
    return 1.0e6;
  }
  const double da = (speed * speed) / (2.0 * accel);
  const double dd = (speed * speed) / (2.0 * decel);
  if (da + dd < length) {
    if (distance <= da) {
      return std::sqrt(2.0 * distance / accel);
    }
    const double cruise_end = length - dd;
    if (distance <= cruise_end) {
      return speed / accel + (distance - da) / speed;
    }
    const double into = distance - cruise_end;
    const double disc = speed * speed - 2.0 * decel * into;
    const double t_dec = disc <= 0.0 ? speed / decel : (speed - std::sqrt(disc)) / decel;
    return speed / accel + (cruise_end - da) / speed + t_dec;
  }
  const double peak = std::sqrt(2.0 * length / (1.0 / accel + 1.0 / decel));
  const double d_peak = (peak * peak) / (2.0 * accel);
  if (distance <= d_peak) {
    return std::sqrt(2.0 * distance / accel);
  }
  return peak / accel + peak / decel - std::sqrt(2.0 * (length - distance) / decel);
}

inline double zHover(double height) {
  const double dropped = kBeltZMm - height - kHoverClearMm;
  return dropped > kSafeZMm ? dropped : kSafeZMm;
}

inline double zTop(double height) { return kBeltZMm - height; }

inline double zGrasp(double height) {
  const double half = 0.5 * height;
  return zTop(height) + (half < kGraspCapMm ? half : kGraspCapMm);
}

struct Plant {
  double v = 0.0;
  double sin_a = 0.0;
  double cos_a = 0.0;
  double half_w = 0.0;
  double t_along = 1.0e6;
  double t_clear = 1.0e6;
  double tau_band = 0.0;
  double tau_grasp = 0.0;
  double finger_w = kFingerWidthMm;
  bool past_stroke = false;
  bool bad = false;
};

inline Plant makePlant(const Body& body) {
  Plant p;
  p.v = body.speed_mm_s;
  p.half_w = 0.5 * body.width_mm;
  p.finger_w = body.finger_width_mm;
  const double rad = body.theta_deg * 0.017453292519943295;
  p.sin_a = std::fabs(std::sin(rad));
  p.cos_a = std::fabs(std::cos(rad));
  const double half_along = 0.5 * (body.length_mm + body.finger_length_mm);
  if (p.sin_a < 1e-12 || p.v <= 0.0) {
    p.t_clear = 1.0e6;
  } else {
    p.t_clear = (kAOpen - p.half_w) / (p.v * p.sin_a);
  }
  if (p.cos_a < 1e-12 || p.v <= 0.0) {
    p.t_along = 1.0e6;
  } else {
    p.t_along = half_along / (p.v * p.cos_a);
  }
  const double zg = zGrasp(body.height_mm);
  const double zh = zHover(body.height_mm);
  const double drop = zg - zh;
  p.tau_grasp = timeAlong(drop, drop, kVzMmS, kAccMmS2, kAccMmS2);
  const double to_top = zTop(body.height_mm) - zh;
  const double tau_top = timeAlong(to_top > 0.0 ? to_top : 0.0, drop, kVzMmS, kAccMmS2, kAccMmS2);
  p.tau_band = p.tau_grasp - tau_top;
  if (p.tau_band < 0.0) {
    p.tau_band = 0.0;
  }
  p.past_stroke = zg > kStrokeMm + 1e-6;
  p.bad = body.width_mm <= 0.0 || body.width_mm > kOpenMm || body.length_mm <= 0.0 ||
          body.height_mm <= 0.0 || body.speed_mm_s <= 0.0;
  return p;
}

inline double clipOverlap(double t0, double t1, double d0, double d1, double lo, double hi) {
  if (t1 < t0) {
    return 1.0e9;
  }
  const double span = t1 - t0;
  if (span < 1e-15) {
    return (d0 >= lo && d0 <= hi) ? t0 : 1.0e9;
  }
  const double slope = (d1 - d0) / span;
  if (std::fabs(slope) < 1e-15) {
    return (d0 >= lo && d0 <= hi) ? t0 : 1.0e9;
  }
  double enter;
  double leave;
  if (slope > 0.0) {
    enter = d0 >= lo ? t0 : t0 + (lo - d0) / slope;
    leave = d1 <= hi ? t1 : t0 + (hi - d0) / slope;
  } else {
    enter = d0 <= hi ? t0 : t0 + (hi - d0) / slope;
    leave = d1 >= lo ? t1 : t0 + (lo - d0) / slope;
  }
  if (enter < t0) {
    enter = t0;
  }
  if (leave > t1) {
    leave = t1;
  }
  return enter <= leave + 1e-12 ? enter : 1.0e9;
}

inline double pieceHit(double vsin, double half_w, double finger_w, double tm, double t0, double t1,
                       bool bilateral) {
  const auto u_of = [&](double t) { return vsin * std::fabs(t); };
  if (bilateral) {
    const double d0 = u_of(t0) + jawA(t0, tm);
    const double d1 = u_of(t1) + jawA(t1, tm);
    return clipOverlap(t0, t1, d0, d1, -1.0e9, half_w);
  }
  const double d0 = u_of(t0) - jawA(t0, tm);
  const double d1 = u_of(t1) - jawA(t1, tm);
  return clipOverlap(t0, t1, d0, d1, -half_w, finger_w + half_w);
}

inline double analyticFirst(const Plant& plant, double tm, double t0, double t1, bool bilateral) {
  const double neg = -plant.t_along;
  const double lo = t0 > neg ? t0 : neg;
  const double hi = t1 < plant.t_along ? t1 : plant.t_along;
  if (lo > hi + 1e-15) {
    return 1.0e9;
  }
  const double edge[5] = {lo, hi, 0.0, tm, tm + kCloseS};
  double best = 1.0e9;
  const double vsin = plant.v * plant.sin_a;
  for (int i = 0; i < 5; ++i) {
    const double a = edge[i];
    if (a < lo - 1e-15 || a > hi + 1e-15) {
      continue;
    }
    double nxt = hi;
    for (int j = 0; j < 5; ++j) {
      const double b = edge[j];
      if (b > a + 1e-12 && b < nxt) {
        nxt = b;
      }
    }
    if (nxt <= a + 1e-12) {
      continue;
    }
    const double hit = pieceHit(vsin, plant.half_w, plant.finger_w, tm, a, nxt, bilateral);
    if (hit < best) {
      best = hit;
    }
  }
  return best;
}

struct Capture {
  bool ok = false;
  double t1 = 0.0;
  double t2 = 0.0;
  double drag = 0.0;
};

inline Capture evaluate(const Plant& plant, double tm, double tg) {
  Capture out;
  const double t_cross = tg - plant.tau_band;
  if (analyticFirst(plant, tm, t_cross, tg - 1e-6, false) < 1.0e8) {
    return out;
  }
  const double gap = jawA(tg, tm) - plant.half_w - plant.v * std::fabs(tg) * plant.sin_a;
  if (!(gap > 1e-6 && gap <= kPreGraspGapMm + 1e-4)) {
    return out;
  }
  if (plant.cos_a >= 1e-12 && std::fabs(plant.v * tg * plant.cos_a) > plant.t_along * plant.v * plant.cos_a + 1e-6) {
    return out;
  }
  out.ok = true;
  out.t1 = tg;
  out.t2 = tg;
  out.drag = plant.v * (gap / kQdot);
  return out;
}

inline Capture constrainedCapture(const Plant& plant, double tm, double tg) {
  Capture out;
  const double t_cross = tg - plant.tau_band;
  double t1;
  if (analyticFirst(plant, tm, t_cross, tg - 1e-6, false) < 1.0e8) {
    t1 = analyticFirst(plant, tm, t_cross, t_cross + 1.5, false);
  } else {
    t1 = analyticFirst(plant, tm, tg, tg + 1.5, false);
  }
  if (t1 > 1.0e8) {
    return out;
  }
  const double a = jawA(t1, tm);
  double t2 = t1;
  if (a > plant.half_w) {
    t2 = t1 + (a - plant.half_w) / kQdot;
    const double along = plant.cos_a < 1e-12 ? 0.0 : std::fabs(plant.v * t2 * plant.cos_a);
    const double limit = plant.t_along * plant.v * plant.cos_a;
    if (plant.cos_a >= 1e-12 && along > limit + 1e-6) {
      return out;
    }
  }
  out.ok = true;
  out.t1 = t1;
  out.t2 = t2;
  out.drag = plant.v * (t2 - t1);
  return out;
}

inline double expandOk(const Plant& plant, double tm, double tg, bool shift_jaw, double direction) {
  const auto ok = [&](double shift) {
    return evaluate(plant, shift_jaw ? tm + shift : tm, shift_jaw ? tg : tg + shift).ok;
  };
  if (!ok(0.0)) {
    return 0.0;
  }
  double step = 0.002 * direction;
  double last = 0.0;
  for (int i = 0; i < 12; ++i) {
    if (ok(last + step)) {
      last += step;
      step *= 2.0;
    } else {
      break;
    }
  }
  double lo = last;
  double hi = last + step;
  for (int i = 0; i < 10; ++i) {
    const double mid = 0.5 * (lo + hi);
    if (ok(mid)) {
      lo = mid;
    } else {
      hi = mid;
    }
  }
  return lo;
}

inline Plan solve(const Body& body) {
  Plan plan;
  const Plant plant = makePlant(body);
  if (plant.past_stroke || plant.bad) {
    plan.status = Status::InvalidInput;
    return plan;
  }
  const double q_center = kAOpen - (plant.half_w + kPreGraspGapMm);
  const double tm0 = -q_center / kQdot;
  bool have = false;
  double best_drag = 0.0;
  double best_tm = tm0;
  double best_tg = 0.0;
  double best_t1 = 0.0;
  double best_t2 = 0.0;
  const auto consider = [&](double tm, double tg) {
    const Capture got = evaluate(plant, tm, tg);
    if (!got.ok) {
      return;
    }
    if (!have || std::fabs(tg) < std::fabs(best_tg) - 1e-6 ||
        (std::fabs(std::fabs(tg) - std::fabs(best_tg)) <= 1e-6 && got.drag < best_drag)) {
      have = true;
      best_drag = got.drag;
      best_tm = tm;
      best_tg = tg;
      best_t1 = got.t1;
      best_t2 = got.t2;
    }
  };
  consider(tm0, 0.0);
  const double span = plant.t_clear < plant.t_along ? plant.t_clear : plant.t_along;
  const double capped = span < 0.40 ? span : 0.40;
  const double tg_lo = -capped + plant.tau_band;
  const double tg_hi = capped;
  if (tg_hi >= tg_lo) {
    for (int i = 0; i < 17; ++i) {
      const double tg = tg_lo + (tg_hi - tg_lo) * (static_cast<double>(i) / 16.0);
      const double u = plant.v * std::fabs(tg) * plant.sin_a;
      const double a_star = plant.half_w + u + kPreGraspGapMm;
      if (a_star > kAOpen + 1e-9 || a_star < 0.0) {
        continue;
      }
      const double tm = tg - (kAOpen - a_star) / kQdot;
      consider(tm, tg);
    }
  }
  if (!have) {
    Capture assisted = constrainedCapture(plant, tm0, 0.0);
    if (!assisted.ok) {
      const double tg = tg_lo > 0.0 ? tg_lo : 0.0;
      assisted = constrainedCapture(plant, tm0, tg);
    }
    if (assisted.ok) {
      plan.status = Status::ContactAssisted;
      plan.drag_mm = assisted.drag;
      plan.t_jaw_s = tm0;
      plan.t_close_s = tm0 - kValveOnS;
      plan.t_z_s = 0.0 - plant.tau_grasp;
      plan.t_first_s = assisted.t1;
      plan.t_second_s = assisted.t2;
      plan.robust_s = -1.0;
      return plan;
    }
    plan.status = Status::OutsideLimit;
    return plan;
  }
  const double z_lo = expandOk(plant, best_tm, best_tg, false, -1.0);
  const double z_hi = expandOk(plant, best_tm, best_tg, false, 1.0);
  const double m_lo = expandOk(plant, best_tm, best_tg, true, -1.0);
  const double m_hi = expandOk(plant, best_tm, best_tg, true, 1.0);
  const double z_width = z_hi - z_lo > 0.0 ? z_hi - z_lo : 0.0;
  const double m_width = m_hi - m_lo > 0.0 ? m_hi - m_lo : 0.0;
  const double robust_z = z_width - kZUncertaintyS;
  const double robust_m = m_width - kCloseUncertaintyS;
  plan.robust_s = robust_z < robust_m ? robust_z : robust_m;
  plan.drag_mm = best_drag;
  plan.t_jaw_s = best_tm;
  plan.t_grasp_s = best_tg;
  plan.t_close_s = best_tm - kValveOnS;
  plan.t_z_s = best_tg - plant.tau_grasp;
  plan.t_first_s = best_t1;
  plan.t_second_s = best_t2;
  if (best_drag >= kDragLimitMm) {
    plan.status = Status::DragExceeds;
  } else if (best_drag >= kDragTightenMm) {
    plan.status = Status::Tighten;
  } else {
    plan.status = Status::AnalyticClear;
  }
  return plan;
}

// Bench arm. Production leaves this disarmed. pk_dropin is not consulted.
struct Arm {
  uint32_t generation = 0;
  bool armed = false;
  bool z_pending = false;
  bool close_pending = false;
  bool z_fired = false;
  bool close_fired = false;
  int64_t z_at_us = 0;
  int64_t close_at_us = 0;
  int64_t retract_at_us = 0;
  Plan plan{};
};

inline void disarm(Arm& arm) {
  arm.armed = false;
  arm.z_pending = false;
  arm.close_pending = false;
}

inline void armPair(Arm& arm, const Plan& plan, int64_t t_a_us, uint32_t generation) {
  disarm(arm);
  arm.generation = generation;
  arm.plan = plan;
  if (plan.status != Status::AnalyticClear && plan.status != Status::Tighten) {
    return;
  }
  arm.z_at_us = t_a_us + static_cast<int64_t>(std::llround(plan.t_z_s * 1.0e6));
  arm.close_at_us = t_a_us + static_cast<int64_t>(std::llround(plan.t_close_s * 1.0e6));
  arm.retract_at_us = t_a_us + static_cast<int64_t>(std::llround(plan.t_grasp_s * 1.0e6));
  arm.z_pending = true;
  arm.close_pending = true;
  arm.armed = true;
}

inline bool fireDue(Arm& arm, uint32_t generation, int64_t now_us, bool want_z) {
  if (!arm.armed || generation != arm.generation) {
    return false;
  }
  if (want_z) {
    if (!arm.z_pending || now_us + 1000 < arm.z_at_us) {
      return false;
    }
    arm.z_pending = false;
    arm.z_fired = true;
    return true;
  }
  if (!arm.close_pending || now_us + 1000 < arm.close_at_us) {
    return false;
  }
  arm.close_pending = false;
  arm.close_fired = true;
  return true;
}

}  // namespace CoordinatedGrasp

#endif

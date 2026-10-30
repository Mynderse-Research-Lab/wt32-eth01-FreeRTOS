/**
 * @file strike_plan.h
 * @brief Zero-dwell strike timing ported from the simulation scheduler.
 *
 * Port of compute_strike_plan() in tools/sim/gantry_twin.py. Times are seconds
 * relative to the cell centre reaching the pick plane (t = 0). Geometry and
 * limits come from the caller (firmware config), jaw speed included; the jaw
 * valve time constant keeps the simulation value. The three operator offsets
 * act independently.
 */

#ifndef STRIKE_PLAN_H
#define STRIKE_PLAN_H

#include <cmath>
#include <cstdint>

namespace StrikePlan {

// Pneumatic jaw model from the simulation (gantry_twin.py). The firmware passes
// the configured jaw speed (fin_spd_mm_s); 450 is only the sim default.
constexpr double kJawSpeedMmS = 450.0;
constexpr double kValveTauS = 0.015;
// Jaw gap over the width at touchdown contact (the sim's 1.8 mm invariant).
constexpr double kTargetExcessMm = 1.8;
// Retract starts this long before the jaws reach the width faces.
constexpr double kRetractLeadS = 0.008;
// Hover clears the object top by at least this much.
constexpr double kHoverClearMm = 6.0;
// The plunge crosses the top only after the cell is this far inside the gap.
constexpr double kEnterMarginMm = 2.0;
constexpr double kEnterGuardS = 0.005;
// No crossing motion (belt along the jaw): enter time used by the sim.
constexpr double kEnterAlongS = -0.5;
// Grasp depth must be reached within this long after arrival...
constexpr double kZDoneMaxS = 0.015;
// ...and before the belt carries the cell this far past the pick plane.
constexpr double kZDoneMaxBeltMm = 20.0;
// Smallest gap kept between plunge, close and retract.
constexpr double kMinGapS = 0.002;
// Operator offsets are whole milliseconds within +/- this.
constexpr int kOffsetLimitMs = 100;
constexpr int kNewtonIters = 8;
constexpr double kMinStrokeMm = 0.2;
constexpr double kMinCrossMmS = 1.0;
constexpr double kMinTopMm = 0.005;

struct Inputs {
    double width_mm = 0.0;           // jaw-close size (vision width)
    double height_mm = 0.0;          // object top above the belt
    double theta_deg = 0.0;          // cell long axis: 0° along New +X (across belt), 90° along +Y (along belt)
    double belt_mm_s = 0.0;
    double z_belt_mm = 0.0;          // joint Z of the belt surface (+Z down)
    double z_ceiling_mm = 0.0;       // SAFE_Z band ceiling (deepest hover)
    double z_min_mm = 0.0;           // joint Z min (retracted)
    double z_grasp_mm = 0.0;         // grasp depth, chosen by the caller
    double pad_half_open_mm = 0.0;   // half of the open jaw gap
    double vz_mm_s = 0.0;            // Z plunge speed cap
    double az_mm_s2 = 0.0;           // Z acceleration
    double dz_mm_s2 = 0.0;           // Z deceleration, 0 = same as az_mm_s2
    double valve_delay_s = 0.0;      // close command to jaw motion
    double valve_tau_s = kValveTauS;
    double jaw_speed_mm_s = kJawSpeedMmS;
    int plunge_offset_ms = 0;        // moves only the Z release
    int close_offset_ms = 0;         // moves only the grip close
    int retract_offset_ms = 0;       // moves only the Z retract
};

struct Plan {
    bool ok = false;
    bool clamped = false;            // an offset was limited by an invariant
    const char* reason = "ok";
    double z_hover_mm = 0.0;
    double z_grasp_mm = 0.0;
    double tau_plunge_s = 0.0;       // hover to grasp depth
    double tau_band_s = 0.0;         // part of the plunge below the object top
    double tau_full_close_s = 0.0;   // close command to jaws on the width faces
    double t_z_nominal_s = 0.0;
    double t_close_nominal_s = 0.0;
    double t_retract_nominal_s = 0.0;
    double t_z_s = 0.0;              // release the held Z plunge
    double t_close_s = 0.0;          // fire the grip valve
    double t_retract_s = 0.0;        // start the Z retract
    double dwell_s = 0.0;            // bottom dwell the offsets ask for, signed
};

/// Time-optimal trapezoid/triangle duration (physics_engines.trapezoid_duration)
/// with separate accel and decel ramps.
inline double trapezoidSeconds(double distance_mm, double v_max, double a_max, double d_max) {
    const double dist = std::fabs(distance_mm);
    if (dist < 1e-6 || v_max <= 0.0 || a_max <= 0.0 || d_max <= 0.0) {
        return 0.0;
    }
    const double d_ramps = 0.5 * v_max * v_max * (1.0 / a_max + 1.0 / d_max);
    if (dist < d_ramps) {
        const double v_peak = std::sqrt(2.0 * dist * a_max * d_max / (a_max + d_max));
        return v_peak / a_max + v_peak / d_max;
    }
    return v_max / a_max + v_max / d_max + (dist - d_ramps) / v_max;
}

/// Time for one jaw to travel stroke_mm after valve opening:
/// q(t) = v (t - tau (1 - e^(-t/tau))), solved by Newton as in the sim.
inline double jawStrokeSeconds(double stroke_mm, double speed_mm_s, double tau_s) {
    const double tau = tau_s > 0.001 ? tau_s : 0.001;
    double t = stroke_mm / speed_mm_s + tau;
    for (int i = 0; i < kNewtonIters; ++i) {
        const double e = std::exp(-t / tau);
        const double f = speed_mm_s * (t - tau * (1.0 - e)) - stroke_mm;
        const double df = speed_mm_s * (1.0 - e);
        if (std::fabs(df) > 1e-6) {
            t -= f / df;
        }
    }
    return t > 0.0 ? t : 0.0;
}

inline int clampOffsetMs(int ms, bool& clamped) {
    if (ms > kOffsetLimitMs) {
        clamped = true;
        return kOffsetLimitMs;
    }
    if (ms < -kOffsetLimitMs) {
        clamped = true;
        return -kOffsetLimitMs;
    }
    return ms;
}

inline Plan plan(const Inputs& in) {
    Plan p;
    if (!(in.width_mm > 0.0) || !(in.height_mm > 0.0)) {
        p.reason = "size";
        return p;
    }
    if (in.width_mm > 2.0 * in.pad_half_open_mm) {
        p.reason = "wider than the jaws";
        return p;
    }
    if (!(in.belt_mm_s > 0.0)) {
        p.reason = "belt stopped";
        return p;
    }
    if (in.vz_mm_s <= 0.0 || in.az_mm_s2 <= 0.0 || in.jaw_speed_mm_s <= 0.0) {
        p.reason = "Z or jaw limits";
        return p;
    }

    const double z_top = in.z_belt_mm - in.height_mm;
    double zh = z_top - kHoverClearMm;
    if (zh > in.z_ceiling_mm) {
        zh = in.z_ceiling_mm;
    }
    if (zh < in.z_min_mm) {
        zh = in.z_min_mm;
    }
    const double zg = in.z_grasp_mm;
    if (zg <= zh) {
        p.reason = "grasp above hover";
        return p;
    }
    p.z_hover_mm = zh;
    p.z_grasp_mm = zg;

    const double to_top = (z_top - zh) > kMinTopMm ? (z_top - zh) : kMinTopMm;
    const double dz = in.dz_mm_s2 > 0.0 ? in.dz_mm_s2 : in.az_mm_s2;
    p.tau_plunge_s = trapezoidSeconds(zg - zh, in.vz_mm_s, in.az_mm_s2, dz);
    p.tau_band_s = p.tau_plunge_s - std::sqrt(2.0 * to_top / in.az_mm_s2);

    double q_full = in.pad_half_open_mm - 0.5 * in.width_mm;
    if (q_full < kMinStrokeMm) {
        q_full = kMinStrokeMm;
    }
    p.tau_full_close_s =
        in.valve_delay_s + jawStrokeSeconds(q_full, in.jaw_speed_mm_s, in.valve_tau_s);
    const double dt_retract_from_close = p.tau_full_close_s - kRetractLeadS;

    // Strike window: Z crosses the top once the cell is inside the open gap
    // and reaches depth before it runs off the pads.
    // Coordinate convention: 0° is along +X (across belt), 90° is along +Y (along belt).
    // The crossing velocity into the jaw gap is therefore belt_speed * |cos(theta)|.
    const double rad = std::fabs(in.theta_deg) * (3.141592653589793 / 180.0);
    const double cos_th = std::fabs(std::cos(rad));
    double c_clear = in.pad_half_open_mm - 0.5 * in.width_mm;
    if (c_clear < 1.0) {
        c_clear = 1.0;
    }
    const double v_cross = in.belt_mm_s * cos_th;
    const double t_enter =
        v_cross > kMinCrossMmS ? -(c_clear - kEnterMarginMm) / v_cross : kEnterAlongS;
    double t_safe_cross = t_enter + kEnterGuardS;
    if (t_safe_cross < -p.tau_band_s) {
        t_safe_cross = -p.tau_band_s;
    }
    double max_z_done = kZDoneMaxBeltMm / in.belt_mm_s;
    if (max_z_done > kZDoneMaxS) {
        max_z_done = kZDoneMaxS;
    }
    double t_z_done = t_safe_cross + p.tau_band_s;
    if (t_z_done > max_z_done) {
        t_z_done = max_z_done;
    }
    if (t_z_done < 0.0) {
        t_z_done = 0.0;
    }

    p.t_z_nominal_s = t_z_done - p.tau_plunge_s;
    p.t_retract_nominal_s = t_z_done;  // zero dwell at depth
    p.t_close_nominal_s = p.t_retract_nominal_s - dt_retract_from_close;

    bool clamped = false;
    const int plunge_ms = clampOffsetMs(in.plunge_offset_ms, clamped);
    const int close_ms = clampOffsetMs(in.close_offset_ms, clamped);
    const int retract_ms = clampOffsetMs(in.retract_offset_ms, clamped);
    p.t_z_s = p.t_z_nominal_s + plunge_ms * 0.001;
    p.t_close_s = p.t_close_nominal_s + close_ms * 0.001;
    p.t_retract_s = p.t_retract_nominal_s + retract_ms * 0.001;

    // Full plunge (no mid-air reversal) and plunge < close < retract.
    const double retract_floor = p.t_z_s + p.tau_plunge_s;
    // Retract minus plunge end as asked: retract offset minus plunge offset.
    // A negative dwell is not applied; the floor below holds the retract.
    p.dwell_s = p.t_retract_s - retract_floor;
    if (p.t_retract_s < retract_floor) {
        p.t_retract_s = retract_floor;
        clamped = true;
    }
    if (p.t_close_s < p.t_z_s + kMinGapS) {
        p.t_close_s = p.t_z_s + kMinGapS;
        clamped = true;
    }
    if (p.t_close_s > p.t_retract_s - kMinGapS) {
        p.t_close_s = p.t_retract_s - kMinGapS;
        clamped = true;
    }
    p.clamped = clamped;
    p.ok = true;
    return p;
}

}  // namespace StrikePlan

#endif  // STRIKE_PLAN_H

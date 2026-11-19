// Theta trace analysis: one drive-profiled HCS01 move recorded at 1 kHz.
//
// The HCS01 runs its own trapezoid (drive-controlled positioning), so the
// firmware never sees the commanded angle per sample. The ideal reference is
// rebuilt from the commanded speed/accel/decel with TrajectoryPlanner and
// aligned to the measured accel phase (see analyze). Pure logic, host-tested.

#ifndef THETA_TRACE_ANALYSIS_H
#define THETA_TRACE_ANALYSIS_H

#include <cmath>
#include <cstdint>

#include "GantryTrajectory.h"

namespace ThetaTrace {

// The axis left its start angle.
constexpr double kMotionEpsDeg = 0.01;
// Settle band: S-0-0057 position window 0.1 deg (live `drv read t`).
constexpr double kSettleBandDeg = 0.10;
// The ideal profile is aligned at this fraction of the accel distance.
constexpr double kAlignFrac = 0.2;

struct Move {
    double dist_deg = 0.0;  // target - start
    double vel_deg_s = 0.0;
    double acc_deg_s2 = 0.0;
    double dec_deg_s2 = 0.0;
    double start_edge_us = -1.0;  // from the trace origin; < 0 = not sent
};

struct Result {
    double profile_ms = 0.0;  // ideal trapezoid duration
    int i_motion = -1;        // first sample past kMotionEpsDeg
    double t0_us = -1.0;      // estimated profile start (trace origin)
    double latency_ms = -1.0; // start edge -> estimated profile start
    int i_band = -1;          // first sample inside the settle band
    int i_settle = -1;        // samples stay inside the band from here on
    double overshoot_deg = 0.0;  // past the target, in the move direction
    int i_overshoot = -1;
    double max_follow_deg = 0.0;  // |actual - ideal| while the ideal runs
    int i_follow = -1;
    double peak_vel_deg_s = 0.0;
    int i_peak_vel = -1;
    double final_err_deg = 0.0;  // last sample - target
};

/// Ideal travelled angle (signed) t_s after the profile start.
inline double idealDeg(const Gantry::TrapezoidalProfile& p, double dist_deg, double t_s) {
    if (!p.valid) {
        return dist_deg;
    }
    return static_cast<double>(Gantry::TrajectoryPlanner::interpolate(
        p, 0.0f, static_cast<float>(dist_deg), static_cast<float>(t_s)));
}

/// t_us: sample time from the trace origin; rel_deg: actual - start angle.
inline Result analyze(const int32_t* t_us, const float* rel_deg, const float* vel_deg_s,
                      int n, const Move& m) {
    Result r;
    if (t_us == nullptr || rel_deg == nullptr || n <= 0) {
        return r;
    }
    const Gantry::TrapezoidalProfile p = Gantry::TrajectoryPlanner::calculateProfile(
        0.0f, static_cast<float>(m.dist_deg), static_cast<float>(m.vel_deg_s),
        static_cast<float>(m.acc_deg_s2), static_cast<float>(m.dec_deg_s2));
    r.profile_ms = p.valid ? static_cast<double>(p.total_time) * 1000.0 : 0.0;
    const double dir = m.dist_deg < 0.0 ? -1.0 : 1.0;

    for (int i = 0; i < n; ++i) {
        if (r.i_motion < 0 && std::fabs(static_cast<double>(rel_deg[i])) > kMotionEpsDeg) {
            r.i_motion = i;
        }
        if (vel_deg_s != nullptr &&
            std::fabs(static_cast<double>(vel_deg_s[i])) > r.peak_vel_deg_s) {
            r.peak_vel_deg_s = std::fabs(static_cast<double>(vel_deg_s[i]));
            r.i_peak_vel = i;
        }
        const double over = dir * (static_cast<double>(rel_deg[i]) - m.dist_deg);
        if (over > r.overshoot_deg) {
            r.overshoot_deg = over;
            r.i_overshoot = i;
        }
    }
    r.final_err_deg = static_cast<double>(rel_deg[n - 1]) - m.dist_deg;
    if (r.i_motion < 0) {
        return r;
    }

    // Profile start: invert s = a t^2 / 2 at the first sample past kAlignFrac of
    // the accel distance, using the measured angle there. Aligning at the
    // 0.01 deg onset instead would carry the 1 ms sample step (0.67 deg at
    // 667 deg/s) into the following error. A constant delay therefore shows
    // up as latency, not as following error.
    r.t0_us = static_cast<double>(t_us[r.i_motion]);
    if (p.valid && m.acc_deg_s2 > 0.0) {
        const double accel_dist =
            0.5 * static_cast<double>(p.max_speed) * static_cast<double>(p.t_accel);
        const double align = std::fmax(kMotionEpsDeg, kAlignFrac * accel_dist);
        for (int i = r.i_motion; i < n; ++i) {
            const double s = std::fabs(static_cast<double>(rel_deg[i]));
            if (s >= align) {
                if (s <= accel_dist) {
                    r.t0_us = static_cast<double>(t_us[i]) -
                              std::sqrt(2.0 * s / m.acc_deg_s2) * 1.0e6;
                }
                break;
            }
        }
    }
    if (m.start_edge_us >= 0.0) {
        r.latency_ms = (r.t0_us - m.start_edge_us) / 1000.0;
    }

    const double t_end_us = r.t0_us + r.profile_ms * 1000.0;
    for (int i = r.i_motion; i < n; ++i) {
        const double t = static_cast<double>(t_us[i]);
        const double err = static_cast<double>(rel_deg[i]) - m.dist_deg;
        if (r.i_band < 0 && std::fabs(err) <= kSettleBandDeg) {
            r.i_band = i;
        }
        if (std::fabs(err) > kSettleBandDeg) {
            r.i_settle = -1;
        } else if (r.i_settle < 0) {
            r.i_settle = i;
        }
        if (t <= t_end_us) {
            const double ideal = idealDeg(p, m.dist_deg, (t - r.t0_us) / 1.0e6);
            const double follow = std::fabs(static_cast<double>(rel_deg[i]) - ideal);
            if (follow > r.max_follow_deg) {
                r.max_follow_deg = follow;
                r.i_follow = i;
            }
        }
    }
    return r;
}

}  // namespace ThetaTrace

#endif  // THETA_TRACE_ANALYSIS_H

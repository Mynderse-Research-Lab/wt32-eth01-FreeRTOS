/**
 * @file GantryPathProfile.h
 * @brief Host-testable 2-D X-Z path profile helpers.
 *
 * Console speed / accel / decel are the RESULTANT along the X-Z path.
 * Per-axis Absolute commands are components of that resultant:
 *   v_i = V * |d_i| / L,  a_i = A * |d_i| / L,  with L = hypot(dx, dz).
 *
 * SAFE_Z is an X and theta interlock, not a path via: X Absolute and theta
 * rotation are allowed only while Z is in [z_min, z_min + margin]
 * (retract/traverse band). In that band X and Z may run together. Above the
 * band (toward the belt / Z max) only Z moves.
 * The planner does not insert a stop at the band ceiling; retract from above
 * to an in-band target is one Z Absolute (runtime starts X once Z is in-band).
 * Limit warnings: X A014=min/A015=max; Z A015=min (retract)/A014=max (+Z down).
 * Joint 0 is the sample at min-switch disable (warning deassert), with no
 * extra offset. +Z is toward the belt. Z Absolute joint sense is inverted
 * vs drive so joint − seeks A015.
 */
#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace Gantry {
namespace Path {

inline constexpr float kAxisEpsMm = 0.05f;

/// Theta starts at this fraction of the in-band X+Z traverse segment.
inline constexpr float kThetaStartFrac = 0.25f;
/// Theta is scheduled to finish by this fraction of that same segment.
inline constexpr float kThetaEndFrac   = 0.75f;

struct PathProfile {
    float speed_mm_per_s;
    float accel_mm_per_s2;
    float decel_mm_per_s2;
};

struct AxisComponents {
    float x_speed;
    float x_accel;
    float x_decel;
    float z_speed;
    float z_accel;
    float z_decel;
};

/// Endpoint of one Absolute segment; move_* flags say which axes are armed.
struct PathSegment {
    float x_mm;
    float z_mm;
    bool move_x;
    bool move_z;
};

inline float scaleComponent(float magnitude, float abs_delta, float path_length) {
    if (magnitude == 0.0f) {
        return 0.0f;  // preserve "0 means axis default"
    }
    if (path_length <= kAxisEpsMm) {
        return magnitude;
    }
    return magnitude * (abs_delta / path_length);
}

/// Decompose a path profile into per-axis Absolute components for (dx, dz).
inline AxisComponents decompose(float dx, float dz, const PathProfile& p) {
    const float ax = std::fabs(dx);
    const float az = std::fabs(dz);
    const float L = std::hypot(ax, az);

    AxisComponents c{};
    c.x_speed = scaleComponent(p.speed_mm_per_s, ax, L);
    c.x_accel = scaleComponent(p.accel_mm_per_s2, ax, L);
    c.x_decel = scaleComponent(p.decel_mm_per_s2, ax, L);
    c.z_speed = scaleComponent(p.speed_mm_per_s, az, L);
    c.z_accel = scaleComponent(p.accel_mm_per_s2, az, L);
    c.z_decel = scaleComponent(p.decel_mm_per_s2, az, L);
    return c;
}

/**
 * Plan up to 3 Absolute segments from (x0,z0) to (x1,z1).
 *
 * SAFE_Z (band_ceiling) gates X; it is not a via:
 *  - Pure Z: one Z-alone segment (may cross the ceiling).
 *  - Start above the band, end in-band, need X: one combined segment to
 *    (x1, z1). Runtime defers X until Z is in-band; Z does not stop at ceiling.
 *  - In-band: one X+Z (or X-only / Z-only) segment to (x1, min(z1, ceiling)).
 *  - End above the band: Z-alone after X is at x1.
 *  - Start and end both above, need X: Z-alone into the band, X at ceiling,
 *    Z-alone to z1 (X cannot run until Z is in-band).
 *
 * band_ceiling_z_mm is typically z_min + GANTRY_SAFE_Z_HEIGHT_MM (e.g. 30
 * when z_min=0 and margin=30).
 *
 * @return number of segments written to out[0..2] (0..3).
 */
inline size_t planSegments(float x0, float z0, float x1, float z1,
                           float band_ceiling_z_mm, PathSegment out[3]) {
    size_t n = 0;
    const bool need_x = std::fabs(x1 - x0) > kAxisEpsMm;
    const bool need_z = std::fabs(z1 - z0) > kAxisEpsMm;

    auto push = [&](float x, float z, bool mx, bool mz) {
        if (!mx && !mz) {
            return;
        }
        if (n >= 3) {
            return;
        }
        out[n++] = PathSegment{x, z, mx, mz};
    };

    if (!need_x && !need_z) {
        return 0;
    }

    if (!need_x) {
        push(x0, z1, false, true);
        return n;
    }

    const bool start_above = z0 > band_ceiling_z_mm + kAxisEpsMm;
    const bool end_in_band = z1 <= band_ceiling_z_mm + kAxisEpsMm;

    if (start_above && end_in_band) {
        push(x1, z1, true, need_z);
        return n;
    }

    float cur_x = x0;
    float cur_z = z0;

    if (start_above) {
        push(cur_x, band_ceiling_z_mm, false, true);
        cur_z = band_ceiling_z_mm;
    }

    const float z_coord = end_in_band ? z1 : band_ceiling_z_mm;
    const bool mx = std::fabs(x1 - cur_x) > kAxisEpsMm;
    const bool mz = std::fabs(z_coord - cur_z) > kAxisEpsMm;
    push(x1, z_coord, mx, mz);
    cur_x = x1;
    cur_z = z_coord;

    if (z1 > band_ceiling_z_mm + kAxisEpsMm &&
        std::fabs(z1 - cur_z) > kAxisEpsMm) {
        push(cur_x, z1, false, true);
    }

    return n;
}

/// Max joint-Z for coordinated X+Z (= Z− + margin).
inline float bandCeilingFromZMinus(float z_min_mm, float margin_from_zminus_mm) {
    return z_min_mm + margin_from_zminus_mm;
}

/// True if joint Z is in the coordinated/X-traverse band (at/below ceiling).
inline bool zInTraverseBand(float z_mm, float band_ceiling_mm,
                            float eps_mm = 0.5f) {
    return z_mm <= (band_ceiling_mm + eps_mm);
}

/// Speed for theta to start at kThetaStartFrac and finish by kThetaEndFrac
/// of a constant-speed in-band traverse of length seg_len_mm.
struct ThetaWindow {
    bool  runnable;          // in-band segment exists and |dTheta| > eps
    float start_frac;        // kThetaStartFrac
    float speed_deg_per_s;   // |dTheta| / (0.5 * T_seg), clamped to cap
    bool  speed_clamped;     // true when cap forces finish past 75%
};

inline ThetaWindow planThetaWindow(float seg_len_mm, float path_speed_mm_s,
                                   float delta_theta_deg, float theta_speed_cap) {
    ThetaWindow w{};
    w.start_frac = kThetaStartFrac;
    w.runnable = false;
    w.speed_deg_per_s = 0.0f;
    w.speed_clamped = false;

    const float dth = std::fabs(delta_theta_deg);
    if (dth <= 1.0e-6f) {
        return w;
    }
    if (seg_len_mm <= kAxisEpsMm || path_speed_mm_s <= 0.0f) {
        return w;
    }

    const float T_seg = seg_len_mm / path_speed_mm_s;
    const float window_frac = kThetaEndFrac - kThetaStartFrac;
    if (T_seg <= 0.0f || window_frac <= 0.0f) {
        return w;
    }

    const float required = dth / (window_frac * T_seg);
    if (theta_speed_cap > 0.0f && required > theta_speed_cap) {
        w.speed_deg_per_s = theta_speed_cap;
        w.speed_clamped = true;
    } else {
        w.speed_deg_per_s = required;
    }
    w.runnable = true;
    return w;
}

/// True if axes that this segment armed are within eps of the segment endpoint.
/// Used to refuse advancing a multi-segment path after Absolute timeout/abort
/// (idle busy=0 does not mean the target was reached).
inline bool segmentEndpointsReached(float x_mm, float z_mm,
                                    const PathSegment& seg, float eps_mm) {
    if (seg.move_x && std::fabs(x_mm - seg.x_mm) > eps_mm) {
        return false;
    }
    if (seg.move_z && std::fabs(z_mm - seg.z_mm) > eps_mm) {
        return false;
    }
    return true;
}

}  // namespace Path
}  // namespace Gantry

/**
 * @file GantryPathProfile.h
 * @brief Host-testable 2-D X-Z path profile helpers.
 *
 * Console speed / accel / decel are the RESULTANT along the X-Z path.
 * Per-axis Absolute commands are components of that resultant:
 *   v_i = V * |d_i| / L,  a_i = A * |d_i| / L,  with L = hypot(dx, dz).
 *
 * Simultaneous X+Z is allowed only while Z stays within GANTRY_SAFE_Z_HEIGHT_MM
 * of the Z− endstop / A015 (joint Z <= z_min + margin). Above that band Z
 * moves alone with X held. Limit warnings: X A014=min/A015=max;
 * Z A015=min (−Z)/A014=max (+Z). Z Absolute joint sense is inverted vs drive
 * so joint − seeks A015.
 */
#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace Gantry {
namespace Path {

inline constexpr float kAxisEpsMm = 0.05f;

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
 * Bottom-band coordinated rule (SAFE_Z = margin from Z− / A015):
 *  - Pure Z (no X travel): one Z-alone segment.
 *  - If start is above band_ceiling: Z-alone descend to ceiling first.
 *  - Coordinated X+Z only with both endpoints at Z <= band_ceiling; the
 *    coordinated Z target is min(z1, band_ceiling).
 *  - If final target is above band_ceiling: Z-alone ascend after X arrives.
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

    if (!need_x && !need_z) {
        return 0;
    }

    if (!need_x) {
        out[n++] = PathSegment{x0, z1, /*move_x=*/false, /*move_z=*/true};
        return n;
    }

    float cur_x = x0;
    float cur_z = z0;

    if (cur_z > band_ceiling_z_mm + kAxisEpsMm) {
        out[n++] = PathSegment{cur_x, band_ceiling_z_mm, false, true};
        cur_z = band_ceiling_z_mm;
    }

    const float coord_z = (z1 <= band_ceiling_z_mm) ? z1 : band_ceiling_z_mm;
    const bool coord_move_z = std::fabs(coord_z - cur_z) > kAxisEpsMm;
    out[n++] = PathSegment{x1, coord_z, true, coord_move_z};
    cur_x = x1;
    cur_z = coord_z;

    if (z1 > band_ceiling_z_mm + kAxisEpsMm) {
        out[n++] = PathSegment{cur_x, z1, false, true};
    }

    (void)cur_x;
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

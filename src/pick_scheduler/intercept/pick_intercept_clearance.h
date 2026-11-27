/**
 * @file pick_intercept_clearance.h
 * @brief Clearance and motion time.
 */
#ifndef PICK_INTERCEPT_CLEARANCE_H
#define PICK_INTERCEPT_CLEARANCE_H

#include "pick_intercept_vision.h"

namespace PickIntercept {

// ---------------------------------------------------------------------------
// 4. Clearance. Belt plane, +X across, +Y downstream. half_u runs along deg.
// deg 0 is +X (across belt); positive rotation turns from +X toward +Y (downstream).
// ---------------------------------------------------------------------------

struct Rect {
  float cx = 0.0f;
  float cy = 0.0f;
  float half_u = 0.0f;
  float half_v = 0.0f;
  float deg = 0.0f;
};

inline Rect cellRect(float cx, float cy, float cell_deg, const CellGeom& cell) {
  return Rect{cx, cy, 0.5f * cell.length_mm, 0.5f * beltCrossMm(cell), cell_deg};
}

// Fingers sit either side of the jaw centre along the jaw axis; thickness runs
// along that axis and width across it.
inline void fingerRects(float jx, float jy, float jaw_deg, const FingerGeom& f, Rect out[2]) {
  const float d = 0.5f * (f.open_mm + f.thick_mm);
  const float c = std::cos(jaw_deg * kDegToRad);
  const float s = std::sin(jaw_deg * kDegToRad);
  const float ax = c;
  const float ay = s;
  out[0] = Rect{jx + d * ax, jy + d * ay, 0.5f * f.thick_mm, 0.5f * f.width_mm, jaw_deg};
  out[1] = Rect{jx - d * ax, jy - d * ay, 0.5f * f.thick_mm, 0.5f * f.width_mm, jaw_deg};
}

inline float halfExtentX(const Rect& r) {
  const float c = std::fabs(std::cos(r.deg * kDegToRad));
  const float s = std::fabs(std::sin(r.deg * kDegToRad));
  return r.half_u * c + r.half_v * s;
}

inline float halfExtentY(const Rect& r) {
  const float c = std::fabs(std::cos(r.deg * kDegToRad));
  const float s = std::fabs(std::sin(r.deg * kDegToRad));
  return r.half_u * s + r.half_v * c;
}

inline float projectedRadius(const Rect& r, float nx, float ny) {
  const float c = std::cos(r.deg * kDegToRad);
  const float s = std::sin(r.deg * kDegToRad);
  return r.half_u * std::fabs(c * nx + s * ny) + r.half_v * std::fabs(-s * nx + c * ny);
}

// Separating-axis test for two rotated rectangles.
inline bool rectsOverlap(const Rect& a, const Rect& b) {
  const float dx = b.cx - a.cx;
  const float dy = b.cy - a.cy;
  const float axes[4] = {a.deg, a.deg + 90.0f, b.deg, b.deg + 90.0f};
  for (float deg : axes) {
    const float nx = std::cos(deg * kDegToRad);
    const float ny = std::sin(deg * kDegToRad);
    const float dist = std::fabs(dx * nx + dy * ny);
    if (dist >= projectedRadius(a, nx, ny) + projectedRadius(b, nx, ny)) {
      return false;
    }
  }
  return true;
}

// Across-belt clearance between the cell strip and the nearer finger lane.
// Sweeping the cell along Y hits a finger exactly when their X spans overlap.
inline float xLaneClearanceMm(float cell_deg, float jaw_deg, const CellGeom& cell,
                              const FingerGeom& f) {
  const float hx = halfExtentX(cellRect(0.0f, 0.0f, cell_deg, cell));
  Rect fin[2];
  fingerRects(0.0f, 0.0f, jaw_deg, f, fin);
  float gap = 1.0e9f;
  for (const Rect& r : fin) {
    const float g = std::fabs(r.cx) - halfExtentX(r) - hx;
    if (g < gap) {
      gap = g;
    }
  }
  return gap;
}

// Per-side gap between the cell and an open finger face, along the jaw axis.
inline float sideGapMm(float cell_deg, float jaw_deg, const CellGeom& cell,
                       const FingerGeom& f) {
  const float rel_deg = wrapDeg(cell_deg - jaw_deg);
  const float from_perp = std::fabs(std::fabs(rel_deg) - 90.0f);
  if (from_perp < 0.05f) {
    return jawCloseTravelMm(f.open_mm, beltCrossMm(cell));
  }
  const float rel = rel_deg * kDegToRad;
  const float extent =
      cell.length_mm * std::fabs(std::cos(rel)) + beltCrossMm(cell) * std::fabs(std::sin(rel));
  return 0.5f * (f.open_mm - extent);
}

// Share of belt speed that moves the cell along the jaw axis.
inline float alongFactor(float jaw_deg) {
  return std::fabs(std::sin(jaw_deg * kDegToRad));
}

// ---------------------------------------------------------------------------
// 5. Motion time
// ---------------------------------------------------------------------------

inline float profileSeconds(float distance_abs, float speed, float accel, float decel) {
  if (distance_abs <= 0.0f) {
    return 0.0f;
  }
  if (speed <= 0.0f) {
    return kWaitS;
  }
  if (accel <= 0.0f || decel <= 0.0f) {
    return distance_abs / speed;
  }
  const float ramp_dist = (speed * speed) * (0.5f / accel + 0.5f / decel);
  if (distance_abs >= ramp_dist) {
    return (distance_abs - ramp_dist) / speed + speed / accel + speed / decel;
  }
  const float peak = std::sqrt(2.0f * distance_abs / (1.0f / accel + 1.0f / decel));
  return peak / accel + peak / decel;
}

// Time to cover `distance` along a trapezoid of length `length`.
inline float timeAlongProfile(float distance, float length, float speed, float accel,
                              float decel) {
  if (distance <= 0.0f) {
    return 0.0f;
  }
  if (length <= 0.0f) {
    return 0.0f;
  }
  if (distance >= length) {
    return profileSeconds(length, speed, accel, decel);
  }
  if (speed <= 0.0f) {
    return kWaitS;
  }
  if (accel <= 0.0f || decel <= 0.0f) {
    return distance / speed;
  }
  const float da = (speed * speed) / (2.0f * accel);
  const float dd = (speed * speed) / (2.0f * decel);
  if (da + dd < length) {
    if (distance <= da) {
      return std::sqrt(2.0f * distance / accel);
    }
    const float cruise_end = length - dd;
    if (distance <= cruise_end) {
      return speed / accel + (distance - da) / speed;
    }
    const float into = distance - cruise_end;
    const float disc = speed * speed - 2.0f * decel * into;
    const float t_dec = (disc <= 0.0f) ? (speed / decel)
                                       : (speed - std::sqrt(disc)) / decel;
    return speed / accel + (cruise_end - da) / speed + t_dec;
  }
  const float peak = std::sqrt(2.0f * length / (1.0f / accel + 1.0f / decel));
  const float d_peak = (peak * peak) / (2.0f * accel);
  if (distance <= d_peak) {
    return std::sqrt(2.0f * distance / accel);
  }
  const float remain = length - distance;
  const float t_from_end = std::sqrt(2.0f * remain / decel);
  return peak / accel + peak / decel - t_from_end;
}

// Drop-in Z tail: seconds from release until the fingertips pass the cell top.
inline float zToTopS(float z_hover, float z_top, float z_grip, float speed, float accel,
                     float decel) {
  const float len = z_grip - z_hover;
  const float to_top = z_top - z_hover;
  if (len <= 0.0f || to_top <= 0.0f) {
    return 0.0f;
  }
  return timeAlongProfile(to_top, len, speed, accel, decel);
}


}  // namespace PickIntercept

#endif  // PICK_INTERCEPT_CLEARANCE_H

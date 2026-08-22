/**
 * @file GantryThetaOrigin.h
 * @brief Map HCS01 drive-abs (S-0-0051) onto the firmware joint frame.
 *
 * Joint and drive must share one origin. A firmware-only offset
 * (`zero_puu_ = current S-0-0051`) makes `home t` report joint 0 while the
 * drive still enforces travel on the HIPERFACE absolute value — F2057 when
 * joint +thetalim steps past S-0-0278 / S-0-0049.
 *
 * Aligned: |drive_abs| <= eps after C0300 (S-0-0447) at the mechanical home.
 *   joint = drive, envelope = hard thetalim ∩ drive travel.
 * Unaligned: keep a firmware offset so we do not slew to encoder-native 0
 *   (that would wrap the gripper cable), but shrink thetalim to remaining
 *   drive travel. Full ±180 requires C0300 then `home t` again.
 */
#pragma once

#include <cmath>

namespace Gantry {
namespace ThetaOrigin {

inline constexpr float kAlignEpsDeg = 2.0f;
inline constexpr float kTravelMarginDeg = 0.5f;

struct Plan {
  bool aligned;
  bool ok;
  float zero_deg;  // 0 when aligned; otherwise the captured drive abs
  float joint_min_deg;
  float joint_max_deg;
};

/// drive_min/max are the F2057 window in S-0-0051 degrees (commissioned
/// S-0-0050 / S-0-0049, or S-0-0278 when software limits are 0).
inline Plan plan(float drive_abs_deg, float drive_min_deg, float drive_max_deg,
                 float hard_min_deg, float hard_max_deg,
                 float align_eps_deg = kAlignEpsDeg,
                 float margin_deg = kTravelMarginDeg) {
  Plan p{};
  p.aligned = std::fabs(drive_abs_deg) <= align_eps_deg;
  const float origin = p.aligned ? 0.0f : drive_abs_deg;
  p.zero_deg = origin;

  float jmin = (drive_min_deg + margin_deg) - origin;
  float jmax = (drive_max_deg - margin_deg) - origin;
  if (jmin < hard_min_deg) {
    jmin = hard_min_deg;
  }
  if (jmax > hard_max_deg) {
    jmax = hard_max_deg;
  }
  p.joint_min_deg = jmin;
  p.joint_max_deg = jmax;
  p.ok = (jmax - jmin) > 1.0f;
  return p;
}

}  // namespace ThetaOrigin
}  // namespace Gantry

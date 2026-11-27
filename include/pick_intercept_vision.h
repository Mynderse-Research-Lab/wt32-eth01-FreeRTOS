/**
 * @file pick_intercept_vision.h
 * @brief Frames, body and the vision contract.
 */
#ifndef PICK_INTERCEPT_VISION_H
#define PICK_INTERCEPT_VISION_H

#include "GantryControlPeriod.h"
#include "GantryKinematics.h"
#include "GantryLinearPhases.h"
#include "GantryPathProfile.h"
#include "GantryRotaryPhases.h"
#include "GantryUtils.h"

#include <cmath>
#include <cstdint>

namespace PickIntercept {

// ---------------------------------------------------------------------------
// 1. Frames
// ---------------------------------------------------------------------------

constexpr float kWaitS = 1.0e6f;
// Motion timing and tolerances belong to the Gantry; these only name them.
constexpr float kTickS = Gantry::kLegacyTickS;
constexpr float kPreloadS = Gantry::LinearPhase::kPreloadS;
constexpr float kSettleS = Gantry::LinearPhase::kArrivalSettleS;
constexpr float kThetaStartS = Gantry::RotaryPhase::kStartMaxS;
constexpr float kArriveEpsMm = Gantry::Path::kArriveEpsMm;
// Arrival check tolerance for pre-position / staging. HCS01 S-0-0057 stops at
// 0.10 deg and can coast 0.03-0.06 deg on AF -> AH; 1.0 deg is < 0.3 mm across
// the jaw pads and avoids false aborts at -45.14 deg.
constexpr float kArriveEpsDeg = 1.0f;
constexpr int64_t kConveyorPeriodUs = 250000;
constexpr int64_t kStaleAfterUs = 2 * kConveyorPeriodUs;
constexpr int kFlatFramesForFallback = 3;
constexpr float kGraspHeightMaxMm = 25.4f;
constexpr float kLaneMarginMm = 4.0f;
constexpr float kHoverClearanceMm = 3.0f;
constexpr float kMinThetaSpanDeg = 180.0f;
constexpr float kUpstreamMm = 1500.0f;
constexpr float kDownstreamMm = 25.0f;
constexpr float kDegToRad = 0.0174532925f;
constexpr int kClassCount = 10;
constexpr int kUnusedPackCount = 3;
constexpr int kKnownClassCount = kClassCount + kUnusedPackCount;
constexpr int kTrackerCap = 16;
constexpr int kTakenCap = 8;

// ---------------------------------------------------------------------------
// 2. Body
// ---------------------------------------------------------------------------

struct CellGeom {
  float length_mm = 65.0f;   // In-plane long axis
  float height_mm = 18.3f;   // Vertical size above the belt. Class default is also the diameter.
  float width_mm = 0.0f;     // Jaw cross from vision. 0 uses the class diameter.
  float class_cross_mm = 0.0f;  // Class diameter, kept when vision replaces the vertical size.
};

constexpr float kEndEffectorOpenMm = 80.0f;

// Belt-plane dimension perpendicular to the long axis. Length is never this size.
inline float beltCrossMm(const CellGeom& cell) {
  if (cell.width_mm > 0.0f) {
    return cell.width_mm;
  }
  if (cell.class_cross_mm > 0.0f) {
    return cell.class_cross_mm;
  }
  return cell.height_mm;
}

// Joint Z of the grasp. Measured down from the object top (+Z down): half the
// height, and never more than kGraspHeightMaxMm below that top.
inline float graspJointZMm(float belt_joint_z_mm, float height_from_belt_mm) {
  const float half = 0.5f * height_from_belt_mm;
  const float h_g = half < kGraspHeightMaxMm ? half : kGraspHeightMaxMm;
  const float top = belt_joint_z_mm - height_from_belt_mm;
  return top + h_g;
}

// Jaw travel from the open stop to the object faces. Zero when width equals the opening.
inline float jawCloseTravelMm(float open_mm, float width_mm) {
  return 0.5f * (open_mm - width_mm);
}

// Width is the side the jaws close across, and it is the smaller in-plane size.
// Length is the long axis. A frame that names them the other way around is
// swapped, and the long-axis angle turns 90°, so the pads stay on the length.
struct GripAxes {
  float width_mm = 0.0f;
  float length_mm = 0.0f;
  float theta_deg = 0.0f;
};

inline GripAxes gripAcrossWidth(float width_mm, float length_mm, float theta_deg) {
  GripAxes axes;
  axes.width_mm = width_mm;
  axes.length_mm = length_mm;
  axes.theta_deg = theta_deg;
  if (width_mm > 0.0f && length_mm > 0.0f && width_mm > length_mm) {
    axes.width_mm = length_mm;
    axes.length_mm = width_mm;
    axes.theta_deg = theta_deg + 90.0f;
  }
  return axes;
}

// Every live detection carries all three sizes. The jaws close across the
// smaller in-plane size, which must be in (0, 80]. Length is the long axis.
// A missing or non-finite field is rejected; the class id does not fill it.
inline const char* visionSizeReject(float width_mm, float length_mm, float height_from_belt_mm,
                                   float open_mm) {
  const float limit = open_mm < kEndEffectorOpenMm ? open_mm : kEndEffectorOpenMm;
  const GripAxes axes = gripAcrossWidth(width_mm, length_mm, 0.0f);
  if (!std::isfinite(width_mm) || !std::isfinite(length_mm) || !(axes.width_mm > 0.0f) ||
      axes.width_mm > limit) {
    return "width";
  }
  if (!(axes.length_mm > 0.0f)) {
    return "length";
  }
  if (!std::isfinite(height_from_belt_mm) || !(height_from_belt_mm > 0.0f)) {
    return "height";
  }
  return nullptr;
}

// Cells this gantry picks. Length × jaw width × height above the belt. Grasp Z
// is at belt joint Z 115. Cylinders lie on their side, so width and height are
// the diameter. These are not a firmware fallback: a live frame carries its sizes.
// Ids: 1 AAA, 2 AA, 3 D, 4 C, 5 9V, 6 CR123A, 7 18650, 8 21700, 9 26650, 10 32650.
struct DevProfile {
  const char* name;
  float length_mm;
  float width_mm;
  float height_from_belt_mm;
  float grasp_joint_z_mm;
};

constexpr DevProfile kDevProfiles[kClassCount] = {
    {"AAA", 44.5f, 10.5f, 10.5f, 109.75f},
    {"AA", 50.5f, 14.5f, 14.5f, 107.75f},
    {"D", 61.5f, 34.2f, 34.2f, 97.9f},
    {"C", 50.0f, 26.2f, 26.2f, 101.9f},
    {"9V", 48.5f, 17.5f, 26.5f, 101.75f},
    {"CR123A", 34.5f, 17.0f, 17.0f, 106.5f},
    {"18650", 65.0f, 18.3f, 18.3f, 105.85f},
    {"21700", 70.0f, 21.2f, 21.2f, 104.4f},
    {"26650", 65.0f, 26.5f, 26.5f, 101.75f},
    {"32650", 65.0f, 32.0f, 32.0f, 99.0f},
};

// Packs vision may still report. They occupy the belt and are not pick targets.
// Ids 11 laptop, 12 Li-ion pack, 13 power-tool.
constexpr DevProfile kUnusedPackProfiles[kUnusedPackCount] = {
    {"laptop", 205.0f, 50.0f, 22.0f, 104.0f},
    {"cellpack", 114.0f, 66.0f, 25.0f, 102.5f},
    {"powertool", 120.0f, 75.0f, 80.0f, 60.4f},
};

inline bool isPickClass(uint8_t battery_class) {
  return battery_class >= 1 && battery_class <= kClassCount;
}

inline bool isUnusedPack(uint8_t battery_class) {
  return battery_class > kClassCount && battery_class <= kKnownClassCount;
}

// nullptr when this gantry may pick the class. Packs are tracked and skipped.
// Any other id is not a known body.
inline const char* classPickReject(uint8_t battery_class) {
  if (isPickClass(battery_class)) {
    return nullptr;
  }
  if (isUnusedPack(battery_class)) {
    return "pack not targeted";
  }
  return "class";
}

inline const DevProfile* devProfile(uint8_t battery_class) {
  if (isPickClass(battery_class)) {
    return &kDevProfiles[battery_class - 1];
  }
  if (isUnusedPack(battery_class)) {
    return &kUnusedPackProfiles[battery_class - kClassCount - 1];
  }
  return nullptr;
}

inline CellGeom devCell(uint8_t battery_class) {
  const DevProfile* profile = devProfile(battery_class);
  CellGeom cell;
  if (profile == nullptr) {
    return cell;
  }
  cell.length_mm = profile->length_mm;
  cell.height_mm = profile->height_from_belt_mm;
  cell.width_mm = profile->width_mm;
  return cell;
}

inline CellGeom cellFromVision(float width_mm, float length_mm, float height_from_belt_mm) {
  CellGeom cell;
  cell.width_mm = width_mm;
  cell.length_mm = length_mm;
  cell.height_mm = height_from_belt_mm;
  return cell;
}

// Wire contract for one VISION_DETECT. Capture displacement is required; a
// latency estimate is not a substitute.
struct VisionSample {
  uint32_t item_id = 0;
  uint8_t battery_class = 0;
  bool capture_valid = false;
  float x_across_mm = 0.0f;
  float y_bat_mm = 0.0f;
  float theta_deg = 0.0f;
  float width_mm = 0.0f;
  float length_mm = 0.0f;
  float height_from_belt_mm = 0.0f;
};

inline const char* visionContractReject(const VisionSample& sample, float open_mm) {
  if (sample.item_id == 0) {
    return "item id";
  }
  if (sample.battery_class < 1 || sample.battery_class > kKnownClassCount) {
    return "class";
  }
  if (!sample.capture_valid) {
    return "capture displacement";
  }
  if (!std::isfinite(sample.x_across_mm) || !std::isfinite(sample.y_bat_mm) ||
      !std::isfinite(sample.theta_deg)) {
    return "pose";
  }
  if (sample.theta_deg < -180.0f || sample.theta_deg >= 180.0f) {
    return "theta";
  }
  return visionSizeReject(sample.width_mm, sample.length_mm, sample.height_from_belt_mm, open_mm);
}

inline CellGeom applyVisionSize(CellGeom cell, float width_mm, float length_mm,
                                float height_from_belt_mm) {
  if (cell.class_cross_mm <= 0.0f) {
    cell.class_cross_mm = cell.height_mm;
  }
  if (visionSizeReject(width_mm, length_mm, height_from_belt_mm, kEndEffectorOpenMm) != nullptr) {
    return cell;
  }
  const float limit = kEndEffectorOpenMm;
  if (width_mm > 0.0f && width_mm <= limit) {
    cell.width_mm = width_mm;
  }
  if (length_mm > 0.0f) {
    cell.length_mm = length_mm;
  }
  if (height_from_belt_mm > 0.0f) {
    cell.height_mm = height_from_belt_mm;
  }
  return cell;
}

struct FingerGeom {
  float open_mm = 80.0f;
  float thick_mm = 10.0f;  // Finger width along the close axis.
  float width_mm = 50.0f;  // Finger length along the pads.
  float jaw_mm_s = 210.0f;
};

inline int classIndex(uint8_t battery_class) {
  if (battery_class >= 1 && battery_class <= kClassCount) {
    return static_cast<int>(battery_class) - 1;
  }
  return 0;
}

inline CellGeom defaultCell(uint8_t battery_class) {
  return devCell(battery_class);
}

inline float wrapDeg(float deg) {
  while (deg > 180.0f) {
    deg -= 360.0f;
  }
  while (deg < -180.0f) {
    deg += 360.0f;
  }
  return deg;
}

// Cell long-axis angle folded into [-90, 90).
inline float halfTurnDeg(float deg) {
  float d = std::fmod(deg + 90.0f, 180.0f);
  if (d < 0.0f) {
    d += 180.0f;
  }
  return d - 90.0f;
}

struct JawChoice {
  bool ok = false;
  float joint_deg = 0.0f;  // theta joint command
  float pose_deg = 0.0f;   // pad direction in the pose frame (joint_deg + 90°)
  float travel_deg = 0.0f;  // |joint_deg - current joint|
};

// Side grip: the pads run along the cell's long axis, so the pad direction in the pose
// frame is fold(cell_deg) + jaw_offset_deg (jaw_off_deg, 0° by default). The joint is
// pose - 90° (GantryKinematics): joint 0° holds the pads along +Y. current_deg and the
// limits are joint angles. The two equivalent joints differ by 180° and are ranked by
// linear travel; theta is not modulo.
inline JawChoice chooseSideJaw(float current_deg, float cell_deg, float jaw_offset_deg,
                               float lim_min_deg, float lim_max_deg) {
  const float base = Gantry::thetaJointFromPoseDeg(halfTurnDeg(cell_deg) + jaw_offset_deg);
  JawChoice best;
  for (int k = -3; k <= 3; ++k) {
    const float cand = base + 180.0f * static_cast<float>(k);
    if (cand < lim_min_deg - 1.0e-3f || cand > lim_max_deg + 1.0e-3f) {
      continue;
    }
    const float d = std::fabs(cand - current_deg);
    if (!best.ok || d < best.travel_deg) {
      best.ok = true;
      best.joint_deg = cand;
      best.pose_deg = Gantry::thetaPoseFromJointDeg(cand);
      best.travel_deg = d;
    }
  }
  return best;
}

// Open/close direction for the finger rectangles, in the pose frame. Perpendicular to the
// pads.
inline float closeAxisDeg(float pad_pose_deg) {
  return pad_pose_deg + 90.0f;
}

inline bool jointsArrived(float x, float z, float theta, float x_t, float z_t, float theta_t,
                          float eps_mm = kArriveEpsMm, float eps_deg = kArriveEpsDeg) {
  return std::fabs(x - x_t) <= eps_mm && std::fabs(z - z_t) <= eps_mm &&
         std::fabs(theta - theta_t) <= eps_deg;
}

inline bool inConveyorCollision(float x_mm, float z_mm, float col_x_min_mm, float col_z_min_mm) {
  return x_mm >= col_x_min_mm && z_mm > col_z_min_mm;
}


}  // namespace PickIntercept

#endif  // PICK_INTERCEPT_VISION_H

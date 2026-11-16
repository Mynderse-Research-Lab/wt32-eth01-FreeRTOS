/**
 * @file pick_intercept.h
 * @brief Frames, body, belt, clearance, and motion time for one pick. No FreeRTOS.
 *
 * Grouped as in docs/PICK_SCHEDULER_KINEMATICS.md: frames, body, belt odometer,
 * clearance, motion time, then neighbours and the part tracker. Vision Y is
 * camera-relative. Cell angles are in the vision frame: 0° is the long axis along +Y
 * (downstream), positive toward −X. The jaw offset maps them to the gantry pose frame
 * (0° along New +X). The jaws grip across the cell.
 * Fingers go to grasp height early only when the X-projection misses both
 * finger lanes; otherwise Z waits and must still be at grasp height before contact.
 */

#ifndef PICK_INTERCEPT_H
#define PICK_INTERCEPT_H

#include "GantryControlPeriod.h"
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
constexpr float kArriveEpsDeg = Gantry::Constants::EIP_THETA_SETTLED_TOL_DEG;
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
  float joint_deg = 0.0f;
  float travel_deg = 0.0f;
};

// Returns a gantry pose angle, not a joint angle. cell_deg is the vision angle; the
// jaw offset (cv_jaw_deg, 90° by default) turns it into the pose frame, and
// Kinematics::inverse gives joint = 90° − pose. Joint 0° holds the pads parallel to +Y
// and closes across the belt (side grip). The finger rectangles use the close axis, 90°
// from the pads. The two equivalent poses differ by 180° and are ranked by linear travel;
// theta is not modulo.
inline JawChoice chooseSideJaw(float current_deg, float cell_deg, float jaw_offset_deg,
                               float lim_min_deg, float lim_max_deg) {
  const float base = halfTurnDeg(cell_deg) + jaw_offset_deg;
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
      best.travel_deg = d;
    }
  }
  return best;
}

// Open/close direction for the finger rectangles. Perpendicular to the pads.
inline float closeAxisDeg(float theta_joint_deg) {
  return theta_joint_deg + 90.0f;
}

inline bool jointsArrived(float x, float z, float theta, float x_t, float z_t, float theta_t,
                          float eps_mm = kArriveEpsMm, float eps_deg = kArriveEpsDeg) {
  return std::fabs(x - x_t) <= eps_mm && std::fabs(z - z_t) <= eps_mm &&
         std::fabs(theta - theta_t) <= eps_deg;
}

inline bool inConveyorCollision(float x_mm, float z_mm, float col_x_min_mm, float col_z_min_mm) {
  return x_mm >= col_x_min_mm && z_mm > col_z_min_mm;
}

// ---------------------------------------------------------------------------
// 3. Belt
// ---------------------------------------------------------------------------

struct BeltClock {
  bool valid = false;
  uint32_t epoch = 0;
  int64_t offset_us = 0;
  int64_t conveyor_us = 0;
  int64_t anchor_gantry_us = 0;
  int64_t last_frame_gantry_us = 0;
  float speed_mm_s = 0.0f;
  double displacement_mm = 0.0;
  double raw_disp_mm = 0.0;
  double disp_bias_mm = 0.0;
  double pending_mm = 0.0;
  int flat_frames = 0;
  bool fallback = false;
  uint8_t last_sequence = 0;
  bool have_sequence = false;
  int64_t delay_window_us[8] = {};
  int delay_count = 0;
  int delay_next = 0;
};

struct ConveyorFrame {
  int64_t gantry_us = 0;
  uint32_t timestamp_us = 0;
  float speed_mm_s = 0.0f;
  float displacement_m = 0.0f;
  uint8_t sequence = 0;
};

enum class BeltNote {
  Anchored,
  Restart,
  Ignored,
  Fallback,
};

inline int64_t alignConveyorUs(const BeltClock& belt, uint32_t conveyor_ts_us) {
  const int64_t base = belt.conveyor_us & ~static_cast<int64_t>(0xFFFFFFFFLL);
  int64_t candidate = base + static_cast<int64_t>(conveyor_ts_us);
  constexpr int64_t kHalf = static_cast<int64_t>(0x80000000LL);
  constexpr int64_t kWrap = static_cast<int64_t>(0x100000000LL);
  if (candidate - belt.conveyor_us > kHalf) {
    candidate -= kWrap;
  } else if (belt.conveyor_us - candidate > kHalf) {
    candidate += kWrap;
  }
  return candidate;
}

inline void pushMinDelay(BeltClock& belt, int64_t offset_us) {
  belt.delay_window_us[belt.delay_next] = offset_us;
  belt.delay_next = (belt.delay_next + 1) % 8;
  if (belt.delay_count < 8) {
    ++belt.delay_count;
  }
  int64_t best = belt.delay_window_us[0];
  for (int i = 1; i < belt.delay_count; ++i) {
    if (belt.delay_window_us[i] < best) {
      best = belt.delay_window_us[i];
    }
  }
  belt.offset_us = best;
}

// A flat displacement_m while the belt reports speed is integrated from speed
// after kFlatFramesForFallback frames; the bias keeps position continuous.
inline void anchorFrame(BeltClock& belt, const ConveyorFrame& frame, int64_t conveyor_us) {
  const double raw = static_cast<double>(frame.displacement_m) * 1000.0;
  if (belt.valid) {
    const bool flat = std::fabs(raw - belt.raw_disp_mm) < 1.0e-6 && frame.speed_mm_s > 0.0f &&
                      belt.speed_mm_s > 0.0f;
    if (flat) {
      const double dt_s = static_cast<double>(conveyor_us - belt.conveyor_us) * 1.0e-6;
      const double step = dt_s > 0.0 ? static_cast<double>(belt.speed_mm_s) * dt_s : 0.0;
      ++belt.flat_frames;
      if (belt.fallback) {
        belt.disp_bias_mm += step;
      } else {
        belt.pending_mm += step;
        if (belt.flat_frames >= kFlatFramesForFallback) {
          belt.fallback = true;
          belt.disp_bias_mm += belt.pending_mm;
          belt.pending_mm = 0.0;
        }
      }
    } else {
      belt.flat_frames = 0;
      belt.pending_mm = 0.0;
      belt.fallback = false;
    }
  }
  belt.valid = true;
  belt.conveyor_us = conveyor_us;
  pushMinDelay(belt, frame.gantry_us - conveyor_us);
  belt.anchor_gantry_us = conveyor_us + belt.offset_us;
  belt.last_frame_gantry_us = frame.gantry_us;
  belt.speed_mm_s = frame.speed_mm_s;
  belt.raw_disp_mm = raw;
  belt.displacement_mm = raw + belt.disp_bias_mm;
  belt.last_sequence = frame.sequence;
  belt.have_sequence = true;
}

inline BeltNote noteConveyor(BeltClock& belt, const ConveyorFrame& frame) {
  const double disp_mm = static_cast<double>(frame.displacement_m) * 1000.0;
  if (belt.valid && belt.have_sequence) {
    const uint8_t forward = static_cast<uint8_t>(frame.sequence - belt.last_sequence);
    if (forward == 0) {
      return BeltNote::Ignored;
    }
    const int64_t aligned = alignConveyorUs(belt, frame.timestamp_us);
    const bool seq_reset = forward > 32;
    const bool disp_back = disp_mm + 5.0 < belt.raw_disp_mm;
    const bool time_back = aligned + 500000 < belt.conveyor_us;
    if (seq_reset || disp_back || time_back) {
      const uint32_t epoch = belt.epoch + 1;
      belt = BeltClock{};
      belt.epoch = epoch;
      anchorFrame(belt, frame, static_cast<int64_t>(frame.timestamp_us));
      return BeltNote::Restart;
    }
    const bool was_fallback = belt.fallback;
    anchorFrame(belt, frame, aligned);
    return (!was_fallback && belt.fallback) ? BeltNote::Fallback : BeltNote::Anchored;
  }
  anchorFrame(belt, frame, static_cast<int64_t>(frame.timestamp_us));
  return BeltNote::Anchored;
}

inline bool beltStale(const BeltClock& belt, int64_t gantry_us) {
  if (!belt.valid) {
    return true;
  }
  return gantry_us - belt.last_frame_gantry_us > kStaleAfterUs;
}

inline double beltPositionMm(const BeltClock& belt, int64_t gantry_us) {
  if (!belt.valid) {
    return 0.0;
  }
  const double dt_s = static_cast<double>(gantry_us - belt.anchor_gantry_us) * 1.0e-6;
  return belt.displacement_mm + static_cast<double>(belt.speed_mm_s) * dt_s;
}

inline double capturePositionMm(const BeltClock& belt, int64_t receipt_us, int64_t latency_us,
                                bool capture_disp_valid, float capture_displacement_m) {
  if (capture_disp_valid) {
    return static_cast<double>(capture_displacement_m) * 1000.0 + belt.disp_bias_mm;
  }
  int64_t at = receipt_us - latency_us;
  if (at < 0) {
    at = 0;
  }
  return beltPositionMm(belt, at);
}

// y_bat is the offset from the camera optical centre at capture (+Y
// downstream), so the cell's world Y at capture is cam_y + y_bat.
inline double arriveBeltMm(double s_capture_mm, float y_pick_mm, float cam_y_mm, float y_bat_mm) {
  return s_capture_mm + static_cast<double>(y_pick_mm - (cam_y_mm + y_bat_mm));
}

inline float secondsToPlane(float remaining_mm, float speed_mm_s) {
  if (remaining_mm <= 0.0f) {
    if (speed_mm_s > 0.0f) {
      return remaining_mm / speed_mm_s;
    }
    return 0.0f;
  }
  if (speed_mm_s <= 0.0f) {
    return kWaitS;
  }
  return remaining_mm / speed_mm_s;
}

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

// Filled from the global gantry settings (pickConfig in pick_scheduler.cpp).
// `speed` is the normal approach; `fast_speed` is used only when a later
// tracked part would otherwise be missed. Z does not use either.
struct MotionLimits {
  float speed = 0.0f;
  float fast_speed = 0.0f;
  float accel = 0.0f;
  float decel = 0.0f;
  float theta_cap = 0.0f;
  float theta_accel = 0.0f;
  float theta_decel = 0.0f;
  float z_cap = 0.0f;
  float theta_start_frac = 0.0f;
};

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
  PostBinCycle cycle;
  cycle.grip_dwell_s = grip_dwell_s;
  cycle.retract_s = profileSeconds(std::fabs(pick_z_mm - z_safe_mm), z_cap, accel, decel) +
                    kPreloadS + kSettleS;
  cycle.transfer_s = approachSeconds(bin_x_mm - pick_x_mm, bin_theta_deg - pick_theta_deg, m.speed,
                                     m.theta_cap, accel, decel, m.theta_accel, m.theta_decel);
  cycle.descend_s = profileSeconds(std::fabs(bin_z_mm - z_safe_mm), z_cap, accel, decel) +
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
                                float z_cap) {
  const float dx = seg.x_mm - x0;
  const float dz = seg.z_mm - z0;
  const float ax = std::fabs(dx);
  const float az = std::fabs(dz);
  const float z_speed = z_cap > 1.0f ? z_cap : speed;
  const bool deferred = seg.move_x && !Gantry::Path::zInTraverseBand(z0, ceiling);

  if (!seg.move_x && !seg.move_z) {
    return 0.0f;
  }
  if (deferred && seg.move_z) {
    const float t_z = profileSeconds(az, z_speed, accel, decel) + kPreloadS;
    const float to_band = std::fabs(z0 - ceiling);
    const float t_enter =
        timeAlongProfile(to_band, az, z_speed, accel, decel) + kPreloadS;
    const float t_x = profileSeconds(ax, speed, accel, decel) + kPreloadS;
    const float done = t_z > (t_enter + t_x) ? t_z : (t_enter + t_x);
    return done + kSettleS;
  }
  if (!seg.move_x) {
    return profileSeconds(az, z_speed, accel, decel) + kPreloadS + kSettleS;
  }
  if (!seg.move_z) {
    return profileSeconds(ax, speed, accel, decel) + kPreloadS + kSettleS;
  }
  const float path = std::hypot(ax, az);
  const float v = Gantry::Path::zCappedPathSpeed(speed, dx, dz, z_cap);
  return profileSeconds(path, v, accel, decel) + kPreloadS + kSettleS;
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
  for (size_t i = 0; i < n; ++i) {
    const float dx = segs[i].x_mm - cx;
    const float dz = segs[i].z_mm - cz;
    const float len = std::hypot(dx, dz);
    const bool deferred = segs[i].move_x && !Gantry::Path::zInTraverseBand(cz, z_safe);
    float start = t;
    if (deferred && segs[i].move_z) {
      const float z_speed = m.z_cap > 1.0f ? m.z_cap : m.speed;
      start += timeAlongProfile(std::fabs(cz - z_safe), std::fabs(dz), z_speed, m.accel,
                                m.decel) +
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
    t += segmentWallSeconds(cx, cz, segs[i], z_safe, m.speed, m.accel, m.decel, m.z_cap);
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
      const Gantry::Path::ThetaWindow w =
          Gantry::Path::planThetaWindow(win_len, m.speed, dth, m.theta_cap, m.theta_start_frac);
      if (w.runnable) {
        v_th = w.speed_deg_per_s;
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
    b.hover_s = profileSeconds(z_hover - z_safe, b.descend_speed_mm_s, m.accel, m.decel) +
                kPreloadS + kSettleS;
    b.approach_s += b.hover_s;
    z_from = z_hover;
  }
  b.z_motion_s = profileSeconds(std::fabs(z_pick - z_from), b.descend_speed_mm_s, m.accel,
                                m.decel);
  return b;
}

struct StrikeSchedule {
  bool feasible = false;
  Clearance mode = Clearance::Skip;
  float arrive_s = 0.0f;
  float approach_s = 0.0f;
  float z_motion_s = 0.0f;
  float z_start_s = 0.0f;
  float z_done_s = 0.0f;
  float grip_s = 0.0f;
  float slack_s = 0.0f;
  float window_s = 0.0f;
  float x_clearance_mm = 0.0f;
  float side_gap_mm = 0.0f;
  float contact_mm_s = 0.0f;
  float approach_speed_mm_s = 0.0f;
  float descend_speed_mm_s = 0.0f;
  const char* reason = "ok";
};

// Early Z starts right after the pre-position. Late Z releases from the hover
// so the move finishes one guard before the earliest jaw/object contact, which
// may be earlier than center arrival by beltContactLeadS. Both require z_g by
// then. Provisional: rigid flat rectangles, exact vision pose, 4 mm lanes,
// 3 mm hover, and the configured guard (15 ms until measured).
inline StrikeSchedule planPick(float arrive_s, float belt_mm_s, const MotionBudget& budget,
                               const GripGeometry& grip, const StrikeTiming& timing) {
  StrikeSchedule s;
  s.mode = grip.mode;
  s.arrive_s = arrive_s;
  s.approach_s = budget.approach_s;
  s.z_motion_s = budget.z_motion_s;
  s.x_clearance_mm = grip.x_clearance_mm;
  s.side_gap_mm = grip.side_gap_mm;
  s.approach_speed_mm_s = budget.approach_speed_mm_s;
  s.descend_speed_mm_s = budget.descend_speed_mm_s;
  if (grip.mode == Clearance::Skip) {
    s.reason = grip.reason;
    return s;
  }
  const StrikeLeads leads = strikeLeads(timing, belt_mm_s);
  const float contact_s =
      arrive_s - beltContactLeadS(timing.side_gap_mm, belt_mm_s, timing.along_factor);
  s.window_s = leads.window_s;
  s.contact_mm_s = leads.contact_mm_s;
  s.grip_s = arrive_s - leads.grip_lead_s;
  if (grip.mode == Clearance::EarlyZ) {
    s.z_start_s = budget.approach_s + kPreloadS;
    s.z_done_s = s.z_start_s + budget.z_motion_s;
    s.slack_s = contact_s - timing.guard_s - s.z_done_s;
  } else {
    s.z_start_s = contact_s - leads.release_lead_s;
    s.z_done_s = s.z_start_s + budget.z_motion_s;
    s.slack_s = s.z_start_s - budget.approach_s;
  }
  if (!leads.ok) {
    s.reason = leads.reason;
    return s;
  }
  if (belt_mm_s <= 0.0f || arrive_s >= kWaitS) {
    s.reason = "belt stopped";
    return s;
  }
  if (arrive_s <= 0.0f) {
    s.reason = "missed";
    return s;
  }
  if (s.z_done_s > contact_s - timing.guard_s) {
    s.reason = "late";
    return s;
  }
  if (budget.approach_s >= kWaitS || s.slack_s < 0.0f || s.grip_s < 0.0f) {
    s.reason = "late";
    return s;
  }
  s.feasible = true;
  return s;
}

// Armed pick: new belt sample, frozen jaw and current Z.
struct Retime {
  bool ok = false;
  int64_t arrive_us = 0;
  int64_t grip_us = 0;
  int64_t release_us = 0;
  float contact_mm_s = 0.0f;
  const char* reason = "ok";
};

inline Retime retimePick(const StrikeTiming& t, const BeltClock& belt, int64_t now_us) {
  Retime r;
  if (!belt.valid) {
    r.reason = "no conveyor clock";
    return r;
  }
  if (belt.epoch != t.epoch) {
    r.reason = "conveyor restart";
    return r;
  }
  if (beltStale(belt, now_us)) {
    r.reason = "stale frames";
    return r;
  }
  if (belt.speed_mm_s <= 0.0f) {
    r.reason = "belt stopped";
    return r;
  }
  const StrikeLeads leads = strikeLeads(t, belt.speed_mm_s);
  if (!leads.ok) {
    r.reason = leads.reason;
    return r;
  }
  const double remain = t.s_arrive_mm - beltPositionMm(belt, now_us);
  r.arrive_us = now_us + static_cast<int64_t>(remain / static_cast<double>(belt.speed_mm_s) * 1.0e6);
  r.grip_us = r.arrive_us - static_cast<int64_t>(leads.grip_lead_s * 1.0e6f);
  const float contact_lead = beltContactLeadS(t.side_gap_mm, belt.speed_mm_s, t.along_factor);
  r.release_us =
      r.arrive_us - static_cast<int64_t>((leads.release_lead_s + contact_lead) * 1.0e6f);
  r.contact_mm_s = leads.contact_mm_s;
  r.ok = true;
  return r;
}

// Armed strike whose plunge, close and retract are fixed offsets from arrival
// (strike_plan.h): only the arrival moves with the belt. grip_us and
// release_us are left 0; the caller adds its own offsets.
inline Retime retimeArrival(const StrikeTiming& t, const BeltClock& belt, int64_t now_us) {
    Retime r;
    if (!belt.valid) {
        r.reason = "no conveyor clock";
        return r;
    }
    if (belt.epoch != t.epoch) {
        r.reason = "conveyor restart";
        return r;
    }
    if (beltStale(belt, now_us)) {
        r.reason = "stale frames";
        return r;
    }
    if (belt.speed_mm_s <= 0.0f) {
        r.reason = "belt stopped";
        return r;
    }
    const double remain = t.s_arrive_mm - beltPositionMm(belt, now_us);
    r.arrive_us =
        now_us + static_cast<int64_t>(remain / static_cast<double>(belt.speed_mm_s) * 1.0e6);
    r.ok = true;
    return r;
}

// ---------------------------------------------------------------------------
// Neighbours: other tracked cells against the lowered fingers
// ---------------------------------------------------------------------------

struct Occupant {
  float dx_mm = 0.0f;
  double remain_mm = 0.0;
  float cell_deg = 0.0f;
  CellGeom cell;
};

// Earliest time >= t_from (s from now) at which the fingers can go below the
// cell tops and stay there until t_up without an occupant crossing a finger.
// Bounding boxes keep the check conservative. kWaitS when no such time exists.
inline float fingersFreeFromS(const Occupant* occ, int count, float jaw_deg,
                              const FingerGeom& f, float belt_mm_s, float t_from, float t_up) {
  Rect fin[2];
  fingerRects(0.0f, 0.0f, jaw_deg, f, fin);
  float start = t_from;
  for (int i = 0; i < count; ++i) {
    const Rect cell = cellRect(0.0f, 0.0f, occ[i].cell_deg, occ[i].cell);
    const float hx = halfExtentX(cell);
    const float hy = halfExtentY(cell);
    for (const Rect& r : fin) {
      if (std::fabs(occ[i].dx_mm - r.cx) >= hx + halfExtentX(r)) {
        continue;
      }
      const float reach = hy + halfExtentY(r);
      const float y_now = -static_cast<float>(occ[i].remain_mm);
      if (belt_mm_s <= 0.0f) {
        if (std::fabs(y_now - r.cy) < reach) {
          return kWaitS;
        }
        continue;
      }
      const float t_in = (r.cy - reach - y_now) / belt_mm_s;
      const float t_out = (r.cy + reach - y_now) / belt_mm_s;
      if (t_out <= start || t_in >= t_up) {
        continue;
      }
      if (t_out >= t_up) {
        return kWaitS;
      }
      start = t_out;
    }
  }
  return start;
}

// ---------------------------------------------------------------------------
// Part tracker
// ---------------------------------------------------------------------------

struct TrackedPart {
  uint32_t token = 0;
  uint32_t item_id = 0;
  uint32_t epoch = 0;
  float x_across_mm = 0.0f;
  float y_bat_mm = 0.0f;
  float theta_deg = 0.0f;
  float width_mm = 0.0f;              // 0: class table. Jaws close across this.
  float length_mm = 0.0f;             // 0: class table. In-plane long axis.
  float height_from_belt_mm = 0.0f;   // Required from vision. Belt surface to the top.
  uint8_t battery_class = 1;
  bool approach_fast = false;         // This part's own approach, chosen from the post-bin pose.
  double s_capture_mm = 0.0;
  double s_arrive_mm = 0.0;
};

struct TakenPart {
  uint32_t item_id = 0;
  float x_across_mm = 0.0f;
  double s_arrive_mm = 0.0;
};

enum class TrackAdd {
  Added,
  Duplicate,
  Outside,
  Infeasible,
  Full,
  Epoch,
  Taken,
};

struct PartTracker {
  TrackedPart parts[kTrackerCap] = {};
  int count = 0;
  uint32_t next_token = 1;
  TakenPart taken[kTakenCap] = {};
  int taken_count = 0;
  int taken_next = 0;
};

inline bool sameCell(uint32_t id_a, float x_a, double s_a, uint32_t id_b, float x_b, double s_b,
                     float dedupe_mm) {
  if (id_a != 0 && id_a == id_b) {
    return true;
  }
  return std::fabs(s_a - s_b) < static_cast<double>(dedupe_mm) && std::fabs(x_a - x_b) < dedupe_mm;
}

inline void trackClear(PartTracker& tracker) {
  tracker.count = 0;
  tracker.taken_count = 0;
  tracker.taken_next = 0;
}

inline void trackNoteTaken(PartTracker& tracker, const TrackedPart& part) {
  tracker.taken[tracker.taken_next] = TakenPart{part.item_id, part.x_across_mm, part.s_arrive_mm};
  tracker.taken_next = (tracker.taken_next + 1) % kTakenCap;
  if (tracker.taken_count < kTakenCap) {
    ++tracker.taken_count;
  }
}

// A repeat detection of a tracked cell refreshes its timing and angle.
inline TrackAdd trackAdd(PartTracker& tracker, TrackedPart part, double s_now_mm,
                         float dedupe_mm, bool time_ok, uint32_t belt_epoch) {
  if (part.epoch != belt_epoch) {
    return TrackAdd::Epoch;
  }
  const double remain = part.s_arrive_mm - s_now_mm;
  if (remain > static_cast<double>(kUpstreamMm) || remain < -static_cast<double>(kDownstreamMm)) {
    return TrackAdd::Outside;
  }
  if (!time_ok) {
    return TrackAdd::Infeasible;
  }
  for (int i = 0; i < tracker.taken_count; ++i) {
    const TakenPart& t = tracker.taken[i];
    if (sameCell(part.item_id, part.x_across_mm, part.s_arrive_mm, t.item_id, t.x_across_mm,
                 t.s_arrive_mm, dedupe_mm)) {
      return TrackAdd::Taken;
    }
  }
  for (int i = 0; i < tracker.count; ++i) {
    TrackedPart& have = tracker.parts[i];
    if (sameCell(part.item_id, part.x_across_mm, part.s_arrive_mm, have.item_id,
                 have.x_across_mm, have.s_arrive_mm, dedupe_mm)) {
      have.s_capture_mm = part.s_capture_mm;
      have.s_arrive_mm = part.s_arrive_mm;
      have.x_across_mm = part.x_across_mm;
      have.theta_deg = part.theta_deg;
      have.width_mm = part.width_mm;
      have.length_mm = part.length_mm;
      have.height_from_belt_mm = part.height_from_belt_mm;
      return TrackAdd::Duplicate;
    }
  }
  if (tracker.count >= kTrackerCap) {
    return TrackAdd::Full;
  }
  part.token = tracker.next_token++;
  if (tracker.next_token == 0) {
    tracker.next_token = 1;
  }
  tracker.parts[tracker.count++] = part;
  return TrackAdd::Added;
}

inline void trackDropDownstream(PartTracker& tracker, double s_now_mm) {
  int w = 0;
  for (int i = 0; i < tracker.count; ++i) {
    const double remain = tracker.parts[i].s_arrive_mm - s_now_mm;
    if (remain >= -static_cast<double>(kDownstreamMm)) {
      tracker.parts[w++] = tracker.parts[i];
    }
  }
  tracker.count = w;
}

inline bool trackSetApproachFast(PartTracker& tracker, uint32_t token, bool fast) {
  for (int i = 0; i < tracker.count; ++i) {
    if (tracker.parts[i].token == token) {
      tracker.parts[i].approach_fast = fast;
      return true;
    }
  }
  return false;
}

inline void trackRemove(PartTracker& tracker, uint32_t token) {
  int w = 0;
  for (int i = 0; i < tracker.count; ++i) {
    if (tracker.parts[i].token != token) {
      tracker.parts[w++] = tracker.parts[i];
    }
  }
  tracker.count = w;
}

inline int trackCopy(const PartTracker& tracker, TrackedPart* out, int cap) {
  int n = tracker.count;
  if (n > cap) {
    n = cap;
  }
  for (int i = 0; i < n; ++i) {
    out[i] = tracker.parts[i];
  }
  return n;
}

}  // namespace PickIntercept

#include "coordinated_grasp.h"

#endif  // PICK_INTERCEPT_H

/**
 * @file pick_intercept_belt.h
 * @brief Belt clock.
 */
#ifndef PICK_INTERCEPT_BELT_H
#define PICK_INTERCEPT_BELT_H

#include "pick_intercept_vision.h"

namespace PickIntercept {

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


}  // namespace PickIntercept

#endif  // PICK_INTERCEPT_BELT_H

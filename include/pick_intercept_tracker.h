/**
 * @file pick_intercept_tracker.h
 * @brief Neighbours and the part tracker.
 */
#ifndef PICK_INTERCEPT_TRACKER_H
#define PICK_INTERCEPT_TRACKER_H

#include "pick_intercept_clearance.h"

namespace PickIntercept {

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

#endif  // PICK_INTERCEPT_TRACKER_H

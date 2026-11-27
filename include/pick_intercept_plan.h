/**
 * @file pick_intercept_plan.h
 * @brief Strike schedule and retiming.
 */
#ifndef PICK_INTERCEPT_PLAN_H
#define PICK_INTERCEPT_PLAN_H

#include "pick_intercept_strike.h"
#include "pick_intercept_belt.h"

namespace PickIntercept {

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

// Armed strike event schedule for retimeStrikeEvents().
// Preserves physical invariants once ballistic phases begin:
//   - Before Z fires: Z, grip, and retract track arrive_us together.
//   - After Z fires (before grip): retract never moves before Z finishes its
//     physical plunge (z_fired_us + min_z_to_retract_us), and grip stays at
//     the fixed planned spacing before retract.
//   - After grip fires: both Z and the pneumatic jaws are ballistic, so retract
//     locks to the later of plunge completion and full jaw close spacing.
struct ArmedStrikeTimes {
    bool z_pending = false;
    bool grip_pending = false;
    bool retract_pending = false;
    int64_t z_at_us = 0;
    int64_t grip_at_us = 0;
    int64_t retract_at_us = 0;
    int64_t z_rel_us = 0;
    int64_t close_rel_us = 0;
    int64_t retract_rel_us = 0;
    int64_t min_z_to_retract_us = 0;
    int64_t z_fired_us = 0;
    int64_t grip_fired_us = 0;
};

inline void retimeStrikeEvents(ArmedStrikeTimes& st, int64_t arrive_us) {
    const int64_t close_to_retract_us = st.retract_rel_us - st.close_rel_us;
    if (st.z_pending) {
        st.z_at_us = arrive_us + st.z_rel_us;
        if (st.grip_pending) {
            st.grip_at_us = arrive_us + st.close_rel_us;
        }
        if (st.retract_pending) {
            st.retract_at_us = arrive_us + st.retract_rel_us;
        }
        return;
    }

    const int64_t z_anchor_us = st.z_fired_us > 0 ? st.z_fired_us : st.z_at_us;
    const int64_t z_floor_us = z_anchor_us + st.min_z_to_retract_us;

    if (st.grip_pending) {
        int64_t target_retract_us = arrive_us + st.retract_rel_us;
        if (target_retract_us < z_floor_us) {
            target_retract_us = z_floor_us;
        }
        if (st.retract_pending) {
            st.retract_at_us = target_retract_us;
        }
        st.grip_at_us = target_retract_us - close_to_retract_us;
        return;
    }

    if (st.retract_pending) {
        const int64_t grip_anchor_us = st.grip_fired_us > 0 ? st.grip_fired_us : st.grip_at_us;
        const int64_t grip_floor_us = grip_anchor_us + close_to_retract_us;
        st.retract_at_us = z_floor_us > grip_floor_us ? z_floor_us : grip_floor_us;
    }
}

/// True when the scheduled plunge release has already slipped past the allowed
/// late margin; aborting at hover avoids blind punches and mid-plunge timeouts.
inline bool isPlungeOverdue(int64_t now_us, int64_t z_at_us, int64_t max_late_us) {
    return (now_us - z_at_us) > max_late_us;
}


}  // namespace PickIntercept

#endif  // PICK_INTERCEPT_PLAN_H

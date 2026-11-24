/**
 * @file pick_belt.cpp
 * @brief Pick scheduler: belt clock, part tracking, vision intake and the dev belt clock.
 *
 * One of the pick_*.cpp files that make up the pick scheduler;
 * pick_scheduler_internal.h declares what they share.
 */

#include "pick_scheduler_internal.h"

namespace PickSchedulerImpl {

// vis_batt_x trigger sensor (Sharp GP2Y0A21) along the belt from the camera
// optical centre, measured on the rig 2026-10-30.
constexpr float kTriggerSensorYFromCamMm = -130.0f;

// The 9V centre is still this far upstream when the sensor fires: it passes
// the sensor a median 20 ms after the trigger (Saleae, n = 55, belt ~300 mm/s
// assumed). It depends on the battery's pose on the belt.
constexpr float kTriggerCentreBehindMm = 6.0f;

PickIntercept::BeltClock s_belt;
PickIntercept::PartTracker s_parts;
PickIntercept::TrackedPart s_active;
bool s_active_valid = false;
portMUX_TYPE s_belt_mux = portMUX_INITIALIZER_UNLOCKED;

// Dev belt clock (beltsim): synthetic conveyor frames at a fixed speed and the
// real conveyor cadence. Real frames are ignored while it runs.
esp_timer_handle_t s_belt_sim_timer = nullptr;
std::atomic<bool> s_belt_sim_on{false};
static float s_belt_sim_mm_s = 0.0f;
static double s_belt_sim_disp_m = 0.0;
static int64_t s_belt_sim_last_us = 0;
static uint8_t s_belt_sim_seq = 0;

void noteBelt(const PickIntercept::ConveyorFrame& frame) {
  portENTER_CRITICAL(&s_belt_mux);
  const PickIntercept::BeltNote note = PickIntercept::noteConveyor(s_belt, frame);
  if (note == PickIntercept::BeltNote::Restart) {
    PickIntercept::trackClear(s_parts);
  }
  portEXIT_CRITICAL(&s_belt_mux);
  if (note == PickIntercept::BeltNote::Restart) {
    ESP_LOGW(TAG, "conveyor clock restarted — queued parts dropped");
  } else if (note == PickIntercept::BeltNote::Fallback) {
    ESP_LOGW(TAG, "conveyor displacement_m is flat while moving — integrating speed");
  }
  if (s_strike_armed.load(std::memory_order_acquire)) {
    strikeNotify(kStrikeRetime);
  }
}

void clearParts() {
  portENTER_CRITICAL(&s_belt_mux);
  PickIntercept::trackClear(s_parts);
  s_active_valid = false;
  portEXIT_CRITICAL(&s_belt_mux);
}

// New epoch with no anchor: the next frame (real or synthetic) starts the clock.
static void restartBelt() {
    portENTER_CRITICAL(&s_belt_mux);
    const uint32_t epoch = s_belt.epoch + 1;
    s_belt = PickIntercept::BeltClock{};
    s_belt.epoch = epoch;
    PickIntercept::trackClear(s_parts);
    s_active_valid = false;
    portEXIT_CRITICAL(&s_belt_mux);
}

void beltSimCb(void*) {
    if (!s_belt_sim_on.load(std::memory_order_acquire)) {
        return;
    }
    const int64_t now = esp_timer_get_time();
    s_belt_sim_disp_m += static_cast<double>(s_belt_sim_mm_s) * 1.0e-3 *
                         static_cast<double>(now - s_belt_sim_last_us) * 1.0e-6;
    s_belt_sim_last_us = now;
    PickIntercept::ConveyorFrame frame;
    frame.gantry_us = now;
    frame.timestamp_us = static_cast<uint32_t>(now);
    frame.speed_mm_s = s_belt_sim_mm_s;
    frame.displacement_m = static_cast<float>(s_belt_sim_disp_m);
    frame.sequence = ++s_belt_sim_seq;
    noteBelt(frame);
}

int copyTracked(PickIntercept::TrackedPart* out) {
  portENTER_CRITICAL(&s_belt_mux);
  const int n = PickIntercept::trackCopy(s_parts, out, PickIntercept::kTrackerCap);
  portEXIT_CRITICAL(&s_belt_mux);
  return n;
}

bool copyActive(PickIntercept::TrackedPart* out) {
  portENTER_CRITICAL(&s_belt_mux);
  const bool valid = s_active_valid;
  if (valid) {
    *out = s_active;
  }
  portEXIT_CRITICAL(&s_belt_mux);
  return valid;
}

static float dedupeMm() {
  const float d = Config::AppConfig::instance().data().pick_dedupe_mm;
  return d < 5.0f ? 40.0f : d;
}

// A repeat detection of the cell being picked refines its timing; the jaw is
// frozen once armed, so an angle that no longer fits aborts the strike.
static bool refreshActive(const PickIntercept::TrackedPart& part, const PickConfig& p) {
  PickIntercept::TrackedPart active;
  bool hit = false;
  portENTER_CRITICAL(&s_belt_mux);
  if (s_active_valid &&
      PickIntercept::sameCell(part.item_id, part.x_across_mm, part.s_arrive_mm, s_active.item_id,
                              s_active.x_across_mm, s_active.s_arrive_mm, dedupeMm())) {
    s_active.s_capture_mm = part.s_capture_mm;
    s_active.s_arrive_mm = part.s_arrive_mm;
    s_active.x_across_mm = part.x_across_mm;
    s_active.theta_deg = part.theta_deg;
    s_active.width_mm = part.width_mm;
    s_active.length_mm = part.length_mm;
    s_active.height_from_belt_mm = part.height_from_belt_mm;
    active = s_active;
    hit = true;
  }
  portEXIT_CRITICAL(&s_belt_mux);
  if (!hit) {
    return false;
  }
  if (!s_strike_armed.load(std::memory_order_acquire)) {
    return true;
  }
  // Armed: Z is preloaded to the planned depth and the strike offsets stay
  // fixed. A refinement only moves the arrival, unless it no longer fits.
  const PickIntercept::GripAxes axes =
      PickIntercept::gripAcrossWidth(active.width_mm, active.length_mm, active.theta_deg);
  const PickIntercept::CellGeom cell =
      PickIntercept::cellFromVision(axes.width_mm, axes.length_mm, active.height_from_belt_mm);
  const ZPlan z = zPlan(p, cell.height_mm, cell.height_mm);
  const StrikePlan::Plan sim =
      strikePlan(p, s_gantry->getJointLimits(), PickIntercept::beltCrossMm(cell), cell.height_mm,
                 axes.theta_deg, copyBelt().speed_mm_s, z.grip);
  if (!sim.ok) {
      strikeAbort("refined size");
      return true;
  }
  portENTER_CRITICAL(&s_strike_mux);
  s_ctl.timing.s_arrive_mm = active.s_arrive_mm;
  portEXIT_CRITICAL(&s_strike_mux);
  strikeNotify(kStrikeRetime);
  return true;
}

void classifyFollowers(const PickConfig& p, const Gantry::JointLimits& lim,
                              const PickIntercept::TrackedPart& active, const PickEval& eval,
                              const PickIntercept::BeltClock& belt, int64_t now_us) {
  if (!eval.strike.feasible || belt.speed_mm_s <= 0.0f) {
    return;
  }
  const auto& c = Config::AppConfig::instance().data();
  const float bin_theta = c.pick_drop_any_theta ? eval.pick.theta : c.gantry_cal_theta_park_deg;
  const PickIntercept::PostBinCycle cycle = PickIntercept::postBinCycle(
      eval.pick.z, eval.pick.x, eval.pick.theta, c.gantry_cal_x_park_mm, p.z_safe,
      c.gantry_cal_z_park_mm, bin_theta, p.close_s,
      static_cast<float>(c.gantry_gripper_open_time_ms) * 0.001f, p.limits);
  const double s_now = PickIntercept::beltPositionMm(belt, now_us);
  const float ready_s = eval.strike.arrive_s + cycle.total_s;
  uint32_t drop_tokens[PickIntercept::kTrackerCap];
  int n_drop = 0;
  portENTER_CRITICAL(&s_belt_mux);
  for (int i = 0; i < s_parts.count; ++i) {
    PickIntercept::TrackedPart& other = s_parts.parts[i];
    if (other.token == active.token || other.s_arrive_mm <= active.s_arrive_mm) {
      continue;
    }
    const float other_arrive = PickIntercept::secondsToPlane(
        static_cast<float>(other.s_arrive_mm - s_now), belt.speed_mm_s);
    const PickIntercept::JawChoice jaw = PickIntercept::chooseSideJaw(
        bin_theta, other.theta_deg, p.jaw_offset, lim.theta_min, lim.theta_max);
    const float dth = jaw.ok ? jaw.joint_deg - bin_theta : 180.0f;
    const float dx = (p.x_off + other.x_across_mm) - c.gantry_cal_x_park_mm;
    const float slow_s = PickIntercept::approachSeconds(
        dx, dth, p.limits.speed, p.limits.theta_cap, p.limits.accel,
        p.limits.decel, p.limits.theta_accel, p.limits.theta_decel);
    const float fast_s = PickIntercept::approachSeconds(
        dx, dth, p.limits.fast_speed, p.limits.theta_cap, p.limits.accel,
        p.limits.decel, p.limits.theta_accel, p.limits.theta_decel);
    const PickIntercept::FollowerPace pace =
        PickIntercept::followerPace(ready_s, other_arrive, slow_s, fast_s);
    if (pace == PickIntercept::FollowerPace::Skip) {
      if (n_drop < PickIntercept::kTrackerCap) {
        drop_tokens[n_drop++] = other.token;
      }
    } else {
      other.approach_fast = pace == PickIntercept::FollowerPace::Fast;
    }
  }
  portEXIT_CRITICAL(&s_belt_mux);
  for (int i = 0; i < n_drop; ++i) {
    portENTER_CRITICAL(&s_belt_mux);
    PickIntercept::trackRemove(s_parts, drop_tokens[i]);
    portEXIT_CRITICAL(&s_belt_mux);
    ESP_LOGW(TAG, "follower token=%lu skipped (neither approach pair reaches it)",
             static_cast<unsigned long>(drop_tokens[i]));
  }
}

static void acceptDetect(const VisionItem& item) {
  visMark(0);
  if (s_gantry == nullptr) {
    return;
  }
  const char* source = item.simulated ? "picksim" : "vision";
  L2VisionDetectPayload detect = item.detect;
  const PickIntercept::BeltClock belt = copyBelt();
  if (!belt.valid) {
    return;
  }
  const auto& c = Config::AppConfig::instance().data();
  // vis_batt_x: a trigger sensor sends one fixed 9V frame; only X is not
  // measured, so it comes from batt_x (joint X, at theta 0) instead.
  if (!item.simulated && c.vision_x_from_batt != 0) {
    const float tcp_x =
        s_gantry->forwardKinematics(Gantry::JointConfig(c.test_battery_x_mm, 0.0f, 0.0f)).x;
    detect.x_across_mm = tcp_x - c.conveyor_x_across_to_gantry_x_offset_mm;
    // The frame's own y is a fixed placeholder: the cell centre is just
    // upstream of the trigger sensor when it fires.
    detect.y_bat_mm = kTriggerSensorYFromCamMm - kTriggerCentreBehindMm;
  }
  const Gantry::JointLimits lim = s_gantry->getJointLimits();
  const PickConfig p = pickConfig(lim);
  PickIntercept::VisionSample sample;
  sample.item_id = detect.item_id;
  sample.battery_class = detect.battery_class;
  // Allow trigger sensor (vis_batt_x) and latency fallback when encoder displacement is absent.
  sample.capture_valid = item.capture_valid || (c.vision_x_from_batt != 0) || (c.pick_vision_latency_ms > 0);
  sample.x_across_mm = detect.x_across_mm;
  sample.y_bat_mm = detect.y_bat_mm;
  sample.theta_deg = detect.theta_deg;
  sample.width_mm = detect.width_mm;
  sample.length_mm = detect.length_mm;
  sample.height_from_belt_mm = detect.height_from_belt_mm;
  if (const char* bad = PickIntercept::visionContractReject(sample, p.fingers.open_mm)) {
    ESP_LOGW(TAG, "%s rejected item=%lu (%s)", source, static_cast<unsigned long>(detect.item_id),
             bad);
    return;
  }
  const int64_t latency_us = item.latency_us >= 0
                                 ? item.latency_us
                                 : static_cast<int64_t>(c.pick_vision_latency_ms) * 1000;
  const double s_capture = PickIntercept::capturePositionMm(
      belt, item.receipt_us, latency_us, item.capture_valid, detect.capture_displacement_m);
  PickIntercept::TrackedPart part;
  part.item_id = detect.item_id;
  part.epoch = item.epoch;
  part.x_across_mm = detect.x_across_mm;
  part.y_bat_mm = detect.y_bat_mm;
  const PickIntercept::GripAxes axes =
      PickIntercept::gripAcrossWidth(detect.width_mm, detect.length_mm, detect.theta_deg);
  part.theta_deg = axes.theta_deg;
  part.width_mm = axes.width_mm;
  part.length_mm = axes.length_mm;
  part.height_from_belt_mm = detect.height_from_belt_mm;
  part.battery_class = detect.battery_class;
  part.s_capture_mm = s_capture;
  part.s_arrive_mm = PickIntercept::arriveBeltMm(s_capture, c.conveyor_y_pick_mm,
                                                 c.conveyor_y_cam_mm, detect.y_bat_mm);

  if (part.epoch == belt.epoch && refreshActive(part, p)) {
    return;
  }

  const int64_t now = esp_timer_get_time();
  PickIntercept::TrackedPart others[PickIntercept::kTrackerCap];
  const int n = copyTracked(others);
  const bool busy = s_gantry->isBusy() || s_active_valid;
  const PickEval eval = evaluate(p, s_gantry->getCurrentJointConfig(), lim, part, belt, now,
                                 others, n, nullptr);
  const bool pack = PickIntercept::isUnusedPack(part.battery_class);
  const bool time_ok = pack || (!eval.permanent && eval.strike.arrive_s > 0.0f &&
                                (busy || eval.strike.feasible));
  const double s_now = PickIntercept::beltPositionMm(belt, now);
  portENTER_CRITICAL(&s_belt_mux);
  // Parts already past the gripper free their slot here too, so the tracker
  // drains with the belt even while the cell is stopped (no takeNextPart).
  PickIntercept::trackDropDownstream(s_parts, s_now);
  const PickIntercept::TrackAdd added =
      PickIntercept::trackAdd(s_parts, part, s_now, dedupeMm(), time_ok, belt.epoch);
  portEXIT_CRITICAL(&s_belt_mux);
  if (added == PickIntercept::TrackAdd::Added) {
    ESP_LOGI(TAG, "%s queued item=%lu x=%.1f y=%.1f th=%.1f %s jaw %.1f arrive %.3fs",
             source, static_cast<unsigned long>(detect.item_id),
             static_cast<double>(detect.x_across_mm), static_cast<double>(detect.y_bat_mm),
             static_cast<double>(detect.theta_deg), PickIntercept::clearanceName(eval.strike.mode),
             static_cast<double>(eval.jaw_world_deg), static_cast<double>(eval.strike.arrive_s));
    PickIntercept::TrackedPart active;
    if (copyActive(&active)) {
      const PickEval active_eval =
          evaluate(p, s_gantry->getCurrentJointConfig(), lim, active, belt, now, others, n, nullptr);
      classifyFollowers(p, lim, active, active_eval, belt, now);
    }
    return;
  }
  if (added == PickIntercept::TrackAdd::Duplicate || added == PickIntercept::TrackAdd::Taken) {
    return;
  }
  ESP_LOGW(TAG, "%s rejected item=%lu (%s)", source, static_cast<unsigned long>(detect.item_id),
           added == PickIntercept::TrackAdd::Outside ? "outside window"
           : added == PickIntercept::TrackAdd::Full  ? "tracker full"
           : added == PickIntercept::TrackAdd::Epoch ? "conveyor restart"
                                                     : eval.reason);
}

void processVision() {
  if (s_vision_queue == nullptr) {
    return;
  }
  VisionItem item;
  while (xQueueReceive(s_vision_queue, &item, 0) == pdTRUE) {
    acceptDetect(item);
  }
}

bool queueVision(const VisionItem& item) {
  if (s_vision_queue == nullptr || xQueueSend(s_vision_queue, &item, 0) != pdTRUE) {
    return false;
  }
  return true;
}

uint32_t beltEpoch() {
  portENTER_CRITICAL(&s_belt_mux);
  const uint32_t epoch = s_belt.epoch;
  portEXIT_CRITICAL(&s_belt_mux);
  return epoch;
}

}  // namespace PickSchedulerImpl

using namespace PickSchedulerImpl;

bool pickSchedulerBeltSim(float mm_s, char* out, size_t out_len) {
    if (out == nullptr || out_len == 0) {
        return false;
    }
    if (s_belt_sim_timer == nullptr) {
        snprintf(out, out_len, "belt sim timer unavailable");
        return false;
    }
    if (s_strike_armed.load(std::memory_order_acquire)) {
        snprintf(out, out_len, "strike armed — retry after the pick");
        return false;
    }
    if (!(mm_s >= 0.0f) || mm_s > 1000.0f) {
        snprintf(out, out_len, "speed %.1f mm/s outside 0..1000", static_cast<double>(mm_s));
        return false;
    }
    s_belt_sim_on.store(false, std::memory_order_release);
    esp_timer_stop(s_belt_sim_timer);
    restartBelt();
    if (mm_s == 0.0f) {
        snprintf(out, out_len, "beltsim off — real conveyor frames resume");
        return true;
    }
    s_belt_sim_mm_s = mm_s;
    s_belt_sim_disp_m = 0.0;
    s_belt_sim_last_us = esp_timer_get_time();
    s_belt_sim_seq = 0;
    s_belt_sim_on.store(true, std::memory_order_release);
    beltSimCb(nullptr);
    esp_timer_start_periodic(s_belt_sim_timer,
                             static_cast<uint64_t>(PickIntercept::kConveyorPeriodUs));
    snprintf(out, out_len, "beltsim %.1f mm/s, synthetic frame every %lld ms (real frames ignored)",
             static_cast<double>(mm_s),
             static_cast<long long>(PickIntercept::kConveyorPeriodUs / 1000));
    return true;
}

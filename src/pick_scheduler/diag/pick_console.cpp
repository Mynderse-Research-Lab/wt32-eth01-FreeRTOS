/**
 * @file pick_console.cpp
 * @brief Pick scheduler: picksim, pickplan, pickcover and pickcal console entry points.
 *
 * One of the pick_*.cpp files that make up the pick scheduler;
 * pick_scheduler_internal.h declares what they share.
 */

#include "pick_scheduler_internal.h"

namespace PickSchedulerImpl {

static double s_cam_mark_mm = 0.0;
static bool s_cam_marked = false;

}  // namespace PickSchedulerImpl

using namespace PickSchedulerImpl;

bool pickSchedulerInject(float x_across_mm, float y_bat_mm, float theta_deg, uint8_t battery_class,
                         uint32_t item_id, float width_mm, float length_mm,
                         float height_from_belt_mm) {
  if (!s_cell_run || s_cell_paused) {
    ESP_LOGW(TAG, "picksim ignored: cell is not running");
    return false;
  }
  const PickIntercept::BeltClock belt = copyBelt();
  if (!belt.valid) {
    ESP_LOGW(TAG, "picksim ignored: no conveyor clock yet");
    return false;
  }
  PickIntercept::VisionSample sample;
  sample.item_id = item_id;
  sample.battery_class = battery_class;
  sample.capture_valid = true;
  sample.x_across_mm = x_across_mm;
  sample.y_bat_mm = y_bat_mm;
  sample.theta_deg = theta_deg;
  sample.width_mm = width_mm;
  sample.length_mm = length_mm;
  sample.height_from_belt_mm = height_from_belt_mm;
  const float open_mm = Config::AppConfig::instance().data().pick_finger_open_mm;
  if (const char* bad = PickIntercept::visionContractReject(sample, open_mm)) {
    ESP_LOGW(TAG, "picksim rejected (%s)", bad);
    return false;
  }
  VisionItem item = {};
  item.detect.item_id = item_id;
  item.detect.battery_class = battery_class;
  item.detect.x_across_mm = x_across_mm;
  item.detect.y_bat_mm = y_bat_mm;
  item.detect.theta_deg = theta_deg;
  item.detect.width_mm = width_mm;
  item.detect.length_mm = length_mm;
  item.detect.height_from_belt_mm = height_from_belt_mm;
  item.detect.reserved[0] = kVisionCaptureDispValid;
  item.detect.capture_displacement_m = static_cast<float>(belt.raw_disp_mm / 1000.0);
  item.receipt_us = esp_timer_get_time();
  item.latency_us = 0;
  item.epoch = belt.epoch;
  item.capture_valid = true;
  item.simulated = true;
  if (!queueVision(item)) {
    ESP_LOGW(TAG, "picksim ignored: vision queue full");
    return false;
  }
  return true;
}

bool pickSchedulerPlan(float x_across_mm, float y_bat_mm, float theta_deg, uint8_t battery_class,
                       uint32_t item_id, char* out, size_t out_len, float width_mm, float length_mm,
                       float height_from_belt_mm) {
  if (out == nullptr || out_len == 0) {
    return false;
  }
  out[0] = '\0';
  if (s_gantry == nullptr) {
    std::snprintf(out, out_len, "pick planner not running");
    return false;
  }
  const PickIntercept::BeltClock belt = copyBelt();
  if (!belt.valid) {
    std::snprintf(out, out_len, "no conveyor clock yet");
    return false;
  }
  const auto& c = Config::AppConfig::instance().data();
  PickIntercept::VisionSample sample;
  sample.item_id = item_id;
  sample.battery_class = battery_class;
  sample.capture_valid = true;
  sample.x_across_mm = x_across_mm;
  sample.y_bat_mm = y_bat_mm;
  sample.theta_deg = theta_deg;
  sample.width_mm = width_mm;
  sample.length_mm = length_mm;
  sample.height_from_belt_mm = height_from_belt_mm;
  if (const char* bad = PickIntercept::visionContractReject(sample, c.pick_finger_open_mm)) {
    std::snprintf(out, out_len, "%s", bad);
    return false;
  }
  const int64_t now = esp_timer_get_time();
  PickIntercept::TrackedPart part;
  part.item_id = item_id;
  part.battery_class = battery_class;
  part.x_across_mm = x_across_mm;
  part.y_bat_mm = y_bat_mm;
  part.theta_deg = theta_deg;
  part.width_mm = width_mm;
  part.length_mm = length_mm;
  part.height_from_belt_mm = height_from_belt_mm;
  part.epoch = belt.epoch;
  part.s_capture_mm = PickIntercept::beltPositionMm(belt, now);
  part.s_arrive_mm = PickIntercept::arriveBeltMm(part.s_capture_mm, c.conveyor_y_pick_mm,
                                                 c.conveyor_y_cam_mm, y_bat_mm);
  const Gantry::JointLimits lim = s_gantry->getJointLimits();
  PickIntercept::TrackedPart others[PickIntercept::kTrackerCap];
  const int n = copyTracked(others);
  const PickEval eval = evaluate(pickConfig(lim), s_gantry->getCurrentJointConfig(), lim, part,
                                 belt, now, others, n, nullptr);
  formatEval(eval, out, out_len);
  return true;
}

bool pickSchedulerCover(uint8_t battery_class, char* out, size_t out_len) {
  if (out == nullptr || out_len == 0) {
    return false;
  }
  out[0] = '\0';
  if (s_gantry == nullptr) {
    std::snprintf(out, out_len, "pick planner not running");
    return false;
  }
  const PickIntercept::BeltClock belt = copyBelt();
  const auto& c = Config::AppConfig::instance().data();
  const float v = belt.valid ? belt.speed_mm_s : c.pick_default_belt_speed_mm_s;
  const Gantry::JointLimits lim = s_gantry->getJointLimits();
  const PickConfig p = pickConfig(lim);
  const PickIntercept::DevProfile* profile = PickIntercept::devProfile(battery_class);
  if (PickIntercept::isUnusedPack(battery_class)) {
    std::snprintf(out, out_len, "pack not targeted");
    return false;
  }
  if (profile == nullptr) {
    std::snprintf(out, out_len, "class");
    return false;
  }
  const PickIntercept::CellGeom cell = PickIntercept::devCell(battery_class);
  const ZPlan z = zPlan(p, cell.height_mm, cell.height_mm);
  size_t used = static_cast<size_t>(std::snprintf(
      out, out_len, "v=%.0f %s P=pick S=skip:",
      static_cast<double>(v), profile->name));
  for (int deg = -90; deg < 90 && used + 8 < out_len; deg += 5) {
    const float cell_deg = static_cast<float>(deg);
    const PickIntercept::JawChoice jaw =
        PickIntercept::chooseSideJaw(0.0f, cell_deg, p.jaw_offset, lim.theta_min, lim.theta_max);
    char mode = 'S';
    if (jaw.ok) {
        const StrikePlan::Plan sim = strikePlan(p, lim, PickIntercept::beltCrossMm(cell),
                                                cell.height_mm, cell_deg, v, z.grip);
        mode = sim.ok ? 'P' : 'S';
    }
    used += static_cast<size_t>(std::snprintf(out + used, out_len - used, " %d:%c", deg, mode));
  }
  return true;
}

bool pickSchedulerCalibrate(const char* args, char* out, size_t out_len) {
  if (args == nullptr || out == nullptr || out_len == 0) {
    return false;
  }
  out[0] = '\0';
  auto& app = Config::AppConfig::instance();
  bool reboot = false;
  if (std::strncmp(args, "valve ", 6) == 0) {
    int ms = 0;
    if (std::sscanf(args, "valve %d", &ms) != 1 ||
        !app.setParamInt("grp_valve_ms", ms, reboot)) {
      std::snprintf(out, out_len, "usage: pickcal valve <ms>");
      return false;
    }
    std::snprintf(out, out_len, "valve delay %d ms", ms);
    return true;
  }
  if (std::strncmp(args, "jaw ", 4) == 0) {
    int cls = 0;
    int ms = 0;
    const char* keys[PickIntercept::kClassCount] = {
        "jaw_aaa_ms", "jaw_aa_ms", "jaw_d_ms", "jaw_c_ms", "jaw_9v_ms",
        "jaw_cr123_ms", "jaw_18650_ms", "jaw_21700_ms", "jaw_26650_ms",
        "jaw_32650_ms"};
    if (std::sscanf(args, "jaw %d %d", &cls, &ms) != 2 || cls < 1 ||
        cls > PickIntercept::kClassCount || !app.setParamInt(keys[cls - 1], ms, reboot)) {
      std::snprintf(out, out_len, "usage: pickcal jaw <class 1-10> <ms, 0 = from jaw travel>");
      return false;
    }
    std::snprintf(out, out_len, "class %d jaw contact %d ms%s", cls, ms,
                  ms == 0 ? " (derived from jaw travel)" : "");
    return true;
  }
  if (std::strncmp(args, "jawoff ", 7) == 0) {
    float deg = 0.0f;
    if (std::sscanf(args, "jawoff %f", &deg) != 1 ||
        !app.setParamFloat("jaw_off_deg", deg, reboot)) {
      std::snprintf(out, out_len, "usage: pickcal jawoff <deg>");
      return false;
    }
    std::snprintf(out, out_len, "jaw offset %.1f deg", static_cast<double>(deg));
    return true;
  }
  if (std::strcmp(args, "cam mark") == 0) {
    const PickIntercept::BeltClock belt = copyBelt();
    if (!belt.valid) {
      std::snprintf(out, out_len, "no conveyor clock yet");
      return false;
    }
    s_cam_mark_mm = PickIntercept::beltPositionMm(belt, esp_timer_get_time());
    s_cam_marked = true;
    std::snprintf(out, out_len, "camera mark at belt %.1f mm — advance until the part is under the jaws, then pickcal cam apply",
                  s_cam_mark_mm);
    return true;
  }
  if (std::strcmp(args, "cam apply") == 0) {
    if (!s_cam_marked) {
      std::snprintf(out, out_len, "pickcal cam mark first");
      return false;
    }
    const PickIntercept::BeltClock belt = copyBelt();
    if (!belt.valid) {
      std::snprintf(out, out_len, "no conveyor clock yet");
      return false;
    }
    const double now_mm = PickIntercept::beltPositionMm(belt, esp_timer_get_time());
    const double travel = now_mm - s_cam_mark_mm;
    if (travel < 50.0) {
      std::snprintf(out, out_len, "belt only moved %.1f mm", travel);
      return false;
    }
    const float y_pick = app.data().conveyor_y_pick_mm;
    const float y_cam = y_pick - static_cast<float>(travel);
    if (!app.setParamFloat("vis_cam_y_mm", y_cam, reboot)) {
      std::snprintf(out, out_len, "camera Y %.1f rejected", static_cast<double>(y_cam));
      return false;
    }
    s_cam_marked = false;
    std::snprintf(out, out_len,
                  "camera Y %.1f mm (%0.1f mm of belt). TCP Y offset is %.1f mm; set cv_pick_y_mm if the jaws are not on the gantry plane",
                  static_cast<double>(y_cam), travel,
                  static_cast<double>(app.data().gantry_z_axis_y_offset_mm));
    return true;
  }
  std::snprintf(out, out_len, "usage: pickcal valve|jaw|jawoff|cam mark|cam apply");
  return false;
}

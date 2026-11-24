/**
 * @file pick_motion.cpp
 * @brief Pick scheduler: joint moves, retract, drop and next-part selection.
 *
 * One of the pick_*.cpp files that make up the pick scheduler;
 * pick_scheduler_internal.h declares what they share.
 */

#include "pick_scheduler_internal.h"

namespace PickSchedulerImpl {

// GANTRY_STATUS.active_slot: 0 for ota_0, 1 for ota_1, 0xFF otherwise. Read
// once: the running partition cannot change without a reboot. The CellNet OTA
// sender checks the slot change from it (N-2).
static uint8_t runningOtaSlot() {
  static const uint8_t slot = []() -> uint8_t {
    const esp_partition_t *running = esp_ota_get_running_partition();
    if (running == nullptr) {
      return 0xFF;
    }
    if (running->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_0) {
      return 0;
    }
    if (running->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_1) {
      return 1;
    }
    return 0xFF;
  }();
  return slot;
}

void publishGantryStatus(PickSchedulerTaskConfig *cfg) {
  if (cfg == nullptr || cfg->net_l2 == nullptr || !cfg->net_l2->isReady()) {
    return;
  }
  const int64_t now_us = esp_timer_get_time();
  if (now_us - s_status_us < 500000) {
    return;
  }
  s_status_us = now_us;

  const Gantry::JointConfig joints = cfg->gantry->getCurrentJointConfig();
  uint8_t motion = 0;
  uint16_t fault = 0;
  if (cfg->gantry->isAlarmActive()) {
    motion = 4;
    fault = 1;
  } else if (cfg->gantry->eipBringUpInProgress()) {
    motion = 1;
  } else if (cfg->gantry->isBusy()) {
    motion = cfg->gantry->isGripperActive() ? 3 : 2;
  }
  cfg->net_l2->sendGantryStatus(
      motion, runningOtaSlot(), joints.x, joints.z, joints.theta, s_last_cycle_ms, fault);
}

bool running() {
  return !s_halt_motion.load() && s_cell_run;
}

static void idleSlice(PickSchedulerTaskConfig* cfg, int ms) {
  vTaskDelay(pdMS_TO_TICKS(ms));
  processVision();
  publishGantryStatus(cfg);
}

static bool delayMs(PickSchedulerTaskConfig* cfg, int ms) {
  int left = ms;
  while (left > 0) {
    if (!running()) {
      return false;
    }
    const int slice = left > 20 ? 20 : left;
    idleSlice(cfg, slice);
    left -= slice;
  }
  return running();
}

static bool waitMotion(PickSchedulerTaskConfig *cfg) {
  while (cfg->gantry->isBusy()) {
    if (!running()) {
      disarmStrike();
      cfg->gantry->requestAbort();
      return false;
    }
    idleSlice(cfg, 2);
  }
  return running();
}

bool startJointMove(PickSchedulerTaskConfig* cfg, const Gantry::JointConfig& joint,
                           float speed, float accel, float decel, float theta_speed,
                           float theta_accel, float theta_start_frac, bool theta_full_speed) {
  if (cfg->gantry->isAlarmActive()) {
    ESP_LOGW(TAG, "axis alarm — move skipped");
    return false;
  }
  const auto err = cfg->gantry->moveTo(joint, u32(speed), u32(theta_speed), u32(accel),
                                       u32(decel), u32(theta_accel), u32(theta_accel),
                                       theta_start_frac, theta_full_speed);
  if (err != Gantry::GantryError::OK) {
    ESP_LOGW(TAG, "joint move rejected (%d) x=%.2f z=%.2f th=%.2f", static_cast<int>(err),
             static_cast<double>(joint.x), static_cast<double>(joint.z),
             static_cast<double>(joint.theta));
    return false;
  }
  return true;
}

bool finishJointMove(PickSchedulerTaskConfig* cfg, const Gantry::JointConfig& joint) {
  if (!waitMotion(cfg)) {
    return false;
  }
  const Gantry::JointConfig now = cfg->gantry->getCurrentJointConfig();
  if (!PickIntercept::jointsArrived(now.x, now.z, now.theta, joint.x, joint.z, joint.theta)) {
    ESP_LOGW(TAG, "move ended at x=%.2f z=%.2f th=%.2f, want %.2f %.2f %.2f",
             static_cast<double>(now.x), static_cast<double>(now.z), static_cast<double>(now.theta),
             static_cast<double>(joint.x), static_cast<double>(joint.z),
             static_cast<double>(joint.theta));
    return false;
  }
  return true;
}

bool moveJoint(PickSchedulerTaskConfig* cfg, const Gantry::JointConfig& joint,
                      float speed, float accel, float decel, float theta_speed,
                      float theta_accel, float theta_start_frac,
                      bool theta_full_speed) {
  return startJointMove(cfg, joint, speed, accel, decel, theta_speed, theta_accel,
                        theta_start_frac, theta_full_speed) &&
         finishJointMove(cfg, joint);
}

bool moveApproach(PickSchedulerTaskConfig* cfg, const Gantry::JointConfig& joint,
                         const PickConfig& p, float x_mm_s, float theta_deg_s) {
  return moveJoint(cfg, joint, x_mm_s, p.limits.accel, p.limits.decel, theta_deg_s,
                   p.limits.theta_accel, p.limits.theta_start_frac, p.limits.theta_full_speed);
}

bool moveZ(PickSchedulerTaskConfig* cfg, const Gantry::JointConfig& joint,
                  const PickConfig& p, float z_mm_s) {
  return moveJoint(cfg, joint, z_mm_s, zAccel(p), zDecel(p), p.limits.theta_cap,
                   p.limits.theta_accel, 0.0f, p.limits.theta_full_speed);
}

bool retract(PickSchedulerTaskConfig* cfg, const PickConfig& p) {
  const auto err = cfg->gantry->retractToSafeZ(u32(p.limits.z_cap), u32(zAccel(p)),
                                               u32(zDecel(p)));
  if (err != Gantry::GantryError::OK) {
    ESP_LOGE(TAG, "Z retract refused (%d)", static_cast<int>(err));
    return false;
  }
  while (cfg->gantry->isBusy()) {
    idleSlice(cfg, 10);
  }
  return cfg->gantry->zInTraverseBand();
}

void abortStrike(PickSchedulerTaskConfig* cfg, const PickConfig& p, const char* why) {
  disarmStrike();
  const bool gripped = s_cell_in_jaws.load(std::memory_order_acquire);
  ESP_LOGW(TAG, "strike aborted (%s) — retracting Z%s", why,
           gripped ? " with the jaws closed" : "");
  cfg->gantry->requestAbort();
  if (!retract(cfg, p)) {
    ESP_LOGE(TAG, "Z is not in the SAFE_Z band after abort");
  }
}

const char* strikeAbortReason() {
  return s_strike_abort.load(std::memory_order_acquire);
}

bool dropHeld(PickSchedulerTaskConfig* cfg, const PickConfig& p, float approach_mm_s,
                     float approach_deg_s,
                     const Gantry::JointConfig* already_moving_to) {
  const auto& c = Config::AppConfig::instance().data();
  Gantry::JointConfig drop;
  if (already_moving_to != nullptr) {
    drop = *already_moving_to;
    if (!finishJointMove(cfg, drop)) {
      ESP_LOGW(TAG, "transfer failed; gripper stays closed");
      return false;
    }
  } else {
    const Gantry::JointConfig here = cfg->gantry->getCurrentJointConfig();
    const float theta = c.pick_drop_any_theta ? here.theta : c.gantry_cal_theta_park_deg;
    drop = Gantry::JointConfig(c.gantry_cal_x_park_mm, c.gantry_cal_z_park_mm, theta);
    if (PickIntercept::inConveyorCollision(drop.x, drop.z, c.gantry_conveyor_collision_x_min_mm,
                                           c.gantry_conveyor_collision_z_min_mm)) {
      ESP_LOGW(TAG, "bin drop violates the conveyor envelope; gripper stays closed");
      return false;
    }
    // Single multi-segment move: retracts Z (if below SAFE_Z) with deferred X+theta,
    // traverses to bin X at SAFE_Z, then descends immediately to drop.z without pause.
    if (!moveApproach(cfg, drop, p, approach_mm_s, approach_deg_s)) {
      ESP_LOGW(TAG, "transfer failed; gripper stays closed");
      return false;
    }
  }
  if (!cfg->gantry->grip(false)) {
    ESP_LOGE(TAG, "gripper open write failed at the bin");
    return false;
  }
  s_cell_in_jaws.store(false, std::memory_order_release);
  ESP_LOGI(TAG, "At bin drop X=%.2f mm Z=%.2f mm theta=%.2f deg. Opening gripper.",
           static_cast<double>(drop.x), static_cast<double>(drop.z),
           static_cast<double>(drop.theta));
  return delayMs(cfg, c.gantry_gripper_open_time_ms);
}

bool takeNextPart(PickIntercept::TrackedPart* out) {
  const PickIntercept::BeltClock belt = copyBelt();
  const int64_t now = esp_timer_get_time();
  if (!belt.valid || PickIntercept::beltStale(belt, now)) {
    return false;
  }
  portENTER_CRITICAL(&s_belt_mux);
  const double s_now = PickIntercept::beltPositionMm(s_belt, now);
  PickIntercept::trackDropDownstream(s_parts, s_now);
  PickIntercept::TrackedPart copy[PickIntercept::kTrackerCap];
  const int n = PickIntercept::trackCopy(s_parts, copy, PickIntercept::kTrackerCap);
  portEXIT_CRITICAL(&s_belt_mux);

  const Gantry::JointLimits lim = s_gantry->getJointLimits();
  const PickConfig p = pickConfig(lim);
  const Gantry::JointConfig pose = s_gantry->getCurrentJointConfig();
  const PickIntercept::TrackedPart* best = nullptr;
  for (int i = 0; i < n; ++i) {
    if (copy[i].epoch != belt.epoch) {
      continue;
    }
    const PickEval eval = evaluate(p, pose, lim, copy[i], belt, now, copy, n, nullptr);
    if (!eval.strike.feasible) {
      if (eval.permanent || eval.strike.arrive_s <= 0.0f ||
          std::strcmp(eval.reason, "late") == 0) {
        portENTER_CRITICAL(&s_belt_mux);
        PickIntercept::trackRemove(s_parts, copy[i].token);
        portEXIT_CRITICAL(&s_belt_mux);
        ESP_LOGW(TAG, "pick item=%lu dropped (%s)", static_cast<unsigned long>(copy[i].item_id),
                 eval.reason);
      }
      continue;
    }
    if (best == nullptr || copy[i].s_arrive_mm < best->s_arrive_mm) {
      best = &copy[i];
    }
  }
  if (best == nullptr) {
    return false;
  }
  *out = *best;
  portENTER_CRITICAL(&s_belt_mux);
  PickIntercept::trackRemove(s_parts, best->token);
  s_active = *best;
  s_active_valid = true;
  portEXIT_CRITICAL(&s_belt_mux);
  return true;
}

PickEval evaluateActive(const PickConfig& p, const Gantry::JointLimits& lim,
                               const PickIntercept::TrackedPart& fallback,
                               const float* frozen_joint) {
  PickIntercept::TrackedPart part = fallback;
  (void)copyActive(&part);
  PickIntercept::TrackedPart others[PickIntercept::kTrackerCap];
  const int n = copyTracked(others);
  return evaluate(p, s_gantry->getCurrentJointConfig(), lim, part, copyBelt(),
                  esp_timer_get_time(), others, n, frozen_joint);
}

}  // namespace PickSchedulerImpl

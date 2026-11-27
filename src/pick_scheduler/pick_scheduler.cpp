/**
 * @file pick_scheduler.cpp
 * @brief On-the-fly side-grip pick scheduler. Joint moves only; the strike is
 *        timer-fired and re-timed on every conveyor frame.
 *
 * This file holds the task: cell commands, staging, MCP health and the main
 * loop. It is one of the pick_*.cpp files that make up the pick scheduler;
 * pick_scheduler_internal.h declares what they share.
 */

#include "pick_scheduler_internal.h"

namespace PickSchedulerImpl {

constexpr uint8_t kCmdStart = 1;
constexpr uint8_t kCmdStop = 2;
constexpr uint8_t kCmdPause = 3;
constexpr uint8_t kCmdClearFault = 4;
constexpr uint8_t kCmdPark = 5;
constexpr uint8_t kCmdSetGripper = 6;
constexpr uint8_t kCmdRebootToOta = 7;  // param_u16 = OTA session (N-2)
// A held cell (disabled / uncalibrated) repeats its reason at most this often.
constexpr int64_t kHoldLogEveryUs = 5000000;
// MCP23S17 health: the gripper and field I/O go through it, and a silent chip
// still "accepts" every SPI write (2026-10-20: latched up twice).
constexpr int64_t kMcpCheckEveryUs = 1000000;

struct CellCmd {
  uint8_t command_id;
  uint8_t param_u8;
  uint16_t param_u16;
};

static QueueHandle_t s_cmd_queue = nullptr;
QueueHandle_t s_vision_queue = nullptr;
float s_last_cycle_ms = 0.0f;
int64_t s_status_us = 0;
static int64_t s_hold_log_us = 0;
std::atomic<bool> s_halt_motion{false};
bool s_cell_run = false;
bool s_cell_paused = false;
Gantry::Gantry* s_gantry = nullptr;
CellNet::CellNetL2Node* s_net_l2 = nullptr;  // for netstat
std::atomic<bool> s_cell_in_jaws{false};

static void applyCellCommand(PickSchedulerTaskConfig *cfg, const CellCmd &cmd) {
  switch (cmd.command_id) {
    case kCmdRebootToOta:
      // Refused while busy or on an unconfirmed image; on success it restarts.
      (void)gantryOtaRebootToCellNetMode(cfg->gantry, cmd.param_u16);
      break;
    case kCmdStart: {
      clearParts();
      s_cell_paused = false;
      s_cell_run = true;
      s_halt_motion.store(false);
      cfg->gantry->enable();
      ESP_LOGI(TAG, "CELL_COMMAND START — servos %s",
               cfg->gantry->isEnabled() ? "enabled" : "not enabled");
      const Gantry::JointLimits lim = cfg->gantry->getJointLimits();
      if (lim.theta_max - lim.theta_min < PickIntercept::kMinThetaSpanDeg) {
        ESP_LOGW(TAG, "theta span %.1f deg is under 180 — some cell angles cannot be gripped",
                 static_cast<double>(lim.theta_max - lim.theta_min));
      }
      break;
    }
    case kCmdStop:
      s_cell_run = false;
      s_cell_paused = false;
      s_halt_motion.store(true);
      disarmStrike();
      clearParts();
      cfg->gantry->requestAbort();
      cfg->gantry->disable();
      s_halt_motion.store(false);
      ESP_LOGI(TAG, "CELL_COMMAND STOP — motion stopped, motors disabled");
      break;
    case kCmdPause:
      s_cell_paused = true;
      ESP_LOGI(TAG, "CELL_COMMAND PAUSE — new picks held");
      break;
    case kCmdClearFault:
      s_halt_motion.store(true);
      if (cfg->gantry->clearAlarm()) {
        ESP_LOGI(TAG, "CELL_COMMAND CLEAR_FAULT — alarm reset sent");
      } else {
        ESP_LOGW(TAG, "CELL_COMMAND CLEAR_FAULT — alarm reset failed");
      }
      s_halt_motion.store(false);
      break;
    case kCmdPark: {
      const auto &cdata = Config::AppConfig::instance().data();
      const Gantry::JointConfig park(cdata.gantry_cal_x_park_mm,
                                     cfg->gantry->traverseClearanceZMm(),
                                     cdata.gantry_cal_theta_park_deg);
      const float speed = static_cast<float>(cdata.gantry_default_speed_mm_per_s);
      const float accel = static_cast<float>(cdata.gantry_path_max_accel_mm_per_s2);
      const float decel = static_cast<float>(cdata.gantry_path_max_decel_mm_per_s2);
      if (!moveJoint(cfg, park, speed, accel, decel, cdata.axis_theta_max_speed_deg_per_s,
                     cdata.axis_theta_accel_deg_per_s2, -1.0f)) {
        ESP_LOGW(TAG, "CELL_COMMAND PARK rejected");
        break;
      }
      ESP_LOGI(TAG, "CELL_COMMAND PARK — X=%.1f mm Z=%.1f mm theta=%.1f deg",
               static_cast<double>(park.x), static_cast<double>(park.z),
               static_cast<double>(park.theta));
      break;
    }
    case kCmdSetGripper: {
      const bool close = cmd.param_u8 != 0;
      if (cfg->gantry->grip(close)) {
        s_cell_in_jaws.store(close, std::memory_order_release);
        ESP_LOGI(TAG, "CELL_COMMAND SET_GRIPPER %s", close ? "close" : "open");
      } else {
        ESP_LOGW(TAG, "CELL_COMMAND SET_GRIPPER %s failed", close ? "close" : "open");
      }
      break;
    }
    default:
      ESP_LOGW(TAG, "CELL_COMMAND ignored id=%u", static_cast<unsigned>(cmd.command_id));
      break;
  }
  s_status_us = 0;
  publishGantryStatus(cfg);
}

static void drainCellCommands(PickSchedulerTaskConfig *cfg) {
  if (s_cmd_queue == nullptr) {
    return;
  }
  CellCmd cmd = {};
  while (xQueueReceive(s_cmd_queue, &cmd, 0) == pdTRUE) {
    applyCellCommand(cfg, cmd);
  }
}

// Staging: belt middle at SAFE_Z. Theta 0° holds the pads parallel to +Y and
// the open/close motion along ±X, so a cell along the belt needs no turn.
static void goToStaging(PickSchedulerTaskConfig* cfg) {
  const auto& c = Config::AppConfig::instance().data();
  const Gantry::JointLimits lim = cfg->gantry->getJointLimits();
  float theta = 0.0f;
  if (theta < lim.theta_min) {
    theta = lim.theta_min;
  } else if (theta > lim.theta_max) {
    theta = lim.theta_max;
  }
  const Gantry::JointConfig stage(c.pick_stage_x_mm, cfg->gantry->traverseClearanceZMm(), theta);
  const Gantry::JointConfig here = cfg->gantry->getCurrentJointConfig();
  if (PickIntercept::jointsArrived(here.x, here.z, here.theta, stage.x, stage.z, stage.theta)) {
    return;
  }
  const PickConfig stage_cfg = pickConfig(lim);
  // Single deferred-X move: retracts Z (if below SAFE_Z) and launches X+theta
  // on the exact tick Z enters the SAFE_Z band with no pause between Z and X.
  if (!moveApproach(cfg, stage, stage_cfg, stage_cfg.limits.speed, stage_cfg.limits.theta_cap)) {
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

static bool s_mcp_ok = true;
static int64_t s_mcp_check_us = 0;

// Read back the MCP23S17 configuration once a second. A mismatch (all 0x00
// when the chip has latched up) holds picks; an SPI3 timeout keeps the last
// state. Detection only: nothing here reconfigures the chip.
static void checkMcpHealth() {
    const int64_t now = esp_timer_get_time();
    if (now - s_mcp_check_us < kMcpCheckEveryUs) return;
    s_mcp_check_us = now;
    uint8_t iocon = 0;
    uint8_t iodira = 0;
    uint8_t iodirb = 0;
    const esp_err_t err = gpio_expander_verify(&iocon, &iodira, &iodirb);
    if (err == ESP_ERR_INVALID_RESPONSE) {
        if (s_mcp_ok) {
            ESP_LOGE(TAG,
                     "MCP23S17 not responding (IOCON=0x%02X IODIRA=0x%02X IODIRB=0x%02X) - "
                     "picks held; power-cycle the 3.3 V, then reboot",
                     iocon, iodira, iodirb);
        }
        s_mcp_ok = false;
    } else if (err == ESP_OK && !s_mcp_ok) {
        ESP_LOGW(TAG, "MCP23S17 configuration reads back correctly again");
        s_mcp_ok = true;
    }
}

}  // namespace PickSchedulerImpl

using namespace PickSchedulerImpl;

void pickSchedulerTask(void *param) {
  auto *cfg = static_cast<PickSchedulerTaskConfig *>(param);
  if (cfg == nullptr || cfg->gantry == nullptr) {
    ESP_LOGE(TAG, "Invalid pick scheduler config");
    vTaskDelete(nullptr);
    return;
  }
  s_gantry = cfg->gantry;

  s_cmd_queue = xQueueCreate(8, sizeof(CellCmd));
  s_vision_queue = xQueueCreate(16, sizeof(VisionItem));
  if (s_cmd_queue == nullptr || s_vision_queue == nullptr) {
    ESP_LOGE(TAG, "Failed to create scheduler queues");
    vTaskDelete(nullptr);
    return;
  }

  const BaseType_t strike_ok = xTaskCreatePinnedToCore(
      strikeTask, "PickStrike", PICK_STRIKE_TASK_STACK, nullptr, PICK_STRIKE_TASK_PRIORITY,
      &s_strike_task, PICK_STRIKE_TASK_CORE);
  esp_timer_create_args_t z_args = {};
  z_args.callback = &zTimerCb;
  z_args.name = "pick_z";
  esp_timer_create_args_t g_args = {};
  g_args.callback = &gripTimerCb;
  g_args.name = "pick_grip";
  esp_timer_create_args_t r_args = {};
  r_args.callback = &retractTimerCb;
  r_args.name = "pick_retract";
  const bool timers_ok = strike_ok == pdPASS &&
                         esp_timer_create(&z_args, &s_z_timer) == ESP_OK &&
                         esp_timer_create(&g_args, &s_grip_timer) == ESP_OK &&
                         esp_timer_create(&r_args, &s_retract_timer) == ESP_OK;
  if (!timers_ok) {
    ESP_LOGE(TAG, "strike timer unavailable — picks will not run");
  }
  esp_timer_create_args_t b_args = {};
  b_args.callback = &beltSimCb;
  b_args.name = "belt_sim";
  if (esp_timer_create(&b_args, &s_belt_sim_timer) != ESP_OK) {
    s_belt_sim_timer = nullptr;
  }
  if (xTaskCreatePinnedToCore(traceTask, "PickTrace", PICK_TRACE_TASK_STACK, nullptr,
                              PICK_TRACE_TASK_PRIORITY, &s_trace_task,
                              PICK_TRACE_TASK_CORE) != pdPASS) {
    s_trace_task = nullptr;
    ESP_LOGW(TAG, "strike trace task unavailable — ztrace stays empty");
  }

    s_net_l2 = cfg->net_l2;
    if (cfg->net_l2 != nullptr) {
    cfg->net_l2->onVisionDetect([](const L2VisionDetectPayload &payload, const L2CellHeader &) {
      VisionItem item = {};
      item.detect = payload;
      item.receipt_us = esp_timer_get_time();
      visMark(1);
      item.latency_us = -1;
      item.epoch = beltEpoch();
      item.capture_valid = (payload.reserved[0] & kVisionCaptureDispValid) != 0;
      (void)queueVision(item);
    });
    cfg->net_l2->onCellCommand([](const L2CellCommandPayload &payload, const L2CellHeader &hdr) {
      if (hdr.sender_id != static_cast<uint8_t>(CellNodeId::SUPERVISOR) ||
          s_cmd_queue == nullptr) {
        return;
      }
      if (payload.command_id == kCmdStop || payload.command_id == kCmdClearFault) {
        s_halt_motion.store(true);
        disarmStrike();
      }
      const CellCmd cmd = {payload.command_id, payload.param_u8, payload.param_u16};
      if (xQueueSend(s_cmd_queue, &cmd, 0) != pdTRUE) {
        ESP_LOGW(TAG, "CELL_COMMAND queue full, id=%u",
                 static_cast<unsigned>(payload.command_id));
      }
    });
    cfg->net_l2->onConveyorSpeed([](const L2ConveyorSpeedPayload &payload, const L2CellHeader &hdr) {
      if (s_belt_sim_on.load(std::memory_order_acquire)) {
        return;
      }
      PickIntercept::ConveyorFrame frame;
      frame.gantry_us = esp_timer_get_time();
      frame.timestamp_us = hdr.timestamp_us_low;
      frame.speed_mm_s = payload.speed_mm_s;
      frame.displacement_m = payload.displacement_m;
      frame.sequence = hdr.sequence;
      noteBelt(frame);
    });
  }

  ESP_LOGI(TAG, "Pick scheduler started; picks run after CELL_COMMAND START");

  while (true) {
    drainCellCommands(cfg);
    checkMcpHealth();
    processVision();
    publishGantryStatus(cfg);

    if (!s_cell_run || s_cell_paused || !timers_ok) {
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }
    // A dropped Class 1 link reads every position as 0 (the SAFE_Z band test
    // passes falsely) and refuses every move: plan nothing until it is back.
    // The drives raise their alarm (e.g. E602) only after the reconnect.
    if (!cfg->gantry->drivesOnline()) {
        const int64_t now = esp_timer_get_time();
        if (now - s_hold_log_us > kHoldLogEveryUs) {
            s_hold_log_us = now;
            ESP_LOGW(TAG, "drive link offline (Class 1 reconnecting) — picks held");
        }
        vTaskDelay(pdMS_TO_TICKS(50));
        continue;
    }
    if (cfg->gantry->isAlarmActive()) {
      const int64_t now = esp_timer_get_time();
      if (now - s_hold_log_us > 2000000) {
        s_hold_log_us = now;
        ESP_LOGW(TAG, "axis alarm — picks held");
      }
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }
    // Every move would be refused (MOTOR_NOT_ENABLED / NOT_CALIBRATED): hold
    // with one reason instead of retrying staging and retract each loop.
    const char* hold = !cfg->gantry->isEnabled()             ? "motors disabled (run 'enable')"
                       : !cfg->gantry->isWorkspaceCalibrated() ? "workspace not calibrated (run 'calibrate all')"
                                                               : nullptr;
    if (hold != nullptr) {
        const int64_t now = esp_timer_get_time();
        if (now - s_hold_log_us > kHoldLogEveryUs) {
            s_hold_log_us = now;
            ESP_LOGW(TAG, "%s — picks held", hold);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
        continue;
    }
    // A silent MCP23S17 means the gripper cannot close: hold, do not pick blind.
    // Cleared only when the chip reads back its configuration again (power
    // cycle + ESP reboot re-applies it).
    if (!s_mcp_ok) {
        const int64_t now = esp_timer_get_time();
        if (now - s_hold_log_us > kHoldLogEveryUs) {
            s_hold_log_us = now;
            ESP_LOGW(TAG, "MCP23S17 not responding (gripper / field I/O) — picks held");
        }
        vTaskDelay(pdMS_TO_TICKS(50));
        continue;
    }
    // After a drive trip the servo stays off even once alarmreset clears the
    // alarm; every move would be refused. Hold until the operator re-enables.
    if (const char* axis = cfg->gantry->inactiveServoAxis()) {
        const int64_t now = esp_timer_get_time();
        if (now - s_hold_log_us > kHoldLogEveryUs) {
            s_hold_log_us = now;
            ESP_LOGW(TAG, "%s servo not on (run 'enable') — picks held", axis);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
        continue;
    }
    if (s_cell_in_jaws.load(std::memory_order_acquire) || cfg->gantry->isGripperActive()) {
      const PickConfig p = pickConfig(cfg->gantry->getJointLimits());
      if (!dropHeld(cfg, p, p.limits.speed, p.limits.theta_cap)) {
        vTaskDelay(pdMS_TO_TICKS(1000));
      }
      continue;
    }
    const PickIntercept::BeltClock belt = copyBelt();
    if (PickIntercept::beltStale(belt, esp_timer_get_time())) {
      const int64_t now = esp_timer_get_time();
      if (belt.valid && now - s_hold_log_us > 2000000) {
        s_hold_log_us = now;
        ESP_LOGW(TAG, "conveyor frames stale — picks held");
      }
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    PickIntercept::TrackedPart part;
    if (!takeNextPart(&part)) {
      goToStaging(cfg);
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }
    if (!running() || s_cell_paused) {
      clearParts();
      continue;
    }
    runPick(cfg, part);
    portENTER_CRITICAL(&s_belt_mux);
    s_active_valid = false;
    portEXIT_CRITICAL(&s_belt_mux);
  }
}

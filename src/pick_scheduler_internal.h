/**
 * @file pick_scheduler_internal.h
 * @brief Declarations shared by the pick_*.cpp files that make up the pick
 *        scheduler. Private to src/; include it only from those files.
 */
#pragma once

#include "pick_scheduler.h"

#include "pick_intercept.h"
#include "strike_plan.h"
#include "AppConfig.h"
#include "axis_drivetrain_params.h"
#include "gantry_app_constants.h"
#include "gpio_expander.h"
#include "gantry_ota.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "soc/emac_dma_struct.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include <algorithm>
#include <atomic>
#include <cstdio>
#include <cstring>

namespace PickSchedulerImpl {

[[maybe_unused]] static const char *TAG = "PickScheduler";  // not every file logs

// Shared constants.
constexpr uint8_t kStrikeRetime = 4;
constexpr int64_t kMaxLatePlungeUs = 15000;

// Shared types.
struct VisionItem {
  L2VisionDetectPayload detect;
  int64_t receipt_us;
  int64_t latency_us;  // < 0: vis_lat_ms
  uint32_t epoch;
  bool capture_valid;
  bool simulated;
};

struct PickConfig {
  PickIntercept::FingerGeom fingers;
  float jaw_override_s[PickIntercept::kClassCount] = {};
  PickIntercept::MotionLimits limits;
  float valve_s = 0.0f;
  float guard_s = 0.0f;
  float close_s = 0.0f;
  float valve_on_s = 0.0f;
  float z_plunge_dead_s = 0.0f;
  float z_retract_dead_s = 0.0f;
  int off_plunge_ms = 0;
  int off_close_ms = 0;
  int off_retract_ms = 0;
  bool dropin = false;
  float col_x_min = 0.0f;
  float z_belt = 0.0f;
  float z_safe = 0.0f;
  float stroke = 0.0f;
  float jaw_offset = 0.0f;
  float x_off = 0.0f;
  float cam_y = 0.0f;
  float pick_y = 0.0f;
};

struct ZPlan {
  float grip = 0.0f;
  float hover = 0.0f;
  float top = 0.0f;
  float top_s = 0.0f;
  float tail_s = 0.0f;
};

struct PickEval {
  PickIntercept::StrikeSchedule strike;
  PickIntercept::StrikeTiming timing;
  Gantry::JointConfig approach;
  Gantry::JointConfig hover;
  Gantry::JointConfig pick;
  ZPlan z;
  float jaw_world_deg = 0.0f;
  float approach_deg_s = 0.0f;
  double s_arrive_mm = 0.0;
  bool permanent = false;
  StrikePlan::Plan sim{};
  const char* reason = "ok";
};

// Armed strike. Plunge, close and retract are fixed offsets from arrival;
// a conveyor frame moves arrive_us and all three follow.
struct StrikeCtl {
  PickIntercept::StrikeTiming timing;
  float jaw_world_deg = 0.0f;
  uint8_t battery_class = 1;
  bool z_pending = false;
  bool grip_pending = false;
  bool retract_pending = false;
  int64_t z_at_us = 0;
  int64_t grip_at_us = 0;
  int64_t retract_at_us = 0;
  int64_t arrive_us = 0;
  int64_t z_rel_us = 0;
  int64_t close_rel_us = 0;
  int64_t retract_rel_us = 0;
  int64_t min_z_to_retract_us = 0;
  int64_t z_fired_us = 0;
  int64_t grip_fired_us = 0;
  float retract_mm_s = 0.0f;
  float accel_mm_s2 = 0.0f;
  float decel_mm_s2 = 0.0f;
  float contact_mm_s = 0.0f;
};

// Shared state (each defined once, in the file named).
// pick_scheduler.cpp
extern QueueHandle_t s_vision_queue;
extern float s_last_cycle_ms;
extern int64_t s_status_us;
extern std::atomic<bool> s_halt_motion;
extern bool s_cell_run;
extern bool s_cell_paused;
extern Gantry::Gantry* s_gantry;
extern CellNet::CellNetL2Node* s_net_l2;
extern std::atomic<bool> s_cell_in_jaws;
// pick_strike.cpp
extern TaskHandle_t s_strike_task;
extern esp_timer_handle_t s_z_timer;
extern esp_timer_handle_t s_grip_timer;
extern esp_timer_handle_t s_retract_timer;
extern std::atomic<bool> s_strike_armed;
extern std::atomic<int64_t> s_z_fired_us;
extern std::atomic<int64_t> s_grip_fired_us;
extern std::atomic<int64_t> s_retract_fired_us;
extern std::atomic<float> s_retract_z_mm;
extern std::atomic<float> s_peak_z_mm;
extern std::atomic<const char*> s_strike_abort;
extern StrikeCtl s_ctl;
extern portMUX_TYPE s_strike_mux;
extern std::atomic<int64_t> s_grip_cmd_us;
extern std::atomic<int64_t> s_retract_done_us;
extern std::atomic<bool> s_grip_mark;
extern std::atomic<bool> s_vis_mark;
// pick_belt.cpp
extern PickIntercept::BeltClock s_belt;
extern PickIntercept::PartTracker s_parts;
extern PickIntercept::TrackedPart s_active;
extern bool s_active_valid;
extern portMUX_TYPE s_belt_mux;
extern esp_timer_handle_t s_belt_sim_timer;
extern std::atomic<bool> s_belt_sim_on;
// pick_trace.cpp
extern TaskHandle_t s_trace_task;

// Shared functions (defined in the file named).
// pick_strike.cpp
void visMark(uint32_t level);
PickIntercept::BeltClock copyBelt();
StrikeCtl copyCtl();
void strikeNotify(uint8_t bit);
void zTimerCb(void*);
void gripTimerCb(void*);
void retractTimerCb(void*);
void disarmStrike();
void strikeAbort(const char* why);
int64_t secondsToUs(double s);
void armStrike(const PickIntercept::StrikeTiming& timing, float jaw_world_deg,
                      uint8_t battery_class, const StrikePlan::Plan& sim, int64_t arrive_at,
                      float retract_mm_s, float accel_mm_s2, float decel_mm_s2);
void strikeTask(void*);
// pick_belt.cpp
void noteBelt(const PickIntercept::ConveyorFrame& frame);
void clearParts();
void beltSimCb(void*);
int copyTracked(PickIntercept::TrackedPart* out);
bool copyActive(PickIntercept::TrackedPart* out);
void classifyFollowers(const PickConfig& p, const Gantry::JointLimits& lim,
                              const PickIntercept::TrackedPart& active, const PickEval& eval,
                              const PickIntercept::BeltClock& belt, int64_t now_us);
void processVision();
bool queueVision(const VisionItem& item);
uint32_t beltEpoch();
// pick_trace.cpp
void traceTask(void*);
void traceStart(uint32_t item_id);
void traceFinish(const PickEval& eval, float vz_mm_s, float az_mm_s2, float belt_mm_s);
// pick_eval.cpp
uint32_t u32(float v);
PickConfig pickConfig(const Gantry::JointLimits& lim);
float zAccel(const PickConfig& p);
float zDecel(const PickConfig& p);
ZPlan zPlan(const PickConfig& p, float object_h_mm, float tallest_mm,
                   float hover_at_mm = -1.0f);
StrikePlan::Plan strikePlan(const PickConfig& p, const Gantry::JointLimits& lim,
                                   float width_mm, float height_mm, float theta_deg,
                                   float belt_mm_s, float z_grasp_mm);
PickEval evaluate(const PickConfig& p, const Gantry::JointConfig& pose,
                         const Gantry::JointLimits& lim, const PickIntercept::TrackedPart& part,
                         const PickIntercept::BeltClock& belt, int64_t now_us,
                         const PickIntercept::TrackedPart* others, int n_others,
                         const float* frozen_joint);
void formatEval(const PickEval& eval, char* out, size_t n);
// pick_motion.cpp
void publishGantryStatus(PickSchedulerTaskConfig *cfg);
bool running();
bool startJointMove(PickSchedulerTaskConfig* cfg, const Gantry::JointConfig& joint,
                           float speed, float accel, float decel, float theta_speed,
                           float theta_accel, float theta_start_frac, bool theta_full_speed);
bool finishJointMove(PickSchedulerTaskConfig* cfg, const Gantry::JointConfig& joint);
bool moveJoint(PickSchedulerTaskConfig* cfg, const Gantry::JointConfig& joint,
                      float speed, float accel, float decel, float theta_speed,
                      float theta_accel, float theta_start_frac,
                      bool theta_full_speed = false);
bool moveApproach(PickSchedulerTaskConfig* cfg, const Gantry::JointConfig& joint,
                         const PickConfig& p, float x_mm_s, float theta_deg_s);
bool moveZ(PickSchedulerTaskConfig* cfg, const Gantry::JointConfig& joint,
                  const PickConfig& p, float z_mm_s);
bool retract(PickSchedulerTaskConfig* cfg, const PickConfig& p);
void abortStrike(PickSchedulerTaskConfig* cfg, const PickConfig& p, const char* why);
const char* strikeAbortReason();
bool dropHeld(PickSchedulerTaskConfig* cfg, const PickConfig& p, float approach_mm_s,
                     float approach_deg_s,
                     const Gantry::JointConfig* already_moving_to = nullptr);
bool takeNextPart(PickIntercept::TrackedPart* out);
PickEval evaluateActive(const PickConfig& p, const Gantry::JointLimits& lim,
                               const PickIntercept::TrackedPart& fallback,
                               const float* frozen_joint);
// pick_run.cpp
void runPick(PickSchedulerTaskConfig* cfg, const PickIntercept::TrackedPart& part);

}  // namespace PickSchedulerImpl

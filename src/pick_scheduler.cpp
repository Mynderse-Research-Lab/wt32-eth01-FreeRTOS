/**
 * @file pick_scheduler.cpp
 * @brief On-the-fly side-grip pick scheduler. Joint moves only; the strike is
 *        timer-fired and re-timed on every conveyor frame.
 */

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

static const char *TAG = "PickScheduler";

constexpr uint8_t kCmdStart = 1;
constexpr uint8_t kCmdStop = 2;
constexpr uint8_t kCmdPause = 3;
constexpr uint8_t kCmdClearFault = 4;
constexpr uint8_t kCmdPark = 5;
constexpr uint8_t kCmdSetGripper = 6;
constexpr uint8_t kCmdRebootToOta = 7;  // param_u16 = OTA session (N-2)

constexpr uint8_t kStrikeZ = 1;
constexpr uint8_t kStrikeGrip = 2;
constexpr uint8_t kStrikeRetime = 4;
constexpr uint8_t kStrikeRetract = 8;

// Timer bits earlier than this before their (re-timed) target are stale.
constexpr int64_t kEarlyUs = 1000;
constexpr int64_t kPreloadDeadlineUs = 20000;
constexpr int64_t kGripDeadlineUs = 30000;
constexpr int64_t kRetractDeadlineUs = 30000;
// Retract to the SAFE_Z ceiling must finish within this after it fires.
constexpr int64_t kRetractDoneUs = 2000000;
// Deepest Z reached this far short of the grasp depth is logged.
constexpr float kDepthShortWarnMm = 0.3f;
// Offsets are whole ms, so any bottom dwell they ask for is at least 1 ms.
constexpr double kDwellWarnS = 0.0005;
// vis_batt_x trigger sensor (Sharp GP2Y0A21) along the belt from the camera
// optical centre, measured on the rig 2026-10-30.
constexpr float kTriggerSensorYFromCamMm = -130.0f;
// The 9V centre is still this far upstream when the sensor fires: it passes
// the sensor a median 20 ms after the trigger (Saleae, n = 55, belt ~300 mm/s
// assumed). It depends on the battery's pose on the belt.
constexpr float kTriggerCentreBehindMm = 6.0f;
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
  float retract_mm_s = 0.0f;
  float accel_mm_s2 = 0.0f;
  float decel_mm_s2 = 0.0f;
  float contact_mm_s = 0.0f;
};

static QueueHandle_t s_cmd_queue = nullptr;
static QueueHandle_t s_vision_queue = nullptr;
static PickIntercept::BeltClock s_belt;
static PickIntercept::PartTracker s_parts;
static PickIntercept::TrackedPart s_active;
static bool s_active_valid = false;
static portMUX_TYPE s_belt_mux = portMUX_INITIALIZER_UNLOCKED;
static float s_last_cycle_ms = 0.0f;
static int64_t s_status_us = 0;
static int64_t s_hold_log_us = 0;
static std::atomic<bool> s_halt_motion{false};
static bool s_cell_run = false;
static bool s_cell_paused = false;
static Gantry::Gantry* s_gantry = nullptr;
static CellNet::CellNetL2Node* s_net_l2 = nullptr;  // for netstat
static std::atomic<bool> s_cell_in_jaws{false};

static TaskHandle_t s_strike_task = nullptr;
static esp_timer_handle_t s_z_timer = nullptr;
static esp_timer_handle_t s_grip_timer = nullptr;
static esp_timer_handle_t s_retract_timer = nullptr;
static std::atomic<bool> s_strike_armed{false};
static std::atomic<uint8_t> s_strike_bits{0};
static std::atomic<int64_t> s_z_fired_us{0};
static std::atomic<int64_t> s_grip_fired_us{0};
static std::atomic<int64_t> s_retract_fired_us{0};
static std::atomic<float> s_retract_z_mm{0.0f};
static std::atomic<float> s_peak_z_mm{0.0f};
static std::atomic<const char*> s_strike_abort{nullptr};
static StrikeCtl s_ctl;
static portMUX_TYPE s_strike_mux = portMUX_INITIALIZER_UNLOCKED;

static std::atomic<int64_t> s_grip_cmd_us{0};
static std::atomic<int64_t> s_retract_done_us{0};

// Grip timing marker (gripmark): borrows the TFT backlight pin (GPIO17, no
// free output GPIO on the board). High when the strike task fires the grip,
// low once the MCP write returns. Logic analyser: GPIO17 vs MCP PA0 (DOUT0)
// vs the valve coil gives SPI and relay delay. Off restores the backlight PWM.
static std::atomic<bool> s_grip_mark{false};
// Vision receipt marker (vismark), same pin: high when the CellNetL2 callback
// stamps receipt_us, low when acceptDetect() takes the item. A Saleae on the
// trigger sensor and GPIO17 gives sensor-to-receipt latency (vis_lat_ms).
static std::atomic<bool> s_vis_mark{false};

static void gripMark(uint32_t level) {
    if (s_grip_mark.load(std::memory_order_relaxed)) {
        gpio_set_level(static_cast<gpio_num_t>(TFT_BLK_GPIO), level);
    }
}

static void visMark(uint32_t level) {
    if (s_vis_mark.load(std::memory_order_relaxed)) {
        gpio_set_level(static_cast<gpio_num_t>(TFT_BLK_GPIO), level);
    }
}

static double s_cam_mark_mm = 0.0;
static bool s_cam_marked = false;

// Dev belt clock (beltsim): synthetic conveyor frames at a fixed speed and the
// real conveyor cadence. Real frames are ignored while it runs.
static esp_timer_handle_t s_belt_sim_timer = nullptr;
static std::atomic<bool> s_belt_sim_on{false};
static float s_belt_sim_mm_s = 0.0f;
static double s_belt_sim_disp_m = 0.0;
static int64_t s_belt_sim_last_us = 0;
static uint8_t s_belt_sim_seq = 0;

// Strike trace: Z feedback sampled at 1 kHz into a ring from the held preload
// until kTraceTailUs after the retract command, so it keeps the last
// kTraceCap ms (the hover wait can be many seconds). Read with `ztrace`.
constexpr int kTraceCap = 1500;
constexpr int64_t kTraceTailUs = 400000;
constexpr int64_t kTraceMaxUs = 120000000;
// Sampling starts this long before the plunge release. A 1 kHz sampler on
// core 0 through a seconds-long hover wait stalled the console and LAN log
// for its whole run (2026-10-25; cause not yet found), so it covers the strike only.
constexpr int64_t kTraceLeadUs = 300000;
// Z moved when it leaves the reference by more than this.
constexpr float kTraceMoveMm = 0.05f;
// Z reached grasp depth within this.
constexpr float kTraceReachMm = 0.10f;
constexpr int kTraceRawPerLine = 20;

struct ZTrace {
    uint32_t item_id = 0;
    int64_t t0_us = 0;
    int n = 0;      // valid samples, oldest first once the trace stops
    int total = 0;  // samples written into the ring
    bool complete = false;
    StrikeCtl ctl;
    int64_t z_fired_us = 0;
    int64_t grip_cmd_us = 0;
    int64_t grip_fired_us = 0;
    int64_t retract_cmd_us = 0;
    int64_t retract_done_us = 0;
    float z_hover_mm = 0.0f;
    float z_grasp_mm = 0.0f;
    float vz_mm_s = 0.0f;
    float az_mm_s2 = 0.0f;
    float belt_mm_s = 0.0f;
    float tau_plunge_s = 0.0f;
    int32_t t_us[kTraceCap];
    float z_mm[kTraceCap];
};
static ZTrace s_trace;
static std::atomic<bool> s_trace_run{false};
static std::atomic<int64_t> s_trace_stop_at_us{0};
static TaskHandle_t s_trace_task = nullptr;

static PickIntercept::BeltClock copyBelt() {
  portENTER_CRITICAL(&s_belt_mux);
  const PickIntercept::BeltClock belt = s_belt;
  portEXIT_CRITICAL(&s_belt_mux);
  return belt;
}

static StrikeCtl copyCtl() {
  portENTER_CRITICAL(&s_strike_mux);
  const StrikeCtl ctl = s_ctl;
  portEXIT_CRITICAL(&s_strike_mux);
  return ctl;
}

static void strikeNotify(uint8_t bit) {
  s_strike_bits.fetch_or(bit, std::memory_order_acq_rel);
  if (s_strike_task != nullptr) {
    xTaskNotifyGive(s_strike_task);
  }
}

static void zTimerCb(void*) { strikeNotify(kStrikeZ); }
static void gripTimerCb(void*) { strikeNotify(kStrikeGrip); }
static void retractTimerCb(void*) { strikeNotify(kStrikeRetract); }

static void stopTimers() {
    if (s_z_timer != nullptr) {
        esp_timer_stop(s_z_timer);
    }
    if (s_grip_timer != nullptr) {
        esp_timer_stop(s_grip_timer);
    }
    if (s_retract_timer != nullptr) {
        esp_timer_stop(s_retract_timer);
    }
}

static void disarmStrike() {
    s_strike_armed.store(false, std::memory_order_release);
    stopTimers();
    portENTER_CRITICAL(&s_strike_mux);
    s_ctl.z_pending = false;
    s_ctl.grip_pending = false;
    s_ctl.retract_pending = false;
    portEXIT_CRITICAL(&s_strike_mux);
    s_strike_bits.store(0, std::memory_order_release);
}

static void strikeAbort(const char* why) {
    const char* none = nullptr;
    s_strike_abort.compare_exchange_strong(none, why, std::memory_order_acq_rel);
    disarmStrike();
}

static void armOne(esp_timer_handle_t timer, int64_t fire_us, uint8_t bit) {
    const int64_t delay = fire_us - esp_timer_get_time();
    if (timer == nullptr || delay <= 1000) {
        strikeNotify(bit);
        return;
    }
    esp_timer_stop(timer);
    esp_timer_start_once(timer, static_cast<uint64_t>(delay));
}

static int64_t secondsToUs(double s) {
    return static_cast<int64_t>(std::llround(s * 1.0e6));
}

static void armPending(const StrikeCtl& ctl) {
    if (ctl.z_pending) {
        armOne(s_z_timer, ctl.z_at_us, kStrikeZ);
    }
    if (ctl.grip_pending) {
        armOne(s_grip_timer, ctl.grip_at_us, kStrikeGrip);
    }
    if (ctl.retract_pending) {
        armOne(s_retract_timer, ctl.retract_at_us, kStrikeRetract);
    }
}

// Z is preloaded and held; the three events fire at fixed offsets from
// arrive_at (strike_plan.h).
static void armStrike(const PickIntercept::StrikeTiming& timing, float jaw_world_deg,
                      uint8_t battery_class, const StrikePlan::Plan& sim, int64_t arrive_at,
                      float retract_mm_s, float accel_mm_s2, float decel_mm_s2) {
    disarmStrike();
    s_strike_abort.store(nullptr, std::memory_order_release);
    s_z_fired_us.store(0, std::memory_order_release);
    s_grip_fired_us.store(0, std::memory_order_release);
    s_retract_fired_us.store(0, std::memory_order_release);
    s_grip_cmd_us.store(0, std::memory_order_release);
    s_retract_done_us.store(0, std::memory_order_release);
    s_peak_z_mm.store(0.0f, std::memory_order_release);
    portENTER_CRITICAL(&s_strike_mux);
    s_ctl = StrikeCtl{};
    s_ctl.timing = timing;
    s_ctl.jaw_world_deg = jaw_world_deg;
    s_ctl.battery_class = battery_class;
    s_ctl.z_rel_us = secondsToUs(sim.t_z_s);
    s_ctl.close_rel_us = secondsToUs(sim.t_close_s);
    s_ctl.retract_rel_us = secondsToUs(sim.t_retract_s);
    s_ctl.arrive_us = arrive_at;
    s_ctl.z_at_us = arrive_at + s_ctl.z_rel_us;
    s_ctl.grip_at_us = arrive_at + s_ctl.close_rel_us;
    s_ctl.retract_at_us = arrive_at + s_ctl.retract_rel_us;
    s_ctl.z_pending = true;
    s_ctl.grip_pending = true;
    s_ctl.retract_pending = true;
    s_ctl.retract_mm_s = retract_mm_s;
    s_ctl.accel_mm_s2 = accel_mm_s2;
    s_ctl.decel_mm_s2 = decel_mm_s2;
    const StrikeCtl ctl = s_ctl;
    portEXIT_CRITICAL(&s_strike_mux);
    s_strike_armed.store(true, std::memory_order_release);
    armPending(ctl);
}

// Only the arrival moves with the belt; pending events keep their offsets.
static void retimeArmed(int64_t now) {
    StrikeCtl ctl = copyCtl();
    if (!ctl.z_pending && !ctl.grip_pending && !ctl.retract_pending) {
        return;
    }
    const PickIntercept::Retime r = PickIntercept::retimeArrival(ctl.timing, copyBelt(), now);
    if (!r.ok) {
        strikeAbort(r.reason);
        return;
    }
    portENTER_CRITICAL(&s_strike_mux);
    s_ctl.arrive_us = r.arrive_us;
    if (s_ctl.z_pending) {
        s_ctl.z_at_us = r.arrive_us + s_ctl.z_rel_us;
    }
    if (s_ctl.grip_pending) {
        s_ctl.grip_at_us = r.arrive_us + s_ctl.close_rel_us;
    }
    if (s_ctl.retract_pending) {
        s_ctl.retract_at_us = r.arrive_us + s_ctl.retract_rel_us;
    }
    ctl = s_ctl;
    portEXIT_CRITICAL(&s_strike_mux);
    armPending(ctl);
}

enum class StrikeEvent : uint8_t { Z, Grip, Retract };

static bool takeDue(StrikeEvent ev, int64_t now) {
    bool due = false;
    portENTER_CRITICAL(&s_strike_mux);
    switch (ev) {
        case StrikeEvent::Z:
            due = s_ctl.z_pending && now + kEarlyUs >= s_ctl.z_at_us;
            if (due) {
                s_ctl.z_pending = false;
            }
            break;
        case StrikeEvent::Grip:
            due = s_ctl.grip_pending && now + kEarlyUs >= s_ctl.grip_at_us;
            if (due) {
                s_ctl.grip_pending = false;
            }
            break;
        case StrikeEvent::Retract:
            due = s_ctl.retract_pending && now + kEarlyUs >= s_ctl.retract_at_us;
            if (due) {
                s_ctl.retract_pending = false;
            }
            break;
        default:
            break;
    }
    portEXIT_CRITICAL(&s_strike_mux);
    return due;
}

static void strikeTask(void*) {
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        for (;;) {
            const uint8_t bits = s_strike_bits.exchange(0, std::memory_order_acq_rel);
            if (bits == 0) {
                break;
            }
            if (!s_strike_armed.load(std::memory_order_acquire) || s_gantry == nullptr) {
                continue;
            }
            const int64_t now = esp_timer_get_time();
            if ((bits & kStrikeRetime) != 0) {
                retimeArmed(now);
            } else if ((bits & (kStrikeZ | kStrikeGrip | kStrikeRetract)) != 0) {
                // No frame since the last retime: the belt may have gone stale or stopped.
                const PickIntercept::Retime r =
                    PickIntercept::retimeArrival(copyCtl().timing, copyBelt(), now);
                if (!r.ok) {
                    strikeAbort(r.reason);
                    continue;
                }
            }
            if ((bits & kStrikeZ) != 0 && takeDue(StrikeEvent::Z, now)) {
                if (s_gantry->releasePreloadedMotion()) {
                    s_z_fired_us.store(now, std::memory_order_release);
                    // Queue the retract behind the plunge now; its edge fires at
                    // t_retract and the drive starts it as the plunge completes.
                    const StrikeCtl ctl = copyCtl();
                    const auto err = s_gantry->queueRetractToSafeZ(
                        static_cast<uint32_t>(ctl.retract_mm_s + 0.5f),
                        static_cast<uint32_t>(ctl.accel_mm_s2 + 0.5f),
                        static_cast<uint32_t>(ctl.decel_mm_s2 + 0.5f));
                    if (err != Gantry::GantryError::OK) {
                        strikeAbort("Z retract queue refused");
                    }
                } else {
                    strikeAbort("Z release failed");
                }
            }
            if ((bits & kStrikeGrip) != 0 && takeDue(StrikeEvent::Grip, now)) {
                s_grip_cmd_us.store(esp_timer_get_time(), std::memory_order_release);
                gripMark(1);
                const bool gripped = s_gantry->grip(true);
                gripMark(0);
                if (gripped) {
                    s_cell_in_jaws.store(true, std::memory_order_release);
                    s_grip_fired_us.store(esp_timer_get_time(), std::memory_order_release);
                } else {
                    strikeAbort("grip write failed");
                }
            }
            if ((bits & kStrikeRetract) != 0 && takeDue(StrikeEvent::Retract, now)) {
                // Zero dwell: the edge for the queued retract. The drive holds it
                // until the plunge completes, then reverses with no stop between.
                const int64_t cmd_us = esp_timer_get_time();
                if (s_gantry->releasePreloadedMotion()) {
                    s_retract_fired_us.store(cmd_us, std::memory_order_release);
                    s_retract_done_us.store(esp_timer_get_time(), std::memory_order_release);
                    float z = 0.0f;
                    if (s_gantry->sampleZFeedbackMm(z)) {
                        s_retract_z_mm.store(z, std::memory_order_release);
                    }
                } else {
                    strikeAbort("Z retract edge refused (queue not ready)");
                }
            }
        }
    }
}

static void noteBelt(const PickIntercept::ConveyorFrame& frame) {
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

static void clearParts() {
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

static void beltSimCb(void*) {
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

static void traceTask(void*) {
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // Retimes move z_at_us, so it is re-read while waiting.
        while (s_trace_run.load(std::memory_order_acquire) &&
               s_trace_stop_at_us.load(std::memory_order_acquire) == 0 &&
               esp_timer_get_time() < copyCtl().z_at_us - kTraceLeadUs) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        TickType_t wake = xTaskGetTickCount();
        while (s_trace_run.load(std::memory_order_acquire)) {
            const int64_t now = esp_timer_get_time();
            const int64_t stop_at = s_trace_stop_at_us.load(std::memory_order_acquire);
            if ((stop_at != 0 && now >= stop_at) || now - s_trace.t0_us > kTraceMaxUs) {
                break;
            }
            float z = 0.0f;
            if (s_gantry != nullptr && s_gantry->sampleZFeedbackMm(z)) {
                const int at = s_trace.total % kTraceCap;
                s_trace.t_us[at] = static_cast<int32_t>(now - s_trace.t0_us);
                s_trace.z_mm[at] = z;
                ++s_trace.total;
            }
            vTaskDelayUntil(&wake, 1);
        }
        if (s_trace.total > kTraceCap) {
            const int oldest = s_trace.total % kTraceCap;
            std::rotate(s_trace.t_us, s_trace.t_us + oldest, s_trace.t_us + kTraceCap);
            std::rotate(s_trace.z_mm, s_trace.z_mm + oldest, s_trace.z_mm + kTraceCap);
        }
        s_trace.n = s_trace.total < kTraceCap ? s_trace.total : kTraceCap;
        s_trace_run.store(false, std::memory_order_release);
    }
}

// Starts once the plunge is preloaded and held, so the first samples are the hover.
static void traceStart(uint32_t item_id) {
    if (s_trace_task == nullptr || s_trace_run.load(std::memory_order_acquire)) {
        return;
    }
    s_trace.item_id = item_id;
    s_trace.n = 0;
    s_trace.total = 0;
    s_trace.complete = false;
    s_trace.t0_us = esp_timer_get_time();
    s_trace_stop_at_us.store(0, std::memory_order_release);
    s_trace_run.store(true, std::memory_order_release);
    xTaskNotifyGive(s_trace_task);
}

// Records the strike stamps and lets the trace run on to the retract settle.
static void traceFinish(const PickEval& eval, float vz_mm_s, float az_mm_s2, float belt_mm_s) {
    if (!s_trace_run.load(std::memory_order_acquire)) {
        return;
    }
    s_trace.ctl = copyCtl();
    s_trace.z_fired_us = s_z_fired_us.load(std::memory_order_acquire);
    s_trace.grip_cmd_us = s_grip_cmd_us.load(std::memory_order_acquire);
    s_trace.grip_fired_us = s_grip_fired_us.load(std::memory_order_acquire);
    s_trace.retract_cmd_us = s_retract_fired_us.load(std::memory_order_acquire);
    s_trace.retract_done_us = s_retract_done_us.load(std::memory_order_acquire);
    s_trace.z_hover_mm = eval.hover.z;
    s_trace.z_grasp_mm = eval.pick.z;
    s_trace.vz_mm_s = vz_mm_s;
    s_trace.az_mm_s2 = az_mm_s2;
    s_trace.belt_mm_s = belt_mm_s;
    s_trace.tau_plunge_s = static_cast<float>(eval.sim.tau_plunge_s);
    s_trace.complete = true;
    const int64_t from = s_trace.retract_cmd_us != 0 ? s_trace.retract_cmd_us
                                                     : esp_timer_get_time();
    s_trace_stop_at_us.store(from + kTraceTailUs, std::memory_order_release);
}

static uint32_t u32(float v) {
  if (v < 1.0f) {
    return 1;
  }
  if (v > 100000.0f) {
    return 100000;
  }
  return static_cast<uint32_t>(v + 0.5f);
}

static PickConfig pickConfig(const Gantry::JointLimits& lim) {
  const auto& c = Config::AppConfig::instance().data();
  PickConfig p;
  p.fingers.open_mm = c.pick_finger_open_mm;
  p.fingers.thick_mm = c.pick_finger_thick_mm;
  p.fingers.width_mm = c.pick_finger_width_mm;
  p.fingers.jaw_mm_s = c.pick_jaw_speed_mm_s;
  for (int i = 0; i < PickIntercept::kClassCount; ++i) {
    p.jaw_override_s[i] = static_cast<float>(c.pick_jaw_ms[i]) * 0.001f;
  }
  // Picks run on the global gantry motion profile (live). The X approach uses
  // the default speed unless a later part needs the speed ceiling; theta is
  // capped by the axis maximum.
  p.limits.speed = static_cast<float>(c.gantry_default_speed_mm_per_s);
  p.limits.fast_speed = static_cast<float>(c.gantry_path_max_speed_mm_per_s);
  p.limits.accel = static_cast<float>(c.gantry_default_accel_mm_per_s2);
  p.limits.decel = static_cast<float>(c.gantry_default_decel_mm_per_s2);
  p.limits.theta_cap = static_cast<float>(c.gantry_default_speed_deg_per_s);
  if (c.axis_theta_max_speed_deg_per_s > 0.0f &&
      c.axis_theta_max_speed_deg_per_s < p.limits.theta_cap) {
    p.limits.theta_cap = c.axis_theta_max_speed_deg_per_s;
  }
  p.limits.theta_accel = static_cast<float>(c.gantry_default_accel_deg_per_s2);
  p.limits.theta_decel = static_cast<float>(c.gantry_default_decel_deg_per_s2);
  p.limits.z_cap = AXIS_Z_PICK_SPEED_FRACTION * s_gantry->zSpeedCapMmS();
  p.limits.theta_start_frac = 0.0f;
  p.valve_s = static_cast<float>(c.pick_valve_ms) * 0.001f;
  p.guard_s = static_cast<float>(c.pick_guard_ms) * 0.001f;
  p.close_s = static_cast<float>(c.gantry_gripper_close_time_ms) * 0.001f;
  p.valve_on_s = static_cast<float>(c.pick_valve_on_ms) * 0.001f;
  p.off_plunge_ms = static_cast<int>(c.pick_offset_plunge_ms);
  p.off_close_ms = static_cast<int>(c.pick_offset_close_ms);
  p.off_retract_ms = static_cast<int>(c.pick_offset_retract_ms);
  p.dropin = c.pick_dropin_enable;
  p.col_x_min = c.gantry_conveyor_collision_x_min_mm;
  p.z_belt = c.gantry_conveyor_collision_z_min_mm;
  p.z_safe = s_gantry->traverseClearanceZMm();
  // Z stroke measured by this session's calibration (switch-limited, so the
  // gripper just reaches the belt). Before calibration, picks are blocked
  // anyway; the joint limit is the fallback.
  const int32_t z_len_mm = s_gantry->zAxisLengthMm();
  p.stroke = z_len_mm > 0 ? static_cast<float>(z_len_mm) : lim.z_max;
  if (lim.z_max > 1.0f && lim.z_max < p.stroke) {
      p.stroke = lim.z_max;
  }
  p.jaw_offset = c.pick_jaw_offset_deg;
  p.x_off = c.conveyor_x_across_to_gantry_x_offset_mm;
  p.cam_y = c.conveyor_y_cam_mm;
  p.pick_y = c.conveyor_y_pick_mm;
  return p;
}

// Grasp: a cell under 30 mm tall is gripped 3 mm above the belt; a taller one
// half its height down from the object top, capped at 25.4 mm below that top.
// The hover clears the tallest tracked cell unless the fingers already hover
// at hover_at_mm.
static ZPlan zPlan(const PickConfig& p, float object_h_mm, float tallest_mm,
                   float hover_at_mm = -1.0f) {
  ZPlan z;
  if (object_h_mm > 0.0f && object_h_mm < 30.0f) {
    z.grip = p.z_belt - 3.0f;
  } else {
    z.grip = PickIntercept::graspJointZMm(p.z_belt, object_h_mm);
  }
  z.top = p.z_belt - object_h_mm;
  z.hover = p.z_belt - tallest_mm - PickIntercept::kHoverClearanceMm;
  if (hover_at_mm >= 0.0f) {
    z.hover = hover_at_mm;
  }
  if (z.hover < p.z_safe) {
    z.hover = p.z_safe;
  }
  const float v = p.limits.z_cap > 1.0f ? p.limits.z_cap : p.limits.speed;
  z.top_s = PickIntercept::kTickS +
            PickIntercept::zToTopS(z.hover, z.top, z.grip, v, p.limits.accel, p.limits.decel);
  z.tail_s = PickIntercept::kTickS +
             PickIntercept::profileSeconds(z.grip - z.hover, v, p.limits.accel, p.limits.decel);
  return z;
}

// Simulation scheduler timing (strike_plan.h). Geometry and Z limits come
// from the firmware config; the pneumatic jaw model keeps the sim constants.
static StrikePlan::Plan strikePlan(const PickConfig& p, const Gantry::JointLimits& lim,
                                   float width_mm, float height_mm, float theta_deg,
                                   float belt_mm_s, float z_grasp_mm) {
    StrikePlan::Inputs in;
    in.width_mm = width_mm;
    in.height_mm = height_mm;
    in.theta_deg = theta_deg;
    in.belt_mm_s = belt_mm_s;
    in.z_belt_mm = p.z_belt;
    in.z_ceiling_mm = p.z_safe;
    in.z_min_mm = lim.z_min;
    in.z_grasp_mm = z_grasp_mm;
    in.pad_half_open_mm = 0.5 * p.fingers.open_mm;
    in.vz_mm_s = p.limits.z_cap;
    in.az_mm_s2 = p.limits.accel;
    in.dz_mm_s2 = p.limits.decel;
    in.valve_delay_s = p.valve_on_s;
    // fin_spd_mm_s: 210 mm/s = SCHUNK KGG 100-80 datasheet, 40 mm per jaw in
    // 0.19 s at 6 bar (movement only; valve and hose time are not included).
    in.jaw_speed_mm_s = p.fingers.jaw_mm_s;
    in.plunge_offset_ms = p.off_plunge_ms;
    in.close_offset_ms = p.off_close_ms;
    in.retract_offset_ms = p.off_retract_ms;
    return StrikePlan::plan(in);
}

// Strike schedule (seconds from now) for a strike fixed to arrival.
static PickIntercept::StrikeSchedule simSchedule(float arrive_s, float belt_mm_s,
                                                 const PickIntercept::MotionBudget& budget,
                                                 const PickIntercept::GripGeometry& grip,
                                                 const StrikePlan::Plan& sim) {
    PickIntercept::StrikeSchedule s;
    s.mode = grip.mode;
    s.arrive_s = arrive_s;
    s.approach_s = budget.approach_s;
    s.x_clearance_mm = grip.x_clearance_mm;
    s.side_gap_mm = grip.side_gap_mm;
    s.approach_speed_mm_s = budget.approach_speed_mm_s;
    s.descend_speed_mm_s = budget.descend_speed_mm_s;
    s.window_s = PickIntercept::kWaitS;
    if (!sim.ok) {
        s.reason = sim.reason;
        return s;
    }
    s.z_motion_s = static_cast<float>(sim.tau_plunge_s);
    s.z_start_s = arrive_s + static_cast<float>(sim.t_z_s);
    s.z_done_s = s.z_start_s + s.z_motion_s;
    s.grip_s = arrive_s + static_cast<float>(sim.t_close_s);
    s.slack_s = s.z_start_s - (budget.approach_s + PickIntercept::kPreloadS + PickIntercept::kSettleS);
    if (belt_mm_s <= 0.0f || arrive_s >= PickIntercept::kWaitS) {
        s.reason = "belt stopped";
        return s;
    }
    if (arrive_s <= 0.0f) {
        s.reason = "missed";
        return s;
    }
    if (budget.approach_s >= PickIntercept::kWaitS || s.slack_s < 0.0f) {
        s.reason = "late";
        return s;
    }
    s.feasible = true;
    return s;
}

static float partHeightMm(const PickIntercept::TrackedPart& part) {
  return part.height_from_belt_mm;
}

static float tallestMm(const PickIntercept::TrackedPart* others, int n,
                       const PickIntercept::TrackedPart& self) {
  float tallest = partHeightMm(self);
  for (int i = 0; i < n; ++i) {
    const float h = partHeightMm(others[i]);
    if (h > tallest) {
      tallest = h;
    }
  }
  return tallest;
}

static PickIntercept::StrikeTiming strikeTiming(const PickConfig& p, int cls,
                                                const PickIntercept::GripGeometry& grip,
                                                const ZPlan& z, uint32_t epoch,
                                                double s_arrive_mm) {
  PickIntercept::StrikeTiming t;
  t.mode = grip.mode;
  t.epoch = epoch;
  t.s_arrive_mm = s_arrive_mm;
  t.side_gap_mm = grip.side_gap_mm;
  t.along_factor = grip.along_factor;
  t.jaw_mm_s = p.fingers.jaw_mm_s;
  t.jaw_override_s = p.jaw_override_s[cls];
  t.valve_s = p.valve_s;
  t.guard_s = p.guard_s;
  t.z_top_s = z.top_s;
  t.z_tail_s = z.tail_s;
  return t;
}

static void markLate(PickEval& eval, const char* why) {
  if (eval.strike.feasible) {
    eval.strike.feasible = false;
    eval.reason = why;
  }
}

// frozen_joint: jaw joint already committed (Z below the band); the pose is
// then the hover / depth and the budget starts there.
static PickEval evaluate(const PickConfig& p, const Gantry::JointConfig& pose,
                         const Gantry::JointLimits& lim, const PickIntercept::TrackedPart& part,
                         const PickIntercept::BeltClock& belt, int64_t now_us,
                         const PickIntercept::TrackedPart* others, int n_others,
                         const float* frozen_joint) {
  PickEval eval;
  eval.s_arrive_mm = part.s_arrive_mm;
  if (part.item_id == 0) {
    eval.strike.reason = "item id";
    eval.reason = "item id";
    eval.permanent = true;
    return eval;
  }
  if (const char* bad = PickIntercept::classPickReject(part.battery_class)) {
    eval.strike.reason = bad;
    eval.reason = bad;
    eval.permanent = true;
    return eval;
  }
  const int cls = PickIntercept::classIndex(part.battery_class);
  if (const char* size_bad =
          PickIntercept::visionSizeReject(part.width_mm, part.length_mm, part.height_from_belt_mm,
                                          p.fingers.open_mm)) {
    eval.strike.reason = size_bad;
    eval.reason = size_bad;
    eval.permanent = true;
    return eval;
  }
  const PickIntercept::GripAxes axes =
      PickIntercept::gripAcrossWidth(part.width_mm, part.length_mm, part.theta_deg);
  const PickIntercept::CellGeom cell =
      PickIntercept::cellFromVision(axes.width_mm, axes.length_mm, part.height_from_belt_mm);

  float jaw_joint = 0.0f;
  if (frozen_joint != nullptr) {
    jaw_joint = *frozen_joint;
  } else {
    const PickIntercept::JawChoice choice = PickIntercept::chooseSideJaw(
        pose.theta, axes.theta_deg, p.jaw_offset, lim.theta_min, lim.theta_max);
    if (!choice.ok) {
      eval.strike.reason = "theta range";
      eval.reason = "theta range";
      eval.permanent = true;
      return eval;
    }
    jaw_joint = Gantry::Path::commandThetaDeg(pose.theta, choice.joint_deg);
  }
  eval.jaw_world_deg = Gantry::thetaPoseFromJointDeg(jaw_joint);

  PickIntercept::Occupant occ[PickIntercept::kTrackerCap];
  int n_occ = 0;
  for (int i = 0; i < n_others && n_occ < PickIntercept::kTrackerCap; ++i) {
    if (part.token != 0 && others[i].token == part.token) {
      continue;
    }
    const PickIntercept::GripAxes other_grip = PickIntercept::gripAcrossWidth(
        others[i].width_mm, others[i].length_mm, others[i].theta_deg);
    occ[n_occ].dx_mm = others[i].x_across_mm - part.x_across_mm;
    occ[n_occ].remain_mm = others[i].s_arrive_mm - PickIntercept::beltPositionMm(belt, now_us);
    occ[n_occ].cell_deg = other_grip.theta_deg;
    occ[n_occ].cell = PickIntercept::cellFromVision(other_grip.width_mm, other_grip.length_mm,
                                                   others[i].height_from_belt_mm);
    ++n_occ;
  }

  eval.z = zPlan(p, partHeightMm(part), tallestMm(others, n_others, part),
                 frozen_joint != nullptr ? pose.z : -1.0f);
  const float tcp_x = p.x_off + part.x_across_mm;
  const float joint_x = s_gantry != nullptr
                            ? s_gantry->inverseKinematics(Gantry::EndEffectorPose(tcp_x, 0.0f, 0.0f, 0.0f)).x
                            : tcp_x;
  eval.pick = Gantry::JointConfig(joint_x, eval.z.grip, jaw_joint);

  // Lane and side gap are kept for the log and the neighbour test.
  const PickIntercept::GripGeometry grip = PickIntercept::classifyClearance(
      axes.theta_deg, PickIntercept::closeAxisDeg(eval.jaw_world_deg), cell, p.fingers, p.dropin);
  eval.timing = strikeTiming(p, cls, grip, eval.z, belt.epoch, part.s_arrive_mm);

  // Simulation scheduler: hover, plunge, close and retract from the vision size.
  // The pre-position is the hover itself, inside the SAFE_Z band.
  eval.sim = strikePlan(p, lim, PickIntercept::beltCrossMm(cell), cell.height_mm, axes.theta_deg,
                        belt.speed_mm_s, eval.z.grip);
  const float hover_z = eval.sim.ok ? static_cast<float>(eval.sim.z_hover_mm) : p.z_safe;
  eval.hover = Gantry::JointConfig(joint_x, hover_z, jaw_joint);
  eval.approach = eval.hover;

  PickIntercept::MotionLimits motion = p.limits;
  if (part.approach_fast && frozen_joint == nullptr) {
      motion.speed = motion.fast_speed;
  }
  const PickIntercept::MotionBudget budget =
      frozen_joint != nullptr
          ? PickIntercept::budgetPick(pose.x, pose.z, pose.theta, pose.x, pose.z, pose.z,
                                      eval.pick.z, pose.theta, motion)
          : PickIntercept::budgetPick(pose.x, pose.z, pose.theta, eval.approach.x,
                                      eval.approach.z, eval.approach.z, eval.pick.z,
                                      eval.approach.theta, motion);
  eval.approach_deg_s = motion.theta_cap;

  const double s_now = PickIntercept::beltPositionMm(belt, now_us);
  const float arrive_s = PickIntercept::secondsToPlane(
      static_cast<float>(part.s_arrive_mm - s_now), belt.speed_mm_s);
  eval.strike = simSchedule(arrive_s, belt.speed_mm_s, budget, grip, eval.sim);
  eval.reason = eval.strike.reason;
  if (!eval.sim.ok && belt.speed_mm_s > 0.0f) {
      // Size, jaw fit or Z limits: the same detection never becomes pickable.
      eval.permanent = true;
  }

  const Gantry::JointConfig& lowest = eval.pick;
  if (lowest.z > p.stroke + 0.05f || lowest.z < lim.z_min || eval.z.grip > p.z_belt) {
    eval.strike.feasible = false;
    eval.reason = "pick Z unreachable";
    eval.permanent = true;
  } else if (PickIntercept::inConveyorCollision(lowest.x, lowest.z, p.col_x_min, p.z_belt) ||
             PickIntercept::inConveyorCollision(eval.approach.x, eval.approach.z, p.col_x_min,
                                                p.z_belt)) {
    eval.strike.feasible = false;
    eval.reason = "collision";
    eval.permanent = true;
  }
  if (!eval.strike.feasible) {
    return eval;
  }

  // Neighbours must stay out of the finger lanes from the plunge until Z is up.
  const float retract_s =
      PickIntercept::profileSeconds(std::fabs(eval.pick.z - eval.hover.z), p.limits.z_cap,
                                    p.limits.accel, p.limits.decel) +
      PickIntercept::kPreloadS + PickIntercept::kSettleS;
  const float t_up = arrive_s + static_cast<float>(eval.sim.t_retract_s) + retract_s;
  const float free_s = PickIntercept::fingersFreeFromS(
      occ, n_occ, PickIntercept::closeAxisDeg(eval.jaw_world_deg), p.fingers, belt.speed_mm_s,
      eval.strike.z_start_s, t_up);
  if (free_s > eval.strike.z_start_s + 1.0e-3f) {
      markLate(eval, "neighbour");
  }
  return eval;
}

static void formatEval(const PickEval& eval, char* out, size_t n) {
  std::snprintf(out, n,
                "%s jaw %.1f arrive %.3fs prepos %.3fs z_start %.3fs z_done %.3fs grip %.3fs "
                "slack %.3fs window %.3fs xgap %.1fmm side %.1fmm contact %.0fmm/s vz %.0f "
                "joint %.1f %.1f %.1f %s | sim z/close/retract %.1f/%.1f/%.1f ms "
                "dwell %.0f ms nom %.1f/%.1f/%.1f hover %.1f%s",
                PickIntercept::clearanceName(eval.strike.mode),
                static_cast<double>(eval.jaw_world_deg),
                static_cast<double>(eval.strike.arrive_s),
                static_cast<double>(eval.strike.approach_s),
                static_cast<double>(eval.strike.z_start_s),
                static_cast<double>(eval.strike.z_done_s),
                static_cast<double>(eval.strike.grip_s),
                static_cast<double>(eval.strike.slack_s),
                static_cast<double>(eval.strike.window_s >= PickIntercept::kWaitS
                                        ? -1.0f
                                        : eval.strike.window_s),
                static_cast<double>(eval.strike.x_clearance_mm),
                static_cast<double>(eval.strike.side_gap_mm),
                static_cast<double>(eval.strike.contact_mm_s),
                static_cast<double>(eval.strike.descend_speed_mm_s),
                static_cast<double>(eval.pick.x), static_cast<double>(eval.pick.z),
                static_cast<double>(eval.pick.theta), eval.reason,
                eval.sim.t_z_s * 1000.0, eval.sim.t_close_s * 1000.0, eval.sim.t_retract_s * 1000.0,
                eval.sim.dwell_s * 1000.0, eval.sim.t_z_nominal_s * 1000.0,
                eval.sim.t_close_nominal_s * 1000.0, eval.sim.t_retract_nominal_s * 1000.0,
                eval.sim.z_hover_mm,
                eval.sim.clamped ? " clamped" : "");
}

static int copyTracked(PickIntercept::TrackedPart* out) {
  portENTER_CRITICAL(&s_belt_mux);
  const int n = PickIntercept::trackCopy(s_parts, out, PickIntercept::kTrackerCap);
  portEXIT_CRITICAL(&s_belt_mux);
  return n;
}

static bool copyActive(PickIntercept::TrackedPart* out) {
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

static void classifyFollowers(const PickConfig& p, const Gantry::JointLimits& lim,
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

static void processVision() {
  if (s_vision_queue == nullptr) {
    return;
  }
  VisionItem item;
  while (xQueueReceive(s_vision_queue, &item, 0) == pdTRUE) {
    acceptDetect(item);
  }
}

static bool queueVision(const VisionItem& item) {
  if (s_vision_queue == nullptr || xQueueSend(s_vision_queue, &item, 0) != pdTRUE) {
    return false;
  }
  return true;
}

static uint32_t beltEpoch() {
  portENTER_CRITICAL(&s_belt_mux);
  const uint32_t epoch = s_belt.epoch;
  portEXIT_CRITICAL(&s_belt_mux);
  return epoch;
}

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

static void publishGantryStatus(PickSchedulerTaskConfig *cfg) {
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

static bool running() {
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
    idleSlice(cfg, 10);
  }
  return running();
}

static bool moveJoint(PickSchedulerTaskConfig* cfg, const Gantry::JointConfig& joint,
                      float speed, float accel, float decel, float theta_speed,
                      float theta_accel, float theta_start_frac) {
  if (cfg->gantry->isAlarmActive()) {
    ESP_LOGW(TAG, "axis alarm — move skipped");
    return false;
  }
  const auto err = cfg->gantry->moveTo(joint, u32(speed), u32(theta_speed), u32(accel),
                                       u32(decel), u32(theta_accel), u32(theta_accel),
                                       theta_start_frac);
    if (err != Gantry::GantryError::OK) {
    ESP_LOGW(TAG, "joint move rejected (%d) x=%.2f z=%.2f th=%.2f", static_cast<int>(err),
             static_cast<double>(joint.x), static_cast<double>(joint.z),
             static_cast<double>(joint.theta));
    return false;
  }
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

static bool moveApproach(PickSchedulerTaskConfig* cfg, const Gantry::JointConfig& joint,
                         const PickConfig& p, float x_mm_s, float theta_deg_s) {
  return moveJoint(cfg, joint, x_mm_s, p.limits.accel, p.limits.decel, theta_deg_s,
                   p.limits.theta_accel, p.limits.theta_start_frac);
}

static bool moveZ(PickSchedulerTaskConfig* cfg, const Gantry::JointConfig& joint,
                  const PickConfig& p, float z_mm_s) {
  return moveJoint(cfg, joint, z_mm_s, p.limits.accel, p.limits.decel, p.limits.theta_cap,
                   p.limits.theta_accel, 0.0f);
}

static bool retract(PickSchedulerTaskConfig* cfg, const PickConfig& p) {
  const auto err = cfg->gantry->retractToSafeZ(u32(p.limits.z_cap), u32(p.limits.accel),
                                               u32(p.limits.decel));
  if (err != Gantry::GantryError::OK) {
    ESP_LOGE(TAG, "Z retract refused (%d)", static_cast<int>(err));
    return false;
  }
  while (cfg->gantry->isBusy()) {
    idleSlice(cfg, 10);
  }
  return cfg->gantry->zInTraverseBand();
}

static void abortStrike(PickSchedulerTaskConfig* cfg, const PickConfig& p, const char* why) {
  disarmStrike();
  const bool gripped = s_cell_in_jaws.load(std::memory_order_acquire);
  ESP_LOGW(TAG, "strike aborted (%s) — retracting Z%s", why,
           gripped ? " with the jaws closed" : "");
  cfg->gantry->requestAbort();
  if (!retract(cfg, p)) {
    ESP_LOGE(TAG, "Z is not in the SAFE_Z band after abort");
  }
}

static const char* strikeAbortReason() {
  return s_strike_abort.load(std::memory_order_acquire);
}

static bool dropHeld(PickSchedulerTaskConfig* cfg, const PickConfig& p, float approach_mm_s,
                     float approach_deg_s) {
  const auto& c = Config::AppConfig::instance().data();
  const Gantry::JointConfig here = cfg->gantry->getCurrentJointConfig();
  const float theta = c.pick_drop_any_theta ? here.theta : c.gantry_cal_theta_park_deg;
  const Gantry::JointConfig drop_safe(c.gantry_cal_x_park_mm, p.z_safe, theta);
  const Gantry::JointConfig drop(c.gantry_cal_x_park_mm, c.gantry_cal_z_park_mm, theta);
  if (PickIntercept::inConveyorCollision(drop.x, drop.z, c.gantry_conveyor_collision_x_min_mm,
                                         c.gantry_conveyor_collision_z_min_mm)) {
    ESP_LOGW(TAG, "bin drop violates the conveyor envelope; gripper stays closed");
    return false;
  }
  if (!retract(cfg, p) ||
      !moveApproach(cfg, drop_safe, p, approach_mm_s, approach_deg_s) ||
      !moveZ(cfg, drop, p, p.limits.z_cap)) {
    ESP_LOGW(TAG, "transfer failed; gripper stays closed");
    return false;
  }
  ESP_LOGI(TAG, "At bin drop X=%.2f mm Z=%.2f mm theta=%.2f deg. Opening gripper.",
           static_cast<double>(drop.x), static_cast<double>(drop.z),
           static_cast<double>(drop.theta));
  if (!cfg->gantry->grip(false)) {
    ESP_LOGE(TAG, "gripper open write failed at the bin");
    return false;
  }
  s_cell_in_jaws.store(false, std::memory_order_release);
  return delayMs(cfg, c.gantry_gripper_open_time_ms);
}

static bool takeNextPart(PickIntercept::TrackedPart* out) {
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

static PickEval evaluateActive(const PickConfig& p, const Gantry::JointLimits& lim,
                               const PickIntercept::TrackedPart& fallback,
                               const float* frozen_joint) {
  PickIntercept::TrackedPart part = fallback;
  (void)copyActive(&part);
  PickIntercept::TrackedPart others[PickIntercept::kTrackerCap];
  const int n = copyTracked(others);
  return evaluate(p, s_gantry->getCurrentJointConfig(), lim, part, copyBelt(),
                  esp_timer_get_time(), others, n, frozen_joint);
}

// Waits for plunge, close and retract to fire and for the retract to finish.
// False on halt, abort, or an event that did not fire in time.
static bool waitStrikeDone(PickSchedulerTaskConfig* cfg) {
    while (true) {
        if (!running()) {
            disarmStrike();
            cfg->gantry->requestAbort();
            return false;
        }
        if (strikeAbortReason() != nullptr) {
            return false;
        }
        // Clock first, then the snapshot: events only go pending -> fired, so
        // a snapshot that still says pending was also pending at `now`. The
        // other order let the strike task (higher priority) fire the retract
        // in between and produce a false "retract did not fire" (2026-10-20).
        const int64_t now = esp_timer_get_time();
        const StrikeCtl ctl = copyCtl();
        if (ctl.z_pending && cfg->gantry->isPreloadHeld() && now > ctl.z_at_us + kPreloadDeadlineUs) {
            strikeAbort("preload deadline");
            return false;
        }
        if (ctl.grip_pending && now > ctl.grip_at_us + kGripDeadlineUs) {
            strikeAbort("grip did not fire");
            return false;
        }
        if (ctl.retract_pending && now > ctl.retract_at_us + kRetractDeadlineUs) {
            strikeAbort("retract did not fire");
            return false;
        }
        const int64_t retract_us = s_retract_fired_us.load(std::memory_order_acquire);
        if (retract_us != 0) {
            if (!cfg->gantry->isBusy()) {
                return true;
            }
            if (now > retract_us + kRetractDoneUs) {
                strikeAbort("retract did not finish");
                return false;
            }
        }
        // +Z is down: the deepest point the plunge reached before reversing.
        float z = 0.0f;
        if (cfg->gantry->sampleZFeedbackMm(z) &&
            z > s_peak_z_mm.load(std::memory_order_relaxed)) {
            s_peak_z_mm.store(z, std::memory_order_relaxed);
        }
        vTaskDelay(pdMS_TO_TICKS(2));
        processVision();
    }
}

static double errMs(int64_t fired_us, int64_t target_us) {
    return fired_us == 0 ? 0.0 : static_cast<double>(fired_us - target_us) / 1000.0;
}

// One pick with the simulation scheduler: pre-position at the hover (in the
// SAFE_Z band), preload and hold the Z plunge, then fire plunge, close and
// zero-dwell retract at fixed offsets from arrival.
static void runPick(PickSchedulerTaskConfig* cfg, const PickIntercept::TrackedPart& part) {
    const int64_t started = esp_timer_get_time();
    const Gantry::JointLimits lim = cfg->gantry->getJointLimits();
    const PickConfig p = pickConfig(lim);

    PickEval eval = evaluateActive(p, lim, part, nullptr);
    if (!eval.strike.feasible) {
        ESP_LOGW(TAG, "pick item=%lu skipped (%s)", static_cast<unsigned long>(part.item_id),
                 eval.reason);
        return;
    }
    char plan[512];
    formatEval(eval, plan, sizeof(plan));
    ESP_LOGI(TAG, "pick item=%lu %s", static_cast<unsigned long>(part.item_id), plan);
    if (std::fabs(eval.sim.dwell_s) >= kDwellWarnS) {
        ESP_LOGW(TAG, "pick item=%lu bottom dwell %+.0f ms (pk_off_retract %d - pk_off_plunge %d)%s",
                 static_cast<unsigned long>(part.item_id), eval.sim.dwell_s * 1000.0,
                 p.off_retract_ms, p.off_plunge_ms,
                 eval.sim.dwell_s > 0.0 ? ": Z waits at depth"
                                        : ": not applied, the retract waits for the plunge end");
    }
    classifyFollowers(p, lim, part, eval, copyBelt(), esp_timer_get_time());

    if (!retract(cfg, p) ||
        !moveApproach(cfg, eval.approach, p, eval.strike.approach_speed_mm_s, eval.approach_deg_s)) {
        return;
    }
    eval = evaluateActive(p, lim, part, nullptr);
    if (eval.strike.feasible) {
        const Gantry::JointConfig here = cfg->gantry->getCurrentJointConfig();
        if (std::fabs(here.theta - eval.approach.theta) > PickIntercept::kArriveEpsDeg) {
            if (!moveApproach(cfg, eval.approach, p, eval.strike.approach_speed_mm_s,
                              eval.approach_deg_s)) {
                return;
            }
            eval = evaluateActive(p, lim, part, nullptr);
        }
    }
    if (!eval.strike.feasible) {
        ESP_LOGW(TAG, "pick item=%lu missed during pre-position (%s)",
                 static_cast<unsigned long>(part.item_id), eval.reason);
        return;
    }

    // The hover can sit above the band ceiling for a tall object.
    const float jaw_joint = cfg->gantry->getCurrentJointConfig().theta;
    if (!moveZ(cfg, eval.hover, p, eval.strike.descend_speed_mm_s)) {
        (void)retract(cfg, p);
        return;
    }
    eval = evaluateActive(p, lim, part, &jaw_joint);
    if (!eval.strike.feasible) {
        abortStrike(cfg, p, eval.reason);
        return;
    }

    const int64_t arrive_at =
        esp_timer_get_time() + static_cast<int64_t>(eval.strike.arrive_s * 1.0e6f);
    const Gantry::JointConfig here = cfg->gantry->getCurrentJointConfig();
    const Gantry::JointConfig strike(here.x, eval.pick.z, here.theta);
    const auto pre = cfg->gantry->preloadJointMove(
        strike, u32(eval.strike.descend_speed_mm_s), u32(p.limits.accel), u32(p.limits.decel));
    if (pre != Gantry::GantryError::OK || !cfg->gantry->isPreloadHeld()) {
        abortStrike(cfg, p, "Z preload did not hold StartMotion");
        return;
    }
    armStrike(eval.timing, eval.jaw_world_deg, part.battery_class, eval.sim, arrive_at,
              p.limits.z_cap, p.limits.accel, p.limits.decel);
    traceStart(part.item_id);

    const bool struck = waitStrikeDone(cfg);
    traceFinish(eval, eval.strike.descend_speed_mm_s, p.limits.accel, copyBelt().speed_mm_s);
    if (!struck) {
        const char* why = strikeAbortReason();
        if (why != nullptr) {
            abortStrike(cfg, p, why);
        }
        return;
    }
    const StrikeCtl ctl = copyCtl();
    disarmStrike();
    const int64_t z_us = s_z_fired_us.load(std::memory_order_acquire);
    const int64_t grip_us = s_grip_fired_us.load(std::memory_order_acquire);
    const int64_t retract_us = s_retract_fired_us.load(std::memory_order_acquire);
    const int64_t retract_done_us = s_retract_done_us.load(std::memory_order_acquire);
    const float edge_z_mm = s_retract_z_mm.load(std::memory_order_acquire);
    const float depth_mm = s_peak_z_mm.load(std::memory_order_acquire);
    portENTER_CRITICAL(&s_belt_mux);
    PickIntercept::trackNoteTaken(s_parts, part);
    s_active_valid = false;
    portEXIT_CRITICAL(&s_belt_mux);

    ESP_LOGI(TAG,
             "pick actual item=%lu plunge_err_ms=%.1f close_err_ms=%.1f retract_err_ms=%.1f "
             "retract_call_ms=%.1f z_at_retract_edge %.2f deepest %.2f/%.2f mm "
             "offsets %d/%d/%d ms",
             static_cast<unsigned long>(part.item_id), errMs(z_us, ctl.z_at_us),
             errMs(grip_us, ctl.grip_at_us), errMs(retract_us, ctl.retract_at_us),
             errMs(retract_done_us, retract_us), static_cast<double>(edge_z_mm),
             static_cast<double>(depth_mm), static_cast<double>(eval.pick.z), p.off_plunge_ms,
             p.off_close_ms, p.off_retract_ms);
    if (depth_mm < eval.pick.z - kDepthShortWarnMm) {
        ESP_LOGW(TAG, "pick item=%lu Z reversed %.2f mm short of grasp depth",
                 static_cast<unsigned long>(part.item_id),
                 static_cast<double>(eval.pick.z - depth_mm));
    }

    if (!dropHeld(cfg, p, eval.strike.approach_speed_mm_s, eval.approach_deg_s)) {
        return;
    }
    s_last_cycle_ms = static_cast<float>(esp_timer_get_time() - started) / 1000.0f;
    s_status_us = 0;
    publishGantryStatus(cfg);
}

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
  if (!retract(cfg, stage_cfg) ||
      !moveApproach(cfg, stage, stage_cfg, stage_cfg.limits.speed, stage_cfg.limits.theta_cap)) {
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

// First sample at or after t_rel_us, or n when none.
static int traceIndexFrom(int32_t t_rel_us) {
    for (int i = 0; i < s_trace.n; ++i) {
        if (s_trace.t_us[i] >= t_rel_us) {
            return i;
        }
    }
    return s_trace.n;
}

static double traceMs(int64_t abs_us, int64_t ref_us) {
    return (abs_us == 0 || ref_us == 0) ? -1.0 : static_cast<double>(abs_us - ref_us) / 1000.0;
}

static void traceSummary(char* out, size_t out_len) {
    const ZTrace& t = s_trace;
    const int32_t z_rel = static_cast<int32_t>(t.z_fired_us - t.t0_us);
    const int32_t r_rel = static_cast<int32_t>(t.retract_cmd_us - t.t0_us);
    const int iz = traceIndexFrom(z_rel);
    const float z0 = t.z_mm[iz > 0 ? iz - 1 : 0];
    // +Z is down: the plunge raises z, the retract lowers it.
    int i_move = -1;
    int i_reach = -1;
    int i_peak = iz < t.n ? iz : 0;
    for (int i = iz; i < t.n; ++i) {
        if (i_move < 0 && t.z_mm[i] > z0 + kTraceMoveMm) {
            i_move = i;
        }
        if (i_reach < 0 && t.z_mm[i] >= t.z_grasp_mm - kTraceReachMm) {
            i_reach = i;
        }
        if (t.z_mm[i] > t.z_mm[i_peak]) {
            i_peak = i;
        }
    }
    const int ir = t.retract_cmd_us != 0 ? traceIndexFrom(r_rel) : t.n;
    const float z_at_r = ir < t.n ? t.z_mm[ir] : 0.0f;
    int i_rev = -1;
    float z_max_after = z_at_r;
    for (int i = ir; i < t.n; ++i) {
        if (t.z_mm[i] > z_max_after) {
            z_max_after = t.z_mm[i];
        }
        if (t.z_mm[i] < z_max_after - kTraceMoveMm) {
            i_rev = i;
            break;
        }
    }
    auto relMs = [&](int i, int32_t ref) {
        return i < 0 || i >= t.n ? -1.0 : static_cast<double>(t.t_us[i] - ref) / 1000.0;
    };
    snprintf(out, out_len,
             "ztrace item=%lu n=%d belt=%.0f vz=%.0f az=%.0f hover=%.2f grasp=%.2f "
             "z_fire_err=%.1f plunge_start=%.1f reach=%.1f (plan %.1f) peak=%.2f@%.1f "
             "retract_cmd=%.1f z_at_cmd=%.2f retract_start=%.1f call=%.1f "
             "grip_cmd=%.1f grip_write=%.1f [ms from z fire]",
             static_cast<unsigned long>(t.item_id), t.n, static_cast<double>(t.belt_mm_s),
             static_cast<double>(t.vz_mm_s), static_cast<double>(t.az_mm_s2),
             static_cast<double>(t.z_hover_mm), static_cast<double>(t.z_grasp_mm),
             traceMs(t.z_fired_us, t.ctl.z_at_us), relMs(i_move, z_rel), relMs(i_reach, z_rel),
             static_cast<double>(t.tau_plunge_s) * 1000.0, static_cast<double>(t.z_mm[i_peak]),
             relMs(i_peak, z_rel), traceMs(t.retract_cmd_us, t.z_fired_us),
             static_cast<double>(z_at_r), relMs(i_rev, z_rel),
             traceMs(t.retract_done_us, t.retract_cmd_us), traceMs(t.grip_cmd_us, t.z_fired_us),
             traceMs(t.grip_fired_us, t.grip_cmd_us));
}

bool pickSchedulerTrace(const char* args, char* out, size_t out_len) {
    if (out == nullptr || out_len == 0) {
        return false;
    }
    if (s_trace_run.load(std::memory_order_acquire)) {
        snprintf(out, out_len, "trace still recording (%d samples)", s_trace.total);
        return false;
    }
    if (!s_trace.complete || s_trace.n < 2) {
        snprintf(out, out_len, "no strike trace yet");
        return false;
    }
    int start = 0;
    int count = 0;
    if (args != nullptr && sscanf(args, "raw %d %d", &start, &count) >= 1) {
        if (count <= 0) {
            count = 200;
        }
        if (start < 0) {
            start = 0;
        }
        const int end = start + count < s_trace.n ? start + count : s_trace.n;
        const ZTrace& t = s_trace;
        ESP_LOGI(TAG, "ztrace meta item=%lu t0=0 z_at=%lld grip_at=%lld retract_at=%lld "
                 "z_fired=%lld grip_cmd=%lld grip_done=%lld retract_cmd=%lld retract_done=%lld",
                 static_cast<unsigned long>(t.item_id),
                 static_cast<long long>(t.ctl.z_at_us - t.t0_us),
                 static_cast<long long>(t.ctl.grip_at_us - t.t0_us),
                 static_cast<long long>(t.ctl.retract_at_us - t.t0_us),
                 static_cast<long long>(t.z_fired_us ? t.z_fired_us - t.t0_us : -1),
                 static_cast<long long>(t.grip_cmd_us ? t.grip_cmd_us - t.t0_us : -1),
                 static_cast<long long>(t.grip_fired_us ? t.grip_fired_us - t.t0_us : -1),
                 static_cast<long long>(t.retract_cmd_us ? t.retract_cmd_us - t.t0_us : -1),
                 static_cast<long long>(t.retract_done_us ? t.retract_done_us - t.t0_us : -1));
        for (int i = start; i < end; i += kTraceRawPerLine) {
            char line[400];
            int len = snprintf(line, sizeof(line), "zt %d", i);
            for (int j = i; j < end && j < i + kTraceRawPerLine && len > 0 &&
                            len < static_cast<int>(sizeof(line)) - 20;
                 ++j) {
                len += snprintf(line + len, sizeof(line) - len, " %ld:%.3f",
                                static_cast<long>(t.t_us[j]), static_cast<double>(t.z_mm[j]));
            }
            ESP_LOGI(TAG, "%s", line);
        }
        snprintf(out, out_len, "ztrace raw %d..%d of %d", start, end, s_trace.n);
        return true;
    }
    traceSummary(out, out_len);
    return true;
}

// gripmark and vismark share GPIO17 (the TFT backlight). The pin leaves the
// backlight PWM when the first marker turns on and returns when both are off.
static bool setMarker(std::atomic<bool>& flag, const std::atomic<bool>& other, bool on,
                      const char* name, const char* meaning, char* out, size_t out_len) {
    if (out == nullptr || out_len == 0) {
        return false;
    }
    const gpio_num_t pin = static_cast<gpio_num_t>(TFT_BLK_GPIO);
    if (on) {
        // Routing the pin to plain GPIO detaches the backlight PWM; the UI's
        // duty updates then go nowhere until both markers are off.
        gpio_config_t io = {};
        io.pin_bit_mask = 1ULL << TFT_BLK_GPIO;
        io.mode = GPIO_MODE_OUTPUT;
        if (gpio_config(&io) != ESP_OK) {
            snprintf(out, out_len, "GPIO%d config failed", TFT_BLK_GPIO);
            return false;
        }
        gpio_set_level(pin, 0);
        flag.store(true, std::memory_order_release);
        snprintf(out, out_len, "%s marker on GPIO%d (TFT backlight off): %s%s", name,
                 TFT_BLK_GPIO, meaning,
                 other.load(std::memory_order_acquire) ? " (other marker also on)" : "");
        return true;
    }
    flag.store(false, std::memory_order_release);
    gpio_set_level(pin, 0);
    if (other.load(std::memory_order_acquire)) {
        snprintf(out, out_len, "%s marker off; GPIO%d stays a marker for the other one", name,
                 TFT_BLK_GPIO);
        return true;
    }
    if (ledc_set_pin(TFT_BLK_GPIO, LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0) != ESP_OK) {
        snprintf(out, out_len, "marker off; backlight PWM not restored (reboot restores it)");
        return false;
    }
    snprintf(out, out_len, "%s marker off; backlight PWM restored on GPIO%d", name, TFT_BLK_GPIO);
    return true;
}

bool pickSchedulerGripMark(bool on, char* out, size_t out_len) {
    return setMarker(s_grip_mark, s_vis_mark, on, "grip",
                     "high from grip fire to MCP write done", out, out_len);
}

bool pickSchedulerVisMark(bool on, char* out, size_t out_len) {
    return setMarker(s_vis_mark, s_grip_mark, on, "vision",
                     "high from frame receipt to acceptDetect", out, out_len);
}

// Belt locator (beltcal): after bring-up the operator jogs (`move`) to each belt
// edge, the conveyor frame edges and the test battery; each joint X is saved to
// its cfg key at once. "z" records the joint Z with the open jaw tips
// on the belt (RAM until apply: the planner's belt surface, col_z_min_mm).
static float s_belt_z_mm = 0.0f;
static bool s_belt_z_set = false;

static bool beltCalReady(char* out, size_t out_len) {
    if (s_gantry == nullptr || !s_gantry->isWorkspaceCalibrated()) {
        snprintf(out, out_len, "run bring-up (calibrate all) first");
        return false;
    }
    return true;
}

bool pickSchedulerBeltCal(const char* args, char* out, size_t out_len) {
    if (args == nullptr || out == nullptr || out_len == 0) {
        return false;
    }
    auto& app = Config::AppConfig::instance();
    const auto& c = app.data();
    bool reboot = false;

    struct Edge {
        const char* arg;
        const char* key;
        const char* what;
    };
    static const Edge kEdges[] = {
        {"near", "cv_belt_near_mm", "belt near edge"},
        {"far", "cv_belt_far_mm", "belt far edge"},
        {"cnvyr near", "cv_cnv_near_mm", "conveyor near edge"},
        {"cnvyr far", "cv_cnv_far_mm", "conveyor far edge"},
        {"batt", "batt_x", "test battery"},
    };
    for (const Edge& e : kEdges) {
        if (std::strcmp(args, e.arg) != 0) {
            continue;
        }
        if (!beltCalReady(out, out_len)) {
            return false;
        }
        const float x = s_gantry->getCurrentJointConfig().x;
        if (!app.setParamFloat(e.key, x, reboot)) {
            snprintf(out, out_len, "%s %.1f mm rejected", e.key, static_cast<double>(x));
            return false;
        }
        snprintf(out, out_len, "%s at joint X %.1f mm (saved %s)", e.what,
                 static_cast<double>(x), e.key);
        return true;
    }
    if (std::strcmp(args, "z") == 0) {
        if (!beltCalReady(out, out_len)) {
            return false;
        }
        s_belt_z_mm = s_gantry->getCurrentJointConfig().z;
        s_belt_z_set = true;
        snprintf(out, out_len,
                 "belt surface at joint Z %.2f mm (col_z_min_mm %.2f; beltcal apply saves it)",
                 static_cast<double>(s_belt_z_mm),
                 static_cast<double>(c.gantry_conveyor_collision_z_min_mm));
        return true;
    }

    const float width = std::fabs(c.conveyor_belt_far_x_mm - c.conveyor_belt_near_x_mm);
    const float centre = 0.5f * (c.conveyor_belt_near_x_mm + c.conveyor_belt_far_x_mm);
    // Vision and picksim x are end-effector X (tcp_x = vis_x_off_mm + x_across);
    // the recorded X are joint X. Picks run at theta 0 here, as when jogging.
    const auto tcpX = [](float joint_x) {
        return s_gantry != nullptr
                   ? s_gantry->forwardKinematics(Gantry::JointConfig(joint_x, 0.0f, 0.0f)).x
                   : joint_x;
    };
    const float centre_tcp = tcpX(centre);
    const float batt_tcp = tcpX(c.test_battery_x_mm);
    if (args[0] == '\0' || std::strcmp(args, "show") == 0) {
        int len =
            snprintf(out, out_len,
                     "belt joint X %.1f..%.1f: width %.1f centre %.1f (cv_belt_w_mm %.1f, "
                     "vis_x_off_mm %.1f) | conveyor joint X %.1f..%.1f (col_x_min_mm %.1f)",
                     static_cast<double>(c.conveyor_belt_near_x_mm),
                     static_cast<double>(c.conveyor_belt_far_x_mm), static_cast<double>(width),
                     static_cast<double>(centre), static_cast<double>(c.conveyor_belt_width_mm),
                     static_cast<double>(c.conveyor_x_across_to_gantry_x_offset_mm),
                     static_cast<double>(c.conveyor_frame_near_x_mm),
                     static_cast<double>(c.conveyor_frame_far_x_mm),
                     static_cast<double>(c.gantry_conveyor_collision_x_min_mm));
        // picksim x is across the belt from the camera: batt_x - vis_x_off_mm.
        if (len > 0 && static_cast<size_t>(len) < out_len) {
            len += snprintf(out + len, out_len - len, " | batt_x %.1f (picksim x %.1f)",
                            static_cast<double>(c.test_battery_x_mm),
                            static_cast<double>(batt_tcp -
                                                c.conveyor_x_across_to_gantry_x_offset_mm));
        }
        if (len > 0 && static_cast<size_t>(len) < out_len) {
            if (s_belt_z_set) {
                snprintf(out + len, out_len - len, " | belt Z %.2f (col_z_min_mm %.2f)",
                         static_cast<double>(s_belt_z_mm),
                         static_cast<double>(c.gantry_conveyor_collision_z_min_mm));
            } else {
                snprintf(out + len, out_len - len, " | belt Z unset (col_z_min_mm %.2f)",
                         static_cast<double>(c.gantry_conveyor_collision_z_min_mm));
            }
        }
        return true;
    }

    const bool with_centre = std::strcmp(args, "apply center") == 0;
    if (with_centre || std::strcmp(args, "apply") == 0) {
        const float old_w = c.conveyor_belt_width_mm;
        const float old_x = c.conveyor_x_across_to_gantry_x_offset_mm;
        const float old_cx = c.gantry_conveyor_collision_x_min_mm;
        const float old_z = c.gantry_conveyor_collision_z_min_mm;
        const float cnv_near = c.conveyor_frame_near_x_mm;
        if (!app.setParamFloat("cv_belt_w_mm", width, reboot)) {
            snprintf(out, out_len, "belt width %.1f mm rejected", static_cast<double>(width));
            return false;
        }
        // The camera sits at the belt centre (Kconfig CONVEYOR_X_OFFSET), so
        // vision x_across = 0 maps to the midpoint. Opt-in: it moves every pick.
        if (with_centre && !app.setParamFloat("vis_x_off_mm", centre_tcp, reboot)) {
            snprintf(out, out_len, "width saved; camera X %.1f rejected",
                     static_cast<double>(centre_tcp));
            return false;
        }
        // The collision zone starts at the conveyor's near frame edge (joint X).
        if (!app.setParamFloat("col_x_min_mm", cnv_near, reboot)) {
            snprintf(out, out_len, "conveyor near edge %.1f mm rejected",
                     static_cast<double>(cnv_near));
            return false;
        }
        if (s_belt_z_set && !app.setParamFloat("col_z_min_mm", s_belt_z_mm, reboot)) {
            snprintf(out, out_len, "belt Z %.2f mm rejected", static_cast<double>(s_belt_z_mm));
            return false;
        }
        int len = snprintf(out, out_len, "cv_belt_w_mm %.1f -> %.1f, col_x_min_mm %.1f -> %.1f",
                           static_cast<double>(old_w), static_cast<double>(width),
                           static_cast<double>(old_cx), static_cast<double>(cnv_near));
        if (with_centre && len > 0 && static_cast<size_t>(len) < out_len) {
            len += snprintf(out + len, out_len - len, ", vis_x_off_mm %.1f -> %.1f",
                            static_cast<double>(old_x), static_cast<double>(centre_tcp));
        }
        if (s_belt_z_set && len > 0 && static_cast<size_t>(len) < out_len) {
            snprintf(out + len, out_len - len, ", col_z_min_mm %.2f -> %.2f",
                     static_cast<double>(old_z), static_cast<double>(s_belt_z_mm));
        }
        return true;
    }
    snprintf(out, out_len,
             "usage: beltcal [near|far|cnvyr near|cnvyr far|batt|z|show|apply|apply center]");
    return false;
}

// netstat (N-2): CellNet receive counters, plus the EMAC's own drop counters:
// frames the DMA dropped for want of a free receive buffer, and receive FIFO
// overflows. The EMAC register clears on read, so the counts are summed here.
bool pickSchedulerNetStats(bool reset, char* out, size_t out_len) {
    static uint32_t s_emac_missed = 0;
    static uint32_t s_emac_overflow = 0;
    static bool s_emac_rollover = false;
    const uint32_t reg = EMAC_DMA.dmamissedfr.val;  // one read: it clears
    s_emac_missed += reg & 0xFFFFu;
    s_emac_overflow += (reg >> 17) & 0x7FFu;
    s_emac_rollover = s_emac_rollover || ((reg >> 16) & 1u) != 0 || ((reg >> 28) & 1u) != 0;
    if (s_net_l2 == nullptr) {
        snprintf(out, out_len, "CellNet is not running");
        return false;
    }
    if (reset) {
        s_net_l2->resetStats();
        s_emac_missed = 0;
        s_emac_overflow = 0;
        s_emac_rollover = false;
        snprintf(out, out_len, "netstat reset");
        return true;
    }
    const CellNet::CellNetStats st = s_net_l2->getStats();
    snprintf(out, out_len,
             "netstat t_ms=%lld tx=%lu rx=%lu rx_invalid=%lu seq_drops=%lu "
             "emac_missed=%lu emac_overflow=%lu%s",
             static_cast<long long>(esp_timer_get_time() / 1000),
             static_cast<unsigned long>(st.tx_frames), static_cast<unsigned long>(st.rx_frames),
             static_cast<unsigned long>(st.rx_invalid_frames),
             static_cast<unsigned long>(st.rx_sequence_drops),
             static_cast<unsigned long>(s_emac_missed),
             static_cast<unsigned long>(s_emac_overflow),
             s_emac_rollover ? " (an EMAC counter rolled over)" : "");
    return true;
}

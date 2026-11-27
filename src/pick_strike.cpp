/**
 * @file pick_strike.cpp
 * @brief Pick scheduler: timer-fired strike (arming, re-timing, strike task) and GPIO markers.
 *
 * One of the pick_*.cpp files that make up the pick scheduler;
 * pick_scheduler_internal.h declares what they share.
 */

#include "pick_scheduler_internal.h"

namespace PickSchedulerImpl {

constexpr uint8_t kStrikeZ = 1;

constexpr uint8_t kStrikeGrip = 2;

constexpr uint8_t kStrikeRetract = 8;

// Timer bits earlier than this before their (re-timed) target are stale.
constexpr int64_t kEarlyUs = 1000;


TaskHandle_t s_strike_task = nullptr;
esp_timer_handle_t s_z_timer = nullptr;
esp_timer_handle_t s_grip_timer = nullptr;
esp_timer_handle_t s_retract_timer = nullptr;
std::atomic<bool> s_strike_armed{false};
static std::atomic<uint8_t> s_strike_bits{0};
std::atomic<int64_t> s_z_fired_us{0};
std::atomic<int64_t> s_grip_fired_us{0};
std::atomic<int64_t> s_retract_fired_us{0};
std::atomic<float> s_retract_z_mm{0.0f};
std::atomic<float> s_peak_z_mm{0.0f};
std::atomic<const char*> s_strike_abort{nullptr};
StrikeCtl s_ctl;
portMUX_TYPE s_strike_mux = portMUX_INITIALIZER_UNLOCKED;

std::atomic<int64_t> s_grip_cmd_us{0};
std::atomic<int64_t> s_retract_done_us{0};

// Grip timing marker (gripmark): borrows the TFT backlight pin (GPIO17, no
// free output GPIO on the board). High when the strike task fires the grip,
// low once the MCP write returns. Logic analyser: GPIO17 vs MCP PA0 (DOUT0)
// vs the valve coil gives SPI and relay delay. Off restores the backlight PWM.
std::atomic<bool> s_grip_mark{false};
// Vision receipt marker (vismark), same pin: high when the CellNetL2 callback
// stamps receipt_us, low when acceptDetect() takes the item. A Saleae on the
// trigger sensor and GPIO17 gives sensor-to-receipt latency (vis_lat_ms).
std::atomic<bool> s_vis_mark{false};

static void gripMark(uint32_t level) {
    if (s_grip_mark.load(std::memory_order_relaxed)) {
        gpio_set_level(static_cast<gpio_num_t>(TFT_BLK_GPIO), level);
    }
}

void visMark(uint32_t level) {
    if (s_vis_mark.load(std::memory_order_relaxed)) {
        gpio_set_level(static_cast<gpio_num_t>(TFT_BLK_GPIO), level);
    }
}

PickIntercept::BeltClock copyBelt() {
  portENTER_CRITICAL(&s_belt_mux);
  const PickIntercept::BeltClock belt = s_belt;
  portEXIT_CRITICAL(&s_belt_mux);
  return belt;
}

StrikeCtl copyCtl() {
  portENTER_CRITICAL(&s_strike_mux);
  const StrikeCtl ctl = s_ctl;
  portEXIT_CRITICAL(&s_strike_mux);
  return ctl;
}

void strikeNotify(uint8_t bit) {
  s_strike_bits.fetch_or(bit, std::memory_order_acq_rel);
  if (s_strike_task != nullptr) {
    xTaskNotifyGive(s_strike_task);
  }
}

void zTimerCb(void*) { strikeNotify(kStrikeZ); }
void gripTimerCb(void*) { strikeNotify(kStrikeGrip); }
void retractTimerCb(void*) { strikeNotify(kStrikeRetract); }

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

void disarmStrike() {
    s_strike_armed.store(false, std::memory_order_release);
    stopTimers();
    portENTER_CRITICAL(&s_strike_mux);
    s_ctl.z_pending = false;
    s_ctl.grip_pending = false;
    s_ctl.retract_pending = false;
    portEXIT_CRITICAL(&s_strike_mux);
    s_strike_bits.store(0, std::memory_order_release);
}

void strikeAbort(const char* why) {
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

int64_t secondsToUs(double s) {
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
void armStrike(const PickIntercept::StrikeTiming& timing, float jaw_world_deg,
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
    s_ctl.min_z_to_retract_us = secondsToUs(sim.z_to_retract_floor_s);
    s_ctl.z_fired_us = 0;
    s_ctl.grip_fired_us = 0;
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

static PickIntercept::ArmedStrikeTimes toArmedTimes(const StrikeCtl& ctl) {
    PickIntercept::ArmedStrikeTimes st;
    st.z_pending = ctl.z_pending;
    st.grip_pending = ctl.grip_pending;
    st.retract_pending = ctl.retract_pending;
    st.z_at_us = ctl.z_at_us;
    st.grip_at_us = ctl.grip_at_us;
    st.retract_at_us = ctl.retract_at_us;
    st.z_rel_us = ctl.z_rel_us;
    st.close_rel_us = ctl.close_rel_us;
    st.retract_rel_us = ctl.retract_rel_us;
    st.min_z_to_retract_us = ctl.min_z_to_retract_us;
    st.z_fired_us = ctl.z_fired_us;
    st.grip_fired_us = ctl.grip_fired_us;
    return st;
}

// Moves arrival with the belt while preserving ballistic plunge/grip invariants
// once Z or the pneumatic valve has fired (retimeStrikeEvents).
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
    PickIntercept::ArmedStrikeTimes st = toArmedTimes(s_ctl);
    PickIntercept::retimeStrikeEvents(st, r.arrive_us);
    s_ctl.z_at_us = st.z_at_us;
    s_ctl.grip_at_us = st.grip_at_us;
    s_ctl.retract_at_us = st.retract_at_us;
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

void strikeTask(void*) {
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
                const StrikeCtl snap = copyCtl();
                if (PickIntercept::isPlungeOverdue(now, snap.z_at_us, kMaxLatePlungeUs)) {
                    strikeAbort("late before plunge");
                    continue;
                }
                if (s_gantry->releasePreloadedMotion()) {
                    s_z_fired_us.store(now, std::memory_order_release);
                    portENTER_CRITICAL(&s_strike_mux);
                    s_ctl.z_fired_us = now;
                    const StrikeCtl ctl = s_ctl;
                    portEXIT_CRITICAL(&s_strike_mux);
                    // Queue the retract behind the plunge now; its edge fires at
                    // t_retract and the drive starts it as the plunge completes.
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
                const int64_t cmd_us = esp_timer_get_time();
                s_grip_cmd_us.store(cmd_us, std::memory_order_release);
                gripMark(1);
                const bool gripped = s_gantry->grip(true);
                gripMark(0);
                if (gripped) {
                    s_cell_in_jaws.store(true, std::memory_order_release);
                    s_grip_fired_us.store(esp_timer_get_time(), std::memory_order_release);
                    // Lock pending retract to the actual valve fire timestamp so
                    // pneumatic close duration stays constant despite belt jitter.
                    portENTER_CRITICAL(&s_strike_mux);
                    s_ctl.grip_fired_us = cmd_us;
                    PickIntercept::ArmedStrikeTimes st = toArmedTimes(s_ctl);
                    PickIntercept::retimeStrikeEvents(st, s_ctl.arrive_us);
                    s_ctl.retract_at_us = st.retract_at_us;
                    const StrikeCtl ctl = s_ctl;
                    portEXIT_CRITICAL(&s_strike_mux);
                    if (ctl.retract_pending) {
                        armOne(s_retract_timer, ctl.retract_at_us, kStrikeRetract);
                    }
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

}  // namespace PickSchedulerImpl

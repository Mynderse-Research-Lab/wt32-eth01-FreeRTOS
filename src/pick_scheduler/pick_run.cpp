/**
 * @file pick_run.cpp
 * @brief Pick scheduler: one pick, from approach to drop, and the strike wait.
 *
 * One of the pick_*.cpp files that make up the pick scheduler;
 * pick_scheduler_internal.h declares what they share.
 */

#include "pick_scheduler_internal.h"

namespace PickSchedulerImpl {

constexpr int64_t kPreloadDeadlineUs = 20000;

constexpr int64_t kGripDeadlineUs = 30000;

constexpr int64_t kRetractDeadlineUs = 30000;

// Skip a redundant moveZ(hover) when Z is already within this band of hover.
constexpr float kHoverAlreadyEpsMm = 0.25f;

// Retract to the SAFE_Z ceiling must finish within this after it fires.
constexpr int64_t kRetractDoneUs = 2000000;

// Deepest Z reached this far short of the grasp depth is logged.
constexpr float kDepthShortWarnMm = 0.3f;

// Offsets are whole ms, so any bottom dwell they ask for is at least 1 ms.
constexpr double kDwellWarnS = 0.0005;


// Waits for plunge, close and retract to fire and for the retract to finish
// (or hand off to a chained bin drop once Z enters SAFE_Z). False on halt,
// abort, or an event that did not fire in time.
static bool waitStrikeDone(PickSchedulerTaskConfig* cfg, const PickConfig* p = nullptr,
                           const Gantry::JointConfig* chain_drop = nullptr,
                           float approach_mm_s = 0.0f, float approach_deg_s = 0.0f,
                           bool* out_chained = nullptr) {
    bool chained = false;
    if (out_chained != nullptr) {
        *out_chained = false;
    }
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
            if (chain_drop != nullptr && p != nullptr && !chained &&
                !cfg->gantry->isPreloadHeld()) {
                chained = cfg->gantry->chainMoveAfterRetract(
                    *chain_drop, u32(approach_mm_s), u32(approach_deg_s), u32(p->limits.accel),
                    u32(p->limits.decel), u32(p->limits.theta_accel), u32(p->limits.theta_decel),
                    p->limits.theta_start_frac, p->limits.theta_full_speed);
                if (out_chained != nullptr) {
                    *out_chained = chained;
                }
            }
            if (chained ? cfg->gantry->isRetractHandedOff() : !cfg->gantry->isBusy()) {
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
void runPick(PickSchedulerTaskConfig* cfg, const PickIntercept::TrackedPart& part) {
    const int64_t started = esp_timer_get_time();
    const Gantry::JointLimits lim = cfg->gantry->getJointLimits();
    const PickConfig p = pickConfig(lim);

    PickEval eval = evaluateActive(p, lim, part, nullptr);
    if (!eval.strike.feasible) {
        ESP_LOGW(TAG, "pick item=%lu skipped (%s)", static_cast<unsigned long>(part.item_id),
                 eval.reason);
        return;
    }
    // Start approach immediately (including deferred-X retract if starting below SAFE_Z)
    // so UART logging and follower classification run in parallel with axis acceleration.
    const bool approach_started =
        startJointMove(cfg, eval.approach, eval.strike.approach_speed_mm_s, p.limits.accel,
                       p.limits.decel, eval.approach_deg_s, p.limits.theta_accel,
                       p.limits.theta_start_frac, p.limits.theta_full_speed);
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

    if (!approach_started || !finishJointMove(cfg, eval.approach)) {
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

    // The hover can sit above the band ceiling for a tall object; skip a no-op
    // Z move when already parked at hover (saves ~120 ms on SAFE_Z picks).
    const Gantry::JointConfig pre_hover = cfg->gantry->getCurrentJointConfig();
    const float jaw_joint = pre_hover.theta;
    if (std::fabs(pre_hover.z - eval.hover.z) > kHoverAlreadyEpsMm) {
        if (!moveZ(cfg, eval.hover, p, eval.strike.descend_speed_mm_s)) {
            (void)retract(cfg, p);
            return;
        }
    }
    eval = evaluateActive(p, lim, part, &jaw_joint);
    if (!eval.strike.feasible) {
        abortStrike(cfg, p, eval.reason);
        return;
    }

    const Gantry::JointConfig here = cfg->gantry->getCurrentJointConfig();
    const Gantry::JointConfig strike(here.x, eval.pick.z, here.theta);
    const float za = zAccel(p);
    const float zd = zDecel(p);
    const auto pre = cfg->gantry->preloadJointMove(
        strike, u32(eval.strike.descend_speed_mm_s), u32(za), u32(zd));
    if (pre != Gantry::GantryError::OK || !cfg->gantry->isPreloadHeld()) {
        abortStrike(cfg, p, "Z preload did not hold StartMotion");
        return;
    }
    const int64_t now_pre = esp_timer_get_time();
    const PickIntercept::Retime pre_retime =
        PickIntercept::retimeArrival(eval.timing, copyBelt(), now_pre);
    if (!pre_retime.ok) {
        abortStrike(cfg, p, pre_retime.reason);
        return;
    }
    const int64_t z_target_us = pre_retime.arrive_us + secondsToUs(eval.sim.t_z_s);
    if (PickIntercept::isPlungeOverdue(now_pre, z_target_us, kMaxLatePlungeUs)) {
        abortStrike(cfg, p, "late after preload");
        return;
    }
    armStrike(eval.timing, eval.jaw_world_deg, part.battery_class, eval.sim, pre_retime.arrive_us,
              p.limits.z_cap, za, zd);
    traceStart(part.item_id);

    const auto& c = Config::AppConfig::instance().data();
    const float bin_theta = c.pick_drop_any_theta ? here.theta : c.gantry_cal_theta_park_deg;
    const Gantry::JointConfig drop(c.gantry_cal_x_park_mm, c.gantry_cal_z_park_mm, bin_theta);
    const Gantry::JointConfig* chain_drop =
        PickIntercept::inConveyorCollision(drop.x, drop.z, c.gantry_conveyor_collision_x_min_mm,
                                           c.gantry_conveyor_collision_z_min_mm)
            ? nullptr
            : &drop;
    bool chained = false;
    const bool struck = waitStrikeDone(cfg, &p, chain_drop, eval.strike.approach_speed_mm_s,
                                       eval.approach_deg_s, &chained);
    traceFinish(eval, eval.strike.descend_speed_mm_s, za, copyBelt().speed_mm_s);
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

    bool drop_started = chained;
    if (!drop_started && chain_drop != nullptr) {
        drop_started = startJointMove(cfg, drop, eval.strike.approach_speed_mm_s, p.limits.accel,
                                      p.limits.decel, eval.approach_deg_s, p.limits.theta_accel,
                                      p.limits.theta_start_frac, p.limits.theta_full_speed);
    }

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

    if (!dropHeld(cfg, p, eval.strike.approach_speed_mm_s, eval.approach_deg_s,
                  drop_started ? &drop : nullptr)) {
        return;
    }
    s_last_cycle_ms = static_cast<float>(esp_timer_get_time() - started) / 1000.0f;
    s_status_us = 0;
    publishGantryStatus(cfg);
}

}  // namespace PickSchedulerImpl

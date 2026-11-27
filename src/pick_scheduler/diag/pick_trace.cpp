/**
 * @file pick_trace.cpp
 * @brief Pick scheduler: strike trace capture and the ztrace report.
 *
 * One of the pick_*.cpp files that make up the pick scheduler;
 * pick_scheduler_internal.h declares what they share.
 */

#include "pick_scheduler_internal.h"

namespace PickSchedulerImpl {

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
    float z_plunge_dead_s = 0.0f;
    int32_t t_us[kTraceCap];
    float z_mm[kTraceCap];
};
static ZTrace s_trace;
static std::atomic<bool> s_trace_run{false};
static std::atomic<int64_t> s_trace_stop_at_us{0};
TaskHandle_t s_trace_task = nullptr;

void traceTask(void*) {
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
void traceStart(uint32_t item_id) {
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
void traceFinish(const PickEval& eval, float vz_mm_s, float az_mm_s2, float belt_mm_s) {
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
    s_trace.z_plunge_dead_s = static_cast<float>(eval.sim.z_plunge_dead_s);
    s_trace.complete = true;
    const int64_t from = s_trace.retract_cmd_us != 0 ? s_trace.retract_cmd_us
                                                     : esp_timer_get_time();
    s_trace_stop_at_us.store(from + kTraceTailUs, std::memory_order_release);
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
             "z_fire_err=%.1f plunge_start=%.1f reach=%.1f (plan %.1f+%.0fd) peak=%.2f@%.1f "
             "retract_cmd=%.1f z_at_cmd=%.2f retract_start=%.1f call=%.1f "
             "grip_cmd=%.1f grip_write=%.1f [ms from z fire]",
             static_cast<unsigned long>(t.item_id), t.n, static_cast<double>(t.belt_mm_s),
             static_cast<double>(t.vz_mm_s), static_cast<double>(t.az_mm_s2),
             static_cast<double>(t.z_hover_mm), static_cast<double>(t.z_grasp_mm),
             traceMs(t.z_fired_us, t.ctl.z_at_us), relMs(i_move, z_rel), relMs(i_reach, z_rel),
             static_cast<double>(t.tau_plunge_s) * 1000.0,
             static_cast<double>(t.z_plunge_dead_s) * 1000.0, static_cast<double>(t.z_mm[i_peak]),
             relMs(i_peak, z_rel), traceMs(t.retract_cmd_us, t.z_fired_us),
             static_cast<double>(z_at_r), relMs(i_rev, z_rel),
             traceMs(t.retract_done_us, t.retract_cmd_us), traceMs(t.grip_cmd_us, t.z_fired_us),
             traceMs(t.grip_fired_us, t.grip_cmd_us));
}

}  // namespace PickSchedulerImpl

using namespace PickSchedulerImpl;

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

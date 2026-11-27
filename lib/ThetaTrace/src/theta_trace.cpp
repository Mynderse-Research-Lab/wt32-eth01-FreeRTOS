/**
 * @file theta_trace.cpp
 * @brief 1 kHz trace of one theta move for HCS01 loop tuning (console `thtrace`).
 *
 * The HCS01 profiles the move itself, so the trace holds the drive's actual
 * angle, velocity and status word per sample plus the move as commanded. The
 * ideal trapezoid is rebuilt afterwards (theta_trace_analysis.h). Class 1
 * feedback refreshes every CONFIG_EIP_THETA_RPI_US (2 ms in sdkconfig.defaults),
 * so consecutive 1 kHz samples can repeat a value.
 */

#include "theta_trace.h"

#include "gantry_app_constants.h"
#include "theta_trace_analysis.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <atomic>
#include <cstdio>
#include <cstring>
#include <new>

namespace {

const char *TAG = "ThetaTrace";

constexpr int kCap = 1500;                      // 1.5 s at 1 kHz
constexpr int64_t kTailUs = 300000;             // keep recording after the axis ends the move
constexpr int64_t kArmTimeoutUs = 120000000;    // disarm when no move comes
// Raw dumps stay small and paced: the LAN side has gone silent during a large
// trace dump (OPEN_ITEMS G-E1).
constexpr int kRawPerLine = 10;
constexpr int kRawMaxPerCall = 300;
constexpr uint32_t kRawLinePaceMs = 20;

enum class State : uint8_t { kIdle, kArmed, kRecording, kDone };

struct Trace {
  Gantry::RotaryMoveStamp stamp;
  int n = 0;
  bool full = false;
  bool next_move = false;  // stopped because the next theta move started
  int32_t t_us[kCap];      // from stamp.cmd_us
  float rel_deg[kCap];     // actual - stamp.start_deg
  float vel_deg_s[kCap];
  uint16_t status[kCap];
};

// Allocated on the first `thtrace arm`, so the buffer costs nothing unless used.
Trace *s_tr = nullptr;
std::atomic<uint8_t> s_state{static_cast<uint8_t>(State::kIdle)};
std::atomic<bool> s_stop{false};
Gantry::Gantry *s_gantry = nullptr;
TaskHandle_t s_task = nullptr;
uint32_t s_arm_seq = 0;
int64_t s_arm_us = 0;

State state() { return static_cast<State>(s_state.load(std::memory_order_acquire)); }

void setState(State st) { s_state.store(static_cast<uint8_t>(st), std::memory_order_release); }

const char *endName(Gantry::RotaryMoveEnd end) {
  switch (end) {
    case Gantry::RotaryMoveEnd::kExact: return "exact";
    case Gantry::RotaryMoveEnd::kStiction: return "stiction";
    case Gantry::RotaryMoveEnd::kTimeout: return "timeout";
    case Gantry::RotaryMoveEnd::kAbort: return "abort";
    default: return "none";
  }
}

void record(const Gantry::RotaryMoveStamp &first) {
  Gantry::RotaryMoveStamp st = first;
  s_tr->n = 0;
  s_tr->full = false;
  s_tr->next_move = false;
  int64_t stop_at = 0;
  TickType_t wake = xTaskGetTickCount();
  while (!s_stop.load(std::memory_order_acquire)) {
    const int64_t now = esp_timer_get_time();
    Gantry::RotaryTraceSample smp;
    if (s_gantry->sampleThetaTrace(smp)) {
      const int i = s_tr->n;
      s_tr->t_us[i] = static_cast<int32_t>(now - st.cmd_us);
      s_tr->rel_deg[i] = static_cast<float>(smp.deg - static_cast<double>(st.start_deg));
      s_tr->vel_deg_s[i] = smp.vel_deg_s;
      s_tr->status[i] = smp.status;
      if (++s_tr->n >= kCap) {
        s_tr->full = true;
        break;
      }
    }
    Gantry::RotaryMoveStamp cur;
    if (s_gantry->getThetaMoveStamp(cur)) {
      if (cur.seq != st.seq) {
        s_tr->next_move = true;
        break;
      }
      st = cur;
      if (st.done_us != 0 && stop_at == 0) {
        stop_at = st.done_us + kTailUs;
      }
    }
    if (stop_at != 0 && now >= stop_at) {
      break;
    }
    vTaskDelayUntil(&wake, 1);
  }
  s_tr->stamp = st;
}

void traceTask(void *) {
  while (true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    // Waiting polls the move stamp only (atomics), at 10 ms. A move preloads
    // its target for several Gantry ticks before the start edge, so the
    // recording still begins ahead of the drive's profile.
    Gantry::RotaryMoveStamp st;
    bool got = false;
    while (!s_stop.load(std::memory_order_acquire) &&
           esp_timer_get_time() - s_arm_us < kArmTimeoutUs) {
      if (s_gantry->getThetaMoveStamp(st) && st.seq != s_arm_seq) {
        got = true;
        break;
      }
      vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (!got) {
      setState(s_tr->n > 0 ? State::kDone : State::kIdle);
      continue;
    }
    setState(State::kRecording);
    record(st);
    setState(State::kDone);
  }
}

ThetaTrace::Result analyzeTrace() {
  const Gantry::RotaryMoveStamp &st = s_tr->stamp;
  ThetaTrace::Move m;
  m.dist_deg = static_cast<double>(st.target_deg) - static_cast<double>(st.start_deg);
  m.vel_deg_s = st.vel_deg_s;
  m.acc_deg_s2 = st.acc_deg_s2;
  m.dec_deg_s2 = st.dec_deg_s2;
  m.start_edge_us = st.start_us != 0 ? static_cast<double>(st.start_us - st.cmd_us) : -1.0;
  return ThetaTrace::analyze(s_tr->t_us, s_tr->rel_deg, s_tr->vel_deg_s, s_tr->n, m);
}

double atMs(int i) {
  return i < 0 || i >= s_tr->n ? -1.0 : static_cast<double>(s_tr->t_us[i]) / 1000.0;
}

void summary(char *out, size_t out_len) {
  const Gantry::RotaryMoveStamp &st = s_tr->stamp;
  const ThetaTrace::Result r = analyzeTrace();
  const double start_edge_ms =
      st.start_us != 0 ? static_cast<double>(st.start_us - st.cmd_us) / 1000.0 : -1.0;
  const double done_ms =
      st.done_us != 0 ? static_cast<double>(st.done_us - st.cmd_us) / 1000.0 : -1.0;
  const double t0_ms = r.t0_us / 1000.0;
  const double settle_ms = atMs(r.i_settle);
  const double after_profile_ms =
      r.i_settle >= 0 && r.i_motion >= 0 ? settle_ms - (t0_ms + r.profile_ms) : -1.0;
  snprintf(out, out_len,
           "thtrace move=%lu n=%d%s start=%.3f target=%.3f v=%.0f a=%.0f d=%.0f | "
           "start_edge=%.1f onset=%.1f t0=%.1f latency=%.1f profile=%.1f band=%.1f settle=%.1f "
           "(+%.1f after profile) overshoot=%.3f@%.1f follow_max=%.3f@%.1f "
           "vpk=%.0f@%.1f final_err=%.4f end=%s@%.1f [ms from cmd, deg]",
           static_cast<unsigned long>(st.seq), s_tr->n,
           s_tr->full ? " FULL" : (s_tr->next_move ? " NEXT-MOVE" : ""),
           static_cast<double>(st.start_deg), static_cast<double>(st.target_deg),
           static_cast<double>(st.vel_deg_s), static_cast<double>(st.acc_deg_s2),
           static_cast<double>(st.dec_deg_s2), start_edge_ms, atMs(r.i_motion), t0_ms,
           r.latency_ms,
           r.profile_ms, atMs(r.i_band), settle_ms, after_profile_ms, r.overshoot_deg,
           atMs(r.i_overshoot), r.max_follow_deg, atMs(r.i_follow), r.peak_vel_deg_s,
           atMs(r.i_peak_vel), r.final_err_deg, endName(st.end), done_ms);
}

void dumpRaw(int start, int count) {
  const Gantry::RotaryMoveStamp &st = s_tr->stamp;
  ESP_LOGI(TAG,
           "thtrace meta move=%lu n=%d start=%.4f target=%.4f v=%.1f a=%.1f d=%.1f "
           "start_edge_us=%lld done_us=%lld end=%s",
           static_cast<unsigned long>(st.seq), s_tr->n, static_cast<double>(st.start_deg),
           static_cast<double>(st.target_deg), static_cast<double>(st.vel_deg_s),
           static_cast<double>(st.acc_deg_s2), static_cast<double>(st.dec_deg_s2),
           static_cast<long long>(st.start_us != 0 ? st.start_us - st.cmd_us : -1),
           static_cast<long long>(st.done_us != 0 ? st.done_us - st.cmd_us : -1),
           endName(st.end));
  const int end = start + count < s_tr->n ? start + count : s_tr->n;
  for (int i = start; i < end; i += kRawPerLine) {
    char line[400];
    int len = snprintf(line, sizeof(line), "tt %d", i);
    for (int j = i; j < end && j < i + kRawPerLine && len > 0 &&
                    len < static_cast<int>(sizeof(line)) - 40;
         ++j) {
      len += snprintf(line + len, sizeof(line) - len, " %ld:%.4f:%.1f:%04X",
                      static_cast<long>(s_tr->t_us[j]), static_cast<double>(s_tr->rel_deg[j]),
                      static_cast<double>(s_tr->vel_deg_s[j]),
                      static_cast<unsigned>(s_tr->status[j]));
    }
    ESP_LOGI(TAG, "%s", line);
    vTaskDelay(pdMS_TO_TICKS(kRawLinePaceMs));
  }
}

}  // namespace

bool thetaTraceCommand(Gantry::Gantry *gantry, const char *args, char *out, size_t out_len) {
  if (out == nullptr || out_len == 0) {
    return false;
  }
  if (gantry == nullptr || !gantry->hasThetaAxis()) {
    snprintf(out, out_len, "no theta axis");
    return false;
  }
  if (args == nullptr) {
    args = "";
  }
  const State st = state();

  if (strcmp(args, "arm") == 0) {
    if (st == State::kArmed || st == State::kRecording) {
      snprintf(out, out_len, "thtrace already %s", st == State::kArmed ? "armed" : "recording");
      return false;
    }
    if (s_tr == nullptr) {
      s_tr = new (std::nothrow) Trace();
      if (s_tr == nullptr) {
        snprintf(out, out_len, "no memory for the theta trace (%u bytes)",
                 static_cast<unsigned>(sizeof(Trace)));
        return false;
      }
    }
    if (s_task == nullptr) {
      s_gantry = gantry;
      if (xTaskCreatePinnedToCore(traceTask, "ThetaTrace", THETA_TRACE_TASK_STACK, nullptr,
                                  THETA_TRACE_TASK_PRIORITY, &s_task,
                                  THETA_TRACE_TASK_CORE) != pdPASS) {
        s_task = nullptr;
        snprintf(out, out_len, "theta trace task unavailable");
        return false;
      }
    }
    Gantry::RotaryMoveStamp now_stamp;
    s_arm_seq = gantry->getThetaMoveStamp(now_stamp) ? now_stamp.seq : 0;
    s_arm_us = esp_timer_get_time();
    s_stop.store(false, std::memory_order_release);
    setState(State::kArmed);
    xTaskNotifyGive(s_task);
    snprintf(out, out_len, "thtrace armed: the next theta move is recorded (1 kHz, %d ms max)",
             kCap);
    return true;
  }

  if (strcmp(args, "off") == 0) {
    if (st == State::kArmed || st == State::kRecording) {
      s_stop.store(true, std::memory_order_release);
      snprintf(out, out_len, "thtrace stopping");
    } else {
      snprintf(out, out_len, "thtrace not armed");
    }
    return true;
  }

  if (st == State::kArmed) {
    snprintf(out, out_len, "thtrace armed, waiting for a theta move");
    return false;
  }
  if (st == State::kRecording) {
    snprintf(out, out_len, "thtrace recording (%d samples)", s_tr->n);
    return false;
  }
  if (st != State::kDone || s_tr == nullptr || s_tr->n < 2) {
    snprintf(out, out_len, "no theta trace yet (thtrace arm, then move theta)");
    return false;
  }

  int start = 0;
  int count = 0;
  if (strncmp(args, "raw", 3) == 0) {
    if (sscanf(args, "raw %d %d", &start, &count) < 1) {
      start = 0;
    }
    if (count <= 0 || count > kRawMaxPerCall) {
      count = kRawMaxPerCall;
    }
    if (start < 0) {
      start = 0;
    }
    dumpRaw(start, count);
    const int end = start + count < s_tr->n ? start + count : s_tr->n;
    snprintf(out, out_len, "thtrace raw %d..%d of %d", start, end, s_tr->n);
    return true;
  }
  if (args[0] != '\0') {
    snprintf(out, out_len, "usage: thtrace [arm|off|raw <start> <count>]");
    return false;
  }
  summary(out, out_len);
  return true;
}

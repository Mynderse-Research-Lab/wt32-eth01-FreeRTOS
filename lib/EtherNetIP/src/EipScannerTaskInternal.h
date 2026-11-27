/**
 * @file EipScannerTaskInternal.h
 * @brief Cyclic Class 1 helpers and the state the scanner files share.
 *
 * Private to this library. The helpers are inline so the exchange loop
 * still sees their bodies.
 */
#pragma once

#include "EipScannerTask.h"

#include "sdkconfig.h"

#if CONFIG_EIP_SCANNER_ENABLED

#include "EipClass1Timing.h"
#include "EipClass1TimingStats.h"
#include "EipReliabilityStats.h"
#include "EipMultiScanner.h"
#include "EipRunTimeTrace.h"
#include "EipProcessImage.h"
#include "EipScanner.h"
#include "EipScannerPolicy.h"
#include "EipSocketW5500.h"
#include "Hcs01Assembly.h"
#include "W5500.h"
#include "W5500BusMutex.h"
#include "W5500DeferredLog.h"
#include "W5500Diag.h"
#include "W5500Socket.h"
#include "Spi3Bus.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <atomic>
#include <cstdio>
#include <cstring>
#include <mutex>

#include "driver/gpio.h"
namespace eip {

ScannerConfig makeKinetixConfig(const char* ip, uint16_t connection_serial,
                                uint32_t ot_connection_id);
#if defined(CONFIG_EIP_AXIS_THETA)
ScannerConfig makeThetaConfig();
void singleAxisTask(void* arg);
#endif
void logW5500FailureSnapshot(w5500::W5500Hal& hal, w5500::FailReason fallback);
uint8_t logW5500LinkMode(w5500::W5500Hal& hal);
int64_t logW5500LinkDown(w5500::W5500Hal& hal);
bool w5500LostConfig(w5500::W5500Hal& hal);
void logClass1StallDiag(bool force);
void captureSnapshotForTrickle(w5500::W5500Hal& hal, size_t axis);
void setLogScanner(EipMultiScanner* scanner);
void class1LogTask(void*);

static const char* TAG = "EipScanner";
static constexpr UBaseType_t kScannerPriority = 6;
static constexpr UBaseType_t kKeepalivePriority = 7;
static constexpr BaseType_t kScannerCore = 1;
static constexpr uint32_t kScannerStack = 8192;
static constexpr uint32_t kKeepaliveStack = 4096;
static constexpr uint32_t kLinkPollMs = 500;
static constexpr uint32_t kLinkSettleMs = 300;
static constexpr uint32_t kReconnectIdleMs = 500;
static constexpr uint32_t kPostResetSettleMs = 100;
static constexpr uint32_t kTimingLogEveryN = 200;
inline
uint32_t g_class1_budget_rpi_us =
#if defined(CONFIG_EIP_AXIS_X) || defined(CONFIG_EIP_AXIS_Z)
    CONFIG_EIP_X_RPI_US;
#elif defined(CONFIG_EIP_AXIS_THETA)
    CONFIG_EIP_THETA_RPI_US;
#else
    5000;
#endif
struct ScannerTaskCtx {
  W5500* chip;
  w5500::W5500Hal* hal;
  ILinkStatus* link;
  EipProcessImage* image_x;
  EipProcessImage* image_z;
  EipProcessImage* image_theta;
};

struct KeepaliveCtx {
  EipMultiScanner* scanner;
  std::atomic<bool> run;
  std::atomic<bool> failed;
  uint32_t period_ms;
  // Also drain T->O so connected axes keep fresh feedback (per-axis reopen,
  // where the cyclic exchange is paused for the whole FO).
  bool drain = false;
  // Set by the helper just before it deletes itself.
  std::atomic<bool> exited{false};
};

// Keepalive pumping during a per-axis reopen. 2 ms matches the Kinetix RPI so
// X/Z neither starve the drive (16x RPI) nor go stale on our side (10x RPI).
static constexpr uint32_t kReopenKeepalivePeriodMs = 2;
static constexpr uint32_t kKeepaliveExitWaitMs = 50;
inline
void paceClass1RemainderUs(uint32_t rem_us) {
  if (rem_us == 0) return;
  // Fractional top-up only (< 1 FreeRTOS tick). DelayUntil already blocked.
#if defined(ESP_PLATFORM)
  esp_rom_delay_us(rem_us);
#else
  const int64_t deadline = esp_timer_get_time() + static_cast<int64_t>(rem_us);
  while (esp_timer_get_time() < deadline) {
  }
#endif
}

inline
void paceClass1Cycle(TickType_t& last_wake, uint32_t rpi_us) {
  constexpr uint32_t kTickUs = 1000u;
  static uint32_t overrun_streak = 0;
  const TickType_t period =
      static_cast<TickType_t>(class1PaceTicks(rpi_us, kTickUs));
  // pdFALSE => deadline already passed; last_wake still advanced by period.
  if (xTaskDelayUntil(&last_wake, period) == pdFALSE) {
    class1TimingStats().notePaceOverrun();
    if (class1OverrunAction(overrun_streak) == Class1OverrunAction::kYieldTick) {
      class1TimingStats().notePaceYield();
      vTaskDelay(1);
      last_wake = xTaskGetTickCount();
      return;
    }
    const TickType_t now = xTaskGetTickCount();
    if (class1ShouldReseedWake(static_cast<uint32_t>(now),
                               static_cast<uint32_t>(last_wake),
                               static_cast<uint32_t>(period))) {
      last_wake = now;
    }
    return;
  }
  overrun_streak = 0;
  const uint32_t frac = class1RpiFractionUs(rpi_us, kTickUs);
  if (frac != 0) {
    paceClass1RemainderUs(frac);
  }
}
constexpr uint32_t kSpiCheckEveryN = 50;  // 100 ms at RPI 2 ms
// A real 100BASE-TX link loss lasts >1 s (renegotiation). Require LNK=0 on
// this many consecutive cycles before tearing down: on 2026-10-20 16:24 one
// LNK=0 read (link back within 1 ms, no renegotiation) caused a full drop.
constexpr uint32_t kLinkDownConfirmReads = 3;
constexpr uint32_t kLoopGapReportUs = 6000;
constexpr uint32_t kBusHoldReportUs = 3000;

struct LoopGap {
    uint32_t gap_us = 0;
    uint32_t exchange_us = 0;
    uint32_t ot_us = 0;     // of exchange: O->T sends
    uint32_t drain_us = 0;  // of exchange: T->O drain
    uint32_t flush_us = 0;
    uint32_t rest_us = 0;  // periodic checks + pacing + anything preempting
    uint32_t pace_us = 0;  // of rest: inside paceClass1Cycle (delay + wake latency)
};
inline
portMUX_TYPE g_diag_mux = portMUX_INITIALIZER_UNLOCKED;
inline
LoopGap g_worst_gap;    // guarded by g_diag_mux
inline
int64_t g_pace_us = 0;  // previous iteration's time in paceClass1Cycle
// Run-time trace (G-B6): class1LogTask snapshots per-task CPU time each period,
// and a gap of kRunTimeGapUs or more is reported as CPU time per task across it.
constexpr uint32_t kRunTimeGapUs = 10000;
constexpr int64_t kRunTimeMinIntervalUs = 5000000;
constexpr size_t kRunTimeReportTasks = 8;
inline
std::atomic<bool> g_rt_pending{false};
inline
uint32_t g_rt_gap_us = 0;  // set before g_rt_pending, with the breakdown below
inline
uint32_t g_rt_end_us = 0;
inline
uint32_t g_rt_exchange_us = 0;
inline
uint32_t g_rt_rest_us = 0;
inline
uint32_t g_rt_pace_us = 0;
inline
int64_t g_rt_last_us = 0;  // scanner task only
inline
void noteRunTimeGap(int64_t t0, uint32_t gap_us, uint32_t exchange_us, uint32_t rest_us,
                    uint32_t pace_us) {
    if (gap_us < kRunTimeGapUs || g_rt_pending.load() ||
        t0 - g_rt_last_us < kRunTimeMinIntervalUs) {
        return;
    }
    g_rt_gap_us = gap_us;
    g_rt_end_us = static_cast<uint32_t>(t0);
    g_rt_exchange_us = exchange_us;
    g_rt_rest_us = rest_us;
    g_rt_pace_us = pace_us;
    g_rt_last_us = t0;
    g_rt_pending.store(true);
}
// paceClass1Cycle, timed for the loop-gap breakdown (multi-axis loop).
inline
void pacedClass1Cycle(TickType_t& last_wake, uint32_t rpi_us) {
    const int64_t tp = class1NowUs();
    paceClass1Cycle(last_wake, rpi_us);
    g_pace_us = class1NowUs() - tp;
}

inline
void noteLoopGap(int64_t t0, int64_t prev_t0, int64_t prev_t1, int64_t prev_t2,
                 uint32_t prev_ot_us, uint32_t prev_drain_us) {
    if (prev_t0 <= 0 || t0 <= prev_t0) return;
    const uint32_t gap = static_cast<uint32_t>(t0 - prev_t0);
    noteRunTimeGap(t0, gap, static_cast<uint32_t>(prev_t1 - prev_t0),
                   static_cast<uint32_t>(t0 - prev_t2), static_cast<uint32_t>(g_pace_us));
    portENTER_CRITICAL(&g_diag_mux);
    if (gap > g_worst_gap.gap_us) {
        g_worst_gap.gap_us = gap;
        g_worst_gap.exchange_us = static_cast<uint32_t>(prev_t1 - prev_t0);
        g_worst_gap.ot_us = prev_ot_us;
        g_worst_gap.drain_us = prev_drain_us;
        g_worst_gap.flush_us = static_cast<uint32_t>(prev_t2 - prev_t1);
        g_worst_gap.rest_us = static_cast<uint32_t>(t0 - prev_t2);
        g_worst_gap.pace_us = static_cast<uint32_t>(g_pace_us);
    }
    portEXIT_CRITICAL(&g_diag_mux);
}
// Logic-analyzer marker (cfg eip_scope_gpio): high during each exchange.
inline
int g_scope_gpio = -1;

inline void scopeMark(int level) {
    if (g_scope_gpio >= 0) {
        gpio_set_level(static_cast<gpio_num_t>(g_scope_gpio), static_cast<uint32_t>(level));
    }
}
constexpr UBaseType_t kLogTaskPriority = 2;
constexpr BaseType_t kLogTaskCore = 0;
constexpr uint32_t kLogTaskStack = 4096;
constexpr uint32_t kLogTaskPeriodMs = 20;
// Class 1 UDP destinations are written once and cached, so a write that lands
// in Sn_DIPR/Sn_DPORT would silently send every later frame elsewhere (SENDOK
// still fires). Compare them with the cache; log each socket once per change.
inline
void checkW5500SocketDests(w5500::W5500Hal& hal) {
    static uint8_t reported = 0;
    w5500::DestMismatch bad[8];
    const size_t n =
        w5500::checkSocketDests(hal, w5500::socketSoftTxState(), bad, 8);
    uint8_t now_bad = 0;
    for (size_t k = 0; k < n; ++k) {
        const w5500::DestMismatch& m = bad[k];
        const uint8_t bit = static_cast<uint8_t>(1u << m.sock);
        now_bad = static_cast<uint8_t>(now_bad | bit);
        if (reported & bit) continue;
        (void)w5500::deferLog(TAG, 'W', esp_log_timestamp(),
                 "W5500 s%u destination changed: chip %u.%u.%u.%u:%u, written "
                 "%lu.%lu.%lu.%lu:%u",
                 static_cast<unsigned>(m.sock), m.raw[0], m.raw[1], m.raw[2],
                 m.raw[3],
                 static_cast<unsigned>((static_cast<unsigned>(m.raw[4]) << 8) | m.raw[5]),
                 static_cast<unsigned long>((m.exp_ip >> 24) & 0xFF),
                 static_cast<unsigned long>((m.exp_ip >> 16) & 0xFF),
                 static_cast<unsigned long>((m.exp_ip >> 8) & 0xFF),
                 static_cast<unsigned long>(m.exp_ip & 0xFF),
                 static_cast<unsigned>(m.exp_port));
    }
    reported = now_bad;
}

// Read back VERSIONR/SHAR/SIPR every 100 ms; a mismatch is a corrupted read.
// The same read reports PHYCFGR, so any link-mode change is logged too.
inline
void checkW5500SpiPeriodic(w5500::W5500Hal& hal, uint32_t& cycles,
                           uint8_t& last_phy) {
    if (++cycles < kSpiCheckEveryN) return;
    cycles = 0;
    const w5500::SpiCheckResult res = w5500::spiIntegrityCheck(hal);
    if (res.phycfgr != last_phy) {
        char was[24];
        char now[24];
        w5500::formatPhy(last_phy, was, sizeof(was));
        w5500::formatPhy(res.phycfgr, now, sizeof(now));
        (void)w5500::deferLog(TAG, 'W', esp_log_timestamp(),
                              "W5500 link changed: %s (0x%02X) -> %s (0x%02X)", was,
                              last_phy, now, res.phycfgr);
        last_phy = res.phycfgr;
    }
    checkW5500SocketDests(hal);
    if (res.ok) return;
    (void)w5500::deferLog(
        TAG, 'W', esp_log_timestamp(),
        "W5500 SPI read-back mismatch: ver=0x%02X mac=%02X:%02X:%02X:%02X:%02X:%02X "
        "ip=%u.%u.%u.%u",
        res.version, res.mac[0], res.mac[1], res.mac[2], res.mac[3],
             res.mac[4], res.mac[5], res.ip[0], res.ip[1], res.ip[2], res.ip[3]);
}

inline
void logTimingSnapshotPeriodic(uint32_t& cycles) {
  ++cycles;
  if ((cycles % kTimingLogEveryN) != 0) return;
  const Class1TimingSnapshot ex = class1TimingStats().exchange();
  const Class1TimingSnapshot ot = class1TimingStats().otSend();
  const Class1TimingSnapshot cy = class1TimingStats().cycle();
  const Class1TimingSnapshot cs = class1TimingStats().cmdToStart();
  ESP_LOGD(TAG,
           "Class1 timing n=%lu exchange p50/p99=%lu/%lu us ot=%lu/%lu "
           "cycle=%lu/%lu cmd2start=%lu/%lu (budget: exchange p99 < RPI=%lu us)",
           static_cast<unsigned long>(ex.count),
           static_cast<unsigned long>(ex.p50_us),
           static_cast<unsigned long>(ex.p99_us),
           static_cast<unsigned long>(ot.p50_us),
           static_cast<unsigned long>(ot.p99_us),
           static_cast<unsigned long>(cy.p50_us),
           static_cast<unsigned long>(cy.p99_us),
           static_cast<unsigned long>(cs.p50_us),
           static_cast<unsigned long>(cs.p99_us),
           static_cast<unsigned long>(g_class1_budget_rpi_us));
}
inline
void keepaliveTask(void* arg) {
  auto* ctx = static_cast<KeepaliveCtx*>(arg);
  while (ctx->run.load()) {
    if (!ctx->scanner->sendKeepaliveAll()) {
      ctx->failed = true;
    }
    if (ctx->drain) {
      (void)ctx->scanner->drainBufferedInputs();
    }
    vTaskDelay(pdMS_TO_TICKS(ctx->period_ms > 0 ? ctx->period_ms : 5));
  }
  ctx->exited = true;
  vTaskDelete(nullptr);
}

// Chip recover after disconnect. After kMaxChipRecovers consecutive recovers
// that still cannot sustain Class 1, back off and let supervision handle it.
inline
void escalateChipRecover(ScannerTaskCtx* ctx, unsigned& recover_streak,
                         uint32_t& backoff_ms) {
    if (chipRecoverOnFailure(recover_streak) == ChipRecoverDecision::kRestart) {
      ESP_LOGE(TAG,
               "W5500 recover exhausted (%u) — backing off, will retry",
               recover_streak);
      vTaskDelay(pdMS_TO_TICKS(10000));
      recover_streak = 0;
      backoff_ms = 1000;
    }

  const bool ok = ctx->chip != nullptr && ctx->chip->recover();
  reliabilityStats().noteChipRecover();
  if (ok) {
    ESP_LOGW(TAG, "W5500 recover #%u OK — settling link", recover_streak);
    backoff_ms = chipRecoverNextBackoffMs(backoff_ms, true);
    vTaskDelay(pdMS_TO_TICKS(kLinkSettleMs + kPostResetSettleMs));
  } else {
    ESP_LOGW(TAG, "W5500 recover #%u failed — backoff %lu ms", recover_streak,
             static_cast<unsigned long>(backoff_ms));
    vTaskDelay(pdMS_TO_TICKS(backoff_ms));
    backoff_ms = chipRecoverNextBackoffMs(backoff_ms, false);
  }
}

// Stop a keepalive helper and wait for it to leave (its ctx lives on our
// stack, so it must be gone before the ctx goes out of scope).
inline
void stopKeepalive(KeepaliveCtx& ka) {
  ka.run = false;
  const TickType_t start = xTaskGetTickCount();
  while (!ka.exited.load() &&
         (xTaskGetTickCount() - start) < pdMS_TO_TICKS(kKeepaliveExitWaitMs)) {
    vTaskDelay(1);
  }
  if (!ka.exited.load()) {
    ESP_LOGE(TAG, "Keepalive helper did not exit within %lu ms",
             static_cast<unsigned long>(kKeepaliveExitWaitMs));
  }
}

// Per-axis reopen (HCS01 theta) while X/Z stay up. The FO blocks this task
// for up to the TCP connect + explicit-message timeouts, so a helper keeps
// sending X/Z O->T and draining their T->O meanwhile. Returns false if the
// helper saw a Kinetix O->T failure (caller escalates to full recover).
inline
bool reopenAxisWithKeepalive(EipMultiScanner& scanner, size_t axis) {
  // Mark the axis idle *before* the helper exists so it never touches it.
  scanner.prepareAxisReopen(axis);

  KeepaliveCtx ka{&scanner, true, false, kReopenKeepalivePeriodMs};
  ka.drain = true;
  TaskHandle_t handle = nullptr;
  if (xTaskCreatePinnedToCore(keepaliveTask, "EipReopenKA", kKeepaliveStack, &ka,
                              kKeepalivePriority, &handle, kScannerCore) != pdPASS) {
    ESP_LOGE(TAG, "EipReopenKA create failed — axis%u reopen deferred",
             static_cast<unsigned>(axis));
    scanner.deferAxisReopen(axis);
    return true;
  }

  (void)scanner.reopenAxis(axis);
  stopKeepalive(ka);
  (void)scanner.flushDeferredLogs();
  return !ka.failed.load();
}
// Each axis must see its ForwardClose before its connection times out, so a
// helper keeps O->T running to the axes still open while the others close.
inline
void closeClass1InOrder(EipMultiScanner& scanner) {
  KeepaliveCtx ka{&scanner, true, false, kReopenKeepalivePeriodMs};
  ka.drain = true;
  TaskHandle_t handle = nullptr;
  const bool helper = xTaskCreatePinnedToCore(keepaliveTask, "EipCloseKA", kKeepaliveStack, &ka,
                                              kKeepalivePriority, &handle,
                                              kScannerCore) == pdPASS;
  uint32_t refused = 0;
  for (size_t i = 0; i < scanner.axisCount(); ++i) {
    if (!scanner.gracefulCloseAxis(i)) {
      refused |= 1u << i;
    }
  }
  if (helper) {
    stopKeepalive(ka);
  }
  if (!helper || refused != 0) {
    ESP_LOGW(TAG, "Class 1 close not clean (helper=%d, refused axis mask=0x%lX)", helper ? 1 : 0,
             static_cast<unsigned long>(refused));
  }
}

}  // namespace eip

#endif  // CONFIG_EIP_SCANNER_ENABLED

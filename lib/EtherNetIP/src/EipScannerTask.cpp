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

namespace {

static const char* TAG = "EipScanner";

// Above gantry update (5) / pick (4) / console (1) so Class 1 preempts SPI3 users.
// Pin to Core 1 so Core 0 IDLE / UART / NetConsole / SPI3 are not starved (TWDT).
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

// Last Class 1 RPI used for pacing (granted API or CONFIG fallback).
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

// Tick-aligned Class 1 cadence. xTaskDelayUntil always blocks when on time so
// IDLE1 can feed the task WDT. Sub-ms RPI fraction (if any) is paced after
// that block. On overrun, catch up without reseeding last_wake; yield one tick
// every N consecutive overruns (TWDT safety if exchange > RPI).
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
    }
    return;
  }
  overrun_streak = 0;
  const uint32_t frac = class1RpiFractionUs(rpi_us, kTickUs);
  if (frac != 0) {
    paceClass1RemainderUs(frac);
  }
}

// Dropout diagnostics (W5500Diag): read-only, nothing here recovers anything.
constexpr uint32_t kSpiCheckEveryN = 50;  // 100 ms at RPI 2 ms

// Dump the chip state captured at the first failure. Runs after the exchange
// released the bus and before teardown or the chip reset touch it.
void logW5500FailureSnapshot(w5500::W5500Hal& hal, w5500::FailReason fallback) {
    w5500::FailureSnapshot snap;
    if (!w5500::takeFailureSnapshot(snap)) {
        // No failed send recorded one (stale T->O / reopen): capture now.
        {
            std::lock_guard<w5500::BusMutex> lock(w5500::spiBusMutex());
            w5500::captureFailureSnapshot(hal, 0xFF, fallback,
                                          w5500::socketSoftTxState());
        }
        if (!w5500::takeFailureSnapshot(snap)) {
            ESP_LOGW(TAG, "W5500 snapshot: none captured");
            return;
        }
    }
    char line[320];
    w5500::formatSnapshotCommon(snap, line, sizeof(line));
    ESP_LOGW(TAG, "W5500 snapshot: %s", line);
    {
        // Second PHYCFGR read, milliseconds later: tells a real speed/duplex
        // change from a one-off bad read of the snapshot value.
        uint8_t phy_now = 0;
        {
            std::lock_guard<w5500::BusMutex> lock(w5500::spiBusMutex());
            phy_now = hal.readReg(kBlockCommon(), REG_PHYCFGR);
        }
        char mode[24];
        w5500::formatPhy(phy_now, mode, sizeof(mode));
        ESP_LOGW(TAG, "W5500 snapshot: phy re-read %lld ms later = 0x%02X (%s)",
                 static_cast<long long>((esp_timer_get_time() - snap.t_us) / 1000),
                 phy_now, mode);
    }
    for (uint8_t i = 0; i < 8; ++i) {
        w5500::formatSnapshotSocket(snap, i, line, sizeof(line));
        ESP_LOGW(TAG, "W5500 snapshot: %s", line);
    }
    const w5500::SpiCheckStats st = w5500::spiCheckStats();
    ESP_LOGW(TAG,
             "W5500 SPI check: %lu reads, bad ver/mac/ip=%lu/%lu/%lu, bad rx hdr=%lu",
             static_cast<unsigned long>(st.checks),
             static_cast<unsigned long>(st.version_bad),
             static_cast<unsigned long>(st.mac_bad),
             static_cast<unsigned long>(st.ip_bad),
             static_cast<unsigned long>(st.rx_hdr_bad));
}

// Negotiated link mode, logged whenever Class 1 starts so a 10M/half-duplex
// link after a chip reset is visible.
uint8_t logW5500LinkMode(w5500::W5500Hal& hal) {
    uint8_t phy = 0;
    {
        std::lock_guard<w5500::BusMutex> lock(w5500::spiBusMutex());
        phy = hal.readReg(kBlockCommon(), REG_PHYCFGR);
    }
    char mode[24];
    w5500::formatPhy(phy, mode, sizeof(mode));
    ESP_LOGI(TAG, "W5500 link: %s (phy=0x%02X)", mode, phy);
    return phy;
}

// Cyclic loop left because PHYCFGR LNK read 0. Log it with a snapshot, then
// probe for a short blip (connector break) vs a real drop. Read-only: teardown
// and reconnect proceed exactly as before.
constexpr int kLinkBlipProbeMs = 50;

int64_t logW5500LinkDown(w5500::W5500Hal& hal) {
    const int64_t down_us = esp_timer_get_time();
    uint8_t phy = 0;
    {
        std::lock_guard<w5500::BusMutex> lock(w5500::spiBusMutex());
        phy = hal.readReg(kBlockCommon(), REG_PHYCFGR);
    }
    char mode[24];
    w5500::formatPhy(phy, mode, sizeof(mode));
    ESP_LOGW(TAG, "W5500 link DOWN (%s, phy=0x%02X) — Class 1 teardown", mode, phy);
    logW5500FailureSnapshot(hal, w5500::FailReason::kLinkDown);
    for (int ms = 1; ms <= kLinkBlipProbeMs; ++ms) {
        vTaskDelay(pdMS_TO_TICKS(1));
        {
            std::lock_guard<w5500::BusMutex> lock(w5500::spiBusMutex());
            phy = hal.readReg(kBlockCommon(), REG_PHYCFGR);
        }
        if (phy & 0x01) {
            w5500::formatPhy(phy, mode, sizeof(mode));
            ESP_LOGW(TAG, "W5500 link back after %d ms (%s) — a blip, not a renegotiation",
                     ms, mode);
            return down_us;
        }
    }
    ESP_LOGW(TAG, "W5500 link still down after %d ms", kLinkBlipProbeMs);
    return down_us;
}

// A real 100BASE-TX link loss lasts >1 s (renegotiation). Require LNK=0 on
// this many consecutive cycles before tearing down: on 2026-10-20 16:24 one
// LNK=0 read (link back within 1 ms, no renegotiation) caused a full drop.
constexpr uint32_t kLinkDownConfirmReads = 3;

// True when the W5500 no longer holds the MAC/IP written at init/recover: the
// chip reset itself (RSTn pulse or brownout) and every connect would fail.
// The caller restores the chip config; motion stays held (servos off, E602).
bool w5500LostConfig(w5500::W5500Hal& hal) {
    const w5500::SpiCheckResult res = w5500::spiIntegrityCheck(hal);
    if (res.ok) return false;
    ESP_LOGW(TAG,
             "W5500 reset itself (ver=0x%02X mac=%02X:%02X:%02X:%02X:%02X:%02X "
             "ip=%u.%u.%u.%u) — restoring chip config; motion stays held",
             res.version, res.mac[0], res.mac[1], res.mac[2], res.mac[3],
             res.mac[4], res.mac[5], res.ip[0], res.ip[1], res.ip[2], res.ip[3]);
    return true;
}

// Class 1 stall diagnostics: on 2026-10-20 the stale ages showed the multi
// task skipping 10-40 ms between exchanges. Track the worst loop gap (split
// into the previous iteration's exchange / log flush / rest) and the W5500
// bus mutex's longest hold and wait. The Class 1 task only records; the
// class1LogTask (core 0) prints, so no ESP_LOG or console TCP send runs in
// the cyclic loop.
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
portMUX_TYPE g_diag_mux = portMUX_INITIALIZER_UNLOCKED;
LoopGap g_worst_gap;    // guarded by g_diag_mux
int64_t g_pace_us = 0;  // previous iteration's time in paceClass1Cycle

// Run-time trace (G-B6): class1LogTask snapshots per-task CPU time each period,
// and a gap of kRunTimeGapUs or more is reported as CPU time per task across it.
constexpr uint32_t kRunTimeGapUs = 10000;
constexpr int64_t kRunTimeMinIntervalUs = 5000000;
constexpr size_t kRunTimeReportTasks = 8;
RunTimeHistory g_rt_history;  // class1LogTask only, as are the tables below

// Listing the tasks takes the kernel lock while scanning every stack, so it
// runs only when the task count changes. Each period reads the known tasks'
// counters without the stack scan.
struct KnownTask {
    TaskHandle_t handle = nullptr;
    uint32_t number = 0;
    int8_t core = -1;
    char name[configMAX_TASK_NAME_LEN] = {};
};
TaskStatus_t g_rt_status[kRunTimeMaxTasks];
KnownTask g_rt_known[kRunTimeMaxTasks];
size_t g_rt_known_count = 0;
UBaseType_t g_rt_known_total = 0;
std::atomic<bool> g_rt_pending{false};
uint32_t g_rt_gap_us = 0;  // set before g_rt_pending, with the breakdown below
uint32_t g_rt_end_us = 0;
uint32_t g_rt_exchange_us = 0;
uint32_t g_rt_rest_us = 0;
uint32_t g_rt_pace_us = 0;
int64_t g_rt_last_us = 0;  // scanner task only

// The last report, kept for 'gaptrace': a log burst (cfg export) can make the
// net console drop the line, as it did on 2026-11-03.
std::mutex g_rt_report_mutex;
char g_rt_report[400] = "none since boot";
uint32_t g_rt_report_count = 0;

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

bool listRunTimeTasks() {
    configRUN_TIME_COUNTER_TYPE total = 0;
    const UBaseType_t n = uxTaskGetSystemState(g_rt_status, kRunTimeMaxTasks, &total);
    g_rt_known_count = 0;
    if (n == 0) {
        return false;  // more tasks than kRunTimeMaxTasks
    }
    for (UBaseType_t i = 0; i < n; ++i) {
        KnownTask& k = g_rt_known[g_rt_known_count++];
        const BaseType_t core = xTaskGetCoreID(g_rt_status[i].xHandle);
        k.handle = g_rt_status[i].xHandle;
        k.number = static_cast<uint32_t>(g_rt_status[i].xTaskNumber);
        k.core = (core == tskNO_AFFINITY) ? -1 : static_cast<int8_t>(core);
        strncpy(k.name, g_rt_status[i].pcTaskName, sizeof(k.name) - 1);
        k.name[sizeof(k.name) - 1] = '\0';
    }
    g_rt_known_total = uxTaskGetNumberOfTasks();
    return true;
}

bool takeRunTimeSnapshot(RunTimeSnapshot& snap) {
    if (g_rt_known_count == 0 || uxTaskGetNumberOfTasks() != g_rt_known_total) {
        if (!listRunTimeTasks()) {
            return false;
        }
    }
    snap.t_us = static_cast<uint32_t>(esp_timer_get_time());
    snap.count = static_cast<uint8_t>(g_rt_known_count);
    for (size_t i = 0; i < g_rt_known_count; ++i) {
        TaskStatus_t st;
        // pdFALSE: no stack scan; a known state skips eTaskGetState().
        vTaskGetInfo(g_rt_known[i].handle, &st, pdFALSE, eRunning);
        snap.tasks[i].task_number = g_rt_known[i].number;
        snap.tasks[i].run_us = static_cast<uint32_t>(st.ulRunTimeCounter);
        snap.tasks[i].core = g_rt_known[i].core;
    }
    return true;
}

const char* runTimeTaskName(uint32_t task_number) {
    for (size_t i = 0; i < g_rt_known_count; ++i) {
        if (g_rt_known[i].number == task_number) {
            return g_rt_known[i].name;
        }
    }
    return "?";
}

// Called by class1LogTask each period: one snapshot, and the report when due.
void serviceRunTimeTrace() {
    static RunTimeSnapshot snap;
    if (!takeRunTimeSnapshot(snap)) {
        return;
    }
    if (g_rt_pending.load()) {
        const uint32_t start_us = g_rt_end_us - g_rt_gap_us;
        const RunTimeSnapshot* before = g_rt_history.atOrBefore(start_us);
        static char line[220];  // static: EipLog has a 4 KB stack
        size_t used = 0;
        if (before != nullptr) {
            TaskRunDelta top[kRunTimeReportTasks];
            const size_t n = diffRunTimes(*before, snap, top, kRunTimeReportTasks);
            used = static_cast<size_t>(snprintf(line, sizeof(line), " CPU ms over %lu ms:",
                static_cast<unsigned long>((snap.t_us - before->t_us) / 1000)));
            for (size_t i = 0; i < n && used < sizeof(line); ++i) {
                char core[6];
                if (top[i].core < 0) {
                    snprintf(core, sizeof(core), "any");
                } else {
                    snprintf(core, sizeof(core), "c%d", static_cast<int>(top[i].core));
                }
                const int w = snprintf(line + used, sizeof(line) - used, " %s/%s %lu.%lu |",
                                       runTimeTaskName(top[i].task_number), core,
                                       static_cast<unsigned long>(top[i].delta_us / 1000),
                                       static_cast<unsigned long>((top[i].delta_us % 1000) / 100));
                if (w > 0) {
                    used += static_cast<size_t>(w);
                }
            }
        } else {
            snprintf(line, sizeof(line), " run-time history too short to cover it");
        }
        {
            std::lock_guard<std::mutex> lock(g_rt_report_mutex);
            ++g_rt_report_count;
            snprintf(g_rt_report, sizeof(g_rt_report),
                     "#%lu at %lu ms: gap %lu us (exchange %lu, rest %lu, pace %lu);%s",
                     static_cast<unsigned long>(g_rt_report_count),
                     static_cast<unsigned long>(g_rt_end_us / 1000),
                     static_cast<unsigned long>(g_rt_gap_us),
                     static_cast<unsigned long>(g_rt_exchange_us),
                     static_cast<unsigned long>(g_rt_rest_us),
                     static_cast<unsigned long>(g_rt_pace_us), line);
            ESP_LOGW(TAG, "Class 1 gap report %s", g_rt_report);
        }
        g_rt_pending.store(false);
    }
    g_rt_history.push(snap);
}

// paceClass1Cycle, timed for the loop-gap breakdown (multi-axis loop).
void pacedClass1Cycle(TickType_t& last_wake, uint32_t rpi_us) {
    const int64_t tp = class1NowUs();
    paceClass1Cycle(last_wake, rpi_us);
    g_pace_us = class1NowUs() - tp;
}

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

void logClass1StallDiag(bool force) {
    const w5500::BusHoldStats bus = w5500::spiBusMutex().take();
    portENTER_CRITICAL(&g_diag_mux);
    const LoopGap gap = g_worst_gap;
    g_worst_gap = LoopGap{};
    portEXIT_CRITICAL(&g_diag_mux);
    if (force || gap.gap_us >= kLoopGapReportUs) {
        ESP_LOGW(TAG,
                 "Class 1 loop gap %lu us: exchange %lu (ot %lu, drain %lu), "
                 "log flush %lu, rest %lu (pace %lu)",
                 static_cast<unsigned long>(gap.gap_us),
                 static_cast<unsigned long>(gap.exchange_us),
                 static_cast<unsigned long>(gap.ot_us),
                 static_cast<unsigned long>(gap.drain_us),
                 static_cast<unsigned long>(gap.flush_us),
                 static_cast<unsigned long>(gap.rest_us),
                 static_cast<unsigned long>(gap.pace_us));
    }
    if (force || bus.max_hold_us >= kBusHoldReportUs ||
        bus.max_wait_us >= kBusHoldReportUs) {
        ESP_LOGW(TAG, "W5500 bus mutex: longest hold %lu us by %s, longest wait %lu us by %s",
                 static_cast<unsigned long>(bus.max_hold_us), bus.holder,
                 static_cast<unsigned long>(bus.max_wait_us), bus.waiter);
    }
}

// Logic-analyzer marker (cfg eip_scope_gpio): high during each exchange.
int g_scope_gpio = -1;

inline void scopeMark(int level) {
    if (g_scope_gpio >= 0) {
        gpio_set_level(static_cast<gpio_num_t>(g_scope_gpio), static_cast<uint32_t>(level));
    }
}

// Snapshot taken while X/Z stay up (theta reopen); class1LogTask prints it
// one line per 100 ms.
std::mutex g_trickle_mutex;
w5500::FailureSnapshot g_trickle_snap;  // guarded by g_trickle_mutex
int g_trickle_line = -1;  // -1 idle; 0 announce; 1 common line; 2..9 sockets
uint8_t g_trickle_axis = 0;

void captureSnapshotForTrickle(w5500::W5500Hal& hal, size_t axis) {
    {
        std::lock_guard<w5500::BusMutex> lock(w5500::spiBusMutex());
        w5500::captureFailureSnapshot(hal, 0xFF, w5500::FailReason::kInputStale,
                                      w5500::socketSoftTxState());
    }
    std::lock_guard<std::mutex> lock(g_trickle_mutex);
    if (g_trickle_line < 0 && w5500::takeFailureSnapshot(g_trickle_snap)) {
        g_trickle_axis = static_cast<uint8_t>(axis);
        g_trickle_line = 0;
    }
}

void trickleSnapshotLine() {
    char line[320];
    int which = -1;
    uint8_t axis = 0;
    {
        std::lock_guard<std::mutex> lock(g_trickle_mutex);
        if (g_trickle_line < 0) return;
        which = g_trickle_line;
        axis = g_trickle_axis;
        if (which == 1) {
            w5500::formatSnapshotCommon(g_trickle_snap, line, sizeof(line));
        } else if (which >= 2) {
            w5500::formatSnapshotSocket(g_trickle_snap, static_cast<uint8_t>(which - 2),
                                        line, sizeof(line));
        }
        g_trickle_line = (which >= 9) ? -1 : which + 1;
    }
    if (which == 0) {
        ESP_LOGW(TAG, "axis%u silent — W5500 snapshot before reopen follows",
                 static_cast<unsigned>(axis));
    } else {
        ESP_LOGW(TAG, "W5500 snapshot: %s", line);
    }
}

// Scanner whose deferred Class 1 log the logger task drains while the cyclic
// loop runs. Cleared (under the mutex) before the scanner goes out of scope.
std::mutex g_log_scanner_mutex;
EipMultiScanner* g_log_scanner = nullptr;

void setLogScanner(EipMultiScanner* scanner) {
    std::lock_guard<std::mutex> lock(g_log_scanner_mutex);
    g_log_scanner = scanner;
}

constexpr UBaseType_t kLogTaskPriority = 2;
constexpr BaseType_t kLogTaskCore = 0;
constexpr uint32_t kLogTaskStack = 4096;
constexpr uint32_t kLogTaskPeriodMs = 20;

void class1LogTask(void*) {
    uint32_t ticks = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(kLogTaskPeriodMs));
        {
            std::lock_guard<std::mutex> lock(g_log_scanner_mutex);
            if (g_log_scanner != nullptr) {
                (void)g_log_scanner->flushDeferredLogs();
            }
        }
        serviceRunTimeTrace();
        if (++ticks >= 100 / kLogTaskPeriodMs) {
            ticks = 0;
            trickleSnapshotLine();
            logClass1StallDiag(false);
        }
    }
}

// Class 1 UDP destinations are written once and cached, so a write that lands
// in Sn_DIPR/Sn_DPORT would silently send every later frame elsewhere (SENDOK
// still fires). Compare them with the cache; log each socket once per change.
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
        ESP_LOGW(TAG,
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
        ESP_LOGW(TAG, "W5500 link changed: %s (0x%02X) -> %s (0x%02X)", was,
                 last_phy, now, res.phycfgr);
        last_phy = res.phycfgr;
    }
    checkW5500SocketDests(hal);
    if (res.ok) return;
    ESP_LOGW(TAG,
             "W5500 SPI read-back mismatch: ver=0x%02X mac=%02X:%02X:%02X:%02X:%02X:%02X "
             "ip=%u.%u.%u.%u",
             res.version, res.mac[0], res.mac[1], res.mac[2], res.mac[3],
             res.mac[4], res.mac[5], res.ip[0], res.ip[1], res.ip[2], res.ip[3]);
}

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

ScannerConfig makeKinetixConfig(const char* ip, uint16_t connection_serial,
                                uint32_t ot_connection_id) {
  KinetixScannerArgs a;
  a.ip = ip;
  a.connection_serial = connection_serial;
  a.ot_connection_id = ot_connection_id;
  a.config_assembly_instance = CONFIG_EIP_CONFIG_ASSEMBLY_INSTANCE;
  a.rpi_us = CONFIG_EIP_X_RPI_US;
  a.originator_vendor_id = CONFIG_EIP_ORIGINATOR_VENDOR_ID;
#if defined(CONFIG_EIP_TO_POINT_TO_POINT) && !CONFIG_EIP_TO_POINT_TO_POINT
  a.to_point_to_point = false;
#endif
  return makeKinetixScannerConfig(a);
}

#if defined(CONFIG_EIP_AXIS_THETA)
ScannerConfig makeThetaConfig() {
  ThetaScannerArgs a;
  a.ip = CONFIG_EIP_TARGET_IP_THETA;
  a.rpi_us = CONFIG_EIP_THETA_RPI_US;
  a.originator_vendor_id = CONFIG_EIP_ORIGINATOR_VENDOR_ID;
#if defined(CONFIG_EIP_TO_POINT_TO_POINT) && !CONFIG_EIP_TO_POINT_TO_POINT
  a.to_point_to_point = false;
#endif
  return makeThetaScannerConfig(a);
}

[[maybe_unused]] void singleAxisTask(void* arg) {
  auto* ctx = static_cast<ScannerTaskCtx*>(arg);
  uint32_t backoff_ms = 1000;
  unsigned recover_streak = 0;

  while (true) {
    while (!ctx->link->isUp()) {
      backoff_ms = 1000;
      vTaskDelay(pdMS_TO_TICKS(kLinkPollMs));
    }

    EipSocketW5500Tcp tcp(*ctx->hal);
    EipSocketW5500Udp udp(*ctx->hal);
    EipScanner scanner(tcp, udp, makeThetaConfig());
    if (ctx->image_theta) scanner.setProcessImage(ctx->image_theta);

    if (!scanner.connect()) {
      // Missing/unreachable HCS01 (ARP/TCP) is not a W5500 fault. Soft-retry
      // so X/Z Class 1 is not yanked by escalateChipRecover / esp_restart.
      ESP_LOGW(TAG,
               "Theta connect failed — soft retry (no chip recover); "
               "check HCS01 CIP IP %s (FKM, not eng .22) / daisy-chain",
               CONFIG_EIP_TARGET_IP_THETA);
      scanner.disconnect();
      recover_streak = 0;
      vTaskDelay(pdMS_TO_TICKS(backoff_ms));
      backoff_ms = (backoff_ms < 15000) ? backoff_ms * 2 : 15000;
      continue;
    }
    recover_streak = 0;
    backoff_ms = 1000;

    const uint32_t rpi_us =
        (scanner.openReply().to_api_us > 0) ? scanner.openReply().to_api_us
                                            : CONFIG_EIP_THETA_RPI_US;
    g_class1_budget_rpi_us = rpi_us;
    const uint32_t recv_timeout_ms = scanner.recvTimeoutMs();
    uint32_t input_miss_streak = 0;
    bool need_chip_recover = false;
    uint32_t timing_cycles = 0;
    int64_t cycle_anchor_us = class1NowUs();
    TickType_t last_wake = xTaskGetTickCount();

    while (ctx->link->isUp()) {
      const int64_t t0 = class1NowUs();
      Bytes input;
      spi3_class1_critical_enter();
      (void)ctx->chip->acquireBus();
      const bool ok = scanner.exchangeOnce(recv_timeout_ms, input);
      ctx->chip->releaseBus();
      spi3_class1_critical_exit();
      if (!ok) {
        ++input_miss_streak;
        ESP_LOGW(TAG, "Theta exchange miss #%lu",
                 static_cast<unsigned long>(input_miss_streak));
        if (shouldTeardownAfterInputMisses(input_miss_streak)) {
          need_chip_recover = true;
          break;
        }
        paceClass1Cycle(last_wake, rpi_us);
        continue;
      }
      input_miss_streak = 0;
      const int64_t t1 = class1NowUs();
      const uint32_t elapsed_us =
          (t1 > t0) ? static_cast<uint32_t>(t1 - t0) : 0u;
      class1TimingStats().recordExchangeUs(elapsed_us);
      if (cycle_anchor_us > 0) {
        const int64_t cdt = t1 - cycle_anchor_us;
        if (cdt >= 0 && cdt <= static_cast<int64_t>(UINT32_MAX)) {
          class1TimingStats().recordCycleUs(static_cast<uint32_t>(cdt));
        }
      }
      cycle_anchor_us = t1;
      logTimingSnapshotPeriodic(timing_cycles);
      paceClass1Cycle(last_wake, rpi_us);
    }
    scanner.disconnect();
    vTaskDelay(pdMS_TO_TICKS(kReconnectIdleMs));
    if (need_chip_recover) {
      escalateChipRecover(ctx, recover_streak, backoff_ms);
    }
  }
}
#endif

// Each axis must see its ForwardClose before its connection times out, so a
// helper keeps O->T running to the axes still open while the others close.
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

// Requested by pauseClass1(); the multi-axis task closes Class 1 and waits here.
std::atomic<bool> g_class1_pause_request{false};
std::atomic<bool> g_class1_paused{false};

void holdClass1WhilePaused() {
  if (!g_class1_pause_request.load()) {
    return;
  }
  ESP_LOGI(TAG, "Class 1 closed for a parameter save");
  g_class1_paused.store(true);
  while (g_class1_pause_request.load()) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  g_class1_paused.store(false);
}

void multiAxisTask(void* arg) {
  auto* ctx = static_cast<ScannerTaskCtx*>(arg);
  uint32_t backoff_ms = 1000;
  unsigned recover_streak = 0;
  int64_t link_down_us = 0;  // set when the cyclic loop saw LNK=0

  MultiAxisSlot slots[EipMultiScanner::kMaxAxes];
  size_t n = 0;

#if defined(CONFIG_EIP_AXIS_X)
  slots[n].config =
      makeKinetixConfig(CONFIG_EIP_TARGET_IP_X, 0x0001, 0x10000001);
  slots[n].image = ctx->image_x;
  ++n;
#endif
#if defined(CONFIG_EIP_AXIS_Z)
  slots[n].config =
      makeKinetixConfig(CONFIG_EIP_TARGET_IP_Z, 0x0002, 0x10000002);
  slots[n].image = ctx->image_z;
  ++n;
#endif
#if defined(CONFIG_EIP_AXIS_THETA)
  if (ctx->image_theta != nullptr) {
    slots[n].config = makeThetaConfig();
    slots[n].image = ctx->image_theta;
    ++n;
  }
#endif

  if (n == 0) {
    ESP_LOGW(TAG, "No X/Z axes enabled; scanner idle");
    vTaskDelete(nullptr);
    return;
  }

  ESP_LOGI(TAG, "Multi-axis scanner: %u slot(s)", static_cast<unsigned>(n));

  while (true) {
    holdClass1WhilePaused();
    uint32_t link_wait_polls = 0;
    while (!ctx->link->isUp()) {
      holdClass1WhilePaused();
      backoff_ms = 1000;
      // This wait is otherwise silent: report a W5500 that never shows link
      // (unreadable chip, held in reset, cable) every 5 s.
      if (++link_wait_polls % 10 == 0) {
        uint8_t phy = 0;
        {
          std::lock_guard<w5500::BusMutex> lock(w5500::spiBusMutex());
          phy = ctx->hal->readReg(kBlockCommon(), REG_PHYCFGR);
        }
        ESP_LOGW(TAG, "W5500 link not up after %lu s (phy=0x%02X) — waiting",
                 static_cast<unsigned long>(link_wait_polls * kLinkPollMs / 1000), phy);
      }
      vTaskDelay(pdMS_TO_TICKS(kLinkPollMs));
    }
    if (link_down_us != 0) {
      // Upper bound: teardown plus the link poll period sit in between.
      ESP_LOGW(TAG, "W5500 link up again (seen %lld ms after it dropped)",
               static_cast<long long>((esp_timer_get_time() - link_down_us) / 1000));
      link_down_us = 0;
    }
    // PHYCFGR can assert LNK before the partner finishes auto-neg; brief settle
    // avoids INIT→CLOSED ARP failures right after cable/drive recovery.
    vTaskDelay(pdMS_TO_TICKS(kLinkSettleMs));
    if (!ctx->link->isUp()) {
      continue;
    }

    EipSocketW5500Tcp tcp0(*ctx->hal);
    EipSocketW5500Tcp tcp1(*ctx->hal);
    EipSocketW5500Tcp tcp2(*ctx->hal);
    EipSocketW5500Udp udp(*ctx->hal);
    ITcpClient* tcps[EipMultiScanner::kMaxAxes] = {&tcp0, &tcp1, &tcp2};

    EipMultiScanner scanner(tcps, n, udp, slots);

    // Stage connect so keepalive can run during later ForwardOpens.
    bool ok = scanner.openAxis(0);
    if (ok) ok = scanner.bindSharedUdp();

    KeepaliveCtx ka{&scanner, false, false, 5};
    TaskHandle_t ka_handle = nullptr;
    if (ok && n > 1) {
      ka.run = true;
      if (xTaskCreatePinnedToCore(keepaliveTask, "EipHoldKA", kKeepaliveStack, &ka,
                                  kKeepalivePriority, &ka_handle,
                                  kScannerCore) != pdPASS) {
        ESP_LOGE(TAG, "EipHoldKA create failed — peer FO without keepalive");
        ka.run = false;
        ka.exited = true;
      }
      for (size_t i = 1; i < n; ++i) {
        const bool opened = scanner.openAxis(i);
        if (opened) {
          continue;
        }
        if (slots[i].config.drive_family ==
            ScannerConfig::DriveFamily::kHcs01) {
          ESP_LOGW(TAG,
                   "Theta ForwardOpen failed — X/Z stay up; check HCS01 "
                   "map/FO sizes (per-axis reopen will retry)");
          scanner.requestAxisReopen(i);
          continue;
        }
        ok = false;
        break;
      }
      stopKeepalive(ka);
      if (ka.failed) {
        ESP_LOGW(TAG, "HoldKA O->T send failed during peer FO");
        ok = false;
      }
    }

    // Connect events print here. From the last priming keepalive below to the
    // first cyclic O->T this task must not log: a log that blocked (UART ring
    // full of the boot help text) left a drive without O->T long enough for
    // its connection to time out (G-B12).
    (void)scanner.flushDeferredLogs();

    if (!ok && w5500LostConfig(*ctx->hal)) {
      // Not a CIP reject: the chip lost its MAC/IP, so soft retries can
      // never connect. Restore it through the existing chip recover.
      scanner.disconnect();
      escalateChipRecover(ctx, recover_streak, backoff_ms);
      continue;
    }
    if (!ok) {
      // CIP ForwardOpen reject (ownership / RPI / size) is not a W5500 fault.
      // Chip recover + esp_restart() only masks the real extended status and
      // can fight a PC hold. Soft-backoff and retry instead.
      ESP_LOGW(TAG,
               "Multi-axis connect failed — soft retry (no chip recover); "
               "check FO extended status / PC exclusive hold / RPI");
      scanner.disconnect();
      recover_streak = 0;
      vTaskDelay(pdMS_TO_TICKS(backoff_ms));
      backoff_ms = (backoff_ms < 15000) ? backoff_ms * 2 : 15000;
      continue;
    }

    recover_streak = 0;
    backoff_ms = 1000;
    const uint32_t rpi_us =
        (scanner.openReply(0).to_api_us > 0) ? scanner.openReply(0).to_api_us
                                            : CONFIG_EIP_X_RPI_US;

    ESP_LOGI(TAG, "Multi-axis Class 1 running (RPI=%lu us)",
             static_cast<unsigned long>(rpi_us));
    uint8_t last_phy = logW5500LinkMode(*ctx->hal);
    g_class1_budget_rpi_us = rpi_us;

    // Prime UDP dest/ARP for every axis and empty RX before cyclic start.
    // No logging from here on: priming events wait for the log task
    // (setLogScanner below), so cyclic O->T follows the keepalives at once.
    for (int prime = 0; prime < 4; ++prime) {
      (void)scanner.sendKeepaliveAll();
      (void)scanner.drainBufferedInputs();
      vTaskDelay(pdMS_TO_TICKS(2));
    }
    class1TimingStats().reset();
    // Seed feedback clocks at cyclic start — FO timestamps must not count.
    scanner.beginCyclicExchange();

    uint32_t soft_miss_last_warn_ms = 0;
    uint32_t input_miss_streak = 0;
    bool need_chip_recover = false;
    bool planned_close = false;
    uint32_t timing_cycles = 0;
    uint32_t spi_check_cycles = 0;
    int64_t cycle_anchor_us = class1NowUs();
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t lnk_low_reads = 0;
    int64_t prev_t0 = 0;
    int64_t prev_t1 = 0;
    int64_t prev_t2 = 0;
    setLogScanner(&scanner);

    while (true) {
      if (g_class1_pause_request.load()) {
        planned_close = true;
        break;
      }
      if (!ctx->link->isUp()) {
        if (++lnk_low_reads >= kLinkDownConfirmReads) break;
      } else if (lnk_low_reads != 0) {
        ESP_LOGW(TAG, "W5500 LNK=0 on %lu read(s), then up — not a link loss",
                 static_cast<unsigned long>(lnk_low_reads));
        lnk_low_reads = 0;
      }
      const int64_t t0 = class1NowUs();
      noteLoopGap(t0, prev_t0, prev_t1, prev_t2, scanner.lastOtSendUs(),
                  scanner.lastDrainUs());
      scopeMark(1);
      spi3_class1_critical_enter();
      (void)ctx->chip->acquireBus();
      const ExchangeStatus st = scanner.exchangeOnce(0);
      ctx->chip->releaseBus();
      spi3_class1_critical_exit();
      scopeMark(0);
      const int64_t t1 = class1NowUs();
      // Queued Class 1 events are printed by class1LogTask (core 0): a log
      // line here waits on stdout and the LAN console's lwIP send.
      prev_t0 = t0;
      prev_t1 = t1;
      prev_t2 = class1NowUs();
      const uint32_t elapsed_us =
          (t1 > t0) ? static_cast<uint32_t>(t1 - t0) : 0u;
      class1TimingStats().recordExchangeUs(elapsed_us);
      if (cycle_anchor_us > 0) {
        const int64_t cdt = t1 - cycle_anchor_us;
        if (cdt >= 0 && cdt <= static_cast<int64_t>(UINT32_MAX)) {
          class1TimingStats().recordCycleUs(static_cast<uint32_t>(cdt));
        }
      }
      cycle_anchor_us = t1;

      if (st == ExchangeStatus::kOutputSendFailed) {
        reliabilityStats().noteSendOkFail();
        ESP_LOGW(TAG, "O->T send failed — reconnect + chip recover");
        logW5500FailureSnapshot(*ctx->hal, w5500::FailReason::kSendOkTimeout);
        need_chip_recover = true;
        break;
      }
      if (st == ExchangeStatus::kInputMiss) {
        reliabilityStats().noteSoftMiss();
        ++input_miss_streak;
        const uint32_t now_ms =
            static_cast<uint32_t>(esp_timer_get_time() / 1000LL);
        if (shouldWarnSoftMiss(soft_miss_last_warn_ms, now_ms)) {
          ESP_LOGW(TAG, "T->O soft-miss #%lu (stale beyond %lux RPI after cyclic)",
                   static_cast<unsigned long>(input_miss_streak),
                   static_cast<unsigned long>(kClass1StaleTeardownRpiMultiples));
        }
        // Keep pumping O->T for a few cycles before dual teardown + recover.
        if (shouldTeardownAfterInputMisses(input_miss_streak)) {
          ESP_LOGW(TAG, "T->O stale beyond soft-retry — reconnect + chip recover");
          logClass1StallDiag(true);
          logW5500FailureSnapshot(*ctx->hal, w5500::FailReason::kInputStale);
          need_chip_recover = true;
          break;
        }
        pacedClass1Cycle(last_wake, rpi_us);
        continue;
      }
      input_miss_streak = 0;

      // Theta (HCS01) dropped while X/Z are healthy: re-open just that axis.
      const size_t reopen_axis = scanner.nextAxisToReopen();
      if (reopen_axis < EipMultiScanner::kMaxAxes) {
        // Chip state before the reopen rebuilds the axis socket; printed a
        // line per 100 ms so X/Z keep their cycle.
        captureSnapshotForTrickle(*ctx->hal, reopen_axis);
        if (!reopenAxisWithKeepalive(scanner, reopen_axis)) {
          reliabilityStats().noteSendOkFail();
          ESP_LOGW(TAG, "O->T send failed during axis%u reopen — reconnect + chip recover",
                   static_cast<unsigned>(reopen_axis));
          logW5500FailureSnapshot(*ctx->hal, w5500::FailReason::kSendOkTimeout);
          need_chip_recover = true;
          break;
        }
        // The reopen blocked this task; restart pacing from now.
        last_wake = xTaskGetTickCount();
        cycle_anchor_us = class1NowUs();
        continue;
      }

      // Class 1 is not encapsulation activity, so without this each drive
      // closes its explicit TCP session after its inactivity timeout (G-B2).
      scanner.serviceEncapKeepalive(class1NowUs());
      logTimingSnapshotPeriodic(timing_cycles);
      checkW5500SpiPeriodic(*ctx->hal, spi_check_cycles, last_phy);
      pacedClass1Cycle(last_wake, rpi_us);
    }

    if (planned_close) {
      setLogScanner(nullptr);
      closeClass1InOrder(scanner);
      (void)scanner.flushDeferredLogs();
      scanner.disconnect();
      continue;
    }

    // Every other break sets need_chip_recover, so a plain exit is a
    // confirmed LNK=0. A chip that reset itself also needs its config back.
    if (!need_chip_recover) {
      link_down_us = logW5500LinkDown(*ctx->hal);
      need_chip_recover = w5500LostConfig(*ctx->hal);
    }
    setLogScanner(nullptr);
    (void)scanner.flushDeferredLogs();
    scanner.disconnect();
    reliabilityStats().noteReconnect();
    vTaskDelay(pdMS_TO_TICKS(kReconnectIdleMs));
    if (need_chip_recover) {
      escalateChipRecover(ctx, recover_streak, backoff_ms);
    }
  }
}

}  // namespace

bool pauseClass1(uint32_t timeout_ms) {
  g_class1_pause_request.store(true);
  const int64_t deadline_us = esp_timer_get_time() + static_cast<int64_t>(timeout_ms) * 1000;
  while (!g_class1_paused.load()) {
    if (esp_timer_get_time() > deadline_us) {
      g_class1_pause_request.store(false);
      return false;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  return true;
}

void resumeClass1() {
  g_class1_pause_request.store(false);
}

void setScopeMarkerGpio(int gpio) {
  if (gpio < 0) {
    g_scope_gpio = -1;
    return;
  }
  gpio_config_t io = {};
  io.pin_bit_mask = (1ULL << gpio);
  io.mode = GPIO_MODE_OUTPUT;
  if (gpio_config(&io) != ESP_OK) return;
  gpio_set_level(static_cast<gpio_num_t>(gpio), 0);
  g_scope_gpio = gpio;
  ESP_LOGW(TAG, "Class 1 exchange marker on GPIO%d (cfg eip_scope_gpio)", gpio);
}

void printLastClass1GapReport() {
    std::lock_guard<std::mutex> lock(g_rt_report_mutex);
    ESP_LOGI(TAG, "Class 1 gap report (10 ms or more; %lu since boot): %s",
             static_cast<unsigned long>(g_rt_report_count), g_rt_report);
}

void dumpClass1TimingStats() {
  const Class1TimingSnapshot ex = class1TimingStats().exchange();
  const Class1TimingSnapshot ot = class1TimingStats().otSend();
  const Class1TimingSnapshot dr = class1TimingStats().toDrain();
  const Class1TimingSnapshot cy = class1TimingStats().cycle();
  const Class1TimingSnapshot cs = class1TimingStats().cmdToStart();
  const uint32_t budget_us = g_class1_budget_rpi_us;
  const bool nogo = (ex.count > 0) && (ex.p99_us >= budget_us);
  ESP_LOGI(TAG, "=== Class 1 timing (budget: exchange p99 < RPI=%lu us) ===",
           static_cast<unsigned long>(budget_us));
  ESP_LOGI(TAG, "  exchange  n=%lu min/p50/p99/max=%lu/%lu/%lu/%lu us%s",
           static_cast<unsigned long>(ex.count),
           static_cast<unsigned long>(ex.min_us),
           static_cast<unsigned long>(ex.p50_us),
           static_cast<unsigned long>(ex.p99_us),
           static_cast<unsigned long>(ex.max_us),
           nogo ? "  NO-GO (p99>=RPI)" : (ex.count > 0 ? "  GO" : ""));
  ESP_LOGI(TAG, "  ot_send   n=%lu min/p50/p99/max=%lu/%lu/%lu/%lu us",
           static_cast<unsigned long>(ot.count),
           static_cast<unsigned long>(ot.min_us),
           static_cast<unsigned long>(ot.p50_us),
           static_cast<unsigned long>(ot.p99_us),
           static_cast<unsigned long>(ot.max_us));
  ESP_LOGI(TAG, "  drain     n=%lu min/p50/p99/max=%lu/%lu/%lu/%lu us",
           static_cast<unsigned long>(dr.count),
           static_cast<unsigned long>(dr.min_us),
           static_cast<unsigned long>(dr.p50_us),
           static_cast<unsigned long>(dr.p99_us),
           static_cast<unsigned long>(dr.max_us));
  ESP_LOGI(TAG, "  cycle     n=%lu min/p50/p99/max=%lu/%lu/%lu/%lu us",
           static_cast<unsigned long>(cy.count),
           static_cast<unsigned long>(cy.min_us),
           static_cast<unsigned long>(cy.p50_us),
           static_cast<unsigned long>(cy.p99_us),
           static_cast<unsigned long>(cy.max_us));
  ESP_LOGI(TAG, "  cmd2start n=%lu min/p50/p99/max=%lu/%lu/%lu/%lu us",
           static_cast<unsigned long>(cs.count),
           static_cast<unsigned long>(cs.min_us),
           static_cast<unsigned long>(cs.p50_us),
           static_cast<unsigned long>(cs.p99_us),
           static_cast<unsigned long>(cs.max_us));
  ESP_LOGI(TAG, "  pace      overrun=%lu yield=%lu",
           static_cast<unsigned long>(class1TimingStats().paceOverrunCount()),
           static_cast<unsigned long>(class1TimingStats().paceYieldCount()));
  const ReliabilitySnapshot rs = reliabilityStats().snapshot();
  ESP_LOGI(TAG,
           "  reliability soft_miss=%lu sendok_fail=%lu chip_recover=%lu "
           "reconnect=%lu",
           static_cast<unsigned long>(rs.soft_miss),
           static_cast<unsigned long>(rs.sendok_fail),
           static_cast<unsigned long>(rs.chip_recover),
           static_cast<unsigned long>(rs.reconnect));
}

void startScannerTask(W5500& chip, w5500::W5500Hal& hal, ILinkStatus& link,
                      EipProcessImage* image_x, EipProcessImage* image_z,
                      EipProcessImage* image_theta) {
  static ScannerTaskCtx ctx;
  ctx.chip = &chip;
  ctx.hal = &hal;
  ctx.link = &link;
  ctx.image_x = image_x;
  ctx.image_z = image_z;
  ctx.image_theta = image_theta;

  if (xTaskCreatePinnedToCore(class1LogTask, "EipLog", kLogTaskStack, nullptr,
                              kLogTaskPriority, nullptr, kLogTaskCore) != pdPASS) {
    ESP_LOGE(TAG, "Failed to create EipLog task (Class 1 events will not print)");
  }

#if defined(CONFIG_EIP_AXIS_X) || defined(CONFIG_EIP_AXIS_Z)
  BaseType_t ok_multi =
      xTaskCreatePinnedToCore(multiAxisTask, "EipScannerM", kScannerStack, &ctx,
                              kScannerPriority, nullptr, kScannerCore);
  if (ok_multi != pdPASS) {
    ESP_LOGE(TAG, "Failed to create EipScanner task for X/Z");
  } else {
    ESP_LOGI(TAG, "EipScanner task started (multi, core %d)",
             static_cast<int>(kScannerCore));
  }
#elif defined(CONFIG_EIP_AXIS_THETA)
  BaseType_t ok_theta =
      xTaskCreatePinnedToCore(singleAxisTask, "EipScannerT", kScannerStack, &ctx,
                              kScannerPriority, nullptr, kScannerCore);
  if (ok_theta != pdPASS) {
    ESP_LOGE(TAG, "Failed to create EipScanner task for Theta");
  } else {
    ESP_LOGI(TAG, "EipScanner task started (theta %s, core %d)",
             CONFIG_EIP_TARGET_IP_THETA, static_cast<int>(kScannerCore));
  }
#endif
}

}  // namespace eip

#else  // !CONFIG_EIP_SCANNER_ENABLED

namespace eip {
void startScannerTask(W5500&, w5500::W5500Hal&, ILinkStatus&, EipProcessImage*,
                      EipProcessImage*, EipProcessImage*) {}
void dumpClass1TimingStats() {}
void printLastClass1GapReport() {}
void setScopeMarkerGpio(int) {}
bool pauseClass1(uint32_t) { return true; }
void resumeClass1() {}
}  // namespace eip

#endif

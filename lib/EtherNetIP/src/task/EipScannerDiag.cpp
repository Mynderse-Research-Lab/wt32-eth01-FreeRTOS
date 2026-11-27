/**
 * @file EipScannerDiag.cpp
 * @brief Class 1 gap report, run-time trace, log task and snapshot trickle.
 */

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
#include "EipScannerTaskInternal.h"

namespace eip {

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

// The last report, kept for 'gaptrace': a log burst (cfg export) can make the
// net console drop the line, as it did on 2026-11-03.
std::mutex g_rt_report_mutex;
char g_rt_report[400] = "none since boot";
uint32_t g_rt_report_count = 0;
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


void class1LogTask(void*) {
    uint32_t ticks = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(kLogTaskPeriodMs));
        (void)w5500::drainDeferredLogs();
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

}  // namespace eip

#endif  // CONFIG_EIP_SCANNER_ENABLED

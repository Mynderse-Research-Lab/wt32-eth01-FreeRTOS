#include "EipScannerTask.h"

#include "sdkconfig.h"

#if CONFIG_EIP_SCANNER_ENABLED

#include "EipClass1Timing.h"
#include "EipClass1TimingStats.h"
#include "EipReliabilityStats.h"
#include "EipMultiScanner.h"
#include "EipProcessImage.h"
#include "EipScanner.h"
#include "EipScannerPolicy.h"
#include "EipSocketW5500.h"
#include "Hcs01Assembly.h"
#include "W5500.h"
#include "Spi3Bus.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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
static constexpr uint32_t kReconnectIdleMs = 2500;
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
  volatile bool run;
  volatile bool failed;
  uint32_t period_ms;
};

// Tick-aligned Class 1 cadence. xTaskDelayUntil always blocks when on time so
// IDLE1 can feed the task WDT. Sub-ms RPI fraction (if any) is spun only after
// that block. On overrun, catch up without reseeding last_wake; yield one tick
// every N consecutive overruns (TWDT safety if exchange > RPI).
void paceClass1RemainderUs(uint32_t rem_us) {
  if (rem_us == 0) return;
  // Fractional top-up only (< 1 FreeRTOS tick). DelayUntil already blocked.
  const int64_t deadline = esp_timer_get_time() + static_cast<int64_t>(rem_us);
  while (esp_timer_get_time() < deadline) {
  }
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
  while (ctx->run) {
    if (!ctx->scanner->sendKeepaliveAll()) {
      ctx->failed = true;
    }
    vTaskDelay(pdMS_TO_TICKS(ctx->period_ms > 0 ? ctx->period_ms : 5));
  }
  vTaskDelete(nullptr);
}

// Chip recover after disconnect. After kMaxChipRecovers consecutive recovers
// that still cannot sustain Class 1, hard-reset the ESP (matches bench recovery).
void escalateChipRecover(ScannerTaskCtx* ctx, unsigned& recover_streak,
                         uint32_t& backoff_ms) {
  if (chipRecoverOnFailure(recover_streak) == ChipRecoverDecision::kRestart) {
    ESP_LOGE(TAG,
             "W5500 recover exhausted (%u) — esp_restart()",
             recover_streak);
    esp_restart();
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

void singleAxisTask(void* arg) {
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

void multiAxisTask(void* arg) {
  auto* ctx = static_cast<ScannerTaskCtx*>(arg);
  uint32_t backoff_ms = 1000;
  unsigned recover_streak = 0;

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
    while (!ctx->link->isUp()) {
      backoff_ms = 1000;
      vTaskDelay(pdMS_TO_TICKS(kLinkPollMs));
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
      xTaskCreatePinnedToCore(keepaliveTask, "EipHoldKA", kKeepaliveStack, &ka,
                              kKeepalivePriority, &ka_handle, kScannerCore);
      for (size_t i = 1; i < n; ++i) {
        const bool opened = scanner.openAxis(i);
        if (opened) {
          continue;
        }
        if (slots[i].config.drive_family ==
            ScannerConfig::DriveFamily::kHcs01) {
          ESP_LOGW(TAG,
                   "Theta ForwardOpen failed — X/Z stay up; check HCS01 "
                   "map/FO sizes");
          continue;
        }
        ok = false;
        break;
      }
      ka.run = false;
      vTaskDelay(pdMS_TO_TICKS(20));
      if (ka.failed) {
        ESP_LOGW(TAG, "HoldKA O->T send failed during peer FO");
        ok = false;
      }
    }

    // Prime UDP dest/ARP for every axis and empty RX before cyclic start.
    if (ok) {
      for (int prime = 0; prime < 4; ++prime) {
        (void)scanner.sendKeepaliveAll();
        (void)scanner.drainBufferedInputs();
        vTaskDelay(pdMS_TO_TICKS(2));
      }
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
    g_class1_budget_rpi_us = rpi_us;
    class1TimingStats().reset();
    // Seed feedback clocks at cyclic start — FO timestamps must not count.
    scanner.beginCyclicExchange();

    uint32_t soft_miss_last_warn_ms = 0;
    uint32_t input_miss_streak = 0;
    bool need_chip_recover = false;
    uint32_t timing_cycles = 0;
    int64_t cycle_anchor_us = class1NowUs();
    TickType_t last_wake = xTaskGetTickCount();

    while (ctx->link->isUp()) {
      const int64_t t0 = class1NowUs();
      spi3_class1_critical_enter();
      (void)ctx->chip->acquireBus();
      const ExchangeStatus st = scanner.exchangeOnce(0);
      ctx->chip->releaseBus();
      spi3_class1_critical_exit();
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

      if (st == ExchangeStatus::kOutputSendFailed) {
        reliabilityStats().noteSendOkFail();
        ESP_LOGW(TAG, "O->T send failed — reconnect + chip recover");
        need_chip_recover = true;
        break;
      }
      if (st == ExchangeStatus::kInputMiss) {
        reliabilityStats().noteSoftMiss();
        ++input_miss_streak;
        const uint32_t now_ms =
            static_cast<uint32_t>(esp_timer_get_time() / 1000LL);
        if (shouldWarnSoftMiss(soft_miss_last_warn_ms, now_ms)) {
          ESP_LOGW(TAG, "T->O soft-miss #%lu (stale beyond 3x RPI after cyclic)",
                   static_cast<unsigned long>(input_miss_streak));
        }
        // Keep pumping O->T for a few cycles before dual teardown + recover.
        if (shouldTeardownAfterInputMisses(input_miss_streak)) {
          ESP_LOGW(TAG, "T->O stale beyond soft-retry — reconnect + chip recover");
          need_chip_recover = true;
          break;
        }
        paceClass1Cycle(last_wake, rpi_us);
        continue;
      }
      input_miss_streak = 0;

      logTimingSnapshotPeriodic(timing_cycles);
      paceClass1Cycle(last_wake, rpi_us);
    }

    scanner.disconnect();
    reliabilityStats().noteReconnect();
    vTaskDelay(pdMS_TO_TICKS(kReconnectIdleMs));
    if (need_chip_recover) {
      escalateChipRecover(ctx, recover_streak, backoff_ms);
    }
  }
}

}  // namespace

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
}  // namespace eip

#endif

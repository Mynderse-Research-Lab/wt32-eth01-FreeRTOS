#include "EipScannerTask.h"

#include "sdkconfig.h"
#include "EipScannerTaskInternal.h"

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

namespace {


// Above gantry update (5) / pick (4) / console (1) so Class 1 preempts SPI3 users.
// Pin to Core 1 so Core 0 IDLE / UART / NetConsole / SPI3 are not starved (TWDT).

// Last Class 1 RPI used for pacing (granted API or CONFIG fallback).


// Tick-aligned Class 1 cadence. xTaskDelayUntil always blocks when on time so
// IDLE1 can feed the task WDT. Sub-ms RPI fraction (if any) is paced after
// that block. On a single-period overrun, catch up on the period grid; on a
// multi-period stall or after yielding a tick every N consecutive overruns,
// reseed last_wake to now so tick debt cannot lock Core 1 in nonstop catch-up.

// Dropout diagnostics (W5500Diag): read-only, nothing here recovers anything.




// Class 1 stall diagnostics: on 2026-10-20 the stale ages showed the multi
// task skipping 10-40 ms between exchanges. Track the worst loop gap (split
// into the previous iteration's exchange / log flush / rest) and the W5500
// bus mutex's longest hold and wait. The Class 1 task only records; the
// class1LogTask (core 0) prints, so no ESP_LOG or console TCP send runs in
// the cyclic loop.











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
        (void)w5500::deferLog(TAG, 'W', esp_log_timestamp(),
                              "W5500 LNK=0 on %lu read(s), then up — not a link loss",
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
          // Queued: the loop must keep pumping O->T to recover (G-B13).
          (void)w5500::deferLog(TAG, 'W', esp_log_timestamp(),
                                "T->O soft-miss #%lu (stale beyond %lux RPI after cyclic)",
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

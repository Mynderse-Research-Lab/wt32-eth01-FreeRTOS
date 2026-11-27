/**
 * @file EipScannerConfig.cpp
 * @brief Kinetix and theta scanner configuration, and the theta-only task.
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

}  // namespace eip

#endif  // CONFIG_EIP_SCANNER_ENABLED

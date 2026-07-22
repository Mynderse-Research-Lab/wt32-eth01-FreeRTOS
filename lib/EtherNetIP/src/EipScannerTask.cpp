#include "EipScannerTask.h"

#include "sdkconfig.h"

#if CONFIG_EIP_SCANNER_ENABLED

#include "EipClass1Timing.h"
#include "EipMultiScanner.h"
#include "EipProcessImage.h"
#include "EipScanner.h"
#include "EipSocketW5500.h"
#include "Hcs01Assembly.h"
#include "W5500.h"

#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace eip {

namespace {

static const char* TAG = "EipScanner";

static constexpr UBaseType_t kScannerPriority = 3;
static constexpr UBaseType_t kKeepalivePriority = 4;
static constexpr uint32_t kScannerStack = 8192;
static constexpr uint32_t kKeepaliveStack = 4096;
static constexpr uint32_t kLinkPollMs = 500;
static constexpr uint32_t kLinkSettleMs = 300;
static constexpr uint32_t kReconnectIdleMs = 2500;
static constexpr uint32_t kPostResetSettleMs = 100;
static constexpr unsigned kMaxChipRecovers = 3;

struct ScannerTaskCtx {
  W5500* chip;
  w5500::W5500Hal* hal;
  ILinkStatus* link;
  EipProcessImage* image_x;
  EipProcessImage* image_z;
};

struct KeepaliveCtx {
  EipMultiScanner* scanner;
  volatile bool run;
  volatile bool failed;
  uint32_t period_ms;
};

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
  ++recover_streak;
  if (recover_streak > kMaxChipRecovers) {
    ESP_LOGE(TAG,
             "W5500 recover exhausted (%u) — esp_restart()",
             recover_streak);
    esp_restart();
  }

  if (ctx->chip != nullptr && ctx->chip->recover()) {
    ESP_LOGW(TAG, "W5500 recover #%u OK — settling link", recover_streak);
    backoff_ms = 1000;
    vTaskDelay(pdMS_TO_TICKS(kLinkSettleMs + kPostResetSettleMs));
  } else {
    ESP_LOGW(TAG, "W5500 recover #%u failed — backoff %lu ms", recover_streak,
             static_cast<unsigned long>(backoff_ms));
    vTaskDelay(pdMS_TO_TICKS(backoff_ms));
    backoff_ms = (backoff_ms < 30000) ? backoff_ms * 2 : 30000;
  }
}

ScannerConfig makeKinetixConfig(const char* ip, uint16_t connection_serial,
                                uint32_t ot_connection_id) {
  ScannerConfig cfg;
  cfg.target_ip = ip;
  cfg.drive_family = ScannerConfig::DriveFamily::kKinetix5100;
  cfg.config_assembly_instance = CONFIG_EIP_CONFIG_ASSEMBLY_INSTANCE;
  cfg.ot_assembly_instance = 104;
  cfg.to_assembly_instance = 154;
  cfg.ot_assembly_size = 40;
  cfg.to_assembly_size = 52;
  cfg.ot_rpi_us = CONFIG_EIP_X_RPI_US;
  cfg.to_rpi_us = CONFIG_EIP_X_RPI_US;
  cfg.ot_connection_id = ot_connection_id;
  // Non-zero T->O CID so demux matches sequenced-address IDs in T->O frames
  // (Kinetix FO reply often echoes this; 0 forces a fragile O->T fallback).
  cfg.to_connection_id = 0x20000000u | static_cast<uint32_t>(connection_serial);
  cfg.connection_serial = connection_serial;
  cfg.originator_vendor_id = CONFIG_EIP_ORIGINATOR_VENDOR_ID;
  // Generous timeout: FO of the second axis must not starve the first.
  cfg.connection_timeout_multiplier = 7;
  cfg.include_run_idle_header = true;
  cfg.include_run_idle_bit_in_net_params = false;
  cfg.to_connection_type = ConnectionType::kPointToPoint;
#if defined(CONFIG_EIP_TO_POINT_TO_POINT) && !CONFIG_EIP_TO_POINT_TO_POINT
  cfg.to_connection_type = ConnectionType::kMulticast;
#endif
  return cfg;
}

#if defined(CONFIG_EIP_AXIS_THETA)
ScannerConfig makeThetaConfig() {
  ScannerConfig cfg;
  cfg.target_ip = CONFIG_EIP_TARGET_IP;
  cfg.drive_family = ScannerConfig::DriveFamily::kHcs01;
  cfg.config_assembly_instance = CONFIG_EIP_CONFIG_ASSEMBLY_INSTANCE;
  cfg.ot_assembly_instance = 101;
  cfg.to_assembly_instance = 102;
  cfg.ot_assembly_size = sizeof(hcs01::Hcs01PositioningCommand);
  cfg.to_assembly_size = sizeof(hcs01::Hcs01PositioningActual);
  cfg.ot_rpi_us = CONFIG_EIP_THETA_RPI_US;
  cfg.to_rpi_us = CONFIG_EIP_THETA_RPI_US;
  cfg.originator_vendor_id = CONFIG_EIP_ORIGINATOR_VENDOR_ID;
  cfg.connection_timeout_multiplier = 7;
  cfg.include_run_idle_header = true;
  cfg.include_run_idle_bit_in_net_params = true;
  return cfg;
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
    if (ctx->image_x) scanner.setProcessImage(ctx->image_x);

    if (!scanner.connect()) {
      ESP_LOGW(TAG, "Theta connect failed — chip recover");
      scanner.disconnect();
      escalateChipRecover(ctx, recover_streak, backoff_ms);
      continue;
    }
    recover_streak = 0;
    backoff_ms = 1000;

    const uint32_t rpi_ms =
        (scanner.openReply().to_api_us > 0)
            ? scanner.openReply().to_api_us / 1000
            : 20;
    const uint32_t recv_timeout_ms = scanner.recvTimeoutMs();
    uint32_t input_miss_streak = 0;
    bool need_chip_recover = false;

    while (ctx->link->isUp()) {
      const auto t0 = xTaskGetTickCount();
      Bytes input;
      if (!scanner.exchangeOnce(recv_timeout_ms, input)) {
        ++input_miss_streak;
        ESP_LOGW(TAG, "Theta exchange miss #%lu",
                 static_cast<unsigned long>(input_miss_streak));
        if (shouldTeardownAfterInputMisses(input_miss_streak)) {
          need_chip_recover = true;
          break;
        }
        continue;
      }
      input_miss_streak = 0;
      const uint32_t elapsed_ms =
          (xTaskGetTickCount() - t0) * portTICK_PERIOD_MS;
      const uint32_t rem = rpiRemainderMs(rpi_ms, elapsed_ms);
      if (rem > 0) vTaskDelay(pdMS_TO_TICKS(rem));
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

  if (n == 0) {
    ESP_LOGW(TAG, "No X/Z axes enabled; scanner idle");
    vTaskDelete(nullptr);
    return;
  }

  ESP_LOGI(TAG, "Multi-axis scanner: %u Kinetix slot(s)",
           static_cast<unsigned>(n));

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
    EipSocketW5500Udp udp(*ctx->hal);
    ITcpClient* tcps[EipMultiScanner::kMaxAxes] = {&tcp0, &tcp1};

    EipMultiScanner scanner(tcps, n, udp, slots);

    // Stage connect so keepalive can run during the second ForwardOpen.
    bool ok = scanner.openAxis(0);
    if (ok) ok = scanner.bindSharedUdp();

    KeepaliveCtx ka{&scanner, false, false, 5};
    TaskHandle_t ka_handle = nullptr;
    if (ok && n > 1) {
      ka.run = true;
      xTaskCreate(keepaliveTask, "EipHoldKA", kKeepaliveStack, &ka,
                  kKeepalivePriority, &ka_handle);
      ok = scanner.openAxis(1);
      ka.run = false;
      vTaskDelay(pdMS_TO_TICKS(20));
      if (ka.failed) {
        ESP_LOGW(TAG, "HoldKA O->T send failed during second FO");
        ok = false;
      }
    }

    if (!ok) {
      ESP_LOGW(TAG, "Multi-axis connect failed — chip recover");
      scanner.disconnect();
      escalateChipRecover(ctx, recover_streak, backoff_ms);
      continue;
    }

    recover_streak = 0;
    backoff_ms = 1000;
    const uint32_t rpi_ms =
        (scanner.openReply(0).to_api_us > 0)
            ? scanner.openReply(0).to_api_us / 1000
            : (CONFIG_EIP_X_RPI_US / 1000);
    const uint32_t recv_timeout_ms = scanner.recvTimeoutMs();

    ESP_LOGI(TAG, "Multi-axis Class 1 running (RPI=%lu ms)",
             static_cast<unsigned long>(rpi_ms));

    uint32_t input_miss_streak = 0;
    bool need_chip_recover = false;

    while (ctx->link->isUp()) {
      const TickType_t t0 = xTaskGetTickCount();
      const ExchangeStatus st = scanner.exchangeOnce(recv_timeout_ms);
      if (st == ExchangeStatus::kOutputSendFailed) {
        ESP_LOGW(TAG, "O->T send failed — reconnect + chip recover");
        need_chip_recover = true;
        break;
      }
      if (st == ExchangeStatus::kInputMiss) {
        ++input_miss_streak;
        ESP_LOGW(TAG, "T->O miss #%lu (soft-retry)",
                 static_cast<unsigned long>(input_miss_streak));
        if (shouldTeardownAfterInputMisses(input_miss_streak)) {
          ESP_LOGW(TAG, "T->O miss streak exceeded — reconnect + chip recover");
          need_chip_recover = true;
          break;
        }
      } else {
        input_miss_streak = 0;
      }

      const uint32_t elapsed_ms =
          (xTaskGetTickCount() - t0) * portTICK_PERIOD_MS;
      const uint32_t rem = rpiRemainderMs(rpi_ms, elapsed_ms);
      if (rem > 0) {
        vTaskDelay(pdMS_TO_TICKS(rem));
      }
    }

    scanner.disconnect();
    vTaskDelay(pdMS_TO_TICKS(kReconnectIdleMs));
    if (need_chip_recover) {
      escalateChipRecover(ctx, recover_streak, backoff_ms);
    }
  }
}

}  // namespace

void startScannerTask(W5500& chip, w5500::W5500Hal& hal, ILinkStatus& link,
                      EipProcessImage* image_x, EipProcessImage* image_z) {
  static ScannerTaskCtx ctx;
  ctx.chip = &chip;
  ctx.hal = &hal;
  ctx.link = &link;
  ctx.image_x = image_x;
  ctx.image_z = image_z;

#if defined(CONFIG_EIP_AXIS_THETA)
  BaseType_t ok =
      xTaskCreate(singleAxisTask, "EipScanner", kScannerStack, &ctx,
                  kScannerPriority, nullptr);
  if (ok != pdPASS) {
    ESP_LOGE(TAG, "Failed to create EipScanner task");
  } else {
    ESP_LOGI(TAG, "EipScanner task started (theta %s)", CONFIG_EIP_TARGET_IP);
  }
#else
  BaseType_t ok =
      xTaskCreate(multiAxisTask, "EipScanner", kScannerStack, &ctx,
                  kScannerPriority, nullptr);
  if (ok != pdPASS) {
    ESP_LOGE(TAG, "Failed to create EipScanner task");
  } else {
    ESP_LOGI(TAG, "EipScanner task started (X/Z multi)");
  }
#endif
}

}  // namespace eip

#else  // !CONFIG_EIP_SCANNER_ENABLED

namespace eip {
void startScannerTask(W5500&, w5500::W5500Hal&, ILinkStatus&, EipProcessImage*,
                      EipProcessImage*) {}
}  // namespace eip

#endif

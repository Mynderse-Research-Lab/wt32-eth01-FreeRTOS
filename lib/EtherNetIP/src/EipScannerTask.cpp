#include "EipScannerTask.h"

#include "sdkconfig.h"

#if CONFIG_EIP_SCANNER_ENABLED

#include "EipScanner.h"
#include "EipProcessImage.h"
#include "EipSocketEspIdf.h"

#ifdef CONFIG_EIP_AXIS_THETA
#include "Hcs01Assembly.h"
#endif

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace eip {

namespace {

static const char* TAG = "EipScanner";

// Priority below GantryUpdate (5) and PickScheduler (4); above console (1).
static constexpr UBaseType_t kScannerPriority = 3;
static constexpr uint32_t kScannerStack = 6144;
static constexpr uint32_t kLinkPollMs = 500;
static constexpr uint32_t kReconnectIdleMs = 5000;

struct ScannerTaskCtx {
  ILinkStatus* link;
  EipProcessImage* image;
};

ScannerConfig makeDefaultConfig() {
  ScannerConfig cfg;
  cfg.target_ip = CONFIG_EIP_TARGET_IP;

  // Kconfig-driven provisional values (override in menuconfig when EDS is available).
  cfg.config_assembly_instance = CONFIG_EIP_CONFIG_ASSEMBLY_INSTANCE;
  cfg.include_run_idle_header = IS_ENABLED(CONFIG_EIP_INCLUDE_RUN_IDLE_HEADER);
  cfg.originator_vendor_id = CONFIG_EIP_ORIGINATOR_VENDOR_ID;

#ifdef CONFIG_EIP_AXIS_THETA
  // HCS01: assemblies 101 (command) / 102 (actual)
  cfg.drive_family = ScannerConfig::DriveFamily::kHcs01;
  cfg.ot_assembly_instance = 101;
  cfg.to_assembly_instance = 102;
  cfg.ot_assembly_size = sizeof(hcs01::Hcs01PositioningCommand);
  cfg.to_assembly_size = sizeof(hcs01::Hcs01PositioningActual);
  cfg.ot_rpi_us = CONFIG_EIP_THETA_RPI_US;
  cfg.to_rpi_us = CONFIG_EIP_THETA_RPI_US;
#elif defined(CONFIG_EIP_AXIS_X) || defined(CONFIG_EIP_AXIS_Z)
  // Kinetix 5100: defaults already set (104 / 154).
  cfg.ot_rpi_us = CONFIG_EIP_X_RPI_US;
  cfg.to_rpi_us = CONFIG_EIP_X_RPI_US;
#endif

  return cfg;
}

void scannerTask(void* arg) {
  auto* ctx = static_cast<ScannerTaskCtx*>(arg);
  auto* link = ctx->link;
  auto* image = ctx->image;
  uint32_t backoff_ms = 1000;

  while (true) {
    while (!link->isUp()) {
      vTaskDelay(pdMS_TO_TICKS(kLinkPollMs));
    }

    EipSocketTcpClient tcp;
    EipSocketUdpEndpoint udp;
    EipScanner scanner(tcp, udp, makeDefaultConfig());

    if (image) {
      scanner.setProcessImage(image);
    }

    if (!scanner.connect()) {
      ESP_LOGW(TAG, "Connect to %s failed, retry in %lu ms",
               CONFIG_EIP_TARGET_IP,
               static_cast<unsigned long>(backoff_ms));
      scanner.disconnect();
      vTaskDelay(pdMS_TO_TICKS(backoff_ms));
      backoff_ms = (backoff_ms < 30000) ? backoff_ms * 2 : 30000;
      continue;
    }

    backoff_ms = 1000;
    ESP_LOGI(TAG, "ForwardOpen ok O->T=0x%08lx T->O=0x%08lx API=%lu us",
             static_cast<unsigned long>(scanner.openReply().ot_connection_id),
             static_cast<unsigned long>(scanner.openReply().to_connection_id),
             static_cast<unsigned long>(scanner.openReply().to_api_us));

    const uint32_t rpi_ms =
        (scanner.openReply().to_api_us > 0)
            ? scanner.openReply().to_api_us / 1000
            : 20;
    const uint32_t recv_timeout_ms = scanner.recvTimeoutMs();

    while (link->isUp()) {
      Bytes input;
      if (!scanner.exchangeOnce(recv_timeout_ms, input)) {
        ESP_LOGW(TAG, "T->O timeout or IO error, reconnecting");
        break;
      }
      vTaskDelay(pdMS_TO_TICKS(rpi_ms > 0 ? rpi_ms : 1));
    }

    scanner.disconnect();
    vTaskDelay(pdMS_TO_TICKS(kReconnectIdleMs));
  }
}

}  // namespace

void startScannerTask(ILinkStatus& link, EipProcessImage* image) {
  static ScannerTaskCtx ctx;
  ctx.link  = &link;
  ctx.image = image;

  BaseType_t ok =
      xTaskCreate(scannerTask, "EipScanner", kScannerStack, &ctx,
                  kScannerPriority, nullptr);
  if (ok != pdPASS) {
    ESP_LOGE(TAG, "Failed to create EipScanner task");
  } else {
    ESP_LOGI(TAG, "EipScanner task started (target %s)", CONFIG_EIP_TARGET_IP);
  }
}

}  // namespace eip

#else  // !CONFIG_EIP_SCANNER_ENABLED

namespace eip {
void startScannerTask(ILinkStatus&, EipProcessImage*) {}
}  // namespace eip

#endif

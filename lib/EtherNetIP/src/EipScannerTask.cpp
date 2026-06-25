#include "EipScannerTask.h"

#include "sdkconfig.h"

#if CONFIG_EIP_SCANNER_ENABLED

#include "EipSession.h"
#include "EipSocketEspIdf.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace eip {

namespace {

static const char* TAG = "EipScanner";

// Priority below GantryUpdate (5) and PickScheduler (4); above console (1).
static constexpr UBaseType_t kScannerPriority = 3;
static constexpr uint32_t kScannerStack = 4096;

void scannerTask(void* /*arg*/) {
  uint32_t backoff_ms = 1000;
  while (true) {
    EipSocketTcpClient tcp;
    if (!tcp.connect(CONFIG_EIP_TARGET_IP, EipSession::kDefaultPort)) {
      ESP_LOGW(TAG, "TCP connect to %s failed, retry in %lu ms",
               CONFIG_EIP_TARGET_IP,
               static_cast<unsigned long>(backoff_ms));
      vTaskDelay(pdMS_TO_TICKS(backoff_ms));
      backoff_ms = (backoff_ms < 30000) ? backoff_ms * 2 : 30000;
      continue;
    }
    backoff_ms = 1000;

    EipSession session(tcp);
    if (!session.registerSession()) {
      ESP_LOGW(TAG, "RegisterSession failed");
      tcp.close();
      vTaskDelay(pdMS_TO_TICKS(backoff_ms));
      continue;
    }

    ESP_LOGI(TAG, "RegisterSession ok, handle=0x%08lx",
             static_cast<unsigned long>(session.sessionHandle()));

    // Scaffold: hold session briefly, then close. ForwardOpen deferred.
    vTaskDelay(pdMS_TO_TICKS(1000));
    session.unregisterSession();
    tcp.close();
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

}  // namespace

void startScannerTask() {
  BaseType_t ok = xTaskCreate(scannerTask, "EipScanner", kScannerStack, nullptr,
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
void startScannerTask() {}
}  // namespace eip

#endif

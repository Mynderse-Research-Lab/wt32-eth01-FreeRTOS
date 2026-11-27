/**
 * @file gantry_ota.cpp
 * @brief Ethernet OTA firmware update manager implementation.
 */

#include "gantry_ota.h"

#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"

#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstring>

static const char *TAG = "GantryOTA";

esp_err_t gantryOtaGetStatus(GantryOtaStatus &status) {
  const esp_partition_t *running = esp_ota_get_running_partition();
  if (running == nullptr) {
    ESP_LOGE(TAG, "Failed to get running partition");
    return ESP_FAIL;
  }

  strncpy(status.running_partition, running->label, sizeof(status.running_partition) - 1);
  status.running_address = running->address;
  status.running_size = running->size;

  const esp_partition_t *next = esp_ota_get_next_update_partition(running);
  if (next != nullptr) {
    strncpy(status.next_partition, next->label, sizeof(status.next_partition) - 1);
    status.next_address = next->address;
    status.next_size = next->size;
  } else {
    strncpy(status.next_partition, "none", sizeof(status.next_partition) - 1);
  }

  esp_app_desc_t app_desc;
  esp_err_t err = esp_ota_get_partition_description(running, &app_desc);
  if (err == ESP_OK) {
    strncpy(status.app_version, app_desc.version, sizeof(status.app_version) - 1);
    strncpy(status.project_name, app_desc.project_name, sizeof(status.project_name) - 1);
    strncpy(status.compile_date, app_desc.date, sizeof(status.compile_date) - 1);
    strncpy(status.compile_time, app_desc.time, sizeof(status.compile_time) - 1);
    strncpy(status.idf_version, app_desc.idf_ver, sizeof(status.idf_version) - 1);
  }

  esp_ota_get_state_partition(running, &status.state);
  status.rollback_possible = esp_ota_check_rollback_is_possible();

  return ESP_OK;
}

const char *gantryResetReasonName() {
    switch (esp_reset_reason()) {
        case ESP_RST_POWERON:
            return "POWERON (power applied)";
        case ESP_RST_EXT:
            return "EXT (reset pin)";
        case ESP_RST_SW:
            return "SW (esp_restart / OTA)";
        case ESP_RST_PANIC:
            return "PANIC (crash: exception or abort)";
        case ESP_RST_INT_WDT:
            return "INT_WDT (interrupt watchdog)";
        case ESP_RST_TASK_WDT:
            return "TASK_WDT (task watchdog)";
        case ESP_RST_WDT:
            return "WDT (other watchdog)";
        case ESP_RST_DEEPSLEEP:
            return "DEEPSLEEP";
        case ESP_RST_BROWNOUT:
            return "BROWNOUT (supply dipped)";
        case ESP_RST_SDIO:
            return "SDIO";
        case ESP_RST_UNKNOWN:
            return "UNKNOWN";
        default:
            return "OTHER";
    }
}

void gantryOtaPrintStatus() {
  GantryOtaStatus status;
  if (gantryOtaGetStatus(status) != ESP_OK) {
    ESP_LOGE(TAG, "Could not read OTA partition status");
    return;
  }

  ESP_LOGI(TAG, "============================================================");
  ESP_LOGI(TAG, "=== GANTRY FIRMWARE & DUAL-OTA STATUS                    ===");
  ESP_LOGI(TAG, "============================================================");
  ESP_LOGI(TAG, "App Project      : %s (v%s)", status.project_name, status.app_version);
  ESP_LOGI(TAG, "Compiled         : %s %s (IDF %s)", status.compile_date,
           status.compile_time, status.idf_version);
  ESP_LOGI(TAG, "Running Slot     : %s (0x%06lX, size %lu KB)",
           status.running_partition, (unsigned long)status.running_address,
           (unsigned long)(status.running_size / 1024));
  ESP_LOGI(TAG, "Next Update Slot : %s (0x%06lX, size %lu KB)",
           status.next_partition, (unsigned long)status.next_address,
           (unsigned long)(status.next_size / 1024));

  const char *state_str = "UNDEFINED";
  switch (status.state) {
    case ESP_OTA_IMG_NEW:
      state_str = "NEW (first boot after OTA)";
      break;
    case ESP_OTA_IMG_PENDING_VERIFY:
      state_str = "PENDING_VERIFY (evaluating stability)";
      break;
    case ESP_OTA_IMG_VALID:
      state_str = "VALID (confirmed stable)";
      break;
    case ESP_OTA_IMG_INVALID:
      state_str = "INVALID";
      break;
    case ESP_OTA_IMG_ABORTED:
      state_str = "ABORTED";
      break;
    default:
      state_str = "UNDEFINED";
      break;
  }
  ESP_LOGI(TAG, "Image State      : %s", state_str);
  ESP_LOGI(TAG, "Rollback Possible: %s", status.rollback_possible ? "YES" : "NO");
  ESP_LOGI(TAG, "Last Reset       : %s (reason %d), uptime %lu s", gantryResetReasonName(),
           static_cast<int>(esp_reset_reason()),
           static_cast<unsigned long>(esp_timer_get_time() / 1000000LL));
  ESP_LOGI(TAG, "============================================================");
}

esp_err_t gantryOtaConfirmBootValid() {
  esp_ota_img_states_t state;
  const esp_partition_t *running = esp_ota_get_running_partition();
  if (running == nullptr) {
    return ESP_FAIL;
  }

  if (esp_ota_get_state_partition(running, &state) == ESP_OK) {
    if (state == ESP_OTA_IMG_PENDING_VERIFY || state == ESP_OTA_IMG_NEW) {
      ESP_LOGI(TAG, "Confirming new firmware image as VALID and cancelling rollback...");
      esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
      if (err == ESP_OK) {
        ESP_LOGI(TAG, "Firmware marked VALID successfully.");
      } else {
        ESP_LOGW(TAG, "esp_ota_mark_app_valid_cancel_rollback returned %d", (int)err);
      }
      return err;
    }
  }
  return ESP_OK;
}

esp_err_t gantryOtaDisableMotors(Gantry::Gantry *gantry) {
    if (gantry == nullptr || !gantry->isEnabled()) {
        return ESP_OK;
    }
    ESP_LOGW(TAG, "OTA: disabling motors for the update (re-enable after reboot)");
    gantry->disable();
    if (gantry->isEnabled()) {
        ESP_LOGE(TAG, "OTA REFUSED: motors did not disable");
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}

esp_err_t gantryOtaStartStream(Gantry::Gantry *gantry, esp_ota_handle_t &handle,
                              const esp_partition_t *&target_part, size_t image_size) {
    // A move, home or pick in progress is not interrupted for an update.
    if (gantry != nullptr && gantry->isBusy()) {
        ESP_LOGE(TAG, "OTA REFUSED: Gantry is BUSY (move/home/pick). Stop it before flashing.");
        return ESP_ERR_INVALID_STATE;
    }
    const esp_err_t disable_err = gantryOtaDisableMotors(gantry);
    if (disable_err != ESP_OK) {
        return disable_err;
    }

  const esp_partition_t *running = esp_ota_get_running_partition();
  target_part = esp_ota_get_next_update_partition(running);
  if (target_part == nullptr) {
    ESP_LOGE(TAG, "No passive OTA update partition found in partition table");
    return ESP_ERR_NOT_FOUND;
  }

  ESP_LOGI(TAG, "Initiating OTA update stream into partition '%s' (0x%06lX)...",
           target_part->label, (unsigned long)target_part->address);

  // Erasing the announced range up front lets the flash driver use 64 KB block
  // erases instead of one 4 KB sector erase per write (N-2).
  esp_err_t err = esp_ota_begin(target_part,
                                image_size > 0 ? image_size : OTA_WITH_SEQUENTIAL_WRITES,
                                &handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_ota_begin failed: %s (0x%x)", esp_err_to_name(err), err);
    return err;
  }

  ESP_LOGI(TAG, "OTA stream opened successfully. Ready for incoming binary chunks.");
  return ESP_OK;
}

esp_err_t gantryOtaWriteStream(esp_ota_handle_t handle, const void *data, size_t length) {
  if (data == nullptr || length == 0) {
    return ESP_OK;
  }

  esp_err_t err = esp_ota_write(handle, data, length);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_ota_write failed at length %lu: %s (0x%x)",
             (unsigned long)length, esp_err_to_name(err), err);
  }
  return err;
}

esp_err_t gantryOtaFinishStream(esp_ota_handle_t handle, const esp_partition_t *target_part) {
  if (target_part == nullptr) {
    return ESP_ERR_INVALID_ARG;
  }

  esp_err_t err = esp_ota_end(handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_ota_end failed (image verification error): %s (0x%x)",
             esp_err_to_name(err), err);
    return err;
  }

  err = esp_ota_set_boot_partition(target_part);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s (0x%x)",
             esp_err_to_name(err), err);
    return err;
  }

  ESP_LOGI(TAG, "============================================================");
  ESP_LOGI(TAG, "=== OTA UPDATE COMPLETED SUCCESSFULLY!                   ===");
  ESP_LOGI(TAG, "=== Next boot partition set to: %s (0x%06lX)          ===",
           target_part->label, (unsigned long)target_part->address);
  ESP_LOGI(TAG, "============================================================");

  return ESP_OK;
}

// flashstall (N-2): measure how many frames the EMAC holds while a flash write
// stalls the receive task. "erase" clears the test region; "write" then does
// back-to-back 4 KB writes into it, like an OTA after its up-front erase. Two
// steps, so frames lost during the erase are not counted against the writes.
// The region is the passive OTA slot from 1.5 MB on; images today are about
// 0.85 MB, so the slot's resident image is untouched while images stay under
// 1.5 MB. Flash writes disable the cache on both cores, so the motors must be off.
esp_err_t gantryOtaFlashStallTest(Gantry::Gantry *gantry, bool erase, int writes, char *out,
                                  size_t out_len) {
  constexpr uint32_t kOffset = 0x180000;
  constexpr size_t kBlock = 4096;
  static uint8_t block[kBlock];  // .bss: DRAM, as the OTA block buffer must be
  if (gantry != nullptr && (gantry->isEnabled() || gantry->isBusy())) {
    snprintf(out, out_len, "refused: motors enabled or gantry busy ('stop' first)");
    return ESP_ERR_INVALID_STATE;
  }
  const esp_partition_t *part = esp_ota_get_next_update_partition(nullptr);
  if (part == nullptr || part->size <= kOffset) {
    snprintf(out, out_len, "refused: no passive OTA slot");
    return ESP_ERR_NOT_FOUND;
  }
  const int max_writes = static_cast<int>((part->size - kOffset) / kBlock);
  if (writes < 1 || writes > max_writes) {
    snprintf(out, out_len, "refused: n must be 1..%d", max_writes);
    return ESP_ERR_INVALID_ARG;
  }
  const size_t len = static_cast<size_t>(writes) * kBlock;
  if (erase) {
    const int64_t e0 = esp_timer_get_time();
    const esp_err_t err = esp_partition_erase_range(part, kOffset, len);
    const int64_t erase_us = esp_timer_get_time() - e0;
    if (err != ESP_OK) {
      snprintf(out, out_len, "erase failed: %s", esp_err_to_name(err));
      return err;
    }
    snprintf(out, out_len, "flashstall erase %s+0x%lX: %d KB in %lu ms", part->label,
             (unsigned long)kOffset, writes * 4, (unsigned long)(erase_us / 1000));
    return ESP_OK;
  }
  for (size_t i = 0; i < kBlock; ++i) {
    block[i] = static_cast<uint8_t>(i * 31u + 7u);
  }
  int64_t total_us = 0;
  int64_t min_us = INT64_MAX;
  int64_t max_us = 0;
  const int64_t w0 = esp_timer_get_time();
  for (int i = 0; i < writes; ++i) {
    const int64_t t = esp_timer_get_time();
    const esp_err_t err =
        esp_partition_write(part, kOffset + static_cast<uint32_t>(i) * kBlock, block, kBlock);
    const int64_t dt = esp_timer_get_time() - t;
    if (err != ESP_OK) {
      snprintf(out, out_len, "write %d failed: %s ('flashstall erase %d' first)", i,
               esp_err_to_name(err), writes);
      return err;
    }
    total_us += dt;
    min_us = dt < min_us ? dt : min_us;
    max_us = dt > max_us ? dt : max_us;
  }
  const int64_t span_us = esp_timer_get_time() - w0;
  snprintf(out, out_len,
           "flashstall write %s+0x%lX: %d writes of 4 KB in %lu ms (writing %lu ms, min %lu us, "
           "max %lu us)",
           part->label, (unsigned long)kOffset, writes, (unsigned long)(span_us / 1000),
           (unsigned long)(total_us / 1000), (unsigned long)min_us, (unsigned long)max_us);
  return ESP_OK;
}

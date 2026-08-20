/**
 * @file gantry_ota.cpp
 * @brief Ethernet OTA firmware update manager implementation.
 */

#include "gantry_ota.h"

#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"

#include <cerrno>
#include <cstdio>
#include <cstring>

static const char *TAG = "GantryOTA";

struct OtaServerCtx {
  Gantry::Gantry *gantry;
  int port;
  char password[64];
};

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

esp_err_t gantryOtaStartStream(Gantry::Gantry *gantry, esp_ota_handle_t &handle,
                              const esp_partition_t *&target_part) {
  if (gantry != nullptr && (gantry->isEnabled() || gantry->isBusy())) {
    ESP_LOGE(TAG, "OTA REFUSED: Gantry is ENABLED or BUSY! Disable drives before flashing.");
    return ESP_ERR_INVALID_STATE;
  }

  const esp_partition_t *running = esp_ota_get_running_partition();
  target_part = esp_ota_get_next_update_partition(running);
  if (target_part == nullptr) {
    ESP_LOGE(TAG, "No passive OTA update partition found in partition table");
    return ESP_ERR_NOT_FOUND;
  }

  ESP_LOGI(TAG, "Initiating OTA update stream into partition '%s' (0x%06lX)...",
           target_part->label, (unsigned long)target_part->address);

  esp_err_t err = esp_ota_begin(target_part, OTA_WITH_SEQUENTIAL_WRITES, &handle);
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

namespace {

static bool readLine(int fd, char *buf, size_t maxLen, int timeoutMs) {
  size_t idx = 0;
  struct timeval tv;
  tv.tv_sec = timeoutMs / 1000;
  tv.tv_usec = (timeoutMs % 1000) * 1000;
  setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  while (idx < maxLen - 1) {
    char c = 0;
    ssize_t res = recv(fd, &c, 1, 0);
    if (res <= 0) {
      return false;
    }
    if (c == '\n') {
      break;
    }
    if (c != '\r') {
      buf[idx++] = c;
    }
  }
  buf[idx] = '\0';
  return true;
}

static void sendStr(int fd, const char *str) {
  if (fd >= 0 && str != nullptr) {
    send(fd, str, strlen(str), 0);
  }
}

static void otaServerTask(void *param) {
  auto *ctx = static_cast<OtaServerCtx *>(param);
  int listen_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
  if (listen_fd < 0) {
    ESP_LOGE(TAG, "Unable to create OTA socket: errno %d", errno);
    delete ctx;
    vTaskDelete(nullptr);
    return;
  }

  int opt = 1;
  setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  struct sockaddr_in serv_addr {};
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  serv_addr.sin_port = htons(ctx->port);

  if (bind(listen_fd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) != 0) {
    ESP_LOGE(TAG, "OTA socket bind failed on port %d: errno %d", ctx->port, errno);
    close(listen_fd);
    delete ctx;
    vTaskDelete(nullptr);
    return;
  }

  if (listen(listen_fd, 1) != 0) {
    ESP_LOGE(TAG, "OTA socket listen failed: errno %d", errno);
    close(listen_fd);
    delete ctx;
    vTaskDelete(nullptr);
    return;
  }

  ESP_LOGI(TAG, "OTA TCP Server listening on port %d (LAN8720)", ctx->port);

  while (true) {
    struct sockaddr_in client_addr {};
    socklen_t addr_len = sizeof(client_addr);
    int client_fd = accept(listen_fd, (struct sockaddr *)&client_addr, &addr_len);
    if (client_fd < 0) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    ESP_LOGI(TAG, "Incoming OTA connection from %s:%d",
             inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));

    char line[128] = {};
    if (!readLine(client_fd, line, sizeof(line), 5000)) {
      sendStr(client_fd, "ERR TIMEOUT\n");
      close(client_fd);
      continue;
    }

    // Step 1: Check Auth
    char pass[64] = {};
    if (sscanf(line, "AUTH %63s", pass) != 1 || strcmp(pass, ctx->password) != 0) {
      ESP_LOGW(TAG, "OTA rejected: invalid authentication password");
      sendStr(client_fd, "ERR AUTH\n");
      close(client_fd);
      continue;
    }
    sendStr(client_fd, "OK AUTH\n");

    // Step 2: Receive START command
    if (!readLine(client_fd, line, sizeof(line), 5000)) {
      sendStr(client_fd, "ERR TIMEOUT\n");
      close(client_fd);
      continue;
    }

    size_t image_size = 0;
    if (sscanf(line, "START %zu", &image_size) != 1 || image_size == 0) {
      sendStr(client_fd, "ERR INVALID_SIZE\n");
      close(client_fd);
      continue;
    }

    // Step 3: Safety check & start stream
    esp_ota_handle_t ota_handle = 0;
    const esp_partition_t *target_part = nullptr;
    esp_err_t err = gantryOtaStartStream(ctx->gantry, ota_handle, target_part);
    if (err != ESP_OK) {
      if (err == ESP_ERR_INVALID_STATE) {
        sendStr(client_fd, "ERR SAFETY: Gantry active (motors enabled/moving)\n");
      } else {
        sendStr(client_fd, "ERR OTA_INIT\n");
      }
      close(client_fd);
      continue;
    }

    sendStr(client_fd, "OK READY\n");

    // Step 4: Stream binary chunks
    size_t received_bytes = 0;
    static uint8_t chunk_buf[4096];
    bool stream_failed = false;

    // Set receive timeout to 10 seconds for chunk transfer
    struct timeval tv {};
    tv.tv_sec = 10;
    setsockopt(client_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    while (received_bytes < image_size) {
      size_t to_read = sizeof(chunk_buf);
      if (image_size - received_bytes < to_read) {
        to_read = image_size - received_bytes;
      }

      ssize_t read_bytes = recv(client_fd, chunk_buf, to_read, 0);
      if (read_bytes <= 0) {
        ESP_LOGE(TAG, "OTA socket recv error during stream: read %zd, errno %d",
                 read_bytes, errno);
        stream_failed = true;
        break;
      }

      err = gantryOtaWriteStream(ota_handle, chunk_buf, (size_t)read_bytes);
      if (err != ESP_OK) {
        stream_failed = true;
        break;
      }
      received_bytes += (size_t)read_bytes;
    }

    if (stream_failed || received_bytes != image_size) {
      ESP_LOGE(TAG, "OTA stream aborted: received %zu / %zu bytes",
               received_bytes, image_size);
      esp_ota_abort(ota_handle);
      sendStr(client_fd, "ERR WRITE_FAILED\n");
      close(client_fd);
      continue;
    }

    // Step 5: Finish & validate image
    err = gantryOtaFinishStream(ota_handle, target_part);
    if (err != ESP_OK) {
      sendStr(client_fd, "ERR VALIDATION_FAILED\n");
      close(client_fd);
      continue;
    }

    sendStr(client_fd, "OK COMPLETE: Rebooting into new firmware...\n");
    vTaskDelay(pdMS_TO_TICKS(500));
    close(client_fd);
    close(listen_fd);

    ESP_LOGI(TAG, "Restarting ESP32 now...");
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_restart();
  }
}

}  // namespace

esp_err_t gantryOtaStartServer(Gantry::Gantry *gantry, int port, const char *password) {
  auto *ctx = new OtaServerCtx();
  ctx->gantry = gantry;
  ctx->port = (port > 0) ? port : 8032;
  strncpy(ctx->password, (password != nullptr && password[0]) ? password : "LTU_1932",
          sizeof(ctx->password) - 1);

  BaseType_t res = xTaskCreatePinnedToCore(
      otaServerTask, "gantry_ota_srv", 8192, ctx, 4, nullptr, 0);
  if (res != pdPASS) {
    ESP_LOGE(TAG, "Failed to create OTA server task");
    delete ctx;
    return ESP_FAIL;
  }
  return ESP_OK;
}

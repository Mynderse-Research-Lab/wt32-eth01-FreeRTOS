/**
 * @file gantry_ota_tcp.cpp
 * @brief OTA over TCP on the LAN8720: the server task, its line protocol
 *        (AUTH, START, binary stream, OK COMPLETE) and gantryOtaStartServer.
 *
 * The flash side (start, write, finish, motor interlock) is in gantry_ota.cpp.
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

struct OtaServerCtx {
  Gantry::Gantry *gantry;
  int port;
  char password[64];
};

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
    if (!readLine(client_fd, line, sizeof(line), 1000)) {
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
    if (!readLine(client_fd, line, sizeof(line), 1000)) {
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
    const int64_t erase_t0 = esp_timer_get_time();
    esp_err_t err = gantryOtaStartStream(ctx->gantry, ota_handle, target_part, image_size);
    const int64_t erase_us = esp_timer_get_time() - erase_t0;
    if (err != ESP_OK) {
      if (err == ESP_ERR_INVALID_STATE) {
        sendStr(client_fd, "ERR SAFETY: Gantry busy (move/home/pick in progress)\n");
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

    // Silence longer than 2 s means the sender stalled.
    struct timeval tv {};
    tv.tv_sec = 2;
    setsockopt(client_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    // Where the stream time goes: waiting in recv() (the sender or TCP) or
    // inside esp_ota_write() (flash erase and program). Reported in OK COMPLETE.
    const int64_t stream_t0 = esp_timer_get_time();
    int64_t recv_us = 0;
    int64_t write_us = 0;
    int64_t write_max_us = 0;
    uint32_t recv_calls = 0;

    while (received_bytes < image_size) {
      size_t to_read = sizeof(chunk_buf);
      if (image_size - received_bytes < to_read) {
        to_read = image_size - received_bytes;
      }

      const int64_t recv_t0 = esp_timer_get_time();
      ssize_t read_bytes = recv(client_fd, chunk_buf, to_read, 0);
      const int64_t write_t0 = esp_timer_get_time();
      recv_us += write_t0 - recv_t0;
      recv_calls++;
      if (read_bytes <= 0) {
        ESP_LOGE(TAG, "OTA socket recv error during stream: read %zd, errno %d",
                 read_bytes, errno);
        stream_failed = true;
        break;
      }

      err = gantryOtaWriteStream(ota_handle, chunk_buf, (size_t)read_bytes);
      const int64_t write_dt = esp_timer_get_time() - write_t0;
      write_us += write_dt;
      if (write_dt > write_max_us) {
        write_max_us = write_dt;
      }
      if (err != ESP_OK) {
        stream_failed = true;
        break;
      }
      received_bytes += (size_t)read_bytes;
    }
    const int64_t stream_us = esp_timer_get_time() - stream_t0;

    if (stream_failed || received_bytes != image_size) {
      ESP_LOGE(TAG, "OTA stream aborted: received %zu / %zu bytes",
               received_bytes, image_size);
      esp_ota_abort(ota_handle);
      sendStr(client_fd, "ERR WRITE_FAILED\n");
      close(client_fd);
      continue;
    }

    // Step 5: Finish & validate image
    const int64_t finish_t0 = esp_timer_get_time();
    err = gantryOtaFinishStream(ota_handle, target_part);
    const int64_t finish_us = esp_timer_get_time() - finish_t0;
    if (err != ESP_OK) {
      sendStr(client_fd, "ERR VALIDATION_FAILED\n");
      close(client_fd);
      continue;
    }

    // Keep this line short: the supervisor shows it in a 128-byte status message.
    char done[112];
    snprintf(done, sizeof(done),
             "OK COMPLETE ms: erase=%lu stream=%lu recv=%lu write=%lu wmax=%lu verify=%lu n=%lu\n",
             (unsigned long)(erase_us / 1000),
             (unsigned long)(stream_us / 1000), (unsigned long)(recv_us / 1000),
             (unsigned long)(write_us / 1000), (unsigned long)(write_max_us / 1000),
             (unsigned long)(finish_us / 1000), (unsigned long)recv_calls);
    ESP_LOGI(TAG, "%.*s", (int)strcspn(done, "\n"), done);
    sendStr(client_fd, done);
    vTaskDelay(pdMS_TO_TICKS(500));
    close(client_fd);
    close(listen_fd);

    // A supervisor START during the upload re-enables the motors. Drop them
    // under control before the reboot cuts Class 1.
    (void)gantryOtaDisableMotors(ctx->gantry);
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

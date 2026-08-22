/**
 * @file gantry_ota.h
 * @brief Ethernet OTA firmware update manager with motion safety interlocks.
 *
 * Coordinates Dual-OTA partition streaming, flash validation, anti-rollback,
 * and pre-flight motion state safety gating.
 */

#ifndef GANTRY_OTA_H
#define GANTRY_OTA_H

#include "Gantry.h"
#include "esp_app_format.h"
#include "esp_err.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"

#include <cstdint>

struct GantryOtaStatus {
  char running_partition[16] = {};
  uint32_t running_address = 0;
  uint32_t running_size = 0;

  char next_partition[16] = {};
  uint32_t next_address = 0;
  uint32_t next_size = 0;

  char app_version[32] = {};
  char project_name[32] = {};
  char compile_date[16] = {};
  char compile_time[16] = {};
  char idf_version[32] = {};

  esp_ota_img_states_t state = ESP_OTA_IMG_UNDEFINED;
  bool rollback_possible = false;
};

/**
 * @brief Query current OTA partition and active firmware metadata.
 */
esp_err_t gantryOtaGetStatus(GantryOtaStatus &status);

/**
 * @brief Print OTA status, running slot, and version details to log.
 */
void gantryOtaPrintStatus();

/**
 * @brief Confirm the newly booted firmware image is valid, cancelling rollback.
 * Must be called after system subsystems and motion interfaces initialize cleanly.
 */
esp_err_t gantryOtaConfirmBootValid();

/**
 * @brief Begin an OTA flash stream, verifying motion safety preconditions.
 * @param gantry Active Gantry instance to check for motion idle/disabled state.
 * @param handle Output OTA transaction handle.
 * @param target_part Output pointer to target partition receiving update.
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if gantry is enabled/moving.
 */
esp_err_t gantryOtaStartStream(Gantry::Gantry *gantry, esp_ota_handle_t &handle,
                              const esp_partition_t *&target_part);

/**
 * @brief Write a chunk of the incoming firmware binary to flash.
 */
esp_err_t gantryOtaWriteStream(esp_ota_handle_t handle, const void *data, size_t length);

/**
 * @brief Complete OTA stream, validate image header/CRC, and set next boot partition.
 */
esp_err_t gantryOtaFinishStream(esp_ota_handle_t handle, const esp_partition_t *target_part);

/**
 * @brief Start the dedicated background TCP OTA server task on the LAN8720 interface.
 * @param gantry Pointer to live Gantry motion controller.
 * @param port TCP port to bind (default 8032).
 * @param password Authentication password string (e.g. LTU_1932).
 * @return ESP_OK on success.
 */
esp_err_t gantryOtaStartServer(Gantry::Gantry *gantry, int port, const char *password);

#endif  // GANTRY_OTA_H

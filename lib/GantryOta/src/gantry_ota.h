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
 * @brief Why the ESP32 last reset (esp_reset_reason), as a short label.
 *
 * The UART boot log is lost on a TCP-only bench, so `ota` and `status` print
 * this to tell an OTA restart (SW) from a crash (PANIC / WDT) or BROWNOUT.
 */
const char *gantryResetReasonName();

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
 * @brief Disable the motors if they are enabled (OTA start and pre-reboot).
 * @param gantry Active Gantry instance; nullptr is a no-op.
 * @return ESP_OK when the motors are off, ESP_ERR_INVALID_STATE if they stayed enabled.
 */
esp_err_t gantryOtaDisableMotors(Gantry::Gantry *gantry);

/**
 * @brief Begin an OTA flash stream, verifying motion safety preconditions.
 *
 * Idle motors are disabled automatically. A move, home or pick in progress
 * refuses the update instead of being interrupted.
 * @param gantry Active Gantry instance to check for motion idle state.
 * @param handle Output OTA transaction handle.
 * @param target_part Output pointer to target partition receiving update.
 * @param image_size Size announced by the sender. That range is erased before
 *        the stream, in 64 KB blocks where aligned; 0 erases sector by sector
 *        while writing.
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if the gantry is busy or
 *         the motors would not disable.
 */
esp_err_t gantryOtaStartStream(Gantry::Gantry *gantry, esp_ota_handle_t &handle,
                              const esp_partition_t *&target_part, size_t image_size);

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

/**
 * @brief N-2 diagnostic: erase (erase=true) or write back to back (erase=false)
 *        n 4 KB blocks in the passive slot from 1.5 MB on, to measure EMAC
 *        buffering while a flash write stalls reception.
 * @return ESP_OK with a summary in out; refuses while the motors are enabled.
 */
esp_err_t gantryOtaFlashStallTest(Gantry::Gantry *gantry, bool erase, int writes, char *out,
                                  size_t out_len);

/**
 * @brief CELL_COMMAND 7 (REBOOT_TO_OTA): refuse while busy or while the running
 *        image is not marked valid; otherwise disable the motors, store the
 *        session in noinit RAM and restart into OTA-only mode (does not return).
 *        lib/CellNetL2/CELLNET_OTA_PROTOCOL.md section 3.
 */
esp_err_t gantryOtaRebootToCellNetMode(Gantry::Gantry *gantry, uint16_t session);

/** @brief At boot: true (and the session) only after a software reset with a valid
 *         flag; the flag is cleared either way. Call first in app_main. */
bool gantryOtaCellNetModeRequested(uint16_t *session);

/** @brief Run OTA-only mode: Ethernet, CellNet and the OTA receiver only. Never
 *         returns; reboots into the new image on success, the old one otherwise. */
void gantryOtaRunCellNetMode(uint16_t session);

#endif  // GANTRY_OTA_H

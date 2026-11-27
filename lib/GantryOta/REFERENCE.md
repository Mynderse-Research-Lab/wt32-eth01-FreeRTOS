# GantryOta reference

Public header: `lib/GantryOta/src/gantry_ota.h`.

## API

```cpp
esp_err_t gantryOtaGetStatus(GantryOtaStatus& status);
const char* gantryResetReasonName();
void gantryOtaPrintStatus();
esp_err_t gantryOtaConfirmBootValid();
esp_err_t gantryOtaDisableMotors(Gantry::Gantry* gantry);
esp_err_t gantryOtaStartStream(Gantry::Gantry* gantry, esp_ota_handle_t& handle,
                               const esp_partition_t*& target_part, size_t image_size);
esp_err_t gantryOtaWriteStream(esp_ota_handle_t handle, const void* data, size_t length);
esp_err_t gantryOtaFinishStream(esp_ota_handle_t handle, const esp_partition_t* target_part);
esp_err_t gantryOtaStartServer(Gantry::Gantry* gantry, int port, const char* password);
esp_err_t gantryOtaFlashStallTest(Gantry::Gantry* gantry, bool erase, int writes,
                                  char* out, size_t out_len);
esp_err_t gantryOtaRebootToCellNetMode(Gantry::Gantry* gantry, uint16_t session);
bool gantryOtaCellNetModeRequested(uint16_t* session);
void gantryOtaRunCellNetMode(uint16_t session);
```

`GantryOtaStatus` holds the running and next partition label, address and size, the app description strings, the `esp_ota_img_states_t` and `rollback_possible`. Sizes are bytes. Addresses are flash offsets.

`gantryOtaGetStatus` returns `ESP_FAIL` when there is no running partition. A missing next partition is reported as the label `"none"`. A failed app-description read leaves those strings empty and still returns `ESP_OK`.

`gantryResetReasonName` returns a short label for `esp_reset_reason()`. `SW` is the label for `esp_restart` and OTA.

`gantryOtaConfirmBootValid` returns `ESP_FAIL` when there is no running partition. For `ESP_OTA_IMG_PENDING_VERIFY` or `ESP_OTA_IMG_NEW` it returns the result of `esp_ota_mark_app_valid_cancel_rollback`. Any other state returns `ESP_OK`.

`gantryOtaDisableMotors` returns `ESP_OK` when `gantry` is null or the motors are already off. Otherwise it calls `disable()` and returns `ESP_ERR_INVALID_STATE` if they stay enabled.

`gantryOtaStartStream` returns `ESP_ERR_INVALID_STATE` when `gantry` is non-null and `isBusy()`, and the same when the motors will not disable. It returns `ESP_ERR_NOT_FOUND` when there is no next OTA partition. `image_size` of 0 is passed to `esp_ota_begin` as `OTA_WITH_SEQUENTIAL_WRITES`; a positive size is erased up front. Other errors are the `esp_ota_begin` result.

`gantryOtaWriteStream` returns `ESP_OK` for a null buffer or a zero length, and otherwise the `esp_ota_write` result.

`gantryOtaFinishStream` returns `ESP_ERR_INVALID_ARG` when `target_part` is null. It then returns the first failure from `esp_ota_end` or `esp_ota_set_boot_partition`.

`gantryOtaStartServer` listens on `port`, or on 8032 when `port` is not positive. A null or empty `password` is stored as `LTU_1932`. It returns `ESP_FAIL` when the task is not created.

`gantryOtaFlashStallTest` writes one line into `out`. It returns `ESP_ERR_INVALID_STATE` when the motors are enabled or the gantry is busy, `ESP_ERR_NOT_FOUND` when the passive slot is missing or no larger than the 1.5 MB offset (`0x180000`), and `ESP_ERR_INVALID_ARG` when `writes` is outside `1 .. (slot size - offset) / 4096`. `erase` true erases that many 4 KB blocks. `erase` false writes them. The summary line includes the elapsed time.

`gantryOtaRebootToCellNetMode` returns `ESP_ERR_INVALID_STATE` when the gantry is busy, when the running partition is missing, or when its image state was read and is neither `ESP_OTA_IMG_VALID` nor `ESP_OTA_IMG_UNDEFINED`. It returns the `gantryOtaDisableMotors` error if that fails. Otherwise it stores the session in noinit RAM and calls `esp_restart`. The header states the contract; the message layout is `lib/CellNetL2/CELLNET_OTA_PROTOCOL.md` section 3.

`gantryOtaCellNetModeRequested` is true only after a software reset whose noinit flag matches the magic and the session complement. It clears the flag either way. A null `session` still returns the flag result and writes nothing.

`gantryOtaRunCellNetMode` does not return. It starts Ethernet, sets the private OTA MAC and the EMAC filters, and runs the OTA tasks. Failure paths reboot into the previous image. The filter rules are section 5.1 of the protocol document.

## Task and timing

| Task | Core | Priority | Stack | When |
|---|---:|---:|---:|---|
| `gantry_ota_srv` | 0 | 4 | 8192 | `gantryOtaStartServer` |
| `cellnet_ota` | 0 | 5 | 8192 | OTA-only mode |
| `ota_hb` | 0 | 3 | 3072 | OTA-only mode |

`bootStartNetwork` passes port 8032 and `CONSOLE_TCP_PASSWORD` (`src/main_init_net.cpp`).

The TCP task allows 1 s for the `AUTH` line and 1 s for the `START` line, then 2 s of silence on the binary stream. The stream buffer is 4,096 bytes. After `OK COMPLETE` it waits 500 ms, closes the sockets, disables the motors, waits 200 ms and restarts.

In OTA-only mode the heartbeat period is 250 ms (`kHeartbeatMs`), `READY` repeats every 250 ms until the first data (`kReadyResendMs`), and a failure waits 300 ms before restart (`kRestartDelayMs`). The receive queue holds 24 frames (`kRxQueueDepth`). The mode ends at 120 s (`kHardCapMs`) even if the receiver does not. The receiver's own deadlines, 30 s to `OTA_BEGIN` and 2 s without progress, are `kBeginDeadlineMs` and `expired` in `lib/CellNetL2/src/CellNetOtaReceiver.h`. Ethernet must be up within 10 s or the mode reboots.

## AppConfig

None. The TCP port and password are arguments. The stored `ota_port` and `con_pw` keys have no reader (G-F2); `cfg export` only hides `con_pw`.

## Errors

The `esp_err_t` values above. The TCP replies are `ERR TIMEOUT`, `ERR AUTH`, `ERR INVALID_SIZE`, `ERR SAFETY`, `ERR OTA_INIT`, `ERR WRITE_FAILED`, `ERR VALIDATION_FAILED` and `OK AUTH`, `OK READY`, `OK COMPLETE`. `gantryOtaPrintStatus` logs and returns when the status read fails.

## Console

`ota` (prefix) calls `gantryOtaPrintStatus`. `flashstall` (prefix) calls `gantryOtaFlashStallTest`. Both are in `src/gantry_console_commands.cpp`. `status` prints `gantryResetReasonName` from `src/gantry_console_status.cpp`. This component registers no command.

## Usage

```cpp
uint16_t session = 0;
if (gantryOtaCellNetModeRequested(&session)) {
    gantryOtaRunCellNetMode(session);
}
gantryOtaStartServer(gantry, 8032, "LTU_1932");
```

## Documents

TCP behaviour and the partition map are section 13 of `LOW_LEVEL_GANTRY_CONTROL.md`. CellNet OTA-only mode is `lib/CellNetL2/CELLNET_OTA_PROTOCOL.md`. The receiver state machine is `lib/CellNetL2/src/CellNetOtaReceiver.h`.

# WT32-ETH01 Reset Loop Diagnostics

## Problem Summary

A reset loop was observed during startup immediately after MCP23S17 initialization.

Observed log pattern:
- Application reaches `GantryExample: Initializing MCP23S17 GPIO expander...`
- Logs `MCP23S17 initialized` and `GPIO expander initialized`
- Device resets and bootloader banner appears again (`ets Jun 8 2016 ...`)

This indicates a reset after early initialization, before full gantry startup.

## Why this branch exists

Branch: `diag/reset-loop-analysis`

Purpose:
- Add deterministic boot-step logging around each MCP23S17 setup action
- Record reset cause (`esp_reset_reason`) at startup
- Snapshot key strap pin levels (GPIO2/GPIO12/GPIO15) at app entry
- Isolate exactly which operation executes last before reset

## Diagnostics Added

File changed:
- `src/main.cpp`

Additions:
1. Reset reason logging:
   - `Boot reset reason: <reason> (<id>)`
2. Strap pin snapshot:
   - `GPIO2`, `GPIO12`, `GPIO15` levels at `app_main` entry
3. Boot-step markers:
   - `[BOOT-STEP N] ...` before each critical MCP configuration call
4. Checked return codes:
   - `gpio_expander_set_direction`, `gpio_expander_set_pullup`, `gpio_expander_write`
   - On any error, logs operation + `esp_err_to_name()` and returns
5. Global output-write isolation toggle (compile-time):
   - `DIAG_SKIP_ALL_MCP_INIT_OUTPUT_WRITES`
   - When enabled (`1`), startup skips all initial MCP output writes and logs a skip step.
6. Global output-direction isolation toggle (compile-time):
   - `DIAG_SKIP_MCP_OUTPUT_DIRECTION_CONFIG`
   - When enabled (`1`), startup skips MCP output-direction setup and logs a skip step.
7. Input-direction isolation toggle (compile-time):
   - `DIAG_SKIP_MCP_INPUT_DIRECTION_CONFIG`
   - When enabled (`1`), startup skips MCP input-direction setup and logs a skip step.
8. Input pull-up isolation toggle (compile-time):
   - `DIAG_SKIP_MCP_INPUT_PULLUP_CONFIG`
   - When enabled (`1`), startup skips MCP input pull-up setup and logs a skip step.
9. MCP SPI clock override (compile-time):
   - `DIAG_MCP_SPI_CLOCK_HZ_OVERRIDE`
   - Defaults to `1000000` (1 MHz) on diagnostics branch to reduce bus stress.
10. Post-init halt toggle (compile-time):
   - `DIAG_HALT_AFTER_MCP_INIT`
   - When enabled (`1`), firmware halts after MCP setup to test stability with no downstream init.
11. Phase-based startup halt selector (compile-time):
   - `DIAG_STARTUP_HALT_PHASE`
   - Values:
     - `0` disabled
     - `1` halt after direct pulse GPIO setup
     - `2` halt after gantry driver configuration wiring
     - `3` halt before `gantry.begin()`
     - `4` halt after `gantry.begin()`
     - `5` halt after `gantry.enable()`
12. Direct pulse GPIO setup isolation toggle (compile-time):
   - `DIAG_SKIP_DIRECT_PULSE_GPIO_CONFIG`
   - When enabled (`1`), skips `gpio_config` + initial `gpio_set_level` on `PIN_X_PULSE`/`PIN_Y_PULSE`.
13. Gantry begin sub-step isolation toggles (compile-time):
   - `GANTRY_DIAG_SKIP_AXIS_X_INIT`
   - `GANTRY_DIAG_SKIP_AXIS_Y_INIT`
   - `GANTRY_DIAG_SKIP_THETA_INIT`
   - Added in `lib/Gantry/src/Gantry.cpp` with `[BEGIN] ...` logs around each initialize path.

## How to Run the Diagnostics

1. Build and flash this branch.
2. Open serial monitor and capture complete boot logs.
3. Find the last `[BOOT-STEP N]` emitted before reset.

Optional isolation build flag:
- Add `-DDIAG_SKIP_ALL_MCP_INIT_OUTPUT_WRITES=1` to skip all initial MCP output writes.
- Add `-DDIAG_SKIP_MCP_OUTPUT_DIRECTION_CONFIG=1` to skip MCP output-direction setup.
- Add `-DDIAG_SKIP_MCP_INPUT_DIRECTION_CONFIG=1` to skip MCP input-direction setup.
- Add `-DDIAG_SKIP_MCP_INPUT_PULLUP_CONFIG=1` to skip MCP input pull-up setup.
- Add `-DDIAG_MCP_SPI_CLOCK_HZ_OVERRIDE=1000000` (or lower) to reduce SPI transaction stress.
- Add `-DDIAG_HALT_AFTER_MCP_INIT=1` to stop immediately after MCP setup and observe reset behavior.
- Add `-DDIAG_STARTUP_HALT_PHASE=<0..5>` to move the halt boundary through startup phases.
- Add `-DDIAG_SKIP_DIRECT_PULSE_GPIO_CONFIG=1` to isolate reset behavior from X/Y pulse pin setup.
- Add `-DGANTRY_DIAG_SKIP_AXIS_X_INIT=1` to skip X-axis init inside `Gantry::begin()`.
- Add `-DGANTRY_DIAG_SKIP_AXIS_Y_INIT=1` to skip Y-axis init inside `Gantry::begin()`.
- Add `-DGANTRY_DIAG_SKIP_THETA_INIT=1` to skip theta init inside `Gantry::begin()`.

Interpretation:
- If reset occurs before step logs, issue is very early boot or power.
- If reset always occurs after a specific output write, suspect wiring/current surge on the associated signal.
- If reset reason changes to `BROWNOUT`, prioritize power integrity checks.

## Recommended Diagnostic Procedure

1. **Power integrity**
   - Verify stable 5V/3V3 rails under load
   - Check common ground between WT32, drivers, and MCP board
2. **Isolation test**
   - Boot with motor-driver connectors removed (keep MCP only)
   - Reconnect loads one by one
3. **Pin-level hardware checks**
   - Confirm strap pins are not forced to unsafe states at reset
   - Confirm no shorts on MCP outputs
4. **Progressive enable**
   - Keep outputs in safe defaults and enable downstream hardware incrementally

## Resolution Path (No Functional Compromise)

Resolution target while preserving all features:
- Keep Ethernet, MCP23S17, LEDC, and PCNT functionality intact
- Use step logs to identify exact failing edge
- Apply the minimal hardware/config fix based on the failing step (e.g., pull resistor adjustment, wiring correction, staged enable sequence)

Current pin mapping already avoids non-exposed WT32 pins and is documented.

## Root Cause Identified

From `[BEGIN]` diagnostics:
- Reset occurred immediately after `Configure end-effector pin=7` in `Gantry::begin()`.

Cause:
- `PIN_GRIPPER` is MCP logical pin `7`, but `GantryEndEffector` previously used raw `pinMode()`/`digitalWrite()`.
- On ESP32, raw GPIO 7 is a flash-connected pin and must not be driven for application IO.
- This caused watchdog resets during startup.

Fix applied:
- `lib/Gantry/src/GantryEndEffector.cpp` now routes pins `<16` through `gpio_expander_set_direction()` / `gpio_expander_write()`.
- Direct ESP32 GPIO path is still used for pins `>=16`.

## Secondary Runtime Panic Identified

After startup resets were resolved, a runtime panic occurred with:
- `assert failed: spi_device_transmit ... (ret_trans == trans_desc)`
- Backtrace path: `mcp23s17_read_register` -> `mcp23s17_read_pin` -> `gpio_expander_read` from `gantryUpdateTask`.

Cause:
- Concurrent MCP23S17 SPI transactions from multiple tasks without serialization.

Fix applied:
- `lib/MCP23S17/src/MCP23S17.cpp` now includes a per-device FreeRTOS mutex in the handle.
- All register read/write SPI transactions are serialized with lock/unlock around `spi_device_transmit`.
- Mutex lifecycle is handled in init failure paths and deinit.

Result:
- No further `spi_device_transmit` assertions observed in runtime logs.

## Expected Final Resolution Criteria

Issue considered resolved when:
- Device boots past all MCP configuration steps consistently
- No immediate reset loop observed in repeated cold boots
- Gantry init reaches task creation and command console startup
- No new boot-mode instability on strap pins

Current status:
- Resolved. System reaches full startup (`System ready`), creates update/console tasks, and remains stable in runtime logs.

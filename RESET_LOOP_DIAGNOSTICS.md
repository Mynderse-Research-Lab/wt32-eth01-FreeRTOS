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

## How to Run the Diagnostics

1. Build and flash this branch.
2. Open serial monitor and capture complete boot logs.
3. Find the last `[BOOT-STEP N]` emitted before reset.

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

## Expected Final Resolution Criteria

Issue considered resolved when:
- Device boots past all MCP configuration steps consistently
- No immediate reset loop observed in repeated cold boots
- Gantry init reaches task creation and command console startup
- No new boot-mode instability on strap pins

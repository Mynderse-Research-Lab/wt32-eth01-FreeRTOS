# Gantry Development Journal

**Version:** 1.0.0  
**Last Updated:** Feb 10th 2026

This journal captures the full development path of the current Gantry + SDF08NK8X integration branch, including architecture moves, behavior changes, safety fixes, console/diagnostic additions, and branch/merge decisions.

---

## 1) Major Architecture Changes

- Migrated the application to a FreeRTOS-style structure.
- Split serial/test console logic out of `main.cpp` into `src/gantry_test_console.cpp`.
- Moved pin and app constants into `include/gantry_app_constants.h`.
- Added dual hardware mode support:
  - MCP23S17 IO expander mode.
  - Direct WT32 GPIO mode for test bring-up without MCP23S17.
- Added reusable limit-switch object in Gantry library:
  - `lib/Gantry/src/GantryLimitSwitch.h`
  - `lib/Gantry/src/GantryLimitSwitch.cpp`
- Shifted limit ownership from driver-internal logic to Gantry-level logic for reuse across axes.

---

## 2) Safety, Alarm, and Motion Behavior Updates

- Alarm behavior:
  - Homing and calibration are blocked when alarm is active.
  - Gantry alarm status is driven by driver status state (not raw app-level pin interpretation).
- Motion-stop behavior:
  - Added force-stop behavior when Gantry detects motion-path failures.
  - Expanded stop-on-failure handling beyond X to Y/Theta command paths.
- Calibration robustness:
  - Added MIN-release watchdog after starting MIN->MAX travel.
  - Added explicit warning log when MIN-release watchdog aborts calibration.
- Startup flow:
  - Homing + calibration session gating retained.
  - Move commands remain blocked until startup sequence succeeds.

---

## 3) Console and Runtime Diagnostics

- Added/extended console commands and feedback:
  - `pins`, `limits`, `status`, `alarmreset`/`arst`, `speed`, `accel`, `selftest`.
- Added control-variable flip monitor with debounce and realtime output.
- Added live motion telemetry while busy:
  - `LIVE POS: x_cmd=..., x_enc=..., y=..., theta=...`
- `status` formatting kept stable per operator request; live telemetry carries expanded dual-X visibility.

---

## 4) Pinout and Test Wiring Evolution

- Temporary direct test mapping aligned to SDF08 driver test branch:
  - Pulse/Dir/Enable/ALM/ARST wired directly to WT32 pins.
- ARST test pin enabled and moved to GPIO32.
- Limit switches required in active axis modes and monitored in runtime diagnostics.
- Added spreadsheet-friendly pinout artifacts:
  - `pinout.csv`

---

## 5) Build/Tooling and IDF Integration

- Standardized project indexing/build flow around ESP-IDF and compile database.
- Removed PlatformIO-centric index assumptions from active setup.
- Added/updated IDF component registration and include wiring for:
  - Gantry
  - SDF08NK8X
  - MCP23S17
- Kept build validation loop with `idf.py build` after each substantive behavior change.

---

## 6) Driver-Library Branch Operations

- Converted driver serial prints to ESP logging style.
- Removed driver-side limit-switch instance/debounce handling in driver branch.
- Merged driver-branch changes back into Gantry branch line.
- Preserved app-level visibility of driver logs through test console workflow.

---

## 7) Current Known Operational Signals

- If `x_cmd` changes while `x_enc` stays flat:
  - Command generation is active, but encoder feedback is not changing.
  - This indicates feedback path, encoder mode, or physical wiring/config mismatch.
- If calibration starts at MIN and quickly fails:
  - Check MIN release behavior and direction polarity.
  - Use live telemetry + control flips + `limits` command to diagnose release timing.

---

## 8) Remaining Cleanup Opportunities

- Consider replacing fixed MIN-release timeout with distance-based criterion.
- Continue reducing non-essential debug noise in normal mode while retaining deep diagnostics in debug mode.
- Migrate legacy PCNT usage warnings to new pulse counter API when practical.

---

## 9) Related Files

- Core application:
  - `src/main.cpp`
  - `src/gantry_test_console.cpp`
- Core Gantry library:
  - `lib/Gantry/src/Gantry.cpp`
  - `lib/Gantry/src/Gantry.h`
  - `lib/Gantry/src/GantryLimitSwitch.cpp`
  - `lib/Gantry/src/GantryLimitSwitch.h`
- Driver integration:
  - `lib/SDF08NK8X/src/SDF08NK8X.cpp`
  - `lib/SDF08NK8X/src/SDF08NK8X.h`
- Integration docs:
  - `PROGRAMMING_REFERENCE.md` (root)
  - `LIBRARIES_OVERVIEW.md` (root)
  - `lib/Gantry/docs/API_REFERENCE.md`
  - `lib/Gantry/docs/ARCHITECTURE.md`


# Code Encapsulation

**Status:** Live record, 2026-11-26, read from `refac/code_encapsulation`. Update it in the same commit as each split or move (OPEN_ITEMS X-7).

This page records every source file, the plan to bring each one under 500 lines, and the plan to move matured features into their own `lib/` components. The library review is in [LIBRARY_REVIEW.md](LIBRARY_REVIEW.md).

## Rules

- **Source files stay under 500 lines.** A larger file is split by what its code does, for robustness and review.
- **A split moves code; it does not change it.** Each split is its own commit:
  - Function bodies are copied verbatim.
  - Every original line is used exactly once.
  - The only edits are to linkage: `static` added or dropped, shared declarations in a private header, default arguments moved to the declaration.
  - A function that is itself too long is broken up in a separate commit afterwards.
- **Each split commit updates:**
  - the hand-kept source lists (`lib/*/CMakeLists.txt` SRCS, `idf/main/CMakeLists.txt` APP_SRCS, `test/host/CMakeLists.txt`)
  - every `file:line` reference in the docs, found by its code text
  - this page
- **Class 1 code needs extra care.** The firmware builds at `-Og` without link-time optimization, so a helper moved to another file is no longer inlined into its caller.
  - Keep every helper on the Class 1 path in its caller's file, or make it `inline` in the private header.
  - A Class 1 split counts as done only after an on-cell `eiptiming` and `gaptrace` soak.
- **Gate for every commit:**
  - host tests pass, with Unity case counts unchanged
  - `idf.py build` shows no new warnings
  - the touched files are under 500 lines

## Done

| Commit | File | Before | After |
|---|---|---:|---|
| `3ca7df4` | `lib/Gantry/src/Gantry.cpp` | 3,336 | 9 files, 240–498 lines |
| `64040cb`, `5890c1a` | `src/gantry_test_console.cpp` | 4,422 | 17 files, 98–463 lines; `processCommand` became a command table |
| `285825e` | `src/pick_scheduler.cpp` | 2,785 | 9 files, 252–403 lines; the strike path stays in `pick_strike.cpp` |
| `6bcb0ab` | `lib/AppConfig/src/AppConfig.cpp` | 1,841 | 6 files, 176–398 lines; `loadDefaults()` calls two section functions |
| `d114630` | `src/gantry_net_console.cpp` | 555 | 2 files, 182 and 396 lines; login and the recent-peer cache are in `gantry_net_console_auth.cpp` |
| `8339673` | `src/gantry_ota.cpp` | 562 | 2 files, 270 and 316 lines; the TCP OTA server task and `gantryOtaStartServer` are in `gantry_ota_tcp.cpp` |
| `e971af4`, `fcb89c8` | `src/main.cpp` | 615 | 6 files, 53–231 lines; `app_main` calls five stage functions (config, I/O, motion, tasks, network) in `main_init_*.cpp` and `main_tasks.cpp` |
| `1db4520` | `lib/MCP23S17/src/MCP23S17.cpp` | 546 | 3 files, 66–347 lines; lock, holder logging and register access are in `MCP23S17Bus.cpp` |
| `28c3107` | `lib/Gantry/src/GantryEipRotaryAxis.cpp` | 768 | 3 files, 53–465 lines; `update()` and the trace stamps are in `GantryEipRotaryAxisUpdate.cpp` |
| `e1c549b` | `lib/Gantry/src/GantryEipLinearAxis.cpp` | 955 | 4 files, 50–457 lines; the arm and move phase machine is in `GantryEipLinearAxisPhases.cpp`, alarms and logging in `GantryEipLinearAxisAlarm.cpp` |
| `e4761cb` | `lib/CellNetL2/src/CellNetOtaSender.cpp` | 1,209 | 4 files, 87–424 lines; `Link` and the sender class are in `CellNetOtaSenderInternal.h`, the phases in `CellNetOtaSenderPhases.cpp`, reporting in `CellNetOtaSenderReport.cpp` |
|  | `lib/W5500/src/W5500.cpp` | 731 | 3 files, 16–436 lines; init, reset and recover are in `W5500Init.cpp`. Register, buffer and MACRAW access stay. Waits for an on-cell `eiptiming` and `gaptrace` soak |

## Planned splits

13 files are over 500 lines. The production files come first, lowest risk first, then the headers, then the host tests.

### Production files (3)

| Order | File | Lines | Planned split | Gate |
|---:|---|---:|---|---|
| 1 | `lib/EtherNetIP/src/EipMultiScanner.cpp` | 829 | Forward open, forward close and reopen into `EipMultiScannerConnect.cpp`; the exchange stays | **Class 1**: soak |
| 2 | `lib/W5500/src/W5500Socket.cpp` | 990 | TCP path (connect, listen, send, receive) into `W5500SocketTcp.cpp`; the UDP fast path stays | **Class 1 hot path**: soak |
| 3 | `lib/EtherNetIP/src/EipScannerTask.cpp` | 1,329 | Diagnostics (run-time trace, gap report, log task, snapshot trickle) into `EipScannerDiag.cpp`; configuration builders into `EipScannerConfig.cpp`; then a separate commit breaks `multiAxisTask` (about 300 lines) into setup, cyclic and recover | **Class 1**: soak |

### Headers (3)

| File | Lines | Planned split |
|---|---:|---|
| `include/pick_intercept.h` | 1,531 | Themed headers (belt clock, part tracker, intercept and strike leads, vision contract). `pick_intercept.h` stays as the umbrella include, so callers do not change. Host-tested by `test_pick_intercept` |
| `lib/Gantry/src/Gantry.h` | 597 | Nested types (`RuntimeParams`, the enums, the trace structs) into `GantryTypes.h` |
| `include/coordinated_grasp.h` | 504 | The reference-fixture loader apart from the model |

### Host tests (7)

Each test file splits by test group into several sources for the **same executable**. Gate: the Unity case count per executable is unchanged.

| File | Lines |
|---|---:|
| `test/host/test_eip_transport.cpp` | 1,891 |
| `test/host/test_eip_axis.cpp` | 1,807 |
| `test/host/test_pick_intercept.cpp` | 1,095 |
| `test/host/test_gantry_orchestration.cpp` | 1,082 |
| `test/host/test_w5500_socket_fastpath.cpp` | 1,055 |
| `test/host/test_app_config.cpp` | 680 |
| `test/host/test_cell_net_l2.cpp` | 576 |

## Organizing matured features into `lib/` components

A feature is **matured** when it runs on the gantry and has no open blocker or decision in [OPEN_ITEMS.md](OPEN_ITEMS.md). A matured feature moves out of `src/` into its own ESP-IDF component, `lib/<Feature>/`, containing:
- **`CMakeLists.txt`**: `idf_component_register` with explicit REQUIRES. App headers come in through `${APP_ROOT}/include`, as in `lib/Gantry/CMakeLists.txt`.
- **`src/`**: the code.
- **`README.md`**: purpose, maturity status, dependencies, consumers, build and test, and the open items that concern it.
- **`REFERENCE.md`**:
  - the public API with signatures and units
  - task, core and priority, and timing limits
  - the `AppConfig` keys read
  - errors and return values
  - the console commands it serves
  - a short usage example
  - links to the canonical docs (the math is cited from its canonical document, not restated)

| Feature (today in `src/`) | Status | Component |
|---|---|---|
| `boot_log_ring.*` | Matured | `lib/BootLogRing` |
| `gpio_expander.*` | Matured; G-D3 concerns `MCP23S17` and `Spi3Bus`, not this wrapper (to be confirmed at the move) | `lib/GpioExpander` |
| `theta_trace.*` with `include/theta_trace_analysis.h` | Matured | `lib/ThetaTrace` |
| `gantry_ota.*`, `gantry_cellnet_ota.cpp` | Matured (N-2 done) | `lib/GantryOta` |
| Test console (17 files) with `gantry_net_console.*` | **Not yet.** G-C7 (the `move` speed decision) is open, and the console includes `pick_scheduler.h` | `lib/GantryConsole`, after decoupling |
| Pick scheduler (9 files) | **Not matured.** G-C1, G-C2, G-C11, G-C13 and G-C17 are open | Stays in `src/` |

**Console decoupling** is a separate commit before the console moves:
1. The command table gains a registration call (`gantryConsoleRegisterCommands(table, n)`).
2. `gantry_console_cmd_pick.cpp` and the `selftest` command stay in `src/` and register from `main`.
3. After that, the console includes no `src/` header.

**Each move updates:**
- `idf/main/CMakeLists.txt` (drop the `src` entries, add the component to REQUIRES)
- the test paths in `test/host/CMakeLists.txt`
- `CONTRIBUTING.md` (repo map), `docs/INDEX.md` and `lib/README`
- the code pointers in the Knowledge Transfer page and `LOW_LEVEL_GANTRY_CONTROL.md`
- the paths in OPEN_ITEMS

The existing libraries that lack a `README.md` or `REFERENCE.md` get them from the facts in the library review.

## Order of work

1. This record and the library review.
2. The Class 1 splits, each held until its on-cell soak passes.
3. Header splits, then test splits.
4. Component moves: `BootLogRing`, `GpioExpander`, `ThetaTrace`, `GantryOta`, the console decoupling, then `GantryConsole` once G-C7 is decided.
5. `README.md` and `REFERENCE.md` for the existing libraries.

## Inventory

Every tracked C and C++ source file in `src/`, `include/`, `lib/` and `test/host/`, with its line count on 2026-11-26.

### `include/` (13 files, 3,755 lines)

| File | Lines | Status |
|---|---:|---|
| `axis_drivetrain_params.h` | 496 | under 500 |
| `axis_pulse_motor_params.h` | 113 | under 500 |
| `cell_net_l2_protocol.h` | 260 | under 500 |
| `conveyor_intercept_params.h` | 44 | under 500 |
| `coordinated_grasp.h` | 504 | **over 500: planned** |
| `ethernet_app_config.h` | 122 | under 500 |
| `gantry_app_constants.h` | 127 | under 500 |
| `gantry_kconfig.h` | 19 | under 500 |
| `gantry_runtime_params.h` | 40 | under 500 |
| `gpio_expander.h` | 50 | under 500 |
| `pick_intercept.h` | 1531 | **over 500: planned** |
| `strike_plan.h` | 307 | under 500 |
| `theta_trace_analysis.h` | 142 | under 500 |

### `lib/AppConfig/` (16 files, 3,209 lines)

| File | Lines | Status |
|---|---:|---|
| `src/AppConfig.cpp` | 249 | split done (`6bcb0ab`) |
| `src/AppConfig.h` | 171 | split done (`6bcb0ab`) |
| `src/AppConfigAccess.cpp` | 398 | split done (`6bcb0ab`) |
| `src/AppConfigData.h` | 189 | under 500 |
| `src/AppConfigDefaultsCell.cpp` | 359 | split done (`6bcb0ab`) |
| `src/AppConfigDefaultsMotion.cpp` | 353 | split done (`6bcb0ab`) |
| `src/AppConfigInternal.h` | 32 | split done (`6bcb0ab`) |
| `src/AppConfigParamIo.cpp` | 373 | split done (`6bcb0ab`) |
| `src/AppConfigText.cpp` | 176 | split done (`6bcb0ab`) |
| `src/ConfigSchema.cpp` | 313 | under 500 |
| `src/ConfigSchema.h` | 53 | under 500 |
| `src/ConfigSet.h` | 200 | under 500 |
| `src/ConfigStorage.h` | 44 | under 500 |
| `src/MemoryStorage.h` | 113 | under 500 |
| `src/NvsStorage.cpp` | 150 | under 500 |
| `src/NvsStorage.h` | 36 | under 500 |

### `lib/CellNetL2/` (15 files, 2,935 lines)

| File | Lines | Status |
|---|---:|---|
| `src/CellNetL2.cpp` | 279 | under 500 |
| `src/CellNetL2.h` | 138 | under 500 |
| `src/CellNetL2Framing.cpp` | 279 | under 500 |
| `src/CellNetL2Framing.h` | 138 | under 500 |
| `src/CellNetOtaReceiver.cpp` | 208 | under 500 |
| `src/CellNetOtaReceiver.h` | 122 | under 500 |
| `src/CellNetOtaSender.cpp` | 418 | split done (`e4761cb`) |
| `src/CellNetOtaSender.h` | 95 | under 500 |
| `src/CellNetOtaSenderInternal.h` | 360 | split done (`e4761cb`) |
| `src/CellNetOtaSenderPhases.cpp` | 424 | split done (`e4761cb`) |
| `src/CellNetOtaSenderReport.cpp` | 87 | split done (`e4761cb`) |
| `src/EspEthL2Transport.cpp` | 108 | under 500 |
| `src/EspEthL2Transport.h` | 48 | under 500 |
| `src/IL2Transport.h` | 62 | under 500 |
| `src/LinuxRawSocketL2Transport.h` | 169 | under 500 |

### `lib/EtherNetIP/` (51 files, 7,306 lines)

| File | Lines | Status |
|---|---:|---|
| `src/CipEthernetLink.cpp` | 95 | under 500 |
| `src/CipEthernetLink.h` | 65 | under 500 |
| `src/CipMessageRouter.cpp` | 76 | under 500 |
| `src/CipMessageRouter.h` | 83 | under 500 |
| `src/CipTcpIpInterface.cpp` | 30 | under 500 |
| `src/CipTcpIpInterface.h` | 38 | under 500 |
| `src/EipByteBuffer.h` | 142 | under 500 |
| `src/EipClass1DeferredLog.h` | 106 | under 500 |
| `src/EipClass1Timing.h` | 127 | under 500 |
| `src/EipClass1TimingStats.cpp` | 137 | under 500 |
| `src/EipClass1TimingStats.h` | 90 | under 500 |
| `src/EipConnectionManager.cpp` | 198 | under 500 |
| `src/EipConnectionManager.h` | 128 | under 500 |
| `src/EipCpf.cpp` | 184 | under 500 |
| `src/EipCpf.h` | 85 | under 500 |
| `src/EipEncapsulation.cpp` | 83 | under 500 |
| `src/EipEncapsulation.h` | 90 | under 500 |
| `src/EipIoConnection.cpp` | 57 | under 500 |
| `src/EipIoConnection.h` | 53 | under 500 |
| `src/EipMultiScanner.cpp` | 829 | **over 500: planned** |
| `src/EipMultiScanner.h` | 214 | under 500 |
| `src/EipProcessImage.cpp` | 64 | under 500 |
| `src/EipProcessImage.h` | 41 | under 500 |
| `src/EipReliabilityStats.cpp` | 10 | under 500 |
| `src/EipReliabilityStats.h` | 33 | under 500 |
| `src/EipRunTimeTrace.cpp` | 65 | under 500 |
| `src/EipRunTimeTrace.h` | 59 | under 500 |
| `src/EipScanner.cpp` | 346 | under 500 |
| `src/EipScanner.h` | 102 | under 500 |
| `src/EipScannerPolicy.h` | 128 | under 500 |
| `src/EipScannerTask.cpp` | 1329 | **over 500: planned** |
| `src/EipScannerTask.h` | 49 | under 500 |
| `src/EipSession.cpp` | 157 | under 500 |
| `src/EipSession.h` | 46 | under 500 |
| `src/EipSocketW5500.cpp` | 304 | under 500 |
| `src/EipSocketW5500.h` | 69 | under 500 |
| `src/EipTransport.h` | 120 | under 500 |
| `src/EtherNetIP.h` | 26 | under 500 |
| `src/Hcs01Assembly.cpp` | 37 | under 500 |
| `src/Hcs01Assembly.h` | 57 | under 500 |
| `src/Hcs01ComwsClient.cpp` | 390 | under 500 |
| `src/Hcs01ComwsClient.h` | 85 | under 500 |
| `src/Hcs01ControlStatus.cpp` | 116 | under 500 |
| `src/Hcs01ControlStatus.h` | 106 | under 500 |
| `src/Kinetix5100Assembly.cpp` | 105 | under 500 |
| `src/Kinetix5100Assembly.h` | 147 | under 500 |
| `src/Kinetix5100ParamAccess.cpp` | 68 | under 500 |
| `src/Kinetix5100ParamAccess.h` | 59 | under 500 |
| `src/Kinetix5100TuningClient.cpp` | 76 | under 500 |
| `src/Kinetix5100TuningClient.h` | 32 | under 500 |
| `src/KinetixFaultCodes.h` | 170 | under 500 |

### `lib/EthernetLink/` (2 files, 462 lines)

| File | Lines | Status |
|---|---:|---|
| `src/EthernetLink.cpp` | 404 | under 500 |
| `src/EthernetLink.h` | 58 | under 500 |

### `lib/Gantry/` (42 files, 8,316 lines)

| File | Lines | Status |
|---|---:|---|
| `src/Gantry.cpp` | 472 | split done (`3ca7df4`) |
| `src/Gantry.h` | 597 | **over 500: planned** |
| `src/GantryAxisState.cpp` | 427 | split done (`3ca7df4`) |
| `src/GantryConfig.cpp` | 14 | under 500 |
| `src/GantryConfig.h` | 174 | under 500 |
| `src/GantryControlPeriod.h` | 46 | under 500 |
| `src/GantryDriveRef.h` | 24 | under 500 |
| `src/GantryEipBringUp.cpp` | 498 | split done (`3ca7df4`) |
| `src/GantryEipHomeCal.cpp` | 417 | split done (`3ca7df4`) |
| `src/GantryEipLimitSequence.h` | 103 | under 500 |
| `src/GantryEipLinearAxis.cpp` | 457 | split done (`e1c549b`) |
| `src/GantryEipLinearAxis.h` | 196 | under 500 |
| `src/GantryEipLinearAxisAlarm.cpp` | 119 | split done (`e1c549b`) |
| `src/GantryEipLinearAxisInternal.h` | 50 | split done (`e1c549b`) |
| `src/GantryEipLinearAxisPhases.cpp` | 383 | split done (`e1c549b`) |
| `src/GantryEipRotaryAxis.cpp` | 465 | split done (`28c3107`) |
| `src/GantryEipRotaryAxis.h` | 216 | under 500 |
| `src/GantryEipRotaryAxisInternal.h` | 53 | split done (`28c3107`) |
| `src/GantryEipRotaryAxisUpdate.cpp` | 285 | split done (`28c3107`) |
| `src/GantryEndEffector.cpp` | 91 | under 500 |
| `src/GantryEndEffector.h` | 34 | under 500 |
| `src/GantryEndEffectorPolicy.h` | 31 | under 500 |
| `src/GantryHoming.cpp` | 463 | split done (`3ca7df4`) |
| `src/GantryInternal.h` | 22 | split done (`3ca7df4`) |
| `src/GantryKinematics.cpp` | 50 | under 500 |
| `src/GantryKinematics.h` | 76 | under 500 |
| `src/GantryLimitSwitch.cpp` | 125 | under 500 |
| `src/GantryLimitSwitch.h` | 64 | under 500 |
| `src/GantryLimitSwitchPolicy.h` | 76 | under 500 |
| `src/GantryLinearAxis.h` | 131 | under 500 |
| `src/GantryLinearPhases.h` | 27 | under 500 |
| `src/GantryMotion.cpp` | 488 | split done (`3ca7df4`) |
| `src/GantryPath.cpp` | 272 | split done (`3ca7df4`) |
| `src/GantryPathProfile.h` | 316 | under 500 |
| `src/GantryPathTheta.cpp` | 259 | split done (`3ca7df4`) |
| `src/GantryRotaryAxis.h` | 139 | under 500 |
| `src/GantryRotaryPhases.h` | 26 | under 500 |
| `src/GantrySequential.cpp` | 240 | split done (`3ca7df4`) |
| `src/GantryThetaOrigin.h` | 71 | under 500 |
| `src/GantryTrajectory.cpp` | 106 | under 500 |
| `src/GantryTrajectory.h` | 103 | under 500 |
| `src/GantryUtils.h` | 110 | under 500 |

### `lib/MCP23S17/` (4 files, 755 lines)

| File | Lines | Status |
|---|---:|---|
| `src/MCP23S17.cpp` | 347 | split done (`1db4520`) |
| `src/MCP23S17.h` | 182 | under 500 |
| `src/MCP23S17Bus.cpp` | 160 | split done (`1db4520`) |
| `src/MCP23S17Internal.h` | 66 | split done (`1db4520`) |

### `lib/Spi3Bus/` (3 files, 293 lines)

| File | Lines | Status |
|---|---:|---|
| `src/Spi3Bus.cpp` | 200 | under 500 |
| `src/Spi3Bus.h` | 52 | under 500 |
| `src/Spi3BusCritical.h` | 41 | under 500 |

### `lib/SpiDisplay/` (18 files, 2,340 lines)

| File | Lines | Status |
|---|---:|---|
| `src/SpiDisplay.cpp` | 111 | under 500 |
| `src/SpiDisplay.h` | 48 | under 500 |
| `src/St7789Driver.cpp` | 438 | under 500 |
| `src/St7789Driver.h` | 93 | under 500 |
| `src/TftFont.cpp` | 104 | under 500 |
| `src/TftFont.h` | 18 | under 500 |
| `src/UiDashboard.cpp` | 157 | under 500 |
| `src/UiDashboard.h` | 43 | under 500 |
| `src/UiEditor.cpp` | 357 | under 500 |
| `src/UiEditor.h` | 65 | under 500 |
| `src/UiInput.cpp` | 84 | under 500 |
| `src/UiInput.h` | 42 | under 500 |
| `src/UiManager.cpp` | 240 | under 500 |
| `src/UiManager.h` | 67 | under 500 |
| `src/UiMenu.cpp` | 306 | under 500 |
| `src/UiMenu.h` | 68 | under 500 |
| `src/UiRebootPrompt.cpp` | 72 | under 500 |
| `src/UiRebootPrompt.h` | 27 | under 500 |

### `lib/W5500/` (16 files, 3,066 lines)

| File | Lines | Status |
|---|---:|---|
| `src/W5500.cpp` | 436 | split done |
| `src/W5500.h` | 217 | under 500 |
| `src/W5500BusMutex.cpp` | 84 | under 500 |
| `src/W5500BusMutex.h` | 40 | under 500 |
| `src/W5500DeferredLog.cpp` | 121 | under 500 |
| `src/W5500DeferredLog.h` | 48 | under 500 |
| `src/W5500Diag.cpp` | 305 | under 500 |
| `src/W5500Diag.h` | 149 | under 500 |
| `src/W5500Hal.h` | 33 | under 500 |
| `src/W5500Init.cpp` | 312 | split done |
| `src/W5500Internal.h` | 16 | split done |
| `src/W5500Poll.h` | 115 | under 500 |
| `src/W5500Socket.cpp` | 990 | **over 500: planned** |
| `src/W5500Socket.h` | 135 | under 500 |
| `src/W5500SpiHal.cpp` | 30 | under 500 |
| `src/W5500SpiHal.h` | 35 | under 500 |

### `src/` (51 files, 11,604 lines)

| File | Lines | Status |
|---|---:|---|
| `basic_tests.cpp` | 81 | under 500 |
| `basic_tests.h` | 11 | under 500 |
| `boot_log_ring.cpp` | 79 | under 500 |
| `boot_log_ring.h` | 16 | under 500 |
| `gantry_cellnet_ota.cpp` | 402 | under 500 |
| `gantry_console_bringup.cpp` | 217 | split done (`64040cb`) |
| `gantry_console_cfg.cpp` | 309 | split done (`64040cb`) |
| `gantry_console_cip.cpp` | 241 | split done (`64040cb`) |
| `gantry_console_cmd_motion.cpp` | 247 | split done (`5890c1a`) |
| `gantry_console_cmd_pick.cpp` | 177 | split done (`5890c1a`) |
| `gantry_console_cmd_profile.cpp` | 208 | split done (`5890c1a`) |
| `gantry_console_commands.cpp` | 330 | split done (`64040cb`) |
| `gantry_console_drive_diag.cpp` | 316 | split done (`64040cb`) |
| `gantry_console_drv.cpp` | 463 | split done (`64040cb`) |
| `gantry_console_homing.cpp` | 384 | split done (`64040cb`) |
| `gantry_console_internal.h` | 218 | split done (`64040cb`) |
| `gantry_console_io.cpp` | 255 | split done (`64040cb`) |
| `gantry_console_parse.h` | 193 | under 500 |
| `gantry_console_session.cpp` | 365 | split done (`64040cb`) |
| `gantry_console_status.cpp` | 378 | split done (`64040cb`) |
| `gantry_console_testcycle.cpp` | 450 | split done (`64040cb`) |
| `gantry_console_thetapath.cpp` | 98 | split done (`64040cb`) |
| `gantry_net_console.cpp` | 396 | split done |
| `gantry_net_console.h` | 23 | under 500 |
| `gantry_net_console_auth.cpp` | 182 | split done |
| `gantry_net_console_internal.h` | 29 | split done |
| `gantry_ota.cpp` | 316 | split done |
| `gantry_ota.h` | 131 | under 500 |
| `gantry_ota_tcp.cpp` | 270 | split done |
| `gantry_test_console.cpp` | 345 | split done (`64040cb`) |
| `gantry_test_console.h` | 67 | under 500 |
| `gpio_expander.c` | 206 | under 500 |
| `main.cpp` | 105 | split done |
| `main_init_io.cpp` | 143 | split done |
| `main_init_motion.cpp` | 231 | split done |
| `main_init_net.cpp` | 53 | split done |
| `main_internal.h` | 56 | split done |
| `main_tasks.cpp` | 169 | split done |
| `pick_belt.cpp` | 381 | split done (`285825e`) |
| `pick_bench.cpp` | 255 | split done (`285825e`) |
| `pick_console.cpp` | 253 | split done (`285825e`) |
| `pick_eval.cpp` | 403 | split done (`285825e`) |
| `pick_motion.cpp` | 274 | split done (`285825e`) |
| `pick_run.cpp` | 262 | split done (`285825e`) |
| `pick_scheduler.cpp` | 386 | split done (`285825e`) |
| `pick_scheduler.h` | 65 | under 500 |
| `pick_scheduler_internal.h` | 239 | split done (`285825e`) |
| `pick_strike.cpp` | 347 | split done (`285825e`) |
| `pick_trace.cpp` | 252 | split done (`285825e`) |
| `theta_trace.cpp` | 307 | under 500 |
| `theta_trace.h` | 20 | under 500 |

### `test/host/` (42 files, 13,331 lines)

| File | Lines | Status |
|---|---:|---|
| `fakes/FakeAxes.h` | 365 | under 500 |
| `stubs/driver/gpio.h` | 46 | under 500 |
| `stubs/esp_err.h` | 21 | under 500 |
| `stubs/esp_log.h` | 10 | under 500 |
| `stubs/esp_timer.h` | 13 | under 500 |
| `stubs/freertos/FreeRTOS.h` | 15 | under 500 |
| `stubs/freertos/semphr.h` | 18 | under 500 |
| `stubs/freertos/task.h` | 35 | under 500 |
| `stubs/gpio_expander.h` | 50 | under 500 |
| `stubs/host_test_clock.cpp` | 29 | under 500 |
| `stubs/host_test_clock.h` | 17 | under 500 |
| `test_app_config.cpp` | 680 | **over 500: planned** |
| `test_cell_net_l2.cpp` | 576 | **over 500: planned** |
| `test_cellnet_ota_receiver.cpp` | 403 | under 500 |
| `test_cip_ethernet_link.cpp` | 95 | under 500 |
| `test_console_parse.cpp` | 175 | under 500 |
| `test_coordinated_grasp.cpp` | 150 | under 500 |
| `test_drivetrain_params.cpp` | 80 | under 500 |
| `test_eip_axis.cpp` | 1807 | **over 500: planned** |
| `test_eip_encoding.cpp` | 478 | under 500 |
| `test_eip_limit_sequence.cpp` | 76 | under 500 |
| `test_eip_run_time_trace.cpp` | 96 | under 500 |
| `test_eip_scanner_policy.cpp` | 118 | under 500 |
| `test_eip_socket_w5500.cpp` | 295 | under 500 |
| `test_eip_transport.cpp` | 1891 | **over 500: planned** |
| `test_gantry_orchestration.cpp` | 1082 | **over 500: planned** |
| `test_hcs01_assembly.cpp` | 193 | under 500 |
| `test_hcs01_comws_client.cpp` | 143 | under 500 |
| `test_k5100_param_access.cpp` | 77 | under 500 |
| `test_kinematics.cpp` | 214 | under 500 |
| `test_limit_switch_policy.cpp` | 109 | under 500 |
| `test_path_profile.cpp` | 405 | under 500 |
| `test_pick_intercept.cpp` | 1095 | **over 500: planned** |
| `test_spi3_critical.cpp` | 78 | under 500 |
| `test_strike_plan.cpp` | 313 | under 500 |
| `test_theta_trace_analysis.cpp` | 158 | under 500 |
| `test_trajectory.cpp` | 56 | under 500 |
| `test_w5500_deferred_log.cpp` | 76 | under 500 |
| `test_w5500_diag.cpp` | 325 | under 500 |
| `test_w5500_poll.cpp` | 129 | under 500 |
| `test_w5500_socket_fastpath.cpp` | 1055 | **over 500: planned** |
| `test_w5500_spi.cpp` | 284 | under 500 |

# Code Encapsulation

**Status:** Live record, 2026-11-26, read from `refac/code_encapsulation`. Update it in the same commit as each split or move (OPEN_ITEMS X-7).

This page records the rules for keeping source files under 500 lines, the splits done, and the plan to move matured features into their own `lib/` components. The library review is in [LIBRARY_REVIEW.md](LIBRARY_REVIEW.md).

## Rules

- **Source files stay under 500 lines.** A larger file is split by what its code does, for robustness and review.
- **A split moves code; it does not change it.** Each split is its own commit:
  - Function bodies are copied verbatim.
  - Every original line is used exactly once.
  - The only edits are to linkage: `static` added or dropped, shared declarations in a private header, default arguments moved to the declaration.
  - A function that is itself too long is broken up in a separate commit afterwards. `14d73e5` does this: `multiAxisTask` calls `multiAxisConnectAndRun`, which calls `multiAxisCyclic`, which calls `multiAxisRecover`.
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
| `ecae185` | `lib/Gantry/src/Gantry.cpp` | 3,336 | 9 files, 240–498 lines |
| `a686b01`, `380e1cc` | `src/gantry_test_console.cpp` | 4,422 | 17 files, 98–463 lines; `processCommand` became a command table |
| `5c45148` | `src/pick_scheduler.cpp` | 2,785 | 9 files, 252–403 lines; the strike path stays in `pick_strike.cpp` |
| `e0a8738` | `lib/AppConfig/src/AppConfig.cpp` | 1,841 | 6 files, 176–398 lines; `loadDefaults()` calls two section functions |
| `de5edfb` | `src/gantry_net_console.cpp` | 555 | 2 files, 182 and 396 lines; login and the recent-peer cache are in `gantry_net_console_auth.cpp` |
| `ac8b46b` | `src/gantry_ota.cpp` | 562 | 2 files, 270 and 316 lines; the TCP OTA server task and `gantryOtaStartServer` are in `gantry_ota_tcp.cpp` |
| `a8530e1`, `8a4a716` | `src/main.cpp` | 615 | 6 files, 53–231 lines; `app_main` calls five stage functions (config, I/O, motion, tasks, network) in `main_init_*.cpp` and `main_tasks.cpp` |
| `1d4e164` | `lib/MCP23S17/src/MCP23S17.cpp` | 546 | 3 files, 66–347 lines; lock, holder logging and register access are in `MCP23S17Bus.cpp` |
| `4d38047` | `lib/Gantry/src/GantryEipRotaryAxis.cpp` | 768 | 3 files, 53–465 lines; `update()` and the trace stamps are in `GantryEipRotaryAxisUpdate.cpp` |
| `06fde1a` | `lib/Gantry/src/GantryEipLinearAxis.cpp` | 955 | 4 files, 50–457 lines; the arm and move phase machine is in `GantryEipLinearAxisPhases.cpp`, alarms and logging in `GantryEipLinearAxisAlarm.cpp` |
| `35efffd` | `lib/CellNetL2/src/CellNetOtaSender.cpp` | 1,209 | 4 files, 87–424 lines; `Link` and the sender class are in `CellNetOtaSenderInternal.h`, the phases in `CellNetOtaSenderPhases.cpp`, reporting in `CellNetOtaSenderReport.cpp` |
| `b3f54fc` | `lib/W5500/src/W5500.cpp` | 731 | 3 files, 16–436 lines; init, reset and recover are in `W5500Init.cpp`. Register, buffer and MACRAW access stay. Waits for an on-cell `eiptiming` and `gaptrace` soak |
| `4656599` | `lib/EtherNetIP/src/EipMultiScanner.cpp` | 829 | 3 files, 13–441 lines; forward open, forward close and reopen are in `EipMultiScannerConnect.cpp`. The exchange stays. Waits for an on-cell `eiptiming` and `gaptrace` soak |
| `dfe4e46` | `lib/W5500/src/W5500Socket.cpp` | 990 | 3 files, 136–490 lines; the TCP path is in `W5500SocketTcp.cpp`. The UDP fast path stays. Waits for an on-cell `eiptiming` and `gaptrace` soak |
| `e9f10b4`, `14d73e5` | `lib/EtherNetIP/src/EipScannerTask.cpp` | 1,329 | 4 files, 160–499 lines; diagnostics are in `EipScannerDiag.cpp`, configuration is in `EipScannerConfig.cpp`. `multiAxisConnectAndRun` calls `multiAxisCyclic`, which calls `multiAxisRecover`. The cyclic helpers stay inline in `EipScannerTaskInternal.h`. Waits for an on-cell `eiptiming` and `gaptrace` soak |
| `28f79ec` | `include/pick_intercept.h` | 1,531 | 7 files, 27–406 lines. The umbrella include is unchanged for callers. Vision, the belt clock, clearance, strike leads, the schedule and the part tracker are the themed headers |
| `8c536cd` | `lib/Gantry/src/Gantry.h` | 597 | 2 files, 144 and 474 lines. Error, status, motion state, bring-up phase and the waypoint queue are in `GantryTypes.h`. `RuntimeParams` was already in `GantryUtils.h`; `RotaryTraceSample` was already in `GantryRotaryAxis.h` |
| `dd7c86b` | `include/coordinated_grasp.h` | 504 | 2 files, 69 and 451 lines. The header has no reference-fixture loader. The bench arm (`Arm`, `armPair`, `fireDue`) is in `coordinated_grasp_arm.h`, included from the model header |
| `31620f5` | `test/host/test_cell_net_l2.cpp` | 576 | 4 files, 57, 138, 162 and 279 lines, one executable. Framing, node dispatch and OTA framing are separate sources. 17 Unity cases |
| `9110af1` | `test/host/test_app_config.cpp` | 680 | 5 files, 87–214 lines, one executable. The shared storage and boot helpers are inline in `test_app_config_internal.h` under `app_config_test`. 28 Unity cases |
| `adde859` | `test/host/test_w5500_socket_fastpath.cpp` | 1,055 | 5 files, 58–360 lines, one executable. `CountingHal` is in `test_w5500_socket_fastpath_internal.h` under `w5500_fastpath_test`. Send, receive and TCP are separate sources. 38 Unity cases |
| `9b92557` | `test/host/test_gantry_orchestration.cpp` | 1,082 | 5 files, 88–362 lines, both executables. The harness is inline in `test_gantry_orchestration_internal.h` under `gantry_orch_test`. 55 Unity cases, and 42 with `CONFIG_GANTRY_THETA_SEQUENTIAL` |
| `8f7616a` | `test/host/test_pick_intercept.cpp` | 1,095 | 6 files, 69–371 lines, one executable. Cell and frame helpers are inline in `test_pick_intercept_internal.h` under `pick_intercept_test`. Belt, geometry, budget and tracker are separate sources. 49 Unity cases |
| `8749b13` | `test/host/test_eip_axis.cpp` | 1,807 | 6 files, 141–454 lines, one executable. Fakes and frame helpers are inline in `test_eip_axis_internal.h` under `eip_axis_test`. Linear, alarm, rotary and units are separate sources. 50 Unity cases |
| `8a13124` | `test/host/test_eip_transport.cpp` | 1,891 | 8 files, 53–449 lines, one executable. Session fakes are inline in `test_eip_transport_internal.h` and the multi-axis rigs in `test_eip_transport_rigs.h`, both under `eip_transport_test`. 50 Unity cases |

## On-cell verification

Image `b6601cf`, flashed over CellNet on 2026-11-27 (slot `ota_0` to `ota_1`, 8.8 s, `emac_missed 0`).

| Run | Length | Result |
|---|---|---|
| Idle, motors off | 4 snapshots over 15 min, up to 28 min after boot | `eiptiming` GO every time, exchange p99 1,129–1,307 µs; `soft_miss`, `sendok_fail`, `chip_recover` and `reconnect` 0; `gaptrace`: no gap of 10 ms or more |
| Motion | 15 min: 46 loops of `test_cycle`'s legs through `move` (414 moves, Z capped at 90 mm), 48 snapshots | every move reached its pose (X and Z within 1.5 mm, theta within 0.1°); GO every time, exchange p99 1,120–1,719 µs; the same counters 0; no gap |

So the Class 1 splits (`b3f54fc`, `4656599`, `dfe4e46`, `e9f10b4`, `14d73e5`) count as done. The boot log and console checks also covered the boot stages (`8a4a716`), the console decoupling (`5355a89`) and the component moves. `test_cycle` itself does not pass on this machine, for reasons that predate this work (OPEN_ITEMS G-E8). Z has no brake, so the `stop` that ends a run can let it fall to A014 (G-A12, G-A13).

## Planned splits

No file is over 500 lines.

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
| `boot_log_ring.*` | Moved to `lib/BootLogRing` | `lib/BootLogRing` |
| `gpio_expander.*` | Moved to `lib/GpioExpander`. G-D3 and G-D6 concern `MCP23S17` and `Spi3Bus`, not this wrapper | `lib/GpioExpander` |
| `theta_trace.*` and `theta_trace_analysis.h` | Moved to `lib/ThetaTrace` | `lib/ThetaTrace` |
| `gantry_ota.*`, `gantry_cellnet_ota.cpp` | Moved to `lib/GantryOta` (N-2 done) | `lib/GantryOta` |
| Test console (17 files) with `gantry_net_console.*` | **Not yet.** G-C7 (the `move` speed decision) is open. Pick commands and `selftest` stay in `src/` and register through `gantryConsoleRegisterCommands`; the console no longer includes `pick_scheduler.h` | `lib/GantryConsole`, once G-C7 is decided |
| Pick scheduler (9 files) | **Not matured.** G-C1, G-C2, G-C11, G-C13 and G-C17 are open | Stays in `src/` |

**Console decoupling** is done, ahead of moving the console:
1. `gantryConsoleRegisterCommands(table, n)` adds a table. Registered tables are searched first.
2. `gantry_console_cmd_pick.cpp` and the `selftest` command stay in `src/` and register from `main`.
3. The console no longer includes `pick_scheduler.h`. Its own headers stay in `src/` until it moves.

**Each move updates:**
- `idf/main/CMakeLists.txt` (drop the `src` entries, add the component to REQUIRES)
- the test paths in `test/host/CMakeLists.txt`
- `CONTRIBUTING.md` (repo map), `docs/INDEX.md` and `lib/README`
- the code pointers in the Knowledge Transfer page and `LOW_LEVEL_GANTRY_CONTROL.md`
- the paths in OPEN_ITEMS

The existing libraries that lack a `README.md` or `REFERENCE.md` get them from the facts in the library review.

## Order of work

1. This record and the library review.
2. The Class 1 splits are done; their on-cell soak passed on 2026-11-27 ([On-cell verification](#on-cell-verification)).
3. Header splits, then test splits.
4. Component moves: `BootLogRing`, `GpioExpander`, `ThetaTrace`, `GantryOta` and the console decoupling are done. `GantryConsole` waits until G-C7 is decided.
5. `README.md` and `REFERENCE.md` for the existing libraries are done.

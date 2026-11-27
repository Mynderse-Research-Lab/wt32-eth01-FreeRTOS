# Library Review

**Status:** Review of every library in `lib/`, 2026-11-26, read from `refac/code_encapsulation` at `1caa98c`.

Every source and header file of each library was read in full, together with the callers and tests that the claims depend on. Each finding carries an evidence label, as in [OPEN_ITEMS.md](OPEN_ITEMS.md):

| Label | Meaning |
|---|---|
| **Confirmed** | Traced in the code; the location is given |
| **Confirmed, rechecked** | Confirmed, and checked a second time against the code before entering this page |
| **Suggested** | An unverified lead. It names what would confirm it |

Findings that need action have an OPEN_ITEMS row; the ID is in the last column. The source split plan for the oversized files is in [CODE_ENCAPSULATION.md](CODE_ENCAPSULATION.md).

## Summary

| Library | Lines | Files over 500 | README | Host tests | Most important finding |
|---|---:|---|---|---|---|
| [W5500](#w5500) | 3,168 | none (split done) | none | 6 files | UDP length wrap hands a 65,535-byte view of a 512-byte buffer to the Class 1 parser (G-B15) |
| [EtherNetIP](#ethernetip) | 7,547 | none (split done) | none | 11 files | Synchronous logs on the Class 1 send path (G-B16) |
| [Gantry](#gantry) | 8,337 | none (split done) | 13 lines, stale | 7 files | A drive fault-stop while X moves toward min re-zeroes X (G-A9) |
| [AppConfig](#appconfig) | 3,181 | none (split done) | none | `test_app_config` (27 cases) | Any NVS read error at boot overwrites the stored set with defaults (G-F7) |
| [CellNetL2](#cellnetl2) | 2,935 | none (split done) | 20 lines | 2 files | OTA sender has no test and is not built in CI |
| [MCP23S17](#mcp23s17) | 755 | none (split done) | 70 lines, stale pin map | none | Shadow cache diverges from the chip after a failed write; reads return 0 on error (G-D6) |
| [Spi3Bus](#spi3bus) | 293 | none | none | `test_spi3_critical` | Two independent locks guard SPI3 (G-D6) |
| [SpiDisplay](#spidisplay) | 2,340 | none | none | none | Reboot prompt restarts without saving; save failures are silent (G-E6) |
| [EthernetLink](#ethernetlink) | 462 | none | none | none | `isUp()` stays true after the cable is pulled in static-IP mode (G-E7) |

## Priority across the libraries

1. **Memory safety on the Class 1 path** (G-B15). `readUdpBatchUnlocked` and `readUdpDatagramUnlocked` compute the datagram size in 16 bits. A header length of 0xFFF8 or more wraps it, so a stuck-high MISO line (every byte read as 0xFF) produces a 65,535-byte view of a 512-byte stack buffer, or a copy of `maxLen` bytes out of a 264-byte scratch buffer.
2. **Stored set lost on a read error** (G-F7). `NvsStorage::getBlob` returns false for any NVS error, and `load()` then writes a fresh set of compile-time defaults over the stored one.
3. **Motion safety in Gantry** (G-A9, G-A10, G-A11). X can be re-zeroed at an arbitrary position. `alarmreset` during a move can leave an axis running while the firmware reports idle. `home t` skips the SAFE_Z band check and `home x` skips the busy check.
4. **Blocking and logging on the Class 1 path** (G-B16, G-B17). Socket-level logs are synchronous in the UDP adapter. The process-image and Class 1 log mutexes are shared with core-0 and lower-priority tasks, the same class of mechanism as G-B6.
5. **Silent clock downshift** (G-B18). `W5500::configureAfterReset` quietly lowers the SPI clock when VERSIONR fails, also inside `recover()`. That hides a fault instead of removing it.
6. **Operator-facing correctness:**
   - NaN is accepted into float parameters (G-F8).
   - The TFT reboot discards unsaved edits (G-E6).
   - Static-IP `isUp()` never clears (G-E7).
   - Stored keys are shown as applied but nothing reads them: `eth_ip`, `hcs01_eng_ip`, `th_sequential`, `cal_timeout_ms` (G-B5, G-F2).

---

## W5500

**Purpose.** Driver for the WIZnet W5500 on SPI2, the bus that carries EtherNet/IP Class 1. It provides:
- chip init, reset and recover
- the hardware TCP and UDP socket layer, including the Class 1 UDP fast path
- one shared bus mutex that times its holds
- a deferred log for real-time paths
- read-only fault diagnostics

`W5500Hal` lets the socket and diagnostics code run on the host against a mock.

**Public API.**

| Header | Contents |
|---|---|
| `W5500.h` | `W5500Config`, class `W5500`: `init`, `recover`, `deinit`, MACRAW, `isLinkUp`, `acquireBus`/`releaseBus` |
| `W5500Hal.h` / `W5500SpiHal.h` | Register and buffer access interface, and its SPI implementation |
| `W5500Socket.h` | `socketOpen`, `socketClose`, `socketConnect`, `socketSend` (TCP, SENDOK up to 50 ms), `socketRecv`, `socketSendTo` (Class 1 send, SENDOK 2 ms or 10 ms after a destination change), `socketRecvFromBatch` |
| `W5500BusMutex.h` | `spiBusMutex()` with hold and wait statistics |
| `W5500DeferredLog.h` | 16 lines × 160 bytes, printed by EipLog |
| `W5500Diag.h` | Failure snapshot, SPI integrity check, destination check |
| `W5500Poll.h` | Poll helpers and the timeout table |

**Threading.**
- One process-wide `std::mutex` guards every socket-layer SPI access.
- The burst scratch buffers (`W5500.cpp:27-28`) are safe only under that mutex.
- `diagMutex()` is a plain `std::mutex`. The scanner takes it every 100 ms and the console takes it too.
- The scanner runs at priority 6 on core 1, the keepalive helpers at 7 on core 1, and EipLog at 2 on core 0.

| # | Category | Location | Finding | Evidence | Item |
|---:|---|---|---|---|---|
| W1 | bug | `W5500Socket.cpp:310-327` | `packetBytes = uint16(8 + payloadLen)` wraps for `payloadLen` ≥ 0xFFF8, so the over-claim check passes and `out[n].len` (up to 65,535) points into the caller's buffer (512 bytes on the stack in `EipMultiScanner::drainBufferedInputs`). `readSettledRxPointers` accepts `avail` = 0xFFFF. With all bytes 0xFF, `packetBytes` is 7 and the loop keeps producing bad views | Confirmed, rechecked | G-B15 |
| W2 | bug | `W5500Socket.cpp:257-284` | Same wrap in the single receive. `memcpy(buf, scratch + 8, maxLen)` reads past the 264-byte stack scratch when `maxLen` > 256, and `RX_RD` advances by only 0–7 bytes | Confirmed, rechecked | G-B15 |
| W3 | bug, not in production | `W5500.cpp:198-291` | MACRAW writes the TX buffer and reads the RX buffer at address 0 without using `Sn_TX_WR`/`Sn_RX_RD`. `recvFrame` underflows on a length below 2. `sendFrame` and `waitForCommand` truncate `esp_timer_get_time()` to 32 bits (every wait expires at once after 71.6 min). Used only by `test/w5500_loopback`. Whether the chip requires the pointer offset is not checked against the datasheet | Confirmed (code) | — |
| W4 | risk | `W5500Init.cpp:176-212` | On a VERSIONR failure, `configureAfterReset` steps the SPI clock down to 5, 2 or 1 MHz and keeps it, including inside `recover()`. Only a boot `ESP_LOGW` reports it | Confirmed | G-B18 |
| W5 | risk | `W5500Diag.cpp:22, 266, 289, 295` | `diagMutex` is a `std::mutex` shared by the scanner and the console. This is the G-B6 pattern, though the hold is about 40 bytes. A spinlock removes it | Suggested: read the waiter name in `gaptrace` | G-B17 |
| W6 | risk | `W5500.cpp:86, 111, 390, 404, 412, 425` | Synchronous `ESP_LOGE` on SPI or length errors runs inside the bus mutex on the Class 1 path; G-B13's closing note does not cover these | Confirmed | G-B16 |
| W7 | risk | `W5500.cpp:22`, `W5500Init.cpp:90` | With DMA on, `max_transfer_sz` is 1,539 while bursts go up to 2,048 bytes. A TCP receive of 1,537–2,048 bytes would exceed the bus limit. Only matters with `eip_spi_dma=1` | Confirmed (code); IDF behaviour Suggested | — |
| W8 | risk | `W5500SocketTcp.cpp:155-161, 205-216` | `socketClose` holds the bus mutex across two 10 ms command waits plus a 200 ms poll, and UDP `socketOpen` across a 500 ms poll. A hang blocks Class 1 for that long | Confirmed | G-B17 |
| W9 | bug | `W5500SocketTcp.cpp:415-421` | TCP `socketRecv` returns -1 on CLOSE_WAIT before reading `RX_RSR`, so a reply followed by FIN is lost | Confirmed | — |
| W10 | risk | `W5500Poll.h:38-54` | Waits of 50 ms or less spin without yielding, and longer ones only `taskYIELD()`. A 1,000 ms TCP receive takes a core from every lower-priority task | Confirmed | G-B17 |
| W11 | docs | `W5500.h:164`, `W5500Socket.h:8-10, 32, 55-57, 91-95, 110-111` | Stale or wrong comments: "factory-burned MAC" (the code writes 02:00:00:00:00:01), "yield" polling, the "all functions lock" claim, the UDP ephemeral port, the -1 return, and "oversized datagrams stay queued" (they are truncated and consumed, unflagged) | Confirmed | — |
| W12 | cleanup | `W5500.h:7-9`, `W5500Init.cpp:136-138`, `W5500.cpp:311`, `W5500SocketTcp.cpp:208` | `extern "C"` around C++; literal 0x002E instead of `REG_PHYCFGR`; `socketClose` indexes without `sock < 8`; a failed `init` leaks the device | Confirmed | — |

**Dead code:** `socketListen`, `socketBind`, `socketIsOpen`, `socketStatus`, `nextEphemeralPort`, `MR_WOL`/`MR_PB`, and the MACRAW path (used only by `test/w5500_loopback`).

**Not covered by host tests:** the 16-bit wrap (W1, W2); MISO stuck high on the receive paths; TCP `socketRecv`/`socketConnect`/`socketClose`; the 61-byte no-DMA chunking; all of `W5500.cpp`. `test_integration_macraw_send` encodes the MACRAW addressing at address 0.

**Actions:**
- Fix W1 and W2 in one commit with tests (all-0xFF reads, `payloadLen` 0xFFF8–0xFFFF on both paths).
- Delete MACRAW and `test/w5500_loopback`. This replaces "MACRAW access stays" in the split plan.
- Write a README: role, the rule never to block under the bus mutex, consumers, tests.
- Write a REFERENCE.md: API per header, the bus-mutex contract, the timeout table, socket buffer layout (4/2/2/2/1/1/1/1 KB), fixed MAC, IP from Kconfig (G-B5), return conventions, deferred-log rules, and the stored keys read in `main_init_motion.cpp` (`eip_spi_khz`, `eip_spi_dma`, `eip_txwr_split`).

---

## EtherNetIP

**Purpose.** EtherNet/IP and CIP originator, in three layers:
- Pure C++ encoders and decoders, host-tested: encapsulation, CPF, Message Router, EPATH, Forward_Open/Close, and the Kinetix 5100 and HCS01 assemblies.
- Session and Class 1 connection logic, host-tested with fakes: `EipSession`, `EipIoConnection`, `EipScanner`, `EipMultiScanner`.
- ESP-IDF-only parts: the W5500 socket adapter, the scanner tasks and the diagnostics.

**Public API (main headers).**

| Header | Contents |
|---|---|
| `EipByteBuffer.h`, `EipEncapsulation.h`, `EipCpf.h`, `CipMessageRouter.h`, `EipConnectionManager.h` | Encoders and decoders. RPI and API in µs |
| `CipEthernetLink.h`, `CipTcpIpInterface.h` | Ethernet Link counters; attribute 13. The NOP interval is timeout/4 with a 250 ms floor |
| `Kinetix5100Assembly.h` | Output 104: speed 0.1 RPM, acceleration 0.1 RPM/s, position PUU. Input 154 |
| `Kinetix5100ParamAccess.h`, `Kinetix5100TuningClient.h`, `KinetixFaultCodes.h` | Kinetix parameter access, tuning and fault codes |
| `Hcs01ControlStatus.h`, `Hcs01Assembly.h` | HCS01 instances 101 and 102 |
| `Hcs01ComwsClient.h` | HTTP COMWS client on port 80 |
| `EipSession.h` | Explicit messages; default timeout 500 ms; `kMaxFrameSize` 512 |
| `EipProcessImage.h` | Command and feedback bridge, behind a mutex |
| `EipMultiScanner.h` | Production scanner: open, drain, exchange, reopen, NOP keepalive |
| `EipClass1Timing.h`, `EipClass1TimingStats.h` | Pacing; stale at 10× RPI; grace 3× RPI; soft-miss policy of 2; 128-sample statistics |
| `EipScannerTask.h` | `startScannerTask`, `pauseClass1`/`resumeClass1`, `dumpClass1TimingStats`, `printLastClass1GapReport` |

**Tasks** (`multiAxisTask` in `EipScannerTask.cpp`, `singleAxisTask` in `EipScannerConfig.cpp`, `class1LogTask` in `EipScannerDiag.cpp`, `keepaliveTask` in `EipScannerTaskInternal.h`):

| Task | Core | Priority | Stack | Notes |
|---|---:|---:|---:|---|
| `EipScannerM` (Class 1 loop) | 1 | 6 | 8192 | |
| `EipHoldKA`, `EipReopenKA`, `EipCloseKA` | 1 | 7 | 4096 | 5 ms or 2 ms keepalive |
| `EipLog` | 0 | 2 | 4096 | 20 and 100 ms jobs |

**Steady-state W5500 sockets:** 3 TCP + 1 UDP receive + 3 UDP P2P transmit = 7 of 8.

| # | Category | Location | Finding | Evidence | Item |
|---:|---|---|---|---|---|
| E1 | risk | `EipSocketW5500.cpp:128-153, 250-259` | `sendTo` logs `ESP_LOGW` synchronously on every failed send, and `p2pTxSockFor` logs on the first send to a destination. Both run inside `exchangeOnce` with the bus acquired and the SPI3 Class 1 critical section set. A failing theta send logs every 2 ms | Confirmed, rechecked | G-B16 |
| E2 | risk | `EipProcessImage.cpp`; `EipClass1DeferredLog.h:49` | The process-image mutex taken in `sendAxisOutput`/`applyFeedback` is also taken from PickScheduler (core 0, priority 5) through `Gantry::drivesOnline`, and on every axis read in the Gantry update. The Class 1 log mutex is shared with EipLog (core 0, priority 2). Either holder, preempted by tcpip or emac_rx, keeps the core-1 scanner waiting. Same mechanism class as G-B6; not measured | Suggested: add hold and wait statistics as `BusMutex` has | G-B17 |
| E3 | risk | `EipScannerTaskInternal.h:305-325` | Every 200 cycles the scanner builds four sorted ring snapshots for an `ESP_LOGD` that `CONFIG_LOG_MAXIMUM_LEVEL=3` compiles out | Confirmed (code); cost Suggested | G-B16 |
| E4 | bug, latent | `EipMultiScannerConnect.cpp:161-166` | `bindSharedUdp()` never joins a multicast group, so `CONFIG_EIP_TO_POINT_TO_POINT=n` would leave every axis stale. Production uses P2P | Confirmed | — |
| E5 | bug | `EipSession.cpp:64-88, 123-155` | One `recv` per transaction with no reassembly against the encapsulation length; neither the reply command nor the sender context is checked. A split reply fails to parse, and a late reply to a timed-out request is taken as the reply to the next request on that session | Confirmed (code); occurrence Suggested: host test with a delayed or split reply | G-B19 |
| E6 | risk | `EipMultiScanner.cpp:175, 188` | The drain applies the oldest T->O frame per axis and discards newer ones read in the same cycle. The test asserts this | Confirmed; impact Suggested | — |
| E7 | risk | `EipScannerTaskInternal.h:327-340, 391-410` | During a per-axis reopen (up to about 6.5 s: TCP connect, RegisterSession up to 5 s, attribute 13, Forward_Open) the keepalive helper never checks X/Z feedback age | Confirmed | — |
| E8 | bug, low probability | `EipScannerTaskInternal.h:373-384` | `stopKeepalive` gives up after 50 ms while the helper still holds a pointer to a context on the caller's stack | Confirmed | — |
| E9 | docs/risk | `EipScannerTask.h:36-41`; `gantry_console_cfg.cpp:13, 97-99` | If the 5 s `pauseClass1` timeout expires after the task broke out for the planned close, Class 1 is already closed yet `cfg save` reports that it did not close | Suggested: time a dropped-session save (G-B1) | — |
| E10 | rule | `Hcs01ComwsClient.h:16` | The console uses the constant `kDefaultEngIp`. The stored `hcs01_eng_ip` has no effect, and no G-B5 row lists it | Confirmed | G-B5 |
| E11 | error handling | `Hcs01ComwsClient.cpp:51-58, 98-186, 332-388` | `runCommand` ignores every `setVar` result. `runAutotune` ignores two arguments and always reports success. HTTP status is never checked and responses truncate at 1,023 bytes | Confirmed | — |
| E12 | error handling | `EipScannerConfig.cpp:119`; `EipScannerTask.cpp:174`; `Kinetix5100ParamAccess.cpp:24-31`; `EipMultiScannerConnect.cpp:261-265` | `acquireBus()` result discarded; empty payload for sizes other than 2 or 4; "CIP status 0x00" logged when the transport failed | Confirmed | — |
| E13 | docs, cross-library | `Kinetix5100Assembly.h:111` vs `GantryEipLinearAxisPhases.cpp:288, 364` | The header calls `actual_speed` RPM, as does the manual text extract (2198-UM004D, Table 106). Gantry's `kNearStopSpeedRef = 50` and `LOW_LEVEL_GANTRY_CONTROL.md:600` treat it as 0.1 RPM. If the manual is right, "near stop" is 50 RPM | Suggested: jog at a known speed reference, compare `actual_speed`, check the manual page | — |
| E14 | docs | `EipScannerTask.h:5-7`, `EipMultiScanner.h:69`, `EipMultiScanner.cpp:135`, `EipTransport.h:111`, `Kconfig:10, 116-128, 198-209`; `LOW_LEVEL_GANTRY_CONTROL.md:498-540` | Stale: theta path, stale multiple (10×, not 3×), frame size (512, not 4 KiB), the MQTT mention, PUU help, run/idle help. The doc's recover-ladder and constants tables disagree with the code: soft misses 2 not 3; 10 s backoff, not a restart; cap 5,000 ms; reconnect idle 500 ms; multiplier 2; reset through MCP PB7 | Confirmed | — |
| E15 | risk | `EipSocketW5500.cpp:32-37` | 7 of 8 W5500 sockets are in use. A second concurrent explicit or COMWS connection fails | Confirmed count | — |
| E16 | cleanup | `EipClass1TimingStats`, `EipMultiScanner.cpp:167`, `KinetixFaultCodes.h:110-115` | Unsynchronised diagnostic counters; a static shared across scanner instances; a latent index past `tmp` on `snprintf` truncation | Confirmed | — |

**Open items checked against the code:**
- G-B2, G-B3, G-B4, G-B5, G-B7, G-B8, G-B9 and G-B10 still match the code. G-B9's option, help text and test should go together.
- The automatic recoveries that exist were not recorded as accepted exceptions to the "eliminate, never mitigate" rule. They are: the HCS01 per-axis reopen, a two-miss ride-through then teardown and chip reset, and the chip-configuration restore. Recording that is the owner's call.

**Dead code:**
- unused API: `buildListIdentityRequest`, `encodeSendUnitDataPayload`, `kOutputInstance106`, `udpBound`, the `EtherNetIP.h` umbrella header
- production-dead: `EipScanner` and `singleAxisTask`, theta-only builds
- unread Kconfig options: `EIP_TARGET_IP`, `EIP_INCLUDE_RUN_IDLE_HEADER`
- four unused parameter IDs
- the stray root-level `autotune_cmd.cpp` (X-5, deleted 2026-11-27)

**Not covered by host tests:**
- the `multiAxisTask` state machine, pause and resume, reopen and planned close
- `Kinetix5100TuningClient`
- COMWS `runCommand`, `runAutotune` and HTTP errors
- `formatDriveTripSummary`
- a split or late explicit reply
- multicast
- two frames per axis in one cycle

**Actions:**
- Fix E1 and E3 (defer or remove).
- Instrument E2 before changing it.
- Fix E5 with a test.
- Write a README (layers) and a REFERENCE.md: API with units, task table, lock inventory (and which locks the Class 1 path may take), timing constants, the recovery ladder as built, Kconfig options with the AppConfig keys meant to replace them, `ExchangeStatus`, and the console commands served (`eiptiming`, `gaptrace`, `ethlink`, `drv`, the `cfg save` pause).
- Splits as planned, each followed by a soak.

---

## Gantry

**Purpose.** The only interface application code uses for motion:
- X and Z on Kinetix 5100 and theta on HCS01, over EtherNet/IP.
- X-Z paths planned against the SAFE_Z band, with theta scheduled along them.
- The home, calibrate and bring-up sequences.
- The strike primitives: held Z start, queued retract, retract chaining.
- The gripper, kinematics, and alarm and position-loss checks.

**Public API.**

| Header | Contents |
|---|---|
| `Gantry.h` | `Gantry`: lifecycle; configuration (`setJointLimits`, `setSafeZHeight` mm, `setRuntimeParams`); motion (`moveTo` with mm/s, deg/s, mm/s², deg/s², `preloadJointMove`, `releasePreloadedMotion`, `retractToSafeZ`, `queueRetractToSafeZ`, `chainMoveAfterRetract`, `requestAbort`, `update`); homing (`homeX/Z/Theta`, `calibrateX/Z/Theta`, `startEipBringUp`); status and lock-free trace getters; `grip` |
| `GantryTypes.h` | `GantryError`, `HomingStatus`, `GantryStatus`, `MotionState`, `BringUpPhase`, `Waypoint`, `WaypointQueue` |
| `GantryConfig.h` | `JointConfig {x mm, z mm (+down), theta deg}`, `JointLimits`, `EndEffectorPose` |
| `GantryUtils.h` | `RuntimeParams`, constants |
| `GantryKinematics.h` | Forward and inverse kinematics; theta pose = joint + 90° |
| `GantryTrajectory.h`, `GantryPathProfile.h` | Trapezoid profiles, path segments, theta window, retract lead |
| `GantryEipLinearAxis.h`, `GantryEipRotaryAxis.h` | The two axis classes |
| Policy headers | Limit switch, end effector, theta origin, control period |

**Threading.**

| Caller | Priority | Core | Calls |
|---|---:|---:|---|
| `GantryUpdate` | 5 | 1 | `update()` every 10 ms |
| `PickStrike` | 6 | 1 | `releasePreloadedMotion`, `queueRetractToSafeZ`, `grip` |
| `PickScheduler` | 5 | 0 | Motion calls |
| Trace tasks | 4 | 0 | Trace getters |
| TFT | 2 | 0 | Reads |
| Console and workers | 2–3 | 0 | Console commands, calibration |

- One `std::recursive_mutex` guards every public method. `update()` holds it for the whole tick.
- Each axis read copies the feedback vector under the process-image mutex (see E2).

| # | Category | Location | Finding | Evidence | Item |
|---:|---|---|---|---|---|
| A1 | bug, motion safety | `GantryAxisState.cpp:305-319`; `GantryEipLinearAxis.cpp:401-413`; `Kinetix5100Assembly.h:138` | While no sequence runs and X is moving toward min (pulse count decreasing), an active min switch triggers `axisX_->setCurrentPulses(0)`. With drive-managed limits, `isLikelyLimitStop()` (any fault with the stopped bit) marks both min and max active when neither A014 nor A015 is present. So a fault-stop during a −X move re-zeroes X wherever it stopped, and the workspace latch stays set | Confirmed, rechecked (code path); field occurrence Suggested: host test with fault+stopped while the count decreases, bench check after an X fault | G-A9 |
| A2 | bug | `Gantry.cpp:405-418`; `GantryEipLinearAxisAlarm.cpp:25-40` | `clearAlarm` calls `stopAllMotion()` and each axis `clearAlarm()` in the same tick. The linear axis clears `motion_commanded_` and publishes the settle image with FaultReset, which A603s against a running Absolute move, so a healthy axis may keep moving while the firmware reports idle. The rotary axis goes DriveOff mid-move | Confirmed, rechecked (sequence); drive reaction Suggested: `alarmreset` during a slow X move | G-A10 |
| A3 | risk | `GantryMotion.cpp:191-195, 234` | `retractToSafeZ` writes a stop image and overwrites it with the new preload in the same call, so StopMotion probably never reaches the wire | Suggested: `ztrace` of a retract during a slow descent | — |
| A4 | risk, motion safety | `GantryHoming.cpp:108-144`; `gantry_console_homing.cpp:248-257` | `homeTheta` calls `moveToDeg(0)` with no SAFE_Z band check and no busy check, and `captureSoftHome` resets an in-flight theta move | Confirmed, rechecked | G-A11 |
| A5 | risk | `GantryHoming.cpp:35-52`; `gantry_console_homing.cpp:116-150` vs `:196-200` | `homeX` has no busy guard and the console `home x` lacks the busy check that `home z` has; `update()` then runs the home sequence and the path state machine on X together | Confirmed, rechecked | G-A11 |
| A6 | docs | `GantryPathTheta.cpp:203-216`, `GantryPath.cpp:174-187` vs `Gantry.h:19-20, 127, 182` | The early retract lead moves X and theta below the band; the headers say they move only in the band (G-C14) | Confirmed | G-C14 |
| A7 | risk | `GantryPath.cpp:45-47`; `GantrySequential.cpp:124-143` | With X deferred, a path whose Z goes idle outside the band without an alarm stays busy forever: there is no overall path timeout | Confirmed (logic); trigger Suggested: host test | — |
| A8 | bug | `GantrySequential.cpp:56-79`; `GantryMotion.cpp:101-102` | `startSequentialMotion()` returns `void`; a failed arm or a refused theta interlock still makes `moveTo` return `OK`. `test_orchestrator_bubbles_up_axis_rejection` asserts `OK` | Confirmed, rechecked | — |
| A9 | bug | `GantrySequential.cpp:72`, `GantryPath.cpp:264` | A failed grip in the pose-move PnP sequence still proceeds to retract | Confirmed | — |
| A10 | risk, real-time | `GantryMotion.cpp:475-486`; `GantryEndEffector.cpp:56-89` | `grip()` holds the Gantry mutex across two MCP SPI transactions, each of which can wait about 64 ms for the MCP lock plus an unbounded bus wait. It is called from PickStrike between the Z edge and the retract edge, and blocks the Gantry update for that time | Confirmed (hold path); magnitude Suggested: G-D3 hold log | G-D3 |
| A11 | risk | `GantryEipLinearAxisInternal.h:48`; `GantryEipLinearAxisPhases.cpp:375`; `GantryEipHomeCal.cpp:231`; `ConfigSchema.cpp:184-187` | Fixed 15 s move and 20 s phase timeouts conflict with the stored speed ranges. A 550 mm seek needs at least 37 mm/s; a 180° orient below 9 °/s times out | Confirmed by arithmetic | — |
| A12 | rule | `GantrySequential.cpp:83`, `GantryMotion.cpp:371`; `GantryHoming.cpp:176`; `Gantry.cpp:91` | `CONFIG_GANTRY_THETA_SEQUENTIAL` and the 120 s calibration timeout are compile-time, while the stored `th_sequential` and `cal_timeout_ms` are unread; the bring-up X park and the 90° orient have no stored key | Confirmed | G-F2 |
| A13 | bug | `GantryEipBringUp.cpp:343-349, 481-488`; `GantryEipHomeCal.cpp:389-406` | Bring-up X park, band return and the calibrate return to zero accept "not busy" as success without checking arrival, so a timed-out or aborted move still latches the workspace | Confirmed | — |
| A14 | bug, minor | `GantryEipBringUp.cpp:79-89` | `failEipBringUp` does not stop theta; a phase timeout during the orient leaves it moving | Confirmed | — |
| A15 | risk | `GantryEipLinearAxis.cpp:312-322`; `Gantry.cpp:177-200` | `stopMotion()` on an idle linear axis publishes a Position-mode image with StopMotion to a drive in settle mode and holds busy for 200 ms. `enable()` has no busy guard and `disable()` replaces the stop image with ServoOff in the same call | Confirmed (code); consequence Suggested: watch for A603 after `stop` with the axes idle | — |
| A16 | risk | `GantryEipLinearAxisInternal.h:44`; `GantryEipLinearAxisPhases.cpp:263-278`; `GantryEipRotaryAxisInternal.h:40`; `GantryEipRotaryAxisUpdate.cpp:159-166` | Per-tick thresholds (0.005 mm, 0.015°) do not scale with the control period that Kconfig allows down to 2 ms | Suggested | — |
| A17 | risk | `GantryEipBringUp.cpp:320, 455`; `GantryEipHomeCal.cpp:183` | Measured strokes are rounded to whole mm and feed `x_max` and the kinematic `x_stroke_mm`, so the world-to-joint X map shifts by up to 0.5 mm with every calibration | Confirmed (code); impact Suggested | — |
| A18 | docs | `Gantry.h:85, 150-158, 211-212`; `GantryRotaryAxis.h:96-99, 116`; `GantryThetaOrigin.h:1-15`; `GantryUtils.h:23-24`; `GantryConfig.h:12-14, 43, 111-112`; `GantryKinematics.h:9`; `CHANGELOG.md` | Stale: the aligned/unaligned origin with a shrunk thetalim (the code always uses ±36000, G-C15); settled tolerance 0.1° (code 0.5°); "retract stops motion first"; MCP23S17 "removed"; lock-free trace; the band formula; the Z pose formula (code: `pose.z = z_stroke − joint.z`); CHANGELOG home speeds, PUU and safe Z | Confirmed | — |
| A19 | cleanup | `GantryEndEffector.cpp:10, 62, 69`; `GantryLimitSwitch.cpp:5, 12` | Gantry calls `gpio_expander_*`. The wrapper is `lib/GpioExpander/src/gpio_expander.c`, and Gantry requires the `GpioExpander` component | Confirmed | — |

**Open items checked against the code:**
- G-C14, G-C15 and G-C8 still match.
- G-F4 does not hold on the production path. `x_hard_span_mm` is read only in the unreachable GPIO calibrate branch (`GantryHoming.cpp:444-447`); what `x_lim_max_mm` does is seed `x_max` and the kinematic `x_stroke_mm` at boot. The row can close when the GPIO path is deleted.

**Dead code:**
- unused types and methods: `GantryStatus`, `HomingStatus`, `Waypoint`, `WaypointQueue`, `TrajectoryWaypoint`, the empty `GantryConfig.cpp`, `home()`, `setLimitPins()`, `setHomingSpeed()`, steps-per-revolution, the legacy pulse `moveTo`, `startEipPrecisionCalibrate`, `BringUpPhase::kZEnterBand`, and the `GANTRY_DIAG_SKIP_*` toggles
- the unreachable GPIO limit path (`GantryHoming.cpp:54-85, 290-300, 330-460`)
- PulseMotor and MQTT comments

The legacy limit block in `updateAxisPositions` is not dead: it is A1.

**Not covered by host tests:**
- the EIP home/calibrate sequence through Gantry, `homeTheta` and `calibrateTheta`
- `clearAlarm`, `enable` and `disable` during motion
- the drive-managed limit mirror and the re-zero case (A1)
- the deferred-X hang (A7)
- a 2 ms control period

**Actions:**
- Fix A1, A2, A4, A5, A8, A11, A13 and A14, each with a host test.
- Delete the GPIO limit path, which also closes G-F4.
- Rewrite the README: purpose, file map of the split sources, links.
- Write a REFERENCE.md:
  - public API with units and preconditions
  - the state machines and what aborts each one
  - threading and the rule that nothing blocks under the mutex
  - timing: 10 ms tick, timeouts, 160 ms leads
  - `RuntimeParams` with their AppConfig keys and the remaining compile-time values
  - error codes as actually returned
  - console commands served
  - the A014/A015 map and the coordinate frame
- Splits as planned.

---

## AppConfig

**Purpose.** The single stored runtime-parameter set:
- values start from compile-time defaults and are overlaid from one NVS blob
- they are edited in RAM through typed or text access, with schema bounds and a developer lock
- they are written back only by an explicit save that the caller makes Class 1-safe
- old per-key NVS values and renamed keys are migrated

**Public API.**

| Header | Contents |
|---|---|
| `AppConfig.h` | `instance()`, `init`, `load`, `commitPending`, `saveSet`, `requestSave`/`setSaveHandler`, `factoryReset` (RAM only), `activeSet`, `pendingChanges`, `data()`, developer lock, typed get/set, `isParamVisible`, `formatParamValue`, `applyParamText` |
| `AppConfigData.h` | 139 fields, with units in their names |
| `ConfigSchema.h` | Key ≤ 15 characters, label, type, access, reboot flag, range, units, description; renamed keys |
| `ConfigSet.h` | 24-byte header (`'GCFG'`, seq, unix time, FNV-1a hash, length, count, format 1); records; 2,048-byte cap |
| `ConfigStorage.h`, `NvsStorage.h`, `MemoryStorage.h` | Storage interfaces and the NVS and in-memory backends |

**Persistence:**
- One NVS key (`cfg_set`) with a sequence number; there is no set rotation.
- Surviving a power cut mid-save depends on NVS replacing a blob atomically (Suggested: confirm from the ESP-IDF NVS documentation or a power-cut test during `cfg save`).
- Today's 148 keys take about 1.4–1.7 kB of the 2,048-byte cap.

**The four key lists** (defaults, `readParams`, `writeParams`, typed get/set) agree: same keys, same types. The only intentional exception is `dev_pw`, which is stored but not in the schema. Confirmed by a key-by-key comparison.

| # | Category | Location | Finding | Evidence | Item |
|---:|---|---|---|---|---|
| C1 | bug, persistence | `NvsStorage.cpp:96-99`; `AppConfig.cpp:100-135` | `getBlob` returns false for any NVS error, not only "not found". `load()` then treats the set as absent and writes a fresh set of defaults plus any old per-key values over the stored one. `main.cpp` promises parameters revert only on an operator command | Confirmed, rechecked | G-F7 |
| C2 | risk | `main_init_io.cpp:31-35` | With a damaged set, or no storage, the machine runs on compile-time defaults that do not match it (G-F1) and only a log line says so; nothing holds motion off | Confirmed (no gate); consequence Suggested | G-F1 |
| C3 | bug | `AppConfigText.cpp:162-168`; `AppConfigAccess.cpp:227` | `strtof("nan")` gives NaN, and the range check is false for NaN, so NaN is stored in any float key and later saved, from `cfg` or a parameter-file replay | Confirmed, rechecked | G-F8 |
| C4 | rule | `AppConfigDefaults*.cpp`, `AppConfigData.h` | Three copies of the defaults. The host fallbacks disagree with `sdkconfig.defaults`; for example X speed 3333.3 vs 500, theta accel 1800 vs 18000, LAN IP 192.168.1.100 vs 10.42.0.100. About 30 defaults exist only in C++, against the rule that `sdkconfig.defaults` is the only compile-time default source | Confirmed | — |
| C5 | docs/risk | `ConfigSchema.cpp:194-250` | Network, console, OTA and EtherNet/IP keys are editable and marked "reboot required", yet nothing reads them (`eth_ip`, `con_port`, `ota_port`, `con_pw`, `w5500_ip`, target IPs, RPIs, PUU) | Confirmed | G-B5, G-F2 |
| C6 | docs | `AppConfigText.cpp:92` | `formatParamValue` prints floats with `%.2f`, so `cfg <key>` and the TFT show 0.005 as 0.01; the export uses `%.7g` | Confirmed | — |
| C7 | risk | `ConfigSet.h:180-181` | The reader stops at the first unknown record tag, so a newer format without a version bump would make older firmware drop the rest of the set and rewrite it | Suggested | — |
| C8 | risk | `AppConfig.h:93-98` | `data_` is read by several tasks without a lock; `factoryReset()` or `load()` rewrite the whole struct while readers run | Suggested | — |
| C9 | cleanup | `AppConfigParamIo.cpp:149-150, 307-308`; `CMakeLists.txt:16` | Two empty `for` loops; unused `mutableData()`; an unneeded `${APP_ROOT}/include` export; two keys for the W5500 clock (`w5500_spi_hz` unread, `eip_spi_khz` read) | Confirmed | — |

**Open items checked against the code:**
- G-F1 still holds. It has a second path: the TFT menu runs `factoryReset()` then `requestSave()` in one action, with no warning to download the parameters first (`UiManager.cpp:130-132`).
- G-F2: the substance holds, but a name search counts 56 unread fields, not "about 68". The row should state its counting method.
- G-F3 still holds: 31 DEVELOPER keys are left out unless unlocked; `con_pw` and `dev_pw` are always left out.
- G-B5 is confirmed.

**Not covered by host tests:**
- an NVS read error other than "not found" (C1)
- NaN and infinity (C3)
- a malformed payload given straight to the reader
- hash collisions including `dev_pw` and the old renamed keys
- set size with maximum-length strings

**Actions:**
- Fix C1 and C3 with tests.
- Decide whether a damaged set holds off motion (C2).
- Generate the defaults and the four key lists from one table. That removes C4 and the hand-kept duplication; it is a design decision for the project lead.
- Write a README and a REFERENCE.md:
  - the set format
  - the boot, save and confirm life cycle
  - threading
  - a generated key table: key, type, units, range, access, reboot flag, default source, reader or "unread"
  - the `cfg` commands served

---

## CellNetL2

**Purpose.** Raw Ethernet (EtherType 0x88B5) framing, parsing and dispatch for the cell nodes:
- a transport interface with an ESP-IDF EMAC implementation and a Linux AF_PACKET one
- the OTA receiver the gantry runs in OTA mode
- the Linux OTA sender used by the supervisor and `tools/cellnet_ota_send.cpp`

The message structs are in `include/cell_net_l2_protocol.h`.

**Built into the firmware:** `CellNetL2Framing`, `CellNetL2`, `CellNetOtaReceiver` and `EspEthL2Transport`. The OTA sender (`CellNetOtaSender.cpp`, `CellNetOtaSenderPhases.cpp`, `CellNetOtaSenderReport.cpp`, `CellNetOtaSenderInternal.h`) and `LinuxRawSocketL2Transport.h` are Linux-only and are compiled neither in the firmware nor in CI.

**Threading.**
- `rxHook` runs in `emac_rx` (priority 15, core 0) and calls the user callbacks synchronously.
- `stats_mutex_` is taken on the receive path and by the console.
- `sequence_++` is not atomic, while sends come from several tasks.

**Protocol robustness:**
- `parseFrame` checks: length ≥ 22, EtherType, version, the expected length within 64 bytes (508 for OTA_DATA), and the OTA data length against the frame. The payload copies are fixed size; no out-of-bounds access was found.
- The OTA receiver's bounds hold:
  - the fragment index is checked before it is used
  - 128 fragments per block at most
  - copies stay within 4,096 bytes
  - timeouts are wrap-safe

| # | Category | Location | Finding | Evidence | Item |
|---:|---|---|---|---|---|
| N1 | risk | `EspEthL2Transport.cpp:65, 73` | The receive hook and the undersized-frame register bit depend on the IDF version and may be lost if EthernetLink stops and restarts the driver | Suggested: check the hook and the bit after `esp_eth_stop`/`start` | — |
| N2 | bug, diagnostic | `CellNetL2.cpp:219-225` | A duplicate or reordered frame adds 255 to `rx_sequence_drops` | Confirmed | — |
| N3 | risk | `CellNetL2.cpp:79` and the other senders | `sequence_++` from several tasks can produce duplicate sequence numbers, which trigger N2 at the receiver | Confirmed | — |
| N4 | risk | `CellNetL2.h:96-100`; `EspEthL2Transport.cpp:102-104` | Callback assignments race with dispatch in `emac_rx` if they happen after the hook is live; boot order keeps them apart today | Suggested | — |
| N5 | bug, supervisor | `LinuxRawSocketL2Transport.h:40-49` | The destructor closes the socket before joining the receive thread; a second `begin()` calls `std::terminate`. Consistent with the N-2 note that stopping the service mid-update can crash it at exit | Confirmed | N-2 |
| N6 | bug, supervisor | `LinuxRawSocketL2Transport.h:148-152` | The Linux transport receives its own outgoing frames (N-2 note) | Confirmed | N-2 |
| N7 | test gap | `CellNetOtaSender.cpp`, `CellNetOtaSenderPhases.cpp`, `CellNetOtaSenderReport.cpp` | The sender has no host test and is not compiled in CI. A loopback test against `OtaReceiver` needs an interface for `Link` | Confirmed | — |
| N8 | docs | `IL2Transport.h:48-50`; `cell_net_l2_protocol.h:5`; `CellNetL2.cpp:33-34` | `isLinkUp` claims link state but means only that a handle exists; an unmeasured "sub-50-microsecond" claim; a comment on undersized frames that contradicts the transport | Confirmed | — |
| N9 | cleanup | `CMakeLists.txt`; `CellNetL2Framing.cpp:157-220`; `lib/CellNetL2/Untitled` | Unused `freertos` REQUIRES; `ERR_BUSY` never sent; five near-identical frame builders; no `static_assert` on wire sizes; the stray untracked `Untitled` file | Confirmed | X-5 |

**Open items checked against the code:**
- N-1 holds: there is no time message.
- N-2's notes match the code.
- C-3 holds, but its numbers are stale: the conveyor copy now differs from `include/cell_net_l2_protocol.h` in 115 lines. The conveyor's own frames still parse (Suggested: confirm with a byte-for-byte frame test).

**Actions:**
- Extend the README: which sources are firmware-only and which Linux-only; callbacks run in `emac_rx` and must not block or log; the sender is not built in CI.
- Write a REFERENCE.md:
  - wire format summary pointing at the guide
  - threading per transport
  - timing: 32-bit µs timestamp wrapping every 71.6 min
  - statistics semantics
  - OTA state machine and deadlines
  - sender phases and exit codes
  - EMAC settings depended on
- Split the sender as planned, with a `Link` interface and a loopback test in CI.

---

## MCP23S17

**Purpose.** C driver for the MCP23S17 16-bit SPI GPIO expander. It caches IODIR, GPPU and OLAT, serialises access with a per-handle mutex, defers to Class 1 critical sections through Spi3Bus, and logs lock holders (G-D3).

**Public API (`MCP23S17.h`).**
- `mcp23s17_init` / `mcp23s17_deinit`
- per-pin direction, pull-up, write and read
- `mcp23s17_read_latch`, `mcp23s17_verify_config`
- port write and read
- `mcp23s17_set_pin_interrupt`
- debug register read and write

**Threading.**
- **Callers:**

  | Caller | Priority | Core | Access |
  |---|---:|---:|---|
  | PickStrike | 6 | 1 | through `grip` |
  | PickScheduler | 5 | 0 | through `grip` |
  | TFT task | 2 | 0 | port-B reads at 100 Hz, DC/RES writes |
  | Console | 3 | 0 | commands |
  | Health check | — | — | verify |
- **Lock timing:** `mcp23s17_lock` makes up to 4 attempts, each waiting up to 8 ms for Class 1 to go idle and then 8 ms for the mutex. A waiter gives up after about 64 ms.
- **Unbounded hold:** the lock is held across `spi_device_polling_transmit`, which waits for the ESP-IDF bus lock with `portMAX_DELAY` (esp-idf v6.0 `spi_master.c`). A holder is therefore not bounded by the timeout.

| # | Category | Location | Finding | Evidence | Item |
|---:|---|---|---|---|---|
| M1 | risk | `MCP23S17Internal.h:45`; `MCP23S17.cpp:26`; `Spi3Bus.cpp:15, 67-93` | Two unrelated locks guard SPI3: the gripper, inputs and health check take only the MCP mutex, while the TFT takes Spi3Bus's mutex (and then the MCP one for DC writes). No caller takes them in reverse order, so no deadlock; an MCP transfer may land between a TFT chip-select assert and its data | Confirmed (two locks); display corruption Suggested | G-D6 |
| M2 | risk | `MCP23S17Bus.cpp` lock path | A holder is not bounded by the 8 ms timeout, because the bus-lock wait inside the transfer is unbounded; this fits the 131 ms PickScheduler hold in G-D3 | Confirmed from IDF v6.0 source | G-D3 |
| M3 | bug | `MCP23S17.cpp:151-157, 174-180, 197-203, 259-265` | The shadow cache is updated before the write is known to succeed; after a failed write, the next write to another pin of that port writes the wrong bit | Confirmed, rechecked | G-D6 |
| M4 | bug | `MCP23S17.cpp:242-244, 278-280` | `read_pin` and `read_port` return 0 on any error, which for the active-low inputs looks like "active" | Confirmed, rechecked | G-D6 |
| M5 | risk | `MCP23S17.cpp:98-115, 103-104`; `lib/GpioExpander/src/gpio_expander.c:104-108` | `init` does not check the IODIR and GPPU writes, returns a handle after a failed read-back, and does not write OLAT first, so after an ESP-only reset a DOUT pin (for example the gripper) drives its old level for one transaction | Confirmed | G-D6 |
| M6 | bug, consumer | `gantry_console_io.cpp:166, 183, 186` | `ESP_ERROR_CHECK` around the debug register calls reboots the controller on a lock timeout | Confirmed | G-D6 |
| M7 | cleanup | `MCP23S17Bus.cpp:155-160` | Debug register writes bypass the caches, so writing IODIR or OLAT that way makes `verify_config` fail | Confirmed | — |
| M8 | docs | `README.md`; `MCP23S17.h:51`; `Spi3Bus.h:13` | Stale channel map. The real map (`gantry_app_constants.h:44-71`): TFT CS on ESP GPIO32; PB0 KO, PB1 PUSH, PB2 ENC_B, PB3 ENC_A, PB4 BLK unused, PB5 DC, PB6 RES, PB7 W5500 RSTn. The header says the default clock is 10 MHz; the code uses 1 MHz. `set_pin_interrupt` is described as edge-triggered, but it sets INTCON (compare against DEFVAL) | Confirmed; interrupt semantics Suggested (MCP23S17 datasheet INTCON/DEFVAL) | — |

**Dead code:** `mcp23s17_write_port`, `mcp23s17_set_pin_interrupt`, and the IPOL, INTF and INTCAP defines.

**Not covered by host tests:** the whole driver. Testable with a stubbed SPI transmit: pin-to-register mapping, cache behaviour on failure, a `verify_config` mismatch, the lock retries.

**Actions:**
- Fix M3–M6.
- Use one SPI3 lock for the MCP driver and the TFT, as part of the G-D3 decision.
- Rewrite the README pin map from `gantry_app_constants.h`.
- Write a REFERENCE.md: return semantics, the lock algorithm and its worst case, the advisory Class 1 deferral, the cache model, and the callers.

---

## Spi3Bus

**Purpose.**
- Owns SPI3, shared by the MCP23S17 (own chip select) and the ST7789 TFT.
- Provides `withMcp`/`withTft` helpers, which take a bus mutex after waiting for the Class 1 critical section to clear.
- Provides the Class 1 critical flag the scanner sets around each exchange, with an event group so waiters wake on the exit edge.

**Public API.**
- `spi3::init`, `isReady`, `host`, `mutex`
- `class1CriticalEnter/Exit/Active`, `waitClass1Idle`
- `withMcp` and `withTft`: return `ESP_ERR_TIMEOUT` after the 8 ms Class 1 budget or a 50 ms mutex wait
- C wrappers for the critical flag
- `Spi3BusCritical.h`: the atomic nesting counter

| # | Category | Location | Finding | Evidence | Item |
|---:|---|---|---|---|---|
| S1 | risk/docs | `Spi3Bus.cpp:102`; `MCP23S17.cpp:26` | `withMcp` does not exclude the MCP driver's own accesses, which use a separate mutex (M1). The exported `mutex()` suggests one bus lock exists; there are two, which the G-D3 priority-ceiling design must know | Confirmed | G-D6 |
| S2 | docs | `Spi3Bus.h:15-16` | Only the start of an SPI3 operation is deferred during a Class 1 exchange; a running operation continues (a full TFT fill takes about 61 ms at 20 MHz, calculated). The W5500 is on SPI2, and nothing records why SPI3 must defer at all. This deferral is itself a source of MCP lock failures | Confirmed (start-only); need for the deferral Suggested: soak with it disabled and compare `gaptrace`/`eiptiming` | G-D3 |
| S3 | risk | `Spi3Bus.h` (`std::function` parameters) | Lambdas capturing three words probably exceed the `std::function` small-buffer size on Xtensa, so every TFT data write may allocate on core 0 | Suggested: check `sizeof(std::_Any_data)` on target, or trace the heap | — |
| S4 | cleanup | `Spi3Bus.cpp:80, 91`; `CMakeLists.txt:6`, `Spi3Bus.cpp:4` | Failure logs are not rate-limited; the library includes an app header for pin numbers | Confirmed | — |

**Dead code:** `mutex()` and `deinit()`.

**Actions:**
- Write a README: who shares SPI3, the two locks, the deferral rule and its reason.
- Write a REFERENCE.md: the lock inventory (Spi3Bus mutex, MCP mutex, IDF bus lock), the nesting order (Spi3Bus mutex before the MCP mutex, never reversed), the timing budgets, and the G-D3 ceiling once decided.
- Add host tests for the retry logic behind a fake event group.

---

## SpiDisplay

**Purpose.** ST7789 320×240 TFT driver on SPI3 and the UI on top of it:
- dashboard, parameter menu built from `ConfigSchema`, value and password editors, reboot prompt, About screen
- input from an encoder and two buttons on MCP port B
- driven only from `TftUiTask` (core 0, priority 2)

The TFT stays enabled; nothing here proposes otherwise.

**Public API.**
- `St7789Driver`: init, orientation, drawing primitives, 8×16 text
- `UiInput`: quadrature, 4 transitions per detent, 2-sample debounce
- `UiDashboard`, `UiMenu`, `UiEditor`, `UiRebootPrompt`
- `UiManager` singleton: `update` at 15 Hz, `pollInput` at 100 Hz

| # | Category | Location | Finding | Evidence | Item |
|---:|---|---|---|---|---|
| D1 | bug | `UiRebootPrompt.cpp:56-66`; `UiManager.cpp:158-161` | After editing a "reboot required" parameter, YES on the reboot prompt calls `esp_restart()` without saving and without checking `pendingChanges()`, so the edit is lost; the console refuses the same reboot when changes are pending | Confirmed, rechecked | G-E6 |
| D2 | bug | `UiManager.cpp:127, 132`; `AppConfig.cpp:203-208` | "Save Parameters" and "Factory Reset" discard the `requestSave()` result, so a refused save (motors on) or a failed Class 1 pause shows nothing. Factory Reset replaces the running values at once, whatever the motion state | Confirmed; motion impact Suggested | G-E6 |
| D3 | bug | `St7789Driver.cpp:64-66, 85-87, 259-261, 348-349, 387-388, 164-167` | Every driver write discards its result. A dropped command or DC change makes the panel read data as commands, or the reverse. `init()` reports success even when commands failed | Confirmed | — |
| D4 | bug | `UiManager.cpp:54-58, 213-219`; `UiDashboard.cpp:49-63`; `UiEditor.cpp:93-173`; `St7789Driver.cpp:314-327` | Portrait orientation is selectable, but the layouts assume a 320 px width; characters at the right edge fall into a per-pixel path costing about 6 SPI3 lock cycles per pixel | Confirmed (layout); cost Suggested | — |
| D5 | bug, UX | `UiEditor.cpp:15-16, 217-224, 259, 303` | The string editor cannot change length (`10.42.0.100` cannot become `10.42.0.50`); an empty string makes the cursor advance forever; characters outside its set become `1` | Confirmed | — |
| D6 | docs/consumer | `main_tasks.cpp:73-74, 85, 95` | The dashboard "[H]" homed markers show `!isBusy()`; "Plant LAN UP" comes from `isUp()` (G-E7); the IP fallback shows the stored `eth_static_ip`, which the link does not use | Confirmed | G-E7 |
| D7 | cleanup | `UiManager.cpp:34, 40, 44, 77-78` | LEDC calls unchecked; deprecated `ledc_channel_config_t::intr_type` at line 40 | Confirmed | — |
| D8 | docs | `SpiDisplay.h:11-13`; `UiManager.cpp:142-143` | The header says the TFT chip select is an MCP pin (it is an ESP GPIO); the About screen hard-codes "ESP32-WROVER / Rev 3" and "v1.0.0 (Production)" | Confirmed; module name Suggested | — |

**Dead code:** the `SpiDisplay` class (only its config struct is used), `UiManager::getDriver`, `UiEditor::getCurrentParamKey` and `getPasswordResult`, the MCP software-PWM backlight branch, and the two inverted orientations.

**Actions:**
- Fix D1 and D2 by reusing the console's reboot and save rules.
- Check results in D3 and redraw on failure.
- Remove portrait or lay out from the screen width (D4).
- Write a README: screens, input mapping, the single-task contract.
- Write a REFERENCE.md: the call contract, lock use and cost per primitive, the keys read, the actions triggered and their Class 1 and unsaved-change rules.
- Add host tests for `UiInput`, `UiEditor` and `UiMenu`.

---

## EthernetLink

**Purpose.**
- Brings up the LAN8720: ESP32 EMAC, RMII with external clock on GPIO0, crystal enabled on GPIO16.
- Configures a static IP or DHCP, tracks link and IP state from IDF events, and records boot milestones for `boottime`.
- This link carries CellNet L2, the TCP console and OTA.

**Public API:** `EthernetBootTimes` and `ethernetBootTimes()`; `enablePhyOscillator()` (1,000 ms settle); `start()`; `waitForUp(ms)`; `isUp()`; `getEthHandle()`; `getNetif()`.

**Threading:**
- No tasks of its own; the handlers run in the default event loop.
- `emac_rx` is pinned to the core that calls `start()`, which is core 0 (`app_main` affinity), consistent with G-B10.

| # | Category | Location | Finding | Evidence | Item |
|---:|---|---|---|---|---|
| L1 | bug | `EthernetLink.cpp:366-369` | With a static IP (the default), `ETHERNET_EVENT_DISCONNECTED` clears only `phy_link_connected_`; `link_up_` is cleared only under `#if !ETH_USE_STATIC_IP`, so `isUp()` stays true after the cable is pulled and the TFT shows "UP" | Confirmed, rechecked | G-E7 |
| L2 | risk | `main_init_net.cpp:21-43` | Network services start only if the link is up within `ETH_IP_WAIT_TIMEOUT_MS` (1,500 ms); a later link-up never starts the net console, OTA server or CellNet L2 | Confirmed (code); timing Suggested: `boottime` over several boots and a late cable insert | G-E7 |
| L3 | rule | `EthernetLink.cpp:221-223`; `include/ethernet_app_config.h:24-40` | The static IP comes from compile-time macros; the stored `eth_ip` (shown in the TFT Network menu) is not read | Confirmed | G-B5 |
| L4 | resource | `EthernetLink.cpp:306, 353-358, 382-392` | MAC and PHY objects leak on a failed driver install; a failed IP-handler registration leaves the Ethernet handler registered with `this` | Confirmed; low impact (one start per boot) | — |
| L5 | docs | `EthernetLink.cpp:287-288, 428-432` | A "next step" comment for work the function already does; an error text that still mentions a broker PC | Confirmed | — |

`nvs_flash` in REQUIRES appears unused (Suggested: confirm with a build).

**Actions:**
- Fix L1 and L2.
- Route `eth_ip` (G-B5).
- Extract a link-state reducer and host-test static and DHCP modes.
- Write a short README: wiring, core pinning, static-IP behaviour.
- Write a REFERENCE.md: API, boot timing, link-state semantics, config keys, events, the `boottime` fields.

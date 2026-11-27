# Firmware review and optimizations

**Status:** record of a firmware review carried out in phases in October and November 2026. Corrected against the code on `refac/code_encapsulation` on 2026-11-27: statements that no longer match the code are marked **Correction**. The test logs are as recorded on the day and were not re-run. Open work is tracked in [docs/OPEN_ITEMS.md](docs/OPEN_ITEMS.md), not here.

## 1. Summary

The review found five problems:

- the MCP23S17 driver updated its shadow registers outside its lock
- `Gantry::Gantry` was called from tasks on both cores without a lock
- EtherNet/IP functions put 4 KB frame buffers on the task stack
- Class 1 pacing busy-waited for the whole remainder of each cycle
- the trajectory planner divided by values it had not checked

Phases 1 to 3 fixed these. Phase 4 changed the pick math and stored settings, and phase 5 changed the theta and safe-Z settings. A second review (section 5) listed four more findings.

## 2. Changes by phase

### Phase 1: data integrity and concurrency

1. **MCP23S17 shadow cache.** The port shadow bytes were updated before the driver lock was taken, so two tasks writing pins on the same port could lose a bit.
   - `lib/MCP23S17/src/MCP23S17Bus.cpp` and `MCP23S17.cpp` gained `mcp23s17_write_register_unlocked` and `mcp23s17_read_register_unlocked`. Every read-modify-write of the output, direction, pull-up and interrupt registers runs under `mcp23s17_lock(handle)`.
   - `mcp23s17_lock` waits for Class 1 to be idle (`spi3_wait_class1_idle`) and re-checks after taking the mutex (`MCP23S17Bus.cpp:57–65`).
   - `lib/MCP23S17/CMakeLists.txt` requires `Spi3Bus`.
   - The shadow byte is still updated before the result of its write is known (OPEN_ITEMS G-D6).
2. **Spi3Bus.** `spi3_class1_critical_active()` is exported with C linkage for the driver (`lib/Spi3Bus/src/Spi3Bus.h`).
3. **Gantry lock.** `Gantry::Gantry` has a `mutable std::recursive_mutex mutex_`. The public configuration, status and motion calls take it, including `isBusy()` and `isEnabled()`. `calibrateX()` and `calibrateZ()` scope the lock inside their wait loops so that `GantryUpdate` is not starved.
4. **Atomic flags.** `EthernetLink` holds `link_up_` and `phy_link_connected_` as `std::atomic<bool>` (`lib/EthernetLink/src/EthernetLink.h:46–47`). The console session and worker flags are atomic, now in `src/gantry_console_session.cpp`. **Correction:** the MQTT bridge flags the original listed no longer exist; the MQTT bridge was removed and CellNet L2 is the cell transport.

### Phase 2: memory and real-time behaviour

1. **Frame buffers.** `kMaxFrameSize` in `lib/EtherNetIP/src/EipSession.h` went from 4096 to 512 bytes. `EipSession::transact` and the scanners' `exchangeOnce` use it.
2. **Class 1 pacing.** Whole ticks are waited with `xTaskDelayUntil`. Only the remainder below one tick is waited with `esp_rom_delay_us` (`EipScannerTaskInternal.h:108–118`). **Correction:** the original said the busy-wait was removed. `esp_rom_delay_us` is itself a busy-wait; what changed is that it now covers less than one tick.
3. **Reconnects.** `EipMultiScanner::disconnect()` closes the connections and keeps the `EipSession` and `EipIoConnection` objects, so a reconnect does not allocate again (`EipMultiScannerConnect.cpp:396`).
4. **TCP console log sink.** The sink receives the client socket by value, cast through `intptr_t`, instead of a pointer to a stack variable (`src/gantry_net_console.cpp:156, 203`).

### Phase 3: numerical safety

1. **Trajectory guards.** `calculateProfile()` rejects a start, target, speed, acceleration or deceleration that is not finite or not positive (`lib/Gantry/src/GantryTrajectory.cpp:14–19`). `interpolate()` checks `t_accel` before dividing by it.
2. **Warning.** `singleAxisTask` is marked `[[maybe_unused]]` for multi-axis builds (`lib/EtherNetIP/src/EipScannerConfig.cpp:72`).

### Phase 4: pick math and stored settings

1. **Vision angle frame.** **Correction:** the angle reference was decided again on 2026-11-16 (OPEN_ITEMS G-C3). Vision θ is 0° along +X and 90° along +Y, positive about +Z up, and the theta joint is the pose angle minus 90° (`lib/Gantry/src/GantryKinematics.h:39`). The original's frame (θ 0° along +Y, joint = 90° − θ_world) and the belt-velocity projections derived in it are superseded and not restated here. The canonical math moves to the new math document ([MATH_RELINK_INVENTORY.md](MATH_RELINK_INVENTORY.md)).
2. **Strike timing.** The original replaced a constant-acceleration model of the plunge below the object top with a piecewise model of the acceleration, cruise and deceleration phases for unequal $a_z$ and $d_z$. It recorded a 6.73 ms (2.02 mm) error in the earlier model. Not re-measured.
3. **Pick speed.** **Correction:** there are no separate pick speed keys; `pk_app_spd_mm_s` and `pk_app_th_deg_s` do not exist in the code. Picks use the global profile (`spd_mm_s`, `th_spd_deg_s`), one source for every motion parameter. Console moves share that speed (OPEN_ITEMS G-C7). The compile-time defaults are 50 mm/s and 30 deg/s (`lib/AppConfig/src/AppConfigData.h:10, 13`).
4. **Follower queueing.** The original removed a duplicate 190 ms grip dwell from $T_{\mathrm{ready}} = t_{\mathrm{retract}} + T_{\mathrm{post\_bin}}$, subtracted the strike lead ($T_{\mathrm{lead}} \approx$ 160–360 ms) from follower deadlines, computed follower travel in joint space, and kept non-pick packs (classes 11–13) in tracking as obstacles. Not re-checked against the code on 2026-11-27.
5. **Stored settings.** **Correction:** the keys `cv_trig_y_mm` and `cv_trig_off_mm` do not exist, and the drive target IPs still come from Kconfig (OPEN_ITEMS G-B5). `ztrace raw` has no line pacing in the current code (`src/pick_trace.cpp`); `thtrace raw` is paced at 20 ms per line (`lib/ThetaTrace`, `kRawLinePaceMs`).

## 3. The `test_cycle` command

`test_cycle` is in `src/gantry_console_testcycle.cpp` and runs eight stages:

```text
[1 Servo arming] -> [2 EIP bring-up] -> [3 Software limit rejection]
[4 Kinematic legs A-F] -> [5 Pick and place with gripper] -> [6 Creep and high dynamics]
[7 Theta range G-I] -> [8 Final pose and Class 1 timing]
```

1. **Servo arming:** arms the drives, checks the status bits and clears start-up warnings.
2. **Bring-up:** homes Z (A015) and X (A014), calibrates both strokes, sets the SAFE_Z band, captures the theta origin and moves to the park pose. When the logs below were taken, the strokes measured X 413 mm and Z 149 mm, and SAFE_Z was 30 mm.
3. **Software limits:** commands past the software limits must be rejected by the planner without a drive alarm.
4. **Kinematic legs A to F:** in-band dual move, Z descent, safe retract, two-segment diagonal, three-segment move across the workspace, return to datum.
5. **Pick and place:** approach, descend, `grip(true)` and settle, ascend, transfer, descend, `grip(false)` and settle, clear.
6. **Dynamics:** a 15 mm/s creep leg and a 150 mm/s leg at 3000 mm/s².
7. **Theta range G to I:** sweeps to −180° and +180°, or to the joint limits where they are narrower.
8. **Summary:** final pose and the Class 1 latency statistics.

**Status:** `test_cycle` cannot pass on this machine today. It checks theta to 0.015° after bring-up and after every leg, against the drive's 0.1° window, and some legs reach Z 130 mm, past the 121.5 mm stroke (OPEN_ITEMS G-E8).

## 4. Results recorded after phases 1 to 3

Not re-run. At the time:

- **Host tests:** 28 of 28 passed. The suite has since grown to 32 test programs.
- **Firmware build** (`idf.py -C idf build`): image `0x9dfd0` bytes (647,120), 38% of the app partition free, no warnings.

`test_cycle` on the gantry, over UART:

```text
=== HOLISTIC TEST CYCLE COMPLETE: PASS
=== Total Cycle Duration: 90455 ms (90.4 s)
=== Final Pose: x=0.000 mm, z=-0.007 mm, theta=0.074 deg
=== Class 1 timing (budget: exchange p99 < RPI=2000 us) ===
  exchange  n=128 min/p50/p99/max=890/1049/1414/1497 us  GO
  ot_send   n=128 min/p50/p99/max=507/551/610/618 us
  drain     n=128 min/p50/p99/max=350/476/749/833 us
  cycle     n=128 min/p50/p99/max=962/2000/2874/2969 us
  cmd2start n=33 min/p50/p99/max=597/924/1196/1939 us
  pace      overrun=5236 yield=3
  reliability soft_miss=0 sendok_fail=0 chip_recover=0 reconnect=0
OK test_cycle PASS
```

## 5. Second review: four findings

| # | Finding | State on 2026-11-27 |
|---|---|---|
| 1 | The 100 Hz `GantryUpdate` loop used `vTaskDelay`, so its period was 10 ms plus the work time | Done: `vTaskDelayUntil` (`src/main_tasks.cpp:46`) |
| 2 | The MQTT client was not started when the Ethernet link was down at boot | Obsolete: the MQTT bridge was removed |
| 3 | MQTT publish failures were dropped without a retry | Obsolete: the MQTT bridge was removed |
| 4 | The MCP23S17 driver could take the SPI3 mutex just after Class 1 became active again | Done: `mcp23s17_lock` re-checks Class 1 after taking the mutex (`MCP23S17Bus.cpp:57–65`) |

`test_cycle` after fixes 1 and 4, as recorded:

| Measure | Before | After |
|---|---:|---:|
| Cycle duration | 90.4 s | 86.2 s |
| Exchange p99 / max | 1414 / 1497 µs | 1291 / 1341 µs |
| Pace overruns | 5236 | 735 |

```text
=== HOLISTIC TEST CYCLE COMPLETE: PASS
=== Total Cycle Duration: 86182 ms
=== Final Pose: x=-0.000 mm, z=-0.018 mm, theta=0.069 deg
=== Class 1 timing (budget: exchange p99 < RPI=2000 us) ===
  exchange  n=128 min/p50/p99/max=867/1046/1291/1341 us  GO
  ot_send   n=128 min/p50/p99/max=479/520/649/655 us
  drain     n=128 min/p50/p99/max=355/492/700/708 us
  cycle     n=128 min/p50/p99/max=937/1996/2780/3133 us
  cmd2start n=31 min/p50/p99/max=559/846/1126/1174 us
  pace      overrun=735 yield=4
  reliability soft_miss=0 sendok_fail=0 chip_recover=0 reconnect=0
OK test_cycle PASS
```

## 6. Phase 5: theta and safe Z

1. **Safe Z.** The original set the compile-time SAFE_Z fallback to 35.7 mm (114.3 mm above a 150 mm Z stroke). **Correction:** the fallback is now 48.4 mm (`include/axis_drivetrain_params.h:322`). The value in use is the stored one.
2. **Theta origin.** `GantryEipRotaryAxis` has no software offset (`zero_puu_` was removed). Position feedback and commands use the drive's absolute position (`S-0-0051`, `S-0-0282`).
3. **Theta tolerances.** **Correction:** the original said positioning is held to 0.01°. `AXIS_THETA_POSITION_TOLERANCE_DEG` is 0.01° (`idf/sdkconfig.defaults:120`), but the arrival check `kArrivalEpsDeg` is `EIP_THETA_SETTLED_TOL_DEG`, 0.1° (`lib/Gantry/src/GantryUtils.h:41`), and the drive's own window is 0.1° (G-E8).
4. **Theta command limits.** **Correction:** speed is capped at the drive's S-0-0113, 600 rpm (3600°/s), and acceleration and deceleration at S-0-0138, 1000 rad/s² (`lib/Gantry/src/GantryEipRotaryAxis.h:29–31`). The default acceleration is 1800°/s², which is 31416 in the drive's 0.001 rad/s² units (`:85–86`). Speed is sent in 0.0001 rpm units, so 3600°/s is 6,000,000.
5. **Drive tuning clients.** `Kinetix5100ParamAccess` (CIP class 0x0F) and `Hcs01ComwsClient` (HTTP port 80) are in `lib/EtherNetIP`. **Correction:** the console `autotune` command is compiled out (`GANTRY_CONSOLE_AUTOTUNE 0`, `src/gantry_console_internal.h:13`), so tuning is not done on the gantry today.

`test_cycle` after phase 5, as recorded:

```text
=== HOLISTIC TEST CYCLE COMPLETE: PASS
=== Total Cycle Duration: 101260 ms
=== Final Pose: x=0.000 mm, z=-0.000 mm, theta=-0.003 deg
=== Class 1 timing (budget: exchange p99 < RPI=2000 us) ===
  exchange  n=128 min/p50/p99/max=765/808/1172/1239 us  GO
  ot_send   n=128 min/p50/p99/max=466/495/658/709 us
  drain     n=128 min/p50/p99/max=277/294/496/496 us
  cycle     n=128 min/p50/p99/max=856/2000/2556/2902 us
  cmd2start n=30 min/p50/p99/max=815/954/1300/1449 us
  pace      overrun=487 yield=2
  reliability soft_miss=0 sendok_fail=0 chip_recover=0 reconnect=0
OK test_cycle PASS
```

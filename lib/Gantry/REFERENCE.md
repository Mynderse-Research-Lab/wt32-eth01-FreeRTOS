*[README](../../README.md) › [Knowledge transfer](../../docs/KNOWLEDGE_TRANSFER.md) › [Gantry firmware](../../GANTRY_FIRMWARE.md) › [Libraries](../README.md) › [Gantry](README.md) › Reference*

# Gantry reference

**Read this when** you need an exact fact: a signature, a unit, a timeout, a key or a return value.

Public headers under `lib/Gantry/src/`. The class is `Gantry::Gantry` in `Gantry.h`. Units on `moveTo` are millimetres, degrees, mm/s, deg/s, mm/s² and deg/s².

## Motion API

```c
bool begin();
void enable();
void disable();
void setJointLimits(float xMin, float xMax, float zMin, float zMax,
                    float thetaMin, float thetaMax);
void setSafeZHeight(float safeHeight_mm);
void setRuntimeParams(const RuntimeParams& params);

GantryError moveTo(const JointConfig& joint,
                   uint32_t speed_mm_per_s = 50,
                   uint32_t speed_deg_per_s = 30,
                   uint32_t acceleration_mm_per_s2 = 0,
                   uint32_t deceleration_mm_per_s2 = 0,
                   uint32_t acceleration_deg_per_s2 = 0,
                   uint32_t deceleration_deg_per_s2 = 0,
                   float theta_start_frac = -1.0f,
                   bool theta_full_speed = false);
GantryError moveTo(const EndEffectorPose& pose, /* same speed and accel units */);

GantryError preloadJointMove(const JointConfig& joint, ...);
bool releasePreloadedMotion();
GantryError retractToSafeZ(uint32_t speed_mm_per_s, ...);
GantryError queueRetractToSafeZ(uint32_t speed_mm_per_s, ...);
bool chainMoveAfterRetract(const JointConfig& joint, ...);
void requestAbort();
void update();
bool grip(bool active);
```

`GantryError` (`GantryTypes.h:15–27`): `OK`, `NOT_INITIALIZED`, `MOTOR_NOT_ENABLED`, `NOT_CALIBRATED`, `ALREADY_MOVING`, `INVALID_POSITION`, `INVALID_PARAMETER`, `TIMEOUT`, `LIMIT_SWITCH_FAILED`, `CALIBRATION_FAILED`, `CONVERSION_ERROR`.

`JointConfig` is x mm, z mm (+down), theta degrees (`GantryConfig.h`). Pose z is not restated here; the conversion is `GantryKinematics.h`.

There is also `moveTo(int32_t x, int32_t z, int32_t theta, uint32_t speed)` (`Gantry.h:177`). The production calls use the `JointConfig` overload.

Homing: `homeX`, `homeZ`, `homeTheta`, `calibrateX`, `calibrateZ`, `calibrateTheta`, `startEipBringUp`. `homeTheta` calls a move to 0° with no SAFE_Z check in that function. G-A11.

## Frame

Right-handed: X across the belt, Y along the belt with +Y downstream (no Y actuator), Z positive down, theta about Z. Joint space is (x, z, theta). Joint Z = 0 is the A015 retract datum.

Limit warnings (`Gantry.h:450–451`): joint min is X A014 and Z A015; joint max is X A015 and Z A014.

**Limit aborts during a move.** In the run phase a linear axis aborts the move when the drive reports a limit stop or a fault, or when an A014 or A015 warning trips (`GantryEipLinearAxisPhases.cpp:340–348`). A warning does not trip when:
- it was already present when the move started (`ignore_a014_abort_` and `ignore_a015_abort_`, set at `GantryEipLinearAxis.cpp:269–271`, cleared once the warning goes away), or
- the move heads away from that switch. `noteMoveDirection` (`GantryEipLinearAxis.cpp:230–242`) records this at each move start (`startAbsoluteMm`) and at each queued-move swap (`applyQueuedMove`). The joint end of each switch comes from `setJointMinWarningA015`.

A switch ahead of the move still aborts it. The second rule lets a move leave a switch the drive flags only once the axis moves; the Kinetix 5100 allows motion away from an active limit (2198-UM004D ch. 15, DI functions 0x22/0x23). G-A13: the host tests pass, and the on-cell proof from that failing state is still open.

## Tick and phase times

`GantryUpdate` is priority 5 on core 1 and calls `update` every `GANTRY_UPDATE_PERIOD_MS`, 10 ms (`include/gantry_app_constants.h:91–98`). Phase lengths in `GantryEipLinearAxisInternal.h` are ticks of that loop:

| Constant | Ticks | Comment in the header |
|---|---:|---|
| `kArmServoLowTicks` | 4 | |
| `kArmServoSettleTicks` | 40 | |
| `kMovePreloadTicks` / `kMoveStartMotionTicks` | from `LinearPhase` | |
| `kHomeWaitTicks` | 50 | 0.5 s at 100 Hz |
| `kMoveTimeoutTicks` | 1,500 | 15 s at 100 Hz |
| `kStopStableTicks` | 20 | 200 ms at 100 Hz |
| `kArrivalBandMm` | | 0.5 mm |
| `kNearStopSpeedRef` | | 8 |

`kNearStopSpeedRef` is 8 (`GantryEipLinearAxisInternal.h:43`), and the comment above it reads that as 0.8 RPM, while `Kinetix5100Assembly.h:111` calls `actual_speed` RPM. Which unit the drive sends is not settled. G-A14.

`grip` takes `mutex_` and then writes the expander (`GantryMotion.cpp:475–480`). G-D3.

## Threading

| Task | Priority | Core | Defined |
|---|---:|---:|---|
| `GantryUpdate` | 5 | 1 | `gantry_app_constants.h:92–93` |
| `PickStrike` | 6 | 1 | `gantry_app_constants.h:116–117` |
| `PickScheduler` | 5 | 0 | `gantry_app_constants.h:113–114` |
| Console | 3 | 0 | `gantry_app_constants.h:104` |

`mutex_` is a `std::recursive_mutex` (`Gantry.h:312`).

## AppConfig

`setRuntimeParams` copies the stored motion fields the caller passes in. `th_sequential` and `cal_timeout_ms` are stored keys this component does not read; the sequential flag and the calibration timeout are compile-time. G-F2.

## Console

The test console in `src/gantry_console_*.cpp` is the command surface: `move`, `home`, `calibrate`, `grip`, `stop`, `alarmreset`. This component does not register those commands.

## Errors

`moveTo`, the preload and retract calls and the homing calls return the `GantryError` above. `begin` returns false when the gantry cannot start, and `bootInitMotion` then stops the motion stage (`src/main_init_motion.cpp`).

## Usage

```cpp
// src/main_init_motion.cpp
static Gantry::Gantry gantry(std::move(xAxis), std::move(zAxis), std::move(thetaAxis), PIN_GRIPPER);
gantry.setSafeZHeight(params.gantry_safe_z_height_mm);
gantry.setRuntimeParams(gantryRuntimeParams(params));
if (!gantry.begin()) {
    return;
}
// later, motors enabled and axes calibrated
Gantry::JointConfig target{120.0f, 40.0f, 90.0f};   // x mm, z mm (+down), theta deg
const GantryError err = gantry.moveTo(target, 300, 90);
```

## Known issues

Found in the library review of 2026-11-26 (at `1caa98c`) and not tracked as open items. Not re-checked since; confirm against the code before acting.

| # | Category | Location | Finding | Evidence |
|---|---|---|---|---|
| A3 | risk | `GantryMotion.cpp:191-195, 234` | `retractToSafeZ` writes a stop image and overwrites it with the new preload in the same call, so StopMotion probably never reaches the wire | Suggested: `ztrace` of a retract during a slow descent |
| A7 | risk | `GantryPath.cpp:45-47`; `GantrySequential.cpp:124-143` | With X deferred, a path whose Z goes idle outside the band without an alarm stays busy forever: there is no overall path timeout | Confirmed (logic); trigger Suggested: host test |
| A8 | bug | `GantrySequential.cpp:56-79`; `GantryMotion.cpp:101-102` | `startSequentialMotion()` returns `void`; a failed arm or a refused theta interlock still makes `moveTo` return `OK`. `test_orchestrator_bubbles_up_axis_rejection` asserts `OK` | Confirmed, rechecked |
| A9 | bug | `GantrySequential.cpp:72`, `GantryPath.cpp:264` | A failed grip in the pose-move PnP sequence still proceeds to retract | Confirmed |
| A11 | risk | `GantryEipLinearAxisInternal.h:48`; `GantryEipLinearAxisPhases.cpp:375`; `GantryEipHomeCal.cpp:231`; `ConfigSchema.cpp:184-187` | Fixed 15 s move and 20 s phase timeouts conflict with the stored speed ranges. A 550 mm seek needs at least 37 mm/s; a 180° orient below 9 °/s times out | Confirmed by arithmetic |
| A13 | bug | `GantryEipBringUp.cpp:343-349, 481-488`; `GantryEipHomeCal.cpp:389-406` | Bring-up X park, band return and the calibrate return to zero accept "not busy" as success without checking arrival, so a timed-out or aborted move still latches the workspace | Confirmed |
| A14 | bug, minor | `GantryEipBringUp.cpp:79-89` | `failEipBringUp` does not stop theta; a phase timeout during the orient leaves it moving | Confirmed |
| A15 | risk | `GantryEipLinearAxis.cpp:312-322`; `Gantry.cpp:177-200` | `stopMotion()` on an idle linear axis publishes a Position-mode image with StopMotion to a drive in settle mode and holds busy for 200 ms. `enable()` has no busy guard and `disable()` replaces the stop image with ServoOff in the same call | Confirmed (code); consequence Suggested: watch for A603 after `stop` with the axes idle |
| A16 | risk | `GantryEipLinearAxisInternal.h:44`; `GantryEipLinearAxisPhases.cpp:263-278`; `GantryEipRotaryAxisInternal.h:40`; `GantryEipRotaryAxisUpdate.cpp:159-166` | Per-tick thresholds (0.005 mm, 0.015°) do not scale with the control period that Kconfig allows down to 2 ms | Suggested |
| A17 | risk | `GantryEipBringUp.cpp:320, 455`; `GantryEipHomeCal.cpp:183` | Measured strokes are rounded to whole mm and feed `x_max` and the kinematic `x_stroke_mm`, so the world-to-joint X map shifts by up to 0.5 mm with every calibration | Confirmed (code); impact Suggested |
| A18 | docs | `Gantry.h:85, 150-158, 211-212`; `GantryRotaryAxis.h:96-99, 116`; `GantryThetaOrigin.h:1-15`; `GantryUtils.h:23-24`; `GantryConfig.h:12-14, 43, 111-112`; `GantryKinematics.h:9` | Stale: the aligned/unaligned origin with a shrunk thetalim (the code always uses ±36000, G-C15); settled tolerance 0.1° (code 0.5°); "retract stops motion first"; MCP23S17 "removed"; lock-free trace; the band formula; the Z pose formula (code: `pose.z = z_stroke − joint.z`) | Confirmed |
| A19 | cleanup | `GantryEndEffector.cpp:10, 62, 69`; `GantryLimitSwitch.cpp:5, 12` | Gantry calls `gpio_expander_*`. The wrapper is `lib/GpioExpander/src/gpio_expander.c`, and Gantry requires the `GpioExpander` component | Confirmed |

**Dead code:**
- unused types and methods: `GantryStatus`, `HomingStatus`, `Waypoint`, `WaypointQueue`, `TrajectoryWaypoint`, the empty `GantryConfig.cpp`, `home()`, `setLimitPins()`, `setHomingSpeed()`, steps-per-revolution, the legacy pulse `moveTo`, `startEipPrecisionCalibrate`, `BringUpPhase::kZEnterBand`, and the `GANTRY_DIAG_SKIP_*` toggles
- the unreachable GPIO limit path (`GantryHoming.cpp:54-85, 290-300, 330-460`)
- PulseMotor and MQTT comments

**Not covered by host tests:**
- the EIP home/calibrate sequence through Gantry, `homeTheta` and `calibrateTheta`
- `clearAlarm`, `enable` and `disable` during motion
- the drive-managed limit mirror and the re-zero case (A1)
- the deferred-X hang (A7)
- a 2 ms control period

## Call behaviour

Construction: DI with `unique_ptr` axes + gripper pin. PulseMotor constructor
**removed**.

| Call | Behavior |
|------|----------|
| `begin()` / `enable()` / `disable()` | Lifecycle; enable arms EIP axes (settle → Home34 if needed → hold) |
| `isBusy()` | True while `motionState_ != IDLE`, any axis busy, or homing/calibration flags |
| `moveTo(EndEffectorPose)` | PnP path: 2-D path planner + gripper staging; post-grip retract to clearance |
| `moveTo(JointConfig)` / `moveTo(x,z,theta,…)` | **JOINT_DIRECT**: 2-D path to joint target (no gripper). `moveTo(JointConfig, …, theta_start_frac)` sets where theta starts in the in-band segment (scheduler passes 0; −1 = default 25%) |
| `preloadJointMove()` / `releasePreloadedMotion()` | Held Z preload for a late Z tail; the pick timer fires the StartMotion edge |
| `retractToSafeZ(speed, accel, decel)` | Z-only lift to the traverse band at the Z-capped speed; allowed while X or theta is alarmed; refused (`TIMEOUT`) on a Z alarm or lost Z reference |
| `grip(bool)` | Gripper write; returns false when no pin is configured or the MCP write fails |
| `softHomeJointDatum()` | Zeros firmware `zero_puu_` on X+Z (not drive Homed) |
| `requestAbort()` | Abort motion only — does **not** disable servos |
| Console `stop` | `requestAbort()` + `disable()`; requires home+calibrate again |
| Gripper open/close | MCP Field DOUT0 (PA0) via `GantryEndEffector` |

`update()` must run ~100 Hz (`gantryUpdateTask`). Order inside `update()`:

1. `updateAxisPositions()` — each axis `update()` + X limit polling.
2. `processSequentialMotion()` — so `isBusy()` sees StartMotion preload/pulse before sequencing advances.

## Paths, SAFE_Z and bring-up

Console / Gantry `speed`, `accel`, and `decel` are the **resultant** along the
X-Z path (defaults 50 mm/s, 3000 / 3000 mm/s²; ceiling `GANTRY_PATH_MAX_*` =
3000 mm/s²). There are no per-axis motion-tuning knobs — Absolute components
are derived from path direction:

`v_i = V · |d_i| / L`, `a_i = A · |d_i| / L`, with `L = hypot(dx, dz)`.

Helpers live in [`GantryPathProfile.h`](src/GantryPathProfile.h)
(`decompose`, `planSegments`). Z cap: `armCurrentPathSegment` lowers each
segment's path speed with `Path::zCappedPathSpeed` so the Z component stays under
`AXIS_Z_SPEED_CAP_FROM_CRITICAL_RPM_MM_PER_S` (ballscrew critical speed) and the
segment stays straight. The pick budget uses the same helper.

**Traverse clearance** (`GANTRY_SAFE_Z_HEIGHT_MM`; the stored value is the one in use, and the compile-time
fallback is **48.4 mm**, `include/axis_drivetrain_params.h:322`): margin from Z− / A015 (retracted joint min). Any X Absolute (home/cal/path/EIP seek)
is allowed only while joint Z is in that traverse band
(`z <= z_min + margin`). Beyond it (further +Z / toward the belt), Z moves
alone with X held; console `home x` / `calibrate x` refuse.
SAFE_Z is an **X and theta interlock**, not a path via: in-band X and Z may
run together; theta may turn only while Z is in the band. Retract from above
the band to an in-band target is one Z Absolute (X starts
once Z is in-band). Above the band, Z-only. Theta on a combined move is
scheduled against the **in-band X+Z traverse segment**: start at **25%** of
that segment, finish by **75%**. Required theta speed is
`|dTheta| / (0.5 * T_seg)` (constant-speed `T_seg = L / V`), clamped to
`AXIS_THETA_MAX_SPEED_DEG_PER_S` (360 deg/s). If the cap forces finish past 75%, the
below-band descent segment is **held** until theta is idle. Theta-only
commands while Z is out of band are refused (linear path still runs).
With a per-move `theta_start_frac` ≥ 0 (pick scheduler: 0), theta starts at that
fraction and runs at the move's theta speed, capped at the axis maximum; the
rotary start phases (preload 4, release-halt ≤ 10, ack ≤ 4 ticks) are shared with
the budget through `GantryRotaryPhases.h`.
Limit warning codes: **X** A014/PL = joint min,
A015/NL = joint max; **Z** A015/NL = joint min (retract), A014/PL = joint max
(+Z down / toward belt). Z Absolute joint sense is **inverted** so joint −
seeks A015.
**Joint 0** is the sample at min-switch disable (warning deassert / post-creep
instant) — no extra offset after clear. That datum is the **retracted** end.

**EIP bring-up** (`calibrate all`): Z- A015 → creep until switch disabled →
**Z=0** → X home (A014 clear → X=0) / cal → park X at
`GANTRY_CAL_X_PARK_MM` (35) → Z+ stroke (A014) → return to the SAFE_Z ceiling.
Seek, park, and SAFE_Z return are locked at **100 mm/s** and **2000 mm/s²**
(not console `speed`/`accel`). Switch-clear creep stays **1 mm/s**.

## Orchestration

`MotionState` in [`GantryTypes.h`](src/GantryTypes.h), driven from [`GantryPath.cpp`](src/GantryPath.cpp), [`GantryPathTheta.cpp`](src/GantryPathTheta.cpp), [`GantrySequential.cpp`](src/GantrySequential.cpp) and [`GantryMotion.cpp`](src/GantryMotion.cpp):

| State | Role |
|-------|------|
| `IDLE` | No sequenced motion |
| `Z_DESCENDING` / `Z_RETRACTING` | Z-alone toward belt (+Z) / toward A015 (retract) |
| `GRIPPER_ACTUATING` | Timed open/close (PnP only) |
| `X_MOVING` | X-alone path segment (Z in SAFE_Z band) |
| `XZ_MOVING` | In-band X+Z, or retract with X unlocked once Z is in-band |
| `THETA_MOVING` | Present; used when `CONFIG_EIP_AXIS_THETA=y` |

JOINT_DIRECT skips gripper choreography; both joint and pose moves use the
same path planner / clearance rule.

**X-Z path (production):** in-band X+Z together; above SAFE_Z, Z-alone. X
Absolute and theta are interlocked unless Z is in the retract/traverse band.
Retract from pick height to an in-band target does not stop at SAFE_Z.
Theta window: 25–75% of the in-band traverse (or from `theta_start_frac`);
descent below the band waits for theta idle if the speed cap is hit.

Arm Z first, then X (or defer X until the interlock is true) before publishing
`motionState_` (A603 / StartMotion race guard).

### Abort matrix

| Path | Stops motion | ServoOff | Session home/cal flags |
|------|--------------|----------|------------------------|
| `Gantry::requestAbort()` | Yes (`stopAllMotion`) | No | Cleared by stop path |
| Console `stop` | Yes | Yes (`disable`) | Must home+calibrate again before `move` |
| Axis timeout / fault | Axis enters StopMotion / hold | Depends | See Linear axis below |

## Datums

**Two datums (do not conflate):**

| Mechanism | Who sets it | Effect |
|-----------|-------------|--------|
| HomingMethod **34** (on arm) | Drive via Class 1 | Sets drive `HomedStatus`; on success firmware syncs `zero_puu_ = actual_position` and `target_mm_ = 0` |
| `softHomeJointDatum()` | Firmware only | Sets `zero_puu_` so current actual reads as joint 0 — does **not** set drive Homed |

Joint PUU ↔ absolute PUU:

```
joint_puu = abs_puu - zero_puu_
command_abs_puu = toAbsPuu(mmToPuu(target_mm))
```

Absolute PTP still needs the drive Homed (Home34 on arm, or already-homed drive). Soft-home alone is a **joint-frame** zero for the console / planner; skipping Home34 risks E237 / rejected Absolute.

## Linear axis: Position Absolute

| Field | Value |
|-------|-------|
| Assemblies | O→T **104**, T→O **154**, Class 1, RPI 2 ms |
| IO Mode | `P1.001 = 0xC` |
| OperatingMode | **1** Position |
| TravelMode | **2** non-cyclic |
| NonCyclicMoveType | **0** Absolute |
| Done | `(AtReference ∧ in-band) \|\| (in-band ∧ nearly stopped)`; else timeout → StopMotion |

**Still used (not PTP):** OM=0 + TM=10 for settle / hold / abort StopMotion.  
**Rejected for PTP:** OM=2 Speed + TM=10 + host StopMotion; Index/PR (OM=6) for dynamic picks; CIP Motion / Motion Group.

PC prove: [`tools/eip_position_abs.py`](../../tools/eip_position_abs.py).  
Firmware SM: [`GantryEipLinearAxisPhases`](src/GantryEipLinearAxisPhases.cpp) (ticks @ **100 Hz**).

### ArmPhase (enable)

```
kIdle → kServoLow → kServoSettle → [kHomePreload → kHomeStart → kHomeWait] → kHolding
```

| Phase | Ticks | Behavior |
|-------|-------|----------|
| `kServoLow` | 4 | ServoOff settle |
| `kServoSettle` | 40 (~400 ms) | OM=0 TM=10, ServoOn edge, wait Active; up to 2 FaultReset on A603 |
| `kHomePreload` / `kHomeStart` | 4 / 5 | Only if `!HomedStatus`: OM=Home, method **34**, TM=2, StartMotion edge |
| `kHomeWait` | ≤200 (~2 s) | Wait HomedStatus; on success sync `zero_puu_`; timeout → hold + E237 risk warning |
| `kHolding` | — | Settle image (OM=0 TM=10); moves allowed |

Never command Position before Active (A603). `moveToMm` rejects unless arm is `kIdle` or `kHolding`.

### MovePhase (Absolute PTP)

```
kIdle → kPreload → kStart → kRun → (kStopping on abort/timeout) → hold
```

| Phase | Ticks | Behavior |
|-------|-------|----------|
| `kPreload` | 4 | `moveToMm` publishes OM=1 TM=2 Absolute, **StartMotion=0** (latch Position) |
| `kStart` | 5 | StartMotion=1 edge (same cadence as Home34) |
| `kRun` | ≤6000 (~60 s) | StartMotion=0; wait done predicate |
| `kStopping` | stop pulse 5; stable 10 | `stop_motion` pulse; wait ±0.02 mm stable and speed low; then hold |

Done predicate (`kArrivalBandMm = 0.5`):

```
(at_reference && inPositionBand)
  || (inPositionBand && nearly_stopped)
```

`nearly_stopped` = `isMotionStopped()` OR `|actual_speed| ≤ kNearStopSpeedRef` (8, `GantryEipLinearAxisInternal.h:43`). The unit of `actual_speed` is not settled (G-A14).

Already-there short-circuit: `|here - target| ≤ 0.05 mm` → hold, no StartMotion.  
`finishMoveHold` clears busy, republishes settle hold, keeps commanded Absolute target (for `puuinfo`).

## Scaling

`speed_ref_per_mm_s = 600 × i / lead` (Kinetix 0.1 RPM units):

```
rpm = mm/s × i / lead × 60
ref = rpm × 10
```

| Axis | i | lead | PUU/mm | speed_ref/mm_s |
|------|---|------|--------|----------------|
| X | 5 | 200 | 52428.8 | 15 |
| Z | 1 | 20 | 104857.6 | 30 |

**Worked example (X @ 200 mm/s):**  
`move_speed_ref = round(200 × 15) = 3000` → 300.0 RPM commanded cruise.

Position: `puu = round(mm × puu_per_mm)`; Absolute commands use `toAbsPuu(...)`.  
Accel/decel refs: `round(mm_s2 × accel_ref_per_mm_s2)`; EDS floor applies in axis code.

Menuconfig: **Gantry kinematics** + EtherNet/IP originator (IPs, PUU/mm,
`EIP_ENDSTOP_SOURCE`: drive vs legacy MCP). Host tests use header defaults without
`sdkconfig.h`.

## Homing and limits

| Mode | Behavior |
|------|----------|
| Soft-home (joint zero) | Only when **no** GPIO limits and **no** drive-managed endstops; `softHomeJointDatum()`; Home34 on arm |
| Drive endstops (TBIO) | KNX Forward/Reverse Limit on INPUT1–4; `CONFIG_EIP_ENDSTOP_FROM_DRIVE` → external `GantryLimitSwitch` |
| Drive home / calibrate | Console `home`/`calibrate` (`x`/`z`/`all`) when drive-managed: **X** A014→min / A015→max; **Z** A015→min (−Z) / A014→max (+Z). `calibrate all` = full bring-up. Seek/park 100 mm/s, 2000 mm/s²; creep 1 mm/s. Absolute Run aborts on A014/A015 so busy clears; escape `move` still allowed |
| Hard envelope | Measured stroke from calibrate, or SCHUNK `AXIS_*_HARD_LIMIT_*` via soft-calibrate |
| Abort | See the abort matrix under Orchestration |

**Bench procedure:** see [EXPECTED §4](../../docs/EXPECTED_ELECTROMECHANICAL_ASSEMBLY.md).  
**Author invariants:** motion only via `Gantry`; respect hard limits; never bypass
STO / Servo On checks when debugging EIP arm failures.

## Theta axis (HCS01)

- Assemblies **101/102** (18 / 14 bytes), freely configurable profile (`P-0-4084 = 0xFFFE`). Map: `P-0-4081` = 4077, 0282, 0259, 0260, 0359; `P-0-4080` = 4078, 0051, 0040, 0390. See `Hcs01Assembly.h`.
- Class 1 ForwardOpen (third slot on the X/Z multi-scanner): T→O still demuxes on shared UDP **2222**; O→T uses one W5500 UDP socket per dest so DIPR stays cached. Config instance **0** (HCS01 has no 110), O→T **24** (18+2 seq+4 Run/Idle), T→O **16** (14+2), Run/Idle **bytes on / net-params bit 8 clear**, serial `0x0003`, O→T CID `0x10000003`, T→O CID `0x20000003`, RPI **2000** µs. Theta FO reject is **non-fatal** (X/Z stay up).
- `CONFIG_EIP_AXIS_THETA` default **y** in `idf/sdkconfig.defaults`. `main_init_motion.cpp` constructs `GantryEipRotaryAxis` with `CONFIG_EIP_AXIS_THETA_PUU_PER_DEG` (**10000** = 0.0001 deg LSB; live S-0-0079 = 3600000 inc/rev). Override with `puu t` / `puucal t c m`.
- **Absolute Encoder Tracking & Zero-Offset Architecture**: `GantryEipRotaryAxis` operates in pure absolute frame without software offsets (`zero_puu_` eliminated). `getCurrentDeg()` directly reflects the HIPERFACE absolute encoder position (`S-0-0051`). Because end-effector cabling and pneumatics are routed over pass-through slip-rings, continuous multi-turn rotation is supported for any number of revolutions.
- **Theta tolerances**: `AXIS_THETA_POSITION_TOLERANCE_DEG` is 0.01° (`idf/sdkconfig.defaults`), but the arrival check `kArrivalEpsDeg` is `EIP_THETA_SETTLED_TOL_DEG`, 0.1° (`lib/Gantry/src/GantryUtils.h:41`), matching the drive's 0.1° window (G-E8).
- **Profile units** (`hcs01VelocityUnits` / `hcs01AccelUnits` in `GantryEipRotaryAxis.h`), from the drive export `tools/Parameter_default_1.par` (2026-10-13):
  - S-0-0259 velocity: S-0-0044 = `0b0010`, S-0-0045 = 1, S-0-0046 = −4 → **0.0001 rpm**, motor side (ERD04 direct drive, so = theta rpm). 1 deg/s = 1666.67 units; no PUU factor. Capped at S-0-0113 = 600 rpm (6,000,000 = 3600 deg/s).
  - S-0-0260 / S-0-0359: S-0-0160 = `0b0010`, S-0-0162 = −3 → **0.001 rad/s²**, no PUU factor (1800 deg/s² = 31416). Capped at S-0-0138 = 1000 rad/s².
  - Position: S-0-0076 = `0b0010`, S-0-0079 = 3,600,000 → 0.0001 deg (10000 PUU/deg).
  - The config defaults used when a move passes accel/decel 0 (`home t`, bring-up orient) are built by `eipRotaryAxisConfigFromDeg()` in the same units.
  - History (2026-10-13): `main.cpp` sent accel × PUU/deg (180,000,000), tripping F2028 about 40° into every home/orient (P-0-0098 max deviation 101.9° vs S-0-0159 window 100°). Velocity was sent as 0.0001 deg/min, 360× too large: 15 deg/s became 900 rpm, past the 600 rpm motor max (E2049), so the commanded speed was ignored.
  - S-0-0159 following-error window is **100°** on this drive, which is very loose; review it in IndraWorks.
- **Move timeout**: `thetaMoveTimeoutUs()` = max(15 s, 2 × distance / speed), so a long return at the 15 deg/s home speed is not cut off at 15 s.
- **Home & Bring-up**: `home t` captures the absolute encoder datum and starts the return to physical zero ($0.000^\circ$). It counts as homed only once theta is idle, alarm-free and within `EIP_THETA_SETTLED_TOL_DEG` (0.1°) of 0 (`Gantry::thetaSettledAt`); the console then logs `OK Theta home complete`, or `ERROR: Theta home did not reach 0 deg`. `calibrate t` and theta moves stay blocked until then. Bring-up fails (`theta orient did not arrive`) instead of running the Z+ stroke when the 90° orient faults or times out.
- **Embedded Autotuning**: Console `autotune` is **disabled for this version**
  (`GANTRY_CONSOLE_AUTOTUNE=0`). Use IndraWorks / KNX5100C / `tools/hcs01_eng.py`
  for drive tuning. Re-enable with `-DGANTRY_CONSOLE_AUTOTUNE=1` to restore:
  - **HCS01 Theta (`autotune theta [inertia|tune|zero]`)**: Embedded `Hcs01ComwsClient` HTTP :80 → C1800 / C2200.
  - **Kinetix 5100 X/Z (`autotune <x|z> [mode1|lock|read|gain]`)**: CIP Class 0x0F Parameter Object Mode 1 / Mode 0.
- `CONFIG_GANTRY_THETA_SEQUENTIAL` default **n**: theta is scheduled on the
  in-band X+Z segment (start 25%, finish 75%) unless sequential choreography
  is enabled (theta after the linear path, still SAFE_Z-gated).
- F2174 after HCS01 reboot is expected until `home t` recaptures origin. Status `in_ref=0` is the cue. **Do not** seek X31.

## Documents

The drive profile and the path are above. `GantryKinematics.h` for the transforms. Open items: G-A9, G-A10, G-A11, G-A12, G-A13, G-A14, G-C8, G-C14, G-C15, G-D3, G-F2 in `docs/OPEN_ITEMS.md`.

## Next

- **Up:** [Gantry README](README.md), [Libraries](../README.md).

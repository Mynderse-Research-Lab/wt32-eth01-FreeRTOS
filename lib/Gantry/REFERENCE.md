# Gantry reference

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

## Documents

`LOW_LEVEL_GANTRY_CONTROL.md` for the drive profile and the path. `GantryKinematics.h` for the transforms. Open items: G-A9, G-A10, G-A11, G-A12, G-A13, G-A14, G-C8, G-C14, G-C15, G-D3, G-F2 in `docs/OPEN_ITEMS.md`.

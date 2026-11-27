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

`kNearStopSpeedRef` is 8 (`GantryEipLinearAxisInternal.h:43`). The comment above it says 0.8 RPM. `docs/LOW_LEVEL_GANTRY_CONTROL.md` says the same comparison uses 50. The code value is 8. `Kinetix5100Assembly.h` calls `actual_speed` RPM.

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

## Documents

`docs/LOW_LEVEL_GANTRY_CONTROL.md` for the drive profile and the path. `GantryKinematics.h` for the transforms. Open items: G-A9, G-A10, G-A11, G-C8, G-C14, G-C15, G-D3, G-F2 in `docs/OPEN_ITEMS.md`.

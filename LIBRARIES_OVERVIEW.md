# Libraries Overview — WT32-ETH01 Gantry Controller

**Scope:** what each library is for, how the libraries stack on top of each other, and when you'd reach for each one.

For exact function signatures, pin numbers, and build/flash steps, see `PROGRAMMING_REFERENCE.md`. This file is the "big picture" companion.

---

## 1. System Architecture at a Glance

```
+------------------------------------------------------------+
|                    Application layer                        |
|                                                            |
|   src/main.cpp            src/gantry_test_console.cpp      |
|   (app_main, tasks)       (interactive serial CLI)         |
|                                                            |
|                 src/basic_tests.cpp                        |
+---------+-------------------------+------------------------+
          |                         |
          v                         v
+------------------------+  +-----------------------------+
|   Gantry library       |  |  BergerdaServo::ServoDriver |
|   (motion controller)  |  |   lib/SDF08NK8X             |
|                        |  |                             |
|  GantryAxisStepper  ---|  |   LEDC pulse + PCNT encoder |
|  GantryRotaryServo  ---|  |   trapezoidal profile @5ms  |
|  GantryEndEffector  ---|  |   alarm / homing / travel   |
|  GantryLimitSwitch  ---|  +-----------------------------+
|  Kinematics            |                |
|  TrajectoryPlanner     |                |
|  Waypoint/Queue        |                |
+----+-------------+-----+                |
     |             |                      |
     v             v                      v
+---------------------+    +------------------------------+
|  gpio_expander      |    |  ESP-IDF peripherals         |
|  (MCP-only shim)    |    |  driver/gpio, driver/ledc,   |
|  include/gpio_...   |    |  driver/pulse_cnt,           |
+----------+----------+    |  driver/spi_master,          |
           |               |  esp_timer, freertos/*       |
           v               +------------------------------+
+---------------------+
|  MCP23S17 driver    |
|  lib/MCP23S17       |
|  (SPI, mutex, regs) |
+---------------------+
           |
           v
   [ MIKROE-951 MCP23S17 board over SPI2 @ 1 MHz ]
   [ 16 expander pins → DIR/EN/ALM/ARST/LIMITS/GRIPPER ]
```

Reading the stack top-down: the serial console and `app_main` drive the **Gantry** library. Gantry composes one **SDF08NK8X** servo driver for X (and optionally Y) plus three lightweight sub-axis helpers. Everything that is **not** a direct ESP32 peripheral (step pulse, PWM, quadrature encoder) routes through **gpio_expander**, which is a thin C shim over the **MCP23S17** SPI driver. Only the MCP23S17 layer talks directly to the SPI bus.

---

## 2. Library Catalogue

### 2.1 `lib/MCP23S17` — SPI GPIO expander driver

**Role:** Low-level driver for the Microchip MCP23S17 16-bit SPI GPIO expander chip. Knows the chip's register map and how to talk to it over an ESP-IDF SPI master bus.

**Language:** C (ESP-IDF style), works from both C and C++ callers.

**What it provides**

- A `mcp23s17_handle_t` opaque handle per device.
- Per-pin direction, pull-up, write, read.
- Port-level (8-bit) read/write for fast group operations.
- Interrupt configuration (pin-change).
- Debug-level raw register read/write (`IOCON`, `IODIR`, `GPPU`, `OLAT`, `GPIO`).
- **Internal FreeRTOS mutex** per device that wraps every `spi_device_transmit` call, so multiple tasks can share one expander safely.

**What it does not do**

- It does **not** know anything about which of its pins correspond to which signal in your application. The pin-to-role mapping lives in `include/gantry_app_constants.h` at the application layer.
- It does **not** know about direct ESP32 GPIO. That's the `gpio_expander` layer's job.
- It does **not** manage addresses for multiple MCP devices on the same bus beyond the `device_address` field in its config — the application owns the handle list.

**When to use**

- Any time you need to touch a pin on the MCP23S17 — but in practice you should go through `gpio_expander` instead so the "is this MCP or direct?" question has one answer.
- Direct use is appropriate only for register-level diagnostics (see the `mcp_reg`, `mcp_dump` console commands which call the `*_debug_*_register` helpers).

**Key files**

- `lib/MCP23S17/src/MCP23S17.h` — public C API
- `lib/MCP23S17/src/MCP23S17.cpp` — SPI transaction glue + mutex

**Why the mutex matters**

Without the per-device mutex, concurrent SPI transactions from `GantryUpdate` (reading limit/alarm pins) and `SerialCmd` (register dumps, `grip`) trigger `spi_device_transmit ... ret_trans == trans_desc` assertions. The mutex serializes every read/write register path inside the driver, so callers don't have to coordinate.

---

### 2.2 `include/gpio_expander.h` + `src/gpio_expander.c` — GPIO routing shim

**Role:** Tiny C abstraction sitting between the application and `MCP23S17`. Its job is to own **one** global MCP handle and decide whether a given pin integer means "MCP pin" or "this is invalid, talk to `driver/gpio.h` instead".

**Language:** C.

**API surface (5 functions + encoding macros)**

```c
bool      gpio_expander_init          (const mcp23s17_config_t*);
void      gpio_expander_deinit        (void);
esp_err_t gpio_expander_set_direction (int pin, bool is_output);
esp_err_t gpio_expander_set_pullup    (int pin, bool enable);
esp_err_t gpio_expander_write         (int pin, uint8_t level);
uint8_t   gpio_expander_read          (int pin);
```

Plus two helper macros:

- `GPIO_DIRECT_PIN_BASE = 0x10` — anything at or above this range is a direct ESP32 GPIO number.
- `GPIO_EXPANDER_DIRECT_PIN(n)` — flag a direct GPIO so callers can distinguish it from MCP pin `n` when the bare number would collide (e.g. GPIO2 vs. MCP A.2).

**Semantics**

- `0..15` → routes to `mcp23s17_*` on the single global handle.
- Anything else → returns `ESP_ERR_INVALID_ARG`. The caller is expected to handle direct GPIO themselves via Arduino (`pinMode`/`digitalWrite`) or IDF (`gpio_config`/`gpio_set_level`). `GantryEndEffector` is the canonical example of a consumer that branches on `isMcpLogicalPin` vs the encoded-direct flag.

**When to use**

- Whenever you have an application-level pin integer that *could* be an MCP pin (e.g. the values stored in `DriverConfig.output_pin_nos[]`). The shim isolates the decision.

**When not to use**

- High-frequency pulse outputs (step/PWM) — these need LEDC/GPIO hardware and must never go through SPI.

---

### 2.3 `lib/SDF08NK8X` — Bergerda SDF-08-N-K-8X servo driver

**Role:** Complete driver for the **Bergerda SDF-08-N-K-8X** industrial pulse-direction servo. Generates the step-pulse train, handles direction, monitors alarm/position-reached inputs, runs trapezoidal velocity profiles, counts encoder feedback via PCNT, and can measure axis travel length between limit switches.

**Language:** C++ (`namespace BergerdaServo`), Arduino-compatible but lives comfortably under ESP-IDF when the Arduino component is present.

**What it provides**

- `BergerdaServo::ServoDriver` — one instance per physical drive.
- `DriverConfig` struct with slot-by-slot documentation of which pin is `SON`/`ARST`/`ALM`/`PULS`/`SIGN`/encoder A/B.
- **Pulse generation via LEDC** (hardware PWM, 2-bit resolution, high frequency) — meaning the pulse stream is glitch-free and deterministic without CPU involvement.
- **Encoder counting via `pulse_cnt`** (ESP-IDF v5+ API) with a 64-bit accumulator on top to survive the hardware counter's 16-bit wrap.
- **Trapezoidal motion profile** implemented as an `esp_timer` callback running at 5 ms, with ACCEL/CRUISE/DECEL phases.
- `moveToPosition`, `moveRelative`, `stopMotion`, `eStop`, `startHoming`, `measureTravelDistance`.
- Alarm callbacks, position-reached callbacks, status callbacks — all invoked from timer-ISR context (must be ISR-safe).
- Optional FreeRTOS mutex (`SDF08NK8X_USE_FREERTOS=1`).
- String helpers (`getVersion`, `getConfigStatus`, `getDriverInfo`).

**What it does not do**

- It does **not** know about workspace coordinates or millimeters. Everything is in pulses. Millimeter conversion happens one layer up in the Gantry library via `getPulsesPerMm()`.
- It does **not** own limit switches. Limit ownership lives one layer up in `GantryLimitSwitch` so it can be reused across X, Y, and Theta. The `limit_min_pin`/`limit_max_pin` fields in `DriverConfig` are therefore typically `-1` when the driver is used under `Gantry`.
- It does **not** do kinematics or multi-axis coordination — it's a single-axis driver.

**Use cases**

- **X-axis under Gantry** — the standard use. `main.cpp` builds a `DriverConfig` with pulse/dir/enable/ARST/ALM routed appropriately and hands it to `Gantry::Gantry`.
- **Y-axis under Gantry** (optional) — Gantry has a two-argument constructor that takes a second `DriverConfig` for Y. Currently not wired in `main.cpp`; Y uses the lighter stepper path instead.
- **Standalone testing** — you can instantiate a `ServoDriver` directly from a sketch to verify the drive, independent of the rest of the gantry.

**Key files**

- `lib/SDF08NK8X/src/SDF08NK8X.h`
- `lib/SDF08NK8X/src/SDF08NK8X.cpp`

---

### 2.4 `lib/Gantry` — multi-axis motion controller

**Role:** Top-level controller that composes the servo driver, a stepper axis, a rotary servo, an end-effector, and two limit switches into a 3-DoF pick-and-place system with safe-height motion sequencing, kinematics, and trajectory planning.

**Language:** C++ (`namespace Gantry`).

#### 2.4.1 Core class: `Gantry::Gantry`

Holds one `BergerdaServo::ServoDriver axisX_`, an optional second `axisYServo_`, a `GantryAxisStepper axisY_`, a `GantryRotaryServo axisTheta_`, a `GantryEndEffector endEffector_`, and two `GantryLimitSwitch`es for X.

The interesting piece is the **sequential motion state machine**:

```
IDLE
  ↓  startSequentialMotion()
Y_DESCENDING       ← Y moves down to target if target < current
  ↓
GRIPPER_ACTUATING  ← grip open/close with 100 ms timeout
  ↓
Y_RETRACTING       ← Y returns to safeYHeight_mm_ before X is allowed to move
  ↓
X_MOVING           ← axisX_.moveRelative(...)
  ↓
THETA_MOVING       ← can happen in parallel; doesn't block the others
  ↓
IDLE
```

That's why an "X-only" looking move still gates on Y being at safe height first — it's there to stop the gripper from dragging through pallets or parts during X travel.

#### 2.4.2 Sub-modules (each a single-responsibility class)

| Class | Purpose | Backing hardware |
|---|---|---|
| `GantryAxisStepper` | Cooperative step/dir stepper with its own trapezoidal accel/decel and limit clamping. Driven from the `GantryUpdate` task @100 Hz (`update()`). | Direct GPIO for step/dir/enable, or LEDC for step when configured. |
| `GantryRotaryServo` | PWM hobby-servo driver with configurable angle-to-pulse mapping and pulse range. | LEDC channel. |
| `GantryEndEffector` | On/off digital output, active-high or active-low. Correctly branches to MCP expander for pins `<16` and to direct GPIO for pins `≥16` (or flagged direct). | MCP pin or direct GPIO. |
| `GantryLimitSwitch` | Active-low input with pull-up and N-sample debounce. Centralizes limit ownership so X, Y, and Theta can share one implementation. | MCP input pin. |

#### 2.4.3 Math helpers

- `Gantry::Kinematics::forward` / `::inverse` — map between joint space `(x_mm, y_mm, theta_deg)` and workspace `(x, y, z, theta)` using `KinematicParameters` (Y-axis Z offset, theta-X offset, gripper offsets, ball-screw pitch).
- `Gantry::Kinematics::validate` — bounds-check a `JointConfig` against `JointLimits`.
- `Gantry::TrajectoryPlanner::calculateProfile` / `::interpolate` — trapezoidal profile math (returns a `TrapezoidalProfile` struct usable for time-based interpolation of a single axis).

#### 2.4.4 Waypoint queue

`Gantry::Waypoint` + `Gantry::WaypointQueue<MAX_WAYPOINTS = 16>` — template-sized circular buffer for future multi-segment trajectory work. Not yet consumed by `Gantry::Gantry` itself; it's infrastructure for an upcoming multi-waypoint execution path.

**Use cases**

- **Pick-and-place with safe-height guarantee**: `moveTo(JointConfig{x, y, theta}, ...)` does the Y-down-grip-retract-X-move sequence automatically.
- **Direct joint positioning**: `moveTo(x, y, theta, speed_pps)` bypasses mm conversion and is useful for bench tests when you want to think in pulses.
- **Workspace positioning**: `moveTo(EndEffectorPose{...})` for code that reasons in (x, y, z, theta) workspace coordinates. Calls `inverseKinematics` under the hood.
- **Homing + calibration** flow: `home()` then `calibrate()` then normal `moveTo(...)` usage. The calibration result becomes the authoritative X max after `calibrate()` completes.

**Key files**

- `lib/Gantry/src/Gantry.{h,cpp}` — main class
- `lib/Gantry/src/GantryConfig.h` — all structs
- `lib/Gantry/src/GantryKinematics.{h,cpp}`
- `lib/Gantry/src/GantryTrajectory.{h,cpp}`
- `lib/Gantry/src/GantryAxisStepper.{h,cpp}`
- `lib/Gantry/src/GantryRotaryServo.{h,cpp}`
- `lib/Gantry/src/GantryEndEffector.{h,cpp}`
- `lib/Gantry/src/GantryLimitSwitch.{h,cpp}`
- `lib/Gantry/src/GantryUtils.h` — constants + assert macros
- `lib/Gantry/docs/` — additional deep-dive docs (`ARCHITECTURE.md`, `API_REFERENCE.md`, `CONFIGURATION_GUIDE.md`, `EXAMPLES.md`, `DEVELOPMENT_JOURNAL.md`).

---

## 3. Dependency Graph

```
main.cpp
  ├─> gantry_app_constants.h
  ├─> gpio_expander.h ───────> MCP23S17.h ─> driver/spi_master.h
  ├─> Gantry.h
  │     ├─> GantryConfig.h
  │     ├─> GantryKinematics.h ─> GantryConfig.h
  │     ├─> GantryTrajectory.h ─> GantryConfig.h
  │     ├─> GantryAxisStepper.h
  │     ├─> GantryRotaryServo.h ─> (ESP32: LEDC; else Arduino Servo)
  │     ├─> GantryEndEffector.h ─> gpio_expander.h
  │     ├─> GantryLimitSwitch.h
  │     ├─> GantryUtils.h
  │     └─> SDF08NK8X.h ─> driver/pulse_cnt.h, esp_timer.h
  └─> gantry_test_console.h

gantry_test_console.cpp
  ├─> Gantry.h                    # motion commands
  ├─> gantry_app_constants.h      # default speeds, unit handling
  ├─> gpio_expander.h             # mcp_* diagnostic commands
  └─> basic_tests.h               # selftest command
```

`gpio_expander` is deliberately a **single** C module with a global handle — there is exactly one MCP23S17 per controller board, and centralizing it here keeps the "which pin lives where" logic out of every consumer.

---

## 4. Data Flow for a Typical `move 50 0 0` Command

1. **`gantry_test_console.cpp`** parses `move 50 0 0`, confirms `home`+`calibrate` both ran this session, converts from the currently-selected unit to mm (`convertSelectedToMm`), builds a `Gantry::JointConfig`, and calls `gantry->moveTo(target, speedMmPerS, speedDegPerS, accel, decel)`.
2. **`Gantry::Gantry::moveTo(JointConfig, ...)`** validates with `Kinematics::validate(joint, config_.limits)`, stores targets, calls `startSequentialMotion()`.
3. **State machine** advances from `IDLE` through `Y_DESCENDING`/`GRIPPER_ACTUATING`/`Y_RETRACTING` as needed, finally reaching `X_MOVING` and invoking `startXAxisMotion()`.
4. **`startXAxisMotion()`** reads the current X encoder position, computes `deltaX` in pulses, converts speed/accel from mm units to pulses, and calls `axisX_.moveRelative(deltaX, speed_pps, accel_pps, decel_pps)`.
5. **`BergerdaServo::ServoDriver::moveRelative`** writes DIR (through `gpio_expander_write` → `mcp23s17_write_pin`, serialized by mutex), enables SON, arms the `esp_timer` ramp callback, and LEDC starts emitting the pulse train on `PIN_X_PULSE` (GPIO14).
6. **`GantryUpdate` task @100 Hz** calls `gantry.update()`, which calls `axisY_.update()` (stepper), debounces limit switches, and advances the motion state machine. In parallel, the `esp_timer` 5 ms callback updates the motion profile's phase/freq.
7. **Encoder feedback** via `pulse_cnt` is polled on demand in `axisX_.getEncoderPosition()` and surfaced through `gantry.getXEncoder()` for the `LIVE POS` telemetry and `status` command.
8. On completion, the driver's position-reached path flips `motionState_` back to `IDLE` and `gantry.isBusy()` returns `false`.

(See §11.1 of `PROGRAMMING_REFERENCE.md` for why this chain currently fails in practice for the `move` command.)

---

## 5. Use-Case Matrix

| What you want to do | Library / entry point | Example |
|---|---|---|
| Drive one MCP pin for bring-up | `gpio_expander_*` from any C/C++ file, or the `mcp_pin_mode` / `gpio_drive` console commands | `mcp_pin_mode 7 out1` turns the gripper on. |
| Inspect MCP internal state | `mcp23s17_debug_read_register` via `mcp_reg r 0x00` or `mcp_dump a` | Read IOCON/IODIR/GPPU/OLAT/GPIO per port. |
| Move a single axis without the full gantry stack | Instantiate `BergerdaServo::ServoDriver` directly with a `DriverConfig` | Useful for verifying a freshly-wired drive before involving Gantry. |
| Pick-and-place move in workspace coordinates | `Gantry::Gantry::moveTo(EndEffectorPose{...})` | Calls inverse kinematics; the state machine handles safe-Y. |
| Pick-and-place move in joint space | `Gantry::Gantry::moveTo(JointConfig{...})` | Skips inverse kinematics. |
| Test kinematics / trajectory math with no hardware | `runBasicTests()` or console `selftest` | Exercises `Kinematics::forward` and `TrajectoryPlanner::calculateProfile`. |
| Measure the physical X travel for setup | `gantry.calibrate()` or console `calibrate` | Drives MIN→MAX between limit switches, stores the result as the new X max. |
| Home the X axis | `gantry.home()` or console `home` | Drives to MIN limit at `GANTRY_HOMING_SPEED_PPS`. |
| Grip/release | `gantry.grip(bool)` or console `grip 1`/`grip 0` | Digital write to `PIN_GRIPPER` through `GantryEndEffector`. |
| Clear a servo alarm | `gantry.clearAlarm()` or console `arst` | Pulses ARST on MCP pin 5 via the driver. |
| Abort any in-flight motion safely | `gantry.requestAbort()` + `gantry.disable()` (console `stop`) | Also cancels active calibration. |

---

## 6. What *Isn't* in This Repo (intentional)

- **Networking / MQTT / Ethernet** — no MQTT client or Ethernet stack is wired up in the current firmware. Historical versions had an MQTT client running on the WT32's LAN8720; it was removed to keep bring-up focused on motion. Adding it back is a reintegration job, not a toggle.
- **OTA firmware update** — dependency explicitly removed (`a85cf68f chore(idf): remove unused mqtt dependency and OTA compile flag`).
- **Multi-MCP23S17 support** — the shim is single-device by design. If a second expander is added, `gpio_expander.c` will need a second global handle and an explicit pin-to-handle routing policy.
- **Hardware-timer ISR stepper** — `GantryAxisStepper` is deliberately cooperative (driven from `update()`), which caps usable Y speed. If Y needs higher speeds, the upgrade path is LEDC step generation (fields are already in place: `stepUseLedc_`, `lastLedcFreqHz_`) or a dedicated step-timer module.
- **Waypoint trajectory execution** — the `WaypointQueue` is scaffolding; the `Gantry::Gantry` class doesn't consume it yet. Hooking it into the motion state machine is the next real motion-planning task.

---

## 7. Where the Documentation Lives

| Document | What it covers |
|---|---|
| `PROGRAMMING_REFERENCE.md` | Build/flash, pin map, exact API signatures, console commands, known bugs, diagnostic toggles. |
| `LIBRARIES_OVERVIEW.md` (this file) | Purpose of each library, how they fit together, data flow, use cases. |
| `RESET_LOOP_DIAGNOSTICS.md` | The MCP23S17 boot-reset investigation and the GPIO-7 + SPI-mutex fixes. |
| `WT32_ETH01_PINOUT.md` | Full pinout of the WT32-ETH01 carrier. |
| `pinout.csv` | Machine-readable copy of the application pin assignments. |
| `lib/Gantry/README.md` | Gantry-library-local README. |
| `lib/Gantry/CHANGELOG.md` | Gantry library version history. |
| `lib/Gantry/docs/ARCHITECTURE.md` | Internal architecture of the Gantry library. |
| `lib/Gantry/docs/API_REFERENCE.md` | Deeper API walk-through with examples. |
| `lib/Gantry/docs/CONFIGURATION_GUIDE.md` | How to tune motion/kinematic parameters. |
| `lib/Gantry/docs/EXAMPLES.md` | Code snippets for common tasks. |
| `lib/Gantry/docs/DEVELOPMENT_JOURNAL.md` | Chronological engineering journal. |
| `lib/SDF08NK8X/ReadMe.md` | Servo-driver library notes. |
| `lib/MCP23S17/README.md` | MCP23S17 library usage. |

When in doubt: start at `PROGRAMMING_REFERENCE.md` for "what's the exact name / pin / command?" and at this file for "why does this thing exist?"

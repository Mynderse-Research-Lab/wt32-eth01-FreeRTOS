# Contributing

Software engineering rules for the WT32-ETH01 gantry firmware project.

## What this project is

ESP-IDF v6.0 firmware for a WT32-ETH01-based gantry pick-and-place controller.
Motion control (`Gantry`), Layer-2 closed cell transport (`CellNetL2`), and EtherNet/IP originator
(`EtherNetIP`) share one ESP32. **Production motion is EtherNet/IP only**
(W5500 $\rightarrow$ Kinetix X/Z and Rexroth HCS01 Theta).

## Quick start

```powershell
# Host tests (no ESP-IDF required)
cmake -S test/host -B build/host
cmake --build build/host
ctest --test-dir build/host --output-on-failure

# Firmware build (ESP-IDF v6.0 shell)
cd idf
idf.py set-target esp32   # first time only
idf.py build
idf.py -p COM3 flash monitor
```

CI runs both lanes and a Markdown link check on every push and pull request (`.github/workflows/firmware-ci.yml`).

## Project structure

| Directory | Role |
|-----------|------|
| `idf/` | ESP-IDF project root (build here) |
| `src/` | `app_main`, test console, pick scheduler |
| `include/` | App-wide constants and headers |
| `lib/` | ESP-IDF components (`Gantry`, `EtherNetIP`, `CellNetL2`, `EthernetLink`, `W5500`, `BootLogRing`, `GpioExpander`, `ThetaTrace`, `GantryOta`, ...) |
| `test/host/` | Native Unity tests (primary regression) |
| `docs/` | Hardware docs, schematics, the knowledge transfer page, open items and the doc index. Code docs live with the code: in `lib/<Name>/`, and at the repo root for `src/` |
| `driver_datasheets_and_calculations/` | Drive parameter sets, Kinetix reference data, generated BOM and wire schedule; its `INDEX.md` catalogs the vendor manuals |
| `tools/` | Python and Node helper scripts |

This is an **ESP-IDF-only** firmware project. Do not reintroduce PlatformIO or the Arduino core.

## Hard rules

- **ESP-IDF only** — no PlatformIO, no Arduino core in main firmware.
- **Build from `idf/`** — `idf.py build`, target `esp32`, FreeRTOS @ 1000 Hz.
- **Motion via `Gantry` only** — application layer must not drive motors directly.
- **EIP production path** — no PulseMotor / MCP / PTI for production control.
- **Runtime parameters are stored only** — motion and pick code reads `AppConfig` values
  (`gantryRuntimeParams()` for the Gantry library), never a compile-time macro.
  `idf/sdkconfig.defaults` is the only source of compile-time defaults.
- **Host-test pure logic** — kinematics, trajectory, protocol encoding before firmware wiring.
- **Minimize diff scope** — match existing naming, CMake layout, and comment style.

## Engineering principles

1. **Smallest correct change** — no unrelated refactors in the same PR.
2. **Follow existing patterns** — read surrounding code before adding new abstractions.
3. **Test hardware-independent logic on the host** — kinematics, trajectory, protocol encoding.
4. **Respect architecture boundaries** — motion goes through `Gantry`; network telemetry through `CellNetL2`.
5. **Keep docs in sync** — update `LOW_LEVEL_GANTRY_CONTROL.md` for API/build/console changes; update `lib/CellNetL2/CELL_NET_L2_COMMUNICATION_GUIDE.md` for network framing changes; update `docs/EXPECTED_ELECTROMECHANICAL_ASSEMBLY.md` / `docs/HV_LV_SCHEMATICS.md` for mechanical or electrical design changes.

## Testing policy

| Layer | When to use |
|-------|-------------|
| **Host tests** (`test/host/`) | Primary regression for math and encoding; required before merge |
| **ESP-IDF build** | Proves full firmware compiles; run locally for firmware changes |
| **On-target `selftest`** | Optional hardware smoke test; disable via `CONFIG_GANTRY_SELFTEST=n` for production |

### Testing gate

```powershell
cmake -S test/host -B build/host && cmake --build build/host
ctest --test-dir build/host --output-on-failure
```

Both must pass before a change is finalized. `.github/workflows/firmware-ci.yml` runs the same
two commands on every push, and checks every relative link in the Markdown files.


### Adding a host test

1. Add `test/host/test_<name>.cpp` with Unity `main()`.
2. Register it in `test/host/CMakeLists.txt` via `add_host_test()`.
3. Only include code that compiles without ESP-IDF.

See `test/README` for details.

## Naming conventions

- Library folders: **PascalCase** (`Gantry`, `EtherNetIP`, `CellNetL2`)
- Gantry API: **`namespace Gantry`**
- App headers: **snake_case** (`pick_scheduler.h`)
- EtherNet/IP types: **`Eip*` / `Cip*` / `Kinetix5100*`**
- Per-axis constants: **`include/axis_*_params.h`**

## Documentation

Every document is listed in [`docs/INDEX.md`](docs/INDEX.md), and each library under
`lib/` has a `README.md` and a `REFERENCE.md`. When documents disagree, prefer the
canonical docs listed there.

## Pull requests

- Ensure host tests pass locally.
- For firmware changes, confirm `idf.py build` succeeds.
- Describe what changed and why; note if work is host-tested but not yet firmware-integrated.
- Keep PRs focused — one feature or fix per PR when possible.
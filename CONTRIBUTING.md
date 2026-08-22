# Contributing

Software engineering rules for the WT32-ETH01 gantry firmware project.

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

CI runs both lanes on every push and pull request (`.github/workflows/firmware-ci.yml`).

## Project structure

| Directory | Role |
|-----------|------|
| `idf/` | ESP-IDF project root |
| `src/`, `include/` | Application code |
| `lib/` | Reusable ESP-IDF components |
| `test/host/` | Native Unity unit tests |
| `docs/` | Hardware integration and migration docs |
| `tools/` | Build/analysis scripts |

This is an **ESP-IDF-only** firmware project. Do not reintroduce PlatformIO or the Arduino core.

## Engineering principles

1. **Smallest correct change** — no unrelated refactors in the same PR.
2. **Follow existing patterns** — read surrounding code before adding new abstractions.
3. **Test hardware-independent logic on the host** — kinematics, trajectory, protocol encoding.
4. **Respect architecture boundaries** — motion goes through `Gantry`; network telemetry through `CellNetL2`.
5. **Keep docs in sync** — update `docs/LOW_LEVEL_GANTRY_CONTROL.md` for API/build/console changes; update `docs/CELL_NET_L2_COMMUNICATION_GUIDE.md` for network framing changes; update `docs/EXPECTED_ELECTROMECHANICAL_ASSEMBLY.md` / `docs/HV_LV_SCHEMATICS.md` for mechanical or electrical design changes.

## Testing policy

| Layer | When to use |
|-------|-------------|
| **Host tests** (`test/host/`) | Primary regression for math and encoding; required before merge |
| **ESP-IDF build** | Proves full firmware compiles; run locally for firmware changes |
| **On-target `selftest`** | Optional hardware smoke test; disable via `CONFIG_GANTRY_SELFTEST=n` for production |


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

## Documentation map

| Document | Use for |
|----------|---------|
| [`docs/LOW_LEVEL_GANTRY_CONTROL.md`](docs/LOW_LEVEL_GANTRY_CONTROL.md) | Build, APIs, EIP motion, console, tests |
| [`docs/EXPECTED_ELECTROMECHANICAL_ASSEMBLY.md`](docs/EXPECTED_ELECTROMECHANICAL_ASSEMBLY.md) | Mechanics, drives, networks, endstops |
| [`docs/HV_LV_SCHEMATICS.md`](docs/HV_LV_SCHEMATICS.md) | HV/LV schematics (preliminary) |
| [`docs/CELL_NET_L2_COMMUNICATION_GUIDE.md`](docs/CELL_NET_L2_COMMUNICATION_GUIDE.md) | OSI Layer-2 protocol & deployment manual |
| [`docs/PICK_SCHEDULER_KINEMATICS.md`](docs/PICK_SCHEDULER_KINEMATICS.md) | Pick scheduler coordinate frames and timing math |
| [`docs/FIRMWARE_REVIEW_AND_OPTIMIZATIONS.md`](docs/FIRMWARE_REVIEW_AND_OPTIMIZATIONS.md) | Firmware quality review & 8-stage verification test cycle |
| [`docs/DEVELOPMENT_ROADMAP.md`](docs/DEVELOPMENT_ROADMAP.md) | Architecture phases & Zenoh-DDS migration |
| [`docs/INDEX.md`](docs/INDEX.md) | Full documentation index |
| [`test/README`](test/README) | Testing overview |

When documents disagree, prefer the canonical docs under `docs/` listed above.

## Pull requests

- Ensure host tests pass locally.
- For firmware changes, confirm `idf.py build` succeeds.
- Describe what changed and why; note if work is host-tested but not yet firmware-integrated.
- Keep PRs focused — one feature or fix per PR when possible.
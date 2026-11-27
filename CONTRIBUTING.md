# Contributing

Software engineering rules for the WT32-ETH01 gantry firmware project.

## What this project is

ESP-IDF v6.0 firmware for a WT32-ETH01-based gantry pick-and-place controller.
Motion control (`Gantry`), Layer-2 closed cell transport (`CellNetL2`), and
EtherNet/IP originator (`EtherNetIP`) share one ESP32. **Production motion is
EtherNet/IP only** (W5500 $\rightarrow$ Kinetix X/Z and Rexroth HCS01 Theta).

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

CI runs both lanes and a Markdown link check on every push and pull request
(`.github/workflows/firmware-ci.yml`).

## Project structure

| Directory | Role |
|-----------|------|
| `idf/` | ESP-IDF project root (build here) |
| `src/` | `app_main`, test console, pick scheduler |
| `include/` | App-wide constants and headers |
| `lib/` | The 13 ESP-IDF components, listed by layer in [`lib/README.md`](lib/README.md) |
| `test/host/` | Native Unity tests (primary regression); see Testing policy below |
| `docs/` | Hardware docs, schematics, the knowledge transfer page, open items and the doc index. Code docs live with the code: in `lib/<Name>/`, and at the repo root for `src/` |
| `driver_datasheets_and_calculations/` | Drive parameter sets and Kinetix reference data; its `INDEX.md` catalogs the vendor manuals |
| `tools/` | Python and Node helper scripts |

This is an **ESP-IDF-only** firmware project. Do not reintroduce PlatformIO or
the Arduino core.

## Hard rules

-   **ESP-IDF only** — no PlatformIO, no Arduino core in main firmware.
-   **Build from `idf/`** — `idf.py build`, target `esp32`, FreeRTOS @ 1000 Hz.
-   **Motion via `Gantry` only** — application layer must not drive motors
    directly.
-   **EIP production path** — no PulseMotor / MCP / PTI for production control.
-   **Runtime parameters are stored only** — motion and pick code reads
    `AppConfig` values (`gantryRuntimeParams()` for the Gantry library), never a
    compile-time macro. `idf/sdkconfig.defaults` is the only source of
    compile-time defaults.
-   **Host-test pure logic** — kinematics, trajectory, protocol encoding before
    firmware wiring.
-   **Minimize diff scope** — match existing naming, CMake layout, and comment
    style.

## Engineering principles

1.  **Smallest correct change** — no unrelated refactors in the same PR.
2.  **Follow existing patterns** — read surrounding code before adding new
    abstractions.
3.  **Test hardware-independent logic on the host** — kinematics, trajectory,
    protocol encoding.
4.  **Respect architecture boundaries** — motion goes through `Gantry`; network
    telemetry through `CellNetL2`.
5.  **Keep docs in sync** — in the same commit, update the one doc that is the
    home of what you changed: the library's `README.md` or `REFERENCE.md`,
    `GANTRY_FIRMWARE.md` for `src/`, the hardware docs for mechanics and wiring,
    `docs/OPEN_ITEMS.md` for open items. Each fact has one home; link to it
    instead of restating it.

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

Both must pass before a change is finalized. `.github/workflows/firmware-ci.yml`
runs the same two commands on every push, and checks every relative link in the
Markdown files.

### Adding a host test

1.  Add `test/host/test_<name>.cpp` with Unity `main()`.
2.  Register it in `test/host/CMakeLists.txt` via `add_host_test()`.
3.  Only include code that compiles without ESP-IDF.

### Coverage report

Optional, with no percentage gate:

```powershell
cmake -S test/host -B build/host -DGANTRY_HOST_COVERAGE=ON
cmake --build build/host --target coverage
```

This rebuilds with `--coverage -O0`, re-runs ctest and writes
`build/host/coverage/index.html` and `summary.txt`. Install gcovr first
(`pip install gcovr`).

### On-target selftest

The console command `selftest` runs the kinematics, joint-limit and trajectory
assertions on the gantry (`src/basic_tests.cpp`) and ends with
`Selftest complete: passed=<N> failed=0`. It is built while
`CONFIG_GANTRY_SELFTEST=y`, the default; set it to `n` for production builds.

## Source files and components

-   **Source files stay under 500 lines.** A larger file is split by what its
    code does.
-   **A split moves code; it does not change it.** Each split is its own commit:
    function bodies are copied verbatim, every original line is used exactly
    once, and the only edits are to linkage (`static`, shared declarations in a
    private header, default arguments moved to the declaration). A function that
    is itself too long is broken up in a separate commit afterwards.
-   **Each split or move updates** the hand-kept source lists
    (`lib/*/CMakeLists.txt` SRCS, `idf/main/CMakeLists.txt`,
    `test/host/CMakeLists.txt`) and every `file:line` reference in the docs.
-   **Class 1 code needs extra care.** The firmware builds at `-Og` without
    link-time optimization, so a helper moved to another file is no longer
    inlined. Keep Class 1 helpers in their caller's file or `inline` in a
    private header, and count a Class 1 change as done only after an on-cell
    `eiptiming` and `gaptrace` soak.
-   **Gate for every commit:** host tests pass with unchanged Unity case counts,
    `idf.py build` shows no new warnings, and the touched files are under 500
    lines.
-   **Matured features become components.** A feature is matured when it runs on
    the gantry and has no open blocker or decision in `docs/OPEN_ITEMS.md`. It
    then moves out of `src/` into `lib/<Feature>/` with a `CMakeLists.txt`
    (`idf_component_register` with explicit REQUIRES; app headers through
    `${APP_ROOT}/include`), its code in `src/`, a `README.md` and a
    `REFERENCE.md`. The test console is the next candidate, once G-C7 is decided
    (OPEN_ITEMS X-7).

## Naming conventions

-   Library folders: **PascalCase** (`Gantry`, `EtherNetIP`, `CellNetL2`)
-   Gantry API: **`namespace Gantry`**
-   App headers: **snake_case** (`pick_scheduler.h`)
-   EtherNet/IP types: **`Eip*` / `Cip*` / `Kinetix5100*`**
-   Per-axis constants: **`include/axis_*_params.h`**

## Documentation

Each fact has one home, and other docs link to it. The reading path runs from
the [README](README.md) to the
[knowledge transfer page](docs/KNOWLEDGE_TRANSFER.md), the
[gantry firmware page](GANTRY_FIRMWARE.md), [`lib/README.md`](lib/README.md),
and each library's `README.md` and `REFERENCE.md`. Every doc on that path starts
with a breadcrumb and a "Read this when" line and ends with a "Next" section.
[`docs/INDEX.md`](docs/INDEX.md) lists every document. When documents disagree,
trust the code, then the doc that is the topic's home.

## Pull requests

-   Ensure host tests pass locally.
-   For firmware changes, confirm `idf.py build` succeeds.
-   Describe what changed and why; note if work is host-tested but not yet
    firmware-integrated.
-   Keep PRs focused — one feature or fix per PR when possible.

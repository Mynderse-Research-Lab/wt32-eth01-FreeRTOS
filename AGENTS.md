# Agent Guide

Entry point for AI coding agents working in this repository.

## What this project is

ESP-IDF v6.0 firmware for a WT32-ETH01-based gantry pick-and-place controller.
Motion control (`Gantry`), MQTT bridge (`MqttBridge`), and EtherNet/IP originator
(`EtherNetIP`) share one ESP32. **Production motion is EtherNet/IP only**
(W5500 → Kinetix X/Z; HCS01 theta deferred).

## Repo map

```
idf/          ESP-IDF project root (build here)
src/          app_main, console, pick scheduler
include/      app-wide constants and headers
lib/          ESP-IDF components (Gantry, EtherNetIP, MqttBridge, W5500, ...)
test/host/    native Unity tests (primary regression)
docs/         canonical design docs + schematics
tools/        Python/Node helper scripts
```

## Canonical references

| Topic | Document |
|-------|----------|
| Electro-mechanical assembly | `docs/EXPECTED_ELECTROMECHANICAL_ASSEMBLY.md` |
| HV/LV schematics | `docs/HV_LV_SCHEMATICS.md` |
| Low-level gantry control | `docs/LOW_LEVEL_GANTRY_CONTROL.md` |
| Bring-up status, open defects | `docs/DEV_TRACKER.md` |
| Doc index | `docs/INDEX.md` |
| MQTT bridge | `MQTT_comms_subsys.md` |
| Testing | `test/README`, `CONTRIBUTING.md` |

## Hard rules

- **ESP-IDF only** — no PlatformIO, no Arduino core in main firmware.
- **Build from `idf/`** — `idf.py build`, target `esp32`, FreeRTOS @ 1000 Hz.
- **Motion via `Gantry` only** — application layer must not drive motors directly.
- **EIP production path** — no PulseMotor / MCP / PTI for production control.
- **Host-test pure logic** — kinematics, trajectory, protocol encoding before firmware wiring.
- **Minimize diff scope** — match existing naming, CMake layout, and comment style.

## Testing gate

```powershell
cmake -S test/host -B build/host && cmake --build build/host
ctest --test-dir build/host --output-on-failure
```

Both must pass before a change is finalized. The commit hook
(`.cursor/hooks/devops_gate.py`) enforces this locally, and
`.github/workflows/firmware-ci.yml` runs the same two commands on every push.

## Cursor rules

Detailed guidance is in `.cursor/rules/` (`project-core`, `coding-standards`,
`motion-safety`, `firmware-esp-idf`, `host-testing`, `domains-mqtt-eip`,
`git-conventions`, `peak-hours-alert`).

Human contributors should read [`CONTRIBUTING.md`](CONTRIBUTING.md).

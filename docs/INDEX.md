# Project documentation index

Standalone index for everything under `docs/`. This file is independent of [`driver_datasheets_and_calculations/INDEX.md`](../driver_datasheets_and_calculations/INDEX.md) (vendor PDF catalog).

---

## Hardware / integration

| Document | Summary | Audience |
|----------|---------|----------|
| [MOTION_IO_INTERFACE.md](MOTION_IO_INTERFACE.md) | WT32 → opto-isolated → K5100 (X/Z) + HCS01 (theta) wiring, parts list, commissioning | Panel wiring, bench bring-up |

---

## Firmware / software

| Document | Summary | Audience |
|----------|---------|----------|
| [../CONTRIBUTING.md](../CONTRIBUTING.md) | Engineering rules, testing policy, PR expectations | All contributors |
| [../AGENTS.md](../AGENTS.md) | AI agent entry point and repo map | Cursor / coding agents |
| [../PROGRAMMING_REFERENCE.md](../PROGRAMMING_REFERENCE.md) | Pin map, Gantry API, console, bring-up | Firmware developers |
| [../lib/Gantry/docs/ARCHITECTURE_FLOW.md](../lib/Gantry/docs/ARCHITECTURE_FLOW.md) | Control/feedback signal routing | Architecture review |
| [../MQTT_comms_subsys.md](../MQTT_comms_subsys.md) | MQTT bridge design | Networking / scheduler |
| [../Pickup_algo_and_MQTTBridge_SRS.md](../Pickup_algo_and_MQTTBridge_SRS.md) | Pick scheduler requirements | Integration |
| [EIP_MIGRATION.md](EIP_MIGRATION.md) | EtherNet/IP wire format and migration status | Firmware / networking |

---

## SRS build artifacts

| Path | Summary |
|------|---------|
| [srs/](srs/) | SRS source and build tooling (see `tools/srs_build/`) |

---

## Related vendor PDFs

Filenames live under `../driver_datasheets_and_calculations/`. Commonly used for motion I/O:

| File | Topic |
|------|-------|
| `AB_Kinetix5100_user_manual.pdf` | K5100 PTI, digital I/O, commissioning |
| `R911325518_07_EN_IndraDrive Drive Controllers Power Sections HCS01_Commissioning Manual.pdf` | HCS01 X13, X31, X47 |
| `Rexroth HCS-01 Drive Project Planning Manual R911322210_03.pdf` | HCS01 type code, control voltage sizing |

---

## Missing documents (motion I/O)

| Document | Blocks |
|----------|--------|
| **2198-IN020** (K5100 TBIO installation) | Formal TBIO pin drawing (informal pin list is in MOTION_IO_INTERFACE.md) |
| **MPB-xxVRS Functional Description** (Rexroth) | Exact HCS01 X31 Step/Dir/Enable pin numbers |

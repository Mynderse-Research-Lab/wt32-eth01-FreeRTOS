# Project Documentation Index

Canonical product-design and software architecture set (EIP production architecture):

| Document | Summary | Audience |
|----------|---------|----------|
| [Expected electro-mechanical assembly](EXPECTED_ELECTROMECHANICAL_ASSEMBLY.md) | Mechanics / drives / networks | Mechanical / panel / commissioning |
| [HV/LV schematics](HV_LV_SCHEMATICS.md) | Electrical design basis (preliminary) | Electrical review |
| [Nomenclature CAD Mapping](NOMENCLATURE_CAD_MAPPING.md) | Universal descriptors linking MECH/ELEC/CODE subsystems | All |
| [Low-level gantry control](LOW_LEVEL_GANTRY_CONTROL.md) | Firmware / EIP / console | Firmware developers |
| [FIRMWARE_REVIEW_AND_OPTIMIZATIONS.md](FIRMWARE_REVIEW_AND_OPTIMIZATIONS.md) | Firmware review, data integrity fixes, memory optimizations, and 8-stage holistic test cycle | Firmware developers / QA |
| [CELL_NET_L2_COMMUNICATION_GUIDE.md](CELL_NET_L2_COMMUNICATION_GUIDE.md) | OSI Layer-2 real-time protocol specification, data integrity model, and multi-platform deployment manual (ESP32 / Linux) | Subsystem developers (Vision, Conveyor, Gantry, Supervisor) |
| [PICK_SCHEDULER_KINEMATICS.md](PICK_SCHEDULER_KINEMATICS.md) | Authoritative coordinate conventions (+Y downstream, +Z down) and feasibility/timing algorithms ($\tau = D / v_{belt}$) for the Pick Scheduler. | Firmware / Subsystem developers |
| [DEV_TRACKER.md](DEV_TRACKER.md) | Active hardware bring-up and firmware milestone tracker | Firmware / commissioning |
| [DEVELOPMENT_ROADMAP.md](DEVELOPMENT_ROADMAP.md) | Phase 5 & Phase 6 architecture roadmap (Network extraction, Zenoh-DDS migration) | System architects / developers |

Panel reference: [`schematics/Gantry panel.drawio (7).png`](<schematics/Gantry panel.drawio (7).png>).

---

## Tooling & Guidelines

| Document | Summary |
|----------|---------|
| [../CONTRIBUTING.md](../CONTRIBUTING.md) | Engineering rules, testing policy, git workflow |
| [../AGENTS.md](../AGENTS.md) | AI agent entry point and hard rules |
| [MCP_SETUP.md](MCP_SETUP.md) | Cursor / Antigravity MCP server setup |
| [../test/README](../test/README) | Host unit tests & on-target test documentation |
| [../driver_datasheets_and_calculations/INDEX.md](../driver_datasheets_and_calculations/INDEX.md) | Vendor PDF catalog |

---

## Vendor references (commonly cited)

| File under `driver_datasheets_and_calculations/` | Topic |
|--------------------------------------------------|-------|
| `AB_Kinetix5100_user_manual.pdf` | K5100 commissioning / I/O |
| `2198-in003_-en-p.pdf` | Kinetix installation |
| `R911325518_07_EN_...HCS01_Commissioning Manual.pdf` | HCS01 connectors |
| `Rexroth HCS-01 Drive Project Planning Manual R911322210_03.pdf` | HCS01 type code / control voltage |
| `SCHUNK_design_technical_info.pdf` | Actuator / gripper selection |
| `BOM.xlsx` / `WIRE_SIZE_SELECTION.xlsx` | Generated panel BOM / wire schedule |

---

## Explicitly rejected (do not restore as production docs)

- Pulse-Train / MCP23S17 / opto PTI interface as production motion path
- Speed+TM10 host StopMotion for PTP accuracy
- Duplicate generated markdown copies of vendor PDFs as authoritative sources

# Project Documentation Index

Canonical product-design and software architecture set (EIP production architecture):

| Document | Summary | Audience |
|----------|---------|----------|
| [KNOWLEDGE_TRANSFER.md](KNOWLEDGE_TRANSFER.md) | Start here: subsystems, how they connect, components and working rules | New engineers |
| [OPEN_ITEMS.md](OPEN_ITEMS.md) | Live list of open items, blockers and decisions, with closed items kept for reference | Everyone, before starting work |
| [CODE_ENCAPSULATION.md](CODE_ENCAPSULATION.md) | Every source file with its size, the plan to keep files under 500 lines, and the move of matured features into `lib/` components | Firmware developers |
| [LIBRARY_REVIEW.md](LIBRARY_REVIEW.md) | Review of every library in `lib/`: purpose, API, threading, findings with evidence labels, and the README and reference work each needs | Firmware developers |
| [EXPECTED_ELECTROMECHANICAL_ASSEMBLY.md](EXPECTED_ELECTROMECHANICAL_ASSEMBLY.md) | Expected mechanics, drives, networks, endstops, scaling, verification | Mechanical / panel / commissioning |
| [HV_LV_SCHEMATICS.md](HV_LV_SCHEMATICS.md) | HV/LV design basis + supplied panel image; logic-board schematic pending (**PRELIMINARY — NOT FOR CONSTRUCTION**) | Electrical review |
| [LOW_LEVEL_GANTRY_CONTROL.md](LOW_LEVEL_GANTRY_CONTROL.md) | Firmware design: bring-up, Gantry SMs, Class 1 UDP, Absolute PTP, UART + TCP console, host tests | Firmware developers |
| [FIRMWARE_REVIEW_AND_OPTIMIZATIONS.md](FIRMWARE_REVIEW_AND_OPTIMIZATIONS.md) | Firmware review, data integrity fixes, memory optimizations, and 8-stage holistic test cycle | Firmware developers / QA |
| [CELL_NET_L2_COMMUNICATION_GUIDE.md](CELL_NET_L2_COMMUNICATION_GUIDE.md) | CellNet Layer-2 wire specification: framing, receive rules, payloads, library use and diagnostics | Subsystem developers (Vision, Conveyor, Gantry, Supervisor) |
| [CELLNET_OTA_PROTOCOL.md](CELLNET_OTA_PROTOCOL.md) | Firmware updates over CellNet L2 (N-2): OTA-only mode, messages, flow control, timing | Gantry, Conveyor and Supervisor developers |
| [PICK_SCHEDULER_KINEMATICS.md](PICK_SCHEDULER_KINEMATICS.md) | Authoritative coordinate conventions (+Y downstream, +Z down) and feasibility/timing algorithms ($\tau = D / v_{belt}$) for the Pick Scheduler. | Firmware / Subsystem developers |
| [ON_THE_FLY_PICKUP.md](ON_THE_FLY_PICKUP.md) | End-to-end mathematical narrative of one pickup, from the camera observation through the belt, grasp, bin, and the following object. | Firmware / mechanical / vision |
| [DEV_TRACKER.md](DEV_TRACKER.md) | Active hardware bring-up and firmware milestone tracker | Firmware / commissioning |
| [DEVELOPMENT_ROADMAP.md](DEVELOPMENT_ROADMAP.md) | Phase 5 & Phase 6 architecture roadmap (Network extraction, Zenoh-DDS migration) | System architects / developers |

Panel reference: [`schematics/Gantry panel.drawio (7).png`](<schematics/Gantry panel.drawio (7).png>).

---

## Tooling & Guidelines

| Document | Summary |
|----------|---------|
| [../CONTRIBUTING.md](../CONTRIBUTING.md) | Engineering rules, testing policy, git workflow |
| [../test/README](../test/README) | Host unit tests & on-target test documentation |
| [../driver_datasheets_and_calculations/INDEX.md](../driver_datasheets_and_calculations/INDEX.md) | Catalog of the vendor manuals (kept in a shared folder), drive parameter sets and generated panel files |

---

## Vendor references

Vendor manuals are cited by publication number and section. The [vendor catalog](../driver_datasheets_and_calculations/INDEX.md) lists them and links the shared folder that holds them.

---

## Explicitly rejected (do not restore as production docs)

- Pulse-Train / MCP23S17 / opto PTI interface as production motion path
- Speed+TM10 host StopMotion for PTP accuracy
- Duplicate generated markdown copies of vendor PDFs as authoritative sources

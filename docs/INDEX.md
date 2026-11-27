# Project Documentation Index

Code documentation lives with the code: each library's docs are in `lib/<Name>/`, and the docs that cover the code in `src/` are at the repository root. This folder keeps the hardware docs, the knowledge transfer page, the open items and this index.

Canonical product-design and software architecture set (EIP production architecture):

| Document | Summary | Audience |
|----------|---------|----------|
| [KNOWLEDGE_TRANSFER.md](KNOWLEDGE_TRANSFER.md) | Start here: subsystems, how they connect, components and working rules | New engineers |
| [OPEN_ITEMS.md](OPEN_ITEMS.md) | Live list of open items, blockers and decisions, with closed items kept for reference | Everyone, before starting work |
| [CODE_ENCAPSULATION.md](../CODE_ENCAPSULATION.md) | Rules for keeping source files under 500 lines, the splits done, and the move of matured features into `lib/` components | Firmware developers |
| [LIBRARY_REVIEW.md](../LIBRARY_REVIEW.md) | Review of every library in `lib/`: purpose, API, threading, findings with evidence labels, and the README and reference work each needs | Firmware developers |
| [MATH_RELINK_INVENTORY.md](../MATH_RELINK_INVENTORY.md) | Every piece of code that implements system math, by topic, to be linked to the new math document | Firmware developers |
| [EXPECTED_ELECTROMECHANICAL_ASSEMBLY.md](EXPECTED_ELECTROMECHANICAL_ASSEMBLY.md) | Expected mechanics, drives, networks, endstops, scaling, verification | Mechanical / panel / commissioning |
| [HV_LV_SCHEMATICS.md](HV_LV_SCHEMATICS.md) | HV/LV design basis + supplied panel image; logic-board schematic pending (**PRELIMINARY — NOT FOR CONSTRUCTION**) | Electrical review |
| [LOW_LEVEL_GANTRY_CONTROL.md](../LOW_LEVEL_GANTRY_CONTROL.md) | Firmware design: bring-up, Gantry SMs, Class 1 UDP, Absolute PTP, UART + TCP console, host tests | Firmware developers |
| [FIRMWARE_REVIEW_AND_OPTIMIZATIONS.md](../FIRMWARE_REVIEW_AND_OPTIMIZATIONS.md) | Firmware review, data integrity fixes, memory optimizations, and 8-stage holistic test cycle | Firmware developers / QA |
| [CELL_NET_L2_COMMUNICATION_GUIDE.md](../lib/CellNetL2/CELL_NET_L2_COMMUNICATION_GUIDE.md) | CellNet Layer-2 wire specification: framing, receive rules, payloads, library use and diagnostics | Subsystem developers (Vision, Conveyor, Gantry, Supervisor) |
| [CELLNET_OTA_PROTOCOL.md](../lib/CellNetL2/CELLNET_OTA_PROTOCOL.md) | Firmware updates over CellNet L2 (N-2): OTA-only mode, messages, flow control, timing | Gantry, Conveyor and Supervisor developers |
| [PICK_BENCH_RAMP.md](../PICK_BENCH_RAMP.md) | Bench ramp for the coordinated grasp (not runnable yet) | Firmware / commissioning |
| [PICK_SCHEDULER_KINEMATICS.md](PICK_SCHEDULER_KINEMATICS.md) | Authoritative coordinate conventions (+Y downstream, +Z down) and feasibility/timing algorithms ($\tau = D / v_{belt}$) for the Pick Scheduler. | Firmware / Subsystem developers |
| [ON_THE_FLY_PICKUP.md](ON_THE_FLY_PICKUP.md) | End-to-end mathematical narrative of one pickup, from the camera observation through the belt, grasp, bin, and the following object. | Firmware / mechanical / vision |

Panel reference: [`schematics/Gantry panel.drawio (7).png`](<schematics/Gantry panel.drawio (7).png>).

## Dated records

| Document | Summary |
|----------|---------|
| [ISSUES_AND_RESOLUTIONS_2026-10-20.md](../ISSUES_AND_RESOLUTIONS_2026-10-20.md) | Class 1 dropout campaign (2026-10-20/21): each issue with its evidence and resolution |
| [STRIKE_TIMING_MEASUREMENTS_2026-10-25.md](../STRIKE_TIMING_MEASUREMENTS_2026-10-25.md) | On-gantry measurements of the Z strike against the strike plan |
| [REXROTH_SUPPORT_HCS01_CIP_IP.md](REXROTH_SUPPORT_HCS01_CIP_IP.md) | Rexroth support request (2026-08): the HCS01 EtherNet/IP fieldbus IP was not writable, with later updates |

---

## Tooling & Guidelines

| Document | Summary |
|----------|---------|
| [../CONTRIBUTING.md](../CONTRIBUTING.md) | Engineering rules, testing policy, git workflow |
| [../test/README](../test/README) | Host unit tests & on-target test documentation |
| [../lib/README](../lib/README) | Libraries in `lib/`; each has its own `README.md` and `REFERENCE.md` |
| [../tools/README.md](../tools/README.md) | Helper scripts |
| [../driver_datasheets_and_calculations/INDEX.md](../driver_datasheets_and_calculations/INDEX.md) | Catalog of the vendor manuals (kept in a shared folder), drive parameter sets and generated panel files |

---

## Vendor references

Vendor manuals are cited by publication number and section. The [vendor catalog](../driver_datasheets_and_calculations/INDEX.md) lists them and links the shared folder that holds them.

---

## Explicitly rejected (do not restore as production docs)

- Pulse-Train / MCP23S17 / opto PTI interface as production motion path
- Speed+TM10 host StopMotion for PTP accuracy
- Duplicate generated markdown copies of vendor PDFs as authoritative sources

# Project documentation index

*[README](/README.md) › Doc index*

**Read this when** you are looking for one specific document. To learn the
system, follow the reading path from the [README](/README.md) instead.

Code documentation lives with the code: each library's docs are in
`lib/<Name>/`, and the docs that cover the code in `src/` are at the repository
root. This folder keeps the hardware docs, the knowledge transfer page, the open
items and this index.

Canonical product-design and software architecture set (EIP production
architecture):

| Document | Summary | Audience |
|----------|---------|----------|
| [KNOWLEDGE_TRANSFER.md](KNOWLEDGE_TRANSFER.md) | Start here: subsystems, how they connect, components and working rules | New engineers |
| [OPEN_ITEMS.md](OPEN_ITEMS.md) | Live list of open items, blockers and decisions, with closed items kept for reference | Everyone, before starting work |
| [MATH_RELINK_INVENTORY.md](/MATH_RELINK_INVENTORY.md) | Every piece of code that implements system math, by topic, to be linked to the new math document | Firmware developers |
| [EXPECTED_ELECTROMECHANICAL_ASSEMBLY.md](EXPECTED_ELECTROMECHANICAL_ASSEMBLY.md) | Expected mechanics, drives, networks, endstops, scaling, verification | Mechanical / panel / commissioning |
| [HV_LV_SCHEMATICS.md](HV_LV_SCHEMATICS.md) | HV/LV design basis + supplied panel image; logic-board schematic pending (**PRELIMINARY — NOT FOR CONSTRUCTION**) | Electrical review |
| [GANTRY_FIRMWARE.md](/GANTRY_FIRMWARE.md) | How the gantry firmware works: what runs, boot, tasks, one move and one pick end to end, the console, runtime parameters | Firmware developers |
| [CELL_NET_L2_COMMUNICATION_GUIDE.md](/lib/CellNetL2/CELL_NET_L2_COMMUNICATION_GUIDE.md) | CellNet Layer-2 wire specification: framing, receive rules, payloads, library use and diagnostics | Subsystem developers (Vision, Conveyor, Gantry, Supervisor) |
| [CELLNET_OTA_PROTOCOL.md](/lib/CellNetL2/CELLNET_OTA_PROTOCOL.md) | Firmware updates over CellNet L2 (N-2): OTA-only mode, messages, flow control, timing | Gantry, Conveyor and Supervisor developers |
| [PICK_SCHEDULER_KINEMATICS.md](PICK_SCHEDULER_KINEMATICS.md) | Authoritative coordinate conventions (+Y downstream, +Z down) and feasibility/timing algorithms ($\tau = D / v_{belt}$) for the Pick Scheduler. | Firmware / Subsystem developers |
| [ON_THE_FLY_PICKUP.md](ON_THE_FLY_PICKUP.md) | End-to-end mathematical narrative of one pickup, from the camera observation through the belt, grasp, bin, and the following object. | Firmware / mechanical / vision |

Panel reference:
[`schematics/Gantry panel.drawio (7).png`](<schematics/Gantry panel.drawio (7).png>).

## Dated records

| Document | Summary |
|----------|---------|
| [STRIKE_TIMING_MEASUREMENTS_2026-10-25.md](/STRIKE_TIMING_MEASUREMENTS_2026-10-25.md) | On-gantry measurements of the Z strike against the strike plan |

---

## Tooling and guidelines

| Document | Summary |
|----------|---------|
| [CONTRIBUTING.md](/CONTRIBUTING.md) | Engineering rules, testing policy, git workflow |
| [lib/README.md](/lib/README.md) | Libraries in `lib/`; each has its own `README.md` and `REFERENCE.md` |
| [tools/README.md](/tools/README.md) | Helper scripts |
| [driver_datasheets_and_calculations/INDEX.md](/driver_datasheets_and_calculations/INDEX.md) | Catalog of the vendor manuals (kept in a shared folder), drive parameter sets and Kinetix reference data |

---

## Vendor references

Vendor manuals are cited by publication number and section. The
[vendor catalog](/driver_datasheets_and_calculations/INDEX.md) lists them and
links the shared folder that holds them.

---

## Explicitly rejected (do not restore as production docs)

-   Pulse-Train / MCP23S17 / opto PTI interface as production motion path
-   Speed+TM10 host StopMotion for PTP accuracy
-   Duplicate generated markdown copies of vendor PDFs as authoritative sources

# Project documentation index

Canonical product-design set (EIP production architecture):

| Document | Summary | Audience |
|----------|---------|----------|
| [EXPECTED_ELECTROMECHANICAL_ASSEMBLY.md](EXPECTED_ELECTROMECHANICAL_ASSEMBLY.md) | Expected mechanics, drives, networks, endstops, scaling, verification | Mechanical / panel / commissioning |
| [HV_LV_SCHEMATICS.md](HV_LV_SCHEMATICS.md) | HV/LV design basis + supplied panel image; logic-board schematic pending (**PRELIMINARY — NOT FOR CONSTRUCTION**) | Electrical review |
| [LOW_LEVEL_GANTRY_CONTROL.md](LOW_LEVEL_GANTRY_CONTROL.md) | Firmware design: bring-up, Gantry SMs, Class 1 UDP, Absolute PTP, UART + TCP console, host tests | Firmware developers |

Panel reference: [`schematics/Gantry panel.drawio (7).png`](<schematics/Gantry panel.drawio (7).png>).

---

## Requirements / policy / tooling (retained)

| Document | Summary |
|----------|---------|
| [../CONTRIBUTING.md](../CONTRIBUTING.md) | Engineering rules, testing policy |
| [../AGENTS.md](../AGENTS.md) | AI agent entry point |
| [../Pickup_algo_and_MQTTBridge_SRS.md](../Pickup_algo_and_MQTTBridge_SRS.md) | Pick scheduler requirements |
| [../MQTT_comms_subsys.md](../MQTT_comms_subsys.md) | MQTT bridge design |
| [MCP_SETUP.md](MCP_SETUP.md) | Cursor MCP server setup |
| [SERIAL_OVER_LAN_PLAN.md](SERIAL_OVER_LAN_PLAN.md) | TCP ESP_LOG + console; free GPIO1/3; Python UI |
| [../test/README](../test/README) | Host / on-target tests |
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

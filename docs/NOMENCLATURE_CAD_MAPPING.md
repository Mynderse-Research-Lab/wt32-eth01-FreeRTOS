# Universal Subsystem Descriptor Schema

To keep the mechanical CAD assembly, electrical schematics, and firmware codebase perfectly aligned, we use a single, unified reference designator scheme. 

**Format:** `[SUBSYSTEM]-[ID]`

By removing domain-specific prefixes (like `MECH-` or `ELEC-`), the exact same string can be used as the CAD Subassembly name, the electrical wire label, and the C++ firmware variable prefix.

---

## 1. Gantry X-Axis (Cross-Belt)
Spans across the conveyor belt. Firmware coordinate frame: **+X**.

| Descriptor ID | Domain Use Case | Description / Mapping |
| --- | --- | --- |
| **`AX-X`** | CAD | X-Axis linear actuator (ballscrew/belt), carriage, and motor mount (e.g. AB TLP). |
| **`AX-X`** | Electrical | Kinetix 5100 Drive (.20), motor power/feedback cables, limit switches (X31 TBIO). |
| **`AX-X`** | Firmware | `axisX_` (GantryEipLinearAxis), `AXIS_X_` constants in `axis_drivetrain_params.h`. |

## 2. Gantry Z-Axis (Vertical Descent)
Descends toward the conveyor. Firmware coordinate frame: **+Z is DOWN**.

| Descriptor ID | Domain Use Case | Description / Mapping |
| --- | --- | --- |
| **`AX-Z`** | CAD | Z-Axis linear actuator, motor mount. |
| **`AX-Z`** | Electrical | Kinetix 5100 Drive (.21), motor power/feedback cables. |
| **`AX-Z`** | Firmware | `axisZ_` (GantryEipLinearAxis), `AXIS_Z_` parameters, `GANTRY_SAFE_Z_HEIGHT_MM`. |

## 3. Gantry Theta-Axis (End Effector Rotation)
Rotates the gripper. Firmware coordinate frame: **Right-handed about +Z**.

| Descriptor ID | Domain Use Case | Description / Mapping |
| --- | --- | --- |
| **`AX-TH`** | CAD | Rotary actuator (SCHUNK ERD), Theta carriage mount, slip ring / routing bracket. |
| **`AX-TH`** | Electrical | Rexroth HCS01 Drive (.23 fieldbus / .22 engineering IP), IndraDrive motor. |
| **`AX-TH`** | Firmware | `axisTheta_` (GantryEipRotaryAxis), `AXIS_THETA_` parameters. |

## 4. End Effector / Tooling
The physical mechanism handling the payload.

| Descriptor ID | Domain Use Case | Description / Mapping |
| --- | --- | --- |
| **`EFF-01`** | CAD | Pneumatic gripper assembly, vacuum cup, or parallel jaw tooling. |
| **`EFF-01`** | Electrical | Solenoid valve routed to `FIELD_24V_DOUT0` (MCP23S17 PA0). |
| **`EFF-01`** | Firmware | `GantryEndEffector`, `PIN_GRIPPER`. |

## 5. Vision / Camera System
Upstream inspection component.

| Descriptor ID | Domain Use Case | Description / Mapping |
| --- | --- | --- |
| **`VIS-01`** | CAD | Overhead camera mount, illumination shroud. |
| **`VIS-01`** | Electrical | Vision processing node (e.g. Raspberry Pi / Ubuntu PC). |
| **`VIS-01`** | Firmware | Camera coordinate datum `Topology::kCameraY_mm`. |

## 6. Conveyor / Material Handling
Moves payload downstream. Firmware coordinate frame: **+Y is DOWNSTREAM**.

| Descriptor ID | Domain Use Case | Description / Mapping |
| --- | --- | --- |
| **`CONV-01`** | CAD | Main conveyor belt assembly, framing, roller. |
| **`CONV-01`** | Electrical | Conveyor VFD / Drive, main belt encoder. |
| **`CONV-01`** | Firmware | Pick intercept plane datum `Topology::kPickPlaneY_mm`. |

## 7. System Controller
The "brain" of the cell.

| Descriptor ID | Domain Use Case | Description / Mapping |
| --- | --- | --- |
| **`SYS-CTRL`** | CAD | Main electronics enclosure, DIN rails, WT32-ETH01 carrier board mounts. |
| **`SYS-CTRL`** | Electrical | WT32-ETH01, LAN8720, WIZ850io (W5500), SPI3 bus isolators, MCP23S17, 24V PSU. |
| **`SYS-CTRL`** | Firmware | `Gantry::Gantry` singleton, `EtherNetIP` class 1 originator, `CellNetL2` stack. |

---

### Implementation Rule:
When structuring the feature tree or BOM in SolidWorks, the top-level assembly for that subsystem MUST be named identically to the Descriptor ID (e.g., `AX-X.SLDASM`).
This allows an engineer debugging firmware (`AX-X` variables) to instantly locate the exact mechanical assembly and the exact electrical schematic page.

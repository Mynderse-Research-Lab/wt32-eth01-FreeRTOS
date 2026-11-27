# HV and LV schematics

*[README](/README.md) › [Knowledge transfer](KNOWLEDGE_TRANSFER.md) ›
Panel electrics*

**Read this when** you work on the panel's power, voltage domains or connection
schedule.

**Status:** Canonical electrical design basis.

**Panel schematic:**
[`schematics/Gantry panel.drawio (7).png`](<schematics/Gantry panel.drawio (7).png>).

**Logic-board schematic:** Pending addition.

**Mechanical / software companions:**
[EXPECTED_ELECTROMECHANICAL_ASSEMBLY.md](EXPECTED_ELECTROMECHANICAL_ASSEMBLY.md),
[GANTRY_FIRMWARE.md](/GANTRY_FIRMWARE.md).

> **PRELIMINARY — NOT FOR CONSTRUCTION.**
>
> This package documents the intended HV/LV architecture for review. It does
> **not** constitute a finished safety-validated panel design. Do not build or
> energize from these drawings until the electrical-review register below is
> closed by a qualified reviewer.

---

## 1. Design basis

| Domain | Basis |
|--------|-------|
| AC supply | **200–240 VAC** three-phase class (Kinetix E10xx 170–253 V). Not 480Y drive feed. |
| Drive branches | X E1020 / Z E1004 / Theta HCS01 with filters + E-stop contactors |
| Control power | 1606-XLE240E 24 VDC; Phoenix 5 V for WT32 box |
| Motion control | **EtherNet/IP only** — no PTI/opto production harness |
| Endstops | Drive TBIO / HCS01 X31 (NC sinking) |
| Gripper | MCP23S17 PA0 (Field DOUT0) → R16 220 Ω → U11 VO14642AT solid-state relay (high side: 24VDC on S/S′, DC → FDO0) → valve coil. Turn-on 370 µs typ / 800 µs max at 30 V, 200 mA (Vishay doc 81646, Switching Characteristics). R14 10 kΩ pulls PA0 low. |
| Networks | W5500 EIP daisy-chain **separate** from the LAN8720 cell network (CellNet L2) |

Vendor install references: Rockwell 2198-IN003 / UM004, Rexroth HCS01
commissioning + project planning (listed in the
[vendor catalog](/driver_datasheets_and_calculations/INDEX.md)).

---

## 2. Panel schematic reference

The complete panel-level diagram is maintained as the supplied image below. The
project will **not** recreate the panel layout in KiCad or another CAD package.

![Complete gantry panel schematic](<schematics/Gantry panel.drawio (7).png>)

The image covers:

-   240 VAC three-phase entry, CB0 and AC distribution;
-   X, Z, and theta breaker / contactor / line-filter / drive branches;
-   24 VDC and 5 VDC control supplies;
-   motors, feedback and limit-switch field connections;
-   WT32-ETH01 / WIZ850io control sub-panel and EIP links;
-   gripper relay and E-stop stations.

The PNG is the panel overview, not a netlist-bearing construction drawing.
Unresolved safety details are in the electrical-review register below.

---

## 3. Voltage domains

| Domain | Nominal | Ground |
|--------|---------|--------|
| AC mains | 200–240 V 3φ | PE |
| Field control | 24 VDC | FIELD_0V (star at TBIO DCOM / HCS01 0 V) |
| Logic | 5 V / 3.3 V (WT32 LDO) | LOGIC_GND (isolated from FIELD_0V except intentional barriers) |

---

## 4. Connection schedule (summary)

| From | To | Medium |
|------|----|--------|
| Utility | CB0 → PDB (the three phases go straight to CB0; no separate disconnect) | Feeder / #8 class |
| CB1/K1/LF1 | X drive L1/L2/L3 | Branch AC |
| CB2/K2/LF2 | Z drive | Branch AC |
| CB3/K3/LF3 | HCS01 | Branch AC (theta deferred) |
| CB4 | PS1 AC in | ≥2 A breaker recommended |
| PS1 +24 V | Drive CP, valves, E-stop coils, TBIO aux | #18 AWG class |
| Drive motor connectors | MPL / ERD | Factory 2090 / SCHUNK cables |
| W5500 RJ45 | X PORT1 → … → Z PORT2 | Cat5e+ EIP |
| WT32 LAN8720 | Cell switch (CellNet L2) | Separate from EIP |
| Limit NC | TBIO INPUT1–4 / X31.5–.6 | 22 AWG STP |
| MCP23S17 PA0 (DOUT0) | CR1 coil → SV1 | 18–22 AWG |
| WIZ850io RSTn | MCP23S17 PB7 | Active-low; external pull-up; not hardwired |
| Free ADC 12/32/33/39 | Isolator → 0..3.3 V | Field analog (optional) |

---

## 5. Logic-board schematic

A separate logic-board schematic will be added when available. It should cover
the WT32-ETH01, WIZ850io/W5500 SPI interface, power conditioning, MCP Field
DOUT0 gripper interface and board-level connectors. That schematic may use
KiCad; it does not replace or redraw the panel image.

---

## 6. Electrical review register (must close before construction)

| ID | Item | Status |
|----|------|--------|
| E1 | Dual-channel E-stop + safety relay / safety PLC architecture | **OPEN** — panel image does not establish a validated safety category |
| E2 | STO wiring for Kinetix / HCS01 vs contactor-only stop | **OPEN** — STO out of early scope; confirm for production |
| E3 | CB3 (theta) and CB4 (PSU) final ampacity / inrush | **OPEN** — possible undersize |
| E4 | SCCR / source wye-vs-delta for Type-E breakers | **OPEN** |
| E5 | Final TBIO / X31 terminal landings vs panel image and as-built harness | **OPEN** |
| E6 | PE / shield bonding rules for factory motor cables | **OPEN** |
| E7 | Qualified electrical sign-off | **OPEN** |

Until E1–E7 are closed, treat all schematics as **design intent for review**,
not shop drawings.

## See also

-   **Up:** [Knowledge transfer](KNOWLEDGE_TRANSFER.md).
-   **Also:** [Gantry firmware](/GANTRY_FIRMWARE.md), the software that drives
    this hardware.

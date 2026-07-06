# Endstop Limit Switch Wiring Plan

**Date:** 2026-07-03
**Reference:** [DRIVE_PROTOCOL_AND_ENDSTOP_REVIEW.md](DRIVE_PROTOCOL_AND_ENDSTOP_REVIEW.md)

Wiring endstop limit switches directly to each drive's digital inputs — rather than
through the MCP23S17 GPIO expander to the ESP32 — moves limit monitoring into the
drive firmware. The drive handles overtravel internally and reports Fault/Stopped
status through the EtherNet/IP assembly feedback path.

---

## 1. Kinetix 5100 (X and Z axes)

**Connector:** 50-pin I/O with 2198-TBIO terminal block
**Input type:** Sinking (NPN), 24V DC, 6 mA per input
**Aux power:** 24V DC to TBIO power terminals

### Pin assignments

| Axis | Direction | Switch Type | TBIO Pin | Signal | KNX5100C Assignment |
|------|-----------|-------------|----------|--------|---------------------|
| X | Forward limit | NC | 9 | INPUT1 | Forward Limit |
| X | Reverse limit | NC | 10 | INPUT2 | Reverse Limit |
| Z | Forward limit | NC | 34 | INPUT3 | Forward Limit |
| Z | Reverse limit | NC | 8 | INPUT4 | Reverse Limit |

Spare inputs: INPUT5-INPUT10 (pins 33, 32, 31, 30, 29, 38) available for future use.

### Wiring (sinking, normally-closed)

```
+24V DC ────[ NC Limit Switch ]──── TBIO INPUTx
 0V DC ──────────────────────────── TBIO DCOM (pin 11)
```

### KNX5100C configuration

1. Connect KNX5100C to the drive (USB or Ethernet).
2. Navigate to **Digital IO/Jog Control > Edit DIO Configuration**.
3. Assign INPUT1 as "Forward Limit" for X-axis drive.
4. Assign INPUT2 as "Reverse Limit" for X-axis drive.
5. Repeat for Z-axis drive with INPUT3 (Forward) and INPUT4 (Reverse).
6. The drive handles limit-triggered stops internally; the firmware reads Fault/Stopped
   status through the EtherNet/IP InputAssembly154.

---

## 2. HCS01 IndraDrive Cs (Theta axis)

**Connector:** X31 terminal block
**Input type:** 24V DC, 3 mA
**Max configurable digital inputs:** 8 (2 probe-capable: X31.3, X31.4)

### Pin assignments (Rexroth-recommended)

| Terminal | Signal | Function | Parameter |
|----------|--------|----------|-----------|
| X31.5 | E3 | Positive travel limit switch | P-0-0222 |
| X31.6 | E4 | Negative travel limit switch | P-0-0222 |

Note: The ERD 04-40-D-H-N rotary module has unlimited rotation and does not
use physical limit switches for travel range. X31.5/X31.6 are assigned if
cable-management sensors are wired. If no physical limits are present, the
firmware enforces software limits via `AXIS_THETA_HARD_LIMIT_MIN/MAX_DEG`.

### Wiring (normally-closed)

```
+24V DC ────[ NC Limit Switch (+) ]──── X31.5 (E3, positive travel limit)
+24V DC ────[ NC Limit Switch (-) ]──── X31.6 (E4, negative travel limit)
 0V DC ──────────────────────────────── X31 common reference
```

### IndraWorks DS configuration

1. Open IndraWorks DS and connect to the HCS01 drive.
2. Navigate to **Limit Values > Motion Limit Values**.
3. Set **P-0-0222** to assign X31.5/X31.6 as travel range limit switches.
4. Optionally set software limits via:
   - **S-0-0041** — Positive position limit value
   - **S-0-0043** — Negative position limit value
   - **P-0-0090** — Travel range limit values

---

## 3. Firmware impact

When endstops are wired to drive digital inputs, the firmware:

- **Does NOT** read limit switch state from MCP23S17 GPIOs (the old path).
- **Reads** Fault/Stopped/AtReference status from the EtherNet/IP assembly feedback
  (InputAssembly154 for Kinetix, Hcs01PositioningActual for HCS01).
- The drive stops motion internally on limit activation and reports the fault state.
- The EIP process image conveys this state through `GantryEipLinearAxis::update()` /
  `GantryEipRotaryAxis::update()`.

### Step/direction fallback

When operating in step/direction mode (`PulseMotor`, no EIP), limit switches
remain on MCP23S17 GPIOs as originally wired. The `GantryLimitSwitch` debounced
reading path is unchanged for this mode. Kconfig `EIP_ENDSTOP_SOURCE` selects the
active reporting path at compile time.

---

## 4. Migration checklist

- [ ] Physically disconnect X-axis limit switches from MCP23S17 pins 2/3.
- [ ] Physically disconnect Z-axis limit switches from MCP23S17 pins 10/11.
- [ ] Wire X-axis forward/reverse limit switches to Kinetix X drive TBIO INPUT1/INPUT2.
- [ ] Wire Z-axis forward/reverse limit switches to Kinetix Z drive TBIO INPUT3/INPUT4.
- [ ] Wire Theta limit switches (if present) to HCS01 X31.5/X31.6.
- [ ] Configure KNX5100C Digital IO assignments per Section 1.
- [ ] Configure IndraWorks DS limit switch assignments per Section 2.
- [ ] Set `CONFIG_EIP_ENDSTOP_SOURCE` to "Drive digital inputs (via EIP status)" in menuconfig.
- [ ] Validate: activate each limit switch manually and confirm the drive reports a fault
  (visible in Wireshark T->O frame and serial log).

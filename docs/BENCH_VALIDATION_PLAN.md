# Bench Validation Plan: EtherNet/IP Scanner vs Real Drives

**Date:** 2026-07-03
**Status:** Ready for bench execution
**References:**
- [EIP_MIGRATION.md](EIP_MIGRATION.md) — byte maps, protocol layering, process-data design
- [EIP_VALIDATION_CHECKLIST.md](EIP_VALIDATION_CHECKLIST.md) — step-by-step reference
- [ENDSTOP_WIRING_PLAN.md](ENDSTOP_WIRING_PLAN.md) — limit switch pin assignments
- [MOTION_IO_INTERFACE.md](MOTION_IO_INTERFACE.md) — opto-isolated interface design
- [DRIVE_PROTOCOL_AND_ENDSTOP_REVIEW.md](DRIVE_PROTOCOL_AND_ENDSTOP_REVIEW.md) — drive inventory and protocol compatibility

**Code under test:**
- `lib/EtherNetIP/` — originator stack (encapsulation, CIP, assemblies, scanner)
- `lib/Gantry/src/GantryEipLinearAxis.cpp` — Kinetix 5100 adapter (X, Z)
- `lib/Gantry/src/GantryEipRotaryAxis.cpp` — HCS01 adapter (theta)
- `idf/main/main.cpp` — Kconfig-gated axis wiring

**Host tests (pre-validated):**
All 7 host tests pass: `test_kinematics`, `test_trajectory`, `test_eip_encoding`,
`test_hcs01_assembly`, `test_eip_transport`, `test_eip_axis`, `test_w5500_spi`.

---

## Overview

This plan validates the ESP32 EtherNet/IP originator against all three servo
drives on a single isolated bench LAN. The goal is to confirm the full Class 1
cyclic I/O lifecycle against real hardware — from TCP Connect through ForwardOpen
through cyclic position commands through alarm handling — for each drive family.

### Validation objectives

| Objective | Success criteria |
|-----------|-----------------|
| TCP lifecycle | RegisterSession + ForwardOpen accepted by each drive without CIP errors |
| Cyclic I/O | Bidirectional O->T / T->O frames at configured RPI with no timeouts |
| Position control | Drive moves to commanded PUU/mm positions and reports AtReference |
| Alarm/fault handling | Fault bit is reported; clearAlarm() clears it |
| Endstop integration | Limit switch activation stops drive; fault visible in T->O assembly |
| Multi-axis coexistence | X, Z, and theta all validate independently (single-axis at a time) |

### Drive validation order

```
Phase A:  Kinetix 5100 X-axis    (2198-E1020-ERS, assemblies 104/154)
Phase B:  Kinetix 5100 Z-axis    (2198-E1004-ERS, assemblies 104/154)
Phase C:  HCS01 theta-axis       (HCS01.1E, assemblies 101/102) — DEFERRED
```

Start with X because:
- Fixed, well-documented assemblies (no drive-side process-data configuration needed)
- Lower risk: belt actuator with limited stroke, easy to observe
- Kinetix commissioning is simpler than HCS01 (KNX5100C vs IndraWorks)

Phase C is documented for future execution when 3-phase supply is available.

---

## Stage 0: Bench Setup

### 0.1 Hardware inventory

Gather and verify every item before starting. This avoids mid-session
scavenger hunts.

| # | Item | Check |
|---|------|-------|
| 1 | WT32-ETH01 module, flashed with ESP-IDF v6.0 firmware | [ ] |
| 1 | Kinetix 5100 2198-E1020-ERS (X-axis, dual-port EtherNet/IP) | [ ] |
| 1 | Kinetix 5100 2198-E1004-ERS (Z-axis, dual-port EtherNet/IP) | [ ] |
| 1 | HCS01.1E-W0005-A-03-B-ET-EC (theta-axis) — **DEFERRED: 3-phase supply unavailable** | [ ] |
| 3 | CAT5e patch cables (WT32→Kinetix X, Kinetix X→Kinetix Z, Kinetix Z→PC) | [ ] |
| 1 | 24V DC power supply (1606-XL or equivalent) | [ ] |
| 1 | 5V DC regulator (from 24V rail) | [ ] |
| 1 | Motion I/O interface board (opto-isolated, per MOTION_IO_INTERFACE.md) | [ ] |
| — | 2198-TBIO terminal blocks (one per Kinetix drive) | [ ] |
| — | Limit switches (NC, 24V, one pair per axis) | [ ] |
| — | Wire: 24 AWG signal, 18 AWG power, ferrules or stripped ends | [ ] |
| 1 | USB-serial adapter for WT32 (115200 8N1) | [ ] |
| 1 | Engineering PC with Ethernet port | [ ] |

No external Ethernet switch is used. The Kinetix 5100 drives each have
dual Ethernet ports with an embedded 2-port switch, supporting line
(daisy-chain) topology. The WT32-ETH01 is single-port and sits at the
head of the chain.

**HCS01 (theta-axis) is deferred** until 3-phase AC supply is available.
The HCS01 commissioning procedure (Stage 1.3) and Phase C validation
(Stage 5) are documented below for future reference when the drive can
be powered.

### 0.2 Software on engineering PC

| Tool | Version | Purpose | Installed? |
|------|---------|---------|------------|
| KNX5100C | Latest from Rockwell PCDC | Kinetix 5100 commissioning | [ ] |
| IndraWorks DS (or Engineering) | Latest from Bosch Rexroth | HCS01 commissioning | [ ] |
| Wireshark | 4.x | Packet capture (see note in 0.3) | [ ] |
| PuTTY or idf.py monitor | — | ESP32 serial log (115200 8N1) | [ ] |
| ESP-IDF v6.0 toolchain | — | Build and flash firmware | [ ] |

### 0.3 Network topology: daisy-chain (line)

Three devices in a single line. The Kinetix drives' embedded 2-port
switches forward traffic between PORT1 and PORT2. The WT32 and PC are
the two endpoints.

```
PORT1  PORT2          PORT1  PORT2
┌──────┐    ┌──────────┐    ┌──────────┐
│ WT32 ├────┤ Kinetix X├────┤ Kinetix Z├────┤  PC   │
│ .1.10│    │  .1.20   │    │  .1.21   │    │ .1.1  │
└──────┘    └──────────┘    └──────────┘    └───────┘
  (single     (dual-port     (dual-port
   port)       embedded       embedded
               switch)        switch)

         HCS01 (.1.30) — DEFERRED (no 3-phase supply)
         Will be inserted between Kinetix Z and PC when available:
               ... Kinetix Z → HCS01 → PC
```

| Device | IP Address | Subnet Mask | Gateway | Status |
|--------|-----------|-------------|---------|--------|
| WT32-ETH01 | 192.168.1.10 | 255.255.255.0 | 192.168.1.1 | Active |
| Kinetix 5100 X | 192.168.1.20 | 255.255.255.0 | 192.168.1.1 | Active |
| Kinetix 5100 Z | 192.168.1.21 | 255.255.255.0 | 192.168.1.1 | Active |
| HCS01 theta | 192.168.1.30 | 255.255.255.0 | 192.168.1.1 | **Deferred** |
| Engineering PC | 192.168.1.1 | 255.255.255.0 | — | Active |

**Cable routing (3 hops):**

| Hop | From | To |
|-----|------|----|
| 1 | WT32-ETH01 RJ45 | Kinetix X PORT1 |
| 2 | Kinetix X PORT2 | Kinetix Z PORT1 |
| 3 | Kinetix Z PORT2 | Engineering PC |

**Important topology notes:**

- Both Kinetix drives must be powered for the chain to carry traffic.
  A powered-off Z drive breaks connectivity between X and the PC.
- The Kinetix 5100 embedded switch operates at wire speed with
  cut-through forwarding (~1-2 us per hop). Total forwarding latency
  across both drive hops is negligible (< 5 us).
- The drives must be configured for **linear/star topology** (not Device
  Level Ring). DLR requires a ring supervisor and is not supported by the
  current firmware. No DLR configuration is needed.
- The PC can ping and communicate with any device because each drive's
  embedded switch forwards frames addressed to other nodes. However, the
  PC only receives frames destined for its own MAC address (unicast) or
  broadcast frames. **Wireshark on the PC will NOT see unicast traffic
  between the WT32 and drives.** See 0.5 for capture strategy.

**HCS01 theta-axis (192.168.1.30) is deferred** until 3-phase AC supply
is available. When powered, insert it between Kinetix Z PORT2 and the PC.
The HCS01 commissioning procedure and Phase C validation are documented in
Stages 1.3 and 5 for future reference.

### 0.4 Pre-power checks

Before applying 24V:

1. **Visual inspection:** no stray wire strands, no shorts between adjacent
   terminals on TBIO, X31, or the opto interface board.
2. **Continuity check:** FIELD_0V star — verify all DCOM and 0V returns
   are tied to one point. LOGIC_GND and FIELD_0V must be isolated except
   through optocouplers.
3. **Power sequence:** apply 24V first, then 5V regulator powers the
   74AHCT244 and opto LEDs. WT32 gets 3.3V via USB or its own regulator.
4. **STO:** Safe Torque Off must be wired correctly before enabling any
   drive. The Kinetix 5100 STO inputs (TBIO pins 16-19) must see 24V
   for the drive to produce torque. HCS01 STO is on X32.

### 0.5 Baseline network test (no EIP firmware)

Before flashing EIP firmware, confirm the daisy-chain works end-to-end.

1. **Cable the chain:** WT32 → Kinetix X PORT1 → Kinetix X PORT2 →
   Kinetix Z PORT1 → Kinetix Z PORT2 → PC.
   Use only the 3 cables listed in 0.1.

2. **Power up both Kinetix drives.** Both must be powered because
   Z is the last hop before the PC. Power-up order: apply 24V to both
   drives, wait 10 seconds for boot and PHY negotiation. Verify each
   drive's status LED is solid green (no fault).

3. Power up the WT32 with the **existing step/direction firmware** (no
   EIP). The WT32 gets power via USB.

4. From the PC, ping each IP:
   ```
   ping 192.168.1.20   # Kinetix X (1 hop)
   ping 192.168.1.21   # Kinetix Z (2 hops)
   ping 192.168.1.10   # WT32     (2 hops from PC)
   ```
   All three must respond. If any ping fails, the chain is broken at or
   before that device. Check:
   - Is the non-responding device powered on?
   - Are both ends of the cable to/from that device securely plugged in?
   - Are the link LEDs on both ports of that device lit?

5. Check the WT32 serial log for `MqttBridge` DHCP or static IP output.
   Confirm the WT32 is on 192.168.1.10.

6. **Do not proceed** if any device is unreachable. Chasing network
   issues while also debugging EIP wastes bench time.

### 0.6 Wireshark capture strategy in a line topology

In a daisy-chain, the PC is an endpoint and does not see unicast traffic
between other nodes. This limits passive Wireshark capture.

**What the PC CAN capture:**
- Broadcast traffic (ARP, ListIdentity broadcasts)
- Traffic to/from the PC itself (ping responses, KNX5100C/IndraWorks
  communication)
- During commissioning: all traffic between the PC and the drive being
  configured

**What the PC CANNOT capture:**
- Unicast TCP/UDP between the WT32 and drives (RegisterSession,
  ForwardOpen, cyclic I/O). These frames are forwarded by the embedded
  switches but the PC's NIC filters them.

**Validation strategy (no external tap):**

| Evidence source | What it proves | Priority |
|-----------------|---------------|----------|
| **ESP32 serial log** | ForwardOpen success/failure, connection IDs, RPI, error codes, axis state | Primary |
| **Drive front-panel display** | Fault codes, IP address, link status, operating mode | Primary |
| **KNX5100C / IndraWorks** | Drive parameter verification, DIO configuration, process-data lists | Commissioning |
| **Physical observation** | Motor movement, limit switch activation, actuator travel distance | Motion validation |
| **Wireshark (PC endpoint)** | Broadcast traffic, commissioning traffic, ARP, ICMP | Supplementary |

**If Wireshark capture of WT32↔drive traffic is required for debugging:**

Add a small unmanaged switch **only** at the WT32 end of the chain,
and connect the PC there temporarily:

```
        ┌──────────┐
WT32 ──→│  Switch  ├──→ Kinetix X ──→ Kinetix Z ──→ HCS01
        │ (temp)   │
   PC ──┤          │
        └──────────┘
```

In this hybrid setup, the switch replicates all traffic to the PC port,
enabling full Wireshark capture of the WT32↔drive exchange. Remove the
switch after debugging — the production topology is a pure line.

**For the primary validation path described in this plan, serial logs
and drive diagnostics are sufficient. Wireshark captures are optional
and noted as supplementary.**

---

## Stage 1: Drive Commissioning

Each drive must be commissioned for EtherNet/IP I/O mode before the ESP32
scanner can connect. Commissioning is done once per drive and persists
across power cycles.

### 1.1 Kinetix 5100 — X-axis (192.168.1.20)

**Connect KNX5100C to the drive** via USB (preferred) or Ethernet.

#### 1.1.1 Set drive IP

1. In KNX5100C, navigate to the network/communication settings.
2. Set IP address: `192.168.1.20`, subnet mask: `255.255.255.0`, gateway: `192.168.1.1`.
3. If using rotary switches on the drive, set them to a value that maps to
   the target IP (consult `2198-UM004` for the switch table).
4. Cycle drive power after IP change.

#### 1.1.2 Set operating mode

1. Navigate to the operating mode / control mode settings.
2. Set control mode to **EtherNet/IP I/O** (not pulse-train / PT mode).
3. The drive must be configured to accept motion commands from the
   EtherNet/IP assembly, not from the TBIO pulse inputs.

#### 1.1.3 Configure limit switch digital inputs

Per [ENDSTOP_WIRING_PLAN.md](ENDSTOP_WIRING_PLAN.md):

1. Navigate to **Digital IO/Jog Control > Edit DIO Configuration**.
2. Assign **INPUT1** as **Forward Limit**.
3. Assign **INPUT2** as **Reverse Limit**.
4. Save to drive non-volatile memory.

#### 1.1.4 Extract EDS file

1. In KNX5100C, export or locate the drive's EDS file.
2. Save it as `docs/captures/eds_kinetix5100.eds`.
3. Open the EDS in a text editor and record the following values
   (these are used to configure the ESP32 scanner in Stage 2):

| EDS Field | Current Provisional Value | Confirmed Value |
|-----------|--------------------------|-----------------|
| Config assembly instance | 1 | |
| Output assembly 104 size (bytes) | 40 | |
| Input assembly 154 size (bytes) | 52 | |
| Run/Idle header required? | Yes (`true`) | |
| Connection point (O->T) | — | |
| Connection point (T->O) | — | |

**Gate:** If any confirmed value differs from the provisional value,
update the corresponding Kconfig knob in `idf.py menuconfig` before
building the EIP firmware.

#### 1.1.5 Verify EIP is alive (optional)

If you want to confirm the drive's EIP stack is running before the ESP32
scanner connects:

```
# From the engineering PC, send a ListIdentity broadcast (Python):
python -c "
import socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
# EIP ListIdentity: command 0x0063, length 0, session 0, status 0,
# context 0, options 0
req = bytes([0x63,0x00, 0x00,0x00, 0x00,0x00,0x00,0x00,
             0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,
             0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00])
s.sendto(req, ('255.255.255.255', 44818))
s.settimeout(2)
data, addr = s.recvfrom(1024)
print(f'Response from {addr}: {data.hex()}')
"
```

A Kinetix 5100 should respond with its identity object (vendor ID 1 for
Rockwell, device type 0x002B for AC drive). This confirms EIP is active
without needing ForwardOpen.

### 1.2 Kinetix 5100 — Z-axis (192.168.1.21)

Repeat all steps in 1.1 for the Z-axis drive, substituting:

- IP address: `192.168.1.21`
- Limit switch assignments: **INPUT3** = Forward Limit, **INPUT4** = Reverse Limit

The Z-axis has a different `puu_per_mm` scaling (4000.0 vs 1000.0 for
X-axis). This is set in Kconfig, not in the drive.

### 1.3 HCS01 — Theta-axis (192.168.1.30)

**Connect IndraWorks DS to the drive** via the engineering Ethernet port.

#### 1.3.1 Set IP addresses

The HCS01 has two IP addresses:

1. **Engineering IP** (`S-0-1020`): used by IndraWorks for commissioning.
   Can be on the same subnet or a separate engineering network.
2. **EIP IP** (`P-0-4089`): used by the ESP32 scanner for Class 1 I/O.
   - `P-0-4089.0.13` = 192 (first octet)
   - `P-0-4089.0.14` = 168 (second octet), netmask 255.255.255.0
   - `P-0-4089.0.15` = 1 (third octet), gateway 192.168.1.1
   - Fourth octet is set elsewhere (consult IndraWorks parameter tree)

If the drive uses a single IP for both, set it to 192.168.1.30.

#### 1.3.2 Set freely configurable profile

1. Set **`P-0-4084 = 0xFFFE`** (freely configurable profile).
   This tells the drive to build assembly instances 101/102 from
   the process-data lists below, rather than using a fixed profile.

#### 1.3.3 Configure process data lists

Open the cyclic data configuration dialog in IndraWorks and configure:

**P-0-4081 — Command list (O->T, 10 bytes = 5 words):**

| Word | Parameter | Type | Value |
|------|-----------|------|-------|
| 1 | `P-0-4077` | u16 | Field bus control word |
| 2-3 | `S-0-0282` | i32 | Positioning command value |
| 4-5 | `S-0-0259` | i32 | Positioning velocity |

**P-0-4080 — Actual list (T->O, 14 bytes = 7 words):**

| Word | Parameter | Type | Value |
|------|-----------|------|-------|
| 1 | `P-0-4078` | u16 | Field bus status word |
| 2-3 | `S-0-0051` | i32 | Position feedback value 1 |
| 4-5 | `S-0-0040` | i32 | Velocity feedback value |
| 6-7 | `S-0-0390` | u32 | Diagnostic message number |

#### 1.3.4 Set update rate

1. Set **`P-0-4076`** to the desired API (process-data update clock).
   Range: 2–65 ms in 1 ms steps. Start with **4 ms** (matches Kconfig
   default `EIP_THETA_RPI_US = 4000`).

#### 1.3.5 Configure limit switch digital inputs

Per [ENDSTOP_WIRING_PLAN.md](ENDSTOP_WIRING_PLAN.md):

1. Navigate to **Limit Values > Motion Limit Values**.
2. Set **P-0-0222** to assign **X31.5 (E3)** and **X31.6 (E4)** as
   travel range limit switches.
3. Optionally set software limits:
   - `S-0-0041` — Positive position limit value
   - `S-0-0043` — Negative position limit value

Note: The ERD rotary module has unlimited rotation. X31.5/X31.6 are
assigned only if cable-management limit sensors are wired. If no
physical limits exist, the firmware enforces software limits via
`AXIS_THETA_HARD_LIMIT_MIN/MAX_DEG` in `axis_drivetrain_params.h`.

#### 1.3.6 Determine PUU-to-degree scaling

The HCS01 does not have a fixed PUU-to-degree ratio. Compute it:

1. In IndraWorks, find the encoder resolution: `S-0-0116` (encoder
   counts per revolution, multiplied by any gear ratio).
2. Compute:
   ```
   PUU_per_deg = (encoder_counts_per_rev × gear_ratio) / 360.0
   ```
3. Enter this value as `EIP_AXIS_THETA_PUU_PER_DEG` in Kconfig.

Example: if the encoder is 20-bit (1,048,576 counts/rev) with a 50:1
gear reducer:
```
PUU_per_deg = (1,048,576 × 50) / 360 ≈ 145,635.6
```

#### 1.3.7 Extract EDS file

1. In IndraWorks, export the drive's EDS file.
2. Save it as `docs/captures/eds_hcs01.eds`.
3. Record connection points and assembly sizes from the EDS.

#### 1.3.8 Data format verification (critical)

The Functions manual lists `(H)` before `(L)` in parameter tables
(suggesting big-endian), but section 4.8 states "Intel format"
(little-endian). This must be resolved at the bench.

| Data source | Expected endianness | Actual (bench) |
|-------------|-------------------|----------------|
| `P-0-4078` (status word u16) | Little-endian | |
| `S-0-0282` (position i32) | Little-endian | |
| `S-0-0051` (feedback i32) | Little-endian | |

To verify: after ForwardOpen connects, inspect the T->O frame in
Wireshark. If status word `0x0003` (bits 1/0 = 10 for "operating mode")
appears as `03 00`, it is little-endian (our codec is correct). If it
appears as `00 03`, the codec needs byte-swapping in `Hcs01Assembly.cpp`.

---

## Stage 2: ESP32 Firmware Configuration

Build configurations for each validation phase. All changes are made via
`idf.py menuconfig` from the `idf/` directory.

### 2.1 Common Kconfig settings (all phases)

These settings are correct from the step/direction build. Do not change
unless troubleshooting:

```
Component config → LWIP → [*] Enable IPV4
Component config → Ethernet → [*] Support ESP32 internal EMAC controller
```

The WT32-ETH01 uses RMII Ethernet via the LAN8720 PHY. These settings
are already active and working for MQTT.

### 2.2 Phase A firmware: Kinetix X-axis EIP

```
EtherNet/IP originator
  [*] Enable EtherNet/IP scanner task
      EtherNet/IP axis selection → (X axis over EIP (Kinetix 5100))
      (192.168.1.20) Target drive IP address
      (1000.0) X-axis PUU/mm scaling
      (5000) X/Z axis RPI (microseconds)
      Endstop limit switch reporting path → (Drive digital inputs (via EIP assembly status))
      (1) Config assembly instance for ForwardOpen
      [*] Include 32-bit Run/Idle header in cyclic O->T frames
      (0) Originator vendor ID for ForwardOpen
```

Update the provisional values (config assembly instance, Run/Idle header,
vendor ID) from the EDS data recorded in Stage 1.1.4.

Build: `idf.py -C idf build`
Flash: `idf.py -C idf -p COM<N> flash monitor`

### 2.3 Phase B firmware: Kinetix Z-axis EIP

After Phase A passes, change only:

```
EtherNet/IP axis selection → (Z axis over EIP (Kinetix 5100))
(192.168.1.21) Target drive IP address
(4000.0) Z-axis PUU/mm scaling
```

All other settings (RPI, assembly config, Run/Idle) stay the same
because both Kinetix drives are identical models with the same
assembly instances.

### 2.4 Phase C firmware: HCS01 theta EIP

After both Kinetix axes pass:

```
EtherNet/IP axis selection → (Theta axis over EIP (HCS01))
(192.168.1.30) Target drive IP address
(<computed>) Theta-axis PUU/deg scaling
(4000) Theta axis RPI (microseconds)
```

The Kconfig knobs for config assembly instance, Run/Idle header, and
vendor ID may need different values for the HCS01. Consult the HCS01
EDS (recorded in Stage 1.3.7).

---

## Stage 3: Validation Phase A — Kinetix X-axis TCP + EIP Lifecycle

**Goal:** Confirm that the ESP32 scanner opens a Class 1 connection to
the Kinetix 5100 X-axis drive and exchanges cyclic I/O.

**Pass criteria:**
- TCP connect to 192.168.1.20:44818 succeeds
- RegisterSession returns a valid session handle
- ForwardOpen returns general status 0x00 (success)
- Cyclic UDP O->T and T->O frames flow at the configured RPI
- Serial log confirms "ForwardOpen ok" with connection IDs
- Wireshark capture matches expected packet sequence

### 3.1 Power-on sequence (daisy-chain)

**Critical order:** Both Kinetix drives between the WT32 and PC must be
powered. A powered-off Z drive breaks the chain.

1. Apply 24V to BOTH Kinetix drives (X and Z). Even if only validating
   X today, Z must be powered for chain continuity to the PC.
2. Wait 10 seconds for drive boot, internal self-test, and PHY
   auto-negotiation on all ports.
3. Verify both drives' status LEDs are solid green (no fault).
4. Verify link LEDs on all Ethernet ports along the chain are lit
   (WT32→X PORT1, X PORT2→Z PORT1, Z PORT2→PC).
5. Connect and power the WT32-ETH01 via USB.
6. Open serial monitor: `idf.py -p COM<N> monitor --timestamp`

### 3.2 Expected serial log

The serial log is the primary validation instrument in a line topology.
Every protocol event is logged by the EipScanner task.

```
I (1234) GantryApp: WT32-ETH01 Gantry Controller (PulseMotor)
I (2345) GantryApp: MCP23S17 initialized successfully
I (3456) MqttBridge: Ethernet link up, IP: 192.168.1.10
I (4567) EipScanner: EipScanner task started (target 192.168.1.20)
I (5678) EipScanner: ForwardOpen ok O->T=0x10000001 T->O=0xCOFFEE01 API=5000 us
I (6789) GantryApp: X axis running over EIP (Kinetix 5100, 1000.0 PUU/mm), target 192.168.1.20
```

The T->O connection ID (`0xCOFFEE01` above) is populated from the
ForwardOpen reply. Record the actual value.

If the scanner logs `Connect to 192.168.1.20 failed`, verify:
- Is Kinetix X powered and booted?
- Are BOTH Kinetix drives powered (chain continuity: X→Z→PC)?
- Are link LEDs lit on Kinetix X PORT1 (WT32 side) and PORT2 (Z side)?
- Can the PC ping 192.168.1.20?
- If ping works from PC but WT32 cannot connect, the WT32's own IP
  may be wrong. Check the MqttBridge DHCP/static IP log line.

### 3.3 Confirming the full EIP lifecycle via serial log

The EipScanner logs each lifecycle stage. Expected sequence:

| Log keyword | Phase |
|-------------|-------|
| `EipScanner task started` | Scanner task running, waiting for link-up |
| (MqttBridge link-up message) | Ethernet netif is up, scanner proceeds |
| (no error after ~1s) | TCP connect to target:44818 succeeded |
| (no error after ~2s) | RegisterSession returned a session handle |
| `ForwardOpen ok O->T=... T->O=... API=...` | Class 1 connection open |
| (no `T->O timeout` for 30+ seconds) | Cyclic I/O flowing at granted API |

If ForwardOpen is **rejected**, the scanner logs a warning with the CIP
error. Common causes:

| Serial log message | Likely cause | Action |
|-------------------|-------------|--------|
| `Connect to <ip> failed` | TCP SYN not answered | Ping drive; check chain power and cables |
| ForwardOpen logged with error context | CIP status != 0x00 | See extended status codes in 6.3 |
| `T->O timeout or IO error` after connect | Cyclic UDP frames lost | Check cables; verify RPI; check drive fault state |
| No log after `task started` — stuck | Ethernet link never came up | Check RMII PHY; verify cable WT32→Kinetix X PORT1 |

**Extended status codes logged on ForwardOpen rejection:**

| Extended status | Meaning | Action |
|----------------|---------|--------|
| 0x0107 | Connection point not found | Wrong config-assembly instance. Update `EIP_CONFIG_ASSEMBLY_INSTANCE` in Kconfig from EDS. |
| 0x0111 | Invalid connection size | Assembly size mismatch. Verify 40/52 bytes against EDS. |
| 0x0118 | RPI not supported | Increase `EIP_X_RPI_US` to 10000 or 20000. |
| 0x011A | Vendor ID mismatch | Set `EIP_ORIGINATOR_VENDOR_ID` in Kconfig (try 0x0001). |

### 3.4 Verifying cyclic I/O health

**Pass criteria (observe serial log for 30+ seconds):**

- No `T->O timeout` messages.
- No `ForwardOpen` retry messages (indicating disconnect).
- The scanner continues running without restarting.

**Optional: enable verbose scanner logging.** In `EipScannerTask.cpp`,
uncomment or add temporary `ESP_LOGI` lines in the cyclic loop to log
T->O frame sizes and sequence counts.

### 3.5 Inspect T->O data via debug logging (optional)

Add temporary debug logging to confirm the T->O frame bytes match the
[InputAssembly154 byte map](EIP_MIGRATION.md#32-input-assembly---instance-154-t-o-52-bytes).
Key fields to log:

| Field | Offset | Expected (healthy, no motion) |
|-------|--------|------------------------------|
| Fault | Byte 8, bit 1 | 0 |
| Ready | Byte 9, bit 3 | 1 |
| Stopped | Byte 9, bit 6 | 1 |
| ActualPosition | Bytes 24-27 | Current motor position (changes if shaft turned) |

If Fault is 1, stop and diagnose via the drive's front-panel display.

### 3.6 Verify endstop fault reporting

Confirm that limit switches wired to the drive's TBIO digital inputs
report faults through the EIP assembly path.

1. With the cyclic exchange running, manually activate one limit switch
   (press the switch plunger).
2. Observe the Kinetix X front-panel display: it should show a fault
   code (overtravel or limit-switch fault).
3. Observe the serial log: the alarm path should report the alarm.
4. Release the limit switch.
5. From the serial console, issue `clear_alarm` to reset the fault.
6. Confirm the drive display clears and the serial log shows normal status.

If the drive does not fault:
- Verify the limit switch is wired to the correct TBIO pin (Stage 1.1.3).
- Verify KNX5100C DIO Configuration assigns that input as Forward/Reverse Limit.
- Check switch continuity with a multimeter at the TBIO terminal.

---

## Stage 4: Validation Phase B — Kinetix Motion

**Goal:** Command the Kinetix 5100 to move via the EIP assembly and
verify actual physical motion.

**Safety:** Ensure the motor/actuator has free travel in both directions.
Remove obstructions. Start with very small moves (1-5 mm). Keep E-stop
within reach.

**Pass criteria:**
- ServoOn command transitions drive to enabled state (status LED change)
- Position commands produce physical motion in the correct direction
- ActualPosition tracks the commanded position (visible via serial debug)
- AtReference bit asserts when motion completes (serial log or debug)
- PUU-to-mm scaling produces accurate physical distances
- Alarm bits work: trigger fault, observe alarm, clearAlarm() clears it

### 4.1 Enable the drive

From the serial console (or MQTT), issue:

```
enable
```

Expected observations:
- The drive's status LED changes to indicate enabled state (typically
  blinking green or solid green depending on drive configuration).
- Serial log shows enable confirmation from `GantryEipLinearAxis`.

### 4.2 Small position move

From the serial console, command a small move (e.g., 5 mm):

```
move 5 0 0
```

This commands X=5mm, Z=0, Theta=0. Only X should physically move
(Z and theta use PulseMotor defaults).

Expected observations:
- The X-axis actuator physically moves approximately 5 mm.
- The drive's front-panel display shows the changing position.
- If verbose debug logging is enabled, the serial log shows:
  - `PositionReference` changing to the commanded PUU.
  - `ActualPosition` tracking toward the target.
  - `AtReference` asserting when motion completes.

### 4.3 PUU-to-mm scaling calibration

1. Command a known mm distance (e.g., 10 mm):
   ```
   move 10 0 0
   ```

2. Measure the actual physical distance traveled using calipers or ruler.

3. Compute the scaling correction:
   ```
   new_puu_per_mm = current_puu_per_mm * (commanded_mm / actual_mm)
   ```

   Example: commanded 10 mm, actual travel 9.7 mm:
   ```
   new_puu_per_mm = 1000.0 * (10.0 / 9.7) = 1030.9
   ```

4. Update `EIP_AXIS_X_PUU_PER_MM` in `idf.py menuconfig`, rebuild,
   reflash, and repeat.

5. Iterate until actual travel is within tolerance (typically +/- 0.1 mm
   for this belt actuator).

### 4.4 Direction check

1. Command a positive move: `move 10 0 0`
2. Observe the physical direction of travel.
3. If motion is reversed, invert the drive's motor direction in
   KNX5100C (parameter ID062, `MotorDirection`). Do NOT compensate in
   firmware PUU scaling — the drive handles direction internally.

### 4.5 Alarm handling

1. Trigger a drive alarm by briefly disconnecting the motor encoder
   cable (if accessible) or by activating a limit switch and holding it.
2. The drive's front-panel display shows a fault code.
3. Serial log reports the alarm via the `GantryEipLinearAxis` alarm path.
4. Resolve the fault (reconnect encoder, release limit switch).
5. From the console: `clear_alarm`
6. Confirm the drive display clears and the serial log shows normal status.

### 4.6 Repeat for Z-axis

After X-axis passes all tests:

1. Power down the WT32.
2. Change Kconfig per Section 2.3 (Z-axis, 192.168.1.21, 4000 PUU/mm).
3. Rebuild and reflash.
4. Repeat Stages 3 and 4 for the Z-axis drive.
5. For the ballscrew actuator, use smaller test moves (2-5 mm) and
   verify the PUU/mm scaling (lead = 20 mm/rev for SCHUNK Beta 80-SRS).

### 4.7 Save captures

Save serial logs and optional Wireshark captures for each axis:
- `docs/captures/serial_phaseB_x.log`
- `docs/captures/serial_phaseB_z.log`
- `docs/captures/phaseB_x_position_move.pcapng` (optional — requires switch tap per 0.6)
- `docs/captures/phaseB_z_position_move.pcapng` (optional)

---

## Stage 5: Validation Phase C — HCS01 Theta (DEFERRED)

> **Deferred — 3-phase AC supply unavailable.**
> The HCS01 IndraDrive Cs requires 3-phase AC mains input. Until the
> 3-phase supply is available on the bench, the HCS01 cannot be powered
> and this validation phase cannot execute.
>
> The commissioning procedure below (Stage 1.3) and this validation
> sequence are retained for future reference. When 3-phase power is
> restored, insert the HCS01 between Kinetix Z PORT2 and the PC:
> `WT32 → Kinetix X → Kinetix Z → HCS01 → PC`.
> Assign 192.168.1.30 and follow the steps below.

**Goal:** Confirm the ESP32 scanner opens a Class 1 connection to the
HCS01 with assemblies 101/102 and commands the rotary axis.

**Prerequisites:** All HCS01 commissioning steps from Stage 1.3 must be
complete. The drive must be in freely configurable profile with
P-0-4081/P-0-4080 lists matching the expected 10/14-byte layout.

**Pass criteria:**
- ForwardOpen with assemblies 101/102 is accepted
- Status word P-0-4078 reports correct ready state
- Drive ON (bit 15) + Drive Start (bit 13) transitions to AF (in operation)
- Positioning command moves the rotary module
- Status word bit 10 toggles (command accepted), bit 4 asserts (in position)
- PUU-to-degree scaling produces accurate angular moves

### 5.1 Firmware flash

1. Power down the WT32.
2. Change Kconfig per Section 2.4 (theta, 192.168.1.30, computed PUU/deg).
3. Rebuild and reflash.

### 5.2 TCP lifecycle

Same sequence as Stage 3. Expected serial log:
```
I (xxxx) EipScanner: ForwardOpen ok O->T=0x10000001 T->O=0x... API=4000 us
I (xxxx) GantryApp: Theta axis running over EIP (HCS01, <n> PUU/deg), target 192.168.1.30
```

If ForwardOpen is rejected:
- Check that the HCS01 EDS connection points match the configured
  assembly instances (101/102).
- Verify `ot_assembly_size = 10` and `to_assembly_size = 14`.
- Verify `config_assembly_instance` matches the HCS01 EDS.

### 5.3 Status word check

With the connection open, verify the drive state via the serial log
and drive display. Enable verbose logging temporarily if needed.

**Expected initial state (no enable):**

| Bits | Expected | Meaning |
|------|----------|---------|
| 15/14 | 01 | Ready for power on (bb) |
| 1/0 | 00 | Parameter mode |

### 5.4 Enable the drive

From the serial console:
```
enable
```

This sends the HCS01 enable sequence: bit 15 (`Drive ON`) = 1, bit 13
(`Drive Start`) = 1. Bit 14 auto-sets when field-bus comm is active.

**Expected status word change:**

| Bits | Expected | Meaning |
|------|----------|---------|
| 15/14 | 11 | In operation (AF) |
| 1/0 | 10 | Operating mode |

If bits 15/14 never reach AF:
- Check hardware enable wiring (X31/X47 per MOTION_IO_INTERFACE.md).
- Verify the STO circuit is closed.
- Check that `P-0-4077` bit 14 auto-sets when field-bus comm is active.

### 5.5 Positioning command

1. Command a small angular move from the serial console:
   ```
   move 0 0 90
   ```
   (X=0, Z=0, theta=90 degrees)

2. Observe the rotary module physically rotate approximately 90 degrees.

3. Check the HCS01 front-panel display for position feedback.

4. When motion completes, the drive should indicate "in position" on
   its display, and the serial log (if verbose) should show the
   `command_value_reached` status bit = 1.

### 5.6 PUU-to-degree scaling check

1. Command a known angular move (e.g., 90 degrees).
2. Verify the actual rotation visually or with a protractor.
3. If inaccurate, adjust `EIP_AXIS_THETA_PUU_PER_DEG`:
   ```
   new_puu_per_deg = current_puu_per_deg * (commanded_deg / actual_deg)
   ```
4. Rebuild, reflash, and repeat until within tolerance.

### 5.7 Data format confirmation (P-0-4074)

Enable verbose logging to capture the raw bytes of `P-0-4078` (status
word, first 2 bytes of T->O frame):

1. If status word `0x0003` (bits 1/0 = 10, "operating mode") appears as
   `03 00` in the buffer → **little-endian, codec is correct.**
2. If it appears as `00 03` → **big-endian word order.**
   Update `Hcs01Assembly.cpp` to byte-swap each 16-bit word on
   deserialize, and re-test.

Record the confirmed byte order:
`docs/captures/phaseC_hcs01_byte_order.txt`

### 5.8 Save captures

- `docs/captures/serial_phaseC.log`
- `docs/captures/phaseC_hcs01_byte_order.txt`
- `docs/captures/phaseC_hcs01_lifecycle.pcapng` (optional — requires switch tap per 0.6)
- `docs/captures/phaseC_hcs01_positioning.pcapng` (optional)

---

## Stage 6: Troubleshooting

### 6.1 Chain continuity (line topology only)

The most common failure mode: a drive in the middle is off, breaking
connectivity to everything downstream.

**Symptom:** PC can ping Kinetix X (.1.20) but NOT Kinetix Z (.1.21).

| Cause | Fix |
|-------|-----|
| Kinetix Z is powered off | Power on Z. The chain is X→Z→PC; Z must be on for traffic to reach the PC. |
| Cable X PORT2 → Z PORT1 is loose | Reseat both ends. Check link LEDs on both ports. |

**Symptom:** PC cannot ping either drive.

| Cause | Fix |
|-------|-----|
| Kinetix X is powered off | Power on X. This is the first hop in the chain. |
| WT32→X PORT1 cable is loose | Check link LED on X PORT1. Reseat cable at both ends. |
| PC→Z PORT2 cable is loose | Check link LED on Z PORT2. Reseat at both ends. |

**Symptom:** PC can ping all drives but WT32 serial log shows
`Connect to <ip> failed`.

| Cause | Fix |
|-------|-----|
| WT32→X PORT1 cable is loose | Check link LED on X PORT1. Reseat cable at both ends. |
| WT32 IP is on wrong subnet | Check MqttBridge log line for WT32 IP. Must be 192.168.1.0/24. |
| X PORT1 PHY not negotiating | Try a different cable. Try swapping PORT1 and PORT2 on X. |

### 6.2 TCP connect fails

| Symptom | Check |
|---------|-------|
| `Connect to <ip> failed` | Ping the drive from PC. If ping works but WT32 cannot connect, check WT32 IP and netmask. If ping fails from PC, the chain is broken — check all intermediate drives are powered. |
| TCP RST after SYN | Drive not listening on port 44818. Confirm EIP is enabled (KNX5100C mode or IndraWorks EIP config). |
| Connection hangs then fails | ARP failure. All devices must be on the same subnet 192.168.1.0/24. |

### 6.3 ForwardOpen rejected

The scanner logs the CIP error. Extended status codes:

| Extended | Meaning | Action |
|----------|---------|--------|
| 0x0107 | Connection point not found | Wrong config-assembly instance. Update `EIP_CONFIG_ASSEMBLY_INSTANCE` from EDS. |
| 0x0111 | Invalid connection size | Assembly size mismatch. Verify 40/52 (Kinetix) or 10/14 (HCS01) against EDS. |
| 0x0118 | RPI not supported | Increase RPI. Try 10000 us, then 20000 us. |
| 0x0108 | Connection type not supported | Drive may require Exclusive Owner. Check `transport_class_trigger` in `EipScanner.cpp`. |
| 0x011A | Vendor ID mismatch | Set `EIP_ORIGINATOR_VENDOR_ID` in Kconfig. For Rockwell, try 0x0001. |

### 6.4 ForwardOpen succeeds but no cyclic I/O

| Symptom | Check |
|---------|-------|
| Serial log shows `T->O timeout` repeatedly | UDP frames not arriving. Verify the chain is intact. Check that the drive's T->O connection ID matches the ForwardOpen reply (logged by scanner). Check `EIP_INCLUDE_RUN_IDLE_HEADER` matches EDS. |
| Scanner disconnects and retries | Drive may have closed the connection. Check drive fault display. Check RPI is within drive limits. |

### 6.5 Cyclic exchange works but axis doesn't move

| Symptom | Check |
|---------|-------|
| ServoOn sent, motor doesn't energize | Drive in PT mode instead of EIP I/O mode. Change in KNX5100C. Verify STO circuit is closed. |
| Motor energizes but no motion on command | Speed/accel reference may be zero. Check via serial debug logging. Verify hardware enable wiring. |
| Drive alarms on enable | STO open, encoder fault, or overtravel. Read fault code from drive display. Resolve before clearing. |

### 6.6 Position feedback doesn't match command

| Symptom | Check |
|---------|-------|
| ActualPosition always zero | Drive may not be in Position mode. Check OperatingMode = 1. |
| Position changes but wrong target | PUU scaling incorrect. Recalibrate per Section 4.3. Check drive electronic gearing matches mechanical parameters. |
| Position oscillates or overshoots | Drive tuning (PID gains) needs adjustment. Drive-level configuration, not firmware. |

---

## Stage 7: Data Collection Checklist

For each successful validation phase, capture and archive:

| Phase | File | Description | Required? |
|-------|------|-------------|-----------|
| A | `docs/captures/serial_phaseA.log` | Serial log from first successful ForwardOpen + 30s cyclic I/O | Required |
| B | `docs/captures/serial_phaseB_x.log` | Serial log from X-axis motion validation | Required |
| B | `docs/captures/serial_phaseB_z.log` | Serial log from Z-axis motion validation | Required |
| C | `docs/captures/serial_phaseC.log` | Serial log from HCS01 validation | Required |
| C | `docs/captures/phaseC_hcs01_byte_order.txt` | Confirmed byte order (little vs big endian) | Required |
| — | `docs/captures/eds_kinetix5100.eds` | Kinetix 5100 EDS file | Required |
| — | `docs/captures/eds_hcs01.eds` | HCS01 EDS file | Required |

**Optional Wireshark captures** (requires temporary switch tap per 0.6):

| Phase | File | Description |
|-------|------|-------------|
| A | `docs/captures/phaseA_tcp_lifecycle.pcapng` | First successful RegisterSession + ForwardOpen |
| A | `docs/captures/phaseA_cyclic_io.pcapng` | 30 seconds of cyclic Class 1 exchange |
| B | `docs/captures/phaseB_x_position_move.pcapng` | X-axis position move command + actual feedback |
| B | `docs/captures/phaseB_z_position_move.pcapng` | Z-axis position move command + actual feedback |
| C | `docs/captures/phaseC_hcs01_lifecycle.pcapng` | HCS01 lifecycle |
| C | `docs/captures/phaseC_hcs01_positioning.pcapng` | HCS01 positioning command |

### Serial monitor command

```
idf.py -p COM<N> monitor --timestamp
```

Use `--timestamp` for millisecond-resolution timestamps. Save the full
terminal output to the capture files listed above.

### Wireshark capture (optional — requires switch tap)

If a temporary switch is inserted per 0.6, these display filters are
useful:

```
tcp.port == 44818          # TCP handshake + RegisterSession + ForwardOpen
udp.port == 2222           # Cyclic I/O frames
eip.cip.status != 0x00     # CIP errors (non-zero status)
eip                        # All EIP encapsulation + CIP dissection
```

---

## Stage 8: Acceptance Criteria

The EtherNet/IP bench validation is **complete for Phase A+B** when:

- [ ] Kinetix 5100 X-axis: ForwardOpen accepted, cyclic I/O active,
       ServoOn enables drive, position commands produce physical motion,
       ActualPosition feedback is correct, PUU/mm scaling is calibrated,
       alarm/fault handling works.
- [ ] Kinetix 5100 Z-axis: same criteria as X.
- [ ] Endstop validation: limit switch activation on each axis reports
       fault via the drive display and serial log.
- [ ] Serial logs are saved to `docs/captures/`.
- [ ] Kinetix 5100 EDS file is saved to `docs/captures/`.

**Deferred — Phase C (HCS01 theta):**
- [ ] 3-phase AC supply available
- [ ] HCS01 inserted into chain between Kinetix Z and PC
- [ ] ForwardOpen accepted with assemblies 101/102
- [ ] Status word reports correct ready/AF states
- [ ] Positioning command moves rotary module
- [ ] PUU/deg scaling calibrated
- [ ] Data format endianness confirmed
- [ ] HCS01 EDS file saved

### Post-validation actions

1. **If Phases A+B pass (HCS01 deferred):**
   - The EtherNet/IP originator is bench-validated against both Kinetix
     5100 drives (X, Z) on a line topology.
   - Step/direction remains as a proven fallback.
   - HCS01 theta validation is on hold pending 3-phase supply.
   - Multi-axis simultaneous EIP (connecting to multiple drives
     concurrently) is a future step.

2. **If any phase fails:**
   - Document the specific failure mode and CIP error code.
   - Step/direction fallback remains operational — the gantry can still
     run in the current PulseMotor configuration.

3. **For all outcomes:**
   - Update `docs/EIP_MIGRATION.md` section 7 (Open items) with
     confirmed values from the EDS files.
   - Remove PROVISIONAL markers from `ScannerConfig` defaults once
     the EDS values are confirmed.
   - Update Kconfig defaults from provisional to confirmed values.

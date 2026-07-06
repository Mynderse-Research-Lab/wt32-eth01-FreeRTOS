# EtherNet/IP Bench Validation Checklist

Step-by-step guide to validate the ESP32 EtherNet/IP scanner against real Kinetix 5100 and HCS01 drives on the bench. Read sections sequentially -- later phases depend on the data captured and confirmed in earlier ones.

Reference: [EIP_MIGRATION.md](EIP_MIGRATION.md) for byte maps, protocol layering, and process-data design.
Host test suite: `test/host/test_eip_encoding.cpp`, `test_eip_axis.cpp`, `test_eip_transport.cpp`.

---

## 1. Prerequisites

### Documents still needed (gate items)

| Item | Status | Blocks |
|------|--------|--------|
| Kinetix 5100 EDS | MISSING | ForwardOpen config-assembly instance, connection path, Run/Idle header |
| HCS01 EDS (`IndraDrive_EIP_MPx18.EDS`) | MISSING | Exact connection points + config-assembly instance (Functions manual line 5816) |

These two EDS files are the single biggest blockers. Every ForwardOpen parameter that is currently PROVISIONAL in `ScannerConfig` must be confirmed against them before the drive will accept a connection.

### Hardware on bench

| Item | Qty | Notes |
|------|-----|-------|
| WT32-ETH01 | 1 | Flashed with ESP-IDF v6.0 firmware |
| Kinetix 5100 (2198-E1020-ERS) | 1 | X-axis (belt) — primary validation target, dual-port EtherNet/IP |
| Kinetix 5100 (2198-E1004-ERS) | 1 | Z-axis (ballscrew) — validate after X, dual-port EtherNet/IP |
| HCS01.1E-W0005-A-03-B-ET-EC | 1 | Theta-axis — **DEFERRED** (3-phase supply unavailable; validate later) |
| Ethernet patch cables | 3 | CAT5e or better: WT32→X, X→Z, Z→PC (daisy-chain) |
| 24 V DC power supply | 1 | 1606-XL or equivalent, field-side power |
| 5 V DC regulator | 1 | From 24 V rail, powers opto barrier + 74AHCT244 |
| Motion I/O interface board | 1 | Per [MOTION_IO_INTERFACE.md](MOTION_IO_INTERFACE.md) |

No external Ethernet switch. Line topology: WT32 → Kinetix X → Kinetix Z → PC.
HCS01 will be inserted between Z and PC when 3-phase power is available.
See [BENCH_VALIDATION_PLAN.md](BENCH_VALIDATION_PLAN.md) section 0.3.

### Software on engineering PC

| Tool | Purpose |
|------|---------|
| KNX5100C | Kinetix 5100 commissioning: set IP, mode, assembly config |
| IndraWorks (D, Engineering, or Cs) | HCS01 commissioning: EDS import, P-0-408x parameter setup |
| Wireshark 4.x | Capture TCP 44818 / UDP 2222; verify byte-level correctness |
| Serial terminal (idf.py monitor or PuTTY) | ESP32 log output at 115200 8N1 |
| `idf.py` | Build with menuconfig, flash, monitor |

---

## 2. Network plan

### 2.0 Pre-flight: endstop wiring (NEW - 2026-07-03)

Before powering on any drive for EIP bench validation, confirm that endstop limit switches are wired directly to each drive's digital inputs, not to the MCP23S17 GPIO expander.

**Checklist:**

- [ ] X-axis: forward/reverse limit switches wired to Kinetix 5100 TBIO INPUT1/INPUT2 (pins 9/10)
- [ ] Z-axis: forward/reverse limit switches wired to Kinetix 5100 TBIO INPUT3/INPUT4 (pins 34/8)
- [ ] Theta-axis: limit switches (if present) wired to HCS01 X31.5/X31.6
- [ ] KNX5100C: DIO Configuration assigns INPUT1-4 as Forward/Reverse Limit
- [ ] IndraWorks DS: P-0-0222 configures X31.5/X31.6 as travel range limit switches
- [ ] menuconfig: `CONFIG_EIP_ENDSTOP_SOURCE` set to "Drive digital inputs (via EIP assembly status)"

See [ENDSTOP_WIRING_PLAN.md](ENDSTOP_WIRING_PLAN.md) for full pinout tables and wiring diagrams.

The drive handles limit switch monitoring internally. The firmware reads Fault/Stopped status from the EIP assembly feedback path.

### 2.1 Network plan

Isolated bench subnet, daisy-chain topology (HCS01 deferred).

```
WT32 (.1.10) → Kinetix X (.1.20) → Kinetix Z (.1.21) → PC (.1.1)
     ↑                ↑                    ↑              ↑
  single port     dual-port           dual-port       endpoint
                 embedded switch     embedded switch

  HCS01 (.1.30) — DEFERRED (no 3-phase). Will insert between Z and PC.
```

| Device | IP | Mask | Gateway | Status |
|--------|----|------|---------|--------|
| Engineering PC | 192.168.1.1 | 255.255.255.0 | — | Active |
| WT32-ETH01 | 192.168.1.10 | 255.255.255.0 | 192.168.1.1 | Active |
| Kinetix 5100 X | 192.168.1.20 | 255.255.255.0 | 192.168.1.1 | Active |
| Kinetix 5100 Z | 192.168.1.21 | 255.255.255.0 | 192.168.1.1 | Active |
| HCS01 | 192.168.1.30 | 255.255.255.0 | 192.168.1.1 | **Deferred** |

Both Kinetix drives must be powered for chain continuity.

Verify basic connectivity before flashing EIP firmware:

1. Power up BOTH Kinetix drives and verify link LEDs on all ports.
2. Power up WT32 (no EIP firmware).
3. Ping 192.168.1.20 and 192.168.1.21 from the PC. Both must respond.
4. Check WT32 serial log for `MqttBridge` IP output.

---

## 3. Kinetix 5100 commissioning

Goal: one Kinetix 5100 drive at a known IP, in EtherNet/IP I/O mode, ready to accept a ForwardOpen for assemblies 104 (output) and 154 (input).

### 3.1 Drive IP

- Connect KNX5100C via USB (or Ethernet if pre-configured).
- Set IP to the address chosen in section 2.
- If using rotary switches, set to a value that maps to the target IP (consult drive manual).

### 3.2 EDS extraction (critical)

Load the Kinetix 5100 EDS file into a text editor or EDS viewer and record:

- **Config assembly instance** (the `configuration` CP in the ForwardOpen connection path). Currently PROVISIONAL = 1.
- **Output assembly 104 size**: confirm 40 bytes. If the EDS advertises a different size (e.g. 44), update `ScannerConfig::ot_assembly_size`.
- **Input assembly 154 size**: confirm 52 bytes. Same rule.
- **Run/Idle header**: does the drive require the 32-bit Run/Idle header in the CIP sequence count slot? Currently `include_run_idle_header = true` -- confirm against the EDS's RunIdle capability flag.
- **Connection point numbers**: the EDS may map assembly instance 104 to a connection-point number. Record it for the connection path.

### 3.3 Verification without the scanner

If Wireshark is available, use a simple TCP client (e.g. `ncat` or Python `socket`) to send a `ListIdentity` request to UDP 44818 broadcast. A Kinetix 5100 should respond with its identity object. This confirms EIP is active on the drive without needing ForwardOpen.

### 3.4 RPI

- Kinetix 5100 default RPI: **20 ms** (user manual line 812).
- Minimum RPI: **2.0 ms**.
- Start with the Kconfig default `EIP_X_RPI_US = 5000` (5 ms). If the drive rejects the ForwardOpen with "RPI too low", increase to 10000. If accepted, 5000 is fine -- lower RPI means faster feedback but more CPU load.

---

## 4. HCS01 commissioning

Goal: HCS01 configured for freely-configurable profile (`P-0-4084 = 0xFFFE`) with the process-data map from [EIP_MIGRATION.md section 4.2](EIP_MIGRATION.md#42-recommended-map---drive-controlled-positioning-theta-is-rotary).

### 4.1 IndraWorks setup

1. Import the HCS01 EDS into IndraWorks.
2. Set the **engineering IP** (`S-0-1020`): this is the IP IndraWorks uses to talk to the drive for commissioning. It can be on the same subnet as the EIP IP or a separate engineering network.
3. Set the **EIP IP** via `P-0-4089.0.13` (IP), `P-0-4089.0.14` (netmask), `P-0-4089.0.15` (gateway).

### 4.2 Process data configuration

In IndraWorks, open the cyclic data configuration dialog and set:

| Parameter | Value | Notes |
|-----------|-------|-------|
| `P-0-4084` | `0xFFFE` | Freely configurable profile |
| `P-0-4081` | List of 5 words | O->T command data (see below) |
| `P-0-4080` | List of 7 words | T->O actual data (see below) |
| `P-0-4076` | 2 (ms) or higher | Process-data update clock / API |
| `P-0-4074` | Verify | Data format: must be Intel/little-endian |

**`P-0-4081` command list (O->T, 10 bytes = 5 words):**

| Word | Parameter | Type | Description |
|------|-----------|------|-------------|
| 1 | `P-0-4077` | u16 | Field bus control word |
| 2-3 | `S-0-0282` | i32 | Positioning command value |
| 4-5 | `S-0-0259` | i32 | Positioning velocity |

**`P-0-4080` actual list (T->O, 14 bytes = 7 words):**

| Word | Parameter | Type | Description |
|------|-----------|------|-------------|
| 1 | `P-0-4078` | u16 | Field bus status word |
| 2-3 | `S-0-0051` | i32 | Position feedback value 1 |
| 4-5 | `S-0-0040` | i32 | Velocity feedback value |
| 6-7 | `S-0-0390` | u32 | Diagnostic message number |

### 4.3 Data format tension (P-0-4074)

The Functions manual lists `(H)` before `(L)` in the parameter tables, which suggests big-endian word order. However, section 4.8 explicitly states "Intel format" (little-endian) for the EtherNet/IP path.

Resolution: this must be confirmed at the bench.
- Capture a T->O frame during ForwardOpen and inspect byte order of `P-0-4078` (status word).
- If the status word `0x0003` (bits 1/0 = 10 for "operating mode") appears as `03 00` in the frame, it's little-endian (our codec is correct).
- If it appears as `00 03`, adjust `Hcs01Assembly.cpp` to swap byte order per word.

### 4.4 Scaling (PUU/deg)

The HCS01 does not have a fixed PUU-to-degree ratio -- it depends on the motor encoder, gear ratio, and IndraWorks scaling setup. The Kconfig default `EIP_AXIS_THETA_PUU_PER_DEG = 100.0` is a placeholder.

**To determine the correct value:**
1. In IndraWorks, note the scaling for `S-0-0282` (positioning command value). The raw value per encoder revolution depends on `S-0-0116` (encoder resolution x gear ratio).
2. Compute: `PUU_per_deg = (encoder_counts_per_rev * gear_ratio) / 360.0`.
3. Enter this in Kconfig and verify with a short move command.

---

## 5. ESP32 firmware config

Kconfig selections via `idf.py menuconfig` from the `idf/` directory.

### 5.1 Phase A: scanner idle (first flash)

```
EtherNet/IP originator
  [*] Enable EtherNet/IP scanner task
      EtherNet/IP axis selection (X axis over EIP (Kinetix 5100))
      (192.168.1.20) Target drive IP address
      (1000.0) X-axis PUU/mm scaling
      (5000) X/Z axis RPI (microseconds)
```

Expected behavior after flash: the scanner task waits for Ethernet link-up, then attempts TCP connect -> RegisterSession -> ForwardOpen to 192.168.1.20. The process image is wired so the `GantryEipLinearAxis` adapter is live (but the drive will not move until a ServoOn command is issued).

### 5.2 lwIP and Ethernet (already working)

These settings are already correct from the MQTT configuration -- do not change unless needed:

- `Ethernet -> Support ESP32 internal EMAC controller` = y (RMII on WT32-ETH01)
- `LWIP -> Enable IPV4` = y
- DHCP client or static IP per your bench subnet

### 5.3 Phase B: Z axis (same as X, different IP)

After X passes, change:

```
EtherNet/IP axis selection (Z axis over EIP (Kinetix 5100))
(192.168.1.21) Target drive IP address
(4000.0) Z-axis PUU/mm scaling
```

### 5.4 Phase C: HCS01 theta

After both Kinetix axes pass:

```
EtherNet/IP axis selection (Theta axis over EIP (HCS01))
(192.168.1.30) Target drive IP address
(<computed>) Theta-axis PUU/deg scaling
(4000) Theta axis RPI (microseconds)
```

---

## 6. Validation sequence -- Phase A: Kinetix X-axis TCP + EIP lifecycle

First and most critical phase. All later phases assume this succeeds.

### 6.1 Power-on and link check

1. Power up Kinetix X-axis drive.
2. Power up WT32-ETH01.
3. Confirm Ethernet link LEDs lit on both the drive and the switch port.
4. Serial log should show `MqttBridge` getting an IP, then `EipScanner task started`.
5. If the scanner logs `Connect to 192.168.1.20 failed`, check: drive IP, Ethernet cable, switch.

### 6.2 Wireshark capture

Start Wireshark on the engineering PC, connected to the same switch. Filter:

```
tcp.port == 44818 || udp.port == 2222
```

Expected sequence:

```
ESP32:446xx -> 192.168.1.20:44818 [SYN]
192.168.1.20:44818 -> ESP32:446xx [SYN, ACK]
ESP32:446xx -> 192.168.1.20:44818 [ACK]
... TCP handshake complete ...

ESP32 -> Kinetix: RegisterSession (encapsulation 0x0065)
Kinetix -> ESP32: RegisterSession reply (session handle)
ESP32 -> Kinetix: SendRRData (ForwardOpen, service 0x54)
Kinetix -> ESP32: SendRRData reply (ForwardOpen success)
ESP32 -> Kinetix: UDP O->T frame (idle assembly 104, servo off)
Kinetix -> ESP32: UDP T->O frame (assembly 154, status feedback)
```

If ForwardOpen is **rejected** (general status != 0x00), the extended status code tells you why. Common causes:

- **0x0107** (connection not found): wrong config-assembly instance or connection path. Check EDS.
- **0x0111** (invalid connection size): `ot_assembly_size` or `to_assembly_size` doesn't match the drive. Update `ScannerConfig`.
- **0x0118** (RPI not supported): increase `EIP_X_RPI_US`.
- **0x011A** (vendor ID or serial mismatch): drive may require a specific originator vendor ID. Set `originator_vendor_id` in `ScannerConfig`.

### 6.3 Serial log confirmation

Expected log lines (approximate wording):

```
I (12345) EipScanner: EipScanner task started (target 192.168.1.20)
I (12450) EipScanner: ForwardOpen ok O->T=0x10000001 T->O=0x00000001 API=5000 us
I (12455) app_main: X axis running over EIP (Kinetix 5100, 1000.0 PUU/mm), target 192.168.1.20
```

After this, the scanner loop runs: every RPI interval it sends the idle O->T assembly and receives T->O feedback, feeding both through `EipProcessImage`.

### 6.4 Data parse check

In the serial log, add temporary debug output (or inspect in Wireshark) to confirm the T->O frame bytes match the `InputAssembly154` layout:

- Offset 24-27: `ActualPosition` (PUU) -- should change when the motor shaft is rotated by hand.
- Offset 9 bit 4: `CommandInProgress` -- should be 0 (no motion commanded yet).
- Offset 8 bit 1: `Fault` -- should be 0 if drive is healthy.

### 6.5 Process image wired

Confirm the `GantryEipLinearAxis` is functioning:
1. Call `gantry.begin()` -- should succeed if the scanner is connected and the EIP axis adapter initializes.
2. Call `gantry.enable()` -- should send a ServoOn command through the process image.
3. Observe on Wireshark: O->T frame byte 1 bit 0 transitions from 0 to 1.

---

## 7. Validation sequence -- Phase B: Kinetix motion

After Phase A confirms the TCP + EIP lifecycle, test actual motion.

**Safety:** ensure the drive's STO (Safe Torque Off) is wired correctly and the motor can spin freely without hitting anything. Start with very small moves.

### 7.1 Servo enable

1. With the scanner connected, issue `gantry.enable()` via serial console or MQTT.
2. Verify Wireshark shows `ServoOn` bit = 1 in O->T frame byte 1.
3. The drive's status LED should change to indicate enabled state.

### 7.2 Small position move

1. Issue a small move via serial console (e.g. `move <x> <z> <theta>` or use `gantry.moveTo()`).
2. Verify on Wireshark:
   - O->T byte 1 bit 4 (`StartMotion`) pulses.
   - O->T bytes 16-19 (`PositionReference`) change to the target PUU.
   - T->O bytes 24-27 (`ActualPosition`) start changing as the motor moves.
   - T->O byte 9 bit 4 (`CommandInProgress`) toggles.
3. When motion completes, T->O byte 9 bit 7 (`AtReference`) should be 1.

### 7.3 PUU-to-mm scaling check

1. Command a known mm distance (e.g. 10 mm).
2. Measure the actual distance the actuator traveled.
3. If the distance is off, adjust `EIP_AXIS_X_PUU_PER_MM` in Kconfig:
   ```
   new_puu_per_mm = old_puu_per_mm * (commanded_mm / actual_mm)
   ```
4. Re-flash and repeat until within tolerance.

### 7.4 Alarm handling

1. Trigger a drive alarm (e.g. disconnect motor encoder briefly).
2. T->O byte 8 bit 1 (`Fault`) should be 1.
3. The `GantryEipLinearAxis` alarm path should report the alarm.
4. Resolve the fault, issue `clearAlarm()`, confirm Fault bit returns to 0.

---

## 8. Validation sequence -- Phase C: HCS01 theta

After X and Z Kinetix axes are validated, move to the HCS01.

### 8.1 Prerequisite check

Before flashing with `EIP_AXIS_THETA`, confirm in IndraWorks:

- All commissioning steps from section 4 are complete.
- `P-0-4084 = 0xFFFE`.
- The `P-0-4081`/`P-0-4080` lists match section 4.2.
- Ethernet link is up (check the drive's display or IndraWorks status).

### 8.2 TCP lifecycle

Same pattern as Phase A:

1. Flash with `EIP_AXIS_THETA`, target IP = HCS01.
2. Verify TCP -> RegisterSession -> ForwardOpen (assemblies 101/102).
3. If ForwardOpen is rejected, confirm:
   - Connection path matches the HCS01 EDS.
   - `ot_assembly_size = 10` (bytes) and `to_assembly_size = 14`.
   - `config_assembly_instance` is correct (consult EDS).

### 8.3 Status word check

With the connection open, verify the T->O status word `P-0-4078`:

| Expected state | Bits 15/14 | Bits 1/0 | Meaning |
|---------------|------------|----------|---------|
| Power stage off, no enable | 01 (bb) | 00 | Ready, parameter mode |
| Controller enabled, no motion | 11 (AF) | 10 | In operation, operating mode |

If bits 15/14 never reach AF after issuing Drive ON (control bit 15), check:
- Drive hardware enable (X31 wiring per [MOTION_IO_INTERFACE.md](MOTION_IO_INTERFACE.md)).
- That `P-0-4077` bit 14 auto-sets when field-bus comm is active (per manual).

### 8.4 Positioning command

1. Issue Drive ON (control word bit 15 = 1).
2. Issue Drive Start (control word bit 13 = 1).
3. Toggle command value acceptance (control word bit 0) and write a target position to `S-0-0282`.
4. Wait for status word bit 10 to toggle (command accepted).
5. Wait for status word bit 4 (`in position`) = 1.

### 8.5 PUU-to-degree scaling check

1. Command a known angular move (e.g. 90 degrees).
2. Verify the actual rotation matches.
3. Adjust `EIP_AXIS_THETA_PUU_PER_DEG` if needed.

---

## 9. What to capture and save

For each successful phase, save the following into `docs/captures/`:

| File | Content |
|------|---------|
| `phaseA_tcp_lifecycle.pcapng` | Wireshark capture of first successful RegisterSession + ForwardOpen |
| `phaseA_cyclic_io.pcapng` | 30 seconds of cyclic Class 1 exchange (O->T + T->O) |
| `phaseB_position_move.pcapng` | Capture of a position move command and actual feedback |
| `phaseC_hcs01.pcapng` | HCS01 lifecycle + positioning command |
| `serial_phaseA.log` | Serial log from first successful ForwardOpen |
| `eds_kinetix5100.eds` | Kinetix 5100 EDS file (once obtained) |
| `eds_hcs01.eds` | HCS01 EDS file (once obtained) |

Wireshark display filters to use during capture:

```
# Shows only EIP encapsulation + CIP
eip

# Basic sanity: TCP handshake + RegisterSession
tcp.port == 44818

# Cyclic I/O frames
udp.port == 2222

# Check for CIP errors in ForwardOpen replies
eip.cip.status != 0x00
```

Serial monitor with timestamps:

```
idf.py -p COM<N> monitor --timestamp
```

---

## 10. Troubleshooting tree

### TCP connect fails

| Symptom | Cause | Fix |
|---------|-------|-----|
| `Connect to <ip> failed` | Wrong IP, no link, drive not powered | Ping drive from PC; check Ethernet LEDs; verify IP in KNX5100C/IndraWorks |
| TCP RST after SYN | Drive not listening on 44818 | Confirm EIP is enabled on the drive; check drive configuration mode |
| Connection hangs | Firewall, ARP failure | All devices on same unmanaged switch; no firewall between them |

### RegisterSession fails

| Symptom | Cause | Fix |
|---------|-------|-----|
| No reply to RegisterSession | Encapsulation command bytes wrong | Verify encapsulation header: command 0x0065, length 4, protocol version 1, options 0 |
| Reply with status 0x01 (invalid command) | Drive does not support RegisterSession (unlikely for Class 1 devices) | Double-check target IP is a Kinetix 5100 or HCS01, not another device |
| Reply with status 0x64 (no resources) | Drive already has max sessions | Power-cycle the drive |

### ForwardOpen rejected

| Extended status | Meaning | Fix |
|----------------|---------|-----|
| 0x0107 | Connection point not found | Wrong assembly instance or connection path. Check EDS. |
| 0x0111 | Invalid connection size | Assembly size mismatch. Verify `ot_assembly_size` and `to_assembly_size` against EDS. |
| 0x0118 | RPI not supported | Increase `EIP_X_RPI_US` or `EIP_THETA_RPI_US`. Try 10000 (10 ms). |
| 0x0108 | Connection type not supported | Drive may require Exclusive Owner with specific transport class. Verify `transport_class_trigger` in `EipScanner.cpp`. |
| 0x011A | Vendor ID mismatch | Some drives require a non-zero originator vendor ID. Set `originator_vendor_id` in `ScannerConfig` (e.g. 0x0001 for Rockwell). |

### ForwardOpen succeeds but no UDP frames

| Symptom | Cause | Fix |
|---------|-------|-----|
| O->T frames sent, no T->O reply | Wrong T->O connection ID, or Run/Idle header missing | Verify `to_connection_id` from ForwardOpen reply. Check `include_run_idle_header` against EDS. |
| UDP frames exchanged but `parseInputFrame` fails | Sequence count mismatch or wrong assembly size | Check CIP sequence count handling in `EipIoConnection`. Verify Run/Idle header position. |

### Cyclic exchange works but axis doesn't move

| Symptom | Cause | Fix |
|---------|-------|-----|
| ServoOn bit set, no motion | Drive in wrong mode (PT/step-dir instead of EIP) | In KNX5100C, set operating mode to EtherNet/IP. Check that PT mode is not active. |
| Motion command sent, no response | Speed reference zero, or drive not enabled | Confirm speed/accel reference values are non-zero. Check hardware enable wiring (SON input, STO). |
| Drive alarm / fault | STO open, encoder fault, overtravel | Check drive display for fault code. Clear via `clearAlarm()` after resolving cause. |

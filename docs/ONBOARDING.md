# Engineer Onboarding

**Status:** Living template. Filled in from the code, live logs and project docs, last checked 2026-10-30 at commit `d6e8f8e`.
**Audience:** Engineers continuing development on the battery sorting cell.
**Companion docs:** [INDEX.md](INDEX.md), [../CONTRIBUTING.md](../CONTRIBUTING.md).

This page is the map, not the manual. It says what each subsystem is, how the pieces connect, where the source lives, and what is unfinished. Details stay in the canonical docs it links to.

## How to use and maintain this page

1. Read sections 1 to 4 on day one.
2. Read the section for your subsystem, then its linked docs.
3. Check the open items under each component before starting work, and update them when you finish.

Fields marked `TBD` are gaps to fill in. Appendix A has a blank block for adding a component, or a second gantry.

Every open item carries an evidence label. **Do not treat a label lower than Confirmed as fact.**

| Label | Meaning |
|-------|---------|
| **Confirmed** | Checked against code, a live log or a measurement. |
| **Suggested** | From an AI-generated plan and not yet checked. Treat it as a lead. |
| **Decision** | Needs a call from the project lead before anyone starts. |

Items with `(plan)` also appear in the plans listed in [Appendix B](#appendix-b-suggested-plans-since-2026-10-29).

---

## 1. What the cell does

The cell sorts batteries off a moving conveyor. Vision reports where a battery is and what it is, the conveyor reports how far the belt has moved, and the gantry intercepts the battery on the moving belt, grips it and drops it in a bin. The supervisor coordinates and gives engineers a way in.

```mermaid
flowchart LR
  subgraph cell ["Closed cell network: 10.42.0.0/24 and CellNet Layer 2 (0x88B5)"]
    V["Vision IPC<br/>node 0x01<br/>10.42.0.2"]
    C["Conveyor WT32<br/>node 0x02<br/>address TBD, see C-2"]
    G["Gantry WT32<br/>node 0x03<br/>10.42.0.100"]
    S["Supervisor Pi<br/>node 0x04<br/>10.42.0.1"]
  end
  subgraph bus ["Motion bus: 192.168.1.0/24, EtherNet/IP"]
    X["Kinetix X<br/>.20"]
    Z["Kinetix Z<br/>.21"]
    T["HCS01 theta<br/>.23"]
  end
  V -- "VISION_DETECT" --> G
  C -- "CONVEYOR_SPEED" --> G
  S -- "CELL_COMMAND, HEARTBEAT" --> G
  G -- "GANTRY_STATUS" --> S
  G -- "W5500, Class 1, 2 ms RPI" --> X
  X --> Z --> T
  S -. "TCP console :2323, OTA :8032" .-> G
  S -. "NTP :123" .-> V
  S -. "NTP :123" .-> G
  E["Engineer PC"] -. "ZeroTier 10.168.40.1, toolbox /dev" .-> S
```

**One pick, end to end:**

1. Vision (today, the Gantry team's trigger-sensor workaround) sends `VISION_DETECT` with position, angle, size and class.
2. The gantry places the part on the belt using `CONVEYOR_SPEED` displacement and the configured vision latency.
3. The pick scheduler plans the approach and the strike, and moves to the hover pose.
4. Timers release the Z plunge, fire the grip and start the retract, all timed from the part's arrival.
5. The gantry carries the part to the bin, opens, and returns to staging.

Maths and timing are in [PICK_SCHEDULER_KINEMATICS.md](PICK_SCHEDULER_KINEMATICS.md) and [ON_THE_FLY_PICKUP.md](ON_THE_FLY_PICKUP.md).

## 2. How the subsystems connect

| Link | From → to | Transport | Contract | Failure you will see |
|------|-----------|-----------|----------|----------------------|
| Detections | Vision → gantry, supervisor | CellNet L2 | `L2VisionDetectPayload` | No `vision queued` in the gantry log |
| Belt position | Conveyor → gantry, supervisor | CellNet L2 | `L2ConveyorSpeedPayload` | Picks held, stale belt |
| Commands | Supervisor → gantry | CellNet L2 | `CELL_COMMAND` ids 1 to 6 | No response to START |
| Status | Gantry → supervisor | CellNet L2 | `L2GantryStatusPayload` | `gantry1 online:false` |
| Console, OTA | Supervisor → gantry | TCP `:2323`, `:8032` | Console contract, [LOW_LEVEL §9](LOW_LEVEL_GANTRY_CONTROL.md#9-console-contract) | Console `busy`; send `stop` |
| Motion | Gantry → drives | EtherNet/IP Class 1, 2 ms | Assemblies, [LOW_LEVEL §5](LOW_LEVEL_GANTRY_CONTROL.md#5-class-1-io--wire-format-and-udp-lifecycle) | `E602`, `F4009`, `T->O stale` |
| Time | Supervisor → gantry, vision | NTP | `ntp_server` key | `cfg time` shows `no clock` |

Wire format: [CELL_NET_L2_COMMUNICATION_GUIDE.md](CELL_NET_L2_COMMUNICATION_GUIDE.md). The header is `include/cell_net_l2_protocol.h`, with copies in the conveyor and supervisor projects (see X-3).

**Never connect the motion bus to the cell switch.** [EXPECTED_ELECTROMECHANICAL_ASSEMBLY §3](EXPECTED_ELECTROMECHANICAL_ASSEMBLY.md#3-networks--cell-topology) explains the isolation rule.

## 3. Where everything lives

| Subsystem | Source | Version control | Runs on |
|-----------|--------|-----------------|---------|
| Gantry | This repo | Git, `upstream` = `Mynderse-Research-Lab/wt32-eth01-FreeRTOS` | WT32-ETH01, ESP-IDF v6.0 |
| Conveyor | [wt32-eth01-velmeter](https://github.com/Mynderse-Research-Lab/wt32-eth01-velmeter) | Git, branches `main` and `testing` | WT32-ETH01, ESP-IDF 5.4+ |
| Vision | Computer Science team; repo TBD. Trigger workaround: `~/vision_sim/` on the vision host | **None known**, TBD | Ubuntu 22.04 IPC |
| Supervisor | `~/ADSB-PI-base` on the Pi | **None on the Pi** (see S-1) | Raspberry Pi, Debian 13 |
| Production HMI, planned | `D:\Projects\sorting-cell-coordinator` | Git, last commit 2026-08-20 | Not deployed, see S-5 |

## 4. First-day setup

Ask the project lead for credentials. **Do not put passwords in docs, scripts or commits.** Tools read `ADSB_DEV_PASS` and `PI_PASS` from the environment.

| Step | Command or place | Done means |
|------|------------------|------------|
| Reach the cell | ZeroTier to `10.168.40.1`, or a free address on `10.42.0.0/24` | Toolbox opens at `http://10.168.40.1/dev` |
| Host tests | `cmake -S test/host -B build/host` then `cmake --build build/host` and `ctest --test-dir build/host --output-on-failure` | 28 of 28 pass; both must pass before a change is final |
| Firmware | `idf.py -C idf build`, target `esp32` | Image builds, from `sdkconfig.defaults` only |
| Read the rules | [Working rules](#machine-safety-and-working-rules) below, and [CONTRIBUTING.md](../CONTRIBUTING.md) | You know what not to do |
| Read the live log | Toolbox log view, or `/dev/log` | You can see `pick actual` lines |

**Windows gotchas:** a fresh host build directory needs `-G "MinGW Makefiles"` on the first `cmake`, otherwise CMake picks NMake and fails. A PowerShell `python` alias can break ESP-IDF's `export.ps1`; run `Remove-Item alias:python` first.

### Machine safety and working rules

| Rule | Why |
|------|-----|
| Send `stop`, then `status`, before every OTA flash | Motors must be off and the gantry idle |
| Test with the 9V cell only | The trigger-sensor workaround sends one fixed frame until the ML model is ready |
| At least 7 s between strikes; regulator at 6 bar | The KGG 100-80 closing time of 0.19 s is specified at 6 bar. Reason for the 7 s spacing: TBD |
| Eliminate Class 1 drops; never add auto-recovery | A hidden link fault on a motion system is a safety problem |
| Stored parameters are the only runtime source | Primary objective: avoid NVS read and write delays, so values are loaded once and motion uses RAM. Motion and pick code also never reads a macro for a value with a stored key |
| Pick code never keeps its own copy of a gantry parameter | One unified source of parameters |
| No NVS writes at runtime except `cfg save` | A flash erase outlasts the 32 ms Class 1 timeout |
| Sim gains are not facts | Timing comes from real drive configs or on-system measurement |
| Ask before restarting `adsb-supervisor` | It drops the cell supervisor |
| Keep the TFT enabled | GPIO17 can be used temporarily for experiments, but it is not free and must be reverted to its role |

---

## 5. Gantry

Moves the end effector on X, Z and theta, intercepts cells, and drops them in the bin. **There is one gantry today, and more are planned** (see G-A1). Copy this section for each new gantry.

**Read:** [LOW_LEVEL_GANTRY_CONTROL.md](LOW_LEVEL_GANTRY_CONTROL.md), [EXPECTED_ELECTROMECHANICAL_ASSEMBLY.md](EXPECTED_ELECTROMECHANICAL_ASSEMBLY.md), [HV_LV_SCHEMATICS.md](HV_LV_SCHEMATICS.md), `pinout.csv`.

### 5.1 Axes and mechanics

| Axis | Actuator | Drive | Address |
|------|----------|-------|---------|
| X, across belt | SCHUNK Beta 100-ZRS belt | Kinetix 2198-E1020-ERS | `192.168.1.20` |
| Z, +Z down | SCHUNK Beta 80-SRS ballscrew | Kinetix 2198-E1004-ERS | `192.168.1.21` |
| Theta, about Z | SCHUNK ERD 04-40-D-H-N | HCS01.1E-W0005-A-03-B-ET-EC | `192.168.1.23` |

Code: `lib/Gantry`, with drive limits and homing at the drive. Y has no actuator; the belt provides it.

No conveyor-height sensor will be used. The conveyor region is set by parameters (`col_x_min_mm`, `col_z_min_mm`).

**Open items and blockers**

| ID | Item | Evidence | Next step |
|----|------|----------|-----------|
| G-A1 | Multiple gantries are planned. Preferred: one gantry manages 2 classes, dropping one class left and the other right. Fallback: one gantry for each class (9V, laptop, li-ion pack, powertool battery) | Confirmed plan. Not supported yet: one `GANTRY` node id, one `gantry1` status field, one bin pose | Add node ids and per-class drop poses; start with two-sided drop |
| G-A2 | Z continuous torque is about 0.95× the SCHUNK 1.35 N·m requirement | Confirmed in [DEV_TRACKER](DEV_TRACKER.md), not re-measured | Review before production duty |
| G-A4 | Why Z reads about −42 instead of 48 after a reboot, before `calibrate all` | Confirmed reading; a physical drop was not ruled out | Observe Z across a reboot |

### 5.2 Motion bus (W5500, EtherNet/IP)

WIZ850io on SPI2 runs the originator: Class 1 at a 2 ms RPI, timeout 16 × RPI = 32 ms. Code: `lib/EtherNetIP`, `lib/W5500`. Any stall over 32 ms drops the drives.

**EtherNet/IP background for the items below.** The gantry is the originator and the drives are targets.

| Concept | What it means | On this machine | Source |
|---------|---------------|-----------------|--------|
| Explicit and implicit messaging | Explicit is request/reply over TCP for non-time-critical data. Implicit is real-time I/O over UDP | Cyclic drive I/O is implicit. Forward Open and Close are explicit | [ODVA PUB00213](https://www.odva.org/wp-content/uploads/2020/05/PUB00213R0_EtherNetIP_Developers_Guide.pdf), p. 11, Table 1 |
| Encapsulation session | Required before any CIP traffic; port 44818 | `RegisterSession` on each drive, over TCP | ODVA PUB00213, p. 23 |
| Forward Open and Close | Open a connection and close it. Clean-up happens on a ForwardClose, or when either end times out | A drive that never gets the close times out, which is the G-B1 failure | ODVA PUB00213, pp. 11, 23 |
| Class 1 and RPI | The Kinetix is a Class 1 device exchanging data at a Requested Packet Interval. Minimum 2.0 ms, default 20 ms | We run at the minimum, see G-B7 | Kinetix 5100 manual, [2198-UM004D-EN-P](../driver_datasheets_and_calculations/AB_Kinetix5100_user_manual.pdf), pp. 25, 67, 197 |
| Connection timeout | Timeout = RPI × 4 << code. Code 2 gives ×16, so 32 ms | Why a 64 ms stall drops a drive | Repo only, `EipScannerPolicy.h:16`. Not checked against the CIP specification |
| UDP port 2222 | Implicit I/O port | Scanner binds it | Vendor doc, [SEW-EURODRIVE](https://download.sew-eurodrive.com/download/html/30590930/en-EN/50551348235.html) |
| Encapsulation Inactivity Timeout | TCP/IP Interface object (0xF5) attribute 13. Default 120 s, range 0 to 3600, 0 disables. The target closes the TCP connection after that long with no encapsulation activity | Likely cause of G-B2, not yet verified | [TI EtherNet/IP adapter docs](https://software-dl.ti.com/processor-industrial-sw/esd/ind_comms_sdk/am64x/11_00_00_13/docs/am64x/ethernetip_adapter/group___e_i___a_p_i___a_d_p___t_c_p.html), [HMS KB article](https://www.hms-networks.com/support/tech-support/kb-articles/7379293556882-Ethernet-IP-connection-times-out-every-120-seconds) |
| `E602` | Kinetix fault, control connection lost | Seen on Class 1 loss | Repo only, `KinetixFaultCodes.h`. Not found in the manual's text search |

The authority is the ODVA CIP Networks Library: Volume 1 for CIP and the Connection Manager, Volume 2 for the EtherNet/IP adaptation. It is a paid specification and was not consulted here; the section numbers in `lib/EtherNetIP` comments have not been re-checked.

**Open items and blockers**

| ID | Item | Evidence | Next step |
|----|------|----------|-----------|
| G-B1 | `cfg save` closes Class 1 cleanly on X and Z, including rebuilding dropped sessions. Whether theta stays clear after a dropped-session close is still unknown | Confirmed 2026-10-30 at 723 s: X got no reply for 500 ms, then X and Z reconnected and the close was accepted, with no `E602`. Theta was clear after the 60 s save, but already latched at 723 s (G-B6) | Repeat with the alarm clear first, and only one session on the gantry |
| G-B2 | Kinetix explicit TCP sessions die while Class 1 keeps running. Sometimes the socket shows `CLOSE_WAIT`; on 2026-10-30 the send succeeded and no reply came, so a status check alone does not catch it | Confirmed in logs. Likely the Encapsulation Inactivity Timeout, since Class 1 leaves TCP silent after Forward Open; not verified on these drives, and it does not explain why theta's session survives | Read attribute 13 on X and Z just after Forward Open |
| G-B3 | Class 1 gaps of 6 to 17 ms with no `cfg` write nearby | Confirmed, cause not found | Below the timeout, but the worst leaves about half the margin |
| G-B4 | Cadence gate from `eiptiming` not met | Confirmed in [DEV_TRACKER](DEV_TRACKER.md), dated 2026-08-18 | Re-measure on the current image |
| G-B5 | Target IPs, RPI and PUU scales still come from Kconfig, so those `cfg` keys do nothing (plan P1.3) | Confirmed. Left on purpose: stored values are developer-locked and a wrong one changes the Class 1 target | Read the stored values first, then route them |
| G-B6 | A 65.7 ms Class 1 gap at 498.6 s, over the 32 ms timeout, dropped theta with the motors off and no save or `cfg` write nearby. The log shows a 64 ms wait for the W5500 bus mutex and, 2 s earlier, an MCP23S17 SPI lock timeout | Confirmed, cause not found. Two sessions were polling the console then, which may or may not matter | Reproduce with one console client; this is a drop with no known cause |
| G-B7 | We run the Kinetix at its 2.0 ms minimum RPI. Rockwell recommends 20 ms for this drive, as it is a simple I/O device rather than an integrated-motion drive | Confirmed: manual p. 197 | **Decision:** does the strike timing need 2 ms? A slower RPI would widen the timeout margin |

### 5.3 Pick scheduler

Turns detections into timed strikes. Code: `src/pick_scheduler.cpp`, `include/pick_intercept.h`, `include/strike_plan.h`. Bin drop and traverse run at the safe Z ceiling, 48.4 mm.

**Open items and blockers**

| ID | Item | Evidence | Next step |
|----|------|----------|-----------|
| G-C1 | Tuning: picks lift, but many grip only half the 9V cell | Confirmed 2026-10-30: 7 of 9 overhanging; offsets were still being tuned | Finish offset tuning; run `cfg export` for current values |
| G-C2 | Z reverses short of grasp depth: about 1.15 mm at 5000 mm/s² and zero dwell, past the accepted 1 mm | Confirmed: `pick actual` logs | Offsets of −60 / +10 reached 0.1 to 0.4 mm; **Decision:** is bottom dwell acceptable now? |
| G-C3 | Vision angle frame is unresolved. The wire comment says 0° is along +Y; the sorter script measures from camera +X; `cv_jaw_deg` = 90 compensates on the bench (plan P0.2) | Confirmed conflict; the plan's transform is unverified | **Decision:** fix the reference with the Computer Science team, then convert once. Verify before reusing the plan's formula |
| G-C4 | `classifyFollowers()`: 190 ms grip dwell in ready time, no strike lead in the deadline, world X minus joint X, packs 11 to 13 can be pruned (plan P0.3) | Confirmed in code | Fix each with a host test; hidden while only single cells run |
| G-C5 | Trigger sensor geometry is compile-time: `kTriggerSensorYFromCamMm`, `kTriggerCentreBehindMm` (plan P1.2) | Confirmed: `pick_scheduler.cpp:57,61`; breaks the stored-parameter rule | Add stored keys |
| G-C6 | `tau_band_s` assumes the object top is crossed while accelerating (plan P2.1) | Confirmed in `strike_plan.h:168`. Dormant while `acc_mm_s2` equals `dec_mm_s2` | Compute from the full profile; add a test with unequal ramps |
| G-C7 | Picks and console `move` share `spd_mm_s`, so jogs default to 750 mm/s (plan P0.1) | Confirmed. **The plan's fix adds separate `pk_*` speed keys, which contradicts the single-source rule** | **Decision:** a bench speed limit on `move`, or a deliberate exception |
| G-C8 | Duplicated logic remains between Gantry and scheduler: path timing, theta window, trapezoid math in four copies | Confirmed by a scan on 2026-10-30 | Remove when next touching that code |
| G-C9 | The per-pick dwell warning fires every pick while offsets differ (plan M3) | Suggested; console bursts are a known LAN trigger (G-E1) | Warn once per offset change |

### 5.4 Gripper and field I/O

SCHUNK KGG 100-80 pneumatic gripper, driven through an MCP23S17 output (PA0) and a relay-isolated 24 V valve, on SPI3 shared with the TFT. Air regulator at 6 bar. Code: `lib/MCP23S17`, `lib/Spi3Bus`, `GantryEndEffector`.

**Open items and blockers**

| ID | Item | Evidence | Next step |
|----|------|----------|-----------|
| G-D1 | Generic field I/O and ADC are deferred to hardware Rev02. The LV board schematic is in the Google Drive project folder under `Design Work\Network\Interface Module\WT32-ETH01-IM-REV02` | Confirmed in [DEV_TRACKER](DEV_TRACKER.md) | Plan with Rev02 |
| G-D2 | HV/LV schematic review is pending. The LV board schematic is at the same Google Drive path, `Design Work\Network\Interface Module\WT32-ETH01-IM-REV02` | Confirmed | Electrical review. Owner TBD |

### 5.5 Controller, cell network and tools

WT32-ETH01 (ESP32) running FreeRTOS at 1000 Hz. LAN8720 carries CellNet, the TCP console and OTA. Code: `src/main.cpp`, `lib/CellNetL2`, `lib/EthernetLink`, `src/gantry_test_console.cpp`, `src/gantry_ota.cpp`. Flash with `python tools/supervisor_ota.py`.

**Open items and blockers**

| ID | Item | Evidence | Next step |
|----|------|----------|-----------|
| G-E1 | LAN side can go silent while Class 1 keeps running, seen during `ztrace raw 500 250` (plan P1.1) | Confirmed incident: [STRIKE_TIMING](STRIKE_TIMING_MEASUREMENTS_2026-10-25.md). The plan's TBUS mechanism is Suggested | Avoid large dumps; root-cause the EMAC TX path |
| G-E2 | `ntp_server` pointer is kept by SNTP (plan P2.3) | Suggested. Key is reboot-only, so low risk | Copy to a static buffer |
| G-E3 | The `ota` "Compiled" stamp does not change on incremental builds, and the version string can read `-dirty` | Confirmed | Do not identify a build from it alone |
| G-E4 | Programming harness on IO0 fights the Ethernet clock | Confirmed, accepted | Unplug the harness to run plant Ethernet |

### 5.6 Runtime parameters

One complete set in NVS, loaded at boot. `cfg <key> <value>` changes RAM only; `cfg save` writes NVS. `sdkconfig.defaults` supplies compile-time defaults, used only by keys the set lacks or `cfg factory`. Code: `lib/AppConfig`. Reference: [LOW_LEVEL §14](LOW_LEVEL_GANTRY_CONTROL.md#14-runtime-parameters).

**Open items and blockers**

| ID | Item | Evidence | Next step |
|----|------|----------|-----------|
| G-F1 | `cfg factory` then `cfg save` installs defaults that do not match this machine: for example `col_z_min_mm` 115 against 122 live, `cv_x_off_mm` 245 against 307.5 | Confirmed from the live config | **Download parameters first.** Consider updating `sdkconfig.defaults` |
| G-F2 | About 68 stored keys have no reader, so `cfg` cannot change them (plan P2.2 counts 70) | Confirmed by name search | Wire them up or remove them |
| G-F3 | The parameter file leaves out the 31 developer-locked keys unless unlocked | Confirmed | Unlock before downloading a full backup |
| G-F4 | `x_lim_max_mm` also sets X scale at the next `calibrate x` (plan M2) | Suggested, consistent with `x_hard_span_mm` | Separate the soft limit from the span |

---

## 6. Vision

**Owned entirely by the Computer Science team.** Vision finds and classifies cells and sends `VISION_DETECT`.

The ML model is not ready yet. So that development does not stall, the Gantry team uses a workaround: a 1D sensor on the vision PC's Arduino fires one fixed 9V frame, the gantry takes X from `batt_x`, and all testing is 9V only. The workaround is not part of the vision system.

| Component | Detail | Location |
|-----------|--------|----------|
| Vision IPC | Ubuntu 22.04, `enp6s0` at `10.42.0.2`, SSH host alias `sorter`, time from the supervisor | Cell switch |
| Camera and ML model | Pointing down; +Z up, image +X up, +Y left, origin bottom-right, belt moves image-left | Computer Science team; model repo TBD |
| Trigger sensor (Gantry team workaround) | Sharp GP2Y0A21YK0F on an Arduino, 130 mm upstream of camera centre | On the rig |
| Frame sender (Gantry team workaround) | Reads the Arduino over serial and sends the frame | `~/vision_sim/simulate-vision-output.py` |

**Read:** [CELL_NET_L2_COMMUNICATION_GUIDE.md](CELL_NET_L2_COMMUNICATION_GUIDE.md).

### 6.1 Vision IPC

**Open items and blockers**

| ID | Item | Evidence | Next step |
|----|------|----------|-----------|
| V-1 | Vision code is not known to be in version control | Not checked: host unreachable from the dev PC on 2026-10-30 | Confirm, then put it in git |
| V-2 | SSH from the dev PC to `10.42.0.2` timed out on 2026-10-30, while the Pi still saw the host | Confirmed | Check the dev PC's route to the cell |

### 6.2 Camera and ML model

**Open items and blockers**

| ID | Item | Evidence | Next step |
|----|------|----------|-----------|
| V-3 | ML model not ready; blocks all multi-class and multi-cell testing | Confirmed | Computer Science team; date TBD |
| V-4 | Static camera-to-gantry transform is not done (on hold) | Confirmed | Resume with V-3 |
| V-5 | Angle reference disagrees with the wire contract (same as G-C3) | Confirmed | Agree one definition with the Computer Science team, and fix the header comment |

### 6.3 Trigger sensor workaround (Gantry team)

Chain latency is set in `cv_vis_ms`, measured with a Saleae capture: about 36 ms with the sensor delay. Rig offsets sit in code (G-C5).

**Open items and blockers**

| ID | Item | Evidence | Next step |
|----|------|----------|-----------|
| V-6 | Arduino sketch location unknown | Confirmed: not found in an earlier search | Find it and put it in git |
| V-7 | Sensor timing varies, and the 6 mm centre offset depends on battery pose | Confirmed: 38 ms measurement period, chain jitter about ±10 ms | Replaced by V-3 |

---

## 7. Conveyor

Carries cells past the gantry and reports belt position. The gantry treats `displacement_m` as the belt position; `speed_mm_s` only projects between samples. Recent running speed is about 290 mm/s.

| Component | Detail | Location |
|-----------|--------|----------|
| Belt and drive | Pot-based motor driver: speed is set by hand, to any value | On the rig |
| Encoder | 500 PPR quadrature (`500 * 4` counts), GPIO36 and GPIO39 | Conveyor WT32 |
| Conveyor node | WT32-ETH01, ESP-IDF, CellNet id `0x02` | [wt32-eth01-velmeter](https://github.com/Mynderse-Research-Lab/wt32-eth01-velmeter) |
| Display | 16x2 I2C LCD, PCF8574 at `0x27` | Conveyor WT32 |
| OTA | HTTP on `:8080`, `tools/ota_update.py` (`--zt` goes through the Pi) | Conveyor WT32 |

**Read:** that repository's `README.md`, `docs/architecture.md`, `docs/hardware.md`.

### 7.1 Conveyor node firmware

**Open items and blockers**

| ID | Item | Evidence | Next step |
|----|------|----------|-----------|
| C-1 | The firmware is at [wt32-eth01-velmeter](https://github.com/Mynderse-Research-Lab/wt32-eth01-velmeter). The copy at `D:\Projects\wt32-eth01-velmeter-testing` is not a git checkout, and C-2 and C-3 were read from it | Confirmed | Work from a clone; check C-2 and C-3 against `main` |
| C-2 | Address conflict: the project and the Pi's `NETWORK.md` give the conveyor `10.42.0.2`, which is the vision host. An unidentified device sits at `10.42.0.50` | Confirmed conflict; `.50` not identified | Identify the node's real address, then fix the docs |
| C-3 | Its protocol header differs from the gantry's (see X-3) | Confirmed | Sync, and check the wire format still matches |

### 7.2 Belt, drive and encoder

**Open items and blockers**

| ID | Item | Evidence | Next step |
|----|------|----------|-----------|
| C-4 | Encoder scale (mm per count) and belt hardware are undocumented here | Confirmed gap | Fill in the table above |
| C-5 | The conveyor must act on `CELL_COMMAND` in future: integrated conveyor control is needed instead of manual pot-based control | Confirmed requirement. Current firmware not checked | Design start, stop and speed commands |
| C-6 | Belt speed can be set to any value, but only by hand through the pot-based motor driver. It needs closed-loop control | Confirmed requirement | Replace with closed-loop speed control, driven from C-5 |

---

## 8. Supervisor

Raspberry Pi, CellNet node `0x04`. Sends heartbeats and `CELL_COMMAND`, records traffic, runs the dashboard and developer toolbox, relays the gantry console, runs OTA uploads, and serves time.

| Component | Detail | Location |
|-----------|--------|----------|
| Host | Debian 13. `eth0` `10.42.0.1`, ZeroTier `10.168.40.1`, Wi-Fi to a hotspot | Pi |
| `adsb-supervisor` | C++ service. Config in `/etc/adsb-cell.env` | Source `~/ADSB-PI-base/src`, binary `/usr/local/bin` |
| `adsb-cell-dhcp` | DHCP on the cell switch | Pi service |
| `chrony` | NTP server for `10.42.0.0/24`; replaced `systemd-timesyncd` on 2026-10-30 | `/etc/chrony/conf.d/gantry-cell.conf` |
| Web UI | `index.html` dashboard, `dev.html` toolbox: console, motion, offsets, parameter download, upload and save, firmware | `/usr/local/share/adsb-cell`, source `~/ADSB-PI-base/web` |

Static web files take effect without a restart. Build the service with `scripts/build.sh`.

**Read:** on the Pi, `README.md`, `docs/NETWORK.md`, `docs/INTEGRATION_WT32.md`.

### 8.1 Supervisor service

**Open items and blockers**

| ID | Item | Evidence | Next step |
|----|------|----------|-----------|
| S-1 | **Live source has no git repository on the Pi.** `D:\Projects\ADSB-PI-base` is an old copy | Confirmed | Give it its own repository; it must stay separate from this one |
| S-2 | `docs/INTEGRATION_WT32.md` says the scheduler does not consume `CELL_COMMAND`, but it does | Confirmed: `applyCellCommand` | Update the doc |
| S-3 | A console password appears in `cell.env.example` and in firmware defaults | Confirmed | Remove from the example, rotate it, and use the env file only |

### 8.2 Web UI

**Open items and blockers**

| ID | Item | Evidence | Next step |
|----|------|----------|-----------|
| S-4 | Download, Upload and Save to NVS have not been clicked in a browser against the gantry | Confirmed: only `cfg export` and `cfg save` were run, from a script | Try each once with motors off |

### 8.3 Production HMI

**Open items and blockers**

| ID | Item | Evidence | Next step |
|----|------|----------|-----------|
| S-5 | `sorting-cell-coordinator` overlaps the running supervisor and describes a Zenoh-era design. It was last touched 2026-08-20 | Confirmed, not deployed | **Decision:** adopt, merge or retire. Same for the [roadmap](DEVELOPMENT_ROADMAP.md)'s Zenoh plan |

---

## 9. Repository and documentation

**Open items and blockers**

| ID | Item | Evidence | Next step |
|----|------|----------|-----------|
| X-1 | About 47 commits on `feat/conveyor-traffic-debug` are unpushed, and `main` has 9 others | Confirmed | Push; expect a conflict in `PICK_SCHEDULER_KINEMATICS.md` |
| X-2 | **Uncommitted doc edits describe fixes that are not in the code.** `DEV_TRACKER.md` lists `pk_app_spd_mm_s`, `cv_trig_y_mm` and the follower fixes as done; other edits describe a theta mapping at ingestion | Confirmed: none are in `src`, `lib` or `include` | Review before committing. Trust the code |
| X-3 | Three copies of `cell_net_l2_protocol.h`, and the conveyor's differs from the gantry's | Confirmed | Pick one source; the Pi has `scripts/sync-cellnet.py` |
| X-4 | Stale or broken links: `SERIAL_OVER_LAN_PLAN.md`, `MQTT_comms_subsys.md`, spaces in schematic links, `file:///` URIs. The roadmap still says the gripper is not fitted, and `DEV_TRACKER.md` and the assembly doc still list a conveyor-height sensor as needed | Suggested in the documentation plan; the gripper and sensor lines are Confirmed | Apply the plan's link fixes |
| X-5 | Scratch files and logs clutter the repo root, some tracked | Confirmed | Remove from tracking and ignore |

---

## Appendix A: Blank component block

Copy this for a new component or a second gantry.

```markdown
### N.N Component name

What it does, in one or two sentences.

| Item | Detail |
|------|--------|
| Hardware | Part numbers |
| Source | Repo, path |
| Interfaces | What it talks to, over what |
| Build / flash | Commands |
| Verify | What healthy looks like |
| Owner | Name |
| Docs | Links |

**Open items and blockers**

| ID | Item | Evidence | Next step |
|----|------|----------|-----------|
| X-n | What is wrong or missing | Confirmed / Suggested / Decision | Who does what next |
```

## Appendix B: Suggested plans since 2026-10-29

These AI-generated plans sit in the Antigravity `brain` folder on the lead's PC. **They are suggestions, not approved work.** Line numbers in them drift, and none of their fixes had been applied at `d6e8f8e`.

| Plan | Date | Use it for | Caution |
|------|------|------------|---------|
| `implementation_plan.md` | 10-29 | Strike timing and queueing background | Never approved; partly stale |
| `codebase_status_plan.md` | 10-30 | Risk register and workstreams | Written before `cfg save` |
| `recent_commits_review.md` | 10-30 | Findings H1 to L5 | Predates the later commits |
| `codebase_deep_review_and_open_items.md` | 10-30 | The P0 to P3 register | P0.1 conflicts with the single-source rule |
| `p0_p1_remediation_plan.md` | 10-30 | Worked maths and proposed code | Same conflict; its theta formula differs from the review's, and the frame question (G-C3) is still open |
| `categorical_documentation_plan.md` | 10-30 | Six-pillar doc structure, reader journeys | Applies to `INDEX.md` and `README.md` |

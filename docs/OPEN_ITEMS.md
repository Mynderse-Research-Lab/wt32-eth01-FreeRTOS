# Open Items and Blockers

**Status:** Live list. Reviewed against the code at `d11ebe2`, and checked on the cell from the Pi, on 2026-11-05.
**Companion:** [KNOWLEDGE_TRANSFER.md](KNOWLEDGE_TRANSFER.md) says what each subsystem is and how the pieces connect. This page tracks what is unfinished.

## How to keep this list live

1. Change an item in the same commit as the work that changes it: new evidence, a decision, a fix.
2. Give a new item the next free ID in its group. IDs are never reused, so old references stay valid.
3. To close an item, move it to [Closed items](#closed-items) with the date, the commit and the evidence. Do not delete it.
4. Keep [Blockers and decisions](#blockers-and-decisions) in step with the rows below it.
5. If closing an item teaches a lasting rule, add the rule to the [working rules](KNOWLEDGE_TRANSFER.md#machine-safety-and-working-rules) in the Knowledge Transfer page.
6. After checking the whole list against the code, update the status line.

Every item carries an evidence label. **Do not treat a label lower than Confirmed as fact.**

| Label | Meaning |
|-------|---------|
| **Confirmed** | Checked against code, a live log or a measurement. |
| **Suggested** | From an AI-generated plan and not yet checked. Treat it as a lead. |
| **Decision** | Needs a call from the project lead before anyone starts. |

Items with `(plan)` also appear in the [suggested plans](#suggested-plans-since-2026-10-29). `DEV_TRACKER.md`, cited below, is not in git (X-2).

## Blockers and decisions

These items stop other work, or need a call before anyone starts. The full rows are in the sections below.

| ID | Waiting on | Holds up |
|----|------------|----------|
| [V-3](#vision) | The ML model, from the Computer Science team | All multi-class and multi-cell testing, the camera transform (V-4), and retiring the trigger workaround (V-7) |
| [G-C3](#g-c-pick-scheduler), [V-5](#vision) | **Decision:** one angle reference, agreed with the Computer Science team | Theta from real vision frames. Today `cv_jaw_deg` compensates on the bench |
| [G-C2](#g-c-pick-scheduler) | **Decision:** is a bottom dwell acceptable now? | Finishing the tuning (G-C1), and with it the detection-rate work (G-C10) |
| [G-C7](#g-c-pick-scheduler) | **Decision:** a bench speed limit on `move`, or a deliberate exception | Slow console jogs while picks run at pick speed |
| [G-C11](#g-c-pick-scheduler) | **Decision:** the 15 ms depth cap or the 2 mm entry margin? | Picks of cells lying across the belt (0–35°) |
| [G-C13](#g-c-pick-scheduler) | **Decision:** latency fallback for all frames, or the trigger stand-in only? | Wiring the real camera's frames |
| [G-B7](#g-b-motion-bus-w5500-ethernetip) | **Decision:** does the strike timing need a 2 ms RPI? | A wider Class 1 timeout margin |
| [G-D3](#g-d-gripper-field-io-and-panel) | **Decision:** approve the SPI3 priority ceiling | Ending the MCP23S17 lock failures. The gripper write takes the same lock |
| [C-2](#conveyor) | A conveyor firmware change, flashed over USB (it can carry the N-2 receiver too) | Any IP access to the conveyor, OTA included: today it has no address |
| [X-1](#repository-and-documentation) | A push of this branch | Anyone else building on its work since 2026-09-18 |

## Gantry

### G-A Axes and mechanics

| ID | Item | Evidence | Next step |
|----|------|----------|-----------|
| G-A1 | Multiple gantries are planned. Preferred: one gantry manages 2 classes, dropping one class left and the other right. Fallback: one gantry for each class (9V, laptop, li-ion pack, powertool battery) | Confirmed plan. Not supported yet: one `GANTRY` node id, one `gantry1` status field, one bin pose | Add node ids and per-class drop poses; start with two-sided drop |
| G-A2 | Z continuous torque is about 0.95× the SCHUNK 1.35 N·m requirement | Confirmed in `DEV_TRACKER.md`, not re-measured | Review before production duty. The tracker's options: derate the profile, or move to E1007 |
| G-A4 | Why Z reads about −42 instead of 48 after a reboot, before `calibrate all` | Confirmed reading; a physical drop was not ruled out | Observe Z across a reboot |

### G-B Motion bus (W5500, EtherNet/IP)

Background for these items, with the specification references, is in [§5.2 of the Knowledge Transfer page](KNOWLEDGE_TRANSFER.md#52-motion-bus-w5500-ethernetip).

| ID | Item | Evidence | Next step |
|----|------|----------|-----------|
| G-B1 | `cfg save` closes Class 1 cleanly on X and Z, including rebuilding dropped sessions. Whether theta stays clear after a dropped-session close is still unknown | Confirmed 2026-10-30 at 723 s: X got no reply for 500 ms, then X and Z reconnected and the close was accepted, with no `E602`. Theta was clear after the 60 s save, but already latched at 723 s (G-B6) | Repeat with the alarm clear first, and only one session on the gantry |
| G-B2 | Kinetix explicit TCP sessions die while Class 1 keeps running. Sometimes the socket shows `CLOSE_WAIT`; on 2026-10-30 the send succeeded and no reply came. A status check alone does not catch it, because a broken connection only shows up on a send (Vol 2, §2-5.5.1, p. 2-21) | Drop confirmed in logs. Spec-level cause confirmed: Class 1 is not encapsulation activity (Vol 2, §2-5.5.2), and we send none after Forward Open. The timing fits, not yet proven: the save at 60 s was under the 120 s default and closed cleanly, and the save at 723 s found X dropped. Theta's session sits equally idle and survives, which this does not explain | Fixed by the NOP keepalive (`379b4ac`), one run so far. On 2026-11-03 Class 1 ran 538 s from boot, then `cfg save` closed X, Z and theta each with one Forward_Close on its original session: no rebuild, no `E602`, theta clear, no `NOP failed`, and no Class 1 gap over the 6 ms report threshold after the first 15 s. Theta lacking attribute 13 likely explains why its session always survived (assumed: unsupported attribute taken to mean no timeout). Repeat on a longer run before closing the item. To see which side ends the session, capture with Wireshark through a port-mirroring switch (ODVA PUB00213, p. 20) |
| G-B3 | Class 1 gaps of 6 to 20 ms with no `cfg` write nearby | Confirmed. None of 6 ms or more in the 45 min G-B6 soak, then one on 2026-11-03 on `51f7528`: 20 ms at 774 s, inside the exchange, 11.9 ms of it in the receive drain. Unlike G-B6, the scanner was not blocked. The exchange takes the same bus mutex, so G-B6's preempted `take()` may explain the earlier gaps (likely, not traced) | Below the 32 ms timeout, but the worst left 12 ms of margin. Read `gaptrace` after the next one. Measure packet intervals at a network tap, as in [ODVA's performance tests](../driver_datasheets_and_calculations/EtherNetIP%20Interoperabiltiy%20Recommendations.pdf.pdf), pp. 3 to 5 |
| G-B4 | The Class 1 cadence gate from `eiptiming` is not met | Confirmed 2026-08-18 on image `0x9c6c0`, recorded in `DEV_TRACKER.md`: cycle min, p50 and p99 of 1342, 1999 and 2659 µs, against a gate of min at least about 1900 µs and p99 at most about 2100 µs; `soft_miss=0` | Re-measure on the current image |
| G-B5 | Target IPs, RPI and PUU scales still come from Kconfig, so those `cfg` keys do nothing (plan P1.3) | Confirmed. Left on purpose: stored values are developer-locked and a wrong one changes the Class 1 target | Read the stored values first, then route them |
| G-B7 | We run the Kinetix at its 2.0 ms minimum RPI. Rockwell recommends 20 ms for this drive, as it is a simple I/O device rather than an integrated-motion drive | Confirmed: manual p. 197 | **Decision:** does the strike timing need 2 ms? A slower RPI would widen the timeout margin |
| G-B8 | Class 1 sequence handling differs from Vol 2, §3-4.1: `EipIoConnection::resetSequences()` starts the encapsulation sequence at 0 rather than a random value, and the T->O parser (`EipCpf.cpp:151`) reads the connection ID and ignores the sequence number, so nothing is discarded | Confirmed in code. No fault traced to it. A stall does not skip our O->T numbers, so the gap rule should not bite on a healthy link (assumed; reading the drive's Connection Manager counters would confirm it) | Low priority. Fix alongside the next Class 1 change |
| G-B9 | The `include_run_idle_bit_in_net_params` option sets bit 8 of the Forward_Open network parameters, believing it a Run/Idle flag. Bit 8 belongs to Connection Size (Vol 1, Table 3-5.8, p. 3-57), and no such flag exists | Confirmed in code and spec. Off in production (`EipScannerPolicy.h:89`), so no live effect. It would add 256 bytes to the declared size, which would explain why the Kinetix appeared to reject it (assumed, not tested) | Low priority. Remove the option and its test |
| G-B10 | A CellNet flood on the LAN8720 stalled Class 1 once for 14.9 ms. Theta's T->O went stale and the scanner reconnected; theta's F4009 stayed latched, as before the test | Confirmed 2026-11-13, during the N-2 receive-rate sweep (motors off). The `gaptrace` report gives the gap at 120,736 ms of uptime, which by step timing falls at the 1,014-byte 6,000 to 8,000 frames/s steps. 14,580 µs of it was inside the W5500 exchange, which runs on core 1. Over the surrounding 144 ms, `emac_rx` used 49 ms of core 0. The report keeps only the latest gap. It counted 2 gaps of 10 ms or more since boot, and when the first one happened is not known. Why a core-0 receive load lengthens a core-1 exchange is not known. One unverified lead is a W5500 bus mutex holder on core 0 preempted by `emac_rx` (priority 15); a similar mechanism was the cause of G-B6 | Not urgent: cell traffic today is tens of frames per second. Before CellNet carries bulk data during operation (N-1), reproduce the stall with motors off, one rate at a time, reading `gaptrace` after each step,, and find what the exchange waits for. Do not add recovery (Class 1 policy) |

### G-C Pick scheduler

| ID | Item | Evidence | Next step |
|----|------|----------|-----------|
| G-C1 | Tuning: picks lift, but many grip only half the 9V cell | Confirmed 2026-10-30: 7 of 9 overhanging; offsets were still being tuned | Finish offset tuning; run `cfg export` for current values |
| G-C2 | Z reverses short of grasp depth: about 1.15 mm at 5000 mm/s² and zero dwell, past the accepted 1 mm | Confirmed: `pick actual` logs | Offsets of −60 / +10 reached 0.1 to 0.4 mm; **Decision:** is bottom dwell acceptable now? |
| G-C3 | Vision angle frame is unresolved. The wire comment says 0° is along +Y; the sorter script measures from camera +X; `cv_jaw_deg` = 90 compensates on the bench (plan P0.2) | Confirmed conflict; the plan's transform is unverified | **Decision:** fix the reference with the Computer Science team, then convert once. Verify before reusing the plan's formula |
| G-C4 | `classifyFollowers()`: 190 ms grip dwell in ready time, no strike lead in the deadline, world X minus joint X, packs 11 to 13 can be pruned (plan P0.3) | Confirmed in code | Fix each with a host test; hidden while only single cells run |
| G-C5 | Trigger sensor geometry is compile-time: `kTriggerSensorYFromCamMm`, `kTriggerCentreBehindMm` (plan P1.2) | Confirmed: `pick_scheduler.cpp:57,61`; breaks the stored-parameter rule | Add stored keys |
| G-C6 | `tau_band_s` assumes the object top is crossed while accelerating (plan P2.1) | Confirmed in `strike_plan.h:168`, and not dormant with equal ramps: it is wrong whenever the top lies past mid-plunge. For the bench 9V it is 4.4 ms long at today's 48.4 ceiling (8.4 ms at 35.7). It changes the plan only for cell angles of 33–43° (300 mm/s); at 90°, every bench pick so far, it cancels out (KT 5.3.9) | Compute it from the full profile (`timeAlongProfile`); add a test with the top past mid-plunge |
| G-C7 | Picks and console `move` share `spd_mm_s`, so jogs default to 750 mm/s (plan P0.1) | Confirmed. **The plan's fix adds separate `pk_*` speed keys, which contradicts the single-source rule** | **Decision:** a bench speed limit on `move`, or a deliberate exception |
| G-C8 | Duplicated logic remains between Gantry and scheduler: path timing, theta window, trapezoid math in four copies. `pick_intercept.h` also keeps an earlier EarlyZ/LateZ strike model (`planPick`, `strikeLeads`, `retimePick`) that only the host tests call | Confirmed by a scan on 2026-10-30; the unused model confirmed 2026-11-05 | Remove when next touching that code |
| G-C9 | The per-pick dwell warning fires every pick while offsets differ (plan M3) | Suggested; console bursts are a known LAN trigger (G-E1) | Warn once per offset change |
| G-C10 | Detection rate. Demonstrations space vision detections at least 7 s apart so the unoptimized planner completes each pick; the goal is several detections per second | Decision 2026-11-03: a starting point, not a limit | An optimization problem: measure the pick cycle once G-C1 and G-C2 are closed, then shorten the spacing step by step |
| G-C11 | For cells within 35° of across the belt, the 15 ms depth cap in `StrikePlan::plan` overrides the 2 mm entry margin. At 0° and 300 mm/s the commanded fingertips pass the cell top while the cell is still 1.8 mm short of the upstream pad face | Confirmed by running the code with today's defaults (KT 5.3.9). Not exercised: every bench pick so far had the length along the belt (90°). The drive's 30–35 ms lag probably hides it (assumed) | **Decision:** which constraint wins. Then test a 0° strike at belt speed |
| G-C13 | The vision contract says capture displacement is required and a latency estimate is no substitute (`pick_intercept.h:218`), but `acceptDetect` accepts any frame without it whenever `cv_vis_ms` > 0, the default (`pick_scheduler.cpp:1151`) | Confirmed in code. Needed today: the trigger stand-in sends no capture displacement | **Decision:** keep the latency fallback only for the stand-in (`vis_batt_x`), or allow it for all frames and fix the comment |

### G-D Gripper, field I/O and panel

| ID | Item | Evidence | Next step |
|----|------|----------|-----------|
| G-D1 | Generic field I/O and ADC are deferred to hardware Rev02. The LV board schematic is in the Google Drive project folder under `Design Work\Network\Interface Module\WT32-ETH01-IM-REV02` | Confirmed in `DEV_TRACKER.md` | Plan with Rev02 |
| G-D2 | HV/LV schematic review is pending. The LV board schematic is at the same Google Drive path, `Design Work\Network\Interface Module\WT32-ETH01-IM-REV02` | Confirmed | Electrical review. Owner: Zia-Ahmed-1 (amohamm20@ltu.edu). Include whether `CB0` can be locked off, now that it is the first device on the supply (G-D4) |
| G-D3 | MCP23S17 accesses fail with `SPI mutex lock failed ... (mutex timeout)`. Waiters give up after 8 ms (`kMcpSpiLockTimeoutTicks`), and the gripper, the inputs and the health check all take this lock | Confirmed 2026-11-03 with holder logging (`2f91228`, `722c793`, `51f7528`): 3 lock failures in 45 min, and one in the soak after the revert, with no Class 1 effect. The holder is `TftUiTask` (priority 2, core 0), for up to 365 ms. Once `PickScheduler` held it for 131 ms, most likely waiting for the SPI3 bus itself. With the TFT task on core 1 (`3e8fc51`), the longest hold in the first 3.5 min was 30 ms, but Class 1 starved (gaps of 38 and 46 ms, one `E602` on X), so it was reverted (`d02f324`). That network work preempts the TFT mid-transfer is inferred from the hold pattern, not measured | **Decision:** a priority ceiling for SPI3. Raise any SPI3 or MCP23S17 holder to priority 19 for one transfer, and split the TFT fill into short chunks. First check the ST7789 datasheet on resuming a pixel write after a chip-select toggle. Then soak with the hold log, which reports holds over 20 ms |

### G-E Controller, cell network and tools

| ID | Item | Evidence | Next step |
|----|------|----------|-----------|
| G-E1 | LAN side can go silent while Class 1 keeps running, seen during `ztrace raw 500 250` (plan P1.1) | Confirmed incident: [STRIKE_TIMING](STRIKE_TIMING_MEASUREMENTS_2026-10-25.md). The plan's TBUS mechanism is Suggested | Avoid large dumps; root-cause the EMAC TX path |
| G-E2 | `ntp_server` pointer is kept by SNTP (plan P2.3) | Suggested. Key is reboot-only, so low risk | Copy to a static buffer |
| G-E3 | The `ota` "Compiled" stamp does not change on incremental builds, and the version string can read `-dirty` | Confirmed | Do not identify a build from it alone |
| G-E4 | Programming harness on IO0 fights the Ethernet clock | Confirmed, accepted | Unplug the harness to run plant Ethernet |

### G-F Runtime parameters

| ID | Item | Evidence | Next step |
|----|------|----------|-----------|
| G-F1 | `cfg factory` then `cfg save` installs defaults that do not match this machine: for example `col_z_min_mm` 115 against 122 live, `cv_x_off_mm` 245 against 307.5 | Confirmed from the live config | **Download parameters first.** Consider updating `sdkconfig.defaults` |
| G-F2 | About 68 stored keys have no reader, so `cfg` cannot change them (plan P2.2 counts 70) | Confirmed by name search | Wire them up or remove them |
| G-F3 | The parameter file leaves out the 31 developer-locked keys unless unlocked | Confirmed | Unlock before downloading a full backup |
| G-F4 | `x_lim_max_mm` also sets X scale at the next `calibrate x` (plan M2) | Suggested, consistent with `x_hard_span_mm` | Separate the soft limit from the span |
| G-F6 | The kept `main.cpp` (X-6) breaks the stored-parameter rule in `AGENTS.md`. It sets the joint limits and the safe-Z ceiling from compile-time macros (working-tree `main.cpp:341` and `:346`). It also erases NVS (`:180`), and with it the stored set, when `nvs_flash_init` reports no free pages or a new version | Confirmed in the working-tree file 2026-11-13. The committed `main.cpp` takes these from the stored set (`HEAD:src/main.cpp:423-432`, since `de366e1`) and never erases NVS on its own | Port the stored-set wiring from `HEAD` into the kept file, or accept the gap here |

## Vision

| ID | Item | Evidence | Next step |
|----|------|----------|-----------|
| V-1 | Vision code is not in version control on the vision IPC | Confirmed 2026-11-05: no git checkout for `UHVSorter` (1.3 GB, newest file 2025-03-21), `UHVSorter_yoloOBB_UDP_Stream` (2.1 GB, 2025-09-16) or `vision_sim`, the Gantry team's trigger workaround (`simulate-vision-output.py`, `cell_net_l2.py`). The only repositories there are two third-party libraries | Ask the Computer Science team where their code is kept. Put `vision_sim` in git |
| V-3 | ML model not ready; blocks all multi-class and multi-cell testing | Confirmed | Computer Science team; date TBD |
| V-4 | Static camera-to-gantry transform is not done (on hold) | Confirmed | Resume with V-3 |
| V-5 | Angle reference disagrees with the wire contract (same as G-C3) | Confirmed | Agree one definition with the Computer Science team, and fix the header comment |
| V-6 | Arduino sketch location unknown | Confirmed: not found in an earlier search, and not on the vision IPC (no `.ino` file, no Arduino folder, 2026-11-05) | Find it and put it in git |
| V-7 | Sensor timing varies, and the 6 mm centre offset depends on battery pose | Confirmed: 38 ms measurement period, chain jitter about ±10 ms | Replaced by V-3 |

## Conveyor

| ID | Item | Evidence | Next step |
|----|------|----------|-----------|
| C-1 | The firmware is at [wt32-eth01-velmeter](https://github.com/Mynderse-Research-Lab/wt32-eth01-velmeter). The copy at `D:\Projects\wt32-eth01-velmeter-testing` is not a git checkout, and C-2 and C-3 were read from it | Confirmed. It does not match the running build: it compiles a static `.2` with DHCP off, while the flashed conveyor runs DHCP (2026-11-05) | Work from a clone; check C-2 and C-3 against `main` |
| C-2 | The conveyor moves to `10.42.0.3` (decided 2026-11-03). Today it has no IP address at all. The flashed firmware asks the Pi's DHCP for one about 73 times an hour, is offered `.50` every time, and never takes it. The `.50` device seen earlier was most likely the conveyor (assumed) | Decision. Confirmed 2026-11-05 from the Pi: the `CONVEYOR_SPEED` frames come from `34:ab:95:7e:de:2b`, which sent 1825 DHCP discovers from 2026-11-04 04:10, when the log starts, with no `DHCPACK`. A ping sweep of `10.42.0.0/24` found only the vision IPC, which alone answers ARP for `.2`. `.3` is free. The supervisor's `docs/NETWORK.md` lists `.3` as planned (`b980b06`) | Give the conveyor a static `.3` with DHCP off, and flash it over USB: without an address it has no OTA. Confirm `.3` answers. Point the supervisor's `NETWORK.md` at this page; it still cites `ONBOARDING.md` |
| C-3 | Its protocol header differs from the gantry's | Confirmed 2026-11-05: the `wt32-eth01-velmeter-testing` copy differs from `include/cell_net_l2_protocol.h` in 30 lines | Sync it from the gantry's header, as the supervisor's `scripts/sync-cellnet.py` does, and check the wire format still matches |
| C-4 | Encoder scale (mm per count) and belt hardware are undocumented here | Confirmed gap | Fill in the conveyor table in the Knowledge Transfer page |
| C-5 | The conveyor must act on `CELL_COMMAND` in future: integrated conveyor control is needed instead of manual pot-based control | Confirmed requirement. Current firmware not checked | Design start, stop and speed commands |
| C-6 | Belt speed can be set to any value, but only by hand through the pot-based motor driver. It needs closed-loop control | Confirmed requirement | Replace with closed-loop speed control, driven from C-5 |

## Supervisor

| ID | Item | Evidence | Next step |
|----|------|----------|-----------|
| S-4 | Download, Upload and Save to NVS have not been clicked in a browser against the gantry | Confirmed: only `cfg export` and `cfg save` were run, from a script. On 2026-11-13 how far the browser test got was not known, and the supervisor does not log these buttons | Try each once with motors off, and note the result here |

## Cell transport

| ID | Item | Evidence | Next step |
|----|------|----------|-----------|
| N-1 | CellNet L2 becomes the only transport protocol in the cell. The IP services between nodes move onto it: console (TCP `:2323`), gantry OTA (TCP `:8032`), conveyor OTA (HTTP `:8080`) and time (NTP `:123`). The supervisor web application stays on HTTP for engineers | Decision 2026-11-05 | Migrate one service at a time, OTA first (N-2) |
| N-2 | OTA over CellNet. A node reboots into an OTA-only mode, where the update is its only task, so no queue or task priorities are needed. No authentication (research setup). Larger frames and resends are worth adding only if the whole update is faster than today's TCP OTA | Decision 2026-11-05. **TCP baseline, confirmed 2026-11-13** over five web OTAs through the supervisor. The images were the running 843,360-byte image (built Nov 3 17:21:54) and timing builds of it. With the receiver erasing one 4 KB sector per write, the gantry leg (`OK READY` to `OK COMPLETE`) took 14.9 to 16.2 s. Flash set that rate: 96 to 97 % of the stream was spent inside `esp_ota_write`, against under 0.6 s waiting in `recv()`. **Erasing the image range up front (`esp_ota_begin` with the `START` size, 64 KB block erases) cut the whole update to 6.4 and 6.6 s on the gantry** in two runs. The erase took 2.8 and 2.9 s before `OK READY`, the stream 3.0 and 3.1 s (of which writing 2.8 s, `recv()` 0.2 to 0.3 s), and the image check 0.6 s. One run went through the supervisor: `OK READY` came after about 3 s, inside its 8 s wait. From upload to the gantry answering the console took 16.0 s, against 25.6 s before. The receiver reports these figures in `OK COMPLETE` (`gantry_ota.cpp`). Flash operations also pause the network on both cores, so `recv()` time shows that the sender stays ahead, not how fast TCP itself could go. Today `parseFrame` drops frames over 64 bytes (`CellNetL2Framing.cpp:93`), counted after ESP-IDF strips the FCS (`esp_eth_mac_esp_dma.c:335`) **CellNet receive rate, confirmed 2026-11-13** (`netstat` console counters; the Pi sent test frames of an unassigned type for 2 s per step, with motors off and EtherNet/IP running): 60-byte frames arrived with 0 to 0.16 % loss up to 30,000 frames/s. The ceiling was about 36,700 frames/s, and above it the excess was lost to the EMAC's missed-frame counter (no free receive buffer). 1,014-byte frames arrived with 0 to 0.31 % loss up to 10,000 frames/s, about 10 MB/s. The EMAC missed-frame counter accounts for the losses at every rate; its 16-bit count rolled over once, at 80,000 frames/s. The flood also stalled Class 1 once (G-B10). | The pre-erase receiver is committed. **CellNet must beat about 6.5 s, and the gantry can receive several times the 285 kB/s that needs:** about 6,800 frames/s of 64-byte frames, against at least 30,000 measured. Frame rate is not the limit; three design constraints are. (1) Some frames are lost at every rate above 2,000 frames/s, so resends are needed for correctness. (2) Flash writes pause the receive task, while the EMAC holds only 10 buffers of 512 bytes (`CONFIG_ETH_DMA_RX_BUFFER_NUM`, `CONFIG_ETH_DMA_BUFFER_SIZE`). A 4 KB write takes about 13.5 ms, during which about 92 small frames would arrive, so the sender needs flow control, such as an acknowledgement per 4 KB block. (3) Five 1,014-byte frames fit in those 10 buffers, so with large frames the next block could arrive during a write, an estimate derived from these numbers. Design the block protocol, then compare its whole update with 6.5 s. A bigger image erases longer before `OK READY`: the full 1,920 KB slot would take about 6.7 s, near the supervisor's 8 s wait. The conveyor needs one USB flash before it can receive CellNet OTA (C-2) |

## Repository and documentation

| ID | Item | Evidence | Next step |
|----|------|----------|-----------|
| X-1 | `feat/conveyor-traffic-debug` is unpushed past `74be86b` (2026-09-18), the `upstream` tip: 75 commits up to `48f2c57`. `main` has 9 commits this branch lacks | Confirmed 2026-11-05 against `upstream` | Push; expect a conflict in `PICK_SCHEDULER_KINEMATICS.md` |
| X-2 | **Uncommitted doc edits describe fixes that are not in the code.** `DEV_TRACKER.md` lists `pk_app_spd_mm_s`, `cv_trig_y_mm` and the follower fixes as done. Four more docs carry uncommitted edits (kinematics, pickup, protocol, firmware review), and some of these describe a theta mapping at ingestion. `DEV_TRACKER.md` itself has never been committed, yet `AGENTS.md` and `INDEX.md` list it | Confirmed 2026-11-05: none of those keys are in `src`, `lib`, `include` or `sdkconfig.defaults`, and git has no history for the tracker | Review before committing. Trust the code. Then commit the tracker, or drop it from `AGENTS.md` and `INDEX.md` |
| X-4 | Stale or broken links and claims. `LOW_LEVEL_GANTRY_CONTROL.md` links the missing `MQTT_comms_subsys.md`. `FIRMWARE_REVIEW_AND_OPTIMIZATIONS.md` has `file:///` URIs. The [roadmap](DEVELOPMENT_ROADMAP.md) says the gripper is not assembled and still plans the Zenoh migration retired by S-5. `EXPECTED_ELECTROMECHANICAL_ASSEMBLY.md` still lists a conveyor-height sensor as future work, against G-A3's closing decision. The documentation plan also flags spaces in schematic links | Confirmed 2026-11-05, except the schematic links, which are Suggested | Apply the plan's link fixes, and correct the roadmap and assembly lines |
| X-5 | Scratch files and logs clutter the repo root, some tracked | Confirmed 2026-11-05: `fix_math.py` to `fix_math3.py`, `pktmon.txt` and a screenshot are tracked at the root, and `lib/CellNetL2/Untitled` is untracked | Remove from tracking and ignore |
| X-6 | **The gantry is built from an outdated, uncommitted `src/main.cpp`, kept by decision.** The working-tree file is an older copy: 33 lines added and 132 removed against `HEAD`. It never installs the boot log, and it erases NVS when NVS will not open. It takes the joint limits and safe Z from compile-time macros (G-F6), and it fixes the update period at 10 ms instead of using `GANTRY_UPDATE_PERIOD_MS`. Git does not hold this source | Confirmed 2026-11-13: the file differs as stated. The running image reports `51f7528-d` (dirty), and its `bootlog` captured 0 bytes since reset, which fits this file. A build from the working tree, with the OTA timing code added, came out only 496 bytes larger. **Decision 2026-11-13: keep the working-tree file**; firmware is built from it | Keep a copy outside the working tree, or commit it, so a checkout or stash cannot lose the running firmware's source |

## Closed items

| ID | Item | Closed | Evidence |
|----|------|--------|----------|
| G-A3 | Conveyor-height sensor on the machine but not wired | 2026-10-31, `a4811b9`, by decision | No conveyor-height sensor will be used. `col_x_min_mm` and `col_z_min_mm` set the conveyor region |
| G-B6 | Class 1 gaps of 30 to 90 ms, outside the exchange, dropped theta or X and Z with the motors off | 2026-11-03, `7ee74f3` | Cause: `BusMutex::take()` locked the W5500 bus mutex to read its statistics, every 100 ms from the priority-2 `EipLog` task on core 0. Preempted there by network work (`tcpip`, priority 18), it kept the scanner blocked with core 1 idle; its own hold was never recorded, so reports showed a long wait with no holder. Measured: `take:EipLog` held the mutex 29.6 ms while the scanner waited 28.5 ms in a 31 ms gap. Fix: the statistics have their own spinlock, and `take()` never touches the bus mutex. Before the fix, 3 gaps in 30 min under console load (`cfg export` every 10 s plus `status`); after it, none in 45 min of the same load (1830 commands), no loop gap of 6 ms or more and no bus-mutex wait of 3 ms or more. The lasting rule is in the working rules |
| G-D4 | The panel BOM lost its lockable main disconnect, `DISC1`, in `5bfbe1d` (2026-08-21) | 2026-11-05, by decision | Intentional: the three phases go straight to `CB0`, the 40 A 3-pole main MCCB (Allen-Bradley 140UT-D7D3-C40) at the 240 V three-phase panel input. The BOM lists no lock-off for `CB0`; G-D2 checks it |
| V-2 | SSH from the dev PC to `10.42.0.2` timed out on 2026-10-30 | 2026-11-05 | From the cell network, through the Pi, the vision IPC answered on port 22 and accepted the `sorter` key |
| S-3 | The console and OTA password is compile-time only, with hard-coded fallbacks in four source files | 2026-11-05, by decision | No security is kept beyond the supervisor web application in this research setup |
| G-C12 | `zPlan`'s comment gave only the half-height grasp rule, missing the 3 mm rule for cells under 30 mm | 2026-11-13 | The comment now states both rules (`pick_scheduler.cpp:692`). The 3, 30 and 25.4 mm stay literals |
| S-5 | `sorting-cell-coordinator` overlapped the running supervisor with a Zenoh-era design | 2026-11-13, by decision | Retired, with the roadmap's Zenoh plan. The running supervisor stays the HMI, and CellNet L2 is the cell transport (N-1) |
| S-1 | The live supervisor source had no git repository | 2026-11-03 | [ADSB_Sorter-supervisor](https://github.com/Mynderse-Research-Lab/ADSB_Sorter-supervisor) (private) holds it, checked 2026-11-05 at `b980b06`. The Pi's `~/ADSB-PI-base` is a deployed copy, and each deploy is logged in that repository's `docs/DEPLOYED_STATE.md` |
| S-2 | `docs/INTEGRATION_WT32.md` said the scheduler does not consume `CELL_COMMAND` | 2026-11-03, supervisor `b980b06` | The rewritten doc says the gantry accepts `CELL_COMMAND` only from node `0x04`, checked 2026-11-05 |
| G-F5 | The safe-Z ceiling was said to come from the compile-time `GANTRY_SAFE_Z_HEIGHT_MM`, with the stored `geom_safe_z_mm` unread | 2026-11-13, not a defect in the committed code | The claim was read from the stale uncommitted `src/main.cpp` (X-6). The committed `main.cpp:432` applies the stored `geom_safe_z_mm` at boot, and the scheduler reads it through `traverseClearanceZMm()` (`pick_scheduler.cpp:676`). The kept working-tree `main.cpp` the gantry is built from still uses the macro (G-F6) |
| X-3 | Three copies of `cell_net_l2_protocol.h`, and the conveyor's differs from the gantry's | 2026-11-03 | This repository is the source. The supervisor's `vendor/cellnet` is refreshed from it by `scripts/sync-cellnet.py`, last from `7ee74f3`, and matched `HEAD` on 2026-11-05. The conveyor's copy still differs, tracked as C-3 |

## Suggested plans since 2026-10-29

These AI-generated plans sit in the Antigravity `brain` folder on the lead's PC. **They are suggestions, not approved work.** Line numbers in them drift. None of their fixes had been applied at `d11ebe2`: the pick scheduler, strike plan, intercept, `AppConfig` and `main.cpp` sources are unchanged since `d6e8f8e`.

| Plan | Date | Use it for | Caution |
|------|------|------------|---------|
| `implementation_plan.md` | 10-29 | Strike timing and queueing background | Never approved; partly stale |
| `codebase_status_plan.md` | 10-30 | Risk register and workstreams | Written before `cfg save` |
| `recent_commits_review.md` | 10-30 | Findings H1 to L5 | Predates the later commits |
| `codebase_deep_review_and_open_items.md` | 10-30 | The P0 to P3 register | P0.1 conflicts with the single-source rule |
| `p0_p1_remediation_plan.md` | 10-30 | Worked maths and proposed code | Same conflict; its theta formula differs from the review's, and the frame question (G-C3) is still open |
| `categorical_documentation_plan.md` | 10-30 | Six-pillar doc structure, reader journeys | Applies to `INDEX.md` and `README.md` |

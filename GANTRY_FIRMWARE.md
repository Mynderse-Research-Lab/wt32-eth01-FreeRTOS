# Gantry firmware

*[README](README.md) › [Knowledge transfer](docs/KNOWLEDGE_TRANSFER.md) ›
Gantry firmware*

**Read this when** you need to know how the gantry controller's firmware fits
together: what runs, in what order, and how a move or a pick flows through it.
Each library's own facts are in its README and REFERENCE under `lib/`. This page
explains how they work together and documents the application code in `src/`.

## 1. What runs on the controller

The WT32-ETH01 runs one ESP-IDF application. The code in `src/` (boot, tasks,
the test console and the pick scheduler) sits on thirteen libraries in `lib/`:

```text
src/   boot · tasks · test console · pick scheduler
 ├─ Gantry ─────────── motion API: paths, homing, gripper; Kinetix X/Z and HCS01 theta axes
 │   └─ EtherNetIP ─── Class 1 scanner, explicit messages ─── W5500 (SPI2) ─── drives
 ├─ CellNetL2 ──────── cell messages ─── EthernetLink (LAN8720) ─── cell switch
 ├─ GpioExpander ───── MCP23S17 ─── Spi3Bus (SPI3) ─── field I/O, gripper valve, encoder
 ├─ SpiDisplay ─────── TFT menu on SPI3
 ├─ AppConfig ──────── stored parameters
 ├─ GantryOta ──────── firmware updates
 └─ ThetaTrace, BootLogRing ─── diagnostics
```

-   Application code commands motion only through `Gantry::Gantry`.
-   The W5500 carries EtherNet/IP Class 1 to the drives on their own daisy
    chain. The LAN8720 carries CellNet L2, the TCP console and OTA on the cell
    switch. The two never share cabling; the cabling, subnets and addresses are
    in
    [EXPECTED_ELECTROMECHANICAL_ASSEMBLY §3](docs/EXPECTED_ELECTROMECHANICAL_ASSEMBLY.md#3-networks-and-cell-topology).
-   Pins and task constants are in `include/gantry_app_constants.h`; the
    mechanics are in `include/axis_drivetrain_params.h`.
-   **Decision:** motion uses the drives' own Position Absolute moves over
    EtherNet/IP, which meet ±1 mm at 200 mm/s. Host-side Speed plus StopMotion
    did not.

The libraries, grouped by layer, are listed in [lib/README.md](lib/README.md).

## 2. Boot sequence

With `CONFIG_EIP_SCANNER_ENABLED`:

From [`src/main.cpp`](src/main.cpp), whose `app_main` calls the stages in
[`src/main_init_io.cpp`](src/main_init_io.cpp),
[`src/main_init_motion.cpp`](src/main_init_motion.cpp),
[`src/main_tasks.cpp`](src/main_tasks.cpp) and
[`src/main_init_net.cpp`](src/main_init_net.cpp):

1.  **SPI3 + MCP** — bus init, Field/UI/TFT CS/W5500 RST; optional TFT stub.
2.  **W5500** — `w5500.init(...)` with MCP-driven RST (static; must outlive the
    scanner).
3.  **Process images** — `eipImageX` / `eipImageZ` / `eipImageTheta`.
4.  **Axes + Gantry** — `GantryEipLinearAxis` X/Z (and `GantryEipRotaryAxis`
    when `CONFIG_EIP_AXIS_THETA=y`) → `Gantry::Gantry(..., PIN_GRIPPER)` → joint
    limits → `gantry.begin()`.
5.  **`gantry.enable()` is deferred** until Class 1 + `GantryUpdate` are running
    (operator `enable` / console).
6.  **Scanner** — `eip::startScannerTask(...)` **before** the LAN8720 network
    (priority **6**, above gantry/SPI3 users).
7.  **CellNet objects constructed** — `EthernetLink`, the L2 transport and the
    gantry's `CellNetL2Node` (not started yet).
8.  **FreeRTOS tasks** — PickScheduler → GantryUpdate (100 Hz) → SerialCmd
    (UART).
9.  **LAN8720 `EthernetLink::start()` + `waitForUp()`** — then **TCP net
    console** on port **2323** and the **OTA server** on **8032**.
10. **CellNet L2** — the transport attaches to the EthernetLink handle and
    `l2Node.begin()` starts the node; a failure is logged, not fatal. Then the
    SNTP client starts.
11. Print console help → `vTaskDelete(nullptr)`.

## 3. Tasks

| Task | Priority | Core | Created by | Role |
|------|---------:|-----:|------------|------|
| `EipHoldKA`, `EipReopenKA`, `EipCloseKA` | 7 | 1 | EtherNetIP | Keep O→T flowing while a connection opens or closes |
| `EipScannerM` | 6 | 1 | EtherNetIP | Class 1 exchange every RPI |
| `PickStrike` | 6 | 1 | `src/pick_scheduler.cpp` | Fires the timed Z release and grip |
| `GantryUpdate` | 5 | 1 | `src/main_tasks.cpp` | 100 Hz axis state machines and path sequencing |
| `PickScheduler` | 5 | 0 | `src/main_tasks.cpp` | Vision queue, tracker, pre-position, bin drop |
| `PickTrace`, `ThetaTrace` | 4 | 0 | `src/pick_scheduler.cpp`, ThetaTrace | Strike and theta traces (`ztrace`, `thtrace`) |
| `gantry_ota_srv` | 4 | 0 | GantryOta | TCP OTA server |
| `SerialCmd`, `NetConsole` | 3 | 0 | `src/main_tasks.cpp`, `src/gantry_net_console.cpp` | UART and TCP console |
| `TftUiTask` | 2 | 0 | `src/main_tasks.cpp` | Encoder at 100 Hz, TFT at 15 Hz |
| `EipLog` | 2 | 0 | EtherNetIP | Prints the deferred Class 1 log |
| `TestCycle`, `TestThetaPath`, `CalibrateTask`, `ZCalibrateTask` | 2 | 0 | test console | Long console commands |

Priorities and cores are the constants in `include/gantry_app_constants.h` and
the libraries' own (their REFERENCE pages). Core 1 is kept for Class 1 and
motion; SPI3 work stays on core 0. The OTA-only boot mode runs its own two tasks
([GantryOta REFERENCE](lib/GantryOta/REFERENCE.md)).

## 4. One move, end to end

1.  A console `move` or the pick scheduler calls `Gantry::moveTo`
    ([Gantry REFERENCE](lib/Gantry/REFERENCE.md)).
2.  The path planner splits the X–Z move into segments around the SAFE_Z band
    and schedules theta inside it
    ([Paths, SAFE_Z and bring-up](lib/Gantry/REFERENCE.md#paths-safe_z-and-bring-up)).
3.  Every 10 ms `GantryUpdate` steps each axis: preload the target, raise
    StartMotion, then wait for arrival
    ([Linear axis: Position Absolute](lib/Gantry/REFERENCE.md#linear-axis-position-absolute)).
4.  The axis writes its command into the process image. The scanner sends it to
    the drive every RPI and writes the drive's feedback back
    ([Process image mailbox](lib/EtherNetIP/REFERENCE.md#process-image-mailbox),
    [Class 1 lifecycle](lib/EtherNetIP/REFERENCE.md#class-1-lifecycle)).
5.  The scanner reaches the drives through the W5500's hardware sockets
    ([W5500 REFERENCE](lib/W5500/REFERENCE.md)).
6.  A limit switch ahead of the move, a drive fault or a timeout stops it
    ([limit aborts](lib/Gantry/REFERENCE.md#frame),
    [abort matrix](lib/Gantry/REFERENCE.md#orchestration)).

## 5. Console contract

Shared command core: `gantryConsoleProcessLine()` in
[`gantry_test_console.cpp`](src/gantry_test_console.cpp). Transports:

| Transport | Path | Notes |
|-----------|------|-------|
| **UART0** | `SerialCmd` / `getchar` | Flash + pre-link logs; always available |
| **TCP :2323** | [`gantry_net_console.cpp`](src/gantry_net_console.cpp) on **LAN8720** | After plant ETH IP up; **not** on W5500/EIP |

Config (idf.py menuconfig): **LAN8720 plant Ethernet** and **TCP gantry console
(LAN8720)** — bridged by
[`ethernet_app_config.h`](include/ethernet_app_config.h). Defaults: gantry IP
**`10.42.0.100/24`** (gateway `10.42.0.1`), TCP **`:2323`**, password
**`LTU_1932`**, up to **4** remembered IPs for **60 s** (timer resets on
reconnect). UART has no password. Single TCP client (backlog 1).

### Connect from a PC (plant / LAN8720 subnet)

Prerequisites: PC on `10.42.0.0/24`, gantry at `10.42.0.100`, link up.

```powershell
# Windows (ncat from Nmap)
ncat 10.42.0.100 2323

# PuTTY: Connection type Raw, Host 10.42.0.100, Port 2323
```

```bash
# Linux / macOS
nc 10.42.0.100 2323
```

1.  At `Password:` prompt, enter the menuconfig password
    (`CONSOLE_TCP_PASSWORD`, default `LTU_1932`) and press Enter. Recent IPs
    skip this for **`CONSOLE_TCP_AUTH_REMEMBER_S`** seconds (default **60**);
    the timer resets on each reconnect from that IP (max
    **`CONSOLE_TCP_AUTH_REMEMBER_MAX`**, default **4**).
2.  On success, type the same commands as serial (`help`, `status`, …), one line
    each.
3.  `logout` / `exit` / `quit` closes the TCP session.

UART remains for bring-up before Ethernet is ready (no password).

### Commands

Primary: `help`, `status`, `calibrated`, `faults` / `alarms`, `enable`,
`disable`, `home`, `calibrate`, `test_cycle`, `speed`, `accel`, `move`, `grip`,
`stop`, `alarmreset`, `puuinfo`, `puucal`, `puu t`, `thetalim`, `livepos`,
`units`, `selftest`.

Also present (see `help`): `limits`, `pins`, `gpio_drive`, `rangelimit`,
`axislog` (EIP axes emit START/END always; periodic MOVE when `axislog` Hz > 0).

| Command | EIP-specific behavior |
|---------|------------------------|
| `enable` / `disable` | Arms / ServoOff via Gantry |
| `home` | Drive-managed: seek joint min per axis (`home x` A014; `home z` A015; `all` = Z then X); else soft-home |
| `calibrate` | Drive-managed: seek joint max (`calibrate x` A015; `calibrate z` A014); **`calibrate all`** = bring-up Z−→band→X→Z+→band; else SCHUNK hard envelope (X soft-cal) |
| `calibrated` | Query workspace calibrated latch. **Yes** after X+Z stroke; **No** only when a live drive reports lost position-ref (Kinetix `homed_status` / HCS01 `in_reference`). Survives disable/stop. **Does not survive an ESP reboot** (OTA, power cycle): the X/Z joint zero is held in RAM, so after a reboot the axes read raw drive position and the latch starts **No** until `calibrate all`. Boot erases the older `gantry_cfg/ws_cal` NVS key that used to restore it. Printed on `status` |
| `test_cycle` | Cannot pass on this machine today (OPEN_ITEMS G-E8). Enable + bring-up (home/cal) + path legs A–F at live `speed`/`accel`, then theta G–I at `thetalim` min/max using kinematic speed/accel caps; `stop` aborts |
| `test_theta_path` | Combined in-band X+Z+theta (25–75% window) with live thetalim-safe dθ; enable + bring-up first |
| `move` | Requires **workspace calibrated**. Per-axis session home/cal still apply. Latch does **not** clear on `stop` |
| `stop` | Abort + disable; does **not** clear session home/cal gates |
| `alarmreset` / `arst` | EIP **FaultReset** (Kinetix) and HCS01 C0500 **bit5**; if Gantry is disabled, theta stays Drive OFF (no WaitAf / AF). HTTP `hcs01_eng.py c0500` if theta T→O is stale |
| `faults` / `alarms` | Decode FaultCode/WarningCode (e.g. A603) |
| `puuinfo` / `puucal` / `puu t` | Scale / suggest PUU/mm; theta PUU/deg is live-settable |
| `speed` / `accel` | Path mm/s and mm/s²; optional theta deg/s and deg/s² |
| `thetalim` | Software theta joint min/max (deg) |
| `picksim <x> <y> <theta> <class> <item_id> <w> <len> <h>` | Queue a detection (needs Start). `y` is camera-relative. Class 1–10 is a pick (AAA, AA, D, C, 9V, CR123A, 18650, 21700, 26650, 32650). Class 11–13 is a pack occupant and is not picked. All three sizes are required. The command stamps the live belt displacement as the capture sample |
| `pickplan <x> <y> <theta> <class> <item_id> <w> <len> <h>` | Same input as `picksim`. Prints pick or skip, jaw angle, window, clearances, contact speed, budget, and the rejection reason |
| `pickcover [class]` | Pick or skip per cell angle (−90..85° in 5° steps, 0° across the belt, 90° along it) at the current belt speed: `P` pick, `S` skip |
| `pickcal valve \| jaw \| jawoff \| cam` | Valve delay, per-class jaw time (`0` = derived from jaw travel), jaw offset, camera mark/apply |
| `ztrace [raw <start> <count>]` | Inspect last strike's 1 kHz vertical Z trace. Summary prints timing, overshoot, settle. Raw dumps are not paced in the firmware (checked 2026-11-19); keep them small (G-E1) |
| `thtrace arm` / `thtrace` / `thtrace raw <start> <count>` / `thtrace off` | 1 kHz trace of the next theta move from any source (`move`, picks): HCS01 actual angle, velocity and status word per sample, plus the move as commanded. Class 1 refreshes every 2 ms, so samples can repeat. Summary (ms from the move command): start edge, onset, profile start `t0` and latency, ideal trapezoid time, entry into and settling within ±0.1° (S-0-0057), overshoot, worst following error against the ideal trapezoid (aligned at 20 % of the accel distance, so a constant delay shows as latency), peak speed, final error, how the axis ended the move. Raw: at most 300 samples per call, 10 per line, 20 ms per line. `tools/theta_trace_fetch.py` saves a whole trace as CSV. Code: `lib/ThetaTrace` |

The MCP diagnostic commands (`mcp_*`, `field_dout`, `field_din`) are built while
`MCP_DEBUG_CMDS` is 1, the default (`src/gantry_console_internal.h`).

## 6. The pick path

CellNetL2 (LAN8720) → vision queue → `PickScheduler` task → tracker →
`Gantry::moveTo(JointConfig)` pre-position → timed strike → bin drop. The
Ethernet receive callback only queues the detection and its receipt time. A
separate `PickStrike` task fires the Z release and grip from `esp_timer`
one-shots, re-timed from the newest belt state on every conveyor frame.

-   **Side grip only.** Cell angle 0° is the long axis along +X (across the
    belt), 90° along +Y, positive about +Z up. The theta joint holds the pads
    along +Y at 0° and turns them counter-clockwise seen from above (Pi camera
    check, 2026-11-16): joint = fold(θ) + `jaw_off_deg` − 90°, with
    `jaw_off_deg` 0° by default. Of the two equivalent angles the one with less
    linear theta travel inside `thetalim` is used, else skip `theta range`
    (never clamped). Any pick turn with |dθ| ≥ 0.1° is commanded and quantized
    to 0.1°; a change of 1.0° or less is left in place (`budgetPick`). On a pick
    or transfer retract, X and theta may start up to 160 ms before Z reaches the
    SAFE_Z band, at most 48 mm below it and within 78 % of the stroke
    (`Path::retractLeadMm`, `Gantry::retractLeadReady`); console moves still
    wait for the band. The early start does not check the conveyor frame's
    collision zone (G-C14).
-   **Pick** when the cell's across-belt strip clears both fingers by ≥ 4 mm: Z
    goes to the grasp right after the pre-position, and only the grip is timed.
    Grasp height is $`\min(h/2,\ 25.4\,\mathrm{mm})`$ down from the object top.
    Width, length, and height-from-belt are required on every detection. A width
    of 80 mm has zero jaw travel; it is still a pick only when that across-belt
    strip misses the fingers. Width above 80 mm is skipped. A successful grip
    write is a commanded close; possession is unverified until a part sensor
    exists.
-   **Late Z** otherwise, if `pk_dropin` is on (default off): hover above the
    tallest tracked cell, then a Z descent that is at grasp height one guard
    before the earliest jaw/object contact. The 4 mm lane margin, 3 mm hover, 15
    ms guard, and flat-rectangle contact model are provisional and need bench
    replacement. With `pk_dropin` off that case is skipped (`late Z off`).
-   **Coordinated grasp.** `coordinated_grasp.h` is host-tested
    (`test_coordinated_grasp`) but the pick path does not call it, and there is
    no bench switch for it. `pk_dropin` is not that switch. The update task
    period is `GANTRY_UPDATE_PERIOD_MS` (10). The strike edge does not wait on
    that task.
-   **Failures** (belt stop, stale frames, conveyor restart, refined angle out
    of the frozen jaw's clearance, grip write failure, missed deadline) abort
    the pick and call `retractToSafeZ()` at the ballscrew cap. A commanded close
    blocks another pick until the bin sequence succeeds. The object is marked
    taken only after the grip command fires.
-   After the drop the gantry retracts at the bin, then goes to the next tracked
    cell, or waits at the staging pose (`pk_stage_x_mm`, SAFE_Z, theta 0).
    `pk_drop_any_th` skips the turn to `geom_cal_th_deg` at the bin.
-   Belt clock: gantry X is vision $x$ plus `vis_x_off_mm` (default 245, the 95
    mm collision edge plus half of the stored 300 mm belt). `cv_belt_w_mm` is
    not read by the intercept, so changing the width does not move the camera. A
    repeated conveyor sequence is ignored. The epoch restarts on a sequence jump
    above 32, a displacement step backward of more than 5 mm, or a conveyor-time
    step backward of more than 500 ms. Three unchanged displacement samples
    while speed stays positive integrate the previous speed. Stale is gantry
    receipt age above `kStaleAfterUs` (500 ms, two configured 250 ms periods),
    not the live gap between conveyor timestamps.
-   The pick scheduler has no motion parameters of its own. The active approach
    and the transfer to the bin run on the global profile: `spd_mm_s`,
    `th_spd_deg_s` (capped by `th_spd_max`), `acc_mm_s2`, `dec_mm_s2`,
    `th_acc_deg_s2`, and `th_dec_deg_s2`. A following part uses the speed
    ceiling `ceil_spd_mm_s` only when its own approach from the post-bin pose
    misses at the default speed and still arrives at the ceiling; otherwise that
    follower is skipped. Z descend, grip, and retract use
    `AXIS_Z_PICK_SPEED_FRACTION` of the stored ballscrew cap (950 mm/s from
    `z_crit_rpm` and `z_lead_mm`). Safe Z comes from
    `Gantry::traverseClearanceZMm()`; tick, preload, settle, and arrival values
    come from `GantryControlPeriod.h`, `GantryLinearPhases.h`,
    `GantryRotaryPhases.h`, and `GantryPathProfile.h`. `grp_valve_ms` and
    `jaw_aaa_ms`…`jaw_32650_ms` (`0` derives $g/v_j$), `fin_spd_mm_s`, and
    `pk_guard_ms` stay tunable.

For real-time protocol specs, see
[CELL_NET_L2_COMMUNICATION_GUIDE.md](lib/CellNetL2/CELL_NET_L2_COMMUNICATION_GUIDE.md).
For kinematics, see
[PICK_SCHEDULER_KINEMATICS.md](docs/PICK_SCHEDULER_KINEMATICS.md).

### Pick model (until the math document)

The model below was read from the code at `20e03eb` on 2026-11-05. Each equation
names the function that implements it.
[PICK_SCHEDULER_KINEMATICS.md](docs/PICK_SCHEDULER_KINEMATICS.md) and
[ON_THE_FLY_PICKUP.md](docs/ON_THE_FLY_PICKUP.md) give the derivations and the
simulation behind the strike plan. Where they disagree with the code, the code
wins (X-2).

#### Frames and symbols

-   **Belt plane:** $+X$ across the belt, $+Y$ downstream. Joint Z points down,
    and $Z = 0$ is fully retracted.
-   **Belt:** $s$ is the belt odometer in mm along $+Y$, and $v$ the belt speed.
-   **Cell:** a rectangle of length $l$ (the long axis), width $w$ (the side the
    jaws close across, never larger than $l$) and height $h$ above the belt. Its
    angle $\theta$ is that of the long axis: $0^\circ$ across the belt, positive
    toward downstream.
-   **Time:** strike times are relative to **arrival**, $t = 0$ when the cell
    centre reaches the pick plane.
-   **Parameters** and their stored keys are in [5.3.10](#parameters).

#### Belt clock

The conveyor sends frame $k$ every 250 ms with its own timestamp $\tau_k$,
displacement $d_k$ and speed $v_k$. The gantry receives it at $r_k$. The clock
offset is the smallest delay over the last 8 frames, and the position is
extrapolated linearly from the newest frame (`pushMinDelay`, `beltPositionMm`):

```math
\delta_k = \min_{j = k-7, \dots, k} (r_j - \tau_j) \qquad (1)
```

```math
s(t) = d_k + v_k \, (t - \tau_k - \delta_k) \qquad (2)
```

-   **Stale:** an armed strike aborts when the newest frame is over 500 ms old.
-   **Restart:** a sequence jump over 32, a displacement drop over 5 mm, or a
    timestamp 0.5 s back starts a new epoch. Parts and strikes from the old
    epoch are dropped.
-   **Flat displacement:** if $d$ stays flat for 3 frames while $v > 0$, the
    position is integrated from $v$, and a bias $b$ keeps it continuous
    (`anchorFrame`).
-   **Assumptions:** $v$ is constant between frames, and the transport delay has
    a stable minimum that at least one of 8 frames reaches.

#### From detection to arrival

A `VISION_DETECT` gives $x$ across the belt, $y_{bat}$ from the camera centre,
$\theta$, $w$, $l$, $h$ and the class. If $w > l$, the two swap and $\theta$
turns $90^\circ$ (`gripAcrossWidth`). The capture position, the belt position at
which the cell reaches the pick plane, and the time left (`capturePositionMm`,
`arriveBeltMm`, `secondsToPlane`):

```math
s_c = \begin{cases} 1000 \, d_{cap} + b & \text{the frame carries capture displacement} \\ s(r - \Delta_{vis}) & \text{otherwise} \end{cases} \qquad (3)
```

```math
s_a = s_c + y_{pick} - (y_{cam} + y_{bat}) \qquad (4)
```

```math
t_a = \frac{s_a - s(t)}{v} \qquad (5)
```

-   **Latency:** $\Delta_{vis}$ is `vis_lat_ms`. The contract comment says
    capture displacement is required, but any `vis_lat_ms` above 0 lets every
    frame use the second line of (3) (G-C13).
-   **Trigger stand-in (`vis_batt_x` = 1):** $x$ comes from `batt_x` through
    forward kinematics, and $y_{bat} = -130 - 6 = -136$ mm, from compile-time
    constants (G-C5). The frame has no capture displacement, so (3) back-dates
    the capture by $\Delta_{vis}$. With the compiled defaults, $s_a - s_c$ = 847
    mm, which is 2.82 s at 300 mm/s.
-   **Gantry X:** the target is the inverse kinematics of $x$ + `vis_x_off_mm`.
-   **Repeats:** a detection of the same cell (same id, or within
    `vis_dedupe_mm` in both $x$ and $s_a$) updates $s_a$. An armed strike then
    moves only its arrival.
-   **Tracking:** up to 16 cells, while $`-25 < s_a - s < 1500`$ mm.

#### Grip geometry

The pads run along the long axis and the jaws close across the width
(`chooseSideJaw`):

```math
\theta_j = \mathrm{fold}(\theta) + \theta_{off} - 90^\circ + k \cdot {180^{\circ}} \qquad (6)
```

Here $\mathrm{fold}$ maps an angle into $[-90^\circ, 90^\circ)$, $\theta$ is the
vision angle (0° across the belt, 90° along it, positive about +Z up),
$\theta_{off}$ is `jaw_off_deg` (0°; joint 0° holds the pads along +Y and a
positive joint turns them counter-clockwise seen from above, so joint = pose −
90°), and $k$ picks the in-range joint angle nearest the current one. With $o$
the open jaw gap (`fin_open_mm`) and $z_b$ the joint Z of the belt surface
(`col_z_min_mm`):

```math
c = \tfrac{1}{2}(o - w) \qquad (7)
```

```math
z_g = \begin{cases} z_b - 3 & h < 30 \text{ mm} \\ z_b - h + \min(h/2, \, 25.4) & \text{otherwise} \end{cases} \qquad (8)
```

```math
z_h = \max\big(\min(z_b - h - 6, \; z_{safe}), \; z_{min}\big), \qquad z_{safe} = z_{min} + 48.4 \qquad (9)
```

-   $c$ is the clearance on each side between the cell and an open pad, which is
    also each jaw's stroke to contact.
-   (8) is `zPlan`.
-   (9) is the hover from `StrikePlan::plan`: 6 mm above the cell top, but never
    lower than the safe-Z ceiling. For a 9V on a belt at Z 122 the ceiling wins,
    so the plunge starts at $z_{safe}$.

#### Strike plan

`StrikePlan::plan` (`strike_plan.h`) places three events around arrival: release
the plunge, fire the grip valve, start the retract. Move times use the
time-optimal trapezoid, or a triangle when the move is too short to reach $v$
(`trapezoidSeconds`):

```math
T(D; v, a, d) = \begin{cases} \dfrac{v}{a} + \dfrac{v}{d} + \dfrac{D - D_r}{v} & D \ge D_r = \dfrac{v^2}{2}\left(\dfrac{1}{a} + \dfrac{1}{d}\right) \\[2ex] \hat{v}\left(\dfrac{1}{a} + \dfrac{1}{d}\right), \quad \hat{v} = \sqrt{\dfrac{2Dad}{a + d}} & D < D_r \end{cases} \qquad (10)
```

```math
\tau_p = T(z_g - z_h; \, v_z, a_z, d_z), \qquad \tau_{band} = \tau_p - \sqrt{2 (z_b - h - z_h) / a_z} \qquad (11)
```

-   $v_z$ = 0.95 × `z_crit_rpm`/60 × `z_lead_mm`: the ball-screw whip limit less
    5 %, 950 mm/s. $a_z$ and $d_z$ are `acc_mm_s2` and `dec_mm_s2`.
-   $\tau_{band}$ is the part of the plunge below the cell top. The square root
    assumes the top is crossed while Z still accelerates; when the top lies past
    mid-plunge, $\tau_{band}$ comes out too long (G-C6).

The jaw close is a first-order valve lag on a constant jaw speed
(`jawStrokeSeconds`):

```math
q(t) = v_j \left( t - \tau_v \left(1 - e^{-t/\tau_v}\right) \right), \qquad \tau_{cl} = t_{valve} + q^{-1}(c) \qquad (12)
```

Here $v_j$ is `fin_spd_mm_s` (210 mm/s: SCHUNK KGG 100-80, 40 mm in 0.19 s at 6
bar), $\tau_v$ = 15 ms (a simulation constant), and $t_{valve}$ is `pk_von_ms`.

The entry window: the fingertips may pass the cell top only once the cell is 2
mm inside the open gap, and Z must reach depth soon after arrival. $v_\times$ is
the belt speed along the close axis:

```math
v_\times = v \, |\cos\theta|, \qquad t_{in} = \begin{cases} -(c - 2) / v_\times & v_\times > 1 \text{ mm/s} \\ -0.5 \text{ s} & \text{otherwise} \end{cases} \qquad (13)
```

```math
t_d = \mathrm{clamp}\Big(\max(t_{in} + 5 \text{ ms}, \; -\tau_{band}) + \tau_{band}, \;\; 0, \;\; \min(15 \text{ ms}, \; 20 \text{ mm} / v)\Big) \qquad (14)
```

The events are the nominal times plus the operator offsets:

```math
t_z = t_d - \tau_p + \Delta_p, \qquad t_c = t_d - (\tau_{cl} - 8 \text{ ms}) + \Delta_c, \qquad t_r = t_d + \Delta_{r} \qquad (15)
```

-   **Offsets:** $\Delta_p$, $\Delta_c$, $\Delta_r$ are `pk_off_plunge`,
    `pk_off_close` and `pk_off_retract`, in whole ms within ±100 ms. Each moves
    only its own event.
-   **Zero offsets:** the retract starts as the plunge ends (zero dwell), and
    the jaws meet the cell 8 ms after the retract starts.
-   **Invariants, applied after the offsets:** $t_r \ge t_z + \tau_p$, so Z
    never reverses before depth, and
    $t_z + 2 \text{ ms} \le t_c \le t_r - 2 \text{ ms}$. The dwell
    $t_r - (t_z + \tau_p)$ is logged with its sign; a negative dwell is not
    applied.

#### Feasibility

`evaluate` accepts a strike only when all of these hold:

-   **Time:** the pre-position finishes before the plunge release.

    $$\text{slack} = (t_a + t_z) - (T_{app} + 40 \text{ ms} + 30 \text{ ms}) \ge 0 \qquad (16)$$

    $T_{app}$ (`budgetPick`) is the move from the current pose to
    $(x, z_h, \theta_j)$, timed the way the Gantry plans it:
    -   X moves only while Z is inside the traverse band.
    -   Each segment is a trapezoid (10) plus 40 ms preload and 30 ms settle.
    -   A combined X+Z segment is slowed until its Z part stays under $v_z$.
    -   Theta runs alongside, starting with the in-band segment after up to 190
        ms of start latency.

    The 40 + 30 ms in (16) are the plunge's own preload and settle.
-   **Reach:** $z_{min} \le z_g \le \min(\text{Z stroke}, z_b)$, and neither the
    pick nor the hover pose lies inside the conveyor box, $x \ge$ `col_x_min_mm`
    with $z > z_b$.
-   **Neighbours:** no other tracked cell's bounding box may enter a finger
    rectangle (`fin_wid_mm` × `fin_len_mm`) from the release at $t_a + t_z$
    until Z is back up, $T(z_g - z_h)$ + 70 ms after $t_a + t_r$
    (`fingersFreeFromS`).
-   **Belt:** $v > 0$ and $t_a > 0$.

#### Execution and retiming

`runPick` retracts into the band, moves X and theta to the hover pose, and
re-evaluates after each move. With the jaw angle then frozen, it preloads the
plunge to $z_g$, holds its StartMotion, and arms three one-shot timers at
$t_a + t_z$, $t_a + t_c$ and $t_a + t_r$.

Every conveyor frame, and once before each event, re-solves the arrival from (2)
and (5). The event offsets stay fixed (`retimeArrival`):

```math
t_a \leftarrow t + \frac{s_a - s(t)}{v} \qquad (17)
```

-   **Abort:** an armed strike aborts on a clock restart, stale frames or a
    stopped belt.
-   **Z event:** releases the preloaded plunge and queues the retract behind it.
-   **Grip event:** turns the valve on through the MCP23S17 (PA0).
-   **Retract event:** the edge for the queued retract. The drive reverses as
    the plunge completes.
-   **Dispatch:** an event fires when it is due within 1 ms.
-   **Log:** `pick actual` gives each event's error against its target and the
    deepest Z, and warns on a shortfall over 0.3 mm. The drive follows its
    command about 30–35 ms behind, so a zero-dwell strike reverses about 0.7 mm
    short of $z_g$ (0.68–0.70 mm on the bench, 5.3.9); that is accepted.

#### Cycle time and followers

After a strike the gantry carries the cell to the bin. `classifyFollowers` times
that cycle and decides how each later cell is approached:

```math
T_{cyc} = t_{close} + T_{up} + T_{app}(\text{bin}) + 2 \, T_{bin} + t_{open} \qquad (18)
```

```math
\text{pace} = \begin{cases} \text{slow} & t_a + T_{cyc} + T_{app}^{slow} \le t_a' \\ \text{fast} & t_a + T_{cyc} + T_{app}^{fast} \le t_a' \\ \text{drop} & \text{otherwise} \end{cases} \qquad (19)
```

-   $t_{close}$ and $t_{open}$ are `grp_close_ms` and `grp_open_ms` (190 ms
    each). $T_{up}$ is Z from $z_g$ up to $z_{safe}$; $T_{bin}$ is Z between
    $z_{safe}$ and the bin, down and back. Each Z move is (10) at $v_z$ plus 70
    ms. The bin pose is `geom_cal_x_mm`, `geom_cal_z_mm`, `geom_cal_th_deg`.
-   $T_{app}^{slow}$ and $T_{app}^{fast}$ run at `spd_mm_s` and `ceil_spd_mm_s`,
    from the bin pose to the follower's X and jaw angle. $t_a'$ is the
    follower's arrival.
-   (19) has four known errors (G-C4).

#### Check against the bench

On 2026-10-25, strikes 7101–7105 picked a 9V on its side with its length along
the belt ($\theta = 90^\circ$, $w$ = 17.5, $h$ = 26.5). The inputs were: belt Z
122, so $z_g$ = 119.0; hover 35.7, that day's ceiling; $v_z$ = 950; $a_z =
d_z$ = 3000; belt 150 mm/s; offsets 0. The current code, compiled with those
inputs, gives:

| Z release to… | Model | Measured ([STRIKE_TIMING](STRIKE_TIMING_MEASUREMENTS_2026-10-25.md)) |
|---|---|---|
| Grip valve fires | +169.5 ms | +169.5 to +170.1 ms (7102–7105); +172.2 ms (7101, whose MCP write took 2.0 ms) |
| Retract commanded | +333.3 ms ($\tau_p$, 83.3 mm triangle) | +333.8 ms (7101) |
| Deepest Z | 119.0 commanded | 118.30–118.32 (the accepted lag) |

At $\theta = 90^\circ$ the belt carries the cell along the pads, so
$v_\times = 0$ and (14) gives $t_d = 0$: depth is reached exactly at arrival.
Three results from the same code at today's compiled ceiling (48.4), 300 mm/s,
3000 mm/s²:

-   **Same cell:** it plunges 70.6 mm in 306.8 ms, and the grip fires 143.0 ms
    after the release. Computed, not yet measured.
-   **Turned across the belt ($\theta = 0^\circ$):** (14) hits its 15 ms cap. On
    the commanded profile the fingertips pass the cell top 110 ms before
    arrival, when the cell is still 1.8 mm short of the upstream pad face rather
    than 2 mm inside it. This holds for $\theta$ from 0° to 35°. The drive's
    30–35 ms lag probably hides it on the machine (assumed, G-C11).
-   **$\tau_{band}$:** (11) overstates it by 4.4 ms for this cell (8.4 ms at the
    35.7 ceiling). That changes the plan only for $\theta$ between 33° and 43°
    (G-C6).

#### Parameters

Defaults are the compiled ones; the live stored set differs where noted.

| Symbol | Meaning | Key | Default |
|---|---|---|---|
| $o$ | Open jaw gap | `fin_open_mm` | 80 mm |
| — | Finger thickness and length | `fin_wid_mm`, `fin_len_mm` | 10, 50 mm |
| $v_j$ | Jaw speed | `fin_spd_mm_s` | 210 mm/s |
| $t_{valve}$ | Valve delay | `pk_von_ms` | 8 ms |
| $\Delta_p, \Delta_c, \Delta_r$ | Operator offsets | `pk_off_plunge`, `pk_off_close`, `pk_off_retract` | 0 ms |
| $\theta_{off}$ | Jaw angle offset | `jaw_off_deg` | 0° |
| $z_b$ | Belt surface, joint Z | `col_z_min_mm` | 115 mm (122 live, G-F1) |
| — | Conveyor box edge, X | `col_x_min_mm` | 95 mm |
| $z_{safe} - z_{min}$ | Traverse ceiling | stored `geom_safe_z_mm`, applied at boot | 48.4 mm by default |
| $v_z$ | Plunge speed | 0.95 × `z_crit_rpm`/60 × `z_lead_mm` | 950 mm/s |
| $a_z, d_z$ | Accel, decel (all linear axes) | `acc_mm_s2`, `dec_mm_s2` | 3000 mm/s² (5000 in the 2026-10-30 runs, G-C2) |
| — | Approach speed | `spd_mm_s` | 50 mm/s (750 live, G-C7) |
| — | Fast approach speed | `ceil_spd_mm_s` | 2000 mm/s |
| — | Theta speed, accel, decel | `th_spd_deg_s` (capped by `th_spd_max`), `th_acc_deg_s2`, `th_dec_deg_s2` | 30°/s, 180°/s², 180°/s² |
| $\Delta_{vis}$ | Vision latency | `vis_lat_ms` | 40 ms |
| $y_{cam}, y_{pick}$ | Camera centre, pick plane | `vis_cam_y_mm`, `cv_pick_y_mm` | −711, 0 mm |
| — | Vision X to gantry X | `vis_x_off_mm` | 245 mm (307.5 live, G-F1) |
| — | Duplicate radius | `vis_dedupe_mm` | 40 mm |
| — | Bin pose | `geom_cal_x_mm`, `geom_cal_z_mm`, `geom_cal_th_deg` | 10 mm, 90.8 mm, 90° |
| $t_{close}, t_{open}$ | Grip close and open dwell | `grp_close_ms`, `grp_open_ms` | 190 ms each |
| — | Trigger stand-in | `vis_batt_x`, `batt_x` | 0, 382.5 mm |

Compile-time constants: valve lag $\tau_v$ 15 ms, retract lead 8 ms, hover
clearance 6 mm, entry margin 2 mm, entry guard 5 ms, depth cap 15 ms or 20 mm of
belt, minimum event gap 2 ms, offset limit ±100 ms (`strike_plan.h`). Grasp rule
3 mm, 30 mm and 25.4 mm (`zPlan`). Frame period 250 ms, stale after 500 ms,
8-frame delay window (`pick_intercept_vision.h`). Trigger geometry −130 and 6 mm
(`pick_belt.cpp:15,20`).

#### Limits of the model

-   **Bodies:** rigid rectangles at the reported pose, with no slip on the belt.
-   **Belt:** constant speed between 250 ms frames.
-   **Open loop:** the plunge is assumed to follow (10) exactly, and the jaw
    (12) with the simulation's 15 ms valve constant. The real drive lags 30–35
    ms. Simulation gains are not facts; the three offsets absorb what the model
    misses.
-   **Provisional constants:** the 6 mm hover clearance, 2 mm entry margin, 5 ms
    guard, 8 ms retract lead and the 15 ms / 20 mm depth cap are not yet
    measured.
-   **Unused model:** `include/pick_intercept_strike.h` and
    `include/pick_intercept_plan.h` hold an earlier EarlyZ/LateZ strike model
    (`planPick`, `strikeLeads`, `retimePick`), still included through
    `pick_intercept.h`. Only the host tests call it (G-C8).

### Bench ramp for the coordinated grasp

**Not runnable yet.** The pick path does not call `coordinated_grasp.h`, and the
firmware has no bench switch or close-trigger trim. The stored keys once meant
for them (`pk_bench`, `pk_voff_ms`, `pk_tr_*`) were read by nothing and were
removed on 2026-11-16. The ladder below needs that path built first.

Production timed picks stay off. `pk_dropin` stays off and is not the switch.
The bench switch is to allow one selected case at a time. The paper grid is in
[PICK_COORDINATED_GRASP_VALIDATION.md](docs/PICK_COORDINATED_GRASP_VALIDATION.md).
Simulation is not a bench result.

Do this at 6 bar, with finger width 10 mm and finger length 50 mm (`fin_wid_mm`,
`fin_len_mm`).

1.  Confirm conveyor travel, speed, and timestamp are one sample.
2.  Measure command-to-valve, jaw-motion start, and the Z response. The bases
    are valve-on 8 ms (`pk_von_ms`), valve-off 17 ms, and full close 190 ms.
    Trims start at 0.
3.  Enable the bench switch for one cell, one angle, and one speed.
4.  Tune the gripper close trigger until the gap between the pads and the cell
    is under 0.5 mm at the moment Z reaches grasp depth.
5.  Retract on that same instant. There is no dwell between the drop and the
    retract. The last of the close happens while Z is moving up.
6.  Start at 20 mm/s. Step by 20 mm/s toward 1500 mm/s.
7.  Record travel, the close-trigger time, the measured pre-grasp gap, Z
    feedback, drag, and whether the cell stayed in the fingers.
8.  Stop on an early pad hit, a pre-grasp gap of 0.5 mm or more, drag at or
    above 10 mm, or a miss.
9.  Change the close trigger and the other trims only. Do not edit the 8 ms, 17
    ms, or 190 ms bases to hide a hardware miss.
10. Promote a region to verified only after repeated trials with the logs kept.
    After a region is stable, lower the drag limit from 10 mm toward 1 mm and
    repeat the ladder.

At 1500 mm/s, 1 ms is 1.5 mm and one 2 ms EtherNet/IP packet is 3 mm. The Z
release edge waits on that packet. It does not also wait on the 10 ms Gantry
update. Leave `GANTRY_UPDATE_PERIOD_MS` at 10 unless a measurement shows preload
or arrival observation is what closes the window.

Gantry update periods of 10, 5, 4, and 2 ms are the candidates. Do not run
faster than the 2 ms RPI without a Class 1 p99 that stays under the RPI. This
change keeps 10 ms because that measurement has not been taken on the machine.

## 7. Runtime parameters

At runtime the stored parameters are the only source. `main_init_motion.cpp` and
the console pass them to the Gantry library through `gantryRuntimeParams()`, and
the pick scheduler reads them from `AppConfig`. Compile-time values come from
`idf/sdkconfig.defaults` alone: the build regenerates `sdkconfig` from it on
every configure, so a local menuconfig edit does not survive. Those values only
fill a key the stored set does not hold, or apply after an explicit
`cfg factory`. Macros in `axis_drivetrain_params.h` seed those defaults and are
not read for motion.

### 7.1 Stored set

NVS keeps one complete parameter set under `cfg_set`, with a write counter, the
time it was saved, and a hash. Boot loads it. Because every parameter is in it,
flashing a build with different defaults does not move a stored value.

Notes on what is and is not stored:
-   **Pick approach motion** has no keys of its own. Picks use the global
    profile: `spd_mm_s`, `ceil_spd_mm_s` only when a later part would otherwise
    be missed, and `th_spd_deg_s` capped by `th_spd_max` (`pickConfig()` in
    `pick_eval.cpp`; G-C7).
-   **Trigger sensor geometry** is still compile-time:
    `kTriggerSensorYFromCamMm` (−130 mm) and `kTriggerCentreBehindMm` (6 mm) in
    `pick_belt.cpp` (G-C5).
-   **Drive IP addresses** have stored keys (`eip_ip_x`, `eip_ip_z`,
    `eip_ip_th`), but the firmware still uses the Kconfig addresses (OPEN_ITEMS
    G-B5).

| Step | What happens |
|------|--------------|
| `cfg <key> <value>` | Applies now, in RAM only. Nothing touches flash, so this cannot affect Class 1. |
| `cfg save` | Refused unless the motors are off. Closes Class 1, writes the set, and reopens Class 1. Run `enable` afterwards. |
| Reboot or power loss | Changes not saved are gone. `cfg reboot` refuses while any are unsaved (`cfg reboot discard` overrides), and `tools/supervisor_ota.py` refuses to flash (`--discard-unsaved` overrides). `status` shows the count. |

Saving is an explicit step because of the Class 1 timeout, 16 × RPI (32 ms at
the 2 ms RPI, `EipScannerPolicy.h`). A flash sector erase stalls both CPUs for
its whole duration. On 2026-10-30, two of 151 per-key `cfg` writes took 81–82 ms
instead of about 8 ms, one with a measured 63.9 ms Class 1 gap, and both dropped
the link. The erase is inferred from the timing and the NVS entry count, not
observed directly.

The close runs one axis at a time, with a helper keeping O→T running to the axes
still open, because each drive needs its ForwardClose before its connection
times out. A Kinetix drops its explicit TCP session while Class 1 carries on
over UDP (W5500 `Sn_SR` reads `CLOSE_WAIT`), so the ForwardClose goes out with
the axis still connected and, if the session is gone or the send fails, is
retried on a rebuilt session. Why the drive drops it has not been established. A
log line `Class 1 close not clean` names any axis that refused.

Per-key values left by older firmware merge into the set at boot, before Class 1
opens, and are erased after `gantryOtaConfirmBootValid()`, so a rollback in that
window still finds them. An image from before this format reads only the per-key
store, so it starts from its own compile-time defaults.

### 7.2 Commands and the parameter file

| Command | Effect |
|---------|--------|
| `cfg <key>` | Shows the value, its label and what it does (the schema description). The 16 keys renamed on 2026-11-16 (`cv_cam_y_mm` → `vis_cam_y_mm`, `cv_x_off_mm` → `vis_x_off_mm`, `cv_vis_ms` → `vis_lat_ms`, `cv_dedupe_mm` → `vis_dedupe_mm`, `cv_jaw_deg` → `jaw_off_deg`, `cv_valve_ms` → `grp_valve_ms`, `cv_jaw_<class>_ms` → `jaw_<class>_ms`) still work under the old name, and a stored set keeps their values. |
| `cfg save` | Stores the running values, as above. The toolbox **Save to NVS** button and the TFT **Save Parameters** item do the same. |
| `cfg export` | Prints every visible parameter as `PARAM key=value`. Passwords are left out. |
| `cfg factory` | Puts compile-time defaults in RAM. `cfg save` stores them over the set, so download the file first. A reboot without saving brings the stored set back. |
| `cfg time` | Shows the clock and the NTP server. |

The toolbox page has **Download parameters**, which saves `cfg export` as a text
file, and **Upload parameters**, which reloads one. Upload compares the file
with the gantry and sends only the values that differ, as `cfg <key> <value>`,
so the changes are in RAM until saved. A key the gantry does not show, such as a
developer key while locked, is skipped and listed.

Boot never erases NVS or replaces a damaged set. If NVS cannot be read, or the
set fails its hash, the firmware logs an error and runs on compile-time
defaults. `cfg save` replaces a damaged set, and `cfg factory` erases unreadable
NVS.

### 7.3 Time

The supervisor runs chrony and serves NTP on `10.42.0.1`. The gantry syncs from
it through the `ntp_server` key (reboot to apply), and vision uses it too.
Motion and picks run on `esp_timer`, so a clock step does not affect them; the
clock only stamps the stored set.

## 8. Explicitly not supported

-   PulseMotor / LEDC step-dir production control
-   MCP23S17 as **motion / limit-switch** path (Field I/O + TFT on SPI3 is
    supported)
-   Opto PTI interface board as production control path
-   **Ephemeral Class 1 UDP** (bind/send/recv/close per frame)
-   OM=2 Speed + TM=10 + host StopMotion for PTP accuracy (OM=0 TM=10
    settle/hold/abort remains)
-   CIP Motion / Motion Group / assembly 106 ECAM for picks
-   LVGL on the TFT (the menu is SpiDisplay's own)

## See also

-   **Down:** [lib/README.md](lib/README.md) lists the libraries by layer; each
    has a README and a REFERENCE.
-   **Up:** [Knowledge transfer](docs/KNOWLEDGE_TRANSFER.md), the cell and its
    subsystems.
-   **Also:** [open items](docs/OPEN_ITEMS.md), [contributing](CONTRIBUTING.md).

# Pick Scheduler Kinematics

**Status:** Canonical mathematical design basis
**Companion Docs:** [EXPECTED_ELECTROMECHANICAL_ASSEMBLY.md](EXPECTED_ELECTROMECHANICAL_ASSEMBLY.md), [LOW_LEVEL_GANTRY_CONTROL.md](LOW_LEVEL_GANTRY_CONTROL.md), [CELL_NET_L2_COMMUNICATION_GUIDE.md](CELL_NET_L2_COMMUNICATION_GUIDE.md)

The pick is one problem: a box of known size arrives on a belt the gantry cannot follow in Y. The equations live in [`include/pick_intercept.h`](../include/pick_intercept.h) and are host-tested in `test/host/test_pick_intercept.cpp`. [`src/pick_scheduler.cpp`](../src/pick_scheduler.cpp) queues, times, and moves. It does not invent a second deadline.

Assumptions: the object bottom is on the belt; the jaw pads meet the width faces at the grasp height (no extra pad below that height); joint Z and world Z share a datum of 0 on this machine (`GANTRY_Z_DATUM_OFFSET_ABOVE_BED_MM` is added only at inverse kinematics).

User-facing text says **pick**. Fingers-down-early and fingers-down-late are two consequences of one clearance test.

---

## 1. Frames

Right-handed world frame, fixed to the gantry:

- **+X** across the belt, **+Y** downstream, **+Z** down. Joint Z = 0 is retracted (A015). The belt surface is the joint \(z_b =\) `col_z_min_mm` (115).
- No Y actuator. The jaw centerline is the plane \(y = y_p\) (`cv_pick_y_mm`, default 0).
- Object angle \(\theta\): long axis parallel to +Y at \(0^\circ\). Positive rotation about +Z turns that axis from +Y toward −X.

\[
\mathbf{u}(\theta) = (-\sin\theta,\ \cos\theta)
\]

\(\pm 90^\circ\) lie across the belt. Fold \(\theta\) into \([-90^\circ, 90)\) before choosing a jaw. The two jaw joints differ by \(180^\circ\) and are ranked by linear travel inside `thetalim`. Travel is linear, not wrapped. \(|\Delta\theta| < 0.1^\circ\) is no move; otherwise the target is quantized to \(0.1^\circ\) (`commandThetaDeg`). Theta runs only while joint Z is in the SAFE_Z band, including any wait in that band, and it is finished before Z leaves the band.

Vision pose is camera-relative. World position of the object top at capture:

\[
\mathbf{p}_c = \big(x_{\mathrm{off}} + x,\ \ y_{\mathrm{cam}} + y_{\mathrm{bat}},\ \ z_b - h\big)
\]

\(h =\) `height_from_belt_mm` is the top above the belt. `length_mm` is the extent along \(\mathbf{u}\) and is not a height. A detection without a finite positive width, length, and height is rejected. `cv_belt_w_mm` stores the belt width, default 300 mm, and the intercept does not read it. Gantry X is \(x_{\mathrm{off}} + x\), with `cv_x_off_mm` default 245: the collision edge `col_x_min_mm` (95) plus half of that width. Changing the width does not change the offset. Both settings apply without a reboot.

---

## 2. The object and the jaws

The object is an axis-aligned box in its own frame, then rotated by \(\theta\):

- half-length \(L/2\) along \(\mathbf{u}\)
- half-width \(w/2\) along the jaw-closing direction
- top at \(z_b - h\), bottom on the belt at \(z_b\)

The jaw axis is perpendicular to the long axis:

\[
\theta_j = \mathrm{fold}(\theta) + \theta_{\mathrm{offset}}
\]

Theta \(0^\circ\) holds the pad faces parallel to \(+Y\). Open/close is perpendicular to the pads, along \(\pm X\). The finger rectangles use that close axis, \(\theta_j + 90^\circ\).

Open jaws are two rectangles of thickness and finger width, centered \(\pm(w_{\mathrm{open}} + t)/2\) along the jaw axis. \(w_{\mathrm{open}} = 80\,\mathrm{mm}\). A sent width must satisfy \(0 < w \le 80\). Length and height-from-belt must also be finite and positive. The class id does not fill a missing size. Supported pick classes, length × width × height above the belt: AAA \(44.5 \times 10.5 \times 10.5\), AA \(50.5 \times 14.5 \times 14.5\), D \(61.5 \times 34.2 \times 34.2\), C \(50 \times 26.2 \times 26.2\), 18650 \(65 \times 18.3 \times 18.3\), 21700 \(70 \times 21.2 \times 21.2\), 9V flatface \(48 \times 27 \times 17\) (lying flat on larger face), 9V shortface \(48 \times 17 \times 27\) (resting on narrower flank), 9V bottomface \(27 \times 17 \times 48\) (upright on base), pack \(114 \times 68 \times 26\) (flat multi-cell), pack shortface \(114 \times 26 \times 68\) (on edge), laptop \(208 \times 50 \times 21\) (long pack), and power-tool \(112 \times 75 \times 80\) (brick pack). All cylindrical cells lie on their side with longitudinal axis along \(\mathbf{u}(\theta)\) (\(\theta = 0^\circ \iff\) parallel to \(+Y\)). Jaw travel to the width faces:

\[
g = \tfrac12 (w_{\mathrm{open}} - w) \ge 0
\]

At \(w = 80\), \(g = 0\): the object fills the opening. That is still a pick only when the clearance test in §4 allows the fingers to be at depth. With the jaw perpendicular to the cell and `cv_jaw_deg = 0`, the along-jaw gap equals \(g\).

### Grasp Height & Elevation Rules

Grasp height is governed by two complementary regimes depending on the physical height \(h\) of the battery above the belt datum \(z_b\):

1. **Short Cell Rule (\(h < 30\,\mathrm{mm}\))**:
   For any battery height strictly smaller than 30 mm, the grasp depth is fixed at **\(3.0\,\mathrm{mm}\) above the conveyor belt**:
   \[
   z_g = z_b - 3.0\,\mathrm{mm}
   \]
   With simulation datum \(z_b = 105.5\,\mathrm{mm}\), this evaluates to \(z_g = 102.50\,\mathrm{mm}\) (or \(112.0\,\mathrm{mm}\) with bench datum \(z_b = 115.0\,\mathrm{mm}\)). This applies to AAA, AA, 18650, 21700, C, 9V flatface (\(h=17\)), 9V shortface (\(h=27\)), pack (\(h=26\)), and laptop (\(h=21\)).

2. **Tall Cell Rule (\(h \ge 30\,\mathrm{mm}\))**:
   For batteries with height \(h \ge 30\,\mathrm{mm}\), grasp depth is measured down from the object top surface, targeting half the height capped at a maximum insertion depth of 25.4 mm (1.0 inch):
   \[
   h_g = \min(h/2,\ 25.4\,\mathrm{mm}), \qquad z_g = (z_b - h) + h_g
   \]
   This applies to 9V bottomface (\(h=48\), \(z_g = 81.50\)), pack shortface (\(h=68\), \(z_g = 62.90\)), power-tool (\(h=80\), \(z_g = 50.90\)), and D-cell (\(h=34.2\), \(z_g = 103.40\)).

Length does not enter \(z_g\). Staging ceiling \(z_{\mathrm{stage}}\) is dynamically calculated to clear the top surface of tall batteries with at least 6.0 mm overhead margin:
\[
z_{\mathrm{stage}} = \min(Z_{\mathrm{safe}},\ \max(15.0,\ z_{\mathrm{top}} - 6.0\,\mathrm{mm}))
\]

---

## 3. Belt

The belt is a scalar odometer \(s\) (mm). Speed \(v\) is piecewise constant between conveyor frames. Position at gantry time \(t\) uses the conveyor timestamp plus the smallest of the last eight receipt delays, not the receipt time of the latest frame:

\[
s(t) = s_k + v_k\,(t - t_k)
\]

World Y of the object center, and the arrival odometer:

\[
y(t) = y_p - \big(s_a - s(t)\big)
\]

\[
s_a = s_{\mathrm{capture}} + y_p - (y_{\mathrm{cam}} + y_{\mathrm{bat}})
\]

\[
t_a - t = (s_a - s(t)) / v \quad (v > 0)
\]

A repeated sequence number is ignored and does not refresh `last_frame_gantry_us`. `noteConveyor` increments the epoch when the sequence jumps by more than 32, raw displacement steps backward by more than 5 mm, or the conveyor timestamp steps backward by more than 0.500 s. A part is valid only in the epoch it was stamped in. Three displacement samples unchanged to \(10^{-6}\) mm, while both the previous and the new speed are positive, switch that piece to speed integration; each step is the previous speed times the conveyor-timestamp gap. `beltStale` is gantry receipt age greater than `kStaleAfterUs`, which is `2 * kConveyorPeriodUs`. That period is the configured 250 ms, so the window is 500 ms. It is not the measured gap between conveyor timestamps. Armed picks abort when the age is exceeded.

---

## 4. Coordinated grasp

The close axis is \(\theta_j = \theta + 90^\circ\). The belt's share along that axis is \(|\sin\theta|\). The older line \(f = |\cos\theta_j|\) is the same number, because \(\cos(\theta+90^\circ) = -\sin\theta\).

Time is zero at center arrival. With finger width \(T = 10\,\mathrm{mm}\) (close axis) and finger length \(F = 50\,\mathrm{mm}\) (along the pads):

\[
\xi = v t \cos\theta,\qquad \eta = v t \sin\theta,\qquad a(t) = 40 - q(t)
\]

The cell occupies \([\xi \pm L/2]\) by \([\eta \pm w/2]\). Each pad occupies \([a,\ a+T]\) or \([-a-T,\ -a]\) on the close axis and \([-F/2,\ F/2]\) along the pads. Those intervals are the contact test. A world-X overlap is the full-pass lane hazard. It is not a second collision test.

Jaw motion starts \(8\,\mathrm{ms}\) after the close command. Then \(q\) runs \(40\,\mathrm{mm}\) in \(190\,\mathrm{ms}\) at 6 bar (\(210.5\,\mathrm{mm/s}\)) until the pads meet. Valve-off \(17\,\mathrm{ms}\) is the later open command.

Hover and grasp, with +Z down:

\[
z_{\mathrm{hover}} = \max(35.7,\ z_b - h - 3),\qquad
z_{\mathrm{top}} = z_b - h
\]
\[
z_g = \begin{cases}
z_b - 3.0\,\mathrm{mm} & \text{if } h < 30\,\mathrm{mm} \\
z_{\mathrm{top}} + \min(h/2,\ 25.4\,\mathrm{mm}) & \text{if } h \ge 30\,\mathrm{mm}
\end{cases}
\]

Z uses symmetric constant speed \(v_{\mathrm{plunge}} = v_{\mathrm{retract}} = 800\,\mathrm{mm/s}\) and acceleration \(a_z = 20{,}000\,\mathrm{mm/s^2}\) (Kinetix 5100 drive rating \(20\,\mathrm{m/s^2} = 2.04\,g\)). All short cells (\(h < 30\,\mathrm{mm}\)) now grasp at \(99.0\,\mathrm{mm}\) (\(3.0\,\mathrm{mm}\) above belt datum \(102.0\,\mathrm{mm}\)), safely within the \(105.5\,\mathrm{mm}\) stroke limit.

The solver picks the close-command time and the Z-release time together. Lowering too soon meets a pad before grasp depth. Lowering too late misses the alongside window. A late close command meets the cell on a pad that has not yet reached the width face. Among schedules that avoid those three, the close trigger is synchronized so that the gripper is never held wide open at grasp depth. Both fingers are in tight proximity to the battery flanks ($W_{\mathrm{jaw}} = w + 1.8\,\mathrm{mm}$ upon Z arrival). At first contact with the downstream finger, the upstream finger clearance is strictly required to be less than $3.0\,\mathrm{mm}$ (calibrated invariant: $d_{\mathrm{up}} = 1.80\,\mathrm{mm}$ across all speeds $200 - 1500\,\mathrm{mm/s}$ and angles $0 - 90^\circ$). The finger material is 3D-printed PLA plastic ($\mu = 0.28$) contacting deep-drawn stainless steel / nickel-plated battery casings ($\mu = 0.35$).

### Zero-Dwell Ascent Clamping Dynamics
- **Zero dwell at grasp depth ($t_{\mathrm{retract}} = t_{\mathrm{z\_done}}$, $\mathrm{dwell\_s} = 0.0$):**
  Z does not dwell at conveyor level. As soon as Z reaches grasp depth $z_g$, it reverses instantly into upward retract at constant speed $v_{\mathrm{retract}} = v_{\mathrm{plunge}} = 800\,\mathrm{mm/s}$ and $a_z = 20{,}000\,\mathrm{mm/s^2}$.
- **Tiny Gap Precondition ($F_N = 0.0\,\mathrm{N}$ at bottom):**
  During the plunge, the grip close command is fired at $t_{\mathrm{close}} = t_{\mathrm{z\_done}} - \tau_{\mathrm{close}}$ such that when Z reaches grasp depth $z_g$, the battery is already between the closing fingers, with each finger having a tiny clearance $\delta_{\mathrm{clear}} = 0.9\,\mathrm{mm}$ ($W_{\mathrm{jaw}} = w + 1.8\,\mathrm{mm}$). Because clearance is non-zero, normal force into the battery flank at bottom is strictly zero ($F_N = 0.0\,\mathrm{N}$) and downward collision into the top is eliminated.
- **Immediate Grip Completion on Ascent:**
  As soon as Z reaches $z_g$, Z retract begins immediately. The pneumatic stroke continues integrating without interruption during the initial upward ascent. Full grasp ($W_{\mathrm{jaw}} = w$) is attained immediately after (and only after) Z retract has started.
- **Strict Firing Sequence Invariant ($\mathrm{START\_PLUNGE} \longrightarrow \mathrm{START\_GRIP} \longrightarrow \mathrm{START\_RETRACT}$):**
  The physical motion execution and control triggers strictly adhere to the causal order under all conditions and across all offset tuning combinations:
  \[
  t_z < t_{\mathrm{close}} < t_{\mathrm{retract}}
  \]
  1. $\mathrm{START\_PLUNGE}$ occurs first at $t_z$: the Z axis begins descent from staging ceiling $z_h$ toward grasp depth $z_g$.
  2. $\mathrm{START\_GRIP}$ fires second at $t_{\mathrm{close}}$: the pneumatic valve energizes while Z is descending, closing fingers toward the battery flanks.
  3. $\mathrm{START\_RETRACT}$ occurs third at $t_{\mathrm{retract}}$: Z reaches grasp depth $z_g$ after completing the full physical stroke duration $\tau_{\mathrm{plunge}} = \mathrm{trapezoid\_duration}(z_h, z_g, v_{z,\mathrm{crit}}, a_z)$ and immediately reverses with zero dwell.
  Zero premature reversal or mid-air abort is permitted ($t_{\mathrm{retract}} \ge t_z + \tau_{\mathrm{plunge}}$).
- **Constant Retract Delay Invariant ($t_{\mathrm{retract}} - t_{\mathrm{close}} \equiv \mathrm{constant}$):**
  Z retract is directly slaved to the pneumatic grip close trigger with a strictly constant delay:
  \[
  t_{\mathrm{retract}} = t_{\mathrm{close}} + \Delta t_{\mathrm{retract\_from\_close}}
  \]
  where $\Delta t_{\mathrm{retract\_from\_close,nominal}} = \tau_{\mathrm{full\_close}} - \Delta t_{\mathrm{lead}}$, and $\tau_{\mathrm{full\_close}} = t_{\mathrm{delay,valve}} + t'_{\mathrm{pneumatic}}(w)$ is the total duration from valve energization to full grasp contact on the battery flank ($W_{\mathrm{jaw}} = w$).
  By enforcing $\Delta t_{\mathrm{lead}} = 8\,\mathrm{ms}$ ($8\,\mathrm{ticks}$), **nominal Z retract begins exactly $8\,\mathrm{ms}$ before the gripper is fully closed**.
  - **Invariance & Operator Retract Tuning:**
    Across all operator tuning states $(\Delta N_{z,\mathrm{offset\_ticks}}, \Delta N_{\mathrm{offset\_ticks}}, \Delta N_{\mathrm{retract\_ticks}})$, the delay between Z retract start and grip close trigger is defined by:
    \[
    \Delta t_{\mathrm{retract\_from\_close}} = t_{\mathrm{retract}} - t_{\mathrm{close}} = \Delta t_{\mathrm{retract\_from\_close,nominal}} + \Delta N_{\mathrm{retract\_ticks}} \cdot T_{\mathrm{tick}}
    \]
    When $\Delta N_{\mathrm{retract\_ticks}} = 0$, this delay is strictly constant ($\Delta t_{\mathrm{retract\_from\_close,nominal}}$), completely independent of how $\Delta N_z$ or $\Delta N_{\mathrm{close}}$ are tuned. When $\Delta N_{\mathrm{retract\_ticks}} \neq 0$, the operator directly trims the retract trigger relative to grip close.
    \[
    t_{\mathrm{retract}} = t_{\mathrm{retract,nominal}} + (\Delta N_{z,\mathrm{offset\_ticks}} + \Delta N_{\mathrm{offset\_ticks}} + \Delta N_{\mathrm{retract\_ticks}}) \cdot T_{\mathrm{tick}}
    \]
    \[
    t_z = t_{\mathrm{retract}} - \tau_{\mathrm{plunge}}, \qquad t_{\mathrm{close}} = t_{\mathrm{retract}} - \Delta t_{\mathrm{retract\_from\_close}}
    \]
    guaranteeing that the plunge always completes the full stroke $\tau_{\mathrm{plunge}}$ down to $z_g$ before retracting.
  - **Negative Delay Relative to Full Close:**
    Relative to the instant of complete grasp contact $t_{\mathrm{full\_close}} = t_{\mathrm{close}} + \tau_{\mathrm{full\_close}}$, the nominal delay is a constant negative offset:
    \[
    \Delta t_{\mathrm{retract\_full,nominal}} = t_{\mathrm{retract,nominal}} - t_{\mathrm{full\_close,nominal}} = -\Delta t_{\mathrm{lead}} = -8.0\,\mathrm{ms}
    \]
    guaranteeing that upward Z acceleration begins just a couple of ticks prior to full squeeze, seamlessly lifting the cell during the first millimeter of vertical travel.
- **Velocity Feedforward (VFF) Lead Compensation:**
  To eliminate servo following error $\Delta z = \frac{k_v}{k_p} \dot{q}$ and prevent velocity spikes during high-speed turn-around at $z_g$, the drive position command executes 100% velocity feedforward:
  \[
  z_{\mathrm{cmd}}(t) = z(t) + \tau_{\mathrm{vff}} \dot{z}(t), \qquad \tau_{\mathrm{vff}} = \frac{k_v}{k_p} = \frac{14{,}000}{2{,}000{,}000} = 0.007\,\mathrm{s} = 7.0\,\mathrm{ms}
  \]
  In the MuJoCo/Kinetix actuator model, this feedforward exacts $F = k_p (z_{\mathrm{traj}} - q) + k_v (v_{\mathrm{traj}} - \dot{q})$, guaranteeing smooth, monotonic reversal from $+800\,\mathrm{mm/s}$ down to $-800\,\mathrm{mm/s}$ upward retract with zero transient spike.
- **Discrete FreeRTOS Three-Parameter Tick Scheduling ($1\,\mathrm{tick} = 1\,\mathrm{ms}$):**
  Firmware executes on FreeRTOS with `CONFIG_FREERTOS_HZ = 1000` ($T_{\mathrm{tick}} = 1.0\,\mathrm{ms}$). Actuators and triggers are scheduled in discrete integer ticks across three independent tuning sliders:
  \[
  t_z = t_{z,\mathrm{nominal}} + \Delta N_{z,\mathrm{offset\_ticks}} \cdot T_{\mathrm{tick}}, \qquad N_{z,\mathrm{plunge\_ticks}} = \mathrm{round}\left(\frac{t_{z}}{T_{\mathrm{tick}}}\right)
  \]
  \[
  N_{\mathrm{close\_ticks}} = \mathrm{round}\left(\frac{t_{\mathrm{close}} - t_z}{T_{\mathrm{tick}}}\right)
  \]
  \[
  N_{\mathrm{retract\_ticks}} = \mathrm{round}\left(\frac{t_{\mathrm{retract}}}{T_{\mathrm{tick}}}\right)
  \]
  where $\Delta N_{z,\mathrm{offset\_ticks}} \in [-100, +100]$, $\Delta N_{\mathrm{offset\_ticks}} \in [-100, +100]$, and $\Delta N_{\mathrm{retract\_ticks}} \in [-100, +100]$ are tunable tick offsets exposed in the simulation control panel.
  - Shifting $\Delta N_{z,\mathrm{offset\_ticks}}$ adjusts the physical instant of vertical descent start in millisecond increments along the belt travel trajectory.
  - Shifting $\Delta N_{\mathrm{offset\_ticks}}$ alters the valve energization instant.
  - Shifting $\Delta N_{\mathrm{retract\_ticks}}$ adjusts the vertical extraction retract trigger relative to grip close.
- **Firing Time in Ticks Since Assigned Battery Detection ($y_{\mathrm{cam}} = -771\,\mathrm{mm}$):**
  When a battery is sighted and assigned by the upstream optical station ($y_{\mathrm{cam}} = -771.0\,\mathrm{mm}$), nominal conveyor transit flight time to the gantry center line ($y = 0.0\,\mathrm{mm}$) is:
  \[
  \tau_{\mathrm{transit}} = \frac{|y_{\mathrm{cam}}|}{v_{\mathrm{belt}}} = \frac{771.0\,\mathrm{mm}}{v_{\mathrm{belt}}}
  \]
  The scheduled countdown firing times in discrete FreeRTOS ticks since assigned battery detection are:
  \[
  N_{z,\mathrm{detect}} = \mathrm{round}\left(\frac{\tau_{\mathrm{transit}} + t_z}{T_{\mathrm{tick}}}\right)
  \]
  \[
  N_{\mathrm{close,detect}} = \mathrm{round}\left(\frac{\tau_{\mathrm{transit}} + t_{\mathrm{close}}}{T_{\mathrm{tick}}}\right)
  \]
  \[
  N_{\mathrm{retract,detect}} = \mathrm{round}\left(\frac{\tau_{\mathrm{transit}} + t_{\mathrm{retract}}}{T_{\mathrm{tick}}}\right)
  \]
  Because $t_z < t_{\mathrm{close}} < t_{\mathrm{retract}}$, the countdown ticks strictly follow $N_{z,\mathrm{detect}} < N_{\mathrm{close,detect}} < N_{\mathrm{retract,detect}}$, and all three metrics are displayed dynamically in real time alongside their respective slider tuning cards in the simulation control panel.

Dual-Z architecture enforces two safe elevations:
- **\(z_{\mathrm{traverse}} = 35.7\,\mathrm{mm}\):** High machine traverse clearance over bin walls, gantry frame, and peripheral obstacles during transfer and return.
- **\(z_{\mathrm{safe}} = 65.0\,\mathrm{mm}\):** Conveyor pick staging ceiling, clearing the tallest cell (D-cell height \(34.2\,\mathrm{mm}\) at belt datum \(105.5\,\mathrm{mm}\)) with \(6.3\,\mathrm{mm}\) overhead margin.

At \(\theta = 0\), \(\eta = 0\), both pads meet when \(q = (80-w)/2\), and the drag is 0. Z-release uncertainty is \(5\,\mathrm{ms}\) (dispatch, the \(2\,\mathrm{ms}\) RPI, and a drive allowance). The \(10\,\mathrm{ms}\) Gantry loop is not on that edge. Close-command uncertainty is \(2\,\mathrm{ms}\). A non-positive window left after those allowances is `outside-current-limit`.

If the free path cannot capture both pads without the first pad shoving the cell, the row is `contact-assisted`. It stays in the grid for the bench. Neighbors, including packs 11–13, block a pick whose Z-down interval meets them. `pk_dropin` is not this coordinator. The grid and the counts are in [PICK_COORDINATED_GRASP_VALIDATION.md](PICK_COORDINATED_GRASP_VALIDATION.md). The bench ladder is in [PICK_BENCH_RAMP.md](PICK_BENCH_RAMP.md).

`fingersFreeFromS` is still the neighbor test in the belt plane. The part tracker keeps the epoch, dedupe, and the short list of cells already taken.

### Firmware port (2026-10-13)

The firmware strike is now `compute_strike_plan()` from `tools/sim/gantry_twin.py`, ported to [`include/strike_plan.h`](../include/strike_plan.h) and host-tested against the sim's own output (`test/host/test_strike_plan.cpp`, within 2 µs). It replaces the `CoordinatedGrasp` / early-Z / drop-in strike selection in `src/pick_scheduler.cpp` for every pick.

- **Geometry and limits come from the firmware config, not the sim constants.** Belt Z is `col_z_min_mm`, the hover ceiling is the SAFE_Z band (`Gantry::traverseClearanceZMm()`), the jaw opening is `fin_open_mm`, Z speed is `AXIS_Z_PICK_SPEED_FRACTION` of the stored ballscrew cap, Z accel and decel are `acc_mm_s2` and `dec_mm_s2`, the valve delay is `pk_von_ms`, and the grasp depth is the §2 rule from the vision height. Only the pneumatic jaw model keeps the sim's values (450 mm/s, \(\tau = 15\) ms).
- **Belt and stroke (bench, 2026-10-13):** the Z+ limit switch is set so the gripper stops less than 1 mm above the belt. The belt is therefore at the measured Z stroke, about joint Z 109.1, so `col_z_min_mm` = 109.1 rather than the older 115. The pick stroke is the Z stroke measured in this session (`Gantry::zAxisLengthMm()`). Before calibration the Z joint limit is the fallback.
- **Hover:** \(z_h = \max(z_{\min}, \min(z_{\mathrm{ceiling}}, z_{\mathrm{top}} - 6))\). The pre-position goes straight to the hover, which is inside the SAFE_Z band, so X and theta finish there.
- **Execution:** Z is preloaded and held at the hover. Three esp_timer events fire at fixed offsets from the arrival time: release the plunge, fire the grip, and the zero-dwell `retractToSafeZ`. Every conveyor frame moves the arrival, and all three follow it (`retimeArrival`).
- **Feasibility:** a pick is late when the plunge would start before the pre-position, preload and settle are done. Wider than the jaws, a non-positive size, or a grasp above the hover is permanent.
- **Operator offsets:** `pk_off_plunge`, `pk_off_close`, `pk_off_retract` (whole ms, −100…+100, live, kept in NVS; toolbox page Profile & Units → *Pick strike offsets*). Unlike the sim's sliders, **each moves only its own event**. The firmware then enforces a full plunge (\(t_{\mathrm{retract}} \ge t_z + \tau_{\mathrm{plunge}}\)) and \(t_z + 2\,\mathrm{ms} \le t_{\mathrm{close}} \le t_{\mathrm{retract}} - 2\,\mathrm{ms}\). A plan limited by those rules logs `clamped`.
- **Logs:** each pick logs the plan (`sim z/close/retract … ms nom …`), then `pick actual` with each event's firing error, the Z depth reached when the retract fired, and the offsets in use. `pk_tr_grp` and `cv_valve_ms` no longer affect the strike.

---

## 5. Motion time

A linear move of length \(D\) at speed \(V\) and accel/decel \(A, B\) is the trapezoid in `profileSeconds` (triangular when \(D\) is shorter than the ramps). A combined X–Z segment is slowed by `Path::zCappedPathSpeed` so the screw stays at or below its critical speed.

Once a vision pose is accepted the jaws stay open and the move uses two speeds.

**Approach.** At SAFE_Z, X and theta move to the pick pose on the global motion profile. The pick scheduler has no motion parameters of its own. The part being picked uses the default speed. The speed ceiling is that part's own approach, chosen earlier from the post-bin pose, and only when the default speed would miss it:

- Default: `spd_mm_s`.
- Ceiling: `ceil_spd_mm_s`.
- Both: theta at `th_spd_deg_s` (capped by `th_spd_max`), ramps from `acc_mm_s2`, `dec_mm_s2`, `th_acc_deg_s2` and `th_dec_deg_s2`.

The time from second-pad contact until the next approach can start is the retract to SAFE_Z, the transfer to the bin at the default speed, bin descent, open dwell, and retract back to SAFE_Z. A following part is skipped when neither speed reaches it from that pose. Close timing uses the 8 ms valve-on (`pk_von_ms`), the 190 ms full close, and the signed trims, all defaulting so the paper profile is unchanged. `pk_dropin` does not select this schedule.

Pre-position time is the later of the X trapezoid and the theta move (one tick plus the rotary preload, release-halt, and ack in `GantryRotaryPhases.h`), plus arrival settle.

**Z stroke.** Descend to \(z_g\), close, and retract to the SAFE_Z ceiling at the Z pick-stroke limit \(v_z = 950\,\mathrm{mm/s}\) (`AXIS_Z_PICK_SPEED_FRACTION` of the stored ballscrew cap), not at the approach speed. It sits below the ballscrew whip ceiling:

\[
v_{z,\mathrm{whip}} = 3000\,\mathrm{rpm} \times 20\,\mathrm{mm/rev} / 60 = 1000\,\mathrm{mm/s} \;>\; v_z = 950\,\mathrm{mm/s}
\]

`3000 rpm` is the SCHUNK Beta 80-SRS whip ceiling (`AXIS_Z_CRITICAL_RPM`), not a motor nameplate. The pick stroke is Z-only, so it is commanded at \(v_z\). `retractToSafeZ` is called with \(v_z\). Do not raise the whip ceiling. If the live Kinetix velocity limit is below 1000 mm/s, raise that drive limit up to \(v_z\) and no further.

The pick is feasible at time \(t\) when the approach finishes and the §4 pair still has a positive robust window relative to \(t_a(t)\). After X and theta are committed, a new conveyor sample recomputes that pair from the new travel and speed. It is not a new approach from the staging pose.

On belt stop, stale frames, restart, a refined angle that changes the clearance, a grip-write failure, or a missed deadline, the scheduler calls `Gantry::retractToSafeZ()` at \(v_z\).

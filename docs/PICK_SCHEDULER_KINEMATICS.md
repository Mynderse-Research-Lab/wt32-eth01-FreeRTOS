# Pick Scheduler Kinematics & Timing Algorithms

**Status:** Canonical mathematical design basis  
**Companion Docs:** [EXPECTED_ELECTROMECHANICAL_ASSEMBLY.md](EXPECTED_ELECTROMECHANICAL_ASSEMBLY.md), [LOW_LEVEL_GANTRY_CONTROL.md](LOW_LEVEL_GANTRY_CONTROL.md), [CELL_NET_L2_COMMUNICATION_GUIDE.md](CELL_NET_L2_COMMUNICATION_GUIDE.md)

This document contains the authoritative mathematical requirements, coordinate frames, and feasibility algorithms for the gantry's PickScheduler (planPick).

---

## 1. Coordinate Convention (Firmware-Wide)

- **X (Across-belt)**: Horizontal traverse along the gantry beam, across the conveyor belt. 
- **Y (Along-belt)**: Along-belt direction. The gantry has **no Y actuator**. Conveyor downstream is the **+Y direction**.
- **Z (Vertical)**: Gantry descent axis. **+Z = down** (toward the belt). Joint Z=0 is A015 (retracted/safe height); pick height is toward joint max / A014.
- **Theta (Rotation)**: Rotation about Z, right-handed about +Z.

---

## 2. Along-Belt Frame and Target Timing

These conventions are mirrored in include/conveyor_intercept_params.h.

### Along-belt axis y (mm)
- **Origin:** Belt **roller axis** datum projected into the belt plane.
- **+Y:** **Downstream** along the belt — the direction of conveyor motion that carries the battery **toward the pickup plane**.
- **Pickup is downstream of the camera** (y_pick_mm > y_cam_mm > 0 in this frame).

### Vertical axis z (mm)
- **+Z = down**. Joint Z=0 is A015 (retracted). z_pick is toward the belt; z_safe is the traverse/retract band (GANTRY_SAFE_Z_HEIGHT_MM = 35.7 mm from Z- / A015).

### Measured Along-Belt Distances
| Symbol | Approx. value (mm) | Meaning |
|--------|-------------------|---------|
| y_cam_mm  | **+336.55** | Along-belt position of the **camera optical center** projected onto the belt. |
| y_pick_mm | **+1016**   | Along-belt position of the **gantry pickup plane**. |

Fixed span from camera to pickup:

$$
L_{\mathrm{cam}\rightarrow\mathrm{pick}} = y_{\mathrm{pick}} - y_{\mathrm{cam}} \approx 679.45\ \mathrm{mm}
$$

---

## 3. Feasibility and Intercept Math (planPick)

The identification system publishes **y_bat_mm** (along-belt coordinate) and **x_across_mm** (across-belt coordinate) at timestamp $t_{\text{epoch\_us}}$.

### Intercept Distance

The distance the battery must travel to reach the pickup plane is:

$$
D_{\mathrm{mm}} = y_{\mathrm{pick,mm}} - y_{\mathrm{bat,mm}}
$$

- **Feasible:** D_mm > 0 (battery is upstream of pick line).
- **Infeasible (SKIP):** D_mm <= 0 (past pickup plane).

### Time to Intercept ($\tau$)

The time until the battery reaches the pickup plane is computed using the absolute belt speed ($v_{\text{belt}}$):

$$
\tau \approx \frac{D_{\mathrm{mm}}}{v_{\mathrm{belt}}}
$$

(e.g., at $v \approx 1500 \text{ mm/s}$ with $D = 679.45 \text{ mm}$, $\tau \approx 0.45 \text{ s}$)

### Across-Belt Mapping (X)
The gantry **X-joint target** is derived through a single calibration offset:

$$
X_{\mathrm{target}} = \text{CONVEYOR\_X\_ACROSS\_TO\_GANTRY\_X\_OFFSET\_MM} \pm x_{\mathrm{across,mm}}
$$

---

## 4. Pick State Machine (Motion Sequencing)

When planPick confirms feasibility, the scheduler commands the Gantry API:

1. **APPROACH**: moveTo(x_target, z_safe_mm, theta_target). Wait for arrival.
2. **WAIT_DEADLINE**: Block until local_time >= t_pick_local_us - grip_margin.
3. **DESCEND**: moveTo(x_target, z_pick_mm, theta_target).
4. **GRIP**: grip(true); wait for pneumatic actuation delay.
5. **RETRACT**: moveTo(x_target, z_safe_mm, theta_target).
6. **TRANSFER**: moveTo(bin_x, z_safe_mm, bin_theta).
7. **RELEASE**: grip(false).

*(If any step fails, the system transitions to ABORT and triggers requestAbort())*

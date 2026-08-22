# Mechanical Datum Export Guide (SSOT)

This guide provides the mechanical engineering (MECH) team with the exact steps to export the cell's physical topology. 

**Why are we doing this?**
The firmware for the WT32-ETH01 gantry controller does not use hardcoded mechanical values. Instead, during compilation, it reads a `datums.csv` file from the network drive. If you change the length of the gripper jaws or move the camera mount, you simply update the CAD and overwrite the CSV. The firmware will automatically adjust its kinematics and crash-prevention limits based on your new geometry.

---

## Step 1: Establish the Firmware Coordinate System
CAD software typically defaults to Z-up, but the firmware kinematics require a specific frame. In your top-level CAD assembly, create a **Reference Coordinate System** named `FIRMWARE_ORIGIN` with the following orientation:
* **Origin (0,0,0):** A fixed structural point (e.g., a corner of the main conveyor frame).
* **+X Axis:** Across the conveyor belt (along the gantry span).
* **+Y Axis:** Downstream along the conveyor belt.
* **+Z Axis:** Pointing **DOWN** toward the conveyor belt surface.

*All points exported in Step 3 MUST be evaluated relative to this `FIRMWARE_ORIGIN` coordinate system.*

---

## Step 2: Define the Datum Points
Place Reference Points (Datums) in your assembly at the following physical locations. **You must name the points exactly as written below** in your CAD feature tree.

| Exact Feature Name | Physical Placement in CAD | Why Firmware Needs It |
| :--- | :--- | :--- |
| `DATUM_CELL_ORIGIN` | Coincident with `FIRMWARE_ORIGIN` (0,0,0). | Validates the export offset. |
| `DATUM_CAM_LENS` | Optical center of the vision camera lens. | Sets the upstream detection Y-coordinate. |
| `DATUM_PICK_PLANE` | Center line of the gantry's reach across the belt. | Defines distance $D$ for intercept timing. |
| `DATUM_AXIS_X_HOME` | Trigger point of the X-axis negative limit switch. | Establishes the X-axis zero post-homing. |
| `DATUM_AXIS_X_MAX` | Trigger point of the X-axis positive limit switch. | Sets firmware soft-limits to prevent crashes. |
| `DATUM_AXIS_Z_HOME` | Trigger point of the Z-axis upper (retracted) limit. | Defines the safe fly-over height. |
| `DATUM_AXIS_Z_BELT` | On the surface of the belt directly beneath Z. | Absolute floor limit to prevent belt strikes. |
| `DATUM_TOOL_CENTER_POINT`| The physical tip of the gripper/vacuum cup. | Automatically adjusts Z-dive depth if tooling changes. |

---

## Step 3: Export the Point Table
Create a point evaluation table or BOM in your CAD software (SolidWorks, Inventor, Fusion360) that measures the X, Y, and Z distances of the 8 points above **relative to `FIRMWARE_ORIGIN`**. 

Ensure your export settings meet these rules:
1. **Units:** Millimeters (mm).
2. **Format:** CSV (Comma Separated Values).
3. **Columns:** The export must contain exactly these headers: `Datum_Name, X_mm, Y_mm, Z_mm, Description`

*Example Export:*
```csv
Datum_Name,X_mm,Y_mm,Z_mm,Description
DATUM_CAM_LENS,500.0,336.55,-800.0,Overhead Vision Camera Center
DATUM_TOOL_CENTER_POINT,0.0,0.0,152.4,Tip of pneumatic gripper
```

---

## Step 4: Publish to the Network
Save or overwrite the exported file to the shared engineering network drive at the agreed location:
`\\SERVER\Engineering\Cell_Config\datums.csv`

**You are done.** 
The CI/CD pipeline and firmware developers will pull this file automatically over the network during the next software build. Your physical design changes are now live in the robot's brain.

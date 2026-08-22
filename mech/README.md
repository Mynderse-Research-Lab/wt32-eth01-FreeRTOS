# Mechanical Datum Export Guide (SSOT)

This guide provides the mechanical engineering (MECH) team with the exact steps to export the cell's physical topology. 

**Why are we doing this?**
The firmware for the WT32-ETH01 gantry controller does not use hardcoded mechanical values. Instead, during compilation, it reads a `datums.csv` file from the network drive. If you change the length of the gripper jaws or move the camera mount, you simply update the CAD and hit the export macro. The firmware will automatically adjust its kinematics and crash-prevention limits based on your new geometry. Furthermore, the Foxglove 3D visualizer will automatically update its models.

---

## Step 1: Establish the Firmware Coordinate System
CAD software typically defaults to Z-up, but the firmware kinematics require a specific frame. In your top-level SolidWorks assembly, create a **Coordinate System** named `FIRMWARE_ORIGIN` with the following orientation:
* **Origin (0,0,0):** A fixed structural point (e.g., a corner of the main conveyor frame).
* **+X Axis:** Across the conveyor belt (along the gantry span).
* **+Y Axis:** Downstream along the conveyor belt.
* **+Z Axis:** Pointing **DOWN** toward the conveyor belt surface.

*All datums and meshes exported in Step 3 will be evaluated relative to this `FIRMWARE_ORIGIN`.*

---

## Step 2: Define the Target Datums
Place Coordinate Systems in your assembly at the following physical locations. **You must name the coordinate systems exactly as written below** in your CAD feature tree.

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

## Step 3: Run the SolidWorks Macro (Dual-Export)
Instead of manually measuring these points and aligning 3D meshes by hand, we have provided a SolidWorks VBA macro: `ExportFirmwareDatums.swp`.

1. Add the macro to your SolidWorks toolbar.
2. Click the macro button.

**What the macro does automatically:**
1. **The CSV:** It traverses your feature tree, extracts the X/Y/Z (in mm) and Roll/Pitch/Yaw (in degrees) for every `DATUM_` relative to `FIRMWARE_ORIGIN`, and formats it into `datums.csv`.
2. **The STLs:** It selects the corresponding subassemblies (like the gripper tooling or camera housing) and exports them as `.STL` files. Crucially, it instructs SolidWorks to use the local `DATUM_` coordinate system as the `0,0,0` origin for that specific STL.

---

## Step 4: Publish to the Network
The macro will prompt you to save the output. Select the shared engineering network drive at the agreed location:
`\\SERVER\Engineering\Cell_Config\`

**You are done.** 
The CI/CD pipeline and firmware developers will pull this file automatically over the network during the next software build. Your physical design changes are now live in the robot's brain, and your perfectly aligned CAD meshes are live in the Foxglove 3D shadow mode visualizer.

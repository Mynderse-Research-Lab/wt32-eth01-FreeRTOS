# Mechanical Topology (SSOT)

This directory acts as the bridge to the Mechanical Engineering (MECH) team's physical topology of the battery sorting cell. 

The master coordinate data (`datums.csv`) is maintained by MECH and lives externally on the Engineering Network Drive, rather than being tracked directly in this git repository.

### Workflow:
1. **MECH:** In the CAD assembly, create named Reference Datums or Coordinate Systems for key components (e.g., `DATUM_CAM_LENS`, `DATUM_AXIS_X_HOME`).
2. **MECH:** Export a point-coordinate table (CSV) of these datums relative to the global cell origin. Save this `datums.csv` to the designated MECH network drive. Ensure values are in **millimeters**.
3. **FIRMWARE / CI:** When building or updating the codebase, generate the local SDF file by pointing the generator script to the network drive:

```bash
# Example via Command Line Argument:
python tools/generate_topology_sdf.py --csv "Z:\Shared\Engineering\datums.csv"

# Example via Environment Variable:
set MECH_DATUMS_CSV=Z:\Shared\Engineering\datums.csv
python tools/generate_topology_sdf.py
```

### What happens next?
The script reads the network CSV, converts the millimeters to SI meters, and injects them into the local `gantry_cell.sdf.template` to output `gantry_cell.sdf`. 

This generated `.sdf` file provides the exact transformations required by the firmware Pick Scheduler and Gazebo simulation without hardcoding magic numbers.

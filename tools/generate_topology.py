#!/usr/bin/env python3
"""
generate_topology.py
Reads the MECH-exported `datums.csv` containing X/Y/Z points in millimeters
from a specified network location or path, converts them to SI meters, and 
injects them into `mech/gantry_cell.urdf.template` to generate the final 
Unified Robot Description Format (URDF) file for Foxglove Shadow Mode.
"""

import csv
import os
import sys
import argparse
import math

def main():
    parser = argparse.ArgumentParser(description="Generate URDF from MECH datums CSV.")
    parser.add_argument('--csv', type=str, 
                        help='Path to the datums.csv (e.g., on a network drive)', 
                        default=os.environ.get('MECH_DATUMS_CSV'))
    args = parser.parse_args()

    if not args.csv:
        print("Error: No CSV path provided.")
        print("Usage: python generate_topology.py --csv \\\\network_share\\MECH\\datums.csv")
        print("Or set the MECH_DATUMS_CSV environment variable.")
        sys.exit(1)

    csv_path = args.csv
    if not os.path.exists(csv_path):
        print(f"Error: Cannot find CSV file at {csv_path}")
        sys.exit(1)

    # Setup paths relative to this script for the template and output
    script_dir = os.path.dirname(os.path.abspath(__file__))
    mech_dir = os.path.join(script_dir, '..', 'mech')
    
    template_path = os.path.join(mech_dir, 'gantry_cell.urdf.template')
    output_path = os.path.join(mech_dir, 'gantry_cell.urdf')

    if not os.path.exists(template_path):
        print(f"Error: Missing template at {template_path}")
        sys.exit(1)

    # 1. Read datums
    datums = {}
    print(f"Reading datums from network location: {csv_path}...")
    with open(csv_path, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            name = row['Datum_Name'].strip()
            # Convert mm to meters for URDF
            x_m = float(row['X_mm']) / 1000.0
            y_m = float(row['Y_mm']) / 1000.0
            z_m = float(row['Z_mm']) / 1000.0
            
            # Convert degrees to radians for URDF
            roll_rad = math.radians(float(row.get('Roll_deg', 0.0)))
            pitch_rad = math.radians(float(row.get('Pitch_deg', 0.0)))
            yaw_rad = math.radians(float(row.get('Yaw_deg', 0.0)))
            
            # Save into dictionary
            datums[f"{name}_X"] = f"{x_m:.4f}"
            datums[f"{name}_Y"] = f"{y_m:.4f}"
            datums[f"{name}_Z"] = f"{z_m:.4f}"
            datums[f"{name}_ROLL"] = f"{roll_rad:.4f}"
            datums[f"{name}_PITCH"] = f"{pitch_rad:.4f}"
            datums[f"{name}_YAW"] = f"{yaw_rad:.4f}"

    # 2. Read template
    with open(template_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # 3. Replace placeholders
    for key, value in datums.items():
        placeholder = f"{{{{{key}}}}}"
        content = content.replace(placeholder, value)

    # 4. Write output
    with open(output_path, 'w', encoding='utf-8') as f:
        f.write(content)
        
    print(f"Successfully generated {output_path}.")

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
generate_topology_sdf.py
Reads the MECH-exported `datums.csv` from a specified network drive,
converts the X/Y/Z points in millimeters to SI meters, and injects them into 
`mech/gantry_cell.sdf.template` to generate the final Simulation Description 
Format (SDF) file.
"""

import csv
import os
import argparse
import sys

def main():
    parser = argparse.ArgumentParser(description="Generate SDF from MECH network datums CSV.")
    parser.add_argument(
        '--csv', 
        type=str, 
        help='Path to the datums.csv file on the network drive. Defaults to MECH_DATUMS_CSV env variable.',
        default=os.environ.get('MECH_DATUMS_CSV')
    )
    args = parser.parse_args()

    if not args.csv:
        print("Error: You must specify --csv or set the MECH_DATUMS_CSV environment variable.")
        sys.exit(1)
        
    if not os.path.exists(args.csv):
        print(f"Error: Could not find network CSV at {args.csv}")
        sys.exit(1)

    # Setup paths relative to this script for the template and output
    script_dir = os.path.dirname(os.path.abspath(__file__))
    mech_dir = os.path.join(script_dir, '..', 'mech')
    template_path = os.path.join(mech_dir, 'gantry_cell.sdf.template')
    output_path = os.path.join(mech_dir, 'gantry_cell.sdf')

    if not os.path.exists(template_path):
        print(f"Error: Missing template at {template_path}")
        sys.exit(1)

    # 1. Read datums from Network Drive
    datums = {}
    print(f"Reading datums from network drive: {args.csv}...")
    try:
        with open(args.csv, 'r', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            for row in reader:
                name = row['Datum_Name'].strip()
                # Convert mm to meters for SDF
                x_m = float(row['X_mm']) / 1000.0
                y_m = float(row['Y_mm']) / 1000.0
                z_m = float(row['Z_mm']) / 1000.0
                
                # Save into dictionary with _X, _Y, _Z suffixes
                datums[f"{name}_X"] = f"{x_m:.4f}"
                datums[f"{name}_Y"] = f"{y_m:.4f}"
                datums[f"{name}_Z"] = f"{z_m:.4f}"
    except Exception as e:
        print(f"Failed to read or parse CSV: {e}")
        sys.exit(1)

    # 2. Read local template
    with open(template_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # 3. Replace placeholders
    for key, value in datums.items():
        placeholder = f"{{{{{key}}}}}"
        content = content.replace(placeholder, value)

    # 4. Write output locally
    with open(output_path, 'w', encoding='utf-8') as f:
        f.write(content)
        
    print(f"Successfully generated {output_path}.")

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
Tuning Tool & Wizard for SCHUNK ERD-04 Theta Axis (Bosch-Rexroth HCS01)
Supports:
  1. Automated Drive-Internal Autotune (C1800 Automatic Control Loop Setting & Inertia Identification)
  2. Guided Heuristic Tuning Wizard (Kp, Tn, Kv acoustic sweep from Optimization_Manual_Axis.pdf)

Target Payload: End-effector mounted on Theta rotary flange (up to 2.0 kg payload).
"""

from __future__ import annotations

import argparse
import sys
import time

try:
    from hcs01_comws import DEFAULT_HOST, ComwsError, Hcs01Comws
    COMWS_AVAILABLE = True
except ImportError:
    COMWS_AVAILABLE = False


def prompt_float(prompt_text: str) -> float:
    while True:
        try:
            val = input(prompt_text + " ")
            return float(val)
        except ValueError:
            print("Please enter a valid number.")


def run_automated_autotune(host: str = "192.168.1.22", damping: float = 1.0, travel: float = 45.0, save: bool = True) -> int:
    if not COMWS_AVAILABLE:
        print("ERROR: hcs01_comws.py module not found in path.")
        return 1

    print("=" * 65)
    print(" AUTOMATED HCS01 / ERD-04 INERTIA AUTO-TUNING (C1800)")
    print("=" * 65)
    print(f"Target Controller IP : {host}")
    print(f"Damping Factor       : {damping} (P-0-0163)")
    print(f"Test Travel Stroke   : +/-{travel} deg (P-0-0169)")
    print(f"Auto-Save to Flash   : {'YES (C2200)' if save else 'NO'}")
    print("\nSAFETY WARNING: Motor will perform test oscillation movements.")
    print("Ensure 2.0 kg end-effector is rigidly attached and has clearance.")
    
    cli = Hcs01Comws(host=host)
    try:
        cli.login()
    except Exception as e:
        print(f"ERROR connecting to HCS01 at {host}: {e}")
        return 1

    print("\n[Step 1/3] Reading initial drive status...")
    try:
        diag = cli.getvar("S-0-0095.0.0")
        print(f"  Drive Diagnostic: {diag}")
    except Exception as e:
        print(f"  Warning reading diagnostic: {e}")

    print("\n[Step 2/3] Configuring excitation & triggering C1800 identification...")
    try:
        cli.setvar_try_element7("P-0-0163.0.0", str(damping))
        cli.setvar_try_element7("P-0-0169.0.0", str(travel))
        print("  Triggering C1800 (P-0-0162.0.0)... (this may take up to 30 seconds)")
        cli.run_command("P-0-0162.0.0", timeout_s=120.0)
        print("  C1800 Execution Complete!")
    except ComwsError as e:
        print(f"  ERROR during C1800: {e}")
        return 1

    print("\n[Step 3/3] Reading identified inertia and tuned control parameters...")
    try:
        inertia = cli.getvar("P-0-4010.0.0")
        kp = cli.getvar("S-0-0100.0.0")
        tn = cli.getvar("S-0-0101.0.0")
        kv = cli.getvar("S-0-0104.0.0")
        ka = cli.getvar("S-0-0348.0.0")
        print("-" * 65)
        print(f"  * Total Load Inertia (P-0-4010)       : {inertia} kg*m^2")
        print(f"  * Velocity Loop Proportional Kp (S-0-0100): {kp}")
        print(f"  * Velocity Loop Integral Tn (S-0-0101)    : {tn} ms")
        print(f"  * Position Loop Proportional Kv (S-0-0104): {kv} 1/s")
        print(f"  * Acceleration Feedforward Ka (S-0-0348)  : {ka} %")
        print("-" * 65)
    except Exception as e:
        print(f"  Warning reading results: {e}")

    if save:
        print("\nSaving parameters to non-volatile flash (C2200 S-0-0264)...")
        try:
            cli.run_command("S-0-0264.0.0", timeout_s=60.0)
            print("  Save to NV flash complete!")
        except Exception as e:
            print(f"  Save failed: {e}")

    print("\nAuto-Tuning Successfully Finished!")
    return 0


def run_manual_heuristic_wizard():
    print("=" * 60)
    print(" SCHUNK ERD-04 (Theta) Heuristic Tuning Wizard ")
    print(" (Target payload: ~2.0 kg)")
    print("=" * 60)
    print("\n[PREPARATION]")
    print("1. Connect Computer with IndraWorks DS software to the motion controller.")
    print("2. Open the tree to the Theta axis control loop.")
    print("3. Command the axis to move continuously in reverse at very low speed (e.g. 5-10 deg/s).")
    input("Press Enter when the axis is moving slowly...")

    print("\n[STEP 1: Tune Kp (Velocity Loop Proportional Gain)]")
    print("1. Set Tn to 0 (zero). If the axis stops moving, set Tn to 10.")
    print("2. Increase Kp slowly, step-by-step, until the axis starts growling audibly.")
    print("3. Decrease Kp slowly until the growling JUST stops.")
    growl_kp = prompt_float("What is the Kp value where it stopped growling?")
    final_kp = (growl_kp / 2.0) * 1.10
    print(f"\n=> Computed Kp: {final_kp:.4f}")
    print(f"-> Enter {final_kp:.4f} into the Kp tag in IndraWorks.")
    input("Press Enter when done...")

    print("\n[STEP 2: Tune Tn (Velocity Loop Integral Time)]")
    print("1. Set Tn to 10.0.")
    print("2. Decrease Tn slowly, step-by-step, until the axis starts growling audibly.")
    print("3. Increase Tn slowly until the growling JUST stops.")
    growl_tn = prompt_float("What is the Tn value where it stopped growling?")
    final_tn = (growl_tn * 2.0) * 0.90
    print(f"\n=> Computed Tn: {final_tn:.4f}")
    print(f"-> Enter {final_tn:.4f} into the Tn tag in IndraWorks.")
    input("Press Enter when done...")

    print("\n[STEP 3: Tune Kv (Position Loop Proportional Gain)]")
    print("1. Take the Kv default value and increase it slowly, step-by-step, until growling starts.")
    print("2. Decrease Kv slowly until the growling JUST stops.")
    growl_kv = prompt_float("What is the Kv value where it stopped growling?")
    final_kv = (growl_kv / 2.0) * 1.10
    print(f"\n=> Computed Kv: {final_kv:.4f}")
    print(f"-> Enter {final_kv:.4f} into the Kv tag in IndraWorks.")

    print("\n" + "=" * 60)
    print(" TUNING COMPLETE ")
    print("=" * 60)
    print("Final Values to save in IndraWorks:")
    print(f"  Kp (Velocity Prop Gain) : {final_kp:.4f}")
    print(f"  Tn (Velocity Integral)  : {final_tn:.4f}")
    print(f"  Kv (Position Prop Gain) : {final_kv:.4f}")
    print("\nIMPORTANT: Don't forget to save the parameter-files from the motion")
    print("controller to your computer and archive them!")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="SCHUNK ERD-04 Theta Axis Inertial Tuning")
    parser.add_argument("--autotune", action="store_true", help="Execute automated drive-level C1800 autotune without human intervention")
    parser.add_argument("--host", default="192.168.1.22", help="HCS01 engineering IP (default 192.168.1.22)")
    parser.add_argument("--damping", type=float, default=1.0, help="Damping factor P-0-0163 (default 1.0)")
    parser.add_argument("--travel", type=float, default=45.0, help="Test travel distance P-0-0169 in deg (default 45.0)")
    parser.add_argument("--no-save", action="store_true", help="Do not automatically save parameters to NV flash")
    args = parser.parse_args(argv)

    if args.autotune:
        return run_automated_autotune(host=args.host, damping=args.damping, travel=args.travel, save=not args.no_save)

    print("=" * 60)
    print(" SCHUNK ERD-04 Theta Tuning Options ")
    print("=" * 60)
    print("1. Automated Driver Auto-Tune (Zero-intervention C1800 over Ethernet)")
    print("2. Guided Manual Acoustic Heuristic (Optimization_Manual_Axis.pdf)")
    choice = input("Select mode (1 or 2, default 1): ").strip()
    if choice == "2":
        run_manual_heuristic_wizard()
        return 0
    else:
        return run_automated_autotune(host=args.host, damping=args.damping, travel=args.travel, save=not args.no_save)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\nAborted.")
        sys.exit(1)

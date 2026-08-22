#!/usr/bin/env python3
"""Kinetix 5100 Position Absolute golden sequence (OM=1, TM=2, Abs).

Bench path that replaces Speed+TM10 host-side StopMotion for accurate PTP.

Golden timeline (assembly 104 / 154, Class 1, RPI 5 ms):

  1. Settle     OM=0  TM=10 ServoOn=1  StartMotion=0   until Active+Ready
  2. Home unlock OM=3 TM=2  HomingMethod=34 (define current as origin)
                preload StartMotion=0 → edge StartMotion=1 → wait HomedStatus
  3. Absolute   OM=1  TM=2  NonCyclicMoveType=0 Absolute
                PositionReference=target_PUU, Speed/Accel/Decel refs set
                preload StartMotion=0 → edge StartMotion=1
  4. Done       wait AtReference (or |err|<band and nearly stopped)
  5. Hold       OM=0  TM=10 ServoOn=1 at current/target PUU

Usage (PC on EIP daisy-chain, exclusive with WT32 W5500 uplink):

  py tools/eip_position_abs.py --ip 192.168.1.20 --delta-mm 20 --puu-per-mm 52428.8
  py tools/eip_position_abs.py --ip 192.168.1.21 --delta-mm 20 --puu-per-mm 104857.6 \\
      --speed-mm-s 200 --accel-mm-s2 3000

Exit 0 on |final_error_mm| <= --tol-mm (default 1.0).
"""

from __future__ import annotations

import argparse
import socket
import sys
import time
from pathlib import Path

# Allow `py tools/eip_position_abs.py` without installing the package.
sys.path.insert(0, str(Path(__file__).resolve().parent))

import eip_test as eip  # noqa: E402


def rpm_from_mm_s(mm_s: float, lead_mm: float, ratio: float) -> float:
    # motor_rpm = mm_s * ratio / lead * 60
    if lead_mm <= 0:
        return 0.0
    return float(mm_s) * float(ratio) / float(lead_mm) * 60.0


def rpm_s_from_mm_s2(mm_s2: float, lead_mm: float, ratio: float) -> float:
    if lead_mm <= 0:
        return eip.DEFAULT_ACCEL_RPM_PER_S
    return float(mm_s2) * float(ratio) / float(lead_mm) * 60.0


def exchange(client, ot_id: int, cmd: bytes, timeout: float = 2.0):
    to = client.exchange_io_frame(ot_id, cmd, include_run_idle=True, timeout=timeout)
    if to is None:
        return None
    st = eip.parse_input_assembly_154(to)
    if "error" in st:
        return None
    return st


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--ip", required=True, help="Drive IP (X=.20 Z=.21 typical)")
    ap.add_argument("--rpi-us", type=int, default=5000)
    ap.add_argument("--puu-per-mm", type=float, required=True)
    ap.add_argument("--lead-mm", type=float, default=None,
                    help="Lead mm/rev (default: infer 200 if puu~52k else 20)")
    ap.add_argument("--ratio", type=float, default=None,
                    help="Gear ratio motor:load (default: 5 if X-like scale else 1)")
    ap.add_argument("--delta-mm", type=float, default=20.0)
    ap.add_argument("--speed-mm-s", type=float, default=50.0)
    ap.add_argument("--accel-mm-s2", type=float, default=3000.0)
    ap.add_argument("--tol-mm", type=float, default=1.0)
    ap.add_argument("--settle-s", type=float, default=2.0)
    ap.add_argument("--home-s", type=float, default=2.0)
    ap.add_argument("--move-timeout-s", type=float, default=30.0)
    args = ap.parse_args()

    lead = args.lead_mm
    ratio = args.ratio
    if lead is None or ratio is None:
        # Heuristic from project defaults.
        if args.puu_per_mm > 200000:
            lead = lead or 20.0
            ratio = ratio or 1.0
        else:
            lead = lead or 200.0
            ratio = ratio or 5.0

    speed_rpm = rpm_from_mm_s(args.speed_mm_s, lead, ratio)
    accel_rpm_s = rpm_s_from_mm_s2(args.accel_mm_s2, lead, ratio)
    decel_rpm_s = accel_rpm_s

    print("=== Position Absolute golden sequence ===")
    print(f"  target {args.ip}  delta={args.delta_mm} mm  "
          f"V={args.speed_mm_s} mm/s ({speed_rpm:.1f} RPM)  "
          f"A={args.accel_mm_s2} mm/s^2 ({accel_rpm_s:.1f} RPM/s)")
    print(f"  scale={args.puu_per_mm} PUU/mm  lead={lead}  i={ratio}")
    print()
    print("  Timeline:")
    print("    1 Settle   OM=0 TM=10 ServoOn until Active")
    print("    2 Home34   OM=3 TM=2 HomingMethod=34 until HomedStatus")
    print("    3 Abs move OM=1 TM=2 NonCyclic=0 Absolute StartMotion edge")
    print("    4 Done     AtReference / in-band")
    print("    5 Hold     OM=0 TM=10")
    print()

    client = eip.EipClient(args.ip)
    if not client.connect():
        print("[FAIL] TCP connect")
        return 2
    if not client.register_session():
        print("[FAIL] RegisterSession")
        client.disconnect()
        return 2

    params = eip.ForwardOpenParams()
    params.to_rpi_us = args.rpi_us
    params.ot_rpi_us = args.rpi_us
    params.raw_ot_conn_size = 40 + 2 + 4
    params.include_run_idle_header = True
    result = client.forward_open(params)
    if result is None:
        print("[FAIL] ForwardOpen")
        client.unregister_session()
        client.disconnect()
        return 2
    ot_id = result.get("ot_connection_id", 0)

    try:
        # --- 1 Settle ---
        print("[1] Settle OM=0 TM=10 ...")
        t0 = time.time()
        st = None
        while time.time() - t0 < args.settle_s:
            cmd = eip.build_output_assembly_104(
                servo_on=True, operating_mode=0, travel_mode=10,
                torque_ramp_time_ms=1000)
            st = exchange(client, ot_id, cmd)
            if st and st.get("active") and st.get("ready") and not st.get("fault"):
                break
            time.sleep(args.rpi_us / 1e6)
        if not st or not st.get("active"):
            print(f"[FAIL] Settle: active={st.get('active') if st else None} "
                  f"{eip.format_status_fault_summary(st) if st else ''}")
            return 3
        print(f"  Active pos={st['actual_position']} Homed={st.get('homed_status')}")

        # --- 2 Home method 34 ---
        if not st.get("homed_status"):
            print("[2] Home unlock HomingMethod=34 ...")
            pos = int(st["actual_position"])
            preload = eip.build_output_assembly_104(
                servo_on=True, operating_mode=3, travel_mode=2,
                homing_method=34, home_return_speed_rpm=6.0,
                speed_rpm=60.0, accel_rpm_per_s=accel_rpm_s,
                decel_rpm_per_s=decel_rpm_s, position_puu=pos,
                start_motion=False, torque_ramp_time_ms=1000)
            for _ in range(4):
                st = exchange(client, ot_id, preload) or st
                time.sleep(args.rpi_us / 1e6)
            start = eip.build_output_assembly_104(
                servo_on=True, operating_mode=3, travel_mode=2,
                homing_method=34, home_return_speed_rpm=6.0,
                speed_rpm=60.0, accel_rpm_per_s=accel_rpm_s,
                decel_rpm_per_s=decel_rpm_s, position_puu=pos,
                start_motion=True, torque_ramp_time_ms=1000)
            st = exchange(client, ot_id, start) or st
            time.sleep(args.rpi_us / 1e6)
            hold = eip.build_output_assembly_104(
                servo_on=True, operating_mode=3, travel_mode=2,
                homing_method=34, home_return_speed_rpm=6.0,
                speed_rpm=60.0, accel_rpm_per_s=accel_rpm_s,
                decel_rpm_per_s=decel_rpm_s, position_puu=pos,
                start_motion=False, torque_ramp_time_ms=1000)
            t0 = time.time()
            while time.time() - t0 < args.home_s:
                st = exchange(client, ot_id, hold) or st
                if st and st.get("homed_status"):
                    break
                time.sleep(args.rpi_us / 1e6)
            if not st or not st.get("homed_status"):
                print(f"[FAIL] HomedStatus not set: {eip.format_status_fault_summary(st) if st else ''}")
                return 4
            print(f"  Homed pos={st['actual_position']}")
        else:
            print("[2] HomedStatus already set — skip Home34")

        # Return to settle hold before Position Absolute.
        settle = eip.build_output_assembly_104(
            servo_on=True, operating_mode=0, travel_mode=10,
            position_puu=int(st["actual_position"]), torque_ramp_time_ms=1000)
        for _ in range(4):
            st = exchange(client, ot_id, settle) or st
            time.sleep(args.rpi_us / 1e6)

        start_puu = int(st["actual_position"])
        delta_puu = int(round(args.delta_mm * args.puu_per_mm))
        target_puu = start_puu + delta_puu
        print(f"[3] Absolute {start_puu} -> {target_puu} PUU ({args.delta_mm} mm)")

        preload = eip.build_output_assembly_104(
            servo_on=True, operating_mode=1, travel_mode=2,
            non_cyclic_move_type=0, position_puu=target_puu,
            speed_rpm=speed_rpm, accel_rpm_per_s=accel_rpm_s,
            decel_rpm_per_s=decel_rpm_s, start_motion=False,
            torque_ramp_time_ms=1000)
        for _ in range(4):
            st = exchange(client, ot_id, preload) or st
            time.sleep(args.rpi_us / 1e6)

        start = eip.build_output_assembly_104(
            servo_on=True, operating_mode=1, travel_mode=2,
            non_cyclic_move_type=0, position_puu=target_puu,
            speed_rpm=speed_rpm, accel_rpm_per_s=accel_rpm_s,
            decel_rpm_per_s=decel_rpm_s, start_motion=True,
            torque_ramp_time_ms=1000)
        st = exchange(client, ot_id, start) or st
        time.sleep(args.rpi_us / 1e6)

        run = eip.build_output_assembly_104(
            servo_on=True, operating_mode=1, travel_mode=2,
            non_cyclic_move_type=0, position_puu=target_puu,
            speed_rpm=speed_rpm, accel_rpm_per_s=accel_rpm_s,
            decel_rpm_per_s=decel_rpm_s, start_motion=False,
            torque_ramp_time_ms=1000)

        t0 = time.time()
        at_ref = False
        while time.time() - t0 < args.move_timeout_s:
            st = exchange(client, ot_id, run) or st
            if not st:
                continue
            err_puu = abs(int(st["actual_position"]) - target_puu)
            err_mm = err_puu / args.puu_per_mm
            if st.get("at_reference") or err_mm <= args.tol_mm:
                # Prefer AtReference; also accept in-band if nearly stopped.
                if st.get("at_reference") or abs(st.get("actual_speed", 0)) < 50:
                    at_ref = True
                    break
            time.sleep(args.rpi_us / 1e6)

        elapsed = time.time() - t0
        final_puu = int(st["actual_position"]) if st else 0
        err_mm = abs(final_puu - target_puu) / args.puu_per_mm
        print(f"[4] Done elapsed={elapsed:.3f}s at_ref={st.get('at_reference') if st else None} "
              f"pos={final_puu} err={err_mm:.3f} mm")

        # --- 5 Hold ---
        hold = eip.build_output_assembly_104(
            servo_on=True, operating_mode=0, travel_mode=10,
            position_puu=final_puu, torque_ramp_time_ms=1000)
        for _ in range(6):
            st = exchange(client, ot_id, hold) or st
            time.sleep(args.rpi_us / 1e6)
        print("[5] Hold OM=0 TM=10")

        if err_mm <= args.tol_mm:
            print(f"[PASS] |error|={err_mm:.3f} mm <= {args.tol_mm}")
            return 0
        print(f"[FAIL] |error|={err_mm:.3f} mm > {args.tol_mm}")
        return 1
    finally:
        try:
            client.forward_close(result.get("params", params))
        except Exception:
            pass
        try:
            client.unregister_session()
        except Exception:
            pass
        client.disconnect()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except socket.error as e:
        print(f"[FAIL] socket: {e}")
        raise SystemExit(2)

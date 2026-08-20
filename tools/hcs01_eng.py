#!/usr/bin/env python3
"""HCS01 engineering-HTTP diagnostic / maintenance CLI.

Uses Service Tool COMWS on the engineering IP (default 192.168.1.22).
Never writes CIP/FKM IP (P-0-4089.0.13/.14/.15).

Examples (PowerShell):
  py tools/hcs01_eng.py status
  py tools/hcs01_eng.py watch
  py tools/hcs01_eng.py c0500
  py tools/hcs01_eng.py pm
  py tools/hcs01_eng.py om
  py tools/hcs01_eng.py c0300 --yes
  py tools/hcs01_eng.py travel --yes
  py tools/hcs01_eng.py save --yes
  py tools/hcs01_eng.py verify-origin
  py tools/hcs01_eng.py reboot --yes
  py tools/hcs01_eng.py get S-0-0095
  py tools/hcs01_eng.py set S-0-0138 1000.000
"""

from __future__ import annotations

import argparse
import sys
import time

from hcs01_comws import (
    CIP_IP_IDNS,
    DEFAULT_CIP_HOST,
    DEFAULT_HOST,
    ComwsError,
    Hcs01Comws,
    decode_control_word,
    decode_status_word,
    normalize_idn,
    parse_status_word,
)

STATUS_IDNS = [
    ("S-0-0095.0.0", "diagnostic"),
    ("S-0-0390.0.0", "diag msg"),
    ("S-0-0032.0.0", "op mode primary"),
    ("S-0-0138.0.0", "bipolar accel"),
    ("S-0-0403.0.0", "position status"),
    ("S-0-0051.0.0", "actual pos"),
    ("S-0-0278.0.0", "travel range"),
    ("S-0-0049.0.0", "pos limit +"),
    ("S-0-0050.0.0", "pos limit -"),
    ("S-0-0055.0.0", "pos polarities"),
    ("S-0-0076.0.0", "scaling type"),
    ("S-0-0282.0.0", "cmd pos"),
    ("S-0-0259.0.0", "cmd vel"),
    ("S-0-0260.0.0", "cmd accel"),
    ("S-0-0359.0.0", "cmd decel"),
    ("P-0-4077.0.0", "control word"),
    ("P-0-4078.0.0", "status word"),
    ("P-0-0115.0.0", "device status"),
    ("P-0-0116.0.0", "effective CW"),
    ("P-0-4073.0.0", "IO status"),
    ("P-0-4084.0.0", "profile"),
    ("P-0-4071.0.0", "MDT size"),
    ("P-0-4082.0.0", "AT size"),
    ("P-0-4089.0.13", "CIP IP (ro)"),
]

WATCH_IDNS = [
    "S-0-0095.0.0",
    "S-0-0390.0.0",
    "P-0-4077.0.0",
    "P-0-4078.0.0",
    "P-0-0115.0.0",
    "P-0-0116.0.0",
    "S-0-0051.0.0",
    "S-0-0282.0.0",
    "S-0-0403.0.0",
    "P-0-4073.0.0",
]


def _client(args: argparse.Namespace) -> Hcs01Comws:
    return Hcs01Comws(host=args.host, cip_host=args.cip_host)


def cmd_status(cli: Hcs01Comws, _args: argparse.Namespace) -> int:
    net = cli.probe_network()
    print(
        f"net eng ping={net['ping_eng']} http80={net['http80']} | "
        f"cip ping={net['ping_cip']} :44818={net['cip44818']}"
    )
    cli.login()
    print("--- snapshot ---")
    for idn, label in STATUS_IDNS:
        try:
            val = cli.getvar(idn)
        except Exception as e:
            print(f"{idn:20s} {label:16s} ERROR {type(e).__name__}: {e}")
            continue
        extra = ""
        if idn.startswith("P-0-4077") or idn.startswith("P-0-0116"):
            sw = parse_status_word(val)
            extra = f"  # 0x{sw:04X} {decode_control_word(sw)}"
        elif idn.startswith("P-0-4078") or idn.startswith("P-0-0115"):
            sw = parse_status_word(val)
            extra = f"  # 0x{sw:04X} {decode_status_word(sw)}"
        print(f"{idn:20s} {label:16s} {val}{extra}")
    print("P-0-4081", cli.getlst("P-0-4081.0.0"))
    print("P-0-4080", cli.getlst("P-0-4080.0.0"))
    return 0


def cmd_watch(cli: Hcs01Comws, args: argparse.Namespace) -> int:
    cli.login()
    interval = args.interval
    print(f"watch every {interval:g}s (Ctrl+C to stop)")
    try:
        while True:
            print(f"\n--- {time.strftime('%H:%M:%S')} ---")
            for idn in WATCH_IDNS:
                try:
                    val = cli.getvar(idn)
                except Exception as e:
                    print(f"{idn:20s} ERROR {e}")
                    continue
                extra = ""
                if "4077" in idn or "0116" in idn:
                    sw = parse_status_word(val)
                    extra = f"  # {decode_control_word(sw)}"
                elif "4078" in idn or "0115" in idn:
                    sw = parse_status_word(val)
                    extra = f"  # {decode_status_word(sw)}"
                print(f"{idn:20s} {val}{extra}")
            time.sleep(interval)
    except KeyboardInterrupt:
        print("\nwatch stopped")
    return 0


def cmd_c0500(cli: Hcs01Comws, _args: argparse.Namespace) -> int:
    cli.login()
    print("diag before", cli.getvar("S-0-0095.0.0"))
    try:
        cli.run_command("S-0-0099.0.0", timeout_s=30.0)
    except ComwsError as e:
        print("C0500 failed:", e)
        return 1
    print("diag after ", cli.getvar("S-0-0095.0.0"))
    print("S-0-0390   ", cli.getvar("S-0-0390.0.0"))
    print("P-0-4078   ", cli.getvar("P-0-4078.0.0"))
    return 0


def cmd_pm(cli: Hcs01Comws, _args: argparse.Namespace) -> int:
    cli.login()
    try:
        cli.run_command("S-0-0420.0.0", timeout_s=40.0)
    except ComwsError as e:
        print("PM failed:", e)
        print("diag", cli.getvar("S-0-0095.0.0"))
        return 1
    print("diag", cli.getvar("S-0-0095.0.0"))
    return 0


def cmd_om(cli: Hcs01Comws, _args: argparse.Namespace) -> int:
    cli.login()
    try:
        cli.run_command("S-0-0422.0.0", timeout_s=40.0)
    except ComwsError as e:
        print("OM failed:", e)
        print("diag", cli.getvar("S-0-0095.0.0"))
        return 1
    print("diag", cli.getvar("S-0-0095.0.0"))
    return 0


def _parse_float(raw: str) -> float:
    s = (raw or "").strip().replace(",", ".")
    if not s:
        raise ValueError("empty numeric IDN")
    return float(s.split()[0])


def _dump_travel(cli: Hcs01Comws) -> dict[str, str]:
    vals = {}
    for idn, label in (
        ("S-0-0051.0.0", "actual"),
        ("S-0-0278.0.0", "travel"),
        ("S-0-0049.0.0", "lim+"),
        ("S-0-0050.0.0", "lim-"),
        ("S-0-0403.0.0", "pos-stat"),
    ):
        try:
            vals[idn] = cli.getvar(idn)
        except Exception as e:
            vals[idn] = f"ERROR {e}"
        print(f"{idn:20s} {label:10s} {vals[idn]}")
    return vals


def cmd_travel(cli: Hcs01Comws, args: argparse.Namespace) -> int:
    if not args.yes:
        print("Writes S-0-0278=36000, S-0-0049=+180, S-0-0050=-180 in PM.")
        print("Does not touch CIP/FKM IP. Re-run with --yes to confirm.")
        return 2
    cli.login()
    print("--- before ---")
    _dump_travel(cli)
    print("PM (S-0-0420)")
    try:
        cli.run_command("S-0-0420.0.0", timeout_s=40.0)
    except ComwsError as e:
        print("PM failed:", e)
        print("diag", cli.getvar("S-0-0095.0.0"))
        return 1
    for idn, value in (
        ("S-0-0278.0.0", "36000.0000"),
        ("S-0-0049.0.0", "180.0000"),
        ("S-0-0050.0.0", "-180.0000"),
    ):
        try:
            r = cli.setvar_try_element7(idn, value)
        except ComwsError as e:
            print("set", idn, "failed:", e)
            return 1
        print("set", idn, r, "readback", cli.getvar(idn))
    print("OM (S-0-0422)")
    try:
        cli.run_command("S-0-0422.0.0", timeout_s=40.0)
    except ComwsError as e:
        print("OM failed (may stay in PM if cyclic bit1=0):", e)
        print("diag", cli.getvar("S-0-0095.0.0"))
        print("Travel IDNs were written; restore OM from keypad/IndraWorks if needed.")
    print("--- after ---")
    _dump_travel(cli)
    print("Next: save --yes, then reboot/power-cycle, then verify-origin.")
    return 0


def cmd_save(cli: Hcs01Comws, args: argparse.Namespace) -> int:
    if not args.yes:
        print("C2200 Backup working memory (S-0-0264) to non-volatile flash.")
        print("Re-run with --yes to confirm.")
        return 2
    cli.login()
    print("diag before", cli.getvar("S-0-0095.0.0"))
    try:
        cli.run_command("S-0-0264.0.0", timeout_s=60.0)
    except ComwsError as e:
        print("C2200 save failed:", e)
        print("diag", cli.getvar("S-0-0095.0.0"))
        print("Fallback: Service Tool / IndraWorks Save, then verify-origin after reboot.")
        return 1
    print("diag after ", cli.getvar("S-0-0095.0.0"))
    print("Saved. Reboot or 24 V cycle, then: py tools/hcs01_eng.py verify-origin")
    return 0


def cmd_verify_origin(cli: Hcs01Comws, _args: argparse.Namespace) -> int:
    cli.login()
    print("--- verify-origin ---")
    vals = _dump_travel(cli)
    try:
        abs_deg = _parse_float(vals["S-0-0051.0.0"])
        t0278 = _parse_float(vals["S-0-0278.0.0"])
        t0049 = _parse_float(vals["S-0-0049.0.0"])
        t0050 = _parse_float(vals["S-0-0050.0.0"])
    except (ValueError, KeyError) as e:
        print("parse failed:", e)
        return 1
    aligned = abs(abs_deg) <= 2.0
    travel_ok = t0278 >= 35999.0 and abs(t0049 - 180.0) < 0.2 and abs(t0050 + 180.0) < 0.2
    print(f"ALIGNED ( |S-0-0051|<=2 ): {aligned}  abs={abs_deg:.4f}")
    print(f"travel overlay: {travel_ok}  0278={t0278}  0049={t0049}  0050={t0050}")
    if aligned and travel_ok:
        print("PASS — console 'home t' should show origin ALIGNED / thetalim ~+/-180")
        return 0
    print("FAIL — C0300 at cable-neutral + travel --yes + save + reboot, then retry")
    return 1


def cmd_c0300(cli: Hcs01Comws, args: argparse.Namespace) -> int:
    if not args.yes:
        print("C0300 zeros / sets absolute position at the current pose.")
        print("Use at cable-neutral, then save parameters, then gantry 'home t'.")
        print("Re-run with --yes to confirm.")
        return 2
    cli.login()
    print("S-0-0448", cli.getvar("S-0-0448.0.0"))
    print("S-0-0051 before", cli.getvar("S-0-0051.0.0"))
    try:
        cli.run_command("S-0-0447.0.0", timeout_s=30.0)
    except ComwsError as e:
        print("C0300 failed:", e)
        return 1
    print("S-0-0403", cli.getvar("S-0-0403.0.0"))
    print("S-0-0051 after ", cli.getvar("S-0-0051.0.0"))
    print("diag", cli.getvar("S-0-0095.0.0"))
    print("Next: save drive parameters, then console 'home t' (status must show ALIGNED).")
    return 0


def cmd_autotune(cli: Hcs01Comws, args: argparse.Namespace) -> int:
    if not args.yes:
        print("=" * 60)
        print(" HCS01 / ERD-04 Auto-Tuning (C1800 Drive Optimization)")
        print("=" * 60)
        print("WARNING: Motor will perform test oscillation movements (+/-45 deg)!")
        print("Ensure end effector (up to 2.0 kg payload) is rigidly mounted and free of obstructions.")
        print("Re-run with --yes to execute (add --save to automatically write to NV flash).")
        return 2

    cli.login()
    print("--- HCS01 Auto-Tuning Sequence Started ---")
    print("Diagnostic before:", cli.getvar("S-0-0095.0.0"))

    # 1. Ensure damping factor and travel amplitude are configured
    damping = str(args.damping)
    travel = str(args.travel)
    print(f"Setting damping factor P-0-0163 = {damping} ...")
    cli.setvar_try_element7("P-0-0163.0.0", damping)

    print(f"Setting test travel distance P-0-0169 = {travel} deg ...")
    cli.setvar_try_element7("P-0-0169.0.0", travel)

    # 2. Trigger C1800 Command Drive Optimization / Automatic control loop setting
    print("Issuing C1800 (P-0-0162.0.0) Drive Optimization command...")
    try:
        cli.run_command("P-0-0162.0.0", timeout_s=120.0)
    except ComwsError as e:
        print("C1800 Drive Optimization failed:", e)
        print("Diagnostic:", cli.getvar("S-0-0095.0.0"))
        return 1

    print("C1800 Complete! Reading tuned control loop parameters:")
    try:
        inertia = cli.getvar("P-0-4010.0.0")
        kp = cli.getvar("S-0-0100.0.0")
        tn = cli.getvar("S-0-0101.0.0")
        kv = cli.getvar("S-0-0104.0.0")
        ka = cli.getvar("S-0-0348.0.0")
        print(f"  Load Inertia (P-0-4010)       : {inertia} kg*m^2")
        print(f"  Velocity Prop Gain Kp (S-0-0100): {kp}")
        print(f"  Velocity Integral Tn (S-0-0101) : {tn} ms")
        print(f"  Position Prop Gain Kv (S-0-0104): {kv} 1/s")
        print(f"  Accel Feedforward Ka (S-0-0348) : {ka} %")
    except Exception as e:
        print("Warning reading result parameters:", e)

    if args.save:
        print("Saving optimized parameters to non-volatile flash (C2200)...")
        try:
            cli.run_command("S-0-0264.0.0", timeout_s=60.0)
            print("Save complete!")
        except Exception as e:
            print("C2200 save failed:", e)

    print("--- Auto-Tuning Successful ---")
    return 0


def cmd_reboot(cli: Hcs01Comws, args: argparse.Namespace) -> int:
    if not args.yes:
        print("C6400 reboots the control section (HTTP/CIP drop briefly).")
        print("Re-run with --yes to confirm.")
        return 2
    cli.login()
    print("diag before", cli.getvar("S-0-0095.0.0"))
    try:
        # Command may kill HTTP mid-poll; treat that as success.
        cli.run_command("S-0-1350.0.0", timeout_s=5.0)
    except (ComwsError, OSError, TimeoutError) as e:
        print("C6400 issued (HTTP may already be down):", e)
    except Exception as e:
        print("C6400 issued (HTTP may already be down):", type(e).__name__, e)

    print("waiting for engineering HTTP ...")
    if not cli.wait_http(timeout_s=90.0):
        print("HTTP did not return within timeout")
        return 1
    cli.login()
    print("diag after", cli.getvar("S-0-0095.0.0"))
    print("S-0-0032 ", cli.getvar("S-0-0032.0.0"))
    print("P-0-4073 ", cli.getvar("P-0-4073.0.0"))
    net = cli.probe_network()
    print(
        f"net eng ping={net['ping_eng']} http80={net['http80']} | "
        f"cip :44818={net['cip44818']}"
    )
    return 0


def cmd_get(cli: Hcs01Comws, args: argparse.Namespace) -> int:
    cli.login()
    idn = normalize_idn(args.idn)
    if args.list:
        print(cli.getlst(idn, count=args.count))
    else:
        print(f"{idn} = {cli.getvar(idn)}")
    return 0


def cmd_set(cli: Hcs01Comws, args: argparse.Namespace) -> int:
    idn = normalize_idn(args.idn)
    bare = idn
    for blocked in CIP_IP_IDNS:
        if bare.startswith(blocked) or blocked in bare:
            print(f"refusing to write {bare}: CIP IP is off-limits")
            return 1
    cli.login()
    try:
        r = cli.setvar_try_element7(idn, args.value)
    except ComwsError as e:
        print(e)
        return 1
    print("set", r)
    print("readback", cli.getvar(idn))
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="HCS01 engineering HTTP (COMWS) diagnostic CLI"
    )
    p.add_argument(
        "--host",
        default=DEFAULT_HOST,
        help=f"engineering IP (default {DEFAULT_HOST})",
    )
    p.add_argument(
        "--cip-host",
        default=DEFAULT_CIP_HOST,
        help=f"CIP/FKM IP for probes (default {DEFAULT_CIP_HOST})",
    )
    sub = p.add_subparsers(dest="cmd")

    sp = sub.add_parser("status", help="network probe + IDN snapshot (default)")
    sp.set_defaults(func=cmd_status)

    sp = sub.add_parser("watch", help="poll key IDs until Ctrl+C")
    sp.add_argument(
        "--interval", type=float, default=1.0, help="seconds between polls"
    )
    sp.set_defaults(func=cmd_watch)

    sp = sub.add_parser("c0500", help="S-0-0099 reset class-1 diagnostics")
    sp.set_defaults(func=cmd_c0500)

    sp = sub.add_parser("pm", help="S-0-0420 enter parameter mode")
    sp.set_defaults(func=cmd_pm)

    sp = sub.add_parser("om", help="S-0-0422 enter operating mode")
    sp.set_defaults(func=cmd_om)

    sp = sub.add_parser(
        "c0300", help="S-0-0447 set absolute position (requires --yes)"
    )
    sp.add_argument("--yes", action="store_true")
    sp.set_defaults(func=cmd_c0300)

    sp = sub.add_parser(
        "travel",
        help="PM + set S-0-0278/0049/0050 (requires --yes; no CIP IP writes)",
    )
    sp.add_argument("--yes", action="store_true")
    sp.set_defaults(func=cmd_travel)

    sp = sub.add_parser(
        "autotune",
        help="C1800 P-0-0162 load inertia identification & controller autotune (requires --yes)",
    )
    sp.add_argument("--yes", action="store_true", help="confirm execution")
    sp.add_argument("--save", action="store_true", help="automatically run C2200 to save to NV flash")
    sp.add_argument("--damping", type=float, default=1.0, help="damping factor P-0-0163 (0.5..10.0, default 1.0)")
    sp.add_argument("--travel", type=float, default=45.0, help="test travel stroke P-0-0169 (deg, default 45.0)")
    sp.set_defaults(func=cmd_autotune)

    sp = sub.add_parser(
        "save", help="C2200 S-0-0264 backup working memory (requires --yes)"
    )
    sp.add_argument("--yes", action="store_true")
    sp.set_defaults(func=cmd_save)

    sp = sub.add_parser(
        "verify-origin",
        help="assert |S-0-0051|<=2 and travel overlay after reboot",
    )
    sp.set_defaults(func=cmd_verify_origin)

    sp = sub.add_parser(
        "reboot", help="S-0-1350 C6400 control-section reboot (requires --yes)"
    )
    sp.add_argument("--yes", action="store_true")
    sp.set_defaults(func=cmd_reboot)

    sp = sub.add_parser("get", help="read one IDN")
    sp.add_argument("idn")
    sp.add_argument("--list", action="store_true", help="use getlst")
    sp.add_argument("--count", type=int, default=16)
    sp.set_defaults(func=cmd_get)

    sp = sub.add_parser("set", help="write one IDN (CIP IP refused)")
    sp.add_argument("idn")
    sp.add_argument("value")
    sp.set_defaults(func=cmd_set)

    return p


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if not args.cmd:
        args.cmd = "status"
        args.func = cmd_status
    cli = _client(args)
    try:
        return args.func(cli, args)
    except (OSError, TimeoutError) as e:
        print(f"network error talking to {args.host}: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())

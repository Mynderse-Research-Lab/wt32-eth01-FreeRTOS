#!/usr/bin/env python3
"""TCP continue: enable, wait Z Active, finish SAFE_Z band X tests."""
from __future__ import annotations

import re
import socket
import sys
import time

HOST, PORT, PW = "192.168.1.100", 2323, "LTU_1932"


def strip_ansi(s: str) -> str:
    return re.sub(r"\x1b\[[0-9;]*[A-Za-z]", "", s)


def connect() -> socket.socket:
    deadline = time.time() + 60
    while time.time() < deadline:
        try:
            s = socket.create_connection((HOST, PORT), timeout=3)
            s.settimeout(1.0)
            return s
        except OSError:
            time.sleep(1)
    raise SystemExit("no TCP")


def drain(s: socket.socket, settle: float = 0.25) -> str:
    time.sleep(settle)
    out = b""
    try:
        while True:
            out += s.recv(4096)
    except (socket.timeout, ConnectionResetError, OSError):
        pass
    return strip_ansi(out.decode("utf-8", "replace"))


def cmd(s: socket.socket, line: str, wait_s: float = 1.0) -> str:
    print(f"\n>>> {line}")
    try:
        s.sendall((line + "\n").encode())
    except OSError as e:
        print("send failed", e)
        return ""
    time.sleep(wait_s)
    out = drain(s, 0.2)
    preview = "\n".join(out.replace("\r", "").splitlines()[-16:])
    if preview.strip():
        print(preview)
    return out


def wait_idle(s: socket.socket, timeout_s: float) -> str:
    deadline = time.time() + timeout_s
    buf = ""
    while time.time() < deadline:
        more = drain(s, 0.5)
        if more:
            buf += more
            print(more.replace("\r", "")[-500:])
        st = cmd(s, "status", 0.4)
        buf += st
        if re.search(r"Busy:\s*No", st, re.I):
            return buf
    print(f"WARN wait_idle timeout {timeout_s}s — issuing stop")
    buf += cmd(s, "stop", 1.0)
    time.sleep(0.5)
    buf += cmd(s, "enable", 4.0)
    drain(s, 1.5)
    return buf


def move_z_only(s: socket.socket, z_mm: float) -> str:
    """Keep commanded X exactly at current X so the planner does not arm X."""
    st = cmd(s, "status", 0.6)
    x, _ = parse_xz(st)
    if x is None:
        x = 35.0
    return cmd(s, f"move {x:.3f} {z_mm:.3f} 0", 1.5)


def parse_xz(st: str):
    xm = re.search(r"X Position:\s*([-+0-9.]+)\s*mm", st)
    zm = re.search(r"Z Position:\s*([-+0-9.]+)\s*mm", st)
    return (float(xm.group(1)) if xm else None, float(zm.group(1)) if zm else None)


def auth(s: socket.socket) -> None:
    b = drain(s, 0.5)
    if "Password:" in b and "OK authenticated" not in b:
        s.sendall((PW + "\n").encode())
        b += drain(s, 1.0)
    print("--- auth ---")
    print(b[-250:])


def wait_servo_active(s: socket.socket, timeout_s: float = 30.0) -> bool:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        more = drain(s, 0.5)
        if more:
            print(more.replace("\r", "")[-400:])
        if "Servo arm complete" in more and "Z:" in more:
            return True
        if "Servo arm complete" in more:
            # X or Z — keep waiting a bit for both
            pass
        time.sleep(0.3)
    # probe with a tiny Z-only move that shouldn't need X cal if same X...
    # but session may need gates. Prefer pins/status.
    return True


def main() -> int:
    fails: list[str] = []
    s = connect()
    auth(s)

    cmd(s, "stop", 0.8)
    time.sleep(0.4)
    cmd(s, "alarmreset", 1.0)
    en = cmd(s, "enable", 2.0)
    # Wait for both axes Active
    deadline = time.time() + 20
    saw_x = saw_z = False
    while time.time() < deadline and not (saw_x and saw_z):
        more = drain(s, 0.6)
        if more:
            print(more.replace("\r", "")[-500:])
            if "X: Servo arm complete" in more or re.search(r"\bX: Servo arm complete", more):
                saw_x = True
            if "Z: Servo arm complete" in more:
                saw_z = True
    et = cmd(s, "eiptiming", 1.2)
    if "GO" not in et:
        fails.append("Class1 not GO")
        cmd(s, "disable", 1)
        s.close()
        print("LIVE_CONT_FAIL class1")
        return 3

    st = cmd(s, "status", 1.0)
    x, z = parse_xz(st)
    print(f"start X={x} Z={z} enabled")

    # Session gates: if move with X change is blocked, run calibrate all once.
    # Pure Z probe first (same X) to confirm Z Active.
    probe = move_z_only(s, 20.0)
    if "X move blocked" in probe or "Run 'home x'" in probe:
        print("\n=== session gates missing — calibrate all ===")
        cmd(s, "calibrate all", 2.0)
        out = ""
        deadline = time.time() + 420
        while time.time() < deadline:
            more = drain(s, 0.7)
            if more:
                out += more
                print(more.replace("\r", "")[-600:])
            if "OK Bring-up complete" in out or "[BRINGUP] complete" in out:
                break
            if "[BRINGUP] failed" in out or "ERROR: Bring-up" in out:
                break
            st = cmd(s, "status", 0.4)
            out += st
        if "OK Bring-up complete" not in out and "[BRINGUP] complete" not in out:
            fails.append("bring-up failed")
            cmd(s, "faults", 1)
            cmd(s, "disable", 1)
            s.close()
            print("LIVE_CONT_FAIL bringup")
            return 2
        st = cmd(s, "status", 1.0)
        x, z = parse_xz(st)
        print(f"after bring-up X={x} Z={z}")
    else:
        wait_idle(s, 60)
        st = cmd(s, "status", 0.8)
        x, z = parse_xz(st)
        print(f"after Z-only probe X={x} Z={z}")
        if "Z arm failed" in probe or "move rejected" in probe:
            print("Z arm failed on probe — re-enable")
            cmd(s, "disable", 1.0)
            time.sleep(0.5)
            cmd(s, "alarmreset", 1.0)
            cmd(s, "enable", 6.0)
            drain(s, 2.0)
            probe2 = move_z_only(s, 20.0)
            wait_idle(s, 60)
            if "Z arm failed" in probe2 or "move rejected" in probe2:
                fails.append("Z still not Active / arm failed after re-enable")
                cmd(s, "faults", 1)
                cmd(s, "disable", 1)
                s.close()
                print("LIVE_CONT_FAIL z_active")
                return 3

    print("\n=== INTERLOCK refuse at Z=90 ===")
    move_z_only(s, 90.0)
    buf = wait_idle(s, 90)
    if "Z arm failed" in buf:
        fails.append("could not lift to Z=90 (Z arm failed)")
    st = cmd(s, "status", 0.8)
    _, z = parse_xz(st)
    print(f"Z={z}")
    hx = cmd(s, "home x", 1.5)
    if "ERROR: home x blocked" not in hx and "above SAFE_Z" not in hx and "INTERLOCK" not in hx:
        if z is not None and z > 50:
            fails.append("home x did not refuse above SAFE_Z")
        else:
            fails.append(f"Z not high enough for refuse test ({z})")
    else:
        print("PASS home x refused")

    print("\n=== descend + X in band ===")
    move_z_only(s, 12.0)
    wait_idle(s, 90)
    # Intentional X change while in band
    st = cmd(s, "status", 0.5)
    _, z = parse_xz(st)
    cmd(s, "move 90 12 0", 1.5)
    path = wait_idle(s, 120)
    st = cmd(s, "status", 0.8)
    x, z = parse_xz(st)
    print(f"in-band result X={x} Z={z}")
    if x is None or abs(x - 90) > 3:
        fails.append(f"expected X~90 got {x}")
    if z is None or z > 31:
        fails.append(f"expected Z<=30 got {z}")
    if "INTERLOCK" in path and "X blocked" in path:
        fails.append("unexpected INTERLOCK")

    print("\n=== high Z then path to (140,15) ===")
    move_z_only(s, 100.0)
    wait_idle(s, 90)
    cmd(s, "move 140 15 0", 1.5)
    buf = wait_idle(s, 150)
    st = cmd(s, "status", 0.8)
    x, z = parse_xz(st)
    print(f"high-start path X={x} Z={z}")
    if "SAFE_Z ceiling=30" in buf or "[PATH] seg" in buf:
        print("PASS path planner logged bottom-band segments")
    if x is None or abs(x - 140) > 4:
        fails.append(f"expected X~140 got {x}")
    if z is None or abs(z - 15) > 4:
        fails.append(f"expected Z~15 got {z}")

    move_z_only(s, 30.0)
    wait_idle(s, 90)
    faults = cmd(s, "faults", 1.0)
    cmd(s, "disable", 1.0)
    cmd(s, "logout", 0.3)
    s.close()

    if "Alarm: Yes" in faults:
        fails.append("Alarm: Yes")

    if fails:
        print("\nLIVE_CONT_FAIL:")
        for f in fails:
            print(" -", f)
        return 2
    print("\nLIVE_CONT_OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

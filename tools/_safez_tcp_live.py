#!/usr/bin/env python3
"""TCP: calibrate all then SAFE_Z interlock + in-band X path."""
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
    preview = "\n".join(out.replace("\r", "").splitlines()[-18:])
    if preview.strip():
        print(preview)
    return out


def wait_for(s: socket.socket, markers: list[str], timeout_s: float) -> str:
    deadline = time.time() + timeout_s
    buf = ""
    while time.time() < deadline:
        more = drain(s, 0.6)
        if more:
            buf += more
            print(more.replace("\r", "")[-700:])
        if any(m in buf for m in markers):
            return buf
        st = cmd(s, "status", 0.4)
        buf += st
        if any(m in st for m in markers):
            return buf
    return buf


def parse_xz(st: str) -> tuple[float | None, float | None]:
    xm = re.search(r"X Position:\s*([-+0-9.]+)\s*mm", st)
    zm = re.search(r"Z Position:\s*([-+0-9.]+)\s*mm", st)
    return (float(xm.group(1)) if xm else None, float(zm.group(1)) if zm else None)


def auth(s: socket.socket) -> None:
    b = drain(s, 0.5)
    if "Password:" in b and "OK authenticated" not in b:
        s.sendall((PW + "\n").encode())
        b += drain(s, 1.0)
    print("--- auth ---")
    print(b[-300:])


def main() -> int:
    fails: list[str] = []
    s = connect()
    auth(s)

    cmd(s, "stop", 1.0)
    time.sleep(0.5)
    cmd(s, "alarmreset", 1.0)
    cmd(s, "enable", 5.0)
    et = cmd(s, "eiptiming", 1.2)
    if "GO" not in et:
        fails.append("Class1 not GO after enable")
        print("ABORT: Class1 down")
        cmd(s, "disable", 1)
        s.close()
        return 3

    print("\n=== calibrate all ===")
    cmd(s, "calibrate all", 2.0)
    out = wait_for(
        s,
        ["OK Bring-up complete", "[BRINGUP] complete", "[BRINGUP] failed", "ERROR: Bring-up"],
        420,
    )
    if "OK Bring-up complete" not in out and "[BRINGUP] complete" not in out:
        fails.append("bring-up did not complete")
        cmd(s, "faults", 1)
        cmd(s, "disable", 1)
        s.close()
        print("LIVE_FAIL bring-up")
        return 2

    st = cmd(s, "status", 1.0)
    x, z = parse_xz(st)
    print(f"after bring-up X={x} Z={z}")
    if z is None or z > 35:
        fails.append(f"expected Z near SAFE_Z ceiling (<=35), got {z}")

    print("\n=== INTERLOCK: Z=90 then home x must refuse ===")
    cmd(s, "move 35 90 0", 1.5)
    wait_for(s, ["Busy: No"], 120)
    st = cmd(s, "status", 0.8)
    _, z = parse_xz(st)
    print(f"Z for refuse test={z}")
    if z is None or z < 50:
        fails.append(f"failed to lift above band for interlock, Z={z}")
    hx = cmd(s, "home x", 1.5)
    if "ERROR: home x blocked" not in hx and "above SAFE_Z" not in hx and "INTERLOCK" not in hx:
        fails.append("home x did not refuse above SAFE_Z")
    else:
        print("PASS home x refused above SAFE_Z")

    print("\n=== descend into band + X traverse ===")
    cmd(s, "move 35 12 0", 1.5)
    wait_for(s, ["Busy: No"], 120)
    cmd(s, "move 90 12 0", 1.5)
    path = wait_for(s, ["Busy: No"], 150)
    st = cmd(s, "status", 0.8)
    x, z = parse_xz(st)
    print(f"after in-band X move X={x} Z={z}")
    if x is None or abs(x - 90) > 3:
        fails.append(f"expected X~90, got {x}")
    if z is None or z > 31:
        fails.append(f"expected Z in band, got {z}")
    if "X blocked" in path and "INTERLOCK" in path:
        fails.append("unexpected INTERLOCK on in-band X")

    print("\n=== high Z start then X change: must Z-alone descend first ===")
    cmd(s, "move 90 100 0", 1.5)
    wait_for(s, ["Busy: No"], 120)
    cmd(s, "move 140 15 0", 1.5)
    buf = wait_for(s, ["Busy: No"], 150)
    if "SAFE_Z ceiling=30" not in buf and "ceiling=30" not in buf:
        # still ok if path segments present
        if "[PATH] seg" not in buf:
            fails.append("missing PATH log for high-start move")
    st = cmd(s, "status", 0.8)
    x, z = parse_xz(st)
    print(f"after high-start path X={x} Z={z}")
    if x is None or abs(x - 140) > 4:
        fails.append(f"expected X~140, got {x}")
    if z is None or abs(z - 15) > 4:
        fails.append(f"expected Z~15, got {z}")

    cmd(s, "move 35 30 0", 1.5)
    wait_for(s, ["Busy: No"], 120)
    faults = cmd(s, "faults", 1.0)
    cmd(s, "disable", 1.0)
    cmd(s, "logout", 0.3)
    s.close()

    if "Alarm: Yes" in faults:
        fails.append("Alarm: Yes at end")

    if fails:
        print("\nLIVE_TCP_FAIL:")
        for f in fails:
            print(" -", f)
        return 2
    print("\nLIVE_TCP_OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

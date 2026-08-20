#!/usr/bin/env python3
"""TCP smoke: SAFE_Z bottom band (from Z-/A015) + X interlock + short path."""
from __future__ import annotations

import re
import socket
import sys
import time

HOST, PORT, PW = "192.168.1.100", 2323, "LTU_1932"


def strip_ansi(s: str) -> str:
    return re.sub(r"\x1b\[[0-9;]*[A-Za-z]", "", s)


def connect(timeout_s: float = 60.0) -> socket.socket:
    deadline = time.time() + timeout_s
    last = None
    while time.time() < deadline:
        try:
            s = socket.create_connection((HOST, PORT), timeout=3)
            s.settimeout(1.0)
            return s
        except OSError as e:
            last = e
            time.sleep(1.0)
    raise SystemExit(f"TCP never came up: {last}")


def recv_until(s: socket.socket, predicates, timeout_s: float = 12.0) -> str:
    buf = b""
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            chunk = s.recv(4096)
        except socket.timeout:
            text = strip_ansi(buf.decode("utf-8", "replace"))
            if any(p(text) for p in predicates):
                return text
            continue
        if not chunk:
            break
        buf += chunk
        text = strip_ansi(buf.decode("utf-8", "replace"))
        if any(p(text) for p in predicates):
            return text
    return strip_ansi(buf.decode("utf-8", "replace"))


def drain(s: socket.socket, settle: float = 0.25) -> str:
    time.sleep(settle)
    out = b""
    try:
        while True:
            out += s.recv(4096)
    except socket.timeout:
        pass
    return strip_ansi(out.decode("utf-8", "replace"))


def cmd(s: socket.socket, line: str, wait_s: float = 1.0) -> str:
    print(f"\n>>> {line}")
    s.sendall((line + "\n").encode())
    time.sleep(wait_s)
    out = drain(s, 0.2)
    preview = "\n".join(out.replace("\r", "").splitlines()[-20:])
    if preview.strip():
        print(preview)
    return out


def wait_idle(s: socket.socket, timeout_s: float) -> str:
    deadline = time.time() + timeout_s
    buf = ""
    while time.time() < deadline:
        more = drain(s, 0.4)
        if more:
            buf += more
            print(more.replace("\r", "")[-500:])
        st = cmd(s, "status", 0.5)
        buf += st
        if re.search(r"Busy:\s*No", st, re.I):
            return buf
    return buf


def parse_z(status: str) -> float | None:
    m = re.search(r"Z Position:\s*([-+0-9.]+)\s*mm", status)
    return float(m.group(1)) if m else None


def parse_x(status: str) -> float | None:
    m = re.search(r"X Position:\s*([-+0-9.]+)\s*mm", status)
    return float(m.group(1)) if m else None


def main() -> int:
    fails: list[str] = []
    s = connect(90)
    banner = recv_until(
        s,
        [
            lambda t: "Password: " in t,
            lambda t: "OK authenticated" in t,
            lambda t: "> " in t,
        ],
        12,
    )
    if "Password: " in banner and "OK authenticated" not in banner:
        s.sendall((PW + "\n").encode())
        banner += recv_until(s, [lambda t: "OK authenticated" in t or "> " in t], 8)
    print("--- auth ---")
    print(banner[-400:])
    drain(s, 0.4)

    cmd(s, "stop", 1.0)
    time.sleep(0.5)
    cmd(s, "alarmreset", 1.0)
    en = cmd(s, "enable", 5.0)
    if "ERROR" in en and "enabled" not in en.lower():
        print("WARN enable response:", en[-200:])

    st0 = cmd(s, "status", 1.0)
    z0 = parse_z(st0)
    x0 = parse_x(st0)
    print(f"--- start pose X={x0} Z={z0} ---")

    # Ensure session gates if bring-up already done earlier: soft accept via status.
    # Lift above SAFE_Z ceiling (30) for interlock refuse test.
    print("\n=== INTERLOCK: lift above SAFE_Z then home x must refuse ===")
    cmd(s, "move 35 80 0", 1.0)
    wait_idle(s, 90)
    st = cmd(s, "status", 0.8)
    z = parse_z(st)
    if z is None or z < 50:
        fails.append(f"expected Z>=50 before interlock refuse, got {z}")
    refuse = cmd(s, "home x", 1.5)
    if "blocked" not in refuse.lower() and "INTERLOCK" not in refuse:
        # Console ERROR path
        if "ERROR: home x blocked" not in refuse and "above SAFE_Z" not in refuse:
            fails.append("home x did not refuse above SAFE_Z band")
        else:
            print("PASS home x refused (console)")
    else:
        print("PASS home x refused")

    print("\n=== return into SAFE_Z band (Z<=30) ===")
    cmd(s, "move 35 15 0", 1.0)
    wait_idle(s, 90)
    st = cmd(s, "status", 0.8)
    z = parse_z(st)
    if z is None or z > 31:
        fails.append(f"expected Z<=30 in band, got {z}")

    print("\n=== path with X travel while in band ===")
    cmd(s, "move 80 15 0", 1.0)
    path_out = wait_idle(s, 120)
    st = cmd(s, "status", 0.8)
    x = parse_x(st)
    z = parse_z(st)
    if x is None or abs(x - 80.0) > 2.0:
        fails.append(f"expected X~80 after in-band move, got {x}")
    if z is None or z > 31:
        fails.append(f"expected Z still in band after X move, got {z}")
    if "[INTERLOCK]" in path_out and "X blocked" in path_out:
        fails.append("unexpected INTERLOCK during in-band X move")

    print("\n=== path from high Z: must descend into band before X ===")
    cmd(s, "move 80 100 0", 1.0)
    wait_idle(s, 90)
    cmd(s, "move 120 20 0", 1.5)
    # Collect path logs while waiting
    buf = ""
    deadline = time.time() + 120
    saw_descend = False
    while time.time() < deadline:
        more = drain(s, 0.5)
        if more:
            buf += more
            print(more.replace("\r", "")[-400:])
            if "move_x=0" in more and "move_z=1" in more:
                saw_descend = True
        st = cmd(s, "status", 0.4)
        buf += st
        if re.search(r"Busy:\s*No", st, re.I):
            break
    if "SAFE_Z ceiling=30" in buf or "ceiling=30" in buf or saw_descend or "[PATH] seg" in buf:
        print("PASS path logged (bottom-band planner active)")
    st = cmd(s, "status", 0.8)
    x = parse_x(st)
    z = parse_z(st)
    if x is None or abs(x - 120.0) > 3.0:
        fails.append(f"expected X~120 after high-start path, got {x}")
    if z is None or abs(z - 20.0) > 3.0:
        fails.append(f"expected Z~20 after high-start path, got {z}")

    print("\n=== park back at bring-up pose ===")
    cmd(s, "move 35 30 0", 1.0)
    wait_idle(s, 90)

    faults = cmd(s, "faults", 1.0)
    cmd(s, "disable", 1.0)
    cmd(s, "logout", 0.4)
    s.close()

    if "Alarm: Yes" in faults:
        fails.append("faults show Alarm: Yes")

    if fails:
        print("\nLIVE_SAFEZ_FAIL:")
        for f in fails:
            print(" -", f)
        return 2
    print("\nLIVE_SAFEZ_OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

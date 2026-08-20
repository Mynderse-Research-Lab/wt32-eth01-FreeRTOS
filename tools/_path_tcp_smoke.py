#!/usr/bin/env python3
"""TCP smoke: bring-up calibrate all + 2-D path coordinated moves."""
from __future__ import annotations

import re
import socket
import sys
import time

HOST, PORT, PW = "192.168.1.100", 2323, "LTU_1932"


def strip_ansi(s: str) -> str:
    return re.sub(r"\x1b\[[0-9;]*[A-Za-z]", "", s)


def wait_tcp(timeout_s: float = 90.0) -> socket.socket:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            s = socket.create_connection((HOST, PORT), timeout=3)
            s.settimeout(1.0)
            print(f"TCP up after {timeout_s - (deadline - time.time()):.0f}s")
            return s
        except OSError:
            time.sleep(1.0)
    raise SystemExit("TCP never came up")


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
    out = drain(s, 0.15)
    preview = "\n".join(out.replace("\r", "").splitlines()[-15:])
    if preview.strip():
        print(preview)
    return out


def wait_markers(s: socket.socket, markers: list[str], timeout_s: float) -> str:
    deadline = time.time() + timeout_s
    buf = ""
    while time.time() < deadline:
        more = drain(s, 0.5)
        if more:
            buf += more
            print(more.replace("\r", "")[-600:])
        if any(m in buf for m in markers):
            return buf
        # poll status for idle occasionally
        st = cmd(s, "status", 0.4)
        buf += st
        if "Bring-up complete" in buf or "OK Bring-up" in buf:
            return buf
        if re.search(r"Busy:\s*No", st, re.I) and "Busy: No" in markers:
            return buf
    return buf


def main() -> int:
    s = wait_tcp(90)
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
    print(banner[-300:])
    drain(s, 0.5)

    cmd(s, "stop", 1.0)
    time.sleep(1.0)
    cmd(s, "alarmreset", 1.0)
    cmd(s, "enable", 4.0)
    cmd(s, "faults", 1.0)

    print("\n=== calibrate all (bring-up) ===")
    cmd(s, "calibrate all", 2.0)
    out = wait_markers(
        s,
        ["OK Bring-up complete", "Bring-up complete", "ERROR: Bring-up"],
        360,
    )
    if "OK Bring-up complete" not in out and "Bring-up complete" not in out:
        print("BRINGUP_FAIL", file=sys.stderr)
        cmd(s, "faults", 1.0)
        cmd(s, "disable", 1.0)
        s.close()
        return 2

    # Near Z+ band: z_max ~147 → clearance ~117. Move to high Z for coordinated.
    print("\n=== move high (expect coordinated if both change) ===")
    cmd(s, "move 35 130 0", 1.0)
    wait_markers(s, ["Busy: No"], 90)
    st = cmd(s, "status", 0.8)
    print("--- after 35/130 ---")
    print("\n".join(st.replace("\r", "").splitlines()[-8:]))

    print("\n=== move across + descend below clearance ===")
    cmd(s, "move 200 10 0", 1.0)
    wait_markers(s, ["Busy: No", "[PATH]"], 120)
    st = cmd(s, "status", 0.8)
    print("--- after 200/10 ---")
    print("\n".join(st.replace("\r", "").splitlines()[-10:]))

    faults = cmd(s, "faults", 1.0)
    cmd(s, "disable", 1.0)
    cmd(s, "logout", 0.5)
    s.close()

    if "Alarm: Yes" in faults:
        print("LIVE_SMOKE_FAIL faults", file=sys.stderr)
        return 2
    print("LIVE_SMOKE_OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

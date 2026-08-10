#!/usr/bin/env python3
"""Self-checks for tools/lan_debug_ui.py.

Not part of the `test/host` ctest suite: this needs Tk and a desktop session, so it
is run by hand rather than by CI or the commit hook.

    py tools/test_lan_debug_ui.py

The byte sequences below are the real thing - they were captured from
`src/gantry_net_console.cpp` and confirmed against a live WT32 at
192.168.1.100:2323. The framing cases matter because the firmware's `Password: `
and `> ` prompts arrive with **no trailing newline**, so a purely line-based
reader would stall on both.
"""

from __future__ import annotations

import codecs
import queue
import sys
import tkinter as tk
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import lan_debug_ui as ui  # noqa: E402

failures: list[str] = []


def check(label: str, got: object, want: object) -> None:
    if got != want:
        failures.append(f"{label}\n     got  {got!r}\n     want {want!r}")
        print(f"  FAIL {label}")
    else:
        print(f"  ok   {label}")


def drain(chunks: list[bytes]) -> tuple[list[tuple[str, str]], str]:
    """Feed byte chunks through the client's framing; return (events, residue).

    Drives ConsoleClient._drain directly so the framing can be tested without a
    socket or a reader thread.
    """
    events: queue.Queue[tuple[str, str]] = queue.Queue()
    client = ui.ConsoleClient(events)
    decoder = codecs.getincrementaldecoder("utf-8")(errors="replace")
    buf = ""
    for chunk in chunks:
        buf += ui.ANSI_RE.sub("", decoder.decode(chunk))
        buf = client._drain(buf)
    out = []
    while not events.empty():
        out.append(events.get_nowait())
    return out, buf


def test_framing() -> None:
    print("== framing ==")

    got, rest = drain([b"Authentication required. Enter password:\r\nPassword: "])
    check(
        "auth prompt splits the line then flushes promptless 'Password: '",
        got,
        [("line", "Authentication required. Enter password:"), ("prompt", "Password: ")],
    )
    check("auth prompt leaves no residue", rest, "")

    got, _ = drain([b"OK authenticated (recent IP)\r\n"])
    check("recent-IP auth arrives as a plain line", got, [("line", "OK authenticated (recent IP)")])

    banner = (
        "WT32 gantry console (TCP 2323) \u2014 authenticated\r\n"
        "LAN log ON (min INFO). Same commands as UART.\r\n"
        "Type help (or logout to disconnect)\r\n> "
    ).encode("utf-8")
    got, rest = drain([banner])
    check(
        "banner yields three lines plus the ready prompt",
        got,
        [
            ("line", "WT32 gantry console (TCP 2323) \u2014 authenticated"),
            ("line", "LAN log ON (min INFO). Same commands as UART."),
            ("line", "Type help (or logout to disconnect)"),
            ("prompt", "> "),
        ],
    )
    check("banner leaves no residue", rest, "")

    got, _ = drain([b"I (12345) GantryConsole: OK Motors enabled\r\n\r\n> "])
    check(
        "command reply followed by the '\\r\\n> ' echo prompt",
        got,
        [("line", "I (12345) GantryConsole: OK Motors enabled"), ("line", ""), ("prompt", "> ")],
    )

    got, _ = drain([b"Passw", b"ord: "])
    check("prompt split across two recv calls", got, [("prompt", "Password: ")])

    em_dash = "x=\u2014 em dash".encode("utf-8")
    _got, rest = drain([em_dash[:5], em_dash[5:]])
    check("multibyte char split across chunks is not corrupted", rest, "x=\u2014 em dash")

    got, _ = drain([b"\x1b[0;32mI (1) T: green\x1b[0m\r\n"])
    check("ANSI colour codes are stripped", got, [("line", "I (1) T: green")])

    got, rest = drain([b"partial line no newline"])
    check(
        "partial non-prompt line is held back",
        (got, rest),
        ([], "partial line no newline"),
    )


def test_parsing_and_gating() -> None:
    print("\n== parsing / gating ==")

    root = tk.Tk()
    root.withdraw()
    app = ui.LanDebugApp(root, "192.168.1.100", 2323, "pw")

    app._handle_line(
        "I (10) GT: LIVE POS: x_cmd=12.34 mm, x_enc=12.30 mm, z=5.60 mm, theta=90.00 deg"
    )
    check("LIVE POS x", app.state_vars["x"].get(), "12.34 mm")
    check("LIVE POS x_enc", app.state_vars["x_enc"].get(), "12.30")
    check("LIVE POS z", app.state_vars["z"].get(), "5.60")
    check("LIVE POS theta", app.state_vars["theta"].get(), "90.00")
    check("LIVE POS units", app.state_vars["units"].get(), "mm")

    app._handle_line("I (11) GT: X Position: 123.456 mm")
    check("status X Position", app.state_vars["x"].get(), "123.456 mm")
    app._handle_line("I (12) GT: Z Position: 7.500 mm (+Z = up; belt = 0)")
    check("status Z Position", app.state_vars["z"].get(), "7.500 mm")
    app._handle_line("I (13) GT: Motor Enabled: Yes")
    check("status Motor Enabled", app.state_vars["enabled"].get(), "Yes")
    app._handle_line("I (14) GT: Busy: No")
    check("status Busy", app.state_vars["busy"].get(), "No")
    app._handle_line("I (15) GT: Alarm: No")
    check("status Alarm", app.state_vars["alarm"].get(), "No")
    app._handle_line("I (16) GT: Theta: 45 deg")
    check("status Theta", app.state_vars["theta"].get(), "45")
    app._handle_line("I (17) GantryConsole: X Encoder : 0 pulses")
    check("status X Encoder", app.state_vars["x_enc"].get(), "0")

    check("move gated while offline", str(app.btn_move["state"]), "disabled")

    # Fake a live socket so gating is exercised in the state it ships in.
    app.client._sock = "fake-socket"
    app._set_online(True)
    check("move still gated when online but not homed", str(app.btn_move["state"]), "disabled")

    app._handle_line("I (20) X: OK X homing started (use 'stop' to abort, 'status' to monitor)")
    check("homed after an accepted home", app.homed, True)
    check("move still gated with only home", str(app.btn_move["state"]), "disabled")
    app._handle_line("I (21) X: OK Calibrated length: 420 mm")
    check("calibrated after a measured length", app.calibrated, True)
    check("move enabled after home + calibrate", str(app.btn_move["state"]), "normal")

    app.client._sock = None
    app._refresh_move_gate()
    check("move re-gated when the link drops", str(app.btn_move["state"]), "disabled")
    app.client._sock = "fake-socket"
    app._refresh_move_gate()

    app._handle_line("E (22) X: Calibration failed")
    check("calibrate gate drops on failure", app.calibrated, False)
    check("move re-gated after a calibration failure", str(app.btn_move["state"]), "disabled")

    app._handle_line(
        "I (23) X: OK X soft-calibrate (no limit switches). Joint envelope X=0.0..420.0 mm"
    )
    check("soft-calibrate also satisfies the gate", app.calibrated, True)

    app.gate_override.set(True)
    app.homed = False
    app.calibrated = False
    app._refresh_move_gate()
    check("'already homed' override re-enables move", str(app.btn_move["state"]), "normal")
    app.gate_override.set(False)
    app._refresh_move_gate()
    check("clearing the override re-gates move", str(app.btn_move["state"]), "disabled")

    app._handle_line("I (24) GT: OK Motors disabled")
    check("disable clears enabled", app.state_vars["enabled"].get(), "No")

    app._handle_line("I (25) GantryConsole: Units: linear=in (internal mm)")
    check("status 'Units: linear=' parsed", app.state_vars["units"].get(), "in")
    app._handle_line(
        "I (26) GantryConsole: OK Linear units set to mm (internal storage remains mm)"
    )
    check("units command reply parsed", app.state_vars["units"].get(), "mm")
    app._handle_line("I (27) GantryConsole:   units <mm|in>        - set linear input/output units")
    check("help text does not clobber units", app.state_vars["units"].get(), "mm")

    app.authed = False
    app._handle_prompt("> ")
    check("ready prompt marks the session authenticated", app.authed, True)

    app._handle_line("I (30) GT: OK soft-home (X+Z). Joint datum = current drive positions")
    check("soft-home satisfies the home gate", app.homed, True)

    before = len(app.records)
    app._handle_line("")
    check("blank lines are not logged", len(app.records), before)

    app.filter_var.set("Calibrated")
    app._rerender()
    shown = app.log.get("1.0", tk.END).strip().splitlines()
    check("filter keeps only matching lines", all("Calibrated" in ln for ln in shown), True)
    check("filter matched at least one line", len(shown) >= 1, True)

    root.destroy()


def main() -> int:
    test_framing()
    test_parsing_and_gating()
    print()
    if failures:
        print(f"{len(failures)} FAILURE(S):")
        for failure in failures:
            print("  - " + failure)
        return 1
    print("all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())

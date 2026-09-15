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
import os
import queue
import socket
import sys
import tempfile
import threading
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
    app._handle_line(
        "I (12) GT: Z Position: 7.500 mm (+Z = down / toward belt; 0 = A015 retract)"
    )
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

    app.homed = False
    app.calibrated = False
    app.z_homed = False
    app.z_calibrated = False
    app._refresh_move_gate()
    check("move gated after clearing all session flags", str(app.btn_move["state"]), "disabled")
    app._handle_line(
        "I (21b) GantryConsole: OK Z homing started (use 'stop' to abort, 'status' to monitor)"
    )
    check("z_homed after Z home", app.z_homed, True)
    check("move still gated with only Z home", str(app.btn_move["state"]), "disabled")
    app._handle_line("I (21c) GantryConsole: OK Z Calibrated length: 148 mm")
    check("z_calibrated after Z cal", app.z_calibrated, True)
    check("move enabled after Z home + calibrate only", str(app.btn_move["state"]), "normal")

    app.client._sock = None
    app._refresh_move_gate()
    check("move re-gated when the link drops", str(app.btn_move["state"]), "disabled")
    app.client._sock = "fake-socket"
    app._refresh_move_gate()

    app._handle_line("E (22) X: Calibration failed")
    check("calibrate gate drops on failure", app.calibrated, False)
    app.z_homed = False
    app.z_calibrated = False
    app._refresh_move_gate()
    check("move re-gated after a calibration failure", str(app.btn_move["state"]), "disabled")

    app._handle_line(
        "I (23) X: OK X soft-calibrate (no limit switches). Joint envelope X=0.0..420.0 mm"
    )
    check("soft-calibrate also satisfies the gate", app.calibrated, True)

    app.gate_override.set(True)
    app.homed = False
    app.calibrated = False
    app.z_homed = False
    app.z_calibrated = False
    app._refresh_move_gate()
    check("'skip calibrated gate' override re-enables move", str(app.btn_move["state"]), "normal")
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

    names = [name for name, _usage, _desc in ui.COMMANDS]
    check("COMMANDS includes test_cycle", "test_cycle" in names, True)
    check("COMMANDS includes ota", "ota" in names, True)
    check("COMMANDS includes thetalim", "thetalim" in names, True)
    check("COMMANDS includes test_theta_path", "test_theta_path" in names, True)
    check("COMMANDS includes autotune", "autotune" in names, True)
    check("COMMANDS includes puu", "puu" in names, True)
    check("COMMANDS includes calibrated", "calibrated" in names, True)
    check("default host is plant LAN", ui.DEFAULT_HOST, "10.42.0.100")
    move_help = next(desc for name, _usage, desc in ui.COMMANDS if name == "move")
    check("move catalog is +Z=down", "+Z=down" in move_help, True)

    app.homed = False
    app.calibrated = False
    app.z_homed = False
    app.z_calibrated = False
    app.t_homed = False
    app.t_calibrated = False
    app._refresh_move_gate()
    app._handle_line("I (50) GantryConsole: OK Theta origin captured (home t)")
    check("t_homed after theta origin", app.t_homed, True)
    check("move still gated with only theta home", str(app.btn_move["state"]), "disabled")
    app._handle_line("I (51) GantryConsole: OK Theta calibrated: stroke=360 deg")
    check("t_calibrated after theta cal", app.t_calibrated, True)
    check("move enabled after theta home + calibrate only", str(app.btn_move["state"]), "normal")

    app._handle_line(
        "I (52) GantryConsole: OK Path speed updated: 250.000 mm/s (resultant), theta=30 deg/s"
    )
    check("Path speed updated also sets theta deg/s", app.speed_deg.get(), "30")
    app._handle_line(
        "I (53) GantryConsole: Theta Profile: speed=45 deg/s, accel=1800 deg/s2, "
        "decel=1800 deg/s2"
    )
    check("Theta Profile speed", app.speed_deg.get(), "45")
    check("Theta Profile accel", app.accel_theta.get(), "1800")

    app.homed = False
    app.calibrated = False
    app.z_homed = False
    app.z_calibrated = False
    app.t_homed = False
    app.t_calibrated = False
    app._refresh_move_gate()
    app._handle_line(
        "I (38) GantryConsole: OK test_cycle started (use 'stop' to abort, 'status' to monitor)"
    )
    check("test_cycle started does not set session gates", app.homed, False)
    check("test_cycle started does not set Z home", app.z_homed, False)
    app._handle_line("E (39) GantryConsole: ERROR: test_cycle FAIL")
    check("test_cycle FAIL does not invent X home", app.homed, False)
    check("test_cycle FAIL does not invent X cal", app.calibrated, False)

    app._handle_line(
        "I (40) GantryConsole: OK Bring-up complete: X stroke=419 mm, Z stroke=149 mm, "
        "SAFE_Z ceiling=30.0 mm (z_min + 30.0)"
    )
    check("Bring-up complete sets X home", app.homed, True)
    check("Bring-up complete sets X cal", app.calibrated, True)
    check("Bring-up complete sets Z home", app.z_homed, True)
    check("Bring-up complete sets Z cal", app.z_calibrated, True)
    check("move enabled after Bring-up complete", str(app.btn_move["state"]), "normal")

    app.homed = False
    app.calibrated = False
    app.z_homed = False
    app.z_calibrated = False
    app._refresh_move_gate()
    app._handle_line("I (41) GantryConsole: OK test_cycle PASS")
    check("test_cycle PASS sets X home", app.homed, True)
    check("test_cycle PASS sets Z cal", app.z_calibrated, True)
    check("move enabled after test_cycle PASS", str(app.btn_move["state"]), "normal")

    app._handle_line(
        "I (42) GantryConsole: 2-D Path Profile: speed=500.000 mm/s, theta=30 deg/s, "
        "accel=3000.000 mm/s2, decel=3000.000 mm/s2"
    )
    check("status Path Profile speed", app.speed_lin.get(), "500")
    check("status Path Profile accel", app.accel_var.get(), "3000")
    app._handle_line(
        "I (43) GantryConsole: [TEST_CYCLE] path profile: speed=50 mm/s accel=3000 mm/s2 "
        "decel=3000 mm/s2 (console speed/accel; next leg picks up live changes)"
    )
    check("TEST_CYCLE path profile speed", app.speed_lin.get(), "50")
    app._handle_line(
        "I (44) GantryConsole: OK Path speed updated: 250.000 mm/s (resultant), theta=30 deg/s"
    )
    check("Path speed updated", app.speed_lin.get(), "250")
    app._handle_line(
        "I (45) GantryConsole: OK Path accel updated: accel=2000.000 mm/s2, decel=2000.000 mm/s2 "
        "(resultant)"
    )
    check("Path accel updated", app.accel_var.get(), "2000")

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
    check(
        "filter keeps only matching lines",
        all("calibrated" in ln.lower() for ln in shown),
        True,
    )
    check("filter matched at least one line", len(shown) >= 1, True)

    app._fade_workspace()
    check("workspace faded after reset", app.grid_active, False)
    app._handle_line("I (60) GantryConsole: OK Calibrated length: 491 mm")
    check("X stroke ingested", app.stroke_x_mm, 491.0)
    check("grid still faded without Z stroke", app.grid_active, False)
    app._handle_line("I (61) GantryConsole: OK Z Calibrated length: 106 mm")
    check("Z stroke ingested", app.stroke_z_mm, 106.0)
    check("grid active after X+Z calibrate", app.grid_active, True)
    app._fade_workspace()
    app._handle_line(
        "I (62) GantryConsole: OK Bring-up complete: X stroke=419 mm, Z stroke=149 mm, "
        "SAFE_Z ceiling=30.0 mm (z_min + 30.0)"
    )
    check("bring-up reconstructs X stroke", app.stroke_x_mm, 419.0)
    check("bring-up reconstructs Z stroke", app.stroke_z_mm, 149.0)
    check("bring-up activates grid", app.grid_active, True)
    x0, y0, x1, y1 = app._workspace_plot_box()

    class _E:
        x = (x0 + x1) / 2
        y = (y0 + y1) / 2
        state = 0

    app._on_workspace_click(_E())  # type: ignore[arg-type]
    check("click places one waypoint", len(app.waypoints), 1)
    check("waypoint near mid-X", abs(app.waypoints[0].x_mm - 419.0 / 2) < 5.0, True)
    check("move_x filled from waypoint", float(app.move_x.get()) > 0.0, True)
    check("new waypoint default grip open", app.waypoints[0].grip, 0)
    check("bring-up parsed SAFE_Z ceiling", app.safe_z_ceiling_mm, 30.0)
    check("listbox exportselection off", int(str(app.wp_list.cget("exportselection"))), 0)

    placed = app.waypoints[0]
    px, py = app._mm_to_canvas(placed.x_mm, placed.z_mm)

    class _Near:
        x = px
        y = py
        state = 0

    app._on_workspace_click(_Near())  # type: ignore[arg-type]
    check("click near existing selects not places", len(app.waypoints), 1)
    check("click near selects that waypoint", list(app.wp_list.curselection()), [0])

    class _CtrlNear:
        x = px
        y = py
        state = 0x0004

    app._on_workspace_click(_CtrlNear())  # type: ignore[arg-type]
    check("ctrl+click places even on existing", len(app.waypoints), 2)

    app.waypoints = [
        ui.Waypoint(10.0, 20.0, 0.0, 0),
        ui.Waypoint(30.0, 40.0, 90.0, 1),
        ui.Waypoint(50.0, 60.0, -45.0, 0),
    ]
    app._refresh_waypoint_list(keep_selection=[])
    app.wp_list.selection_set(0, 2)
    check("multi-select three waypoints", list(app.wp_list.curselection()), [0, 1, 2])
    check("follow ascending indices", app._follow_indices(True), [0, 1, 2])
    check("follow descending indices", app._follow_indices(False), [2, 1, 0])
    app.wp_list.selection_clear(0, tk.END)
    app.wp_list.selection_set(0)
    app.wp_list.selection_set(2)
    check("follow ignores selection (still full list)", app._follow_indices(True), [0, 1, 2])
    check("follow descending still full list", app._follow_indices(False), [2, 1, 0])
    app.wp_list.selection_clear(0, tk.END)
    app.wp_list.selection_set(1)
    app._reorder_waypoints(-1)
    check("reorder up moves mid to front", app.waypoints[0].x_mm, 30.0)
    check("reorder keeps selection on moved item", list(app.wp_list.curselection()), [0])
    check("reorder preserves theta", app.waypoints[0].theta_deg, 90.0)
    check("reorder preserves grip close", app.waypoints[0].grip, 1)
    app._reorder_waypoints(1)
    check("reorder down restores order head", app.waypoints[0].x_mm, 10.0)
    app._commit_waypoint(1, 30.0, 40.0, -12.5, 1)
    check("commit sets theta on index", app.waypoints[1].theta_deg, -12.5)
    check("commit sets grip close", app.waypoints[1].grip, 1)
    check("list label shows close", "close" in app.wp_list.get(1), True)
    app._commit_waypoint(1, 30.0, 40.0, 45.0, 0)
    check("commit sets theta again", app.waypoints[1].theta_deg, 45.0)
    check("commit sets grip open", app.waypoints[1].grip, 0)

    opened: list[int] = []
    app._edit_waypoint_dialog = lambda i: opened.append(i)  # type: ignore[method-assign]
    px, py = app._mm_to_canvas(app.waypoints[0].x_mm, app.waypoints[0].z_mm)

    class _Edit:
        x = px
        y = py
        state = 0

    app._on_workspace_double_click(_Edit())  # type: ignore[arg-type]
    check("double-click canvas edits nearest", opened, [0])
    opened.clear()

    class _ListEvt:
        y = 0

    app._on_waypoint_list_double(_ListEvt())  # type: ignore[arg-type]
    check("double-click list edits a row", len(opened) == 1, True)

    check("trace checkbox present", hasattr(app, "wp_trace"), True)
    app.wp_trace.set(True)
    app.wp_follow_phase = "busy"
    app.wp_trace_pts = []
    app._set_live_xz_from_text("10.0", "20.0")
    app._set_live_xz_from_text("12.0", "22.0")
    check("trace records live follow samples", len(app.wp_trace_pts) >= 2, True)
    app.wp_follow_phase = "idle"
    before = len(app.wp_trace_pts)
    app._set_live_xz_from_text("14.0", "24.0")
    check("trace frozen after follow idle", len(app.wp_trace_pts), before)
    app.wp_trace.set(False)
    app._on_trace_toggle()
    app.wp_follow_phase = "busy"
    app._set_live_xz_from_text("16.0", "26.0")
    check("trace off does not append", len(app.wp_trace_pts), before)

    loaded = ui.parse_waypoints_txt(
        "# comment\n"
        "10.0  20.0  90  close\n"
        "30,40,-12.5,open\n"
    )
    app._apply_loaded_waypoints(loaded)
    check("load txt count", len(app.waypoints), 2)
    check("load txt close grip", app.waypoints[0].grip, 1)
    check("load csv theta", app.waypoints[1].theta_deg, -12.5)
    dumped = ui.format_waypoints_txt(app.waypoints)
    again = ui.parse_waypoints_txt(dumped)
    check("save/load roundtrip count", len(again), 2)
    check("save/load roundtrip x", again[0].x_mm, 10.0)

    check("OTA default bin name", Path(app.ota_bin.get()).name, "wt32_eth01_gantry.bin")
    check("OTA default port", app.ota_port.get(), "8032")
    check("OTA flash enabled while console offline", str(app.btn_ota_flash["state"]), "normal")
    app.state_vars["enabled"].set("Yes")
    check("OTA blocked when motors enabled", app._ota_motion_block_reason() is not None, True)
    app.state_vars["enabled"].set("No")
    app.state_vars["busy"].set("Yes")
    check("OTA blocked when busy", app._ota_motion_block_reason() is not None, True)
    app.state_vars["busy"].set("No")
    check("OTA allowed when disabled idle", app._ota_motion_block_reason(), None)

    app.client._sock = "fake-socket"
    app.workspace_cal_known = False
    app.workspace_calibrated = False
    app.homed = True
    app.calibrated = True
    app.z_homed = False
    app.z_calibrated = False
    app.t_homed = False
    app.t_calibrated = False
    app.gate_override.set(False)
    app._refresh_move_gate()
    check("fallback enables move from X home+cal", str(app.btn_move["state"]), "normal")
    app._handle_line("I (90) Gantry: Workspace calibrated: No")
    check("Workspace No disables move", str(app.btn_move["state"]), "disabled")
    check("Workspace No disables follow", str(app.btn_follow_up["state"]), "disabled")
    check("Workspace No disables theta path", str(app.btn_theta_path["state"]), "disabled")
    check("Workspace No disables test cycle", str(app.btn_test_cycle["state"]), "disabled")
    app._handle_line("I (91) GantryConsole: Workspace calibrated: Yes")
    check("Workspace Yes enables move", str(app.btn_move["state"]), "normal")
    check("Workspace Yes enables follow", str(app.btn_follow_up["state"]), "normal")

    root.destroy()


def test_waypoint_file() -> None:
    print("\n== waypoint txt ==")
    text = ui.format_waypoints_txt(
        [ui.Waypoint(35.2, 10.0, 90.0, 1), ui.Waypoint(0.0, 35.7, 0.0, 0)]
    )
    got = ui.parse_waypoints_txt(text)
    check("format then parse count", len(got), 2)
    check("format then parse grip close", got[0].grip, 1)
    check("format then parse SAFE_Z z", got[1].z_mm, 35.7)
    check("parse skips header words", len(ui.parse_waypoints_txt("x_mm z_mm\n1 2 3 open\n")), 1)
    try:
        ui.parse_waypoints_txt("1 2 3 maybe\n")
        check("bad grip raises", False, True)
    except ValueError:
        check("bad grip raises", True, True)


def test_ota_protocol() -> None:
    print("\n== Dual-OTA protocol ==")
    import eth_ota_flash

    payload = os.urandom(5000)
    received = bytearray()
    port_box: list[int] = []
    ready = threading.Event()
    err_box: list[str] = []

    def server() -> None:
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("127.0.0.1", 0))
        port_box.append(int(srv.getsockname()[1]))
        srv.listen(1)
        ready.set()
        conn, _ = srv.accept()
        try:
            def readline() -> str:
                data = bytearray()
                while True:
                    ch = conn.recv(1)
                    if not ch or ch == b"\n":
                        break
                    if ch != b"\r":
                        data.extend(ch)
                return data.decode("utf-8", errors="replace")

            auth = readline()
            if not auth.startswith("AUTH "):
                err_box.append(f"bad AUTH {auth!r}")
                conn.sendall(b"ERR AUTH\n")
                return
            conn.sendall(b"OK AUTH\n")
            start = readline()
            if not start.startswith("START "):
                err_box.append(f"bad START {start!r}")
                conn.sendall(b"ERR INVALID_SIZE\n")
                return
            expect = int(start.split()[1])
            conn.sendall(b"OK READY\n")
            while len(received) < expect:
                chunk = conn.recv(min(4096, expect - len(received)))
                if not chunk:
                    break
                received.extend(chunk)
            conn.sendall(b"OK COMPLETE\n")
        finally:
            conn.close()
            srv.close()

    thread = threading.Thread(target=server, daemon=True)
    thread.start()
    if not ready.wait(2.0):
        check("OTA mock server bound", False, True)
        return
    handle = tempfile.NamedTemporaryFile(suffix=".bin", delete=False)
    try:
        handle.write(payload)
        handle.close()
        logs: list[str] = []
        ok = eth_ota_flash.flash_ota(
            "127.0.0.1", port_box[0], "pw", handle.name, log=logs.append,
        )
        check("flash_ota mock OK COMPLETE", ok, True)
        check("flash_ota streamed exact bytes", bytes(received), payload)
        check("flash_ota logged success", any("SUCCESS" in line for line in logs), True)
        check("OTA mock server clean", err_box, [])
    finally:
        try:
            os.unlink(handle.name)
        except OSError:
            pass
        thread.join(timeout=2.0)

    missing = eth_ota_flash.flash_ota(
        "127.0.0.1", 1, "pw", "no-such-ota.bin",
        log=lambda _m: None,
    )
    check("flash_ota missing bin fails", missing, False)


def main() -> int:
    test_framing()
    test_parsing_and_gating()
    test_waypoint_file()
    test_ota_protocol()
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

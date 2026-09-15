#!/usr/bin/env python3
"""LAN8720 gantry debug UI - ESP_LOG stream + full console command surface over TCP.

Covers every command in the firmware `help` menu (`gantryTestPrintHelp` in
`src/gantry_test_console.cpp`), grouped into tabs, with an always-visible STOP.
Diagnostics Dual-OTA streams a `.bin` to the LAN8720 OTA port (`:8032`) using
`tools/eth_ota_flash.py`.

Defaults match plant bring-up:
  WT32 LAN8720  10.42.0.100:2323  (console) / :8032 (Dual-OTA)
  Developer PC  on the plant LAN (this host)

Usage:
  py tools/lan_debug_ui.py
  py tools/lan_debug_ui.py --host 10.42.0.100 --port 2323
  py tools/lan_debug_ui.py --list-commands        # no GUI; print the command table
  set GANTRY_TCP_PASSWORD=LTU_1932                # optional; prompted if required

Wire protocol (src/gantry_net_console.cpp):
  - Auth either replies "OK authenticated (recent IP)" or prompts "Password: "
    (no trailing newline), so framing must flush on prompts, not only on newlines.
  - Every command reply is followed by "\\r\\n> ".
  - The server splits input on '\\n', '\\r' AND ';', and drops any byte outside
    ASCII 32..126, so commands are validated before send.
  - logout / exit / quit close the session.

Does not touch the W5500 EIP daisy-chain.
"""

from __future__ import annotations

import argparse
import codecs
import datetime as _dt
import os
import queue
import re
import socket
import threading
import tkinter as tk
from dataclasses import dataclass, replace
from pathlib import Path
from tkinter import filedialog, messagebox, scrolledtext, ttk

import eth_ota_flash

DEFAULT_HOST = "10.42.0.100"
DEFAULT_PORT = 2323
DEFAULT_PASSWORD = os.environ.get("GANTRY_TCP_PASSWORD", "LTU_1932")
DEFAULT_OTA_PORT = eth_ota_flash.DEFAULT_PORT
REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_OTA_BIN = REPO_ROOT / "idf" / "build" / "wt32_eth01_gantry.bin"

PROMPT_PASSWORD = "Password: "
PROMPT_READY = "> "

ANSI_RE = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
ESP_LINE_RE = re.compile(r"^([EWIDV])\s+\((\d+)\)\s+([^:]+):\s?(.*)$")

MAX_LOG_LINES = 6000

# Placeholder envelope (datasheet hard max) until calibrate reports real strokes.
PLACEHOLDER_X_MM = 550.0
PLACEHOLDER_Z_MM = 150.0
PLACEHOLDER_SAFE_Z_MM = 35.7  # firmware GANTRY_SAFE_Z_HEIGHT_MM until bring-up reports
WORKSPACE_CANVAS_W = 560
WORKSPACE_CANVAS_H = 220
WORKSPACE_PAD = (40, 16, 18, 30)  # left, right, top, bottom


@dataclass
class Waypoint:
    x_mm: float
    z_mm: float
    theta_deg: float = 0.0
    grip: int = 0  # 0=open, 1=close

    def grip_label(self) -> str:
        return "close" if self.grip else "open"


def format_waypoints_txt(waypoints: list[Waypoint]) -> str:
    """Plain-text waypoint list. Positions are millimetres (+Z down)."""
    lines = [
        "# WT32 gantry waypoints",
        "# x_mm  z_mm  theta_deg  grip   (+Z down; z=0 is A015 retract)",
        "# grip = open|close or 0|1",
    ]
    for wp in waypoints:
        lines.append(
            f"{wp.x_mm:.4f}  {wp.z_mm:.4f}  {wp.theta_deg:g}  {wp.grip_label()}"
        )
    return "\n".join(lines) + "\n"


def _parse_grip_token(token: str) -> int:
    raw = token.strip().lower()
    if raw in ("1", "close", "closed", "on"):
        return 1
    if raw in ("0", "open", "off"):
        return 0
    raise ValueError(f"grip must be open/close or 0/1 (got {token!r})")


def parse_waypoints_txt(text: str) -> list[Waypoint]:
    """Parse format_waypoints_txt output (comments, blanks, comma or space)."""
    points: list[Waypoint] = []
    for lineno, raw in enumerate(text.splitlines(), start=1):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.replace(",", " ").split()
        if not parts:
            continue
        try:
            float(parts[0])
        except ValueError:
            continue
        if len(parts) < 2:
            raise ValueError(f"line {lineno}: need at least X and Z")
        try:
            x_mm = float(parts[0])
            z_mm = float(parts[1])
            theta = float(parts[2]) if len(parts) > 2 else 0.0
            grip = _parse_grip_token(parts[3]) if len(parts) > 3 else 0
        except ValueError as exc:
            raise ValueError(f"line {lineno}: {exc}") from exc
        points.append(Waypoint(x_mm, z_mm, theta, 1 if grip else 0))
    return points

# Mirrors gantryTestPrintHelp(). Commands marked optional are compiled out unless
# the matching Kconfig symbol is set, so they may answer "Unknown command".
COMMANDS: list[tuple[str, str, str]] = [
    ("help", "help | ?", "show firmware help"),
    ("status", "status", "print gantry status"),
    ("calibrated", "calibrated",
     "query workspace calibrated latch (Yes until a drive loses position)"),
    ("faults", "faults | alarms", "decode X/Z Kinetix and theta HCS01 diag"),
    ("puuinfo", "puuinfo", "print X/Z PUU/mm and theta PUU/deg"),
    ("eiptiming", "eiptiming", "dump Class 1 latency p50/p99 (exchange/ot/cycle/cmd2start)"),
    ("puu", "puu t <scale>", "set live theta PUU/deg (re-run home t)"),
    ("puucal", "puucal <x|z|t> <cmd> <meas>", "suggest (x/z) or apply (t) PUU scale"),
    ("thetalim", "thetalim <min> <max>", "set theta software joint limits (deg)"),
    ("autotune", "autotune [theta|t]",
     "DISABLED this version (firmware GANTRY_CONSOLE_AUTOTUNE=0)"),
    ("ota", "ota", "print Dual-OTA slot, compile stamp, rollback status"),
    ("limits", "limits", "read limit switches"),
    ("pins", "pins", "print active pin configuration"),
    ("mcp_pin_mode", "mcp_pin_mode <pin> <inpu|in|out0|out1>", "force MCP pin mode (optional build)"),
    ("mcp_dump", "mcp_dump <a|b>", "dump MCP IOCON/dir/pullup/olat/gpio (optional build)"),
    ("mcp_reg", "mcp_reg <r|w> <reg> [val]", "raw MCP register read/write (optional build)"),
    ("field_dout", "field_dout <0..3> <0|1>", "set Field 24 V DOUT (0=gripper PA0) (optional build)"),
    ("field_din", "field_din", "read Field DIN + encoder + W5500 INT (optional build)"),
    ("gpio_drive", "gpio_drive <gpio> <0|1>", "drive a direct ESP32 GPIO"),
    ("enable", "enable", "enable motors"),
    ("disable", "disable", "disable motors"),
    ("home", "home [x|z|t|all]", "home (EIP X/Z + HIPERFACE t; all=Z then X then t; X needs SAFE_Z)"),
    ("calibrate", "calibrate [x|z|t|all]",
     "calibrate; 'all' = EIP bring-up (Z- → X → park 35 → Z+ → SAFE_Z 35.7 → theta)"),
    ("units", "units <mm|in>", "set linear input/output units"),
    ("speed", "speed <v> [deg_per_s]", "set 2-D path speed (resultant) and optional theta deg/s"),
    ("accel", "accel <a> [d] [ta] [td]", "path accel/decel; optional theta accel/decel (deg/s2)"),
    ("rangelimit", "rangelimit <0|1>", "enable/disable path speed+accel/decel range clamps"),
    ("livepos", "livepos <hz>", "LIVE POS periodic rate (0=off); hz is required"),
    ("axislog", "axislog <hz>", "per-axis MOVE periodic rate (0=off); hz is required"),
    ("move", "move <x> <z> <theta>", "move to (x_linear, z_linear, theta_deg); +Z=down, z=A015 retract"),
    ("grip", "grip <0|1>", "gripper (0=open, 1=close)"),
    ("test_cycle", "test_cycle",
     "8-stage holistic: arm, bring-up, bounds, A-F, pick+place, dynamics, theta G-I, telemetry"),
    ("test_theta_path", "test_theta_path",
     "combined in-band X+Z+theta (25-75 window); enable+bring-up first"),
    ("stop", "stop", "stop all motion (also disables servos)"),
    ("alarmreset", "alarmreset | arst", "pulse EIP FaultReset / HCS01 C0500 bit5"),
    ("selftest", "selftest", "run basic math/config tests (optional build)"),
    ("logout", "logout | exit | quit", "close the TCP session"),
]


class ConsoleClient:
    """Socket reader/writer with prompt-aware framing.

    Events are pushed onto `events` as (kind, payload) where kind is one of
    "line", "prompt", "info", "closed".
    """

    def __init__(self, events: "queue.Queue[tuple[str, object]]") -> None:
        self.events = events
        self._sock: socket.socket | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._send_lock = threading.Lock()

    @property
    def connected(self) -> bool:
        return self._sock is not None

    def connect(self, host: str, port: int, timeout: float = 5.0) -> None:
        sock = socket.create_connection((host, port), timeout=timeout)
        sock.settimeout(0.4)
        self._sock = sock
        self._stop.clear()
        self._thread = threading.Thread(target=self._pump, name="lan-reader", daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        sock, self._sock = self._sock, None
        if sock is None:
            return
        try:
            sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            sock.close()
        except OSError:
            pass

    def send_line(self, text: str) -> None:
        sock = self._sock
        if sock is None:
            raise OSError("not connected")
        with self._send_lock:
            sock.sendall((text + "\n").encode("ascii", errors="replace"))

    def _pump(self) -> None:
        decoder = codecs.getincrementaldecoder("utf-8")(errors="replace")
        buf = ""
        reason = "connection closed by peer"
        while not self._stop.is_set():
            sock = self._sock
            if sock is None:
                reason = "disconnected locally"
                break
            try:
                chunk = sock.recv(4096)
            except socket.timeout:
                continue
            except OSError as exc:
                reason = f"socket error: {exc}"
                break
            if not chunk:
                break
            buf += ANSI_RE.sub("", decoder.decode(chunk))
            buf = self._drain(buf)
        if buf.strip():
            self.events.put(("line", buf))
        self.events.put(("closed", reason))

    def _drain(self, buf: str) -> str:
        """Emit whole lines, then any trailing prompt that has no newline."""
        while True:
            positions = [buf.find(sep) for sep in ("\r\n", "\n", "\r")]
            positions = [p for p in positions if p >= 0]
            if not positions:
                break
            cut = min(positions)
            sep_len = 2 if buf[cut : cut + 2] == "\r\n" else 1
            self.events.put(("line", buf[:cut]))
            buf = buf[cut + sep_len :]
        for prompt in (PROMPT_PASSWORD, PROMPT_READY):
            if buf.endswith(prompt):
                head = buf[: -len(prompt)]
                if head.strip():
                    self.events.put(("line", head))
                self.events.put(("prompt", prompt))
                return ""
        return buf


class LanDebugApp:
    def __init__(self, root: tk.Tk, host: str, port: int, password: str) -> None:
        self.root = root
        self.events: queue.Queue[tuple[str, object]] = queue.Queue()
        self.client = ConsoleClient(self.events)

        self.authed = False
        self.pw_sent_for_prompt = False
        self.history: list[str] = []
        self.history_idx = 0
        self.records: list[tuple[str, str]] = []  # (severity, text)

        # Workspace calibrated latch (firmware Gantry flag). Until the first
        # "Workspace calibrated:" log, fall back to per-axis home+cal so older
        # firmware still gates. `stop` / disable do NOT clear the latch.
        self.homed = False
        self.calibrated = False
        self.z_homed = False
        self.z_calibrated = False
        self.t_homed = False
        self.t_calibrated = False
        self.workspace_calibrated = False
        self.workspace_cal_known = False
        self.move_command_widgets: list[tk.Widget] = []
        # Workspace grid: faded until X+Z stroke known from successful calibrate.
        self.grid_active = False
        self.stroke_x_mm: float | None = None
        self.stroke_z_mm: float | None = None
        self.waypoints: list[Waypoint] = []
        self.live_xz: tuple[float, float] | None = None
        self._ws_hover: tuple[float, float] | None = None
        self.safe_z_ceiling_mm: float | None = None
        self._wp_suppress_select = False
        self._wp_sel_cache: list[int] = []
        self.wp_trace_pts: list[tuple[float, float]] = []
        # Sequence follow: queue of Waypoint; phase idle|sent|busy|gripping|advancing.
        self.wp_follow_queue: list[Waypoint] = []
        self.wp_follow_i = -1
        self.wp_follow_phase = "idle"
        self.wp_follow_target: Waypoint | None = None
        self.wp_follow_deadline_ms = 0
        self.wp_follow_gen = 0
        self.wp_follow_saw_busy = False
        self.wp_follow_last_grip: int | None = None
        self._wp_file_dir = str(REPO_ROOT)
        self._ota_busy = False
        self._ota_always_on: list[tk.Misc] = []

        root.title(f"WT32 gantry LAN debug - {host}:{port}")
        root.geometry("1180x920")
        root.minsize(900, 700)

        self._build_connection_bar(host, port, password)
        self._init_state_vars()
        self._build_safety_bar()
        self._build_tabs()
        self._build_console_toggle()
        self._build_log_pane()
        self._build_command_bar()

        self.control_widgets: list[ttk.Widget] = []
        self._collect_controls(self.tabs)
        self._set_online(False)
        self._set_ota_widgets_enabled(True)

        root.protocol("WM_DELETE_WINDOW", self.on_close)
        root.bind("<Escape>", lambda _e: self.cmd_stop())
        root.bind("<Control-l>", lambda _e: self.clear_log())
        root.bind("<Control-L>", lambda _e: self.clear_log())
        self.root.after(40, self._drain_events)

    # ---------------------------------------------------------------- layout

    def _build_connection_bar(self, host: str, port: int, password: str) -> None:
        bar = ttk.Frame(self.root, padding=(8, 6))
        bar.pack(fill=tk.X)
        ttk.Label(bar, text="Host").pack(side=tk.LEFT)
        self.host_var = tk.StringVar(value=host)
        ttk.Entry(bar, textvariable=self.host_var, width=16).pack(side=tk.LEFT, padx=(4, 10))
        ttk.Label(bar, text="Port").pack(side=tk.LEFT)
        self.port_var = tk.StringVar(value=str(port))
        ttk.Entry(bar, textvariable=self.port_var, width=7).pack(side=tk.LEFT, padx=(4, 10))
        ttk.Label(bar, text="Password").pack(side=tk.LEFT)
        self.pw_var = tk.StringVar(value=password)
        ttk.Entry(bar, textvariable=self.pw_var, width=14, show="*").pack(side=tk.LEFT, padx=(4, 10))
        self.btn_connect = ttk.Button(bar, text="Connect", command=self.toggle_connect)
        self.btn_connect.pack(side=tk.LEFT)
        self.conn_var = tk.StringVar(value="Disconnected")
        self.conn_label = ttk.Label(bar, textvariable=self.conn_var, foreground="#a00")
        self.conn_label.pack(side=tk.LEFT, padx=10)

    def _init_state_vars(self) -> None:
        """Parsed log state used by Move 'now', units, gates, and Dual-OTA — no strip."""
        self.state_vars = {
            key: tk.StringVar(value="-")
            for key in (
                "x", "x_enc", "z", "theta", "enabled", "busy", "alarm",
                "units", "homed", "calibrated",
            )
        }

    def _build_safety_bar(self) -> None:
        bar = tk.Frame(self.root, padx=8, pady=4)
        bar.pack(fill=tk.X)
        self.btn_stop = tk.Button(
            bar, text="STOP  (Esc)", command=self.cmd_stop,
            bg="#c62828", fg="white", activebackground="#8e0000",
            activeforeground="white", font=("Segoe UI", 12, "bold"),
            height=2, width=16, relief=tk.RAISED, bd=3,
        )
        self.btn_stop.pack(side=tk.LEFT)
        self.btn_disable = tk.Button(
            bar, text="Disable motors", command=lambda: self.send("disable"),
            bg="#ef6c00", fg="white", activebackground="#b53d00",
            activeforeground="white", font=("Segoe UI", 10, "bold"), height=2, width=16,
        )
        self.btn_disable.pack(side=tk.LEFT, padx=8)
        ttk.Label(
            bar,
            text="STOP aborts motion and disables the servos. Both stay live whenever connected.",
            foreground="#555",
        ).pack(side=tk.LEFT, padx=8)

    def _build_tabs(self) -> None:
        self.tabs = ttk.Notebook(self.root)
        self.tabs.pack(fill=tk.BOTH, expand=False, padx=8, pady=4)
        self._tab_motion()
        self._tab_profile()
        self._tab_diagnostics()
        self._tab_io()

    def _tab_motion(self) -> None:
        tab = ttk.Frame(self.tabs, padding=8)
        self.tabs.add(tab, text="Motion")

        arm = ttk.LabelFrame(tab, text="Servo", padding=6)
        arm.grid(row=0, column=0, sticky="nsew", padx=4, pady=4)
        ttk.Button(arm, text="Enable", command=self.cmd_enable, width=14).pack(pady=2)
        ttk.Button(arm, text="Disable", command=lambda: self.send("disable"), width=14).pack(pady=2)
        ttk.Button(arm, text="Alarm reset", command=lambda: self.send("alarmreset"), width=14).pack(pady=2)

        seq = ttk.LabelFrame(tab, text="Home / calibrate", padding=6)
        seq.grid(row=0, column=1, sticky="nsew", padx=4, pady=4)
        ttk.Label(seq, text="Axis").grid(row=0, column=0, sticky="w")
        self.home_axis = tk.StringVar(value="all")
        ttk.Combobox(
            seq, textvariable=self.home_axis, values=["x", "z", "t", "all"],
            width=6, state="readonly",
        ).grid(row=0, column=1, padx=4)
        ttk.Button(seq, text="Home", command=self.cmd_home, width=12).grid(row=1, column=0, pady=2)
        ttk.Button(seq, text="Calibrate", command=self.cmd_calibrate, width=12).grid(
            row=1, column=1, pady=2
        )
        ttk.Button(seq, text="Bring-up (all)", command=self.cmd_bringup, width=12).grid(
            row=2, column=0, pady=2
        )
        self.btn_test_cycle = ttk.Button(
            seq, text="Test cycle", command=self.cmd_test_cycle, width=12
        )
        self.btn_test_cycle.grid(row=2, column=1, pady=2)
        self.btn_theta_path = ttk.Button(
            seq, text="Theta path", command=self.cmd_test_theta_path, width=12
        )
        self.btn_theta_path.grid(row=3, column=0, columnspan=2, pady=2)
        ttk.Label(
            seq,
            text="Bring-up = calibrate all:\n"
                 "Z- → X → park X=35 → Z+ →\n"
                 "SAFE_Z 35.7 → theta.\n"
                 "Seek 100 mm/s, 2000 mm/s².",
            foreground="#555", font=("Segoe UI", 8),
        ).grid(row=4, column=0, columnspan=2, sticky="w", pady=(4, 0))

        mv = ttk.LabelFrame(tab, text="Move (absolute)", padding=6)
        mv.grid(row=0, column=2, sticky="nsew", padx=4, pady=4)
        self.move_x = tk.StringVar(value="0")
        self.move_z = tk.StringVar(value="0")
        self.move_t = tk.StringVar(value="0")
        self.move_x_label = ttk.Label(mv, text="X (mm)")
        self.move_z_label = ttk.Label(mv, text="Z (mm, +down)")
        self.move_t_label = ttk.Label(mv, text="Theta (deg)")
        self.move_x_label.grid(row=0, column=0, sticky="w")
        self.move_z_label.grid(row=0, column=1, sticky="w")
        self.move_t_label.grid(row=0, column=2, sticky="w")
        # Datasheet hard limits (soft max after calibrate all is smaller, e.g. ~491/106).
        self.move_x_spin = ttk.Spinbox(
            mv, textvariable=self.move_x, from_=0.0, to=550.0, increment=1.0, width=9,
        )
        self.move_x_spin.grid(row=1, column=0, padx=3)
        self.move_z_spin = ttk.Spinbox(
            mv, textvariable=self.move_z, from_=0.0, to=150.0, increment=1.0, width=9,
        )
        self.move_z_spin.grid(row=1, column=1, padx=3)
        self.move_t_spin = ttk.Spinbox(
            mv, textvariable=self.move_t, from_=-360.0, to=360.0, increment=1.0, width=9,
        )
        self.move_t_spin.grid(row=1, column=2, padx=3)
        # Live joint pose under each target (X/Z/θ — no Y).
        now_font = ("Consolas", 9)
        for col in range(3):
            ttk.Label(mv, text="now", foreground="#555", font=("Segoe UI", 8)).grid(
                row=2, column=col, sticky="w", padx=3, pady=(4, 0)
            )
        ttk.Label(mv, textvariable=self.state_vars["x"], foreground="#1565c0",
                  font=now_font).grid(row=3, column=0, sticky="w", padx=3)
        ttk.Label(mv, textvariable=self.state_vars["z"], foreground="#1565c0",
                  font=now_font).grid(row=3, column=1, sticky="w", padx=3)
        ttk.Label(mv, textvariable=self.state_vars["theta"], foreground="#1565c0",
                  font=now_font).grid(row=3, column=2, sticky="w", padx=3)
        self.btn_move = ttk.Button(mv, text="Move", command=self.cmd_move, width=12)
        self.btn_move.grid(row=4, column=0, columnspan=2, pady=(6, 0))
        self.gate_override = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            mv, text="skip calibrated gate", variable=self.gate_override,
            command=self._refresh_move_gate,
        ).grid(row=4, column=2, pady=(6, 0), sticky="w")

        grip = ttk.LabelFrame(tab, text="Gripper", padding=6)
        grip.grid(row=0, column=3, sticky="nsew", padx=4, pady=4)
        ttk.Button(grip, text="Open (0)", command=lambda: self.send("grip 0"), width=12).pack(pady=2)
        ttk.Button(grip, text="Close (1)", command=lambda: self.send("grip 1"), width=12).pack(pady=2)

        for col in range(4):
            tab.columnconfigure(col, weight=1)

        self._build_workspace_grid(tab)
        self.move_command_widgets = [
            self.btn_move,
            self.move_x_spin,
            self.move_z_spin,
            self.move_t_spin,
            self.btn_test_cycle,
            self.btn_theta_path,
            self.btn_move_selected,
            self.btn_follow_up,
            self.btn_follow_down,
        ]
        self._refresh_move_gate()

    def _build_workspace_grid(self, tab: ttk.Frame) -> None:
        ws = ttk.LabelFrame(
            tab,
            text="Workspace waypoints (X–Z, +Z down) — click empty to place, click to select, double-click to edit; right-click deletes",
            padding=6,
        )
        ws.grid(row=1, column=0, columnspan=4, sticky="nsew", padx=4, pady=(8, 4))
        tab.rowconfigure(1, weight=1)

        left = ttk.Frame(ws)
        left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.ws_status = tk.StringVar(value="Faded — run calibrate / bring-up to unlock envelope")
        ttk.Label(left, textvariable=self.ws_status, foreground="#666", font=("Segoe UI", 8)).pack(
            anchor="w"
        )
        self.ws_canvas = tk.Canvas(
            left,
            width=WORKSPACE_CANVAS_W,
            height=WORKSPACE_CANVAS_H,
            bg="#eceff1",
            highlightthickness=1,
            highlightbackground="#b0bec5",
            cursor="crosshair",
        )
        self.ws_canvas.pack(fill=tk.BOTH, expand=True, pady=(4, 0))
        self.ws_canvas.bind("<Button-1>", self._on_workspace_click)
        self.ws_canvas.bind("<Double-Button-1>", self._on_workspace_double_click)
        self.ws_canvas.bind("<Button-3>", self._on_workspace_right_click)
        self.ws_canvas.bind("<Motion>", self._on_workspace_motion)
        self.ws_canvas.bind("<Leave>", lambda _e: self._set_ws_hover(None))
        self.ws_canvas.bind("<Configure>", lambda _e: self._redraw_workspace())

        side = ttk.Frame(ws)
        side.pack(side=tk.LEFT, fill=tk.BOTH, padx=(10, 0))

        ttk.Label(side, text="Waypoints (Ctrl/Shift multi-select)").pack(anchor="w")
        self.wp_list = tk.Listbox(
            side, height=8, width=36, font=("Consolas", 8), selectmode=tk.EXTENDED,
            exportselection=False,
        )
        self.wp_list.pack(fill=tk.BOTH, expand=True, pady=2)
        self.wp_list.bind("<<ListboxSelect>>", self._on_waypoint_select)
        self.wp_list.bind("<Double-Button-1>", self._on_waypoint_list_double)

        reorder = ttk.Frame(side)
        reorder.pack(fill=tk.X, pady=2)
        ttk.Button(reorder, text="↑", width=4, command=lambda: self._reorder_waypoints(-1)).pack(
            side=tk.LEFT, padx=1
        )
        ttk.Button(reorder, text="↓", width=4, command=lambda: self._reorder_waypoints(1)).pack(
            side=tk.LEFT, padx=1
        )
        ttk.Button(reorder, text="Del", width=5, command=self._delete_selected_waypoints).pack(
            side=tk.LEFT, padx=1
        )

        self.btn_move_selected = ttk.Button(
            side, text="Move to selected", width=22, command=self.cmd_move_selected_wp
        )
        self.btn_move_selected.pack(pady=2)
        follow = ttk.Frame(side)
        follow.pack(fill=tk.X, pady=2)
        self.btn_follow_up = ttk.Button(
            follow, text="Follow ↑", width=10, command=lambda: self.cmd_follow_waypoints(True),
        )
        self.btn_follow_up.pack(side=tk.LEFT, padx=1)
        self.btn_follow_down = ttk.Button(
            follow, text="Follow ↓", width=10, command=lambda: self.cmd_follow_waypoints(False),
        )
        self.btn_follow_down.pack(side=tk.LEFT, padx=1)
        ttk.Button(side, text="Abort follow", width=22, command=self._abort_wp_follow).pack(pady=2)
        self.wp_trace = tk.BooleanVar(value=False)
        self.wp_trace_check = ttk.Checkbutton(
            side,
            text="Trace follow path",
            variable=self.wp_trace,
            command=self._on_trace_toggle,
        )
        self.wp_trace_check.pack(anchor="w", pady=(2, 0))
        io = ttk.Frame(side)
        io.pack(fill=tk.X, pady=2)
        self.btn_wp_save = ttk.Button(
            io, text="Save…", width=10, command=self.cmd_save_waypoints,
        )
        self.btn_wp_save.pack(side=tk.LEFT, padx=1)
        self.btn_wp_load = ttk.Button(
            io, text="Load…", width=10, command=self.cmd_load_waypoints,
        )
        self.btn_wp_load.pack(side=tk.LEFT, padx=1)
        ttk.Button(side, text="Clear waypoints", width=22, command=self._clear_waypoints).pack(pady=2)
        self.wp_follow_status = tk.StringVar(value="")
        ttk.Label(
            side, textvariable=self.wp_follow_status, foreground="#1565c0", font=("Segoe UI", 8),
        ).pack(anchor="w")
        ttk.Label(
            side,
            text="Double-click a point or row\n"
                 "to edit X/Z/θ/grip. Click empty\n"
                 "grid to place (θ from Move).\n"
                 "Trace paints the live path\n"
                 "during Follow ↑/↓. Save/Load\n"
                 "is a .txt (mm, θ, grip).",
            foreground="#555",
            font=("Segoe UI", 8),
        ).pack(anchor="w", pady=(4, 0))

        self._redraw_workspace()

    def _workspace_plot_box(self) -> tuple[float, float, float, float]:
        """Return canvas plot rectangle (x0, y0, x1, y1)."""
        pl, pr, pt, pb = WORKSPACE_PAD
        w = max(self.ws_canvas.winfo_width(), WORKSPACE_CANVAS_W)
        h = max(self.ws_canvas.winfo_height(), WORKSPACE_CANVAS_H)
        return (pl, pt, w - pr, h - pb)

    def _workspace_envelope_mm(self) -> tuple[float, float]:
        x = self.stroke_x_mm if self.grid_active and self.stroke_x_mm else PLACEHOLDER_X_MM
        z = self.stroke_z_mm if self.grid_active and self.stroke_z_mm else PLACEHOLDER_Z_MM
        return (max(x, 1.0), max(z, 1.0))

    def _linear_unit_scale(self) -> float:
        """mm → display units (1.0 for mm, 1/25.4 for inches)."""
        unit = self.state_vars.get("units", tk.StringVar(value="mm")).get()
        if str(unit).lower().startswith("in"):
            return 1.0 / 25.4
        return 1.0

    def _mm_to_canvas(self, x_mm: float, z_mm: float) -> tuple[float, float]:
        x0, y0, x1, y1 = self._workspace_plot_box()
        xmax, zmax = self._workspace_envelope_mm()
        px = x0 + (x_mm / xmax) * (x1 - x0)
        # +Z down: z=0 at top of plot.
        py = y0 + (z_mm / zmax) * (y1 - y0)
        return (px, py)

    def _canvas_to_mm(self, px: float, py: float) -> tuple[float, float] | None:
        x0, y0, x1, y1 = self._workspace_plot_box()
        if px < x0 or px > x1 or py < y0 or py > y1:
            return None
        xmax, zmax = self._workspace_envelope_mm()
        x_mm = (px - x0) / (x1 - x0) * xmax
        z_mm = (py - y0) / (y1 - y0) * zmax
        return (x_mm, z_mm)

    def _fade_workspace(self) -> None:
        self._abort_wp_follow(silent=True)
        self.grid_active = False
        self.stroke_x_mm = None
        self.stroke_z_mm = None
        self.safe_z_ceiling_mm = None
        self.waypoints.clear()
        self._refresh_waypoint_list()
        if hasattr(self, "ws_status"):
            self.ws_status.set("Faded — run calibrate / bring-up to unlock envelope")
        self._redraw_workspace()

    def _try_activate_workspace(self) -> None:
        if self.stroke_x_mm is None or self.stroke_z_mm is None:
            return
        was = self.grid_active
        self.grid_active = True
        scale = self._linear_unit_scale()
        unit = "in" if scale != 1.0 else "mm"
        self.ws_status.set(
            f"Active — envelope X=0..{self.stroke_x_mm * scale:g} {unit}, "
            f"Z=0..{self.stroke_z_mm * scale:g} {unit} (+Z down); "
            f"SAFE_Z 0…{(self.safe_z_ceiling_mm or PLACEHOLDER_SAFE_Z_MM) * scale:g} {unit}"
        )
        if not was:
            self.waypoints.clear()
            self._refresh_waypoint_list()
        self._redraw_workspace()
        # Match Move spinbox clamps to measured strokes.
        if hasattr(self, "move_x_spin"):
            self.move_x_spin.configure(to=self.stroke_x_mm * scale)
            self.move_z_spin.configure(to=self.stroke_z_mm * scale)

    def _ingest_stroke_x(self, mm: float) -> None:
        if mm <= 0:
            return
        self.stroke_x_mm = mm
        self._try_activate_workspace()

    def _ingest_stroke_z(self, mm: float) -> None:
        if mm <= 0:
            return
        self.stroke_z_mm = mm
        self._try_activate_workspace()

    def _ingest_safe_z(self, mm: float) -> None:
        if mm <= 0:
            return
        self.safe_z_ceiling_mm = mm
        self._redraw_workspace()
        if self.grid_active:
            self._try_activate_workspace()

    def _parse_workspace_envelope(self, body: str) -> None:
        bring = re.search(
            r"OK Bring-up complete:.*X stroke=([\d.]+)\s*mm.*Z stroke=([\d.]+)\s*mm",
            body,
        )
        if bring:
            self._ingest_stroke_x(float(bring.group(1)))
            self._ingest_stroke_z(float(bring.group(2)))
        safe = re.search(r"SAFE_Z ceiling=([\d.]+)", body)
        if safe:
            self._ingest_safe_z(float(safe.group(1)))
        if bring:
            return
        soft = re.search(
            r"OK X soft-calibrate.*Joint envelope X=([\d.]+)\.\.([\d.]+)\s*mm",
            body,
        )
        if soft:
            self._ingest_stroke_x(float(soft.group(2)) - float(soft.group(1)))
        xcal = re.search(r"OK Calibrated length:\s*([\d.]+)\s*mm", body)
        if xcal and "Z Calibrated" not in body:
            self._ingest_stroke_x(float(xcal.group(1)))
        zcal = re.search(r"OK Z Calibrated length:\s*([\d.]+)\s*mm", body)
        if zcal:
            self._ingest_stroke_z(float(zcal.group(1)))

    def _set_live_xz_from_text(self, x_raw: str, z_raw: str) -> None:
        try:
            x = float(x_raw)
            z = float(z_raw)
        except ValueError:
            return
        # LIVE POS / status already in current linear units; convert to mm for plot.
        scale = self._linear_unit_scale()
        self.live_xz = (x / scale, z / scale)
        self._wp_trace_maybe_append()
        self._redraw_workspace()

    def _redraw_workspace(self) -> None:
        if not hasattr(self, "ws_canvas"):
            return
        c = self.ws_canvas
        c.delete("all")
        active = self.grid_active
        bg = "#fafafa" if active else "#eceff1"
        c.configure(bg=bg)
        x0, y0, x1, y1 = self._workspace_plot_box()
        xmax, zmax = self._workspace_envelope_mm()
        scale = self._linear_unit_scale()
        unit = "in" if scale != 1.0 else "mm"

        line = "#78909c" if active else "#cfd8dc"
        axis = "#37474f" if active else "#90a4ae"
        # Border
        c.create_rectangle(x0, y0, x1, y1, outline=axis, width=2 if active else 1)

        # SAFE_Z band: z=0 (A015 retract) .. ceiling. +Z down so band is the top strip.
        ceil = self.safe_z_ceiling_mm if self.safe_z_ceiling_mm else PLACEHOLDER_SAFE_Z_MM
        ceil = min(max(ceil, 0.0), zmax)
        _, py_ceil = self._mm_to_canvas(0.0, ceil)
        band_fill = "#c8e6c9" if active else "#dcedc8"
        c.create_rectangle(x0 + 1, y0 + 1, x1 - 1, py_ceil, fill=band_fill, outline="")
        c.create_line(x0, py_ceil, x1, py_ceil, fill="#2e7d32", width=2, dash=(6, 3))
        c.create_text(
            x1 - 4,
            (y0 + py_ceil) / 2,
            text=f"SAFE_Z  0…{ceil * scale:g} {unit}",
            anchor="e",
            fill="#1b5e20",
            font=("Segoe UI", 8, "bold"),
        )

        # Grid: ~8 divisions on the longer axis.
        step_mm = 50.0 if max(xmax, zmax) >= 200 else 25.0
        if not active:
            step_mm = 100.0
        gx = 0.0
        while gx <= xmax + 0.01:
            px, _ = self._mm_to_canvas(gx, 0.0)
            c.create_line(px, y0, px, y1, fill=line, dash=() if active else (2, 4))
            if gx > 0:
                c.create_text(
                    px, y1 + 10, text=f"{gx * scale:g}", fill=axis, font=("Segoe UI", 7)
                )
            gx += step_mm
        gz = 0.0
        while gz <= zmax + 0.01:
            _, py = self._mm_to_canvas(0.0, gz)
            c.create_line(x0, py, x1, py, fill=line, dash=() if active else (2, 4))
            if gz > 0:
                c.create_text(
                    x0 - 14, py, text=f"{gz * scale:g}", fill=axis, font=("Segoe UI", 7)
                )
            gz += step_mm

        c.create_text(x0, y1 + 22, text=f"X ({unit}) →", anchor="w", fill=axis, font=("Segoe UI", 8))
        c.create_text(x0 - 28, y0, text=f"+Z\n↓", anchor="n", fill=axis, font=("Segoe UI", 8),
                      justify=tk.CENTER)

        if not active:
            c.create_rectangle(x0, y0, x1, y1, fill="#eceff1", stipple="gray50", outline="")
            c.create_text(
                (x0 + x1) / 2,
                (y0 + y1) / 2,
                text="Calibrate to reconstruct grid",
                fill="#78909c",
                font=("Segoe UI", 11, "bold"),
            )

        # Planned polyline (list order) then live follow trace.
        if active and len(self.waypoints) >= 2:
            pts: list[float] = []
            for wp in self.waypoints:
                px, py = self._mm_to_canvas(wp.x_mm, wp.z_mm)
                pts.extend([px, py])
            c.create_line(*pts, fill="#ffcc80", width=2, smooth=False)
        if self.wp_trace.get() and len(self.wp_trace_pts) >= 2:
            tpts: list[float] = []
            for tx, tz in self.wp_trace_pts:
                px, py = self._mm_to_canvas(tx, tz)
                tpts.extend([px, py])
            c.create_line(*tpts, fill="#00838f", width=2, smooth=False)
            lx, lz = self.wp_trace_pts[-1]
            px, py = self._mm_to_canvas(lx, lz)
            c.create_text(
                px + 8, py + 10, text="trace", anchor="w", fill="#006064",
                font=("Segoe UI", 7, "italic"),
            )

        selected = set(self.wp_list.curselection()) if hasattr(self, "wp_list") else set()
        tgt = self.wp_follow_target
        for i, wp in enumerate(self.waypoints):
            px, py = self._mm_to_canvas(wp.x_mm, wp.z_mm)
            is_sel = i in selected
            is_follow = (
                tgt is not None
                and abs(wp.x_mm - tgt.x_mm) < 1e-6
                and abs(wp.z_mm - tgt.z_mm) < 1e-6
            )
            if is_follow:
                color, outline, r = "#2e7d32", "#1b5e20", 7
            elif is_sel:
                color, outline, r = "#ef6c00", "#e65100", 6
            elif active:
                color, outline, r = "#e65100", "#bf360c", 5
            else:
                color, outline, r = "#b0bec5", "#90a4ae", 5
            c.create_oval(px - r, py - r, px + r, py + r, fill=color, outline=outline, width=2)
            gmark = "▾" if wp.grip else "○"
            c.create_text(
                px + 8, py - 8,
                text=f"{i + 1} {wp.theta_deg:g}°{gmark}",
                anchor="w", fill=color, font=("Segoe UI", 8, "bold"),
            )

        # Live pose
        if self.live_xz is not None:
            lx, lz = self.live_xz
            if 0 <= lx <= xmax * 1.05 and 0 <= lz <= zmax * 1.05:
                px, py = self._mm_to_canvas(lx, lz)
                live_c = "#1565c0" if active else "#90caf9"
                c.create_line(px - 8, py, px + 8, py, fill=live_c, width=2)
                c.create_line(px, py - 8, px, py + 8, fill=live_c, width=2)
                c.create_oval(px - 3, py - 3, px + 3, py + 3, fill=live_c, outline="")

        if self._ws_hover is not None and active:
            hx, hz = self._ws_hover
            extra = ""
            if self.safe_z_ceiling_mm and hz <= self.safe_z_ceiling_mm + 0.05:
                extra = "  in SAFE_Z"
            elif self.safe_z_ceiling_mm:
                extra = "  below SAFE_Z"
            c.create_text(
                x1 - 4,
                y0 + 4,
                text=f"{hx * scale:.1f}, {hz * scale:.1f} {unit}{extra}",
                anchor="ne",
                fill="#455a64",
                font=("Consolas", 9),
            )

    def _set_ws_hover(self, xz_mm: tuple[float, float] | None) -> None:
        self._ws_hover = xz_mm
        self._redraw_workspace()

    def _on_workspace_motion(self, event: tk.Event) -> None:
        if not self.grid_active:
            return
        hit = self._canvas_to_mm(event.x, event.y)
        self._set_ws_hover(hit)

    def _nearest_waypoint_index(self, hx: float, hz: float) -> int | None:
        if not self.waypoints:
            return None
        best_i = 0
        best_d = float("inf")
        for i, wp in enumerate(self.waypoints):
            d = (wp.x_mm - hx) ** 2 + (wp.z_mm - hz) ** 2
            if d < best_d:
                best_d = d
                best_i = i
        xmax, zmax = self._workspace_envelope_mm()
        thresh = (xmax ** 2 + zmax ** 2) * 0.0025
        if best_d <= thresh:
            return best_i
        return None

    def _select_waypoint_indices(self, indices: list[int]) -> None:
        if not hasattr(self, "wp_list"):
            return
        self.wp_list.selection_clear(0, tk.END)
        for i in indices:
            if 0 <= i < len(self.waypoints):
                self.wp_list.selection_set(i)
        if indices:
            self.wp_list.see(indices[-1])
        self._on_waypoint_select()

    def _on_workspace_click(self, event: tk.Event) -> None:
        if not self.grid_active:
            self._log("sys", "Workspace grid is faded until X+Z calibrate completes.")
            return
        hit = self._canvas_to_mm(event.x, event.y)
        if hit is None:
            return
        x_mm, z_mm = hit
        ctrl = bool(int(getattr(event, "state", 0)) & 0x0004)
        if not ctrl:
            near = self._nearest_waypoint_index(x_mm, z_mm)
            if near is not None:
                self._select_waypoint_indices([near])
                return
        self._place_waypoint(x_mm, z_mm)

    def _on_workspace_double_click(self, event: tk.Event) -> None:
        if not self.grid_active or not self.waypoints:
            return
        hit = self._canvas_to_mm(event.x, event.y)
        if hit is None:
            return
        near = self._nearest_waypoint_index(hit[0], hit[1])
        if near is not None:
            self._select_waypoint_indices([near])
            self._edit_waypoint_dialog(near)

    def _on_waypoint_list_double(self, event: tk.Event) -> None:
        idx = int(self.wp_list.nearest(event.y))
        if 0 <= idx < len(self.waypoints):
            self._select_waypoint_indices([idx])
            self._edit_waypoint_dialog(idx)

    def _place_default_theta(self) -> float:
        try:
            return float(self.move_t.get())
        except (ValueError, tk.TclError, AttributeError):
            return 0.0

    def _place_waypoint(self, x_mm: float, z_mm: float) -> None:
        theta = self._place_default_theta()
        self.waypoints.append(Waypoint(x_mm, z_mm, theta, 0))
        scale = self._linear_unit_scale()
        self.move_x.set(f"{x_mm * scale:.2f}")
        self.move_z.set(f"{z_mm * scale:.2f}")
        self.move_t.set(f"{theta:g}")
        self._refresh_waypoint_list(keep_selection=[len(self.waypoints) - 1])
        self.wp_list.see(tk.END)

    def _on_workspace_right_click(self, event: tk.Event) -> None:
        if not self.waypoints:
            return
        hit = self._canvas_to_mm(event.x, event.y)
        if hit is None:
            return
        near = self._nearest_waypoint_index(hit[0], hit[1])
        if near is not None:
            del self.waypoints[near]
            self._refresh_waypoint_list()
            self._redraw_workspace()

    def _refresh_waypoint_list(self, keep_selection: list[int] | None = None) -> None:
        if not hasattr(self, "wp_list"):
            return
        prev = list(self.wp_list.curselection()) if keep_selection is None else keep_selection
        self._wp_suppress_select = True
        try:
            self.wp_list.delete(0, tk.END)
            scale = self._linear_unit_scale()
            for i, wp in enumerate(self.waypoints):
                self.wp_list.insert(
                    tk.END,
                    f"{i + 1}: X={wp.x_mm * scale:.1f} Z={wp.z_mm * scale:.1f} "
                    f"T={wp.theta_deg:g} {wp.grip_label()}",
                )
            for idx in prev:
                if 0 <= idx < len(self.waypoints):
                    self.wp_list.selection_set(idx)
            self._wp_sel_cache = [i for i in prev if 0 <= i < len(self.waypoints)]
        finally:
            self._wp_suppress_select = False
        self._redraw_workspace()

    def _selected_waypoint_indices(self) -> list[int]:
        if not hasattr(self, "wp_list"):
            return []
        live = [int(i) for i in self.wp_list.curselection()]
        if live:
            self._wp_sel_cache = live
            return live
        return [i for i in self._wp_sel_cache if 0 <= i < len(self.waypoints)]

    def _on_waypoint_select(self, _event: object = None) -> None:
        if self._wp_suppress_select:
            return
        sel = self._selected_waypoint_indices()
        if not sel:
            self._redraw_workspace()
            return
        wp = self.waypoints[sel[-1]]
        scale = self._linear_unit_scale()
        self.move_x.set(f"{wp.x_mm * scale:.2f}")
        self.move_z.set(f"{wp.z_mm * scale:.2f}")
        self.move_t.set(f"{wp.theta_deg:g}")
        self._redraw_workspace()

    def _commit_waypoint(
        self,
        index: int,
        x_mm: float,
        z_mm: float,
        theta_deg: float,
        grip: int,
    ) -> None:
        if not (0 <= index < len(self.waypoints)):
            return
        wp = self.waypoints[index]
        wp.x_mm = x_mm
        wp.z_mm = z_mm
        wp.theta_deg = theta_deg
        wp.grip = 1 if grip else 0
        scale = self._linear_unit_scale()
        self.move_x.set(f"{x_mm * scale:.2f}")
        self.move_z.set(f"{z_mm * scale:.2f}")
        self.move_t.set(f"{theta_deg:g}")
        self._refresh_waypoint_list(keep_selection=[index])

    def _edit_waypoint_dialog(self, index: int) -> None:
        if not (0 <= index < len(self.waypoints)):
            return
        wp = self.waypoints[index]
        scale = self._linear_unit_scale()
        unit = "in" if scale != 1.0 else "mm"
        dlg = tk.Toplevel(self.root)
        dlg.title(f"Edit waypoint {index + 1}")
        dlg.transient(self.root)
        dlg.resizable(False, False)
        frm = ttk.Frame(dlg, padding=10)
        frm.pack(fill=tk.BOTH, expand=True)
        x_var = tk.StringVar(value=f"{wp.x_mm * scale:.2f}")
        z_var = tk.StringVar(value=f"{wp.z_mm * scale:.2f}")
        t_var = tk.StringVar(value=f"{wp.theta_deg:g}")
        g_var = tk.IntVar(value=1 if wp.grip else 0)
        ttk.Label(frm, text=f"X ({unit})").grid(row=0, column=0, sticky="e", pady=2)
        ttk.Entry(frm, textvariable=x_var, width=12).grid(row=0, column=1, padx=6, pady=2)
        ttk.Label(frm, text=f"Z ({unit})").grid(row=1, column=0, sticky="e", pady=2)
        ttk.Entry(frm, textvariable=z_var, width=12).grid(row=1, column=1, padx=6, pady=2)
        ttk.Label(frm, text="θ (deg)").grid(row=2, column=0, sticky="e", pady=2)
        ttk.Entry(frm, textvariable=t_var, width=12).grid(row=2, column=1, padx=6, pady=2)
        ttk.Label(frm, text="grip").grid(row=3, column=0, sticky="e", pady=2)
        grip_row = ttk.Frame(frm)
        grip_row.grid(row=3, column=1, sticky="w", padx=6, pady=2)
        ttk.Radiobutton(grip_row, text="open", variable=g_var, value=0).pack(side=tk.LEFT)
        ttk.Radiobutton(grip_row, text="close", variable=g_var, value=1).pack(side=tk.LEFT, padx=(8, 0))

        def apply_and_close(_event: object = None) -> None:
            try:
                x_disp = float(x_var.get())
                z_disp = float(z_var.get())
                theta = float(t_var.get())
            except ValueError:
                messagebox.showerror("Waypoint", "X, Z, and θ must be numbers.", parent=dlg)
                return
            self._commit_waypoint(index, x_disp / scale, z_disp / scale, theta, int(g_var.get()))
            dlg.destroy()

        btns = ttk.Frame(frm)
        btns.grid(row=4, column=0, columnspan=2, pady=(10, 0))
        ttk.Button(btns, text="OK", width=10, command=apply_and_close).pack(side=tk.LEFT, padx=4)
        ttk.Button(btns, text="Cancel", width=10, command=dlg.destroy).pack(side=tk.LEFT, padx=4)
        dlg.bind("<Return>", apply_and_close)
        dlg.bind("<Escape>", lambda _e: dlg.destroy())
        try:
            if self.root.winfo_viewable():
                dlg.grab_set()
        except tk.TclError:
            pass
        dlg.focus_set()

    def _on_trace_toggle(self) -> None:
        self._redraw_workspace()

    def _wp_trace_reset(self) -> None:
        self.wp_trace_pts = []
        if self.live_xz is not None:
            self.wp_trace_pts.append(self.live_xz)

    def _wp_trace_maybe_append(self) -> None:
        if not hasattr(self, "wp_trace") or not self.wp_trace.get():
            return
        if self.wp_follow_phase == "idle":
            return
        if self.live_xz is None:
            return
        if self.wp_trace_pts:
            lx, lz = self.wp_trace_pts[-1]
            dx = self.live_xz[0] - lx
            dz = self.live_xz[1] - lz
            if dx * dx + dz * dz < 0.25:
                return
        self.wp_trace_pts.append(self.live_xz)
        if len(self.wp_trace_pts) > 4000:
            del self.wp_trace_pts[: len(self.wp_trace_pts) - 4000]

    def _reorder_waypoints(self, delta: int) -> None:
        sel = self._selected_waypoint_indices()
        if not sel:
            return
        # Move the contiguous? For multi-select, shift each block as a group in list order.
        indices = sorted(sel)
        if delta < 0:
            if indices[0] == 0:
                return
            for i in indices:
                self.waypoints[i - 1], self.waypoints[i] = self.waypoints[i], self.waypoints[i - 1]
            new_sel = [i - 1 for i in indices]
        else:
            if indices[-1] >= len(self.waypoints) - 1:
                return
            for i in reversed(indices):
                self.waypoints[i + 1], self.waypoints[i] = self.waypoints[i], self.waypoints[i + 1]
            new_sel = [i + 1 for i in indices]
        self._refresh_waypoint_list(keep_selection=new_sel)

    def _delete_selected_waypoints(self) -> None:
        sel = self._selected_waypoint_indices()
        if not sel:
            return
        for i in sorted(sel, reverse=True):
            del self.waypoints[i]
        self._refresh_waypoint_list(keep_selection=[])

    def _clear_waypoints(self) -> None:
        self._abort_wp_follow(silent=True)
        self.waypoints.clear()
        self.wp_trace_pts = []
        self._refresh_waypoint_list(keep_selection=[])

    def _apply_loaded_waypoints(self, points: list[Waypoint]) -> None:
        self._abort_wp_follow(silent=True)
        self.waypoints = [replace(wp) for wp in points]
        self.wp_trace_pts = []
        self._refresh_waypoint_list(keep_selection=[0] if self.waypoints else [])

    def cmd_save_waypoints(self) -> None:
        if not self.waypoints:
            messagebox.showinfo("Waypoints", "No waypoints to save.")
            return
        path = filedialog.asksaveasfilename(
            title="Save waypoints",
            initialdir=self._wp_file_dir,
            initialfile="gantry_waypoints.txt",
            defaultextension=".txt",
            filetypes=[("Waypoint text", "*.txt"), ("All files", "*.*")],
        )
        if not path:
            return
        try:
            Path(path).write_text(format_waypoints_txt(self.waypoints), encoding="utf-8")
        except OSError as exc:
            messagebox.showerror("Save waypoints", str(exc))
            return
        self._wp_file_dir = str(Path(path).parent)
        self._log("sys", f"saved {len(self.waypoints)} waypoint(s) to {path}")

    def cmd_load_waypoints(self) -> None:
        if self.waypoints and not self._confirm(
            "Load waypoints",
            f"Replace {len(self.waypoints)} current waypoint(s)?"
            + (" This aborts Follow." if self.wp_follow_phase != "idle" else ""),
        ):
            return
        path = filedialog.askopenfilename(
            title="Load waypoints",
            initialdir=self._wp_file_dir,
            filetypes=[("Waypoint text", "*.txt"), ("All files", "*.*")],
        )
        if not path:
            return
        try:
            text = Path(path).read_text(encoding="utf-8")
            points = parse_waypoints_txt(text)
        except (OSError, ValueError) as exc:
            messagebox.showerror("Load waypoints", str(exc))
            return
        if not points:
            messagebox.showerror("Load waypoints", "No waypoint rows in that file.")
            return
        self._wp_file_dir = str(Path(path).parent)
        self._apply_loaded_waypoints(points)
        self._log("sys", f"loaded {len(points)} waypoint(s) from {path}")

    def cmd_move_selected_wp(self) -> None:
        sel = self._selected_waypoint_indices()
        if not sel:
            if self.waypoints:
                self.wp_list.selection_set(0)
                sel = [0]
            else:
                messagebox.showinfo("Waypoints", "Place a waypoint on the grid first.")
                return
        if len(sel) > 1:
            messagebox.showinfo(
                "Waypoints",
                "Multiple selected — use Follow ↑ / Follow ↓ for a sequence, "
                "or select one waypoint for a single move.",
            )
            return
        wp = self.waypoints[sel[0]]
        scale = self._linear_unit_scale()
        self.move_x.set(f"{wp.x_mm * scale:.2f}")
        self.move_z.set(f"{wp.z_mm * scale:.2f}")
        self.move_t.set(f"{wp.theta_deg:g}")
        self.cmd_move()
        if wp.grip:
            self.send("grip 1")
        else:
            self.send("grip 0")

    def _follow_indices(self, ascending: bool) -> list[int]:
        """Full list order. Selection is for reorder/delete, not follow subset."""
        indices = list(range(len(self.waypoints)))
        if not ascending:
            indices.reverse()
        return indices

    def cmd_follow_waypoints(self, ascending: bool) -> None:
        if self.wp_follow_phase != "idle":
            messagebox.showinfo("Follow", "A follow is already running. Abort it first.")
            return
        if not self.grid_active:
            messagebox.showinfo("Follow", "Calibrate first — workspace grid is faded.")
            return
        if not self.client.connected:
            messagebox.showinfo("Follow", "Not connected.")
            return
        if not self._motion_unlocked():
            messagebox.showinfo(
                "Follow",
                "Workspace not calibrated — run bring-up / calibrate all first.",
            )
            return
        indices = self._follow_indices(ascending)
        if not indices:
            messagebox.showinfo("Follow", "No waypoints to follow.")
            return
        order = "ascending (1→N)" if ascending else "descending (N→1)"
        preview = ", ".join(str(i + 1) for i in indices)
        if not self._confirm(
            "Follow waypoints",
            f"Follow all {len(indices)} waypoint(s) in {order}?\n\n"
            f"Order: {preview}\n"
            "Each point uses its own theta and gripper (open/close on arrival).\n"
            "STOP or Abort follow cancels the remaining legs.",
        ):
            return
        self.wp_follow_gen += 1
        self.wp_follow_queue = [replace(self.waypoints[i]) for i in indices]
        self.wp_follow_i = -1
        self.wp_follow_phase = "idle"
        self.wp_follow_saw_busy = False
        self.wp_follow_last_grip = None
        if self.wp_trace.get():
            self._wp_trace_reset()
        self.wp_follow_status.set(f"Follow {order}: starting…")
        self._wp_follow_advance(self.wp_follow_gen)

    def _abort_wp_follow(self, silent: bool = False) -> None:
        was = self.wp_follow_phase != "idle" or bool(self.wp_follow_queue)
        self.wp_follow_gen += 1
        self.wp_follow_queue = []
        self.wp_follow_i = -1
        self.wp_follow_phase = "idle"
        self.wp_follow_target = None
        self.wp_follow_deadline_ms = 0
        self.wp_follow_saw_busy = False
        self.wp_follow_last_grip = None
        if hasattr(self, "wp_follow_status"):
            self.wp_follow_status.set("Follow aborted" if was and not silent else "")
        if was and not silent:
            self._log("sys", "Waypoint follow aborted")
        self._redraw_workspace()

    def _schedule_wp_follow_advance(self) -> None:
        gen = self.wp_follow_gen
        self.root.after(150, lambda: self._wp_follow_advance(gen))

    def _wp_follow_advance(self, gen: int | None = None) -> None:
        if gen is not None and gen != self.wp_follow_gen:
            return
        if self.wp_follow_phase not in ("idle", "advancing"):
            return
        self.wp_follow_i += 1
        if self.wp_follow_i >= len(self.wp_follow_queue):
            n = len(self.wp_follow_queue)
            self.wp_follow_queue = []
            self.wp_follow_phase = "idle"
            self.wp_follow_target = None
            self.wp_follow_saw_busy = False
            self.wp_follow_status.set(f"Follow complete ({n} legs)")
            self._log("sys", f"Waypoint follow complete ({n} legs)")
            self._redraw_workspace()
            return
        wp = self.wp_follow_queue[self.wp_follow_i]
        self.wp_follow_target = wp
        self.wp_follow_saw_busy = False
        scale = self._linear_unit_scale()
        self.move_x.set(f"{wp.x_mm * scale:.2f}")
        self.move_z.set(f"{wp.z_mm * scale:.2f}")
        self.move_t.set(f"{wp.theta_deg:g}")
        if self._wp_follow_near_target(eps_mm=1.5):
            self.wp_follow_status.set(
                f"Follow leg {self.wp_follow_i + 1}/{len(self.wp_follow_queue)} "
                f"already there — grip then next"
            )
            self._wp_follow_after_arrival()
            return
        cmd = f"move {wp.x_mm * scale:g} {wp.z_mm * scale:g} {wp.theta_deg:g}"
        self.wp_follow_phase = "sent"
        self.wp_follow_deadline_ms = int(_dt.datetime.now().timestamp() * 1000) + 120_000
        self.wp_follow_status.set(
            f"Follow leg {self.wp_follow_i + 1}/{len(self.wp_follow_queue)} → "
            f"X={wp.x_mm * scale:.1f} Z={wp.z_mm * scale:.1f} T={wp.theta_deg:g} "
            f"{wp.grip_label()}"
        )
        self.send(cmd)
        self._redraw_workspace()

    def _wp_follow_after_arrival(self) -> None:
        wp = self.wp_follow_target
        if wp is None:
            self.wp_follow_phase = "advancing"
            self._schedule_wp_follow_advance()
            return
        if self.wp_follow_last_grip != wp.grip:
            self.send(f"grip {wp.grip}")
            self.wp_follow_last_grip = wp.grip
            self.wp_follow_phase = "gripping"
            gen = self.wp_follow_gen
            self.root.after(250, lambda: self._wp_follow_after_grip(gen))
            return
        self.wp_follow_phase = "advancing"
        self._schedule_wp_follow_advance()

    def _wp_follow_after_grip(self, gen: int) -> None:
        if gen != self.wp_follow_gen:
            return
        self.wp_follow_phase = "advancing"
        self._wp_follow_advance(gen)

    def _wp_follow_on_busy(self, busy: bool) -> None:
        if self.wp_follow_phase == "sent" and busy:
            self.wp_follow_saw_busy = True
            self.wp_follow_phase = "busy"
        elif self.wp_follow_phase == "busy" and not busy and self.wp_follow_saw_busy:
            self._wp_follow_after_arrival()

    def _wp_follow_near_target(self, eps_mm: float = 2.0) -> bool:
        if self.live_xz is None or self.wp_follow_target is None:
            return False
        dx = self.live_xz[0] - self.wp_follow_target.x_mm
        dz = self.live_xz[1] - self.wp_follow_target.z_mm
        return (dx * dx + dz * dz) ** 0.5 <= eps_mm

    def _wp_follow_tick(self) -> None:
        if self.wp_follow_phase in ("idle", "advancing", "gripping"):
            return
        now = int(_dt.datetime.now().timestamp() * 1000)
        if self.wp_follow_deadline_ms and now > self.wp_follow_deadline_ms:
            self._log("sys", "Waypoint follow timed out on a leg")
            self._abort_wp_follow()

    def _tab_profile(self) -> None:
        tab = ttk.Frame(self.tabs, padding=8)
        self.tabs.add(tab, text="Profile & Units")

        un = ttk.LabelFrame(tab, text="Units", padding=6)
        un.grid(row=0, column=0, sticky="nsew", padx=4, pady=4)
        self.units_var = tk.StringVar(value="mm")
        ttk.Radiobutton(un, text="mm", variable=self.units_var, value="mm").pack(anchor="w")
        ttk.Radiobutton(un, text="in", variable=self.units_var, value="in").pack(anchor="w")
        ttk.Button(un, text="Apply", width=12, command=self.cmd_units).pack(pady=4)

        prof = ttk.LabelFrame(tab, text="Speed / accel (selected units)", padding=6)
        prof.grid(row=0, column=1, sticky="nsew", padx=4, pady=4)
        self.speed_lin = tk.StringVar(value="50")
        self.speed_deg = tk.StringVar(value="30")
        self.accel_var = tk.StringVar(value="3000")
        self.decel_var = tk.StringVar(value="")
        self.accel_theta = tk.StringVar(value="")
        self.decel_theta = tk.StringVar(value="")
        # Path clamps with rangelimit 1: speed 1–500 mm/s, accel 100–3000 mm/s².
        ttk.Label(prof, text="speed").grid(row=0, column=0, sticky="e")
        ttk.Spinbox(prof, textvariable=self.speed_lin, from_=1, to=500, width=8).grid(row=0, column=1)
        ttk.Label(prof, text="deg/s (opt)").grid(row=0, column=2, sticky="e")
        ttk.Spinbox(prof, textvariable=self.speed_deg, from_=0, to=3600, width=8).grid(row=0, column=3)
        ttk.Button(prof, text="Set speed", command=self.cmd_speed, width=12).grid(
            row=0, column=4, padx=4
        )
        ttk.Label(prof, text="accel").grid(row=1, column=0, sticky="e")
        ttk.Spinbox(prof, textvariable=self.accel_var, from_=1, to=3000, width=8).grid(row=1, column=1)
        ttk.Label(prof, text="decel (opt)").grid(row=1, column=2, sticky="e")
        ttk.Spinbox(prof, textvariable=self.decel_var, from_=0, to=3000, width=8).grid(row=1, column=3)
        ttk.Button(prof, text="Set accel", command=self.cmd_accel, width=12).grid(
            row=1, column=4, padx=4
        )
        ttk.Label(prof, text="θ accel").grid(row=2, column=0, sticky="e")
        ttk.Spinbox(prof, textvariable=self.accel_theta, from_=0, to=18000, width=8).grid(row=2, column=1)
        ttk.Label(prof, text="θ decel").grid(row=2, column=2, sticky="e")
        ttk.Spinbox(prof, textvariable=self.decel_theta, from_=0, to=18000, width=8).grid(row=2, column=3)
        ttk.Button(prof, text="Query (status)", command=lambda: self.send("status")).grid(
            row=2, column=4, padx=4
        )
        ttk.Label(
            prof,
            text="Path / test_cycle legs only. Home/cal seek locked at 100 mm/s, 2000 mm/s².\n"
                 "With rangelimit 1: speed ≤500 mm/s, accel/decel ≤3000 mm/s².\n"
                 "accel cmd: accel <a> [d] [theta_a] [theta_d]",
            foreground="#555", font=("Segoe UI", 8),
        ).grid(row=3, column=0, columnspan=5, sticky="w", pady=(4, 0))

        rl = ttk.LabelFrame(tab, text="Range clamps", padding=6)
        rl.grid(row=0, column=2, sticky="nsew", padx=4, pady=4)
        ttk.Button(rl, text="Enable (1)", width=12,
                   command=lambda: self.send("rangelimit 1")).pack(pady=2)
        ttk.Button(rl, text="Disable (0)", width=12,
                   command=lambda: self.send("rangelimit 0")).pack(pady=2)

        rates = ttk.LabelFrame(tab, text="Periodic logging", padding=6)
        rates.grid(row=0, column=3, sticky="nsew", padx=4, pady=4)
        self.livepos_hz = tk.StringVar(value="0")
        self.axislog_hz = tk.StringVar(value="0")
        for row, (label, var, cmd) in enumerate(
            (("livepos", self.livepos_hz, "livepos"), ("axislog", self.axislog_hz, "axislog"))
        ):
            ttk.Label(rates, text=f"{label} Hz").grid(row=row, column=0, sticky="e")
            ttk.Spinbox(rates, textvariable=var, from_=0, to=100, width=6).grid(row=row, column=1)
            ttk.Button(
                rates, text="Set", width=6,
                command=lambda c=cmd, v=var: self._send_rate(c, v),
            ).grid(row=row, column=2, padx=2)

        for col in range(4):
            tab.columnconfigure(col, weight=1)

    def _tab_diagnostics(self) -> None:
        tab = ttk.Frame(self.tabs, padding=8)
        self.tabs.add(tab, text="Diagnostics")

        rd = ttk.LabelFrame(tab, text="Read-only queries", padding=6)
        rd.grid(row=0, column=0, columnspan=2, sticky="nsew", padx=4, pady=4)
        for col, cmd in enumerate(
            ["status", "faults", "puuinfo", "eiptiming", "ota", "limits", "pins", "selftest", "help"]
        ):
            ttk.Button(rd, text=cmd, width=11, command=lambda c=cmd: self.send(c)).grid(
                row=col // 5, column=col % 5, padx=3, pady=3
            )

        th = ttk.LabelFrame(tab, text="Theta helpers", padding=6)
        th.grid(row=1, column=0, sticky="nsew", padx=4, pady=4)
        self.puu_t_scale = tk.StringVar(value="10000")
        self.thetalim_min = tk.StringVar(value="-180")
        self.thetalim_max = tk.StringVar(value="180")
        ttk.Label(th, text="puu t").grid(row=0, column=0, sticky="e")
        ttk.Entry(th, textvariable=self.puu_t_scale, width=10).grid(row=0, column=1, padx=3)
        ttk.Button(th, text="Set PUU/deg", width=12, command=self.cmd_puu_t).grid(
            row=0, column=2, padx=3
        )
        ttk.Label(th, text="thetalim").grid(row=1, column=0, sticky="e")
        ttk.Entry(th, textvariable=self.thetalim_min, width=8).grid(row=1, column=1, padx=3, sticky="w")
        ttk.Entry(th, textvariable=self.thetalim_max, width=8).grid(row=1, column=2, padx=3, sticky="w")
        ttk.Button(th, text="Set limits", width=12, command=self.cmd_thetalim).grid(
            row=1, column=3, padx=3
        )

        cal = ttk.LabelFrame(tab, text="PUU calibration", padding=6)
        cal.grid(row=1, column=1, sticky="nsew", padx=4, pady=4)
        self.puucal_axis = tk.StringVar(value="x")
        self.puucal_cmd = tk.StringVar(value="")
        self.puucal_meas = tk.StringVar(value="")
        ttk.Label(cal, text="axis").grid(row=0, column=0, sticky="e")
        ttk.Combobox(cal, textvariable=self.puucal_axis, values=["x", "z", "t"], width=4,
                     state="readonly").grid(row=0, column=1, padx=3)
        ttk.Label(cal, text="commanded").grid(row=0, column=2, sticky="e")
        ttk.Entry(cal, textvariable=self.puucal_cmd, width=9).grid(row=0, column=3, padx=3)
        ttk.Label(cal, text="measured").grid(row=0, column=4, sticky="e")
        ttk.Entry(cal, textvariable=self.puucal_meas, width=9).grid(row=0, column=5, padx=3)
        ttk.Button(cal, text="Run", command=self.cmd_puucal, width=10).grid(
            row=0, column=6, padx=4
        )
        ttk.Label(
            cal, text="x/z: mm (suggest). t: deg (applies live PUU/deg). Formula: new = current * cmd/meas.",
            foreground="#555", font=("Segoe UI", 8),
        ).grid(row=1, column=0, columnspan=7, sticky="w", pady=(4, 0))

        ota = ttk.LabelFrame(tab, text="Dual-OTA flash (LAN8720 :8032 — not W5500 / not console :2323)",
                             padding=6)
        ota.grid(row=2, column=0, columnspan=2, sticky="nsew", padx=4, pady=4)
        self.ota_bin = tk.StringVar(value=str(DEFAULT_OTA_BIN))
        self.ota_port = tk.StringVar(value=str(DEFAULT_OTA_PORT))
        self.ota_status = tk.StringVar(value="Idle — motors must be disabled; target reboots after OK COMPLETE")
        self.ota_size = tk.StringVar(value="")
        ttk.Label(ota, text=".bin").grid(row=0, column=0, sticky="e")
        ota_entry = ttk.Entry(ota, textvariable=self.ota_bin, width=62)
        ota_entry.grid(row=0, column=1, columnspan=3, sticky="ew", padx=3)
        btn_browse = ttk.Button(ota, text="Browse…", width=10, command=self._browse_ota_bin)
        btn_browse.grid(row=0, column=4, padx=3)
        ttk.Label(ota, textvariable=self.ota_size, foreground="#555").grid(
            row=0, column=5, sticky="w", padx=4
        )
        ttk.Label(ota, text="OTA port").grid(row=1, column=0, sticky="e", pady=(6, 0))
        ota_port_entry = ttk.Entry(ota, textvariable=self.ota_port, width=8)
        ota_port_entry.grid(row=1, column=1, sticky="w", padx=3, pady=(6, 0))
        ttk.Button(ota, text="Query slot (ota)", width=16, command=lambda: self.send("ota")).grid(
            row=1, column=2, sticky="w", padx=3, pady=(6, 0)
        )
        self.btn_ota_flash = ttk.Button(ota, text="Flash .bin", width=12, command=self.cmd_ota_flash)
        self.btn_ota_flash.grid(row=1, column=3, sticky="w", padx=3, pady=(6, 0))
        self.ota_progress = ttk.Progressbar(ota, mode="determinate", maximum=100.0)
        self.ota_progress.grid(row=2, column=0, columnspan=6, sticky="ew", pady=(8, 2))
        ttk.Label(ota, textvariable=self.ota_status, foreground="#1565c0",
                  font=("Segoe UI", 8)).grid(row=3, column=0, columnspan=6, sticky="w")
        ttk.Label(
            ota,
            text="Uses the connection-bar Host + Password. Firmware rejects START if ENABLED or BUSY. "
                 "CLI remains: py tools/eth_ota_flash.py <bin>",
            foreground="#555",
            font=("Segoe UI", 8),
        ).grid(row=4, column=0, columnspan=6, sticky="w", pady=(2, 0))
        ota.columnconfigure(1, weight=1)
        self._ota_always_on = [ota_entry, btn_browse, ota_port_entry, self.btn_ota_flash]
        self._refresh_ota_bin_info()

        tab.columnconfigure(0, weight=1)
        tab.columnconfigure(1, weight=1)

    def _tab_io(self) -> None:
        tab = ttk.Frame(self.tabs, padding=8)
        self.tabs.add(tab, text="Field I/O & MCP")

        fld = ttk.LabelFrame(tab, text="Field 24 V I/O", padding=6)
        fld.grid(row=0, column=0, sticky="nsew", padx=4, pady=4)
        ttk.Button(fld, text="Read field_din", width=16,
                   command=lambda: self.send("field_din")).grid(row=0, column=0, columnspan=3, pady=2)
        self.dout_idx = tk.StringVar(value="0")
        ttk.Label(fld, text="DOUT").grid(row=1, column=0, sticky="e")
        ttk.Combobox(fld, textvariable=self.dout_idx, values=["0", "1", "2", "3"], width=4,
                     state="readonly").grid(row=1, column=1, padx=3)
        btns = ttk.Frame(fld)
        btns.grid(row=1, column=2)
        ttk.Button(btns, text="Set 1", width=6,
                   command=lambda: self.send(f"field_dout {self.dout_idx.get()} 1")).pack(side=tk.LEFT)
        ttk.Button(btns, text="Set 0", width=6,
                   command=lambda: self.send(f"field_dout {self.dout_idx.get()} 0")).pack(side=tk.LEFT)
        ttk.Label(fld, text="DOUT0 is the gripper (PA0).", foreground="#555",
                  font=("Segoe UI", 8)).grid(row=2, column=0, columnspan=3, sticky="w", pady=(4, 0))

        mcp = ttk.LabelFrame(tab, text="MCP23S17", padding=6)
        mcp.grid(row=0, column=1, sticky="nsew", padx=4, pady=4)
        self.mcp_pin = tk.StringVar(value="0")
        self.mcp_mode = tk.StringVar(value="inpu")
        ttk.Label(mcp, text="pin").grid(row=0, column=0, sticky="e")
        ttk.Spinbox(mcp, textvariable=self.mcp_pin, from_=0, to=15, width=5).grid(row=0, column=1)
        ttk.Label(mcp, text="mode").grid(row=0, column=2, sticky="e")
        ttk.Combobox(mcp, textvariable=self.mcp_mode, values=["inpu", "in", "out0", "out1"],
                     width=6, state="readonly").grid(row=0, column=3, padx=3)
        ttk.Button(mcp, text="Set mode", width=10, command=self.cmd_mcp_pin_mode).grid(
            row=0, column=4, padx=3
        )
        ttk.Label(mcp, text="dump port").grid(row=1, column=0, sticky="e")
        self.mcp_port = tk.StringVar(value="a")
        ttk.Combobox(mcp, textvariable=self.mcp_port, values=["a", "b"], width=4,
                     state="readonly").grid(row=1, column=1)
        ttk.Button(mcp, text="Dump", width=10,
                   command=lambda: self.send(f"mcp_dump {self.mcp_port.get()}")).grid(
            row=1, column=2, padx=3
        )

        reg = ttk.LabelFrame(tab, text="MCP raw register (hex ok, e.g. 0x0A)", padding=6)
        reg.grid(row=1, column=0, columnspan=2, sticky="nsew", padx=4, pady=4)
        self.reg_addr = tk.StringVar(value="")
        self.reg_val = tk.StringVar(value="")
        ttk.Label(reg, text="reg").grid(row=0, column=0, sticky="e")
        ttk.Entry(reg, textvariable=self.reg_addr, width=8).grid(row=0, column=1, padx=3)
        ttk.Button(reg, text="Read", width=8, command=lambda: self.cmd_mcp_reg(False)).grid(
            row=0, column=2, padx=3
        )
        ttk.Label(reg, text="value").grid(row=0, column=3, sticky="e")
        ttk.Entry(reg, textvariable=self.reg_val, width=8).grid(row=0, column=4, padx=3)
        ttk.Button(reg, text="Write", width=8, command=lambda: self.cmd_mcp_reg(True)).grid(
            row=0, column=5, padx=3
        )

        gp = ttk.LabelFrame(tab, text="Direct ESP32 GPIO", padding=6)
        gp.grid(row=2, column=0, columnspan=2, sticky="nsew", padx=4, pady=4)
        self.gpio_num = tk.StringVar(value="")
        ttk.Label(gp, text="gpio").grid(row=0, column=0, sticky="e")
        ttk.Spinbox(gp, textvariable=self.gpio_num, from_=0, to=39, width=5).grid(row=0, column=1)
        ttk.Button(gp, text="Drive 1", width=9, command=lambda: self.cmd_gpio_drive(1)).grid(
            row=0, column=2, padx=3
        )
        ttk.Button(gp, text="Drive 0", width=9, command=lambda: self.cmd_gpio_drive(0)).grid(
            row=0, column=3, padx=3
        )
        ttk.Label(gp, text="Bypasses Gantry. Confirm the pin is not a bus signal.",
                  foreground="#a00", font=("Segoe UI", 8)).grid(
            row=0, column=4, sticky="w", padx=8
        )

        tab.columnconfigure(0, weight=1)
        tab.columnconfigure(1, weight=1)

    def _build_console_toggle(self) -> None:
        self.console_visible = True
        self.console_toggle_bar = ttk.Frame(self.root)
        self.console_toggle_bar.pack(fill=tk.X, padx=8, pady=(0, 0))
        self.btn_console_toggle = ttk.Button(
            self.console_toggle_bar,
            text="▲  Hide console",
            width=18,
            command=self._toggle_console,
        )
        self.btn_console_toggle.pack(side=tk.LEFT)
        ttk.Label(
            self.console_toggle_bar,
            text="Collapse the log to give the waypoint grid more room",
            foreground="#555",
            font=("Segoe UI", 8),
        ).pack(side=tk.LEFT, padx=8)

    def _toggle_console(self) -> None:
        self.console_visible = not self.console_visible
        if self.console_visible:
            self.log_wrap.pack(fill=tk.BOTH, expand=True, padx=8, pady=(4, 0), before=self.cmd_bar)
            self.tabs.pack_configure(expand=False)
            self.btn_console_toggle.configure(text="▲  Hide console")
        else:
            self.log_wrap.pack_forget()
            self.tabs.pack_configure(expand=True)
            self.btn_console_toggle.configure(text="▼  Show console")
        # Let the workspace canvas pick up the new height.
        self.root.after(50, self._redraw_workspace)

    def _build_log_pane(self) -> None:
        self.log_wrap = ttk.Frame(self.root)
        self.log_wrap.pack(fill=tk.BOTH, expand=True, padx=8, pady=(4, 0))

        tools = ttk.Frame(self.log_wrap)
        tools.pack(fill=tk.X)
        ttk.Label(tools, text="Filter").pack(side=tk.LEFT)
        self.filter_var = tk.StringVar()
        ent = ttk.Entry(tools, textvariable=self.filter_var, width=28)
        ent.pack(side=tk.LEFT, padx=4)
        ent.bind("<KeyRelease>", lambda _e: self._rerender())
        self.autoscroll = tk.BooleanVar(value=True)
        ttk.Checkbutton(tools, text="Autoscroll", variable=self.autoscroll).pack(side=tk.LEFT, padx=6)
        ttk.Button(tools, text="Clear", command=self.clear_log).pack(side=tk.LEFT, padx=2)
        ttk.Button(tools, text="Save...", command=self.save_log).pack(side=tk.LEFT, padx=2)
        self.count_var = tk.StringVar(value="0 lines")
        ttk.Label(tools, textvariable=self.count_var, foreground="#555").pack(side=tk.RIGHT)

        self.log = scrolledtext.ScrolledText(
            self.log_wrap, wrap=tk.NONE, font=("Consolas", 9), height=16,
        )
        self.log.pack(fill=tk.BOTH, expand=True, pady=(4, 0))
        self.log.tag_configure("E", foreground="#c62828")
        self.log.tag_configure("W", foreground="#e65100")
        self.log.tag_configure("I", foreground="#111111")
        self.log.tag_configure("D", foreground="#777777")
        self.log.tag_configure("V", foreground="#999999")
        self.log.tag_configure("tx", foreground="#1565c0")
        self.log.tag_configure("sys", foreground="#6a1b9a")
        self.log.configure(state=tk.DISABLED)

    def _build_command_bar(self) -> None:
        self.cmd_bar = ttk.Frame(self.root, padding=(8, 6))
        self.cmd_bar.pack(fill=tk.X)
        ttk.Label(self.cmd_bar, text="Command").pack(side=tk.LEFT)
        self.cmd_var = tk.StringVar()
        self.cmd_entry = ttk.Entry(self.cmd_bar, textvariable=self.cmd_var)
        self.cmd_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=4)
        self.cmd_entry.bind("<Return>", lambda _e: self.send_typed())
        self.cmd_entry.bind("<Up>", self._history_prev)
        self.cmd_entry.bind("<Down>", self._history_next)
        ttk.Button(self.cmd_bar, text="Send", command=self.send_typed).pack(side=tk.LEFT)
        ttk.Label(self.cmd_bar, text="Up/Down = history", foreground="#555").pack(
            side=tk.LEFT, padx=8
        )

    def _collect_controls(self, widget: tk.Misc) -> None:
        for child in widget.winfo_children():
            if isinstance(child, (ttk.Button, ttk.Spinbox, ttk.Combobox, ttk.Radiobutton,
                                  ttk.Checkbutton, ttk.Entry)):
                self.control_widgets.append(child)
            self._collect_controls(child)

    # ------------------------------------------------------------ connection

    def toggle_connect(self) -> None:
        if self.client.connected:
            self.disconnect()
        else:
            self.connect()

    def connect(self) -> None:
        host = self.host_var.get().strip()
        try:
            port = int(self.port_var.get().strip())
        except ValueError:
            messagebox.showerror("Port", "Port must be an integer")
            return
        self.authed = False
        self.pw_sent_for_prompt = False
        # New TCP session: wait for firmware "Workspace calibrated:" (status).
        self.homed = False
        self.calibrated = False
        self.z_homed = False
        self.z_calibrated = False
        self.t_homed = False
        self.t_calibrated = False
        self.workspace_calibrated = False
        self.workspace_cal_known = False
        self._fade_workspace()
        self._set_alarm_strip(False)
        try:
            self.client.connect(host, port)
        except OSError as exc:
            messagebox.showerror("Connect", f"{host}:{port}\n\n{exc}")
            return
        self.btn_connect.configure(text="Disconnect")
        self.conn_var.set(f"Connected {host}:{port} - authenticating")
        self.conn_label.configure(foreground="#e65100")
        self._log("sys", f"=== connected to {host}:{port} ===")
        self._set_online(True)
        self._refresh_move_gate()

    def disconnect(self) -> None:
        self.client.close()
        self._on_closed("disconnected locally")

    def _on_closed(self, reason: str) -> None:
        self.client.close()
        self.authed = False
        self.btn_connect.configure(text="Connect")
        self.conn_var.set("Disconnected")
        self.conn_label.configure(foreground="#a00")
        self._set_online(False)
        self._log("sys", f"=== {reason} ===")

    def _set_online(self, online: bool) -> None:
        state = tk.NORMAL if online else tk.DISABLED
        for widget in self.control_widgets:
            try:
                if isinstance(widget, ttk.Combobox):
                    widget.configure(state="readonly" if online else tk.DISABLED)
                else:
                    widget.configure(state=state)
            except tk.TclError:
                pass
        self.btn_stop.configure(state=state)
        self.btn_disable.configure(state=state)
        self._refresh_move_gate()
        if not self._ota_busy:
            self._set_ota_widgets_enabled(True)
        if hasattr(self, "wp_trace_check"):
            try:
                self.wp_trace_check.configure(state=tk.NORMAL)
            except tk.TclError:
                pass

    def _set_ota_widgets_enabled(self, enabled: bool) -> None:
        state = tk.NORMAL if enabled else tk.DISABLED
        for widget in self._ota_always_on:
            try:
                widget.configure(state=state)
            except tk.TclError:
                pass

    # ---------------------------------------------------------------- events

    def _drain_events(self) -> None:
        try:
            while True:
                kind, payload = self.events.get_nowait()
                if kind == "line":
                    self._handle_line(str(payload))
                elif kind == "prompt":
                    self._handle_prompt(str(payload))
                elif kind == "closed":
                    self._abort_wp_follow(silent=True)
                    self._on_closed(str(payload))
                elif kind == "ota_log":
                    self._log("sys", str(payload))
                elif kind == "ota_progress":
                    sent, total, kbps = payload  # type: ignore[misc]
                    self._on_ota_progress(int(sent), int(total), float(kbps))
                elif kind == "ota_done":
                    self._on_ota_done(str(payload) == "ok")
        except queue.Empty:
            pass
        self._wp_follow_tick()
        self.root.after(40, self._drain_events)

    def _handle_prompt(self, prompt: str) -> None:
        if prompt == PROMPT_PASSWORD:
            if self.pw_sent_for_prompt:
                return
            pw = self.pw_var.get()
            if not pw:
                self._log("sys", "Password required - enter it in the bar and press Connect again.")
                return
            try:
                self.client.send_line(pw)
            except OSError as exc:
                self._log("sys", f"password send failed: {exc}")
                return
            self.pw_sent_for_prompt = True
            self._log("sys", "(password sent)")
        else:
            # "> " means the firmware is ready for a command.
            if not self.authed:
                self._mark_authed()

    def _mark_authed(self) -> None:
        first_time = not self.authed
        self.authed = True
        self.pw_sent_for_prompt = False
        host = self.host_var.get().strip()
        self.conn_var.set(f"Connected {host} - ready")
        self.conn_label.configure(foreground="#2e7d32")
        self._refresh_move_gate()
        if first_time:
            # Read-only, and it fills the live-state strip straight away.
            self.root.after(120, lambda: self.send("status"))

    def _handle_line(self, raw: str) -> None:
        text = raw.rstrip()
        if not text:
            return

        if text.startswith("OK authenticated"):
            if "recent IP" in text:
                self._log("sys", "already authenticated (recent IP)")
            self._mark_authed()
        elif text.startswith("ERROR: bad password"):
            self.pw_sent_for_prompt = False
            self._log("sys", "bad password - fix it in the bar; the firmware will re-prompt.")
        elif text.startswith("ERROR: too many failures"):
            self._log("sys", "auth rejected - too many failures.")

        severity = "I"
        match = ESP_LINE_RE.match(text)
        if match:
            severity = match.group(1)
        if severity == "E" or text.startswith("E ("):
            self._set_alarm_strip(True)
        self._parse_state(text)
        self._log(severity, text)

    def _parse_state(self, text: str) -> None:
        body = text
        match = ESP_LINE_RE.match(text)
        if match:
            body = match.group(4)

        flip = re.search(r"CTRL FLIP:\s*busy\s+(\d)\s*->\s*(\d)", body)
        if flip:
            busy_now = flip.group(2) == "1"
            self.state_vars["busy"].set("Yes" if busy_now else "No")
            self._wp_follow_on_busy(busy_now)

        live = re.search(
            r"LIVE POS:\s*x_cmd=(-?[\d.]+)\s*(\w+),\s*x_enc=(-?[\d.]+)\s*\w+,"
            r"\s*z=(-?[\d.]+)\s*\w+,\s*theta=(-?[\d.]+)",
            body,
        )
        if live:
            self.state_vars["x"].set(f"{live.group(1)} {live.group(2)}")
            self.state_vars["x_enc"].set(live.group(3))
            self.state_vars["z"].set(live.group(4))
            self.state_vars["theta"].set(live.group(5))
            self.state_vars["units"].set(live.group(2))
            self._apply_unit_labels(live.group(2))
            self._set_live_xz_from_text(live.group(1), live.group(4))
            return

        x_pos = re.search(r"^X Position:\s*(-?[\d.]+)\s*(\w+)", body)
        z_pos = re.search(r"^Z Position:\s*(-?[\d.]+)\s*(\w+)", body)
        if x_pos:
            self.state_vars["x"].set(f"{x_pos.group(1)} {x_pos.group(2)}")
            self.state_vars["units"].set(x_pos.group(2))
            self._apply_unit_labels(x_pos.group(2))
            z_raw = self.state_vars["z"].get().split()[0] if self.state_vars["z"].get() != "-" else None
            if z_raw:
                self._set_live_xz_from_text(x_pos.group(1), z_raw)
            return
        if z_pos:
            self.state_vars["z"].set(f"{z_pos.group(1)} {z_pos.group(2)}")
            self.state_vars["units"].set(z_pos.group(2))
            self._apply_unit_labels(z_pos.group(2))
            x_raw = self.state_vars["x"].get().split()[0] if self.state_vars["x"].get() != "-" else None
            if x_raw:
                self._set_live_xz_from_text(x_raw, z_pos.group(1))
            return

        simple = [
            (r"^X Encoder\s*:\s*(-?\d+)", "x_enc"),
            (r"^Theta:\s*(-?[\d.]+)", "theta"),
            (r"^Motor Enabled:\s*(Yes|No)", "enabled"),
            (r"^Busy:\s*(Yes|No)", "busy"),
            (r"^Alarm:\s*(Yes|No)", "alarm"),
            (r"^Units:\s*linear=(\w+)", "units"),
        ]
        for pattern, key in simple:
            hit = re.search(pattern, body)
            if hit:
                self.state_vars[key].set(hit.group(1))
                if key == "busy":
                    self._wp_follow_on_busy(hit.group(1) == "Yes")
                if key == "alarm":
                    self._set_alarm_strip(hit.group(1) == "Yes")
                if key == "units":
                    self._apply_unit_labels(hit.group(1))
                return

        if re.search(r"^OK Motors enabled", body):
            self.state_vars["enabled"].set("Yes")
        elif re.search(r"^OK Motors disabled|^OK Stop requested", body):
            self.state_vars["enabled"].set("No")
            # Firmware `stop` does NOT clear home/calibrate session gates; UI matches.
        else:
            unit = re.match(r"^OK Linear units set to (\w+)", body)
            if unit:
                self.state_vars["units"].set(unit.group(1))
                self._apply_unit_labels(unit.group(1))

        self._sync_profile_from_log(body)
        self._parse_workspace_envelope(body)

        ws_cal = re.search(r"Workspace calibrated:\s*(Yes|No)", body, re.IGNORECASE)
        if ws_cal:
            self.workspace_cal_known = True
            self.workspace_calibrated = ws_cal.group(1).lower() == "yes"
            self._refresh_move_gate()

        # Session gates. Firmware sets per-axis home/cal; Move enables if X, Z, or T
        # pair is ready. Bring-up / test_cycle PASS satisfy X+Z (+ theta when OK).
        if re.search(r"OK Bring-up complete|OK test_cycle PASS", body):
            self._mark_session_xz_ready()
            if "theta failed" not in body.lower():
                self.t_homed = True
                self.t_calibrated = True
                self._refresh_move_gate()
        if re.search(
            r"OK soft-home|OK X drive-managed home accepted|OK X homing started",
            body,
        ):
            self.homed = True
            self._refresh_move_gate()
        if re.search(
            r"OK Z drive-managed home accepted|OK Z homing started",
            body,
        ):
            self.z_homed = True
            self._refresh_move_gate()
        if re.search(r"OK Theta origin captured", body):
            self.t_homed = True
            self._refresh_move_gate()
        if re.search(r"OK Calibrated length|OK X soft-calibrate", body):
            self.calibrated = True
            self._refresh_move_gate()
        if re.search(r"OK Z Calibrated length", body):
            self.z_calibrated = True
            self._refresh_move_gate()
        if re.search(r"OK Theta calibrated:", body):
            self.t_calibrated = True
            self._refresh_move_gate()
        if re.search(r"Calibration failed|Calibration aborted|Z calibration failed|Z calibration aborted", body):
            if "Z calibration" in body:
                self.z_calibrated = False
            else:
                self.calibrated = False
            self._refresh_move_gate()
            # Failed cal keeps prior envelope if any; do not fade mid-session.
        if re.search(r"Run 'home x' first", body):
            self.homed = False
            self._refresh_move_gate()
        if re.search(r"Run 'home z' first", body):
            self.z_homed = False
            self._refresh_move_gate()
        if re.search(r"Run 'home t' first", body):
            self.t_homed = False
            self._refresh_move_gate()
        if re.search(r"ERROR: (X |Z |Theta )?move blocked|ERROR: Move blocked", body, re.I):
            self._log("sys", "Move rejected by firmware - check home/calibrate for axes that move.")

    def _mark_session_xz_ready(self) -> None:
        self.homed = True
        self.calibrated = True
        self.z_homed = True
        self.z_calibrated = True
        self._refresh_move_gate()

    @staticmethod
    def _numeric_text(raw: str) -> str:
        try:
            value = float(raw)
        except ValueError:
            return raw
        if value.is_integer():
            return str(int(value))
        return raw

    def _sync_profile_from_log(self, body: str) -> None:
        """Mirror firmware path speed/accel into the Profile tab spinboxes."""
        if not hasattr(self, "speed_lin"):
            return
        path = re.search(
            r"2-D Path Profile:\s*speed=([\d.]+)\s*\w+/s,.*accel=([\d.]+)\s*\w+/s2",
            body,
        )
        if path:
            self.speed_lin.set(self._numeric_text(path.group(1)))
            self.accel_var.set(self._numeric_text(path.group(2)))
        theta_prof = re.search(
            r"^Theta Profile:\s*speed=(\d+)\s*deg/s,\s*accel=(\d+)\s*deg/s2",
            body,
        )
        if theta_prof:
            self.speed_deg.set(theta_prof.group(1))
            self.accel_theta.set(theta_prof.group(2))
        if path or theta_prof:
            return
        # Legacy / current test_cycle profile summary line.
        cycle = re.search(
            r"\[TEST_CYCLE\] path profile:\s*speed=(\d+)\s*mm/s\s*accel=(\d+)\s*mm/s2",
            body,
        )
        if not cycle:
            cycle = re.search(
                r"\[TEST_CYCLE\].*v=(\d+)\s*mm/s\s*a=(\d+)",
                body,
            )
        if cycle:
            self.speed_lin.set(cycle.group(1))
            self.accel_var.set(cycle.group(2))
            return
        speed = re.search(
            r"^OK Path speed updated:\s*([\d.]+)\s*\w+/s(?:\s*\(resultant\))?(?:,\s*theta=(\d+)\s*deg/s)?",
            body,
        )
        if speed:
            self.speed_lin.set(self._numeric_text(speed.group(1)))
            if speed.group(2):
                self.speed_deg.set(speed.group(2))
        accel = re.search(
            r"^OK Path accel(?: updated)?:\s*(?:accel=)?([\d.]+)\s*(?:/\s*([\d.]+)\s*)?\w+/s2"
            r"(?:\s*\(resultant\))?;?\s*(?:theta\s+(\d+)\s*/\s*(\d+)\s*deg/s2)?",
            body,
        )
        if not accel:
            accel = re.search(
                r"^OK Path accel updated:\s*accel=([\d.]+)\s*\w+/s2(?:,\s*decel=([\d.]+)\s*\w+/s2)?",
                body,
            )
        if accel:
            self.accel_var.set(self._numeric_text(accel.group(1)))
            if accel.lastindex and accel.lastindex >= 2 and accel.group(2):
                self.decel_var.set(self._numeric_text(accel.group(2)))
            if accel.lastindex and accel.lastindex >= 3 and accel.group(3):
                self.accel_theta.set(accel.group(3))
            if accel.lastindex and accel.lastindex >= 4 and accel.group(4):
                self.decel_theta.set(accel.group(4))

    def _motion_unlocked(self) -> bool:
        if not self.client.connected:
            return False
        if self.gate_override.get():
            return True
        if self.workspace_cal_known:
            return self.workspace_calibrated
        x_ok = self.homed and self.calibrated
        z_ok = self.z_homed and self.z_calibrated
        t_ok = self.t_homed and self.t_calibrated
        return x_ok or z_ok or t_ok

    def _refresh_move_gate(self) -> None:
        x_ok = self.homed and self.calibrated
        z_ok = self.z_homed and self.z_calibrated
        t_ok = self.t_homed and self.t_calibrated
        any_homed = self.homed or self.z_homed or self.t_homed
        any_cal = self.calibrated or self.z_calibrated or self.t_calibrated
        self.state_vars["homed"].set("Yes" if any_homed else "no")
        if self.workspace_cal_known:
            self.state_vars["calibrated"].set("Yes" if self.workspace_calibrated else "no")
        else:
            self.state_vars["calibrated"].set("Yes" if any_cal else "no")
        if not hasattr(self, "btn_move"):
            return
        ready = self._motion_unlocked()
        state = tk.NORMAL if ready else tk.DISABLED
        for widget in getattr(self, "move_command_widgets", [self.btn_move]):
            try:
                widget.configure(state=state)
            except tk.TclError:
                pass

    def _apply_unit_labels(self, unit: str) -> None:
        """Switch Move spinbox labels/limits when firmware units are mm vs in."""
        if not hasattr(self, "move_x_label"):
            return
        linear = "in" if unit.lower().startswith("in") else "mm"
        self.units_var.set(linear)
        self.move_x_label.configure(text=f"X ({linear})")
        self.move_z_label.configure(text=f"Z ({linear}, +down)")
        if linear == "in":
            x_to = (self.stroke_x_mm or 550.0) / 25.4
            z_to = (self.stroke_z_mm or 150.0) / 25.4
            self.move_x_spin.configure(to=x_to)
            self.move_z_spin.configure(to=z_to)
        else:
            self.move_x_spin.configure(to=self.stroke_x_mm or 550.0)
            self.move_z_spin.configure(to=self.stroke_z_mm or 150.0)
        self._refresh_waypoint_list()
        self._redraw_workspace()

    def _set_alarm_strip(self, _active: bool) -> None:
        return

    # ------------------------------------------------------------------- log

    def _log(self, severity: str, text: str) -> None:
        self.records.append((severity, text))
        if len(self.records) > MAX_LOG_LINES:
            del self.records[: len(self.records) - MAX_LOG_LINES]
            self._rerender()
            return
        if self._passes_filter(text):
            self._append(severity, text)
        self.count_var.set(f"{len(self.records)} lines")

    def _passes_filter(self, text: str) -> bool:
        needle = self.filter_var.get().strip().lower()
        return not needle or needle in text.lower()

    def _append(self, severity: str, text: str) -> None:
        self.log.configure(state=tk.NORMAL)
        self.log.insert(tk.END, text + "\n", severity)
        if self.autoscroll.get():
            self.log.see(tk.END)
        self.log.configure(state=tk.DISABLED)

    def _rerender(self) -> None:
        self.log.configure(state=tk.NORMAL)
        self.log.delete("1.0", tk.END)
        for severity, text in self.records:
            if self._passes_filter(text):
                self.log.insert(tk.END, text + "\n", severity)
        if self.autoscroll.get():
            self.log.see(tk.END)
        self.log.configure(state=tk.DISABLED)
        self.count_var.set(f"{len(self.records)} lines")

    def clear_log(self) -> None:
        self.records.clear()
        self._rerender()

    def save_log(self) -> None:
        stamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(os.getcwd(), f"lan_debug_{stamp}.log")
        try:
            with open(path, "w", encoding="utf-8") as handle:
                for _severity, text in self.records:
                    handle.write(text + "\n")
        except OSError as exc:
            messagebox.showerror("Save", str(exc))
            return
        self._log("sys", f"saved {len(self.records)} lines to {path}")

    # -------------------------------------------------------------- commands

    def send(self, command: str) -> None:
        if not self.client.connected:
            messagebox.showinfo("Send", "Not connected")
            return
        if ";" in command:
            messagebox.showerror("Send", "';' is a line separator on the firmware side.")
            return
        bad = [ch for ch in command if not (32 <= ord(ch) <= 126)]
        if bad:
            messagebox.showerror("Send", "Only printable ASCII survives the console parser.")
            return
        try:
            self.client.send_line(command)
        except OSError as exc:
            messagebox.showerror("Send", str(exc))
            self._on_closed(f"send failed: {exc}")
            return
        self._log("tx", f"> {command}")

    def send_typed(self) -> None:
        command = self.cmd_var.get().strip()
        if not command:
            return
        self.history.append(command)
        self.history_idx = len(self.history)
        self.send(command)
        self.cmd_var.set("")

    def _history_prev(self, _event: object) -> str:
        if self.history and self.history_idx > 0:
            self.history_idx -= 1
            self.cmd_var.set(self.history[self.history_idx])
        return "break"

    def _history_next(self, _event: object) -> str:
        if self.history_idx < len(self.history) - 1:
            self.history_idx += 1
            self.cmd_var.set(self.history[self.history_idx])
        else:
            self.history_idx = len(self.history)
            self.cmd_var.set("")
        return "break"

    def _send_rate(self, command: str, var: tk.StringVar) -> None:
        value = var.get().strip()
        if not value.isdigit():
            messagebox.showerror(command, "Rate must be a non-negative integer (0 = off).")
            return
        self.send(f"{command} {value}")

    def cmd_stop(self) -> None:
        self._abort_wp_follow(silent=True)
        if self.client.connected:
            self.send("stop")
            if hasattr(self, "wp_follow_status"):
                self.wp_follow_status.set("Follow aborted (STOP)")

    def cmd_enable(self) -> None:
        if self._confirm("Enable motors", "Energise the servos?\n\nEnsure the axes are clear."):
            self.send("enable")

    def cmd_home(self) -> None:
        axis = self.home_axis.get()
        if self._confirm(
            "Home",
            f"Home axis '{axis}'?\n\n"
            "EIP: seek A014/PL (X min) / A015 (Z retract). all = Z then X; "
            "X needs the SAFE_Z band. Seek is locked at 100 mm/s, 2000 mm/s²; "
            "switch-clear creep is 1 mm/s. STOP aborts.",
        ):
            self.send(f"home {axis}")

    def cmd_calibrate(self) -> None:
        axis = self.home_axis.get()
        if self._confirm(
            "Calibrate",
            f"Calibrate axis '{axis}'?\n\n"
            "'all' is full EIP bring-up (Z- then X home/cal then Z+ then SAFE_Z). "
            "Per-axis seeks joint max. Seek is locked at 100 mm/s, 2000 mm/s² "
            "(not Profile path speed). A single-axis cal still needs home this session.",
        ):
            self.send(f"calibrate {axis}")

    def cmd_bringup(self) -> None:
        if self._confirm(
            "Bring-up",
            "Run 'calibrate all' (EIP bring-up)?\n\n"
            "Enable first. Sequence: Z- (A015=0) → X home/cal → park X=35 → "
            "Z+ (A014) → SAFE_Z 35.7 → theta origin. "
            "Seek 100 mm/s / 2000 mm/s². STOP aborts.",
        ):
            self.send("calibrate all")

    def cmd_test_cycle(self) -> None:
        if self._confirm(
            "Test cycle",
            "Run test_cycle?\n\n"
            "Enable + EIP bring-up, then path legs A–F at the live Profile "
            "speed/accel, then theta G–I. Bring-up seek is locked at 100 mm/s / "
            "2000 mm/s². STOP aborts. Motors stay enabled on PASS.",
        ):
            self.send("test_cycle")

    def cmd_test_theta_path(self) -> None:
        if self._confirm(
            "Theta path",
            "Run test_theta_path?\n\n"
            "Combined in-band X+Z+theta (25–75% window). Enable + bring-up first. "
            "Uses live Profile speed/accel. STOP aborts.",
        ):
            self.send("test_theta_path")

    def cmd_move(self) -> None:
        if not self._motion_unlocked():
            messagebox.showinfo(
                "Move",
                "Workspace not calibrated — run bring-up / calibrate all first "
                "(or tick skip calibrated gate).",
            )
            return
        try:
            x = float(self.move_x.get())
            z = float(self.move_z.get())
            theta = float(self.move_t.get())
        except ValueError:
            messagebox.showerror("Move", "X, Z and Theta must be numbers.")
            return
        unit = self.state_vars["units"].get()
        if self._confirm(
            "Move",
            f"Move to X={x:g} {unit}, Z={z:g} {unit}, Theta={theta:g} deg?\n\n"
            "Absolute target. +Z is down (0 = A015 retract, toward belt).",
        ):
            self.send(f"move {x:g} {z:g} {theta:g}")

    def cmd_units(self) -> None:
        unit = self.units_var.get()
        self._apply_unit_labels(unit)
        self.send(f"units {unit}")

    def cmd_speed(self) -> None:
        linear = self.speed_lin.get().strip()
        if not linear.isdigit() or int(linear) <= 0:
            messagebox.showerror("speed", "Speed must be a positive integer.")
            return
        deg = self.speed_deg.get().strip()
        if deg:
            if not deg.isdigit() or int(deg) <= 0:
                messagebox.showerror("speed", "Theta speed must be a positive integer.")
                return
            self.send(f"speed {linear} {deg}")
        else:
            self.send(f"speed {linear}")

    def cmd_accel(self) -> None:
        accel = self.accel_var.get().strip()
        if not accel.isdigit() or int(accel) <= 0:
            messagebox.showerror("accel", "Accel must be a positive integer.")
            return
        d = self.decel_var.get().strip()
        ta = self.accel_theta.get().strip()
        td = self.decel_theta.get().strip()
        for label, raw in (("Decel", d), ("Theta accel", ta), ("Theta decel", td)):
            if raw and (not raw.isdigit() or int(raw) <= 0):
                messagebox.showerror("accel", f"{label} must be a positive integer.")
                return
        # Firmware args are positional: accel [d] [ta] [td].
        if ta and not d:
            messagebox.showerror("accel", "Theta accel requires decel (use same value if unused).")
            return
        if td and (not d or not ta):
            messagebox.showerror(
                "accel", "Theta decel requires decel and theta accel (positional)."
            )
            return
        cmd = f"accel {accel}"
        if d:
            cmd += f" {d}"
        if ta:
            cmd += f" {ta}"
        if td:
            cmd += f" {td}"
        self.send(cmd)

    def cmd_puucal(self) -> None:
        try:
            commanded = float(self.puucal_cmd.get())
            measured = float(self.puucal_meas.get())
        except ValueError:
            messagebox.showerror("puucal", "Commanded and measured must be numbers.")
            return
        if commanded <= 0 or measured <= 0:
            messagebox.showerror("puucal", "Both values must be > 0 (new = current * cmd/meas).")
            return
        axis = self.puucal_axis.get()
        note = (
            "Applies live theta PUU/deg; re-run home t after."
            if axis == "t"
            else "Suggests new PUU/mm (does not write NVS)."
        )
        if self._confirm(
            "puucal",
            f"puucal {axis} {commanded:g} {measured:g}?\n\n{note}",
        ):
            self.send(f"puucal {axis} {commanded:g} {measured:g}")

    def cmd_puu_t(self) -> None:
        scale = self.puu_t_scale.get().strip()
        if not scale.isdigit() or int(scale) <= 0:
            messagebox.showerror("puu t", "Scale must be a positive integer (PUU/deg).")
            return
        if self._confirm(
            "puu t",
            f"Set live theta PUU/deg to {scale}?\n\nRe-run home t after changing scale.",
        ):
            self.send(f"puu t {scale}")

    def cmd_thetalim(self) -> None:
        try:
            lo = float(self.thetalim_min.get())
            hi = float(self.thetalim_max.get())
        except ValueError:
            messagebox.showerror("thetalim", "min and max must be numbers (deg).")
            return
        if lo >= hi:
            messagebox.showerror("thetalim", "min must be < max.")
            return
        self.send(f"thetalim {lo:g} {hi:g}")

    def _refresh_ota_bin_info(self) -> None:
        path = Path(self.ota_bin.get().strip())
        if path.is_file():
            size = path.stat().st_size
            self.ota_size.set(f"{size:,} bytes ({size / 1024:.1f} KB)")
        elif str(path):
            self.ota_size.set("file not found")
        else:
            self.ota_size.set("")

    def _browse_ota_bin(self) -> None:
        initial = Path(self.ota_bin.get().strip())
        start = initial.parent if initial.parent.is_dir() else (REPO_ROOT / "idf" / "build")
        if not start.is_dir():
            start = REPO_ROOT
        chosen = filedialog.askopenfilename(
            title="Firmware .bin (Dual-OTA)",
            initialdir=str(start),
            filetypes=[("Firmware binary", "*.bin"), ("All files", "*.*")],
        )
        if chosen:
            self.ota_bin.set(chosen)
            self._refresh_ota_bin_info()

    def _ota_motion_block_reason(self) -> str | None:
        enabled = str(self.state_vars["enabled"].get()).strip().lower()
        busy = str(self.state_vars["busy"].get()).strip().lower()
        if enabled in ("yes", "1", "true"):
            return "Motors are enabled — Disable motors before Dual-OTA."
        if busy in ("yes", "1", "true"):
            return "Gantry is busy — STOP / wait for idle before Dual-OTA."
        return None

    def cmd_ota_flash(self) -> None:
        if self._ota_busy:
            return
        reason = self._ota_motion_block_reason()
        if reason:
            messagebox.showerror("Dual-OTA", reason)
            return
        path = self.ota_bin.get().strip()
        if not path or not os.path.isfile(path):
            messagebox.showerror("Dual-OTA", f"Firmware file not found:\n{path}")
            return
        host = self.host_var.get().strip()
        if not host:
            messagebox.showerror("Dual-OTA", "Host is empty (connection bar).")
            return
        try:
            port = int(self.ota_port.get().strip())
        except ValueError:
            messagebox.showerror("Dual-OTA", "OTA port must be an integer (default 8032).")
            return
        size = os.path.getsize(path)
        if not self._confirm(
            "Dual-OTA flash",
            f"Stream {os.path.basename(path)} ({size / 1024:.1f} KB) to {host}:{port}?\n\n"
            "Motors must stay disabled. Firmware rejects START if ENABLED or BUSY.\n"
            "The WT32 will reboot into the other OTA slot; this console TCP session will drop.",
        ):
            return
        self._refresh_ota_bin_info()
        self._ota_busy = True
        self._set_ota_widgets_enabled(False)
        self.ota_progress["value"] = 0.0
        self.ota_status.set(f"Connecting to {host}:{port}…")
        self._log("sys", f"=== Dual-OTA start {host}:{port} {path} ({size} bytes) ===")
        password = self.pw_var.get()
        threading.Thread(
            target=self._ota_worker,
            args=(host, port, password, path),
            name="eth-ota",
            daemon=True,
        ).start()

    def _ota_worker(self, host: str, port: int, password: str, path: str) -> None:
        ok = eth_ota_flash.flash_ota(
            host,
            port,
            password,
            path,
            log=lambda msg: self.events.put(("ota_log", msg)),
            progress=lambda sent, total, kbps: self.events.put(
                ("ota_progress", (sent, total, kbps))
            ),
        )
        self.events.put(("ota_done", "ok" if ok else "fail"))

    def _on_ota_progress(self, sent: int, total: int, kbps: float) -> None:
        pct = (100.0 * sent / total) if total else 0.0
        self.ota_progress["value"] = pct
        self.ota_status.set(
            f"{pct:5.1f}%  {sent / 1024:.1f} / {total / 1024:.1f} KB  @ {kbps:.1f} KB/s"
        )

    def _on_ota_done(self, ok: bool) -> None:
        self._ota_busy = False
        self._set_ota_widgets_enabled(True)
        if ok:
            self.ota_progress["value"] = 100.0
            self.ota_status.set("OK COMPLETE — target rebooting; reconnect console after it is up")
            if self.client.connected:
                self.disconnect()
        else:
            self.ota_status.set("OTA failed — see log (motors enabled? AUTH? file size?)")

    def cmd_autotune(self) -> None:
        if self._confirm(
            "Autotune",
            "Run autotune theta?\n\n"
            "Needs free axis / drive AF (HCS01 C1800). Clear the workspace.",
        ):
            self.send("autotune theta")

    def cmd_mcp_pin_mode(self) -> None:
        pin = self.mcp_pin.get().strip()
        if not pin.isdigit() or not 0 <= int(pin) <= 15:
            messagebox.showerror("mcp_pin_mode", "Pin must be 0..15.")
            return
        self.send(f"mcp_pin_mode {pin} {self.mcp_mode.get()}")

    def cmd_mcp_reg(self, write: bool) -> None:
        reg = self._parse_int(self.reg_addr.get(), "register", maximum=0x1F)
        if reg is None:
            return
        if not write:
            self.send(f"mcp_reg r 0x{reg:02X}")
            return
        value = self._parse_int(self.reg_val.get(), "value", maximum=0xFF)
        if value is None:
            return
        if self._confirm(
            "mcp_reg write",
            f"Write 0x{value:02X} to MCP register 0x{reg:02X}?\n\n"
            "Raw register writes can reconfigure the SPI3 expander.",
        ):
            self.send(f"mcp_reg w 0x{reg:02X} 0x{value:02X}")

    def cmd_gpio_drive(self, level: int) -> None:
        gpio = self.gpio_num.get().strip()
        if not gpio.isdigit() or not 0 <= int(gpio) <= 39:
            messagebox.showerror("gpio_drive", "GPIO must be 0..39.")
            return
        if self._confirm(
            "gpio_drive",
            f"Drive GPIO {gpio} to {level}?\n\nThis bypasses Gantry. Driving a bus pin "
            "(SPI, RMII, UART) can break Class 1 or the LAN link.",
        ):
            self.send(f"gpio_drive {gpio} {level}")

    @staticmethod
    def _parse_int(text: str, label: str, maximum: int = 0xFF) -> int | None:
        raw = text.strip()
        if not raw:
            messagebox.showerror("mcp_reg", f"Missing {label}.")
            return None
        try:
            value = int(raw, 16) if raw.lower().startswith("0x") else int(raw, 0)
        except ValueError:
            messagebox.showerror("mcp_reg", f"{label} must be hex (0x0A) or decimal.")
            return None
        if not 0 <= value <= maximum:
            messagebox.showerror(
                "mcp_reg", f"{label} must be 0..0x{maximum:02X} (got 0x{value:X})."
            )
            return None
        return value

    def _confirm(self, title: str, message: str) -> bool:
        return messagebox.askokcancel(title, message, icon=messagebox.WARNING)

    def on_close(self) -> None:
        self.client.close()
        self.root.destroy()


def print_commands() -> None:
    width = max(len(usage) for _n, usage, _d in COMMANDS)
    print("Firmware console commands (mirrors gantryTestPrintHelp):\n")
    for _name, usage, description in COMMANDS:
        print(f"  {usage.ljust(width)}  {description}")
    print(
        "\nOptional builds: mcp_* / field_* need MCP_DEBUG_CMDS, selftest needs "
        "CONFIG_GANTRY_SELFTEST."
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="WT32 LAN8720 gantry debug UI")
    parser.add_argument("--host", default=DEFAULT_HOST, help="WT32 plant IP")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--password", default=DEFAULT_PASSWORD)
    parser.add_argument(
        "--list-commands", action="store_true",
        help="print the console command table and exit (no GUI)",
    )
    parser.add_argument(
        "--connect", action="store_true", help="open the session immediately on startup",
    )
    args = parser.parse_args()

    if args.list_commands:
        print_commands()
        return

    root = tk.Tk()
    style = ttk.Style()
    for theme in ("vista", "clam"):
        if theme in style.theme_names():
            style.theme_use(theme)
            break
    app = LanDebugApp(root, args.host, args.port, args.password)
    if args.connect:
        root.after(200, app.connect)
    root.mainloop()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""LAN8720 gantry debug UI - ESP_LOG stream + full console command surface over TCP.

Covers every command in the firmware `help` menu (`gantryTestPrintHelp` in
`src/gantry_test_console.cpp`), grouped into tabs, with a live status strip parsed
from the inbound log and an always-visible STOP.

Defaults match plant bring-up:
  WT32 LAN8720  192.168.1.100:2323
  Developer PC  192.168.1.10  (this host)

Usage:
  py tools/lan_debug_ui.py
  py tools/lan_debug_ui.py --host 192.168.1.100 --port 2323
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
from tkinter import messagebox, scrolledtext, ttk

DEFAULT_HOST = "192.168.1.100"
DEFAULT_PORT = 2323
DEFAULT_PASSWORD = os.environ.get("GANTRY_TCP_PASSWORD", "LTU_1932")

PROMPT_PASSWORD = "Password: "
PROMPT_READY = "> "

ANSI_RE = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
ESP_LINE_RE = re.compile(r"^([EWIDV])\s+\((\d+)\)\s+([^:]+):\s?(.*)$")

MAX_LOG_LINES = 6000

# Mirrors gantryTestPrintHelp(). Commands marked optional are compiled out unless
# the matching Kconfig symbol is set, so they may answer "Unknown command".
COMMANDS: list[tuple[str, str, str]] = [
    ("help", "help | ?", "show firmware help"),
    ("status", "status", "print gantry status"),
    ("faults", "faults | alarms", "decode X/Z Kinetix FaultCode/WarningCode (e.g. A603)"),
    ("puuinfo", "puuinfo", "print X/Z PUU/mm scale and positions"),
    ("eiptiming", "eiptiming", "dump Class 1 latency p50/p99 (exchange/ot/cycle/cmd2start)"),
    ("puucal", "puucal <x|z> <commanded_mm> <measured_mm>", "suggest new PUU/mm"),
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
    ("home", "home [x|z|t|all]", "home (EIP: seek A014/PL; all=Z then X; X needs SAFE_Z band)"),
    ("calibrate", "calibrate [x|z|t|all]", "calibrate; 'all' = EIP bring-up (Z- → X home/cal → Z+ → SAFE_Z)"),
    ("units", "units <mm|in>", "set linear input/output units"),
    ("speed", "speed <v> [deg_per_s]", "set 2-D path speed (resultant; v in selected linear units/s)"),
    ("accel", "accel <a> [decel]", "set 2-D path accel/decel (resultant; >0, selected linear units/s2)"),
    ("rangelimit", "rangelimit <0|1>", "enable/disable path speed+accel/decel range clamps"),
    ("livepos", "livepos <hz>", "LIVE POS periodic rate (0=off); hz is required"),
    ("axislog", "axislog <hz>", "per-axis MOVE periodic rate (0=off); hz is required"),
    ("move", "move <x> <z> <theta>", "move to (x_linear, z_linear, theta_deg); +Z=down, z=A015 retract"),
    ("grip", "grip <0|1>", "gripper (0=open, 1=close)"),
    ("test_cycle", "test_cycle", "enable + EIP bring-up, then path legs A-F at live speed/accel"),
    ("stop", "stop", "stop all motion and disable"),
    ("alarmreset", "alarmreset | arst", "pulse alarm reset (EIP FaultReset bit)"),
    ("selftest", "selftest", "run basic math/config tests (optional build)"),
    ("logout", "logout | exit | quit", "close the TCP session"),
]


class ConsoleClient:
    """Socket reader/writer with prompt-aware framing.

    Events are pushed onto `events` as (kind, payload) where kind is one of
    "line", "prompt", "info", "closed".
    """

    def __init__(self, events: "queue.Queue[tuple[str, str]]") -> None:
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
        self.events: queue.Queue[tuple[str, str]] = queue.Queue()
        self.client = ConsoleClient(self.events)

        self.authed = False
        self.pw_sent_for_prompt = False
        self.history: list[str] = []
        self.history_idx = 0
        self.records: list[tuple[str, str]] = []  # (severity, text)

        # Session gating mirrors firmware: X move needs X home+cal; Z-only needs
        # Z home+cal. UI enables Move if either pair is ready (firmware enforces
        # per-axis). `stop` does NOT clear gates.
        self.homed = False
        self.calibrated = False
        self.z_homed = False
        self.z_calibrated = False

        root.title(f"WT32 gantry LAN debug - {host}:{port}")
        root.geometry("1180x820")
        root.minsize(900, 620)

        self._build_connection_bar(host, port, password)
        self._build_status_strip()
        self._build_safety_bar()
        self._build_tabs()
        self._build_log_pane()
        self._build_command_bar()

        self.control_widgets: list[ttk.Widget] = []
        self._collect_controls(self.tabs)
        self._set_online(False)

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

    def _build_status_strip(self) -> None:
        # tk.LabelFrame (not ttk) so Alarm can paint the strip background red.
        self.status_strip = tk.LabelFrame(
            self.root, text="Live state (parsed from log)", padx=8, pady=4,
            bg="#f5f5f5", fg="#333",
        )
        self.status_strip.pack(fill=tk.X, padx=8, pady=(0, 4))
        self.state_vars: dict[str, tk.StringVar] = {}
        self.state_value_labels: dict[str, tk.Label] = {}
        fields = [
            ("X", "x"), ("X enc", "x_enc"), ("Z", "z"), ("Theta", "theta"),
            ("Enabled", "enabled"), ("Busy", "busy"), ("Alarm", "alarm"),
            ("Units", "units"), ("Homed", "homed"), ("Calibrated", "calibrated"),
        ]
        for col, (label, key) in enumerate(fields):
            cell = tk.Frame(self.status_strip, bg="#f5f5f5")
            cell.grid(row=0, column=col, padx=6, sticky="w")
            tk.Label(cell, text=label, font=("Segoe UI", 8), bg="#f5f5f5", fg="#555").pack(
                anchor="w"
            )
            var = tk.StringVar(value="-")
            self.state_vars[key] = var
            value = tk.Label(cell, textvariable=var, font=("Consolas", 10, "bold"),
                             bg="#f5f5f5", fg="#111")
            value.pack(anchor="w")
            self.state_value_labels[key] = value
            self.status_strip.columnconfigure(col, weight=1)
        self._status_alarm = False

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
        self.tabs.pack(fill=tk.X, padx=8, pady=4)
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
        ttk.Button(seq, text="Test cycle", command=self.cmd_test_cycle, width=12).grid(
            row=2, column=1, pady=2
        )
        ttk.Label(
            seq,
            text="Bring-up = calibrate all.\n"
                 "Seek 100 mm/s, 2000 mm/s².\n"
                 "Path / test_cycle use Profile.",
            foreground="#555", font=("Segoe UI", 8),
        ).grid(row=3, column=0, columnspan=2, sticky="w", pady=(4, 0))

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
        # Envelope from EXPECTED_ELECTROMECHANICAL_ASSEMBLY: X ≈ 420–550 mm, Z ≈ 150 mm.
        self.move_x_spin = ttk.Spinbox(
            mv, textvariable=self.move_x, from_=0.0, to=550.0, increment=1.0, width=9,
        )
        self.move_x_spin.grid(row=1, column=0, padx=3)
        self.move_z_spin = ttk.Spinbox(
            mv, textvariable=self.move_z, from_=0.0, to=150.0, increment=1.0, width=9,
        )
        self.move_z_spin.grid(row=1, column=1, padx=3)
        ttk.Spinbox(
            mv, textvariable=self.move_t, from_=-360.0, to=360.0, increment=1.0, width=9,
        ).grid(row=1, column=2, padx=3)
        self.btn_move = ttk.Button(mv, text="Move", command=self.cmd_move, width=12)
        self.btn_move.grid(row=2, column=0, columnspan=2, pady=(6, 0))
        self.gate_override = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            mv, text="already homed", variable=self.gate_override,
            command=self._refresh_move_gate,
        ).grid(row=2, column=2, pady=(6, 0), sticky="w")

        grip = ttk.LabelFrame(tab, text="Gripper", padding=6)
        grip.grid(row=0, column=3, sticky="nsew", padx=4, pady=4)
        ttk.Button(grip, text="Open (0)", command=lambda: self.send("grip 0"), width=12).pack(pady=2)
        ttk.Button(grip, text="Close (1)", command=lambda: self.send("grip 1"), width=12).pack(pady=2)

        for col in range(4):
            tab.columnconfigure(col, weight=1)

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
        self.speed_deg = tk.StringVar(value="")
        self.accel_var = tk.StringVar(value="3000")
        self.decel_var = tk.StringVar(value="")
        ttk.Label(prof, text="speed").grid(row=0, column=0, sticky="e")
        ttk.Spinbox(prof, textvariable=self.speed_lin, from_=1, to=2000, width=8).grid(row=0, column=1)
        ttk.Label(prof, text="deg/s (opt)").grid(row=0, column=2, sticky="e")
        ttk.Spinbox(prof, textvariable=self.speed_deg, from_=0, to=3600, width=8).grid(row=0, column=3)
        ttk.Button(prof, text="Set speed", command=self.cmd_speed, width=12).grid(
            row=0, column=4, padx=4
        )
        ttk.Label(prof, text="accel").grid(row=1, column=0, sticky="e")
        ttk.Spinbox(prof, textvariable=self.accel_var, from_=1, to=100000, width=8).grid(row=1, column=1)
        ttk.Label(prof, text="decel (opt)").grid(row=1, column=2, sticky="e")
        ttk.Spinbox(prof, textvariable=self.decel_var, from_=0, to=100000, width=8).grid(row=1, column=3)
        ttk.Button(prof, text="Set accel", command=self.cmd_accel, width=12).grid(
            row=1, column=4, padx=4
        )
        ttk.Button(prof, text="Query (status)", command=lambda: self.send("status")).grid(
            row=2, column=4, padx=4, pady=(4, 0)
        )
        ttk.Label(
            prof,
            text="Path and test_cycle legs only. Home/cal seek is locked at\n"
                 "100 mm/s, 2000 mm/s². rangelimit caps accel at 3000 mm/s².",
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
            ["status", "faults", "puuinfo", "eiptiming", "limits", "pins", "selftest", "help"]
        ):
            ttk.Button(rd, text=cmd, width=11, command=lambda c=cmd: self.send(c)).grid(
                row=col // 4, column=col % 4, padx=3, pady=3
            )

        cal = ttk.LabelFrame(tab, text="PUU calibration suggestion", padding=6)
        cal.grid(row=1, column=0, sticky="nsew", padx=4, pady=4)
        self.puucal_axis = tk.StringVar(value="x")
        self.puucal_cmd = tk.StringVar(value="")
        self.puucal_meas = tk.StringVar(value="")
        ttk.Label(cal, text="axis").grid(row=0, column=0, sticky="e")
        ttk.Combobox(cal, textvariable=self.puucal_axis, values=["x", "z"], width=4,
                     state="readonly").grid(row=0, column=1, padx=3)
        ttk.Label(cal, text="commanded mm").grid(row=0, column=2, sticky="e")
        ttk.Entry(cal, textvariable=self.puucal_cmd, width=9).grid(row=0, column=3, padx=3)
        ttk.Label(cal, text="measured mm").grid(row=0, column=4, sticky="e")
        ttk.Entry(cal, textvariable=self.puucal_meas, width=9).grid(row=0, column=5, padx=3)
        ttk.Button(cal, text="Suggest", command=self.cmd_puucal, width=10).grid(
            row=0, column=6, padx=4
        )
        ttk.Label(
            cal, text="Read-only: prints new = current * (commanded / measured).",
            foreground="#555", font=("Segoe UI", 8),
        ).grid(row=1, column=0, columnspan=7, sticky="w", pady=(4, 0))

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

    def _build_log_pane(self) -> None:
        wrap = ttk.Frame(self.root)
        wrap.pack(fill=tk.BOTH, expand=True, padx=8, pady=(4, 0))

        tools = ttk.Frame(wrap)
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

        self.log = scrolledtext.ScrolledText(wrap, wrap=tk.NONE, font=("Consolas", 9), height=16)
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
        bar = ttk.Frame(self.root, padding=(8, 6))
        bar.pack(fill=tk.X)
        ttk.Label(bar, text="Command").pack(side=tk.LEFT)
        self.cmd_var = tk.StringVar()
        self.cmd_entry = ttk.Entry(bar, textvariable=self.cmd_var)
        self.cmd_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=4)
        self.cmd_entry.bind("<Return>", lambda _e: self.send_typed())
        self.cmd_entry.bind("<Up>", self._history_prev)
        self.cmd_entry.bind("<Down>", self._history_next)
        ttk.Button(bar, text="Send", command=self.send_typed).pack(side=tk.LEFT)
        ttk.Label(bar, text="Up/Down = history", foreground="#555").pack(side=tk.LEFT, padx=8)

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
        # New TCP session: firmware session gates are boot-scoped, but this UI
        # has not observed them yet, so grey Move until home+calibrate (or override).
        self.homed = False
        self.calibrated = False
        self.z_homed = False
        self.z_calibrated = False
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

    # ---------------------------------------------------------------- events

    def _drain_events(self) -> None:
        try:
            while True:
                kind, payload = self.events.get_nowait()
                if kind == "line":
                    self._handle_line(payload)
                elif kind == "prompt":
                    self._handle_prompt(payload)
                elif kind == "closed":
                    self._on_closed(payload)
        except queue.Empty:
            pass
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
            return

        for pattern, key in (
            (r"^X Position:\s*(-?[\d.]+)\s*(\w+)", "x"),
            (r"^Z Position:\s*(-?[\d.]+)\s*(\w+)", "z"),
        ):
            hit = re.search(pattern, body)
            if hit:
                self.state_vars[key].set(f"{hit.group(1)} {hit.group(2)}")
                self.state_vars["units"].set(hit.group(2))
                self._apply_unit_labels(hit.group(2))
                return

        simple = [
            (r"^X Encoder\s*:\s*(-?\d+)", "x_enc"),
            (r"^Theta:\s*(-?\d+)", "theta"),
            (r"^Motor Enabled:\s*(Yes|No)", "enabled"),
            (r"^Busy:\s*(Yes|No)", "busy"),
            (r"^Alarm:\s*(Yes|No)", "alarm"),
            (r"^Units:\s*linear=(\w+)", "units"),
        ]
        for pattern, key in simple:
            hit = re.search(pattern, body)
            if hit:
                self.state_vars[key].set(hit.group(1))
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

        # Session gates. Firmware sets per-axis home/cal; Move enables if X or Z
        # pair is ready (Z-only jog before X bring-up). Bring-up / test_cycle PASS
        # satisfy all four (calibrate all / test_cycle).
        if re.search(r"OK Bring-up complete|OK test_cycle PASS", body):
            self._mark_session_xz_ready()
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
        if re.search(r"OK Calibrated length|OK X soft-calibrate", body):
            self.calibrated = True
            self._refresh_move_gate()
        if re.search(r"OK Z Calibrated length", body):
            self.z_calibrated = True
            self._refresh_move_gate()
        if re.search(r"Calibration failed|Calibration aborted|Z calibration failed|Z calibration aborted", body):
            if "Z calibration" in body:
                self.z_calibrated = False
            else:
                self.calibrated = False
            self._refresh_move_gate()
        if re.search(r"Run 'home x' first", body):
            self.homed = False
            self._refresh_move_gate()
        if re.search(r"ERROR: (X |Z )?Move blocked|ERROR: Move blocked", body):
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
            return
        cycle = re.search(
            r"\[TEST_CYCLE\] path profile:\s*speed=(\d+)\s*mm/s\s*accel=(\d+)\s*mm/s2",
            body,
        )
        if cycle:
            self.speed_lin.set(cycle.group(1))
            self.accel_var.set(cycle.group(2))
            return
        speed = re.search(r"^OK Path speed updated:\s*([\d.]+)\s*\w+/s", body)
        if speed:
            self.speed_lin.set(self._numeric_text(speed.group(1)))
        accel = re.search(r"^OK Path accel updated:\s*accel=([\d.]+)\s*\w+/s2", body)
        if accel:
            self.accel_var.set(self._numeric_text(accel.group(1)))

    def _refresh_move_gate(self) -> None:
        x_ok = self.homed and self.calibrated
        z_ok = self.z_homed and self.z_calibrated
        self.state_vars["homed"].set("Yes" if (self.homed or self.z_homed) else "no")
        self.state_vars["calibrated"].set(
            "Yes" if (self.calibrated or self.z_calibrated) else "no"
        )
        if not hasattr(self, "btn_move"):
            return
        ready = self.client.connected and (
            self.gate_override.get() or x_ok or z_ok
        )
        self.btn_move.configure(state=tk.NORMAL if ready else tk.DISABLED)

    def _apply_unit_labels(self, unit: str) -> None:
        """Switch Move spinbox labels/limits when firmware units are mm vs in."""
        if not hasattr(self, "move_x_label"):
            return
        linear = "in" if unit.lower().startswith("in") else "mm"
        self.units_var.set(linear)
        self.move_x_label.configure(text=f"X ({linear})")
        self.move_z_label.configure(text=f"Z ({linear}, +down)")
        if linear == "in":
            self.move_x_spin.configure(to=550.0 / 25.4)
            self.move_z_spin.configure(to=150.0 / 25.4)
        else:
            self.move_x_spin.configure(to=550.0)
            self.move_z_spin.configure(to=150.0)

    def _set_alarm_strip(self, active: bool) -> None:
        if not hasattr(self, "status_strip"):
            return
        if getattr(self, "_status_alarm", None) == active:
            return
        self._status_alarm = active
        bg = "#ffcdd2" if active else "#f5f5f5"
        fg = "#b71c1c" if active else "#111"
        self.status_strip.configure(bg=bg)
        for child in self.status_strip.winfo_children():
            try:
                child.configure(bg=bg)
            except tk.TclError:
                pass
            for grand in child.winfo_children():
                try:
                    grand.configure(bg=bg)
                except tk.TclError:
                    pass
        alarm_label = self.state_value_labels.get("alarm")
        if alarm_label is not None:
            alarm_label.configure(fg=fg, bg=bg)

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
        if self.client.connected:
            self.send("stop")

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
            "Enable first. Sequence: Z home (A015 = 0) → X home/cal → park X → "
            "Z+ (A014) → SAFE_Z. Seek 100 mm/s / 2000 mm/s². STOP aborts.",
        ):
            self.send("calibrate all")

    def cmd_test_cycle(self) -> None:
        if self._confirm(
            "Test cycle",
            "Run test_cycle?\n\n"
            "Enable + EIP bring-up, then path legs A–F at the live Profile "
            "speed/accel. Bring-up seek is locked at 100 mm/s / 2000 mm/s². "
            "STOP aborts. Motors stay enabled on PASS.",
        ):
            self.send("test_cycle")

    def cmd_move(self) -> None:
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
        decel = self.decel_var.get().strip()
        if decel:
            if not decel.isdigit() or int(decel) <= 0:
                messagebox.showerror("accel", "Decel must be a positive integer.")
                return
            self.send(f"accel {accel} {decel}")
        else:
            self.send(f"accel {accel}")

    def cmd_puucal(self) -> None:
        try:
            commanded = float(self.puucal_cmd.get())
            measured = float(self.puucal_meas.get())
        except ValueError:
            messagebox.showerror("puucal", "Commanded and measured must be numbers.")
            return
        if commanded <= 0 or measured <= 0:
            messagebox.showerror("puucal", "Both distances must be > 0 (new = current * cmd/meas).")
            return
        self.send(f"puucal {self.puucal_axis.get()} {commanded:g} {measured:g}")

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

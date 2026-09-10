#!/usr/bin/env python3
"""driver_config_ui.py — Dedicated EtherNet/IP Driver Configuration & Tuning GUI.

Provides a full control, configuration, and autotuning surface for:
  - Theta Axis: Bosch Rexroth IndraDrive Cs HCS01 / SCHUNK ERD-04
      * Mechanical Zero Calibration (C0300 Set Absolute Position + C2200 NV Save)
      * Inertia Identification & Auto-Tuning (C1800)
      * Live Micro-Jogging for Visual Squaring & Alignment
      * SERCOS Parameter Access & Alarm Decoding
  - X & Z Axes: Rockwell Kinetix 5100 (2198-E1020 / 2198-E1004)
      * Real-Time & Adaptive Gain Tuning (Class 0x0F Parameter Access)
      * Bandwidth & Response Level Configuration
      * Load Inertia & Filter Inspection
      * Travel Scale Verification (PUU/mm Calibration)
  - Unified System Homing & Bring-up:
      * One-click full bring-up (Z- -> X home/cal -> X=35 -> Theta=90 -> Z+ cal -> SAFE_Z -> Theta=0)

Communicates with the WT32-ETH01 gantry bridge over the high-speed TCP console (default: 10.42.0.100:2323).
"""

from __future__ import annotations

import argparse
import os
import queue
import re
import socket
import threading
import time
import tkinter as tk
from tkinter import messagebox, scrolledtext, ttk

DEFAULT_HOST = "10.42.0.100"
DEFAULT_PORT = 2323
DEFAULT_PASSWORD = os.environ.get("GANTRY_TCP_PASSWORD", "LTU_1932")

PROMPT_PASSWORD = "Password: "
PROMPT_READY = "> "

ANSI_RE = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
ESP_LINE_RE = re.compile(r"^([EWIDV])\s+\((\d+)\)\s+([^:]+):\s?(.*)$")


class GantryTcpClient:
    """Socket communication worker with prompt-aware framing and queue dispatch."""

    def __init__(self, event_q: "queue.Queue[tuple[str, str]]") -> None:
        self.event_q = event_q
        self._sock: socket.socket | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._send_lock = threading.Lock()

    @property
    def connected(self) -> bool:
        return self._sock is not None

    def connect(self, host: str, port: int, timeout: float = 4.0) -> None:
        sock = socket.create_connection((host, port), timeout=timeout)
        sock.settimeout(0.3)
        self._sock = sock
        self._stop.clear()
        self._thread = threading.Thread(target=self._pump, name="driver-ui-reader", daemon=True)
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
        self.event_q.put(("closed", "Connection closed"))

    def send(self, cmd: str) -> None:
        if self._sock is None:
            raise RuntimeError("Not connected")
        cleaned = re.sub(r"[^\x20-\x7E]", "", cmd).strip()
        data = (cleaned + "\n").encode("utf-8")
        with self._send_lock:
            self._sock.sendall(data)

    def _pump(self) -> None:
        buf = bytearray()
        sock = self._sock
        while not self._stop.is_set() and sock is not None:
            try:
                chunk = sock.recv(2048)
            except (socket.timeout, TimeoutError):
                continue
            except OSError as err:
                if not self._stop.is_set():
                    self.event_q.put(("closed", f"Socket error: {err}"))
                break

            if not chunk:
                if not self._stop.is_set():
                    self.event_q.put(("closed", "Remote host closed connection"))
                break

            buf.extend(chunk)
            while True:
                idx_n = buf.find(b"\n")
                idx_r = buf.find(b"\r")
                idx = -1
                if idx_n != -1 and idx_r != -1:
                    idx = min(idx_n, idx_r)
                elif idx_n != -1:
                    idx = idx_n
                else:
                    idx = idx_r

                if idx != -1:
                    raw = bytes(buf[:idx])
                    del buf[: idx + 1]
                    while buf and buf[0] in (ord("\r"), ord("\n")):
                        del buf[0]
                    text = raw.decode("utf-8", errors="replace")
                    self.event_q.put(("line", text))
                    continue

                if buf.endswith(PROMPT_PASSWORD.encode("utf-8")):
                    raw = bytes(buf)
                    buf.clear()
                    self.event_q.put(("prompt", raw.decode("utf-8", errors="replace")))
                    break

                if buf.endswith(PROMPT_READY.encode("utf-8")):
                    raw = bytes(buf)
                    buf.clear()
                    self.event_q.put(("prompt", raw.decode("utf-8", errors="replace")))
                    break

                break


class DriverConfigApp(tk.Tk):
    """Main Application Window for Driver Configuration."""

    def __init__(self, host: str, port: int, password: str) -> None:
        super().__init__()
        self.title("EtherNet/IP Driver Configuration & Tuning Suite — WT32-ETH01")
        self.geometry("1180x820")
        self.minsize(960, 680)

        self.host_var = tk.StringVar(value=host)
        self.port_var = tk.IntVar(value=port)
        self.pass_var = tk.StringVar(value=password)
        self.conn_status_var = tk.StringVar(value="Disconnected")

        # Telemetry variables
        self.pos_x_var = tk.StringVar(value="-- mm")
        self.pos_z_var = tk.StringVar(value="-- mm")
        self.pos_th_var = tk.StringVar(value="-- deg")
        self.state_enabled_var = tk.StringVar(value="--")
        self.state_busy_var = tk.StringVar(value="--")
        self.state_alarm_var = tk.StringVar(value="--")
        self.theta_diag_var = tk.StringVar(value="--")

        # Command / Jog state
        self.jog_step_th = tk.DoubleVar(value=1.0)
        self.jog_step_x = tk.DoubleVar(value=5.0)
        self.jog_step_z = tk.DoubleVar(value=5.0)
        self.target_th_var = tk.DoubleVar(value=0.0)
        self.target_x_var = tk.DoubleVar(value=35.0)
        self.target_z_var = tk.DoubleVar(value=35.7)

        self.events: queue.Queue[tuple[str, str]] = queue.Queue()
        self.client = GantryTcpClient(self.events)

        self._build_ui()
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.after(50, self._process_events)
        self.after(500, self._auto_poll)

    def _build_ui(self) -> None:
        # Top Connection Bar
        top_bar = ttk.LabelFrame(self, text="Connection & System Safety", padding=6)
        top_bar.pack(fill="x", padx=8, pady=4)

        ttk.Label(top_bar, text="Host:").pack(side="left", padx=2)
        ttk.Entry(top_bar, textvariable=self.host_var, width=14).pack(side="left", padx=2)

        ttk.Label(top_bar, text="Port:").pack(side="left", padx=2)
        ttk.Entry(top_bar, textvariable=self.port_var, width=6).pack(side="left", padx=2)

        ttk.Label(top_bar, text="Password:").pack(side="left", padx=2)
        ttk.Entry(top_bar, textvariable=self.pass_var, show="*", width=10).pack(side="left", padx=2)

        self.btn_connect = ttk.Button(top_bar, text="Connect", command=self._toggle_connect)
        self.btn_connect.pack(side="left", padx=6)

        ttk.Label(top_bar, text="Status:").pack(side="left", padx=4)
        self.lbl_conn = ttk.Label(top_bar, textvariable=self.conn_status_var, font=("Segoe UI", 9, "bold"))
        self.lbl_conn.pack(side="left", padx=2)

        # Quick Control Buttons on Right
        btn_stop = tk.Button(top_bar, text="STOP MOTION", bg="#d9534f", fg="white", font=("Segoe UI", 9, "bold"),
                             command=lambda: self._send_cmd("stop"), padx=8)
        btn_stop.pack(side="right", padx=4)

        btn_enable = ttk.Button(top_bar, text="Enable Motors", command=lambda: self._send_cmd("enable"))
        btn_enable.pack(side="right", padx=2)

        btn_disable = ttk.Button(top_bar, text="Disable Motors", command=lambda: self._send_cmd("disable"))
        btn_disable.pack(side="right", padx=2)

        btn_arst = ttk.Button(top_bar, text="Reset Alarms", command=lambda: self._send_cmd("alarmreset"))
        btn_arst.pack(side="right", padx=2)

        # Live Status Strip
        status_bar = ttk.Frame(self, padding=4)
        status_bar.pack(fill="x", padx=8, pady=2)

        def add_stat_box(parent, title, var, width=10):
            frame = ttk.LabelFrame(parent, text=title, padding=3)
            frame.pack(side="left", padx=4, fill="both", expand=True)
            lbl = ttk.Label(frame, textvariable=var, font=("Consolas", 11, "bold"), anchor="center")
            lbl.pack(fill="both")
            return frame

        add_stat_box(status_bar, "X Position", self.pos_x_var)
        add_stat_box(status_bar, "Z Position", self.pos_z_var)
        add_stat_box(status_bar, "Theta Position", self.pos_th_var)
        add_stat_box(status_bar, "Enabled", self.state_enabled_var)
        add_stat_box(status_bar, "Moving / Busy", self.state_busy_var)
        add_stat_box(status_bar, "Alarm Active", self.state_alarm_var)

        # Main Tabbed Interface
        self.notebook = ttk.Notebook(self)
        self.notebook.pack(fill="both", expand=True, padx=8, pady=4)

        self._build_theta_tab()
        self._build_x_tab()
        self._build_z_tab()
        self._build_bringup_tab()

        # Bottom Log / Console Drawer
        bottom_frame = ttk.LabelFrame(self, text="EtherNet/IP Controller Telemetry Log", padding=4)
        bottom_frame.pack(fill="both", expand=False, padx=8, pady=4)
        bottom_frame.config(height=180)

        self.txt_log = scrolledtext.ScrolledText(bottom_frame, height=8, font=("Consolas", 9), bg="#1e1e1e", fg="#d4d4d4")
        self.txt_log.pack(fill="both", expand=True)

        cmd_entry_frame = ttk.Frame(bottom_frame)
        cmd_entry_frame.pack(fill="x", pady=2)
        ttk.Label(cmd_entry_frame, text="Raw Command:").pack(side="left", padx=2)
        self.ent_cmd = ttk.Entry(cmd_entry_frame)
        self.ent_cmd.pack(side="left", fill="x", expand=True, padx=4)
        self.ent_cmd.bind("<Return>", lambda e: self._send_raw_entry())
        ttk.Button(cmd_entry_frame, text="Send", command=self._send_raw_entry).pack(side="left", padx=2)
        ttk.Button(cmd_entry_frame, text="Clear Log", command=lambda: self.txt_log.delete("1.0", tk.END)).pack(side="left", padx=2)

    # -------------------------------------------------------------------------
    # TAB 1: THETA AXIS (BOSCH REXROTH HCS01 & SCHUNK ERD-04)
    # -------------------------------------------------------------------------
    def _build_theta_tab(self) -> None:
        tab = ttk.Frame(self.notebook, padding=8)
        self.notebook.add(tab, text="Theta Axis (Rexroth HCS01)")

        left = ttk.Frame(tab)
        left.pack(side="left", fill="both", expand=True, padx=4)

        right = ttk.Frame(tab)
        right.pack(side="right", fill="both", expand=True, padx=4)

        # Panel: Mechanical Zero Calibration (C0300)
        pnl_zero = ttk.LabelFrame(left, text="Mechanical Zero Calibration (C0300 & C2200)", padding=8)
        pnl_zero.pack(fill="x", pady=4)

        desc = ("1. Jog Theta in fine increments until the gripper is 100% square/flat.\n"
                "2. Click 'Zero Drive Origin (C0300)' to burn the zero into NV flash.\n"
                "3. Click 'Re-Home Theta' to lock in ALIGNED 0.000°.")
        ttk.Label(pnl_zero, text=desc, foreground="#555", justify="left").pack(anchor="w", pady=2)

        # Jogging Controls
        jog_frame = ttk.Frame(pnl_zero)
        jog_frame.pack(fill="x", pady=6)

        ttk.Label(jog_frame, text="Jog Step (deg):").grid(row=0, column=0, sticky="w")
        cb_step = ttk.Combobox(jog_frame, textvariable=self.jog_step_th, values=[0.1, 0.5, 1.0, 2.0, 5.0, 10.0, 45.0, 90.0], width=6)
        cb_step.grid(row=0, column=1, padx=4, sticky="w")

        btn_row = ttk.Frame(pnl_zero)
        btn_row.pack(fill="x", pady=4)
        ttk.Button(btn_row, text="<< -5.0°", command=lambda: self._nudge_th(-5.0)).pack(side="left", expand=True, fill="x", padx=1)
        ttk.Button(btn_row, text="< -1.0°", command=lambda: self._nudge_th(-1.0)).pack(side="left", expand=True, fill="x", padx=1)
        ttk.Button(btn_row, text="-0.1°", command=lambda: self._nudge_th(-0.1)).pack(side="left", expand=True, fill="x", padx=1)
        ttk.Button(btn_row, text="+0.1°", command=lambda: self._nudge_th(0.1)).pack(side="left", expand=True, fill="x", padx=1)
        ttk.Button(btn_row, text="+1.0° >", command=lambda: self._nudge_th(1.0)).pack(side="left", expand=True, fill="x", padx=1)
        ttk.Button(btn_row, text="+5.0° >>", command=lambda: self._nudge_th(5.0)).pack(side="left", expand=True, fill="x", padx=1)

        # Move to exact degree
        abs_frame = ttk.Frame(pnl_zero)
        abs_frame.pack(fill="x", pady=4)
        ttk.Label(abs_frame, text="Move to Exact Angle:").pack(side="left", padx=2)
        ent_th = ttk.Entry(abs_frame, textvariable=self.target_th_var, width=8)
        ent_th.pack(side="left", padx=4)
        ttk.Label(abs_frame, text="deg").pack(side="left", padx=1)
        ttk.Button(abs_frame, text="Move Theta", command=lambda: self._move_th_exact(self.target_th_var.get())).pack(side="left", padx=6)

        # Big Action Buttons
        act_frame = ttk.Frame(pnl_zero)
        act_frame.pack(fill="x", pady=8)

        btn_c0300 = tk.Button(act_frame, text="★ ZERO DRIVE ORIGIN (C0300 & C2200) ★",
                              bg="#0275d8", fg="white", font=("Segoe UI", 10, "bold"),
                              command=self._do_c0300, pady=4)
        btn_c0300.pack(fill="x", pady=3)

        btn_homet = ttk.Button(act_frame, text="Re-Home Theta (`home t`)", command=lambda: self._send_cmd("home t"))
        btn_homet.pack(fill="x", pady=2)

        # Panel: Inertia & Automatic Tuning (C1800)
        pnl_tune = ttk.LabelFrame(left, text="Inertia Identification & Auto-Tuning (C1800)", padding=8)
        pnl_tune.pack(fill="x", pady=4)

        ttk.Label(pnl_tune, text="Runs a ±45° sinusoidal identification sweep to calculate total load inertia and tuning gains.", foreground="#555", wraplength=480).pack(anchor="w", pady=2)

        btn_c1800 = tk.Button(pnl_tune, text="Run Autotune Sweep (C1800)", bg="#5cb85c", fg="white", font=("Segoe UI", 9, "bold"),
                              command=lambda: self._send_cmd("autotune theta"), pady=3)
        btn_c1800.pack(fill="x", pady=4)

        # Right Panel: SERCOS IDN & Scaling
        pnl_idn = ttk.LabelFrame(right, text="SERCOS Parameter Access & Diagnosis", padding=8)
        pnl_idn.pack(fill="both", expand=True, pady=4)

        ttk.Button(pnl_idn, text="Refresh Drive Alarms & Diag (`faults`)", command=lambda: self._send_cmd("faults")).pack(fill="x", pady=2)
        ttk.Button(pnl_idn, text="Read Theta Limits & Envelope (`thetalim`)", command=lambda: self._send_cmd("thetalim")).pack(fill="x", pady=2)

        # Theta Joint Limits Config
        lim_frame = ttk.Frame(pnl_idn)
        lim_frame.pack(fill="x", pady=6)
        ttk.Label(lim_frame, text="Set Joint Limits (Min / Max deg):").grid(row=0, column=0, columnspan=2, sticky="w")
        self.th_min_var = tk.DoubleVar(value=-36000.0)
        self.th_max_var = tk.DoubleVar(value=36000.0)
        ttk.Entry(lim_frame, textvariable=self.th_min_var, width=9).grid(row=1, column=0, padx=2, pady=2)
        ttk.Entry(lim_frame, textvariable=self.th_max_var, width=9).grid(row=1, column=1, padx=2, pady=2)
        ttk.Button(lim_frame, text="Apply Limits", command=lambda: self._send_cmd(f"thetalim {self.th_min_var.get()} {self.th_max_var.get()}")).grid(row=1, column=2, padx=4)

        # PUU Scale
        puu_frame = ttk.Frame(pnl_idn)
        puu_frame.pack(fill="x", pady=6)
        ttk.Label(puu_frame, text="Live Theta PUU/deg Scale (Default: 10000.0):").pack(anchor="w")
        self.th_puu_var = tk.DoubleVar(value=10000.0)
        ent_puu = ttk.Entry(puu_frame, textvariable=self.th_puu_var, width=12)
        ent_puu.pack(side="left", padx=2, pady=2)
        ttk.Button(puu_frame, text="Set PUU Scale", command=lambda: self._send_cmd(f"puu t {self.th_puu_var.get()}")).pack(side="left", padx=4)

    # -------------------------------------------------------------------------
    # TAB 2: X AXIS (KINETIX 5100)
    # -------------------------------------------------------------------------
    def _build_x_tab(self) -> None:
        tab = ttk.Frame(self.notebook, padding=8)
        self.notebook.add(tab, text="X Axis (Kinetix 5100)")

        left = ttk.Frame(tab)
        left.pack(side="left", fill="both", expand=True, padx=4)

        right = ttk.Frame(tab)
        right.pack(side="right", fill="both", expand=True, padx=4)

        # Tuning Parameters (Class 0x0F)
        pnl_tune = ttk.LabelFrame(left, text="Kinetix 5100 CIP Tuning Parameters", padding=8)
        pnl_tune.pack(fill="both", expand=True, pady=4)

        ttk.Button(pnl_tune, text="Read All X Parameters (`autotune x read`)", command=lambda: self._send_cmd("autotune x read")).pack(fill="x", pady=3)

        # Gain Adjustment Mode
        mode_frame = ttk.Frame(pnl_tune)
        mode_frame.pack(fill="x", pady=4)
        ttk.Label(mode_frame, text="Gain Mode (ID 217):").pack(side="left")
        self.x_gain_mode = tk.StringVar(value="Mode 1 (Realtime Auto)")
        cb_mode = ttk.Combobox(mode_frame, textvariable=self.x_gain_mode, values=["Manual (0)", "Mode 1 (Realtime Auto)", "Mode 2 (Adaptive Auto)", "Reset (4)"], width=22)
        cb_mode.pack(side="left", padx=4)

        # Response Level
        resp_frame = ttk.Frame(pnl_tune)
        resp_frame.pack(fill="x", pady=4)
        ttk.Label(resp_frame, text="Response Level (ID 216, 1-31):").pack(side="left")
        self.x_resp_lvl = tk.IntVar(value=10)
        ttk.Spinbox(resp_frame, from_=1, to=31, textvariable=self.x_resp_lvl, width=6).pack(side="left", padx=4)
        ttk.Button(resp_frame, text="Set Level", command=lambda: self._send_cmd(f"autotune x gain {self.x_resp_lvl.get()}")).pack(side="left", padx=2)

        # Action Buttons
        ttk.Button(pnl_tune, text="Lock Manual Gains (`autotune x lock`)", command=lambda: self._send_cmd("autotune x lock")).pack(fill="x", pady=2)
        ttk.Button(pnl_tune, text="Reset to Factory Gains (`autotune x reset`)", command=lambda: self._send_cmd("autotune x reset")).pack(fill="x", pady=2)

        # Travel & Calibration Panel
        pnl_travel = ttk.LabelFrame(right, text="X Axis Motion & Calibration", padding=8)
        pnl_travel.pack(fill="both", expand=True, pady=4)

        # Jogging
        jog_f = ttk.Frame(pnl_travel)
        jog_f.pack(fill="x", pady=4)
        ttk.Label(jog_f, text="Jog X (mm):").pack(side="left")
        ttk.Button(jog_f, text="<< -10 mm", command=lambda: self._nudge_x(-10.0)).pack(side="left", padx=2)
        ttk.Button(jog_f, text="< -1 mm", command=lambda: self._nudge_x(-1.0)).pack(side="left", padx=2)
        ttk.Button(jog_f, text="+1 mm >", command=lambda: self._nudge_x(1.0)).pack(side="left", padx=2)
        ttk.Button(jog_f, text="+10 mm >>", command=lambda: self._nudge_x(10.0)).pack(side="left", padx=2)

        # Calibrate PUU
        cal_f = ttk.Frame(pnl_travel)
        cal_f.pack(fill="x", pady=6)
        ttk.Label(cal_f, text="PUU/mm Calibration Tool:").pack(anchor="w")
        c_sub = ttk.Frame(cal_f)
        c_sub.pack(fill="x", pady=2)
        ttk.Label(c_sub, text="Cmd mm:").pack(side="left")
        self.x_cmd_mm = tk.DoubleVar(value=100.0)
        ttk.Entry(c_sub, textvariable=self.x_cmd_mm, width=7).pack(side="left", padx=2)
        ttk.Label(c_sub, text="Measured:").pack(side="left", padx=2)
        self.x_meas_mm = tk.DoubleVar(value=100.0)
        ttk.Entry(c_sub, textvariable=self.x_meas_mm, width=7).pack(side="left", padx=2)
        ttk.Button(c_sub, text="Suggest PUU", command=lambda: self._send_cmd(f"puucal x {self.x_cmd_mm.get()} {self.x_meas_mm.get()}")).pack(side="left", padx=4)

        ttk.Button(pnl_travel, text="Home X Only (`home x`)", command=lambda: self._send_cmd("home x")).pack(fill="x", pady=2)
        ttk.Button(pnl_travel, text="Calibrate X Stroke (`calibrate x`)", command=lambda: self._send_cmd("calibrate x")).pack(fill="x", pady=2)

    # -------------------------------------------------------------------------
    # TAB 3: Z AXIS (KINETIX 5100)
    # -------------------------------------------------------------------------
    def _build_z_tab(self) -> None:
        tab = ttk.Frame(self.notebook, padding=8)
        self.notebook.add(tab, text="Z Axis (Kinetix 5100)")

        left = ttk.Frame(tab)
        left.pack(side="left", fill="both", expand=True, padx=4)

        right = ttk.Frame(tab)
        right.pack(side="right", fill="both", expand=True, padx=4)

        # Tuning Parameters (Class 0x0F)
        pnl_tune = ttk.LabelFrame(left, text="Kinetix 5100 CIP Tuning Parameters", padding=8)
        pnl_tune.pack(fill="both", expand=True, pady=4)

        ttk.Button(pnl_tune, text="Read All Z Parameters (`autotune z read`)", command=lambda: self._send_cmd("autotune z read")).pack(fill="x", pady=3)

        # Response Level
        resp_frame = ttk.Frame(pnl_tune)
        resp_frame.pack(fill="x", pady=4)
        ttk.Label(resp_frame, text="Response Level (ID 216, 1-31):").pack(side="left")
        self.z_resp_lvl = tk.IntVar(value=10)
        ttk.Spinbox(resp_frame, from_=1, to=31, textvariable=self.z_resp_lvl, width=6).pack(side="left", padx=4)
        ttk.Button(resp_frame, text="Set Level", command=lambda: self._send_cmd(f"autotune z gain {self.z_resp_lvl.get()}")).pack(side="left", padx=2)

        ttk.Button(pnl_tune, text="Lock Manual Gains (`autotune z lock`)", command=lambda: self._send_cmd("autotune z lock")).pack(fill="x", pady=2)
        ttk.Button(pnl_tune, text="Reset to Factory Gains (`autotune z reset`)", command=lambda: self._send_cmd("autotune z reset")).pack(fill="x", pady=2)

        # Travel & Calibration Panel
        pnl_travel = ttk.LabelFrame(right, text="Z Axis Motion & Calibration", padding=8)
        pnl_travel.pack(fill="both", expand=True, pady=4)

        # Jogging
        jog_f = ttk.Frame(pnl_travel)
        jog_f.pack(fill="x", pady=4)
        ttk.Label(jog_f, text="Jog Z (mm):").pack(side="left")
        ttk.Button(jog_f, text="<< -5 mm", command=lambda: self._nudge_z(-5.0)).pack(side="left", padx=2)
        ttk.Button(jog_f, text="< -1 mm", command=lambda: self._nudge_z(-1.0)).pack(side="left", padx=2)
        ttk.Button(jog_f, text="+1 mm >", command=lambda: self._nudge_z(1.0)).pack(side="left", padx=2)
        ttk.Button(jog_f, text="+5 mm >>", command=lambda: self._nudge_z(5.0)).pack(side="left", padx=2)

        # Calibrate PUU
        cal_f = ttk.Frame(pnl_travel)
        cal_f.pack(fill="x", pady=6)
        ttk.Label(cal_f, text="PUU/mm Calibration Tool:").pack(anchor="w")
        c_sub = ttk.Frame(cal_f)
        c_sub.pack(fill="x", pady=2)
        ttk.Label(c_sub, text="Cmd mm:").pack(side="left")
        self.z_cmd_mm = tk.DoubleVar(value=50.0)
        ttk.Entry(c_sub, textvariable=self.z_cmd_mm, width=7).pack(side="left", padx=2)
        ttk.Label(c_sub, text="Measured:").pack(side="left", padx=2)
        self.z_meas_mm = tk.DoubleVar(value=50.0)
        ttk.Entry(c_sub, textvariable=self.z_meas_mm, width=7).pack(side="left", padx=2)
        ttk.Button(c_sub, text="Suggest PUU", command=lambda: self._send_cmd(f"puucal z {self.z_cmd_mm.get()} {self.z_meas_mm.get()}")).pack(side="left", padx=4)

        ttk.Button(pnl_travel, text="Home Z Only (`home z`)", command=lambda: self._send_cmd("home z")).pack(fill="x", pady=2)
        ttk.Button(pnl_travel, text="Calibrate Z Stroke (`calibrate z`)", command=lambda: self._send_cmd("calibrate z")).pack(fill="x", pady=2)

    # -------------------------------------------------------------------------
    # TAB 4: UNIFIED BRING-UP & HOMING
    # -------------------------------------------------------------------------
    def _build_bringup_tab(self) -> None:
        tab = ttk.Frame(self.notebook, padding=12)
        self.notebook.add(tab, text="Unified Bring-up & Homing")

        desc_box = ttk.LabelFrame(tab, text="Unified EIP Bring-up Sequence", padding=12)
        desc_box.pack(fill="x", pady=6)

        seq_text = ("The unified bring-up sequence executes all 8 stages automatically:\n\n"
                    "  1. Z- Retract Datum (A015) — seek, creep, latch Z = 0.0 mm\n"
                    "  2. X- Home (A014) — seek, creep, latch X = 0.0 mm\n"
                    "  3. X+ Calibrate (A015) — seek, measure X stroke length\n"
                    "  4. X Park — move X to 35.0 mm\n"
                    "  5. Theta Orient — rotate Theta to 90.0° before vertical stroke\n"
                    "  6. Z+ Calibrate (A014) — seek, measure Z stroke length\n"
                    "  7. Z Retract — return Z to SAFE_Z clearance ceiling (35.7 mm)\n"
                    "  8. Theta Zero — capture origin and settle at physical 0.000°")
        ttk.Label(desc_box, text=seq_text, font=("Consolas", 10), justify="left").pack(anchor="w")

        btn_bringup = tk.Button(tab, text="▶ RUN UNIFIED BRING-UP (`bringup` / `home all`)",
                                bg="#5cb85c", fg="white", font=("Segoe UI", 12, "bold"),
                                command=lambda: self._send_cmd("bringup"), pady=10)
        btn_bringup.pack(fill="x", pady=12)

        btn_cycle = tk.Button(tab, text="▶ RUN 8-STAGE TEST CYCLE (`test_cycle`)",
                              bg="#337ab7", fg="white", font=("Segoe UI", 11, "bold"),
                              command=lambda: self._send_cmd("test_cycle"), pady=8)
        btn_cycle.pack(fill="x", pady=4)

    # -------------------------------------------------------------------------
    # MOTION HELPERS
    # -------------------------------------------------------------------------
    def _do_c0300(self) -> None:
        if messagebox.askyesno("Confirm Drive Zeroing",
                               "Are you sure you want to set the current physical orientation as absolute 0.000°?\n\n"
                               "This will execute C0300 and permanently save to the drive's NV flash (C2200)."):
            self._send_cmd("autotune theta zero")

    def _nudge_th(self, delta: float) -> None:
        try:
            cur_th = float(self.pos_th_var.get().replace("deg", "").strip())
        except ValueError:
            cur_th = 0.0
        new_th = round(cur_th + delta, 2)
        self._move_th_exact(new_th)

    def _move_th_exact(self, angle_deg: float) -> None:
        try:
            cur_x = float(self.pos_x_var.get().replace("mm", "").strip())
        except ValueError:
            cur_x = 35.0
        try:
            cur_z = float(self.pos_z_var.get().replace("mm", "").strip())
        except ValueError:
            cur_z = 35.7
        self._send_cmd(f"move {cur_x:.1f} {cur_z:.1f} {angle_deg:.2f}")

    def _nudge_x(self, delta: float) -> None:
        try:
            cur_x = float(self.pos_x_var.get().replace("mm", "").strip())
            cur_z = float(self.pos_z_var.get().replace("mm", "").strip())
            cur_th = float(self.pos_th_var.get().replace("deg", "").strip())
        except ValueError:
            cur_x, cur_z, cur_th = 35.0, 35.7, 0.0
        new_x = round(cur_x + delta, 1)
        self._send_cmd(f"move {new_x:.1f} {cur_z:.1f} {cur_th:.2f}")

    def _nudge_z(self, delta: float) -> None:
        try:
            cur_x = float(self.pos_x_var.get().replace("mm", "").strip())
            cur_z = float(self.pos_z_var.get().replace("mm", "").strip())
            cur_th = float(self.pos_th_var.get().replace("deg", "").strip())
        except ValueError:
            cur_x, cur_z, cur_th = 35.0, 35.7, 0.0
        new_z = round(cur_z + delta, 1)
        self._send_cmd(f"move {cur_x:.1f} {new_z:.1f} {cur_th:.2f}")

    # -------------------------------------------------------------------------
    # CONNECTION & PROTOCOL HANDLING
    # -------------------------------------------------------------------------
    def _toggle_connect(self) -> None:
        if self.client.connected:
            self.client.close()
            self.btn_connect.config(text="Connect")
            self.conn_status_var.set("Disconnected")
            self.lbl_conn.config(foreground="black")
        else:
            try:
                self.conn_status_var.set("Connecting...")
                self.client.connect(self.host_var.get().strip(), int(self.port_var.get()))
                self.btn_connect.config(text="Disconnect")
                self.conn_status_var.set("Connected")
                self.lbl_conn.config(foreground="#5cb85c")
            except Exception as e:
                messagebox.showerror("Connection Error", str(e))
                self.conn_status_var.set("Error")
                self.lbl_conn.config(foreground="red")

    def _send_cmd(self, cmd: str) -> None:
        if not self.client.connected:
            messagebox.showwarning("Not Connected", "Please connect to the WT32-ETH01 first.")
            return
        self.txt_log.insert(tk.END, f"> {cmd}\n")
        self.txt_log.see(tk.END)
        try:
            self.client.send(cmd)
        except Exception as e:
            self.txt_log.insert(tk.END, f"[ERROR] Send failed: {e}\n")

    def _send_raw_entry(self) -> None:
        cmd = self.ent_cmd.get().strip()
        if cmd:
            self._send_cmd(cmd)
            self.ent_cmd.delete(0, tk.END)

    def _process_events(self) -> None:
        while True:
            try:
                kind, payload = self.events.get_nowait()
            except queue.Empty:
                break

            if kind == "closed":
                self.conn_status_var.set("Disconnected")
                self.lbl_conn.config(foreground="black")
                self.btn_connect.config(text="Connect")
                self.txt_log.insert(tk.END, f"[SYSTEM] {payload}\n")
                self.txt_log.see(tk.END)
            elif kind == "prompt":
                if payload.endswith(PROMPT_PASSWORD):
                    self.txt_log.insert(tk.END, f"{payload}[sending password]\n")
                    self.client.send(self.pass_var.get())
                else:
                    pass
            elif kind == "line":
                clean = ANSI_RE.sub("", payload)
                self.txt_log.insert(tk.END, clean + "\n")
                self.txt_log.see(tk.END)
                self._parse_status_line(clean)

        self.after(40, self._process_events)

    def _parse_status_line(self, line: str) -> None:
        # Match X Position: 35.000 mm
        m = re.search(r"X Position:\s*([-\d.]+)\s*mm", line)
        if m:
            self.pos_x_var.set(f"{float(m.group(1)):.2f} mm")

        m = re.search(r"Z Position:\s*([-\d.]+)\s*mm", line)
        if m:
            self.pos_z_var.set(f"{float(m.group(1)):.2f} mm")

        m = re.search(r"Theta:\s*([-\d.]+)\s*deg", line)
        if m:
            self.pos_th_var.set(f"{float(m.group(1)):.2f} deg")

        m = re.search(r"Motor Enabled:\s*(Yes|No)", line, re.I)
        if m:
            self.state_enabled_var.set(m.group(1).upper())

        m = re.search(r"Busy:\s*(Yes|No)", line, re.I)
        if m:
            self.state_busy_var.set(m.group(1).upper())

        m = re.search(r"Alarm:\s*(Yes|No)", line, re.I)
        if m:
            self.state_alarm_var.set(m.group(1).upper())

    def _auto_poll(self) -> None:
        if self.client.connected:
            try:
                self.client.send("status")
            except Exception:
                pass
        self.after(1500, self._auto_poll)

    def _on_close(self) -> None:
        if self.client.connected:
            self.client.close()
        self.destroy()


def main() -> None:
    parser = argparse.ArgumentParser(description="EtherNet/IP Driver Configuration & Tuning GUI")
    parser.add_argument("--host", default=DEFAULT_HOST, help=f"WT32-ETH01 IP (default: {DEFAULT_HOST})")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help=f"TCP console port (default: {DEFAULT_PORT})")
    parser.add_argument("--password", default=DEFAULT_PASSWORD, help="Console password")
    args = parser.parse_args()

    app = DriverConfigApp(args.host, args.port, args.password)
    app.mainloop()


if __name__ == "__main__":
    main()

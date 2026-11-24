#!/usr/bin/env python3
"""direct_eip_driver_ui.py — Pure PC-Side EtherNet/IP & Direct Drive Configuration Suite.

Connects DIRECTLY from the PC to the servo drive daisy-chain on the 192.168.1.x network
(no WT32 MCU required — PC plugged directly into the drive switch / daisy-chain).

Supported Drives & Capabilities:
  1. Bosch Rexroth IndraDrive Cs HCS01 & SCHUNK ERD-04 Theta Axis (192.168.1.22 HTTP / 192.168.1.23 CIP):
     - Real-Time Class 1 EtherNet/IP Cyclic Motion & Micro-Jogging (Assembly 101/102)
     - Live Angular Positioning & Instant Feedback
     - Mechanical Zero Calibration (C0300) & NV Flash Backup (C2200)
     - Automatic Inertia Identification (C1800) & Factory Reset
     - Live I/O & Status Bit Visualization (AF Mode, Ready, In Reference, Standstill, Diag, Faults)
     - Fault Reset (C0500) & SERCOS IDN Inspector
  2. Rockwell Kinetix 5100 X & Z Linear Axes (192.168.1.20 & 192.168.1.21):
     - Real-Time Class 1 EtherNet/IP Cyclic Motion & Micro-Jogging (Assembly 104/154)
     - Live Millimetre Positioning (X: 52428.8 PUU/mm, Z: 104857.6 PUU/mm)
     - Origin Calibration / Homing (Method 34)
     - Live Hardware I/O Visualization (OT+ Forward Limit, OT- Reverse Limit, Servo Ready, Active, Homed, Fault)
     - Pure CIP Explicit Parameter Object Messaging (Class 0x0F)
     - Real-Time & Adaptive Gain Tuning (Mode ID 217, Response Level ID 216, Inertia Ratio ID 144, Kp ID 185, Kv ID 189)

Requirements:
  PC Ethernet adapter configured on static IP 192.168.1.10 (Subnet: 255.255.255.0).
  Standard Python library only (tkinter, socket, struct, threading, urllib).
"""

from __future__ import annotations

import argparse
import datetime
import os
import queue
import socket
import struct
import sys
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, scrolledtext, ttk
from typing import Dict, Optional, Any

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

import eip_test as eip
from hcs01_comws import Hcs01Comws, ComwsError, normalize_idn, parse_status_word
from hcs01_cyclic import Hcs01CyclicDriver, PUU_PER_DEG
from k5100_cyclic import (
    K5100CyclicDriver,
    X_PUU_PER_MM,
    Z_PUU_PER_MM,
    X_LEAD_MM,
    X_RATIO,
    Z_LEAD_MM,
    Z_RATIO,
    CONVEYOR_COLLISION_X_MIN_MM,
    CONVEYOR_COLLISION_Z_MIN_MM,
)

# Default Drive IPs on the EtherNet/IP private subnet
DEFAULT_X_IP = "192.168.1.20"
DEFAULT_Z_IP = "192.168.1.21"
DEFAULT_THETA_ENG_IP = "192.168.1.22"
DEFAULT_THETA_CIP_IP = "192.168.1.23"
EIP_PORT = 44818

# Kinetix 5100 CIP Parameter IDs (Class 0x0F)
PARAM_GAIN_ADJUST_MODE = 217
PARAM_RESPONSE_LEVEL = 216
PARAM_LOAD_INERTIA_RATIO = 144
PARAM_POS_PROP_GAIN = 185
PARAM_VEL_PROP_GAIN = 189
PARAM_VEL_INT_GAIN = 191
PARAM_TOTAL_INERTIA = 659


class DirectEipDriverApp(tk.Tk):
    """Main Application Window for Direct PC-to-Drive Configuration."""

    def __init__(self) -> None:
        super().__init__()
        self.title("Pure EtherNet/IP & Direct Drive Configurator (PC Only) — WT32-ETH01 Gantry")
        self.geometry("1240x860")
        self.minsize(1020, 720)

        self.ip_x_var = tk.StringVar(value=DEFAULT_X_IP)
        self.ip_z_var = tk.StringVar(value=DEFAULT_Z_IP)
        self.ip_th_eng_var = tk.StringVar(value=DEFAULT_THETA_ENG_IP)
        self.ip_th_cip_var = tk.StringVar(value=DEFAULT_THETA_CIP_IP)

        # Top live status telemetry
        self.x_pos_var = tk.StringVar(value="0.000 mm")
        self.z_pos_var = tk.StringVar(value="0.000 mm")
        self.th_pos_var = tk.StringVar(value="0.000 deg")

        self.x_status_var = tk.StringVar(value="Offline")
        self.z_status_var = tk.StringVar(value="Offline")
        self.th_status_var = tk.StringVar(value="Offline")
        self.th_diag_var = tk.StringVar(value="--")

        # Driver Motion Targets
        self.x_target_mm = tk.DoubleVar(value=0.0)
        self.z_target_mm = tk.DoubleVar(value=0.0)
        self.th_target_deg = tk.DoubleVar(value=0.0)

        # Motion arm buttons
        self.th_arm_btn_var = tk.StringVar(value="▶ START LIVE EIP MOTION (ARM THETA)")
        self.x_arm_btn_var = tk.StringVar(value="▶ START LIVE EIP MOTION (SERVO ON X)")
        self.z_arm_btn_var = tk.StringVar(value="▶ START LIVE EIP MOTION (SERVO ON Z)")

        # Conveyor Collision Guard (Jaws collision: X >= 95mm and Z >= 115mm)
        self.collision_guard_enabled = tk.BooleanVar(value=True)
        self.collision_x_limit = tk.DoubleVar(value=CONVEYOR_COLLISION_X_MIN_MM)
        self.collision_z_limit = tk.DoubleVar(value=CONVEYOR_COLLISION_Z_MIN_MM)
        self.collision_status_var = tk.StringVar(value="SAFE (CLEAR)")

        # Logging & drivers
        self.log_q: queue.Queue[str] = queue.Queue()
        self.hcs01_cli: Hcs01Comws | None = None
        self.th_driver: Hcs01CyclicDriver | None = None
        self.x_driver: K5100CyclicDriver | None = None
        self.z_driver: K5100CyclicDriver | None = None

        # Visual indicator widgets dict
        self.led_widgets: Dict[str, tk.Label] = {}

        self._build_ui()
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.after(40, self._process_logs)
        self.after(100, self._telemetry_poll)

    def _build_ui(self) -> None:
        # Top Subnet & IP Discovery Strip
        top = ttk.LabelFrame(self, text="Direct EtherNet/IP Drive Network (PC on 192.168.1.10)", padding=6)
        top.pack(fill="x", padx=8, pady=4)

        ttk.Label(top, text="Theta HCS01:").grid(row=0, column=0, padx=2)
        ttk.Entry(top, textvariable=self.ip_th_cip_var, width=13).grid(row=0, column=1, padx=2)

        ttk.Label(top, text="X Axis (K5100):").grid(row=0, column=2, padx=4)
        ttk.Entry(top, textvariable=self.ip_x_var, width=13).grid(row=0, column=3, padx=2)

        ttk.Label(top, text="Z Axis (K5100):").grid(row=0, column=4, padx=4)
        ttk.Entry(top, textvariable=self.ip_z_var, width=13).grid(row=0, column=5, padx=2)

        ttk.Button(top, text="Probe All Endpoints", command=self._ping_all_drives).grid(row=0, column=6, padx=8)

        # Status Strip
        stat_bar = ttk.Frame(self, padding=4)
        stat_bar.pack(fill="x", padx=8, pady=2)

        def add_box(parent, title, var, width=16):
            f = ttk.LabelFrame(parent, text=title, padding=3)
            f.pack(side="left", fill="both", expand=True, padx=3)
            ttk.Label(f, textvariable=var, font=("Consolas", 11, "bold"), anchor="center", width=width).pack(fill="both")
            return f

        add_box(stat_bar, "X Position (Actual)", self.x_pos_var)
        add_box(stat_bar, "Z Position (Actual)", self.z_pos_var)
        add_box(stat_bar, "Theta Position (Actual)", self.th_pos_var)
        add_box(stat_bar, "Theta Diag Message", self.th_diag_var, width=28)

        # Conveyor Collision Safety Strip (Precompile editable limits for gripper jaws)
        guard_bar = ttk.LabelFrame(self, text="Conveyor Collision Safety Guard (Current Gripper Jaws)", padding=4)
        guard_bar.pack(fill="x", padx=8, pady=2)

        ttk.Checkbutton(guard_bar, text="Enforce Collision Guard", variable=self.collision_guard_enabled).pack(side="left", padx=6)
        ttk.Label(guard_bar, text="Collision Hazard Zone:  X ≥").pack(side="left", padx=2)
        ttk.Entry(guard_bar, textvariable=self.collision_x_limit, width=6).pack(side="left")
        ttk.Label(guard_bar, text="mm  AND  Z ≥").pack(side="left", padx=2)
        ttk.Entry(guard_bar, textvariable=self.collision_z_limit, width=6).pack(side="left")
        ttk.Label(guard_bar, text="mm").pack(side="left", padx=2)

        self.lbl_guard_status = tk.Label(
            guard_bar,
            textvariable=self.collision_status_var,
            font=("Segoe UI", 9, "bold"),
            bg="#5cb85c",
            fg="white",
            padx=10,
            pady=1,
        )
        self.lbl_guard_status.pack(side="right", padx=6)

        # Main Tabs
        self.notebook = ttk.Notebook(self)
        self.notebook.pack(fill="both", expand=True, padx=8, pady=4)

        self._build_theta_tab()
        self._build_kinetix_tab("X Axis (Kinetix 5100)", self.ip_x_var, is_x=True)
        self._build_kinetix_tab("Z Axis (Kinetix 5100)", self.ip_z_var, is_x=False)
        self._build_system_overview_tab()

        # Bottom Log Window
        bot = ttk.LabelFrame(self, text="Real-Time EtherNet/IP & Drive Telemetry Log", padding=4)
        bot.pack(fill="both", expand=False, padx=8, pady=4)
        bot.config(height=160)

        self.txt_log = scrolledtext.ScrolledText(bot, height=7, font=("Consolas", 9), bg="#1e1e1e", fg="#d4d4d4")
        self.txt_log.pack(fill="both", expand=True)

    # -------------------------------------------------------------------------
    # TAB 1: THETA AXIS (REXROTH HCS01 & ERD-04)
    # -------------------------------------------------------------------------
    def _build_theta_tab(self) -> None:
        tab = ttk.Frame(self.notebook, padding=8)
        self.notebook.add(tab, text="Theta Axis (SCHUNK / Rexroth HCS01)")

        left = ttk.Frame(tab)
        left.pack(side="left", fill="both", expand=True, padx=4)

        right = ttk.Frame(tab)
        right.pack(side="right", fill="both", expand=True, padx=4)

        # Panel: Real-Time Motion & Jogging
        pnl_motion = ttk.LabelFrame(left, text="Real-Time EtherNet/IP Motion & Micro-Jogging", padding=8)
        pnl_motion.pack(fill="x", pady=4)

        self.btn_live_th = tk.Button(
            pnl_motion,
            textvariable=self.th_arm_btn_var,
            bg="#5cb85c",
            fg="white",
            font=("Segoe UI", 10, "bold"),
            command=self._toggle_live_th,
            pady=4,
        )
        self.btn_live_th.pack(fill="x", pady=3)

        ttk.Label(pnl_motion, text="Micro-Jog Theta in fine angular increments to align & square:").pack(anchor="w", pady=2)

        # Jog Step Buttons
        jog_f = ttk.Frame(pnl_motion)
        jog_f.pack(fill="x", pady=4)

        ttk.Button(jog_f, text="<< -5.0°", command=lambda: self._nudge_theta(-5.0)).pack(side="left", expand=True, fill="x", padx=1)
        ttk.Button(jog_f, text="< -1.0°", command=lambda: self._nudge_theta(-1.0)).pack(side="left", expand=True, fill="x", padx=1)
        ttk.Button(jog_f, text="-0.1°", command=lambda: self._nudge_theta(-0.1)).pack(side="left", expand=True, fill="x", padx=1)
        ttk.Button(jog_f, text="+0.1°", command=lambda: self._nudge_theta(0.1)).pack(side="left", expand=True, fill="x", padx=1)
        ttk.Button(jog_f, text="+1.0° >", command=lambda: self._nudge_theta(1.0)).pack(side="left", expand=True, fill="x", padx=1)
        ttk.Button(jog_f, text="+5.0° >>", command=lambda: self._nudge_theta(5.0)).pack(side="left", expand=True, fill="x", padx=1)

        # Move to Absolute Target
        abs_f = ttk.Frame(pnl_motion)
        abs_f.pack(fill="x", pady=4)
        ttk.Label(abs_f, text="Move to Exact Angle:").pack(side="left")
        ttk.Entry(abs_f, textvariable=self.th_target_deg, width=8).pack(side="left", padx=3)
        ttk.Label(abs_f, text="deg").pack(side="left")
        ttk.Button(abs_f, text="Move Theta", command=lambda: self._move_theta_abs(self.th_target_deg.get())).pack(side="left", padx=6)

        # Panel: Visualized Drive I/O & Status Flags
        pnl_io = ttk.LabelFrame(left, text="Theta Drive Status & Fieldbus I/O Indicators", padding=8)
        pnl_io.pack(fill="x", pady=4)

        io_grid = ttk.Frame(pnl_io)
        io_grid.pack(fill="x", pady=2)

        self._create_led_badge(io_grid, "th_af", "AF Mode (Torque Active)", 0, 0)
        self._create_led_badge(io_grid, "th_ready", "In Operation (Ready=3)", 0, 1)
        self._create_led_badge(io_grid, "th_homed", "In Reference (Homed)", 0, 2)
        self._create_led_badge(io_grid, "th_standstill", "In Standstill", 1, 0)
        self._create_led_badge(io_grid, "th_reached", "Target Angle Reached", 1, 1)
        self._create_led_badge(io_grid, "th_fault", "Class 1 Fault", 1, 2)

        # Panel: Mechanical Zero Calibration (C0300 & C2200)
        pnl_zero = ttk.LabelFrame(left, text="Mechanical Zero Calibration (C0300 & C2200)", padding=8)
        pnl_zero.pack(fill="x", pady=4)

        btn_zero = tk.Button(
            pnl_zero,
            text="★ SET MECHANICAL ZERO (C0300) & SAVE NV (C2200) ★",
            bg="#0275d8",
            fg="white",
            font=("Segoe UI", 9, "bold"),
            command=self._do_hcs01_c0300,
            pady=4,
        )
        btn_zero.pack(fill="x", pady=2)

        # Panel: Inertia Auto-Tuning & Reset
        pnl_tune = ttk.LabelFrame(left, text="Inertia Auto-Tuning & Parameter Restore", padding=8)
        pnl_tune.pack(fill="x", pady=4)

        btn_c1800 = tk.Button(
            pnl_tune,
            text="Run C1800 Identification Sweep (±30°)",
            bg="#f0ad4e",
            fg="white",
            font=("Segoe UI", 9, "bold"),
            command=self._do_hcs01_c1800,
            pady=3,
        )
        btn_c1800.pack(fill="x", pady=2)

        btn_reset_tune = ttk.Button(
            pnl_tune,
            text="↺ Reset Factory Tuning (Kp=0.030, Tn=10.0, Kv=1.00)",
            command=self._do_hcs01_factory_reset,
        )
        btn_reset_tune.pack(fill="x", pady=2)

        # Right Panel: SERCOS IDN Inspector & Fault Clearing
        pnl_idn = ttk.LabelFrame(right, text="SERCOS IDN Parameter Access & Faults", padding=8)
        pnl_idn.pack(fill="both", expand=True, pady=4)

        btn_c0500 = ttk.Button(pnl_idn, text="Clear Faults / Alarms (C0500)", command=self._do_hcs01_c0500)
        btn_c0500.pack(fill="x", pady=2)

        # Read IDN Frame
        r_f = ttk.Frame(pnl_idn)
        r_f.pack(fill="x", pady=6)
        ttk.Label(r_f, text="Read IDN:").pack(side="left")
        self.idn_read_var = tk.StringVar(value="S-0-0051")
        ttk.Entry(r_f, textvariable=self.idn_read_var, width=12).pack(side="left", padx=2)
        ttk.Button(r_f, text="Read", command=self._read_idn).pack(side="left", padx=2)
        self.idn_val_var = tk.StringVar(value="")
        ttk.Label(r_f, textvariable=self.idn_val_var, font=("Consolas", 9, "bold")).pack(side="left", padx=4)

        # Write IDN Frame
        w_f = ttk.Frame(pnl_idn)
        w_f.pack(fill="x", pady=6)
        ttk.Label(w_f, text="Write IDN:").grid(row=0, column=0, sticky="w")
        self.idn_write_name = tk.StringVar(value="S-0-0138")
        self.idn_write_val = tk.StringVar(value="1000.000")
        ttk.Entry(w_f, textvariable=self.idn_write_name, width=12).grid(row=0, column=1, padx=2)
        ttk.Label(w_f, text="Val:").grid(row=0, column=2, padx=2)
        ttk.Entry(w_f, textvariable=self.idn_write_val, width=12).grid(row=0, column=3, padx=2)
        ttk.Button(w_f, text="Write", command=self._write_idn).grid(row=0, column=4, padx=4)

    # -------------------------------------------------------------------------
    # TAB 2 & 3: KINETIX 5100 AXIS (X & Z) WITH JOGGING & I/O
    # -------------------------------------------------------------------------
    def _build_kinetix_tab(self, title: str, ip_var: tk.StringVar, is_x: bool) -> None:
        tab = ttk.Frame(self.notebook, padding=8)
        self.notebook.add(tab, text=title)

        left = ttk.Frame(tab)
        left.pack(side="left", fill="both", expand=True, padx=4)

        right = ttk.Frame(tab)
        right.pack(side="right", fill="both", expand=True, padx=4)

        prefix = "x" if is_x else "z"
        arm_var = self.x_arm_btn_var if is_x else self.z_arm_btn_var
        target_var = self.x_target_mm if is_x else self.z_target_mm

        # Panel: Real-Time Motion & Jogging
        pnl_motion = ttk.LabelFrame(left, text=f"{title} — Real-Time Motion & Jogging", padding=8)
        pnl_motion.pack(fill="x", pady=4)

        btn_arm = tk.Button(
            pnl_motion,
            textvariable=arm_var,
            bg="#5cb85c",
            fg="white",
            font=("Segoe UI", 10, "bold"),
            command=lambda: self._toggle_live_k5100(is_x),
            pady=4,
        )
        btn_arm.pack(fill="x", pady=3)
        if is_x:
            self.btn_live_x = btn_arm
        else:
            self.btn_live_z = btn_arm

        ttk.Label(pnl_motion, text="Micro-Jog axis in fine millimetre steps:").pack(anchor="w", pady=2)

        # Jog Step Buttons
        jog_f = ttk.Frame(pnl_motion)
        jog_f.pack(fill="x", pady=4)

        ttk.Button(jog_f, text="<< -10mm", command=lambda: self._nudge_k5100(is_x, -10.0)).pack(side="left", expand=True, fill="x", padx=1)
        ttk.Button(jog_f, text="< -1.0mm", command=lambda: self._nudge_k5100(is_x, -1.0)).pack(side="left", expand=True, fill="x", padx=1)
        ttk.Button(jog_f, text="-0.1mm", command=lambda: self._nudge_k5100(is_x, -0.1)).pack(side="left", expand=True, fill="x", padx=1)
        ttk.Button(jog_f, text="+0.1mm", command=lambda: self._nudge_k5100(is_x, 0.1)).pack(side="left", expand=True, fill="x", padx=1)
        ttk.Button(jog_f, text="+1.0mm >", command=lambda: self._nudge_k5100(is_x, 1.0)).pack(side="left", expand=True, fill="x", padx=1)
        ttk.Button(jog_f, text="+10mm >>", command=lambda: self._nudge_k5100(is_x, 10.0)).pack(side="left", expand=True, fill="x", padx=1)

        # Move to Absolute Target mm
        abs_f = ttk.Frame(pnl_motion)
        abs_f.pack(fill="x", pady=4)
        ttk.Label(abs_f, text="Move to Exact Target:").pack(side="left")
        ttk.Entry(abs_f, textvariable=target_var, width=8).pack(side="left", padx=3)
        ttk.Label(abs_f, text="mm").pack(side="left")
        ttk.Button(abs_f, text=f"Move {title[:1]} Axis", command=lambda: self._move_k5100_abs(is_x, target_var.get())).pack(side="left", padx=6)

        # Homing & Fault Action Strip
        act_f = ttk.Frame(pnl_motion)
        act_f.pack(fill="x", pady=4)
        ttk.Button(act_f, text="★ Set Origin Zero (Method 34)", command=lambda: self._home_k5100(is_x)).pack(side="left", expand=True, fill="x", padx=2)
        ttk.Button(act_f, text="Clear Faults / Reset Alarm", command=lambda: self._clear_k5100_fault(is_x)).pack(side="left", expand=True, fill="x", padx=2)

        # Panel: Visualized Hardware I/O & Drive Status Flags
        pnl_io = ttk.LabelFrame(left, text=f"{title} — Live Hardware I/O & Status Flags", padding=8)
        pnl_io.pack(fill="x", pady=4)

        io_grid = ttk.Frame(pnl_io)
        io_grid.pack(fill="x", pady=2)

        self._create_led_badge(io_grid, f"{prefix}_ot_plus", "OT+ Forward Limit (DI1)", 0, 0)
        self._create_led_badge(io_grid, f"{prefix}_ot_minus", "OT- Reverse Limit (DI2)", 0, 1)
        self._create_led_badge(io_grid, f"{prefix}_ready", "Servo Ready (SRDY)", 0, 2)
        self._create_led_badge(io_grid, f"{prefix}_active", "Servo Active (SON)", 1, 0)
        self._create_led_badge(io_grid, f"{prefix}_at_ref", "At Target (TPOS)", 1, 1)
        self._create_led_badge(io_grid, f"{prefix}_stopped", "Standstill (ZSP)", 1, 2)
        self._create_led_badge(io_grid, f"{prefix}_homed", "Homed Origin Set", 2, 0)
        self._create_led_badge(io_grid, f"{prefix}_fault", "Drive Alarm / Fault", 2, 1)
        self._create_led_badge(io_grid, f"{prefix}_warn", "Warning Present", 2, 2)

        # Right Panel: CIP Parameter Object Configuration
        pnl_cip = ttk.LabelFrame(right, text=f"CIP Class 0x0F Tuning Parameters — {title}", padding=8)
        pnl_cip.pack(fill="both", expand=True, pady=4)

        # Gain Adjustment Mode (ID 217)
        f_mode = ttk.Frame(pnl_cip)
        f_mode.pack(fill="x", pady=3)
        ttk.Label(f_mode, text="Gain Mode (ID 217):", width=22, anchor="w").pack(side="left")
        mode_var = tk.IntVar(value=0)
        ttk.Entry(f_mode, textvariable=mode_var, width=8).pack(side="left", padx=2)
        ttk.Button(f_mode, text="Write", command=lambda: self._set_cip_param(ip_var.get(), PARAM_GAIN_ADJUST_MODE, mode_var.get(), 2)).pack(side="left", padx=2)
        ttk.Button(f_mode, text="Read", command=lambda: self._get_cip_param(ip_var.get(), PARAM_GAIN_ADJUST_MODE)).pack(side="left", padx=2)

        # Response Level (ID 216)
        f_resp = ttk.Frame(pnl_cip)
        f_resp.pack(fill="x", pady=3)
        ttk.Label(f_resp, text="Response Level (ID 216):", width=22, anchor="w").pack(side="left")
        resp_var = tk.IntVar(value=19)
        ttk.Entry(f_resp, textvariable=resp_var, width=8).pack(side="left", padx=2)
        ttk.Button(f_resp, text="Write", command=lambda: self._set_cip_param(ip_var.get(), PARAM_RESPONSE_LEVEL, resp_var.get(), 2)).pack(side="left", padx=2)
        ttk.Button(f_resp, text="Read", command=lambda: self._get_cip_param(ip_var.get(), PARAM_RESPONSE_LEVEL)).pack(side="left", padx=2)

        # Load Inertia Ratio (ID 144)
        f_inr = ttk.Frame(pnl_cip)
        f_inr.pack(fill="x", pady=3)
        ttk.Label(f_inr, text="Inertia Ratio (ID 144):", width=22, anchor="w").pack(side="left")
        inr_var = tk.IntVar(value=50)
        ttk.Entry(f_inr, textvariable=inr_var, width=8).pack(side="left", padx=2)
        ttk.Button(f_inr, text="Write", command=lambda: self._set_cip_param(ip_var.get(), PARAM_LOAD_INERTIA_RATIO, inr_var.get(), 2)).pack(side="left", padx=2)
        ttk.Button(f_inr, text="Read", command=lambda: self._get_cip_param(ip_var.get(), PARAM_LOAD_INERTIA_RATIO)).pack(side="left", padx=2)

        # Position Proportional Gain (ID 185)
        f_kp = ttk.Frame(pnl_cip)
        f_kp.pack(fill="x", pady=3)
        ttk.Label(f_kp, text="Position Gain Kp (ID 185):", width=22, anchor="w").pack(side="left")
        kp_var = tk.IntVar(value=36)
        ttk.Entry(f_kp, textvariable=kp_var, width=8).pack(side="left", padx=2)
        ttk.Button(f_kp, text="Write", command=lambda: self._set_cip_param(ip_var.get(), PARAM_POS_PROP_GAIN, kp_var.get(), 2)).pack(side="left", padx=2)
        ttk.Button(f_kp, text="Read", command=lambda: self._get_cip_param(ip_var.get(), PARAM_POS_PROP_GAIN)).pack(side="left", padx=2)

        # Velocity Proportional Gain (ID 189)
        f_kv = ttk.Frame(pnl_cip)
        f_kv.pack(fill="x", pady=3)
        ttk.Label(f_kv, text="Velocity Gain Kv (ID 189):", width=22, anchor="w").pack(side="left")
        kv_var = tk.IntVar(value=140)
        ttk.Entry(f_kv, textvariable=kv_var, width=8).pack(side="left", padx=2)
        ttk.Button(f_kv, text="Write", command=lambda: self._set_cip_param(ip_var.get(), PARAM_VEL_PROP_GAIN, kv_var.get(), 2)).pack(side="left", padx=2)
        ttk.Button(f_kv, text="Read", command=lambda: self._get_cip_param(ip_var.get(), PARAM_VEL_PROP_GAIN)).pack(side="left", padx=2)

        # Quick Action Buttons
        act_f2 = ttk.Frame(pnl_cip)
        act_f2.pack(fill="x", pady=8)
        ttk.Button(act_f2, text="Read All Drive Parameters", command=lambda: self._read_all_kinetix(ip_var.get())).pack(fill="x", pady=2)
        ttk.Button(act_f2, text="Lock Manual Gains (GainMode=0)", command=lambda: self._set_cip_param(ip_var.get(), PARAM_GAIN_ADJUST_MODE, 0, 2)).pack(fill="x", pady=2)
        ttk.Button(act_f2, text="Reset Factory Tuning (GainMode=4)", command=lambda: self._set_cip_param(ip_var.get(), PARAM_GAIN_ADJUST_MODE, 4, 2)).pack(fill="x", pady=2)

    # -------------------------------------------------------------------------
    # TAB 4: FULL SYSTEM MOTION & LIVE I/O DASHBOARD
    # -------------------------------------------------------------------------
    def _build_system_overview_tab(self) -> None:
        tab = ttk.Frame(self.notebook, padding=8)
        self.notebook.add(tab, text="Full System Motion & I/O Overview")

        top_f = ttk.LabelFrame(tab, text="Synchronized System Coordinates & Drive States", padding=8)
        top_f.pack(fill="x", pady=4)

        row_f = ttk.Frame(top_f)
        row_f.pack(fill="x", pady=4)

        # Axis Summary Cards
        def make_axis_card(parent, title, pos_var, status_var):
            card = ttk.LabelFrame(parent, text=title, padding=6)
            card.pack(side="left", fill="both", expand=True, padx=4)
            ttk.Label(card, textvariable=pos_var, font=("Consolas", 14, "bold"), foreground="#0275d8", anchor="center").pack(fill="x", pady=2)
            ttk.Label(card, textvariable=status_var, font=("Segoe UI", 9), anchor="center").pack(fill="x")

        make_axis_card(row_f, "X Axis (Linear)", self.x_pos_var, self.x_status_var)
        make_axis_card(row_f, "Z Axis (Vertical)", self.z_pos_var, self.z_status_var)
        make_axis_card(row_f, "Theta Axis (Rotary)", self.th_pos_var, self.th_status_var)

        # Global Actions
        btn_bar = ttk.Frame(tab)
        btn_bar.pack(fill="x", pady=6)
        tk.Button(
            btn_bar,
            text="★ CLEAR ALL DRIVE FAULTS ★",
            bg="#f0ad4e",
            fg="white",
            font=("Segoe UI", 9, "bold"),
            command=self._global_fault_clear,
            pady=4,
        ).pack(side="left", expand=True, fill="x", padx=3)

        tk.Button(
            btn_bar,
            text="■ DISARM ALL SERVO MOTORS ■",
            bg="#d9534f",
            fg="white",
            font=("Segoe UI", 9, "bold"),
            command=self._global_disarm,
            pady=4,
        ).pack(side="left", expand=True, fill="x", padx=3)

        # Complete I/O Matrix Frame
        mat_f = ttk.LabelFrame(tab, text="Central Limit Switch & Hardware Interlock Monitor", padding=8)
        mat_f.pack(fill="both", expand=True, pady=4)

        mat_grid = ttk.Frame(mat_f)
        mat_grid.pack(fill="both", expand=True, pady=4)

        self._create_led_badge(mat_grid, "ov_x_ot_plus", "X OT+ Forward Limit (DI1)", 0, 0)
        self._create_led_badge(mat_grid, "ov_x_ot_minus", "X OT- Reverse Limit (DI2)", 0, 1)
        self._create_led_badge(mat_grid, "ov_x_ready", "X Servo Ready", 0, 2)

        self._create_led_badge(mat_grid, "ov_z_ot_plus", "Z OT+ Upper Limit (DI1)", 1, 0)
        self._create_led_badge(mat_grid, "ov_z_ot_minus", "Z OT- Lower Limit (DI2)", 1, 1)
        self._create_led_badge(mat_grid, "ov_z_ready", "Z Servo Ready", 1, 2)

        self._create_led_badge(mat_grid, "ov_th_af", "Theta AF Torque Armed", 2, 0)
        self._create_led_badge(mat_grid, "ov_th_homed", "Theta Origin Referenced", 2, 1)
        self._create_led_badge(mat_grid, "ov_th_ready", "Theta In Operation", 2, 2)

    def _create_led_badge(self, parent: ttk.Frame, key: str, label: str, row: int, col: int) -> None:
        f = ttk.Frame(parent, padding=2)
        f.grid(row=row, column=col, sticky="w", padx=6, pady=3)

        led = tk.Label(f, text="●", font=("Segoe UI", 12), fg="#999999")
        led.pack(side="left")
        ttk.Label(f, text=label, font=("Segoe UI", 9)).pack(side="left", padx=3)
        self.led_widgets[key] = led

    def _update_led(self, key: str, active: bool, color: str = "#5cb85c") -> None:
        if key in self.led_widgets:
            self.led_widgets[key].config(fg=color if active else "#999999")

    # -------------------------------------------------------------------------
    # REAL-TIME THETA MOTION ACTIONS
    # -------------------------------------------------------------------------
    def _toggle_live_th(self) -> None:
        if self.th_driver and self.th_driver.is_active:
            self._log("[THETA] Disarming motor and stopping Class 1 stream...")
            self.th_driver.disable_motor()
            self.th_driver.close()
            self.th_driver = None
            self.th_status_var.set("Offline (Free)")
            self.th_arm_btn_var.set("▶ START LIVE EIP MOTION (ARM THETA)")
            self.btn_live_th.config(bg="#5cb85c")
        else:
            def task():
                self._log("[THETA] Connecting Real-Time Class 1 to Rexroth HCS01 (192.168.1.23)...")
                drv = Hcs01CyclicDriver(self.ip_th_cip_var.get().strip())
                if drv.connect():
                    self._log("[THETA] Class 1 Connected! Arming AF mode...")
                    drv.enable_motor()
                    self.th_driver = drv
                    self.th_status_var.set("Active (AF Armed)")
                    self.th_arm_btn_var.set("■ STOP LIVE MOTION (DISARM THETA)")
                    self.btn_live_th.config(bg="#d9534f")
                    self._log("[THETA] Motor is ARMED and ready for live jogging!")
                else:
                    self._log("[THETA ERROR] Failed to connect Class 1 to HCS01 (192.168.1.23)")
                    messagebox.showerror("Connection Error", "Failed to connect Class 1 to HCS01 (192.168.1.23)")

            threading.Thread(target=task, daemon=True).start()

    def _nudge_theta(self, delta: float) -> None:
        if self.th_driver and self.th_driver.is_active:
            if (self.th_driver.status_word >> 14) & 3 != 3:
                self.th_driver.enable_motor()
            self._log(f"[THETA] Nudging {delta:+.2f}° (Current: {self.th_driver.actual_pos_deg:.3f}°)")
            self.th_driver.nudge_deg(delta)
        else:
            messagebox.showwarning("Motor Not Armed", "Please click 'START LIVE EIP MOTION (ARM THETA)' first.")

    def _move_theta_abs(self, target: float) -> None:
        if self.th_driver and self.th_driver.is_active:
            if (self.th_driver.status_word >> 14) & 3 != 3:
                self.th_driver.enable_motor()
            self._log(f"[THETA] Moving to target {target:.3f}°")
            self.th_driver.move_to_deg(target)
        else:
            messagebox.showwarning("Motor Not Armed", "Please click 'START LIVE EIP MOTION (ARM THETA)' first.")

    # -------------------------------------------------------------------------
    # REAL-TIME KINETIX 5100 MOTION ACTIONS (X & Z)
    # -------------------------------------------------------------------------
    def _toggle_live_k5100(self, is_x: bool) -> None:
        name = "X Axis" if is_x else "Z Axis"
        drv = self.x_driver if is_x else self.z_driver
        ip = self.ip_x_var.get().strip() if is_x else self.ip_z_var.get().strip()
        puu_scale = X_PUU_PER_MM if is_x else Z_PUU_PER_MM
        lead = X_LEAD_MM if is_x else Z_LEAD_MM
        ratio = X_RATIO if is_x else Z_RATIO
        arm_var = self.x_arm_btn_var if is_x else self.z_arm_btn_var
        stat_var = self.x_status_var if is_x else self.z_status_var
        btn = self.btn_live_x if is_x else self.btn_live_z

        if drv and drv.is_active:
            self._log(f"[{name}] Disarming servo and closing Class 1 stream...")
            drv.disable_servo()
            drv.close()
            if is_x:
                self.x_driver = None
            else:
                self.z_driver = None
            stat_var.set("Offline (Free)")
            arm_var.set(f"▶ START LIVE EIP MOTION (SERVO ON {name[:1]})")
            btn.config(bg="#5cb85c")
        else:
            def task():
                self._log(f"[{name}] Connecting Class 1 ForwardOpen to {ip}...")
                new_drv = K5100CyclicDriver(ip, puu_per_mm=puu_scale, lead_mm=lead, ratio=ratio)
                if new_drv.connect():
                    self._log(f"[{name}] Class 1 Connected! Enabling servo...")
                    new_drv.enable_servo()
                    if is_x:
                        self.x_driver = new_drv
                    else:
                        self.z_driver = new_drv
                    stat_var.set("Active (Servo ON)")
                    arm_var.set(f"■ STOP LIVE MOTION (SERVO OFF {name[:1]})")
                    btn.config(bg="#d9534f")
                    self._log(f"[{name}] Servo is ON and ready for live motion!")
                else:
                    self._log(f"[{name} ERROR] ForwardOpen failed for {ip}")
                    messagebox.showerror("Connection Error", f"Failed to connect Class 1 to {name} ({ip})")

            threading.Thread(target=task, daemon=True).start()

    def _check_collision(self, is_x: bool, target_mm: float) -> bool:
        """Returns True if move is allowed, False if blocked by conveyor collision guard."""
        if not self.collision_guard_enabled.get():
            return True

        lim_x = self.collision_x_limit.get()
        lim_z = self.collision_z_limit.get()

        cur_x = self.x_driver.actual_pos_mm if (self.x_driver and self.x_driver.is_active) else 0.0
        cur_z = self.z_driver.actual_pos_mm if (self.z_driver and self.z_driver.is_active) else 0.0

        cand_x = target_mm if is_x else cur_x
        cand_z = target_mm if not is_x else cur_z

        if cand_x >= lim_x and cand_z >= lim_z:
            axis_name = "X" if is_x else "Z"
            err_msg = (
                f"⚠ CONVEYOR COLLISION BLOCKED!\n\n"
                f"Moving {axis_name} to {target_mm:.2f} mm would place the gantry at:\n"
                f"  X = {cand_x:.2f} mm (Collision Limit: ≥ {lim_x:.1f} mm)\n"
                f"  Z = {cand_z:.2f} mm (Collision Limit: ≥ {lim_z:.1f} mm)\n\n"
                f"With the current gripper jaws, this causes a physical collision with the conveyor belt/frame!\n\n"
                f"Motion command has been rejected for mechanical safety."
            )
            self._log(f"[COLLISION GUARD BLOCKED] Rejected {axis_name} move to {target_mm:.2f} mm (Would reach X={cand_x:.2f} ≥ {lim_x:.1f}, Z={cand_z:.2f} ≥ {lim_z:.1f})")
            messagebox.showerror("Conveyor Collision Guard", err_msg)
            return False

        return True

    def _nudge_k5100(self, is_x: bool, delta_mm: float) -> None:
        name = "X" if is_x else "Z"
        drv = self.x_driver if is_x else self.z_driver
        if drv and drv.is_active:
            target_mm = drv.actual_pos_mm + delta_mm
            if not self._check_collision(is_x, target_mm):
                return
            self._log(f"[{name} AXIS] Nudging {delta_mm:+.2f} mm (Target: {target_mm:.3f} mm)")
            drv.nudge_mm(delta_mm)
        else:
            messagebox.showwarning("Servo Not Armed", f"Please click 'START LIVE EIP MOTION (SERVO ON {name})' first.")

    def _move_k5100_abs(self, is_x: bool, target_mm: float) -> None:
        name = "X" if is_x else "Z"
        drv = self.x_driver if is_x else self.z_driver
        if drv and drv.is_active:
            if not self._check_collision(is_x, target_mm):
                return
            self._log(f"[{name} AXIS] Moving to absolute target {target_mm:.3f} mm")
            drv.move_to_mm(target_mm)
        else:
            messagebox.showwarning("Servo Not Armed", f"Please click 'START LIVE EIP MOTION (SERVO ON {name})' first.")

    def _home_k5100(self, is_x: bool) -> None:
        name = "X" if is_x else "Z"
        drv = self.x_driver if is_x else self.z_driver
        if drv and drv.is_active:
            self._log(f"[{name} AXIS] Setting Origin Zero (Method 34)...")
            drv.home_current_position()
            self._log(f"[{name} AXIS] Current position set as 0.000 mm origin!")
        else:
            messagebox.showwarning("Servo Not Armed", f"Please arm {name} axis first.")

    def _clear_k5100_fault(self, is_x: bool) -> None:
        name = "X" if is_x else "Z"
        drv = self.x_driver if is_x else self.z_driver
        if drv and drv.is_active:
            self._log(f"[{name} AXIS] Pulsing fault reset...")
            drv.clear_fault()
        else:
            ip = self.ip_x_var.get() if is_x else self.ip_z_var.get()
            self._set_cip_param(ip, PARAM_GAIN_ADJUST_MODE, 0, 2)

    def _global_fault_clear(self) -> None:
        self._log("[GLOBAL] Clearing faults on all 3 drives...")
        if self.th_driver and self.th_driver.is_active:
            self.th_driver.rearm()
        else:
            self._do_hcs01_c0500()

        if self.x_driver and self.x_driver.is_active:
            self.x_driver.clear_fault()
        if self.z_driver and self.z_driver.is_active:
            self.z_driver.clear_fault()

    def _global_disarm(self) -> None:
        self._log("[GLOBAL] Disarming all axes...")
        if self.th_driver and self.th_driver.is_active:
            self._toggle_live_th()
        if self.x_driver and self.x_driver.is_active:
            self._toggle_live_k5100(is_x=True)
        if self.z_driver and self.z_driver.is_active:
            self._toggle_live_k5100(is_x=False)

    # -------------------------------------------------------------------------
    # TELEMETRY POLLING LOOP
    # -------------------------------------------------------------------------
    def _telemetry_poll(self) -> None:
        # Theta feedback & LEDs
        if self.th_driver and self.th_driver.is_active:
            pos = self.th_driver.actual_pos_deg
            self.th_pos_var.set(f"{pos:.3f} deg")
            sw = self.th_driver.status_word
            diag = self.th_driver.diag_code
            self.th_diag_var.set(f"0x{diag:08X} (SW: 0x{sw:04X})")

            is_af = ((sw >> 14) & 3) == 3
            is_ready = ((sw >> 14) & 3) == 3
            is_homed = (sw & 0x0004) != 0
            is_standstill = (sw & 0x0008) != 0
            is_reached = (sw & 0x0010) != 0
            is_fault = (sw & 0x2000) != 0

            self._update_led("th_af", is_af)
            self._update_led("th_ready", is_ready)
            self._update_led("th_homed", is_homed)
            self._update_led("th_standstill", is_standstill)
            self._update_led("th_reached", is_reached)
            self._update_led("th_fault", is_fault, color="#d9534f")

            self._update_led("ov_th_af", is_af)
            self._update_led("ov_th_homed", is_homed)
            self._update_led("ov_th_ready", is_ready)

        # X Axis feedback & LEDs
        if self.x_driver and self.x_driver.is_active:
            pos_x = self.x_driver.actual_pos_mm
            self.x_pos_var.set(f"{pos_x:.3f} mm")
            st_x = self.x_driver.status
            lim_x = self.x_driver.di_limits

            self._update_led("x_ot_plus", lim_x["ot_plus"], color="#d9534f")
            self._update_led("x_ot_minus", lim_x["ot_minus"], color="#d9534f")
            self._update_led("x_ready", st_x.get("ready", False))
            self._update_led("x_active", st_x.get("active", False))
            self._update_led("x_at_ref", st_x.get("at_reference", False))
            self._update_led("x_stopped", st_x.get("stopped", False))
            self._update_led("x_homed", st_x.get("homed_status", False))
            self._update_led("x_fault", st_x.get("fault", False), color="#d9534f")
            self._update_led("x_warn", st_x.get("warning_present", False), color="#f0ad4e")

            self._update_led("ov_x_ot_plus", lim_x["ot_plus"], color="#d9534f")
            self._update_led("ov_x_ot_minus", lim_x["ot_minus"], color="#d9534f")
            self._update_led("ov_x_ready", st_x.get("ready", False))

        # Z Axis feedback & LEDs
        if self.z_driver and self.z_driver.is_active:
            pos_z = self.z_driver.actual_pos_mm
            self.z_pos_var.set(f"{pos_z:.3f} mm")
            st_z = self.z_driver.status
            lim_z = self.z_driver.di_limits

            self._update_led("z_ot_plus", lim_z["ot_plus"], color="#d9534f")
            self._update_led("z_ot_minus", lim_z["ot_minus"], color="#d9534f")
            self._update_led("z_ready", st_z.get("ready", False))
            self._update_led("z_active", st_z.get("active", False))
            self._update_led("z_at_ref", st_z.get("at_reference", False))
            self._update_led("z_stopped", st_z.get("stopped", False))
            self._update_led("z_homed", st_z.get("homed_status", False))
            self._update_led("z_fault", st_z.get("fault", False), color="#d9534f")
            self._update_led("z_warn", st_z.get("warning_present", False), color="#f0ad4e")

            self._update_led("ov_z_ot_plus", lim_z["ot_plus"], color="#d9534f")
            self._update_led("ov_z_ot_minus", lim_z["ot_minus"], color="#d9534f")
            self._update_led("ov_z_ready", st_z.get("ready", False))

        # Update live Conveyor Collision Guard status
        pos_x = self.x_driver.actual_pos_mm if (self.x_driver and self.x_driver.is_active) else 0.0
        pos_z = self.z_driver.actual_pos_mm if (self.z_driver and self.z_driver.is_active) else 0.0
        try:
            lim_x = self.collision_x_limit.get()
            lim_z = self.collision_z_limit.get()
        except Exception:
            lim_x = CONVEYOR_COLLISION_X_MIN_MM
            lim_z = CONVEYOR_COLLISION_Z_MIN_MM

        if pos_x >= lim_x and pos_z >= lim_z:
            self.collision_status_var.set("🚨 COLLISION ZONE (X≥95 & Z≥115)")
            self.lbl_guard_status.config(bg="#d9534f")
        elif pos_x >= (lim_x - 10.0) and pos_z >= (lim_z - 15.0):
            self.collision_status_var.set("⚠ PROXIMITY WARNING (Near Belt)")
            self.lbl_guard_status.config(bg="#f0ad4e")
        else:
            self.collision_status_var.set("● SAFE ENVELOPE (CLEAR)")
            self.lbl_guard_status.config(bg="#5cb85c")

        self.after(100, self._telemetry_poll)

    # -------------------------------------------------------------------------
    # PROCEDURAL REXROTH HCS01 ACTIONS
    # -------------------------------------------------------------------------
    def _get_hcs01(self) -> Hcs01Comws:
        if not self.hcs01_cli:
            self.hcs01_cli = Hcs01Comws(self.ip_th_eng_var.get().strip())
        return self.hcs01_cli

    def _do_hcs01_c0300(self) -> None:
        def task():
            was_active = False
            if self.th_driver and self.th_driver.is_active:
                was_active = True
                self._log("[HCS01] Pausing Class 1 stream during flash write...")
                self.th_driver.close()
                self.th_driver = None

            try:
                cli = self._get_hcs01()
                self._log("[HCS01] Logging into Service Tool HTTP...")
                cli.login()
                try:
                    mode = cli.getvar("P-0-0115.0.0")
                    if mode and mode.strip().endswith("1"):
                        self._log("[HCS01] Switching to Operating Mode (OM)...")
                        cli.run_command("S-0-0422.0.0", timeout_s=30.0)
                except Exception as e:
                    self._log(f"[HCS01] Mode check notice: {e}")

                try:
                    self._log("[HCS01] Clearing diagnostic latch (C0500)...")
                    cli.run_command("S-0-0099.0.0", timeout_s=10.0)
                except Exception:
                    pass

                self._log("[HCS01] Executing C0300 (Set Absolute Position Procedure)...")
                cli.run_command("S-0-0447.0.0", timeout_s=30.0)
                self._log("[HCS01] C0300 Complete! Saving to Non-Volatile Flash (C2200)...")
                cli.run_command("S-0-0264.0.0", timeout_s=45.0)
                self._log("[HCS01] SUCCESS: Drive origin permanently saved at mechanical zero!")
                self._read_theta_pos()
                messagebox.showinfo("Success", "Drive origin permanently calibrated and saved to NV flash!")
            except Exception as e:
                self._log(f"[HCS01 ERROR] C0300 zeroing failed: {e}")
                messagebox.showerror("Error", f"C0300 failed: {e}")

            if was_active:
                self._log("[HCS01] Resuming Class 1 motion...")
                self._toggle_live_th()

        threading.Thread(target=task, daemon=True).start()

    def _do_hcs01_c1800(self) -> None:
        def task():
            try:
                cli = self._get_hcs01()
                self._log("[HCS01] Logging in for Autotune...")
                cli.login()
                self._log("[HCS01] Setting C1800 excitation: Damping P-0-0163=1.0, Travel P-0-0169=30.0°, Envelope ±180°...")
                cli.setvar("P-0-0163.0.0", "1.0")
                cli.setvar("P-0-0169.0.0", "30.0000")
                cli.setvar("P-0-0166.0.0", "-180.0000")
                cli.setvar("P-0-0167.0.0", "180.0000")

                if self.th_driver and self.th_driver.is_active:
                    self.th_driver.enable_motor()
                    for _ in range(25):
                        if (self.th_driver.status_word >> 14) & 3 == 3:
                            break
                        time.sleep(0.1)

                self._log("[HCS01] Executing C1800 Inertia Identification Sweep (±30°)...")
                cli.run_command("P-0-0162.0.0", timeout_s=60.0)
                inertia = cli.getvar("P-0-4010.0.0")
                kp = cli.getvar("S-0-0100.0.0")
                tn = cli.getvar("S-0-0101.0.0")
                kv = cli.getvar("S-0-0104.0.0")
                self._log(f"[HCS01] Autotune Complete! Identified Load Inertia: {inertia} kg*m^2 | Kp: {kp} | Tn: {tn} ms | Kv: {kv} 1/s")
                self._log("[HCS01] Saving optimized gains to NV flash (C2200)...")
                cli.run_command("S-0-0264.0.0", timeout_s=45.0)
                self._log("[HCS01] Autotune gains saved successfully!")
                messagebox.showinfo("Autotune Complete", f"Identified Inertia: {inertia} kg*m^2\nKp (Velocity Gain): {kp}\nTn (Integral Time): {tn} ms\nKv (Position Gain): {kv} 1/s")
            except Exception as e:
                self._log(f"[HCS01 ERROR] C1800 failed: {e}")
                messagebox.showerror("Error", f"Autotune failed: {e}")

        threading.Thread(target=task, daemon=True).start()

    def _do_hcs01_factory_reset(self) -> None:
        def task():
            try:
                cli = self._get_hcs01()
                self._log("[HCS01] Logging in to reset tuning parameters...")
                cli.login()
                self._log("[HCS01] Restoring SCHUNK ERD-04 factory gains: Kp=0.030, Tn=10.0, Kv=1.00, Inertia=0...")
                cli.setvar("S-0-0100.0.0", "0.030")
                cli.setvar("S-0-0101.0.0", "10.0")
                cli.setvar("S-0-0104.0.0", "1.00")
                cli.setvar("S-0-0348.0.0", "0.00")
                cli.setvar("P-0-4010.0.0", "0.0000000")
                cli.setvar("P-0-0163.0.0", "2.5")
                cli.setvar("P-0-0166.0.0", "0.0000")
                cli.setvar("P-0-0167.0.0", "0.0000")
                cli.setvar("P-0-0169.0.0", "0.0000")

                self._log("[HCS01] Saving factory parameters to NV flash (C2200)...")
                cli.run_command("S-0-0264.0.0", timeout_s=45.0)
                self._log("[HCS01] SUCCESS: Factory parameters restored and saved to flash!")
                messagebox.showinfo("Reset Complete", "Factory gains restored:\nKp = 0.030\nTn = 10.0 ms\nKv = 1.00 1/s\nInertia = 0.0000 kg*m^2\nSaved to NV Flash.")
            except Exception as e:
                self._log(f"[HCS01 ERROR] Factory reset failed: {e}")
                messagebox.showerror("Error", f"Factory reset failed: {e}")

        threading.Thread(target=task, daemon=True).start()

    def _do_hcs01_c0500(self) -> None:
        def task():
            try:
                cli = self._get_hcs01()
                cli.login()
                self._log("[HCS01] Executing C0500 (Reset Diagnostic Messages)...")
                cli.run_command("S-0-0099.0.0", timeout_s=10.0)
                self._log("[HCS01] C0500 Complete — Faults cleared.")
                self._read_theta_pos()
            except Exception as e:
                self._log(f"[HCS01 ERROR] C0500 failed: {e}")

        threading.Thread(target=task, daemon=True).start()

    def _read_theta_pos(self) -> None:
        def task():
            try:
                cli = self._get_hcs01()
                cli.login()
                pos = cli.getvar("S-0-0051.0.0")
                diag = cli.getvar("S-0-0095.0.0")
                self.th_pos_var.set(f"{float(pos):.3f} deg")
                self.th_diag_var.set(diag)
                self._log(f"[HCS01] Pos: {pos} deg | Diag: {diag}")
            except Exception as e:
                self._log(f"[HCS01 ERROR] Read pos failed: {e}")

        threading.Thread(target=task, daemon=True).start()

    def _read_idn(self) -> None:
        def task():
            try:
                cli = self._get_hcs01()
                cli.login()
                idn = normalize_idn(self.idn_read_var.get())
                val = cli.getvar(idn)
                self.idn_val_var.set(val)
                self._log(f"[HCS01] {idn} = {val}")
            except Exception as e:
                self._log(f"[HCS01 ERROR] Read {idn} failed: {e}")

        threading.Thread(target=task, daemon=True).start()

    def _write_idn(self) -> None:
        def task():
            try:
                cli = self._get_hcs01()
                cli.login()
                idn = normalize_idn(self.idn_write_name.get())
                val = self.idn_write_val.get()
                cli.setvar(idn, val)
                self._log(f"[HCS01] Wrote {idn} = {val}")
            except Exception as e:
                self._log(f"[HCS01 ERROR] Write {idn} failed: {e}")

        threading.Thread(target=task, daemon=True).start()

    # -------------------------------------------------------------------------
    # KINETIX 5100 CIP EXPLICIT ACTIONS
    # -------------------------------------------------------------------------
    def _get_cip_param(self, ip: str, param_id: int) -> None:
        def task():
            try:
                client = eip.EipClient(ip)
                client.verbose = False
                if not client.connect() or not client.register_session():
                    self._log(f"[CIP ERROR {ip}] Failed to connect/register session")
                    return
                raw = client.get_attribute_single(0x0F, param_id, 1)
                client.unregister_session()
                client.disconnect()
                if raw is not None:
                    val = int.from_bytes(raw, "little")
                    self._log(f"[CIP {ip}] Read Param ID {param_id} = {val} (0x{val:X})")
                else:
                    self._log(f"[CIP {ip}] Read Param ID {param_id} FAILED")
            except Exception as e:
                self._log(f"[CIP ERROR {ip}] Read Param {param_id} failed: {e}")

        threading.Thread(target=task, daemon=True).start()

    def _set_cip_param(self, ip: str, param_id: int, value: int, size: int) -> None:
        def task():
            try:
                client = eip.EipClient(ip)
                client.verbose = False
                if not client.connect() or not client.register_session():
                    self._log(f"[CIP ERROR {ip}] Failed to connect/register session")
                    return
                val_bytes = value.to_bytes(size, "little")
                ok = client.set_attribute_single(0x0F, param_id, 1, val_bytes)
                client.unregister_session()
                client.disconnect()
                self._log(f"[CIP {ip}] Write Param ID {param_id} = {value}: {'OK (Success)' if ok else 'FAILED'}")
            except Exception as e:
                self._log(f"[CIP ERROR {ip}] Write Param {param_id} failed: {e}")

        threading.Thread(target=task, daemon=True).start()

    def _read_all_kinetix(self, ip: str) -> None:
        def task():
            try:
                client = eip.EipClient(ip)
                client.verbose = False
                if not client.connect() or not client.register_session():
                    self._log(f"[CIP ERROR {ip}] Connection failed")
                    return

                self._log(f"=== Reading Full Parameter Snapshot for {ip} ===")
                for pid, name, sz in [
                    (PARAM_GAIN_ADJUST_MODE, "Gain Adjust Mode", 2),
                    (PARAM_RESPONSE_LEVEL, "Response Level", 2),
                    (PARAM_LOAD_INERTIA_RATIO, "Load Inertia Ratio", 2),
                    (PARAM_POS_PROP_GAIN, "Position Gain Kp", 2),
                    (PARAM_VEL_PROP_GAIN, "Velocity Gain Kv", 2),
                    (PARAM_VEL_INT_GAIN, "Velocity Int Gain Ti", 2),
                    (PARAM_TOTAL_INERTIA, "Total Inertia", 4),
                ]:
                    raw = client.get_attribute_single(0x0F, pid, 1)
                    if raw is not None:
                        val = int.from_bytes(raw, "little")
                        self._log(f"  ID {pid:3d} ({name:20s}) = {val}")
                    else:
                        self._log(f"  ID {pid:3d} ({name:20s}) = FAILED")

                client.unregister_session()
                client.disconnect()
            except Exception as e:
                self._log(f"[CIP ERROR {ip}] Snapshot failed: {e}")

        threading.Thread(target=task, daemon=True).start()

    # -------------------------------------------------------------------------
    # NETWORKING PROBING & LOGGING
    # -------------------------------------------------------------------------
    def _ping_all_drives(self) -> None:
        def task():
            self._log("=== Probing EtherNet/IP & HTTP Drive Endpoints ===")
            endpoints = [
                ("Theta HCS01 HTTP COMWS", self.ip_th_eng_var.get().strip(), 80),
                ("Theta HCS01 CIP/FKM", self.ip_th_cip_var.get().strip(), EIP_PORT),
                ("X Axis Kinetix 5100 CIP", self.ip_x_var.get().strip(), EIP_PORT),
                ("Z Axis Kinetix 5100 CIP", self.ip_z_var.get().strip(), EIP_PORT),
            ]
            for name, ip, port in endpoints:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(0.6)
                try:
                    s.connect((ip, port))
                    s.close()
                    self._log(f"[ONLINE] {name} ({ip}:{port}) responded OK")
                except Exception:
                    self._log(f"[OFFLINE] {name} ({ip}:{port}) did not respond")

        threading.Thread(target=task, daemon=True).start()

    def _log(self, msg: str) -> None:
        ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.log_q.put(f"[{ts}] {msg}\n")

    def _process_logs(self) -> None:
        while not self.log_q.empty():
            msg = self.log_q.get()
            self.txt_log.insert(tk.END, msg)
            self.txt_log.see(tk.END)
        self.after(40, self._process_logs)

    def _on_close(self) -> None:
        if self.th_driver:
            self.th_driver.disable_motor()
            self.th_driver.close()
        if self.x_driver:
            self.x_driver.disable_servo()
            self.x_driver.close()
        if self.z_driver:
            self.z_driver.disable_servo()
            self.z_driver.close()
        self.destroy()


def main() -> int:
    app = DirectEipDriverApp()
    app.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())

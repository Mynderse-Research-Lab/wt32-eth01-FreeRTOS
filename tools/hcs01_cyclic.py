#!/usr/bin/env python3
"""hcs01_cyclic.py — Direct PC-side EtherNet/IP Class 1 Real-Time Cyclic Driver for Rexroth HCS01."""

from __future__ import annotations

import socket
import struct
import sys
import threading
import time
from pathlib import Path
from typing import Optional

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

import eip_test as eip

# SCHUNK ERD-04 Theta scaling: 10,000 PUU per degree (0.0001 deg resolution)
PUU_PER_DEG = 10000.0


# Control word bitmask constants (matching Hcs01ControlStatus.cpp):
# bit 15: drive_on (0x8000), bit 14: drive_enable (0x4000), bit 13: drive_halt (0x2000)
# bit 5: clear_errors (0x0020), bit 4: immediate_block_change (0x0010), bit 1: operating_mode_select (0x0002)
CW_CLEAR_FAULTS = 0x4022  # bit 14, 5, 1
CW_DRIVE_OFF    = 0x4002  # bit 14, 1 (Ab)
CW_DRIVE_HALT   = 0xC012  # bit 15, 14, 4, 1 (AH)
CW_DRIVE_ENABLE = 0xE012  # bit 15, 14, 13, 4, 1 (AF)


class Hcs01CyclicDriver:
    """Provides pure Class 1 cyclic I/O drive control for Bosch Rexroth HCS01 directly from PC."""

    def __init__(self, ip: str = "192.168.1.23") -> None:
        self.ip = ip
        self.client: Optional[eip.EipClient] = None
        self.params = eip.ForwardOpenParams()
        self.params.ot_instance = 101
        self.params.to_instance = 102
        self.params.ot_assembly_size = 18
        self.params.to_assembly_size = 14
        self.params.include_run_idle_header = True
        self.params.ot_rpi_us = 10000  # 10ms
        self.params.to_rpi_us = 10000
        self.params.connection_serial = (int(time.time() * 100) & 0x7FFF) + 1
        self.params.originator_serial = 0xCAFEB00D

        self.ot_id = 0
        self.running = False
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

        # Motion targets
        self.control_word = CW_DRIVE_HALT
        self.target_pos_puu = 0
        self.target_vel_puu = int(45.0 * PUU_PER_DEG)    # 45 deg/s
        self.accel_puu = int(180.0 * PUU_PER_DEG)       # 180 deg/s^2
        self.decel_puu = int(180.0 * PUU_PER_DEG)

        # Feedback
        self.actual_pos_deg = 0.0
        self.actual_vel_deg_s = 0.0
        self.status_word = 0
        self.diag_code = 0
        self.command_accept = 0

    @property
    def is_active(self) -> bool:
        return self.running and self.client is not None

    def connect(self) -> bool:
        """Connect TCP session, register CIP session, and ForwardOpen Class 1."""
        if self.running:
            return True

        self.client = eip.EipClient(self.ip)
        self.client.verbose = False
        if not self.client.connect():
            return False
        if not self.client.register_session():
            self.client.disconnect()
            return False

        res = self.client.forward_open(self.params)
        if not res or "ot_connection_id" not in res:
            self.client.unregister_session()
            self.client.disconnect()
            return False

        self.ot_id = res["ot_connection_id"]
        self.running = True
        self._thread = threading.Thread(target=self._cyclic_loop, name="hcs01-cyclic-io", daemon=True)
        self._thread.start()
        return True

    def enable_motor(self) -> None:
        """Proper Rexroth 2-step arming sequence: Ab -> AH (0xC012) -> AF (0xE012)."""
        self.rearm()

    def rearm(self) -> None:
        """Pulse bit 15 low->high (0xC012 -> 0xE012) to force clean 0->1 rising edge on Drive On."""
        with self._lock:
            self.control_word = CW_CLEAR_FAULTS  # 0x4022 (Pulse C0500 fault clear)
        time.sleep(0.15)
        with self._lock:
            self.control_word = CW_DRIVE_HALT    # 0xC012 (AH Mode)
        time.sleep(0.15)
        with self._lock:
            self.control_word = CW_DRIVE_ENABLE | self.command_accept  # 0xE012 | accept (AF Mode)
        time.sleep(0.15)

    def disable_motor(self) -> None:
        """Set Control Word to 0xC012 (Drive Halt / AH mode)."""
        with self._lock:
            self.control_word = CW_DRIVE_HALT

    def move_to_deg(self, angle_deg: float, speed_deg_s: float = 45.0) -> None:
        """Move Theta to absolute angle in degrees (toggles command_value_accept bit)."""
        if (self.status_word >> 14) & 3 != 3:
            self.rearm()
        with self._lock:
            self.command_accept = 1 - self.command_accept
            self.control_word = CW_DRIVE_ENABLE | self.command_accept
            self.target_pos_puu = int(round(angle_deg * PUU_PER_DEG))
            self.target_vel_puu = max(1000, int(round(speed_deg_s * PUU_PER_DEG)))

    def nudge_deg(self, delta_deg: float, speed_deg_s: float = 30.0) -> None:
        """Jog Theta by relative delta in degrees (toggles command_value_accept bit)."""
        if (self.status_word >> 14) & 3 != 3:
            self.rearm()
        with self._lock:
            self.command_accept = 1 - self.command_accept
            self.control_word = CW_DRIVE_ENABLE | self.command_accept
            self.target_pos_puu = int(round((self.actual_pos_deg + delta_deg) * PUU_PER_DEG))
            self.target_vel_puu = max(1000, int(round(speed_deg_s * PUU_PER_DEG)))

    def close(self) -> None:
        """Stop cyclic thread and close ForwardOpen connection."""
        self.running = False
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None

        if self.client:
            try:
                self.client.forward_close(self.params)
                self.client.unregister_session()
                self.client.disconnect()
            except Exception:
                pass
            self.client = None

    def _cyclic_loop(self) -> None:
        """Continuous ~100Hz cyclic exchange loop."""
        while self.running and self.client:
            with self._lock:
                cw = self.control_word
                t_pos = self.target_pos_puu
                t_vel = self.target_vel_puu
                acc = self.accel_puu
                dec = self.decel_puu

            # Assembly 101: ControlWord(u16), TargetPos(i32), TargetVel(i32), Accel(i32), Decel(i32) = 18 bytes
            asm_ot = (
                struct.pack("<H", cw)
                + struct.pack("<i", t_pos)
                + struct.pack("<i", t_vel)
                + struct.pack("<i", acc)
                + struct.pack("<i", dec)
            )

            try:
                rx = self.client.exchange_io_frame(
                    self.ot_id,
                    asm_ot,
                    include_run_idle=True,
                    timeout=0.2,
                    reuse_socket=True,
                    drain=True,
                )
                if rx and len(rx) >= 14:
                    # Assembly 102: StatusWord(u16), ActualPos(i32), ActualVel(i32), DiagCode(u32)
                    sw, pos_puu, vel_puu, diag = struct.unpack_from("<HiiI", rx, 0)
                    with self._lock:
                        self.status_word = sw
                        self.actual_pos_deg = pos_puu / PUU_PER_DEG
                        self.actual_vel_deg_s = vel_puu / PUU_PER_DEG
                        self.diag_code = diag
            except Exception:
                pass

            time.sleep(0.01)  # 10ms cycle

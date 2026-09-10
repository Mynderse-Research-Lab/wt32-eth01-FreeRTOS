#!/usr/bin/env python3
"""
Kinetix 5100 Class 1 Cyclic EtherNet/IP Driver for PC.
Supports live micro-jogging, absolute positioning, zero homing, fault clearing,
and real-time hardware I/O and drive status monitoring.
"""

import os
import struct
import sys
import threading
import time
from pathlib import Path
from typing import Dict, Optional, Any

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

try:
    import eip_test as eip
except ImportError:
    import eip_test as eip

# Mechanical conversion defaults (matching firmware):
# X: Lead 200 mm/rev, Ratio 5:1 -> 52428.8 PUU/mm
# Z: Lead 20 mm/rev, Ratio 1:1  -> 104857.6 PUU/mm
X_PUU_PER_MM = 52428.8
X_LEAD_MM = 200.0
X_RATIO = 5.0

Z_PUU_PER_MM = 104857.6
Z_LEAD_MM = 20.0
Z_RATIO = 1.0

# Conveyor collision limits with current gripper jaws:
# Prohibit simultaneous X >= 95.0 mm and Z >= 115.0 mm
CONVEYOR_COLLISION_X_MIN_MM = 95.0
CONVEYOR_COLLISION_Z_MIN_MM = 115.0


class K5100CyclicDriver:
    """Real-time Class 1 cyclic I/O driver for Allen-Bradley Kinetix 5100."""

    def __init__(self, ip: str, puu_per_mm: float = X_PUU_PER_MM, lead_mm: float = 200.0, ratio: float = 5.0) -> None:
        self.ip = ip
        self.puu_per_mm = puu_per_mm
        self.lead_mm = lead_mm
        self.ratio = ratio

        self.client: Optional[eip.EipClient] = None
        self.params = eip.ForwardOpenParams()
        self.params.ot_instance = 104
        self.params.to_instance = 154
        self.params.ot_assembly_size = 40
        self.params.to_assembly_size = 52
        self.params.include_run_idle_header = True
        self.params.ot_rpi_us = 10000  # 10 ms RPI
        self.params.to_rpi_us = 10000
        self.params.connection_serial = (int(time.time() * 100) & 0x7FFF) + 1
        self.params.originator_serial = 0xAB510001

        self.ot_id = 0
        self.running = False
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

        # Motion command state
        self.servo_on = False
        self.fault_reset = False
        self.operating_mode = 0  # 0=Settle/Hold, 1=Position, 3=Home
        self.travel_mode = 10    # 10=Cyclic/Settle, 2=NonCyclic
        self.non_cyclic_move_type = 0  # 0=Absolute
        self.target_position_puu = 0
        self.speed_rpm = 60.0
        self.accel_rpm_s = 1000.0
        self.decel_rpm_s = 1000.0
        self.homing_method = 34
        self.start_motion = False

        # Internal state machine for edge-triggered StartMotion
        self._pending_move = False
        self._move_step = 0
        self._move_ticks = 0

        # Live feedback telemetry
        self.actual_pos_puu = 0
        self.actual_pos_mm = 0.0
        self.actual_speed_rpm = 0
        self.actual_speed_mm_s = 0.0
        self.actual_torque_pct = 0.0
        self.fault_code = 0
        self.warning_code = 0

        # Assembly 154 Status flags
        self.status: Dict[str, Any] = {
            "ready": False,
            "active": False,
            "at_reference": False,
            "stopped": False,
            "homed_status": False,
            "fault": False,
            "warning_present": False,
            "command_in_progress": False,
            "run_mode": False,
            "connection_faulted": False,
            "diagnostic_active": False,
        }

        # Hardware Digital Input/Output status
        self.di_status = 0  # Raw bitmask from parameter 281
        self.do_status = 0  # Raw bitmask from parameter 283
        self.di_limits = {"ot_plus": False, "ot_minus": False}

    @property
    def is_active(self) -> bool:
        return self.running and self.client is not None

    def connect(self) -> bool:
        """Open TCP session, register CIP session, and ForwardOpen Class 1."""
        if self.running:
            return True

        self.client = eip.EipClient(self.ip)
        self.client.verbose = False
        if not self.client.connect() or not self.client.register_session():
            return False

        res = self.client.forward_open(self.params)
        if not res or "ot_connection_id" not in res:
            self.client.unregister_session()
            self.client.disconnect()
            return False

        self.ot_id = res["ot_connection_id"]
        self.running = True
        self.servo_on = True
        self.operating_mode = 0
        self.travel_mode = 10
        self._thread = threading.Thread(target=self._cyclic_loop, name=f"k5100-{self.ip}", daemon=True)
        self._thread.start()
        return True

    def enable_servo(self) -> None:
        """Enable servo drive (ServoOn=True, OM=0, TM=10)."""
        with self._lock:
            self.servo_on = True
            self.operating_mode = 0
            self.travel_mode = 10
            self.start_motion = False
            self._pending_move = False

    def disable_servo(self) -> None:
        """Disable servo drive (ServoOn=False)."""
        with self._lock:
            self.servo_on = False
            self.start_motion = False
            self._pending_move = False

    def clear_fault(self) -> None:
        """Pulse fault reset bit."""
        def task():
            with self._lock:
                self.fault_reset = True
            time.sleep(0.15)
            with self._lock:
                self.fault_reset = False
        threading.Thread(target=task, daemon=True).start()

    def home_current_position(self) -> None:
        """Execute Homing Method 34 (Define current position as 0.000 mm origin)."""
        def task():
            with self._lock:
                self.servo_on = True
                self.operating_mode = 3
                self.travel_mode = 2
                self.homing_method = 34
                self.start_motion = False
            time.sleep(0.05)
            with self._lock:
                self.start_motion = True
            time.sleep(0.05)
            with self._lock:
                self.start_motion = False
            time.sleep(0.2)
            with self._lock:
                self.operating_mode = 0
                self.travel_mode = 10
                self.target_position_puu = self.actual_pos_puu
        threading.Thread(target=task, daemon=True).start()

    def move_to_mm(self, target_mm: float, speed_mm_s: float = 50.0, accel_mm_s2: float = 3000.0) -> None:
        """Command absolute position move in mm."""
        target_puu = int(round(target_mm * self.puu_per_mm))
        speed_rpm = max(1.0, float(speed_mm_s) * float(self.ratio) / float(self.lead_mm) * 60.0)
        accel_rpm_s = max(10.0, float(accel_mm_s2) * float(self.ratio) / float(self.lead_mm) * 60.0)

        with self._lock:
            self.target_position_puu = target_puu
            self.speed_rpm = speed_rpm
            self.accel_rpm_s = accel_rpm_s
            self.decel_rpm_s = accel_rpm_s
            self.operating_mode = 1
            self.travel_mode = 2
            self.non_cyclic_move_type = 0
            self._pending_move = True
            self._move_step = 1
            self._move_ticks = 0

    def nudge_mm(self, delta_mm: float, speed_mm_s: float = 30.0) -> None:
        """Command incremental / relative jog in mm."""
        target_mm = self.actual_pos_mm + delta_mm
        self.move_to_mm(target_mm, speed_mm_s=speed_mm_s)

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
        """100Hz real-time cyclic exchange loop."""
        while self.running and self.client:
            with self._lock:
                # Handle edge-triggered StartMotion sequence:
                # Step 1: Preload position with start_motion=False (4 ticks)
                # Step 2: Edge start_motion=True (5 ticks)
                # Step 3: Clear start_motion=False, wait AtReference
                if self._pending_move:
                    if self._move_step == 1:
                        self.start_motion = False
                        self._move_ticks += 1
                        if self._move_ticks >= 4:
                            self._move_step = 2
                            self._move_ticks = 0
                    elif self._move_step == 2:
                        self.start_motion = True
                        self._move_ticks += 1
                        if self._move_ticks >= 5:
                            self._move_step = 3
                            self._move_ticks = 0
                    elif self._move_step == 3:
                        self.start_motion = False
                        if self.status.get("at_reference") or self.status.get("stopped"):
                            self._pending_move = False

                cmd_bytes = eip.build_output_assembly_104(
                    servo_on=self.servo_on,
                    fault_reset=self.fault_reset,
                    operating_mode=self.operating_mode,
                    travel_mode=self.travel_mode,
                    non_cyclic_move_type=self.non_cyclic_move_type,
                    position_puu=self.target_position_puu,
                    speed_rpm=self.speed_rpm,
                    accel_rpm_per_s=self.accel_rpm_s,
                    decel_rpm_per_s=self.decel_rpm_s,
                    homing_method=self.homing_method,
                    start_motion=self.start_motion,
                    torque_ramp_time_ms=1000,
                )

            to = self.client.exchange_io_frame(self.ot_id, cmd_bytes, include_run_idle=True, timeout=0.5)
            if to:
                st = eip.parse_input_assembly_154(to)
                if "error" not in st:
                    with self._lock:
                        self.actual_pos_puu = int(st.get("actual_position", 0))
                        self.actual_pos_mm = self.actual_pos_puu / self.puu_per_mm
                        self.actual_speed_rpm = int(st.get("actual_speed", 0))
                        self.actual_speed_mm_s = self.actual_speed_rpm / 60.0 * self.lead_mm / self.ratio
                        self.actual_torque_pct = float(st.get("actual_torque", 0))
                        self.fault_code = int(st.get("fault_code", 0))
                        self.warning_code = int(st.get("warning_code", 0))

                        for k in self.status.keys():
                            if k in st:
                                self.status[k] = bool(st[k])

                        # Detect Overtravel Limit switch triggers from Fault / Warning codes
                        # A014: Negative Overtravel (OT-), A015: Positive Overtravel (OT+)
                        self.di_limits["ot_minus"] = (self.fault_code == 0xA014 or self.warning_code == 0xA014)
                        self.di_limits["ot_plus"] = (self.fault_code == 0xA015 or self.warning_code == 0xA015)

            time.sleep(0.01)  # ~100Hz cycle

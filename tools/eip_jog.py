#!/usr/bin/env python3
"""Live dual-axis keyboard jog for Kinetix 5100 (X + Z) over EtherNet/IP.

  X: Left / Right arrows
  Z: A / D
  SPACE = stop both   Esc / Q = quit

  py tools/eip_jog.py
  py tools/eip_jog.py --speed 200 --accel 5000

Defaults: X=192.168.1.20, Z=192.168.1.21. Both drives must be in I/O mode
(P1.001 = 0x0C). Persistent servo hold on X+Z is paused for the session and
restored on exit.
"""
from __future__ import annotations

import argparse
import ctypes
import datetime
import re
import socket
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional, TextIO

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

import eip_test as eip  # noqa: E402

DEFAULT_X_IP = "192.168.1.20"
DEFAULT_Z_IP = "192.168.1.21"

# Rotating session logs: .eip_jog.log + .eip_jog.1.log … .eip_jog.4.log (5 total).
JOG_LOG_FILE = SCRIPT_DIR / ".eip_jog.log"
JOG_LOG_VERSIONS = 5
JOG_LOG_STATUS_INTERVAL_S = 0.5

# Windows virtual-key codes (GetAsyncKeyState)
VK_LEFT = 0x25
VK_RIGHT = 0x27
VK_ESCAPE = 0x1B
VK_SPACE = 0x20
VK_Q = 0x51
VK_A = 0x41
VK_D = 0x44

PRELOAD_FRAMES = 2
START_FRAMES = 4
STOP_PULSE_FRAMES = 5
STOP_ZERO_PRELOAD_FRAMES = 2
STOP_ZERO_START_FRAMES = 4
STALL_RETRY_S = 0.35
NEAR_STOP_RPM = 5.0
# Ignore 1-frame key drops so preload isn't aborted by chatter.
RELEASE_DEBOUNCE_FRAMES = 2
# If still spinning this many hold frames after StopMotion, SM-edge to 0 immediately.
STOP_ESCALATE_FRAMES = 2

_ANSI_RE = re.compile(r"\033\[[0-9;]*[A-Za-z]")


def _jog_log_path(index: int) -> Path:
    """index 0 = current .eip_jog.log; 1..N-1 = rotated backups."""
    if index <= 0:
        return JOG_LOG_FILE
    return SCRIPT_DIR / f".eip_jog.{index}.log"


def rotate_jog_logs(versions: int = JOG_LOG_VERSIONS) -> Path:
    """Rotate previous session logs, keeping `versions` files (overwrite oldest)."""
    versions = max(1, int(versions))
    # Delete oldest slot, then shift N-2 → N-1, …, 0 → 1.
    oldest = _jog_log_path(versions - 1)
    try:
        oldest.unlink(missing_ok=True)
    except OSError:
        pass
    for i in range(versions - 2, -1, -1):
        src = _jog_log_path(i)
        dst = _jog_log_path(i + 1)
        if src.exists():
            try:
                src.replace(dst)
            except OSError:
                try:
                    dst.unlink(missing_ok=True)
                    src.replace(dst)
                except OSError:
                    pass
    return JOG_LOG_FILE


class TeeStdout:
    """Mirror console output into a session log (ANSI stripped, status throttled)."""

    def __init__(self, console: TextIO, log_file: TextIO,
                 status_interval_s: float = JOG_LOG_STATUS_INTERVAL_S):
        self.console = console
        self.log_file = log_file
        self.status_interval_s = status_interval_s
        self._last_status_log_s = 0.0
        self._pending_status = ""

    def write(self, data: str) -> int:
        if not isinstance(data, str):
            data = str(data)
        self.console.write(data)
        self.console.flush()

        clean = _ANSI_RE.sub("", data)
        if not clean:
            return len(data)

        # Live status uses \r / cursor-up; throttle those into the log.
        if "\r" in clean and "\n" not in clean:
            self._pending_status = clean.replace("\r", "").rstrip() + "\n"
            now = time.time()
            if now - self._last_status_log_s >= self.status_interval_s:
                self.log_file.write(self._pending_status)
                self.log_file.flush()
                self._last_status_log_s = now
                self._pending_status = ""
            return len(data)

        # Multi-line status block (cursor-up already stripped): treat like status.
        if "\n" in clean and clean.count("\n") >= 1 and "cmd=" in clean and "act=" in clean:
            self._pending_status = clean if clean.endswith("\n") else clean + "\n"
            now = time.time()
            if now - self._last_status_log_s >= self.status_interval_s:
                self.log_file.write(self._pending_status)
                self.log_file.flush()
                self._last_status_log_s = now
                self._pending_status = ""
            return len(data)

        text = clean.replace("\r", "\n")
        if text:
            self.log_file.write(text)
            self.log_file.flush()
        return len(data)

    def flush(self) -> None:
        self.console.flush()
        if self._pending_status:
            self.log_file.write(self._pending_status)
            self._pending_status = ""
            self._last_status_log_s = time.time()
        self.log_file.flush()

    def isatty(self) -> bool:
        return bool(getattr(self.console, "isatty", lambda: False)())

    def fileno(self) -> int:
        return self.console.fileno()


def open_jog_session_log(versions: int = JOG_LOG_VERSIONS) -> tuple[TextIO, Path]:
    """Rotate logs and open a new current session log file."""
    path = rotate_jog_logs(versions=versions)
    log_file = open(path, "w", encoding="utf-8", errors="replace", newline="\n")
    stamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    log_file.write(f"# EIP Jog Log — {stamp}\n")
    log_file.write(f"# Command: {' '.join(sys.argv)}\n")
    log_file.write(f"# Rotating keep={versions}: {path.name}")
    for i in range(1, versions):
        log_file.write(f", {_jog_log_path(i).name}")
    log_file.write("\n\n")
    log_file.flush()
    return log_file, path


def _key_down(vk: int) -> bool:
    return bool(ctypes.windll.user32.GetAsyncKeyState(vk) & 0x8000)


def _axis_intent(neg_vk: int, pos_vk: int) -> int:
    """Return -1 / 0 / +1 from a pair of held keys."""
    neg = _key_down(neg_vk)
    pos = _key_down(pos_vk)
    if neg and not pos:
        return -1
    if pos and not neg:
        return 1
    return 0


def _extract_assembly(rx_data: bytes) -> Optional[bytes]:
    for type_id, item_data in eip.parse_cpf(rx_data):
        if type_id == eip.CPF_CONNECTED_DATA and len(item_data) >= 2:
            return item_data[2:]
    return None


def _build_ot_cpf(ot_conn_id: int, assembly_data: bytes) -> bytes:
    encap_seq = int(time.time() * 1000) & 0xFFFFFFFF
    cip_seq = encap_seq & 0xFFFF
    addr_data = eip.le32(ot_conn_id) + eip.le32(encap_seq)
    data = eip.le16(cip_seq) + eip.le32(0x00000001) + assembly_data
    return eip.build_cpf([
        (eip.CPF_SEQUENCED_ADDRESS, addr_data),
        (eip.CPF_CONNECTED_DATA, data),
    ])


@dataclass
class AxisJog:
    name: str
    host: str
    speed_rpm: float
    accel_rpm_per_s: float
    decel_rpm_per_s: float
    rpi_us: int
    ot_connection_id_seed: int
    connection_serial: int
    client: Optional[eip.EipClient] = None
    fo_result: Optional[dict] = None
    fo_params: Optional[eip.ForwardOpenParams] = None
    ot_id: int = 0
    last_st: dict = field(default_factory=dict)
    phase: str = "hold"  # hold | stop | preload | start | run
    # Within stop: pulse StopMotion, hold speed 0, then SM-edge to 0 if still moving.
    stop_mode: str = "pulse"  # pulse | hold | zero_preload | zero_start
    phase_frames: int = 0
    active_dir: int = 0
    target_dir: int = 0
    target_rpm: float = 0.0
    stall_since: Optional[float] = None
    cmd_rpm: float = 0.0
    release_count: int = 0

    def make_asm(self, speed: float, *, start: bool = False, stop: bool = False) -> bytes:
        return eip.build_output_assembly_104(
            servo_on=True,
            speed_rpm=speed,
            accel_rpm_per_s=self.accel_rpm_per_s,
            decel_rpm_per_s=self.decel_rpm_per_s,
            operating_mode=2,
            travel_mode=10,
            start_motion=start,
            stop_motion=stop,
            torque_ramp_time_ms=1000)

    def _act_rpm(self) -> float:
        return abs(int(self.last_st.get("actual_speed", 0)) / 10.0)

    def _stop_timeout_frames(self) -> int:
        # Fallback only — preferred path escalates via STOP_ESCALATE_FRAMES.
        est_s = abs(self.speed_rpm) / max(self.decel_rpm_per_s, 1.0)
        return max(STOP_ESCALATE_FRAMES + 2,
                   int(est_s / max(self.rpi_us / 1e6, 0.002)) + 4)

    def open(self, shared_udp: socket.socket) -> str | None:
        client = eip.EipClient(host=self.host)
        client.verbose = False
        if not client.connect() or not client.register_session():
            return f"{self.name}: TCP/RegisterSession failed ({self.host})"

        ready = client.check_drive_ready()
        if not ready.get("powered_on") or not ready.get("io_mode"):
            client.unregister_session()
            client.disconnect()
            return f"{self.name}: not powered or not I/O mode (P1.001=0x0C) at {self.host}"

        params = eip.ForwardOpenParams()
        params.ot_assembly_size = 40
        params.to_assembly_size = 52
        params.ot_rpi_us = self.rpi_us
        params.to_rpi_us = self.rpi_us
        params.raw_ot_conn_size = 40 + 2 + 4
        params.include_run_idle_header = True
        params.connection_timeout_multiplier = 7
        params.ot_connection_id = self.ot_connection_id_seed
        params.connection_serial = self.connection_serial

        result = eip.forward_open_with_retry(client, params)
        ot = int((result or {}).get("ot_connection_id", 0) or 0)
        if ot == 0:
            client.unregister_session()
            client.disconnect()
            return f"{self.name}: ForwardOpen failed ({self.host})"

        client.io_sock = shared_udp
        self.client = client
        self.fo_result = result
        self.fo_params = result.get("params", params)
        self.ot_id = ot
        return None

    def settle(self, shared_udp: socket.socket) -> None:
        assert self.client is not None
        settle = self.make_asm(0.0)
        stop_asm = self.make_asm(0.0, stop=True)
        deadline = time.time() + 3.0
        cip_cleared = False

        for _ in range(3):
            self._send_only(shared_udp, eip.build_output_assembly_104(
                servo_on=False, operating_mode=0, travel_mode=10, torque_ramp_time_ms=1000))
            time.sleep(self.rpi_us / 1e6)

        while time.time() < deadline:
            want_stop = (not cip_cleared) and bool(self.last_st.get("command_in_progress"))
            asm = stop_asm if want_stop else settle
            st = self._exchange(shared_udp, asm)
            if st is not None:
                self.last_st = st
                if want_stop and not st.get("command_in_progress"):
                    cip_cleared = True
                if (st.get("active") and st.get("ready") and not st.get("warning_present")
                        and (cip_cleared or not st.get("command_in_progress"))):
                    break
            time.sleep(self.rpi_us / 1e6)

        if self.last_st.get("command_in_progress"):
            print(f"  [INFO] {self.name}: clearing leftover CommandInProgress...")
            for _ in range(8):
                st = self._exchange(shared_udp, stop_asm)
                if st is not None:
                    self.last_st = st
                time.sleep(self.rpi_us / 1e6)
            for _ in range(4):
                self._exchange(shared_udp, settle)
                time.sleep(self.rpi_us / 1e6)

        if not self.last_st.get("active"):
            print(f"  [WARN] {self.name}: Active not confirmed; continuing anyway.")

    def _send_only(self, udp: socket.socket, asm: bytes) -> None:
        udp.sendto(_build_ot_cpf(self.ot_id, asm), (self.host, eip.UDP_PORT))

    def _exchange(self, udp: socket.socket, asm: bytes, timeout: float = 1.0) -> Optional[dict]:
        self._send_only(udp, asm)
        udp.settimeout(timeout)
        deadline = time.time() + timeout
        latest: Optional[bytes] = None
        while time.time() < deadline:
            try:
                rx, addr = udp.recvfrom(4096)
            except (socket.timeout, OSError):
                break
            if addr[0] != self.host:
                continue
            assy = _extract_assembly(rx)
            if assy is not None:
                latest = assy
            udp.setblocking(False)
            try:
                while True:
                    try:
                        rx, addr = udp.recvfrom(4096)
                    except (BlockingIOError, OSError):
                        break
                    if addr[0] != self.host:
                        continue
                    assy = _extract_assembly(rx)
                    if assy is not None:
                        latest = assy
            finally:
                udp.settimeout(timeout)
            break
        if latest is None:
            return None
        st = eip.parse_input_assembly_154(latest)
        return None if "error" in st else st

    def enter_stop(self) -> None:
        self.phase = "stop"
        self.stop_mode = "pulse"
        self.phase_frames = 0
        self.active_dir = 0
        self.target_rpm = 0.0
        self.stall_since = None
        self.release_count = 0

    def enter_preload(self, direction: int) -> None:
        self.phase = "preload"
        self.phase_frames = 0
        self.active_dir = direction
        self.target_rpm = float(direction) * abs(self.speed_rpm)
        self.stall_since = None
        self.release_count = 0

    def _finish_stop_or_rearm(self) -> Optional[bytes]:
        """Leave stop; if a direction is pending, return that preload frame immediately."""
        if self.target_dir != 0:
            self.enter_preload(self.target_dir)
            self.cmd_rpm = self.target_rpm
            self.phase_frames = 1
            return self.make_asm(self.target_rpm, start=False)
        self.phase = "hold"
        self.phase_frames = 0
        self.active_dir = 0
        self.target_rpm = 0.0
        self.cmd_rpm = 0.0
        return None

    def step_fsm(self, desired: int) -> bytes:
        self.target_dir = desired
        if desired == 0:
            self.release_count += 1
        else:
            self.release_count = 0

        # Direction change / start: reverse while running via SM re-edge (no full stop).
        if desired != 0 and desired != self.active_dir:
            if self.phase in ("hold", "run", "preload", "start"):
                self.enter_preload(desired)
            # If still stopping, target_dir is updated; re-arm when nearly stopped.
        elif (desired == 0
              and self.phase in ("preload", "start", "run")
              and self.release_count >= RELEASE_DEBOUNCE_FRAMES):
            self.enter_stop()

        if self.phase == "stop":
            return self._step_stop()

        if self.phase == "preload" and self.phase_frames >= PRELOAD_FRAMES:
            self.phase = "start"
            self.phase_frames = 0
        elif self.phase == "start" and self.phase_frames >= START_FRAMES:
            self.phase = "run"
            self.phase_frames = 0

        if self.phase == "run" and self.target_rpm != 0.0:
            if self._act_rpm() < NEAR_STOP_RPM:
                if self.stall_since is None:
                    self.stall_since = time.time()
                elif time.time() - self.stall_since >= STALL_RETRY_S:
                    self.enter_stop()
                    return self._step_stop()
            else:
                self.stall_since = None

        if self.phase == "preload":
            self.cmd_rpm = self.target_rpm
            asm = self.make_asm(self.target_rpm, start=False)
        elif self.phase == "start":
            self.cmd_rpm = self.target_rpm
            asm = self.make_asm(self.target_rpm, start=True)
        elif self.phase == "run":
            self.cmd_rpm = self.target_rpm
            asm = self.make_asm(self.target_rpm, start=False)
        else:
            self.cmd_rpm = 0.0
            asm = self.make_asm(0.0)

        self.phase_frames += 1
        return asm

    def _step_stop(self) -> bytes:
        """StopMotion pulse -> speed-0 hold -> SM edge to 0 if still spinning."""
        nearly_stopped = self._act_rpm() < NEAR_STOP_RPM or bool(self.last_st.get("stopped"))

        if self.stop_mode == "pulse":
            self.cmd_rpm = 0.0
            asm = self.make_asm(0.0, stop=True)
            self.phase_frames += 1
            if self.phase_frames >= STOP_PULSE_FRAMES:
                self.stop_mode = "hold"
                self.phase_frames = 0
            return asm

        if self.stop_mode == "hold":
            self.phase_frames += 1
            if nearly_stopped:
                rearm = self._finish_stop_or_rearm()
                if rearm is not None:
                    return rearm
                self.cmd_rpm = 0.0
                return self.make_asm(0.0, stop=False)
            # StopMotion alone often leaves the latched speed running — escalate fast.
            if (self.phase_frames >= STOP_ESCALATE_FRAMES
                    or self.phase_frames >= self._stop_timeout_frames()):
                self.stop_mode = "zero_preload"
                self.phase_frames = 0
            # Keep StopMotion asserted while still spinning (helps some cycles).
            self.cmd_rpm = 0.0
            return self.make_asm(0.0, stop=True)

        if self.stop_mode == "zero_preload":
            self.cmd_rpm = 0.0
            asm = self.make_asm(0.0, start=False)
            self.phase_frames += 1
            if self.phase_frames >= STOP_ZERO_PRELOAD_FRAMES:
                self.stop_mode = "zero_start"
                self.phase_frames = 0
            return asm

        self.cmd_rpm = 0.0
        asm = self.make_asm(0.0, start=True)
        self.phase_frames += 1
        if self.phase_frames >= STOP_ZERO_START_FRAMES:
            self.stop_mode = "hold"
            self.phase_frames = 0
        return asm

    def apply_feedback(self, assy: Optional[bytes]) -> Optional[str]:
        if assy is None:
            return None
        st = eip.parse_input_assembly_154(assy)
        if "error" in st:
            return None
        self.last_st = st
        return eip.get_drive_trip_reason(st)

    def close(self) -> None:
        if self.client is None:
            return
        self.client.io_sock = None
        try:
            if self.fo_params is not None:
                self.client.forward_close(self.fo_params)
        except Exception:
            pass
        try:
            self.client.unregister_session()
            self.client.disconnect()
        except Exception:
            pass
        self.client = None


def _status_block(axes: list[AxisJog]) -> str:
    parts = []
    for ax in axes:
        st = ax.last_st
        spd = int(st.get("actual_speed", 0)) / 10.0
        pos = st.get("actual_position", 0)
        stop_tag = ax.stop_mode if ax.phase == "stop" else "-"
        parts.append(
            f"{ax.name}[{ax.phase}/{stop_tag}] "
            f"cmd={ax.cmd_rpm:+.0f} act={spd:+.0f} pos={pos}"
        )
    line = "  " + " || ".join(parts)
    return "\r" + line.ljust(160)[:160]


def _dual_exchange(udp: socket.socket,
                   axes: list[AxisJog],
                   asms: list[bytes],
                   timeout: float = 0.05) -> dict[str, bytes]:
    for ax, asm in zip(axes, asms):
        udp.sendto(_build_ot_cpf(ax.ot_id, asm), (ax.host, eip.UDP_PORT))

    latest: dict[str, bytes] = {}
    hosts = {ax.host for ax in axes}
    udp.settimeout(timeout)
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            rx, addr = udp.recvfrom(4096)
        except (socket.timeout, OSError):
            break
        if addr[0] not in hosts:
            continue
        assy = _extract_assembly(rx)
        if assy is not None:
            latest[addr[0]] = assy

    udp.setblocking(False)
    try:
        while True:
            try:
                rx, addr = udp.recvfrom(4096)
            except (BlockingIOError, OSError):
                break
            if addr[0] not in hosts:
                continue
            assy = _extract_assembly(rx)
            if assy is not None:
                latest[addr[0]] = assy
    finally:
        udp.settimeout(timeout)
    return latest

def run_jog(speed_rpm: float,
            accel_rpm_per_s: float,
            decel_rpm_per_s: float,
            rpi_us: int,
            restore_hold: bool,
            x_ip: str,
            z_ip: str) -> int:
    hold_was_running = eip.get_background_hold_state() is not None
    resume_rpi = rpi_us
    if hold_was_running:
        state = eip.get_background_hold_state() or {}
        resume_rpi = int(state.get("rpi_us", rpi_us))
        if not eip.pause_background_hold_for_handoff("handoff:jog", timeout_s=8.0):
            print("[FAIL] Could not pause persistent servo hold.")
            return 1

    shared_udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    shared_udp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        shared_udp.bind(("0.0.0.0", eip.UDP_PORT))
    except OSError as exc:
        print(f"[FAIL] Could not bind UDP {eip.UDP_PORT}: {exc}")
        if hold_was_running and restore_hold:
            eip.start_background_hold(rpi_us=resume_rpi)
        return 1

    axes = [
        AxisJog("X", x_ip, speed_rpm, accel_rpm_per_s, decel_rpm_per_s, rpi_us,
                ot_connection_id_seed=0x10000001, connection_serial=0x0001),
        AxisJog("Z", z_ip, speed_rpm, accel_rpm_per_s, decel_rpm_per_s, rpi_us,
                ot_connection_id_seed=0x10000002, connection_serial=0x0002),
    ]

    opened: list[AxisJog] = []
    try:
        for ax in axes:
            err = ax.open(shared_udp)
            if err:
                print(f"[FAIL] {err}")
                for o in opened:
                    o.close()
                shared_udp.close()
                if hold_was_running and restore_hold:
                    time.sleep(eip.HOLD_OWNERSHIP_SETTLE_S)
                    eip.start_background_hold(rpi_us=resume_rpi)
                return 1
            opened.append(ax)
            print(f"  [OK] {ax.name} Class 1 open ({ax.host})")

        # Settle one axis at a time so T->O demux stays unambiguous.
        for ax in axes:
            ax.settle(shared_udp)

        print()
        print("  Live dual jog ready. Focus this console window.")
        print(f"  X  LEFT / RIGHT  →  ±{speed_rpm:.1f} RPM  ({x_ip})")
        print(f"  Z  A / D         →  ±{speed_rpm:.1f} RPM  ({z_ip})")
        print("  SPACE = stop both    Esc / Q = quit")
        print("  SAFETY: keep E-Stop reachable.\n")

        time.sleep(0.15)
        frame_dt = max(rpi_us / 1e6, 0.002)
        io_timeout = max(frame_dt * 1.5, 0.008)
        exit_code = 0
        running = True

        try:
            while running:
                t0 = time.perf_counter()
                if _key_down(VK_ESCAPE) or _key_down(VK_Q):
                    break

                if _key_down(VK_SPACE):
                    desired_x, desired_z = 0, 0
                else:
                    desired_x = _axis_intent(VK_LEFT, VK_RIGHT)
                    desired_z = _axis_intent(VK_A, VK_D)

                desires = [desired_x, desired_z]
                asms = [ax.step_fsm(d) for ax, d in zip(axes, desires)]
                rx_map = _dual_exchange(shared_udp, axes, asms, timeout=io_timeout)

                for ax in axes:
                    trip = ax.apply_feedback(rx_map.get(ax.host))
                    if trip is not None:
                        print(f"\n[FAIL] {ax.name} drive trip: {trip}")
                        exit_code = 1
                        running = False
                        break

                if not running:
                    break

                sys.stdout.write(_status_block(axes))
                sys.stdout.flush()
                # Pace to RPI — don't sleep a full frame on top of the I/O wait.
                elapsed = time.perf_counter() - t0
                remain = frame_dt - elapsed
                if remain > 0.0005:
                    time.sleep(remain)
        except KeyboardInterrupt:
            print("\n  Interrupted.")

        print("\n  Stopping...")
        # Match move stop: StopMotion pulse, then speed-0 hold, then SM edge to 0.
        for _ in range(8):
            asms = [ax.make_asm(0.0, stop=True) for ax in axes]
            _dual_exchange(shared_udp, axes, asms, timeout=io_timeout)
            time.sleep(frame_dt)
        for _ in range(6):
            asms = [ax.make_asm(0.0, start=False) for ax in axes]
            _dual_exchange(shared_udp, axes, asms, timeout=io_timeout)
            time.sleep(frame_dt)
        for _ in range(8):
            asms = [ax.make_asm(0.0, start=True) for ax in axes]
            _dual_exchange(shared_udp, axes, asms, timeout=io_timeout)
            time.sleep(frame_dt)
        for _ in range(10):
            asms = [ax.make_asm(0.0) for ax in axes]
            _dual_exchange(shared_udp, axes, asms, timeout=io_timeout)
            time.sleep(frame_dt)

        return exit_code
    finally:
        for ax in opened:
            ax.close()
        try:
            shared_udp.close()
        except Exception:
            pass
        if restore_hold:
            print("  Restoring persistent servo hold (X+Z)...")
            time.sleep(eip.HOLD_OWNERSHIP_SETTLE_CLEAN_S)
            if not eip.start_background_hold(rpi_us=resume_rpi):
                print("  [WARN] Could not restore hold. Run: py tools/eip_test.py servo-on")
            else:
                print("  [OK] Hold restored (X+Z).")


def main() -> int:
    if sys.platform != "win32":
        print("[ABORT] eip_jog.py currently supports Windows key polling only.")
        return 1

    ap = argparse.ArgumentParser(description="Live dual-axis arrow/A-D jog for Kinetix 5100")
    ap.add_argument("--speed", type=float, default=100.0,
                    help="Jog speed magnitude in RPM for both axes (default: 100)")
    ap.add_argument("--accel", type=float, default=5000.0,
                    help="Accel in RPM/s (default: 5000; max 3000000)")
    ap.add_argument("--decel", type=float, default=None,
                    help="Decel in RPM/s (default: same as --accel; max 3000000)")
    ap.add_argument("--rpi", type=int, default=2000,
                    help="Class 1 RPI in microseconds (default: 2000; lower = snappier)")
    ap.add_argument("--x-ip", default=DEFAULT_X_IP,
                    help=f"X-axis drive IP (default: {DEFAULT_X_IP})")
    ap.add_argument("--z-ip", default=DEFAULT_Z_IP,
                    help=f"Z-axis drive IP (default: {DEFAULT_Z_IP})")
    ap.add_argument("--no-restore-hold", action="store_true",
                    help="Do not restart persistent hold on exit")
    ap.add_argument("--no-log", action="store_true",
                    help="Disable rotating session log files")
    ap.add_argument("--log-versions", type=int, default=JOG_LOG_VERSIONS,
                    help=f"How many rotated log files to keep (default: {JOG_LOG_VERSIONS})")
    args = ap.parse_args()

    decel = args.accel if args.decel is None else args.decel

    if abs(args.speed) < 0.1:
        print("[ABORT] --speed must be >= 0.1 RPM")
        return 1
    err = eip.validate_accel_decel_ranges(args.accel, decel)
    if err:
        print(f"[ABORT] {err}")
        return 1

    log_file: Optional[TextIO] = None
    log_path: Optional[Path] = None
    prev_stdout = sys.stdout
    if not args.no_log:
        try:
            log_file, log_path = open_jog_session_log(versions=args.log_versions)
            sys.stdout = TeeStdout(sys.__stdout__, log_file)
        except OSError as exc:
            print(f"[WARN] Could not open jog log ({exc}); continuing without log.")
            log_file = None
            log_path = None

    try:
        print("=== EIP Live Dual Jog (X + Z) ===")
        if log_path is not None:
            print(f"  Session log: {log_path}")
            print(f"  Kept versions: {log_path.name}", end="")
            for i in range(1, max(1, args.log_versions)):
                print(f", {_jog_log_path(i).name}", end="")
            print()
        print(f"  X: {args.x_ip}   Left/Right arrows")
        print(f"  Z: {args.z_ip}   A / D")
        print(f"  Jog speed: ±{abs(args.speed):.1f} RPM")
        print(f"  Accel/Decel: {args.accel:.1f} / {decel:.1f} RPM/s")
        print("  SAFETY: Ensure both motors are clear and E-Stop is accessible!")
        try:
            input("  Press Enter to start (Ctrl+C to abort)... ")
        except (EOFError, KeyboardInterrupt):
            print("\nAborted.")
            return 1

        return run_jog(
            speed_rpm=abs(args.speed),
            accel_rpm_per_s=args.accel,
            decel_rpm_per_s=decel,
            rpi_us=args.rpi,
            restore_hold=not args.no_restore_hold,
            x_ip=args.x_ip,
            z_ip=args.z_ip,
        )
    finally:
        try:
            sys.stdout.flush()
        except Exception:
            pass
        sys.stdout = prev_stdout
        if log_file is not None:
            try:
                log_file.close()
            except Exception:
                pass
            if log_path is not None:
                print(f"  Session log saved: {log_path}")


if __name__ == "__main__":
    raise SystemExit(main())

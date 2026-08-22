#!/usr/bin/env python3
"""
EtherNet/IP Interactive Test Harness for Kinetix 5100 Drive Discovery.

Connect to a Kinetix 5100 drive at 192.168.1.20:44818 and iteratively
discover the correct ForwardOpen parameters. Uses only Python stdlib.

Usage:
    py tools/eip_test.py list-identity
    py tools/eip_test.py register-session
    py tools/eip_test.py forward-open
    py tools/eip_test.py forward-open --ot-size 46 --run-idle
    py tools/eip_test.py forward-open --ot-size 46 --run-idle --ot-instance 106
    py tools/eip_test.py unregister-session
    py tools/eip_test.py full-test

Persistent servo hold (reliability design)
------------------------------------------
`servo-on` (with no --hold-seconds) starts a detached background worker
(`run_background_hold_worker`) that keeps Class 1 connections open on both
bench axes (X=192.168.1.20 and Z=192.168.1.21) and streams hold frames so
torque is maintained after the command returns. One worker process owns both
drives on shared UDP 2222 (demux by source IP). `home`/`move`/`eip_jog`
briefly take ownership and the hold is restored afterwards; `servo-off` stops it.

Handoff ordering (do not regress): pause the hold worker and wait for ownership
settle BEFORE opening a new TCP session for home/move. Opening a competing TCP
session while the worker still owns Class 1 has been observed to abort the
worker's TCP socket (WinError 10053), so ForwardClose never reaches the drive
and the next ForwardOpen fails with ownership conflict 0x0106.

A Class 1 connection only holds torque while O->T frames keep arriving. If the
originator stalls longer than the connection timeout, the drive silently drops
the connection and releases torque WITH NO FAULT shown on the display. Two bugs
used to cause exactly that random "servo goes free" symptom, both now fixed:

  1. exchange_io_frame created/bound/closed a fresh UDP socket every frame. In
     the gap between close and the next bind, the drive's T->O unicast hit a
     closed port and Windows replied ICMP "port unreachable", so the drive tore
     the connection down. Fixed by reusing one persistent UDP socket for the
     whole hold (exchange_io_frame(..., reuse_socket=True, drain=True)).
  2. The worker rewrote its JSON state file (atomic temp+rename) on every frame
     (~100-200/s). On Windows that intermittently stalled for hundreds of ms
     (Defender/disk), exceeding the old ~320 ms connection timeout. Fixed by
     throttling disk I/O (stop poll ~100 ms, heartbeat ~300 ms) while frames
     keep flowing at the RPI, and by raising connection_timeout_multiplier to 7
     (RPI x512 ~= 2.5 s of slack).

The worker also logs to tools/.eip_servo_hold.log (start / error / exit markers)
instead of DEVNULL; `servo-on` and `check-mode` print its path for diagnosis.

Reliability extras (do not regress):
  * Start timeout must not delete the state file while the worker is still
    arming — that used to kill a healthy dual-axis start.
  * Already-open axes keep receiving O->T frames while the next axis
    ForwardOpens (keepalive thread).
  * Arm path clears latched A603 the same way move/home settle does.
  * After enough clean Active frames, fallback position hold promotes to
    neutral idle hold.
  * A detached watchdog restarts supervised hold if the heartbeat goes stale
    or Active drops (paused during intentional handoff via pause marker).
  * Hold worker raises process priority; `hold-setup` / servo-on try to add
    Windows Defender exclusions for tools/ state+log paths.
"""

import argparse
import csv
import json
import os
import re
import socket
import struct
import subprocess
import sys
import threading
import time
import uuid
from datetime import datetime
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

# Keep original print for the timestamp wrapper below.
_ORIGINAL_PRINT = print


def _log_timestamp() -> str:
    """Return local timestamp with millisecond precision."""
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


def _prefix_log_lines(message: str) -> str:
    """Prefix each non-empty line in a message with a timestamp."""
    if message == "":
        return f"[{_log_timestamp()}]"

    lines = message.split("\n")
    prefixed = []
    for line in lines:
        if line == "":
            prefixed.append(line)
        else:
            prefixed.append(f"[{_log_timestamp()}] {line}")
    return "\n".join(prefixed)


def print(*args, sep=" ", end="\n", file=None, flush=False):
    """Timestamped print used by all script logs."""
    if file is None:
        file = sys.stdout
    message = sep.join(str(arg) for arg in args)
    prefixed = _prefix_log_lines(message)
    try:
        _ORIGINAL_PRINT(prefixed, sep="", end=end, file=file, flush=flush)
    except UnicodeEncodeError:
        encoding = getattr(file, "encoding", None) or sys.getdefaultencoding()
        safe = prefixed.encode(encoding, errors="replace").decode(encoding)
        _ORIGINAL_PRINT(safe, sep="", end=end, file=file, flush=flush)

# --- Constants ---
DRIVE_IP_X = "192.168.1.20"
DRIVE_IP_Z = "192.168.1.21"
DRIVE_IP = DRIVE_IP_X  # default CLI target (single-drive commands)
HOLD_DRIVE_IPS = (DRIVE_IP_X, DRIVE_IP_Z)
EIP_PORT = 44818
UDP_PORT = 2222
DEFAULT_ACCEL_RPM_PER_S = 100.0
DEFAULT_DECEL_RPM_PER_S = 100.0
# EDS Accel/Decel defaults are 15000 RPM/s (Param27/28 default raw 150000).
# Use these for Home so mode entry matches the working bench profile.
DEFAULT_HOME_ACCEL_RPM_PER_S = 15000.0
DEFAULT_HOME_DECEL_RPM_PER_S = 15000.0
SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
DRIVE_CODES_CSV = REPO_ROOT / "driver_datasheets_and_calculations" / "2198-RD001_-en-p" / "2198-rd001_-en-p _ CODES.csv"
DEFAULT_EDS_PATH = REPO_ROOT / "docs" / "captures" / "eds_kinetix5100.eds"
HOLD_STATE_FILE = SCRIPT_DIR / ".eip_servo_hold_state.json"
HOLD_LOG_FILE = SCRIPT_DIR / ".eip_servo_hold.log"
HOLD_PAUSE_MARKER = SCRIPT_DIR / ".eip_servo_hold_paused"
HOLD_WATCHDOG_STATE_FILE = SCRIPT_DIR / ".eip_servo_hold_watchdog.json"
HOLD_WATCHDOG_LOG_FILE = SCRIPT_DIR / ".eip_servo_hold_watchdog.log"
HOLD_LOG_MAX_BYTES = 1_048_576
HOLD_STATE_STALE_S = 20.0
# After a failed ForwardClose the drive keeps Class 1 ownership until the
# connection timeout expires (hold uses multiplier=7 -> RPI x512 ~= 2.56 s).
# Callers that take ownership must wait at least this long before ForwardOpen.
HOLD_OWNERSHIP_SETTLE_S = 3.0
HOLD_OWNERSHIP_SETTLE_CLEAN_S = 0.35
# Consecutive clean Active/Ready frames before promoting fallback → idle hold.
HOLD_IDLE_PROMOTE_FRAMES = 50
# Watchdog restarts hold when heartbeat is older than this while supervised.
# Must exceed worst-case A603 clear / reacquire stalls in the worker loop.
HOLD_WATCHDOG_STALE_S = 6.0
HOLD_WATCHDOG_POLL_S = 1.0
RAMP_MIN_WINDOW_S = 0.005

_DRIVE_CODE_TABLE: Optional[dict] = None
_EDS_CACHE: dict[str, dict] = {}

# Input Assembly 154 boolean fields from the Kinetix 5100 EDS Assem2 layout.
DRIVE_INPUT_STATUS_FLAGS = (
    {"field": "run_mode", "byte": 0, "bit": 0, "display": "RunMode", "severity": "info"},
    {"field": "connection_faulted", "byte": 0, "bit": 1, "display": "ConnectionFaulted", "severity": "fault"},
    {"field": "diagnostic_active", "byte": 0, "bit": 2, "display": "DiagnosticActive", "severity": "diagnostic"},
    {"field": "fault", "byte": 8, "bit": 1, "display": "Fault", "severity": "fault"},
    {"field": "uncertain", "byte": 8, "bit": 2, "display": "Uncertain", "severity": "warning"},
    {"field": "warning_present", "byte": 9, "bit": 1, "display": "WarningPresent", "severity": "warning"},
    {"field": "active", "byte": 9, "bit": 2, "display": "Active", "severity": "info"},
    {"field": "ready", "byte": 9, "bit": 3, "display": "Ready", "severity": "info"},
    {"field": "command_in_progress", "byte": 9, "bit": 4, "display": "CommandInProgress", "severity": "info"},
    {"field": "homed_status", "byte": 9, "bit": 5, "display": "HomedStatus", "severity": "info"},
    {"field": "stopped", "byte": 9, "bit": 6, "display": "Stopped", "severity": "info"},
    {"field": "at_reference", "byte": 9, "bit": 7, "display": "AtReference", "severity": "info"},
)

# Encapsulation commands
ENCAP_LIST_IDENTITY = 0x0063
ENCAP_REGISTER_SESSION = 0x0065
ENCAP_UNREGISTER_SESSION = 0x0066
ENCAP_SEND_RR_DATA = 0x006F
ENCAP_SEND_UNIT_DATA = 0x0070

# CPF item types
CPF_NULL_ADDRESS = 0x0000
CPF_LIST_IDENTITY_RESPONSE = 0x000C
CPF_UNCONNECTED_DATA = 0x00B2
CPF_SEQUENCED_ADDRESS = 0x8002
CPF_CONNECTED_DATA = 0x00B1

# CIP service codes
CIP_FORWARD_OPEN = 0x54
CIP_FORWARD_CLOSE = 0x4E
CIP_GET_ATTRIBUTE_SINGLE = 0x0E
CIP_SET_ATTRIBUTE_SINGLE = 0x10

# CIP class IDs
CIP_CLASS_IDENTITY = 0x01
CIP_CLASS_ASSEMBLY = 0x04
CIP_CLASS_CONNECTION_MANAGER = 0x06

# CIP general status codes
CIP_STATUS_SUCCESS = 0x00
CIP_STATUS_CONNECTION_FAILURE = 0x01

# Connection types
CONN_TYPE_NULL = 0
CONN_TYPE_MULTICAST = 1
CONN_TYPE_P2P = 2
CONN_TYPE_RESERVED = 3

# Priority
PRIORITY_LOW = 0
PRIORITY_HIGH = 1
PRIORITY_SCHEDULED = 2
PRIORITY_URGENT = 3

OPERATING_MODE_NAMES = {
    0: "NotSpecified",
    1: "Position",
    2: "Speed",
    3: "Home",
    4: "Torque",
    5: "Gear",
    6: "Index",
    7: "ECAM",
}

MODE_USES_SPEED_REFERENCE = {1, 2, 3, 5}
MODE_USES_POSITION_REFERENCE = {1}
MODE_USES_TORQUE_REFERENCE = {4}
MODE_USES_INDEX_REFERENCE = {6, 7}


def _now_iso() -> str:
    return datetime.now().isoformat(timespec="seconds")


def _read_json_file(path: Path) -> Optional[dict]:
    try:
        if not path.exists():
            return None
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return None


def _write_json_file_atomic(path: Path, data: dict):
    """Atomically write JSON; tolerate Windows file locks (Defender / races)."""
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = json.dumps(data, indent=2, sort_keys=True)
    tmp = path.with_suffix(path.suffix + f".{os.getpid()}.tmp")
    last_exc: Optional[Exception] = None
    try:
        tmp.write_text(payload, encoding="utf-8")
        for attempt in range(8):
            try:
                # On Windows, Path.replace can raise WinError 5 if another
                # process still has the destination open. Unlink first, then
                # rename; fall back to a direct write if replace keeps failing.
                if os.name == "nt" and path.exists():
                    try:
                        path.unlink()
                    except FileNotFoundError:
                        pass
                    except PermissionError:
                        time.sleep(0.02 * (attempt + 1))
                        continue
                tmp.replace(path)
                return
            except PermissionError as exc:
                last_exc = exc
                time.sleep(0.02 * (attempt + 1))
        # Last resort: non-atomic overwrite so callers do not crash.
        try:
            path.write_text(payload, encoding="utf-8")
            return
        except Exception as exc:
            last_exc = exc
            raise
    finally:
        try:
            tmp.unlink(missing_ok=True)
        except Exception:
            pass
    if last_exc is not None:
        raise last_exc


def _is_process_alive(pid: int) -> bool:
    if pid <= 0:
        return False

    # IMPORTANT: os.kill(pid, 0) is unsafe on Windows because non-CTRL signals
    # may terminate the target process. Use Win32 query APIs instead.
    if os.name == "nt":
        try:
            import ctypes
            from ctypes import wintypes

            PROCESS_QUERY_LIMITED_INFORMATION = 0x1000
            STILL_ACTIVE = 259

            kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
            open_process = kernel32.OpenProcess
            open_process.argtypes = [wintypes.DWORD, wintypes.BOOL, wintypes.DWORD]
            open_process.restype = wintypes.HANDLE

            get_exit_code = kernel32.GetExitCodeProcess
            get_exit_code.argtypes = [wintypes.HANDLE, ctypes.POINTER(wintypes.DWORD)]
            get_exit_code.restype = wintypes.BOOL

            close_handle = kernel32.CloseHandle
            close_handle.argtypes = [wintypes.HANDLE]
            close_handle.restype = wintypes.BOOL

            handle = open_process(PROCESS_QUERY_LIMITED_INFORMATION, False, int(pid))
            if not handle:
                return False
            try:
                exit_code = wintypes.DWORD()
                if not get_exit_code(handle, ctypes.byref(exit_code)):
                    return False
                return int(exit_code.value) == STILL_ACTIVE
            finally:
                close_handle(handle)
        except Exception:
            return False

    # POSIX path
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    except OSError:
        return False
    return True


def _load_hold_state_raw() -> Optional[dict]:
    return _read_json_file(HOLD_STATE_FILE)


def _write_hold_state_raw(state: dict):
    _write_json_file_atomic(HOLD_STATE_FILE, state)


def _clear_hold_state_file():
    try:
        HOLD_STATE_FILE.unlink(missing_ok=True)
    except Exception:
        pass


def get_background_hold_state() -> Optional[dict]:
    state = _load_hold_state_raw()
    if not state:
        return None

    status = str(state.get("status", "") or "")
    # Terminal markers are not a live hold service.
    if status in ("stopped", "error"):
        return None

    stale = False
    updated_text = str(state.get("updated_at", "") or "")
    if updated_text:
        try:
            updated_at = datetime.fromisoformat(updated_text)
            age_s = (datetime.now() - updated_at).total_seconds()
            stale = age_s > HOLD_STATE_STALE_S
        except Exception:
            pass
    try:
        pid = int(state.get("pid", 0))
    except Exception:
        pid = 0

    # If heartbeat is fresh, prefer it over a potentially flaky PID probe.
    if not stale:
        return state

    if pid > 0 and _is_process_alive(pid):
        return state

    if pid <= 0 or not _is_process_alive(pid):
        _clear_hold_state_file()
        return None
    return state


def _update_hold_state_for_token(token: str, **updates) -> bool:
    for _ in range(3):
        state = _load_hold_state_raw()
        if state and state.get("token") == token:
            state.update(updates)
            state["updated_at"] = _now_iso()
            _write_hold_state_raw(state)
            return True
        time.sleep(0.01)
    return False


def _spawn_detached_python(command: list[str],
                           log_path: Optional[Path] = None) -> subprocess.Popen:
    log_handle = None
    if log_path is not None:
        try:
            log_handle = open(log_path, "a", encoding="utf-8", errors="replace")
        except Exception:
            log_handle = None
    kwargs = {
        "stdin": subprocess.DEVNULL,
        "stdout": log_handle if log_handle is not None else subprocess.DEVNULL,
        "stderr": subprocess.STDOUT if log_handle is not None else subprocess.DEVNULL,
        "cwd": str(REPO_ROOT),
        "close_fds": True,
    }
    if os.name == "nt":
        detached_process = 0x00000008
        create_new_process_group = 0x00000200
        create_no_window = 0x08000000
        kwargs["creationflags"] = detached_process | create_new_process_group | create_no_window
    try:
        return subprocess.Popen(command, **kwargs)
    finally:
        # The child has inherited its own duplicate of the handle; the parent's
        # copy is no longer needed.
        if log_handle is not None:
            try:
                log_handle.close()
            except Exception:
                pass


def request_background_hold_stop(reason: str) -> bool:
    state = get_background_hold_state()
    if not state:
        return False
    state["stop"] = True
    state["stop_reason"] = reason
    state["stop_requested_at"] = _now_iso()
    _write_hold_state_raw(state)
    return True


def stop_background_hold(reason: str, timeout_s: float = 8.0) -> bool:
    """Ask the hold worker to exit and wait until the process is gone.

    Returns True if no hold was running or the worker process exited in time.
    Does not by itself guarantee the drive has released Class 1 ownership — use
    pause_background_hold_for_handoff() when the next step is a ForwardOpen.
    """
    state = get_background_hold_state()
    if not state:
        return True

    pid = int(state.get("pid", 0))
    token = state.get("token")
    request_background_hold_stop(reason)
    deadline = time.time() + max(timeout_s, 0.1)
    forward_close_ok = None
    while time.time() < deadline:
        latest = _load_hold_state_raw()
        if latest and latest.get("token") == token:
            if latest.get("status") == "stopped":
                forward_close_ok = bool(latest.get("forward_close_ok", False))
            elif latest.get("status") == "error":
                forward_close_ok = bool(latest.get("forward_close_ok", False))
        if not _is_process_alive(pid):
            # Prefer the worker's final status write if present.
            latest = _load_hold_state_raw()
            if latest and latest.get("token") == token:
                if forward_close_ok is None:
                    forward_close_ok = bool(latest.get("forward_close_ok", False))
                # Leave a short-lived handoff marker so callers can settle correctly.
                latest["status"] = "stopped"
                latest["forward_close_ok"] = bool(forward_close_ok)
                latest["stopped_at"] = _now_iso()
                latest["updated_at"] = _now_iso()
                _write_hold_state_raw(latest)
            return True
        time.sleep(0.05)

    return False


def pause_background_hold_for_handoff(reason: str, timeout_s: float = 8.0) -> bool:
    """Stop the hold worker and wait until the drive can accept a new ForwardOpen.

    Critical ordering: call this BEFORE opening a new TCP session for the motion
    command. Opening a competing TCP session while the hold worker still owns the
    Class 1 connection has been observed to abort the worker's TCP socket
    (WinError 10053), so ForwardClose never reaches the drive and the next
    ForwardOpen fails with ownership conflict 0x0106.
    """
    state = get_background_hold_state()
    if not state:
        return True

    print(f"  Pausing persistent servo hold ({reason})...")
    # Tell the watchdog not to revive hold during intentional handoff.
    _set_hold_supervise(False, reason=reason)
    try:
        HOLD_PAUSE_MARKER.write_text(f"{reason}\n{_now_iso()}\n", encoding="utf-8")
    except Exception:
        pass

    if not stop_background_hold(reason, timeout_s=timeout_s):
        print("  [FAIL] Background hold process did not exit in time.")
        return False

    latest = _load_hold_state_raw()
    close_ok = bool(latest.get("forward_close_ok", False)) if latest else False
    settle_s = HOLD_OWNERSHIP_SETTLE_CLEAN_S if close_ok else HOLD_OWNERSHIP_SETTLE_S
    if close_ok:
        print(f"  Hold released cleanly; settling {settle_s:.2f}s before ForwardOpen...")
    else:
        print(f"  [WARN] Hold ForwardClose did not complete; waiting {settle_s:.1f}s "
              f"for drive connection timeout to free ownership...")
    time.sleep(settle_s)
    _clear_hold_state_file()
    return True


def _set_hold_supervise(enabled: bool, reason: str = "") -> None:
    state = _load_hold_state_raw()
    if not state:
        return
    state["supervise"] = bool(enabled)
    state["supervise_reason"] = reason
    state["supervise_updated_at"] = _now_iso()
    state["updated_at"] = _now_iso()
    _write_hold_state_raw(state)


def _hold_pause_marker_active() -> bool:
    return HOLD_PAUSE_MARKER.exists()


def _clear_hold_pause_marker() -> None:
    try:
        HOLD_PAUSE_MARKER.unlink(missing_ok=True)
    except Exception:
        pass


def _boost_current_process_priority() -> None:
    """Raise hold-worker scheduling priority so the RPI loop is less starved."""
    if os.name != "nt":
        try:
            os.nice(-5)
        except Exception:
            pass
        return
    try:
        import ctypes
        k32 = ctypes.windll.kernel32
        # HIGH_PRIORITY_CLASS — above normal without starving the whole machine.
        k32.SetPriorityClass(k32.GetCurrentProcess(), 0x00000080)
    except Exception as exc:
        print(f"[hold worker] priority boost skipped: {exc}")


def ensure_hold_defender_exclusion() -> bool:
    """Best-effort Windows Defender exclusion for hold/jog state+log files.

    Requires elevation for Add-MpPreference. Returns True if exclusion is
    present or was added; False if skipped/failed (hold still works).
    """
    if os.name != "nt":
        return False
    paths = [
        str(SCRIPT_DIR),
        str(HOLD_STATE_FILE),
        str(HOLD_LOG_FILE),
        str(SCRIPT_DIR / ".eip_jog.log"),
    ]
    ps_paths = ",".join(f"'{p}'" for p in paths)
    script = (
        f"$paths = @({ps_paths}); "
        "try { "
        "  $prefs = Get-MpPreference -ErrorAction Stop; "
        "  foreach ($p in $paths) { "
        "    if ($prefs.ExclusionPath -notcontains $p) { "
        "      Add-MpPreference -ExclusionPath $p -ErrorAction Stop "
        "    } "
        "  }; "
        "  'OK' "
        "} catch { "
        "  $_.Exception.Message "
        "}"
    )
    try:
        proc = subprocess.run(
            ["powershell", "-NoProfile", "-Command", script],
            capture_output=True, text=True, timeout=20, check=False)
        out = (proc.stdout or "").strip()
        if out.endswith("OK") or out == "OK":
            print("  [OK] Defender exclusion present for tools/ hold+jog paths.")
            return True
        print(f"  [WARN] Defender exclusion not applied ({out or 'unknown'}). "
              f"Run as Admin: py tools/eip_test.py hold-setup")
        return False
    except Exception as exc:
        print(f"  [WARN] Defender exclusion check failed: {exc}")
        return False


def start_background_hold(rpi_us: int = 5000, wait_start_s: float = 25.0,
                          start_watchdog: bool = True) -> bool:
    state = get_background_hold_state()
    if state:
        status = str(state.get("status", ""))
        # A leftover "stopped"/"error" marker is not a live hold — clear and restart.
        if status in ("stopped", "error"):
            _clear_hold_state_file()
        else:
            pid = int(state.get("pid", 0))
            _set_hold_supervise(True, reason="already-running")
            _clear_hold_pause_marker()
            if start_watchdog:
                start_hold_watchdog(rpi_us=int(state.get("rpi_us", rpi_us)))
            print(f"  [OK] Background servo hold already running (pid={pid}).")
            return True

    token = uuid.uuid4().hex
    initial_state = {
        "token": token,
        "pid": 0,
        "rpi_us": int(rpi_us),
        "status": "starting",
        "stop": False,
        "supervise": True,
        "created_at": _now_iso(),
        "updated_at": _now_iso(),
    }
    _write_hold_state_raw(initial_state)

    command = [
        sys.executable,
        str(Path(__file__).resolve()),
        "__hold-worker",
        "--token",
        token,
        "--rpi",
        str(int(rpi_us)),
    ]

    # Keep the diagnostic log from growing without bound across sessions.
    try:
        if HOLD_LOG_FILE.exists() and HOLD_LOG_FILE.stat().st_size > HOLD_LOG_MAX_BYTES:
            HOLD_LOG_FILE.unlink(missing_ok=True)
    except Exception:
        pass

    try:
        proc = _spawn_detached_python(command, log_path=HOLD_LOG_FILE)
    except Exception as exc:
        _clear_hold_state_file()
        print(f"  [FAIL] Could not start background hold process: {exc}")
        return False

    initial_state["pid"] = int(proc.pid)
    initial_state["updated_at"] = _now_iso()
    _write_hold_state_raw(initial_state)

    def _report_holding(state_now: dict) -> bool:
        active = bool(state_now.get("active", False))
        active_seen = bool(state_now.get("active_seen", False)) or active
        if not active_seen:
            return False
        axes = state_now.get("axes") or {}
        axis_bits = []
        for name in ("X", "Z"):
            ax = axes.get(name) or {}
            if ax:
                axis_bits.append(
                    f"{name}={ax.get('host', '?')} "
                    f"active={bool(ax.get('active', False))}")
        axis_txt = (", ".join(axis_bits) if axis_bits else "X+Z")
        print(f"  [OK] Background servo hold running "
              f"(pid={proc.pid}, active={active}, {axis_txt}).")
        print(f"  Diagnostic log: {HOLD_LOG_FILE}")
        return True

    deadline = time.time() + max(wait_start_s, 0.5)
    while time.time() < deadline:
        state = _load_hold_state_raw()
        if state and state.get("token") == token:
            status = str(state.get("status", ""))
            if status == "holding" and _report_holding(state):
                _clear_hold_pause_marker()
                _set_hold_supervise(True, reason="hold-started")
                if start_watchdog:
                    start_hold_watchdog(rpi_us=rpi_us)
                return True
            if status == "error":
                err = str(state.get('last_error', 'unknown error'))
                print(f"  [FAIL] Background hold failed: {err}")
                print(f"  See diagnostic log: {HOLD_LOG_FILE}")
                if "fault" in err.lower() or "e60a" in err.lower():
                    print("  Hint: clear the drive fault first, then retry:")
                    print("    py tools/eip_test.py clear-fault")
                    print("    py tools/eip_test.py decode E60A")
                # Worker already marked error; safe to clear after it exits.
                if not _is_process_alive(proc.pid):
                    _clear_hold_state_file()
                else:
                    request_background_hold_stop("start-error")
                return False
        if not _is_process_alive(proc.pid):
            break
        time.sleep(0.1)

    # Timed out waiting for "holding". Do NOT delete the state file while the
    # worker is still alive — that used to kill a healthy dual-axis arm.
    state = _load_hold_state_raw()
    if state and state.get("token") == token and _is_process_alive(proc.pid):
        status = str(state.get("status", ""))
        if status == "holding" and _report_holding(state):
            _clear_hold_pause_marker()
            _set_hold_supervise(True, reason="hold-late")
            if start_watchdog:
                start_hold_watchdog(rpi_us=rpi_us)
            return True
        if status in ("starting", "arming", "recovering"):
            print(f"  [WARN] Hold still {status} after {wait_start_s:.0f}s; "
                  f"leaving worker running (pid={proc.pid}).")
            print(f"  Diagnostic log: {HOLD_LOG_FILE}")
            print("  Re-check with: py tools/eip_test.py check-mode")
            _clear_hold_pause_marker()
            _set_hold_supervise(True, reason="hold-arming-late")
            if start_watchdog:
                start_hold_watchdog(rpi_us=rpi_us)
            # Soft-success: worker is progressing; watchdog will restart if it dies.
            return True

        print("  [FAIL] Background hold did not become healthy; stopping worker...")
        request_background_hold_stop("start-timeout")
        stop_deadline = time.time() + 5.0
        while time.time() < stop_deadline and _is_process_alive(proc.pid):
            time.sleep(0.1)
        if not _is_process_alive(proc.pid):
            _clear_hold_state_file()
        return False

    _clear_hold_state_file()
    print("  [FAIL] Background hold did not report healthy state in time.")
    return False


def forward_open_with_retry(client: "EipClient", params: "ForwardOpenParams",
                            retries: int = 4,
                            retry_delay_s: float = 0.75) -> Optional[dict]:
    """ForwardOpen, retrying ownership-conflict (0x0106) after a short settle."""
    last = None
    for attempt in range(max(1, retries)):
        last = client.forward_open(params)
        if last is None:
            return None
        ot_conn_id = int(last.get("ot_connection_id", 0) or 0)
        if ot_conn_id != 0:
            return last
        ext = int(last.get("extended_status", 0) or 0)
        if ext != 0x0106 or attempt + 1 >= retries:
            return last
        print(f"  [WARN] Ownership conflict (0x0106); retry "
              f"{attempt + 2}/{retries} after {retry_delay_s:.2f}s...")
        time.sleep(retry_delay_s)
    return last


@dataclass
class ForwardOpenParams:
    """Configurable ForwardOpen parameters."""
    # Assembly instances
    config_instance: int = 191        # 0xBF
    ot_instance: int = 104           # Output assembly
    to_instance: int = 154           # Input assembly

    # Sizes (assembly data only, CIP seq count and Run/Idle added automatically)
    ot_assembly_size: int = 40
    to_assembly_size: int = 52

    # Raw connection size overrides (if set, bypass assembly+2+4 calculation)
    raw_ot_conn_size: Optional[int] = None
    raw_to_conn_size: Optional[int] = None

    # RPI
    ot_rpi_us: int = 5000
    to_rpi_us: int = 5000

    # Connection flags
    include_run_idle_header: bool = False
    run_idle_bit_only: bool = False  # Set bit 8 in net params but don't add 4 to conn size
    to_connection_type: int = CONN_TYPE_P2P  # 2=PointToPoint, 1=Multicast

    # IDs
    ot_connection_id: int = 0x10000001
    to_connection_id: int = 0
    connection_serial: int = 0x0001
    originator_vendor_id: int = 1
    originator_serial: int = 0xCAFEB00D

    # Timeouts
    priority_time_tick: int = 0
    timeout_ticks: int = 100
    connection_timeout_multiplier: int = 4


# --- Packet Helpers ---

def le16(v: int) -> bytes:
    return struct.pack("<H", v & 0xFFFF)

def le32(v: int) -> bytes:
    return struct.pack("<I", v & 0xFFFFFFFF)

def build_encap_header(command: int, data: bytes, session_handle: int = 0) -> bytes:
    """Build a 24-byte EtherNet/IP encapsulation header + data."""
    hdr = bytearray()
    hdr += le16(command)           # command
    hdr += le16(len(data))         # length
    hdr += le32(session_handle)    # session handle
    hdr += le32(0)                 # status
    hdr += b'\x00' * 8             # sender context
    hdr += le32(0)                 # options
    return bytes(hdr) + data


def parse_encap_header(frame: bytes) -> tuple[int, int, int, int, int]:
    """Parse encap header. Returns (command, length, session_handle, status, data_offset)."""
    cmd, dlen, sh, st = struct.unpack_from("<HHII", frame, 0)
    # skip sender_context(8) + options(4) = 12 bytes after status
    data_offset = 24
    return cmd, dlen, sh, st, data_offset


def build_cpf(items: list[tuple[int, bytes]]) -> bytes:
    """Build CPF: item_count(u16) + for each: type_id(u16) + length(u16) + data."""
    buf = bytearray()
    buf += le16(len(items))
    for type_id, data in items:
        buf += le16(type_id)
        buf += le16(len(data))
        buf += data
    return bytes(buf)


def parse_cpf(data: bytes) -> list[tuple[int, bytes]]:
    """Parse CPF items. Returns list of (type_id, item_data)."""
    items = []
    if len(data) < 2:
        return items
    count = struct.unpack_from("<H", data, 0)[0]
    offset = 2
    for _ in range(count):
        if offset + 4 > len(data):
            break
        type_id, item_len = struct.unpack_from("<HH", data, offset)
        offset += 4
        if offset + item_len > len(data):
            break
        item_data = data[offset:offset + item_len]
        offset += item_len
        items.append((type_id, item_data))
    return items


def build_send_rr_data(cip_message: bytes, session_handle: int = 0) -> bytes:
    """Build SendRRData payload wrapping a CIP message."""
    payload = bytearray()
    payload += le32(0)   # interface handle (0 = CIP)
    payload += le16(0)   # timeout (0 = rely on TCP)

    items = [
        (CPF_NULL_ADDRESS, b''),
        (CPF_UNCONNECTED_DATA, cip_message),
    ]
    payload += build_cpf(items)

    return build_encap_header(ENCAP_SEND_RR_DATA, bytes(payload), session_handle)


def build_epath(class_id: int, instance_id: int, has_attr: bool = False,
                attr_id: int = 0) -> bytes:
    """Build a CIP EPATH: class + instance [+ attribute]."""
    def append_logical(buf: bytearray, type8: int, type16: int, value: int):
        if value <= 0xFF:
            buf.append(type8)
            buf.append(value & 0xFF)
        else:
            buf.append(type16)
            buf.append(0)  # pad
            buf += le16(value)

    ep = bytearray()
    append_logical(ep, 0x20, 0x21, class_id)
    append_logical(ep, 0x24, 0x25, instance_id)
    if has_attr:
        append_logical(ep, 0x30, 0x31, attr_id)
    return bytes(ep)


def build_mr_request(service: int, epath: bytes, request_data: bytes = b'') -> bytes:
    """Build a CIP Message Router request."""
    buf = bytearray()
    buf.append(service & 0xFF)
    buf.append(len(epath) // 2)  # path size in 16-bit words
    buf += epath
    buf += request_data
    return bytes(buf)


def parse_mr_response(data: bytes) -> dict:
    """Parse CIP Message Router response. Returns dict with keys:
    reply_service, general_status, additional_status (bytes), response_data (bytes).
    """
    if len(data) < 4:
        return {"error": "response too short"}

    reply_service = data[0]
    reserved = data[1]
    general_status = data[2]
    add_status_words = data[3]

    add_status_len = add_status_words * 2
    offset = 4
    additional_status = data[offset:offset + add_status_len] if offset + add_status_len <= len(data) else b''
    offset += add_status_len
    response_data = data[offset:] if offset < len(data) else b''

    return {
        "reply_service": reply_service,
        "general_status": general_status,
        "additional_status": additional_status,
        "response_data": response_data,
        "is_success": general_status == CIP_STATUS_SUCCESS,
    }


def make_network_connection_params(connection_size: int, conn_type: int,
                                   priority: int, include_run_idle: bool,
                                   variable_size: bool = False,
                                   redundant_owner: bool = False) -> int:
    """Build the 16-bit Network Connection Parameters field.

    Note: for this harness, Run/Idle is represented by including the 32-bit
    Run/Idle header in connected O->T payloads and sizing the connection
    accordingly. We intentionally do not map include_run_idle to any size bit
    in this field because bits 0..8 are the connection size and Kinetix checks
    that value strictly (extended status 0x0127 on mismatch).
    """
    params = connection_size & 0x01FF  # bits 0-8
    _ = include_run_idle  # kept for API compatibility (see note above)
    if variable_size:
        params |= (1 << 9)
    params |= ((priority & 0x3) << 10)
    params |= ((conn_type & 0x3) << 13)
    if redundant_owner:
        params |= (1 << 15)
    return params


def make_transport_class_trigger(transport_class: int = 1, trigger: int = 0) -> int:
    """Build Transport Class/Trigger byte."""
    return (transport_class & 0x0F) | ((trigger & 0x07) << 4)


def build_connection_path(config_instance: int, ot_instance: int,
                          to_instance: int) -> bytes:
    """Build the ForwardOpen connection path.

    Format: class=0x04 (Assembly), config_instance, OT connection point, TO connection point.
    Uses 8-bit or 16-bit segment encoding depending on value magnitude.
    """
    def append_seg(buf: bytearray, type8: int, type16: int, value: int):
        if value <= 0xFF:
            buf.append(type8)
            buf.append(value & 0xFF)
        else:
            buf.append(type16)
            buf.append(0)
            buf += le16(value)

    path = bytearray()
    path.append(0x20)  # 8-bit class segment
    path.append(CIP_CLASS_ASSEMBLY)  # class 4

    # Config assembly instance
    append_seg(path, 0x24, 0x25, config_instance)

    # O->T connection point
    append_seg(path, 0x2C, 0x2D, ot_instance)

    # T->O connection point
    append_seg(path, 0x2C, 0x2D, to_instance)

    return bytes(path)


def build_forward_open_request(params: ForwardOpenParams) -> bytes:
    """Build the ForwardOpen CIP request data."""
    # CIP Class 1 transport: connection size = 2 (sequence count) + assembly data
    # + optional 4 (Run/Idle header for O->T)
    SEQ_COUNT_SIZE = 2
    RUN_IDLE_SIZE = 4

    if params.raw_ot_conn_size is not None:
        ot_conn_size = params.raw_ot_conn_size
    else:
        ot_conn_size = params.ot_assembly_size + SEQ_COUNT_SIZE
        if params.include_run_idle_header:
            ot_conn_size += RUN_IDLE_SIZE

    if params.raw_to_conn_size is not None:
        to_conn_size = params.raw_to_conn_size
    else:
        to_conn_size = params.to_assembly_size + SEQ_COUNT_SIZE

    # Run/Idle header usage is represented in payload size (+4), not by a
    # dedicated network-parameter bit for this profile.
    set_run_idle_bit = params.run_idle_bit_only

    ot_net_params = make_network_connection_params(
        ot_conn_size, CONN_TYPE_P2P, PRIORITY_SCHEDULED, set_run_idle_bit)

    to_net_params = make_network_connection_params(
        to_conn_size, params.to_connection_type, PRIORITY_SCHEDULED,
        False)  # T->O never has Run/Idle

    transport_ct = make_transport_class_trigger(1, 0)

    conn_path = build_connection_path(
        params.config_instance, params.ot_instance, params.to_instance)

    # ForwardOpen request data (ODVA Vol 1, Table 3-5.11)
    data = bytearray()
    data.append(params.priority_time_tick & 0xFF)
    data.append(params.timeout_ticks & 0xFF)
    data += le32(params.ot_connection_id)
    data += le32(params.to_connection_id)
    data += le16(params.connection_serial)
    data += le16(params.originator_vendor_id)
    data += le32(params.originator_serial)
    data.append(params.connection_timeout_multiplier & 0xFF)
    data += b'\x00' * 3          # reserved
    data += le32(params.ot_rpi_us)
    data += le16(ot_net_params)
    data += le32(params.to_rpi_us)
    data += le16(to_net_params)
    data.append(transport_ct & 0xFF)
    data.append(len(conn_path) // 2)  # path size in words
    data += conn_path

    return bytes(data)


def build_forward_close_request(params: ForwardOpenParams) -> bytes:
    """Build the ForwardClose CIP request data."""
    conn_path = build_connection_path(
        params.config_instance, params.ot_instance, params.to_instance)

    data = bytearray()
    data.append(params.priority_time_tick & 0xFF)
    data.append(params.timeout_ticks & 0xFF)
    data += le16(params.connection_serial)
    data += le16(params.originator_vendor_id)
    data += le32(params.originator_serial)
    data.append(len(conn_path) // 2)  # path size in words
    data += conn_path

    return bytes(data)


def parse_forward_open_reply(data: bytes) -> dict:
    """Parse a successful ForwardOpen reply."""
    if len(data) < 26:
        return {"error": "reply too short", "raw_len": len(data)}

    result = {}
    result["ot_connection_id"] = struct.unpack_from("<I", data, 0)[0]
    result["to_connection_id"] = struct.unpack_from("<I", data, 4)[0]
    result["connection_serial"] = struct.unpack_from("<H", data, 8)[0]
    result["originator_vendor_id"] = struct.unpack_from("<H", data, 10)[0]
    result["originator_serial"] = struct.unpack_from("<I", data, 12)[0]
    result["ot_api_us"] = struct.unpack_from("<I", data, 16)[0]
    result["to_api_us"] = struct.unpack_from("<I", data, 20)[0]

    app_reply_words = data[24]
    app_reply_offset = 26
    app_reply_len = app_reply_words * 2
    if app_reply_offset + app_reply_len <= len(data):
        result["application_reply"] = data[app_reply_offset:app_reply_offset + app_reply_len]

    return result


def format_hex(data: bytes, max_len: int = 64) -> str:
    """Format bytes as hex string, truncated if needed."""
    if len(data) <= max_len:
        return ' '.join(f'{b:02X}' for b in data)
    return ' '.join(f'{b:02X}' for b in data[:max_len]) + f' ... ({len(data)} bytes)'


def extended_status_description(ext_status: int) -> str:
    """Return a human-readable description of common extended status codes."""
    descriptions = {
        0x0100: "Connection in use / duplicate ForwardOpen",
        0x0103: "Transport class and trigger combination not supported",
        0x0106: "Ownership conflict",
        0x0107: "Target connection not found",
        0x0108: "Invalid network connection parameter",
        0x0109: "Invalid connection size",
        0x0110: "Target for connection not configured",
        0x0111: "RPI not supported",
        0x0112: "RPI value(s) not acceptable (often too fast; try 2000-5000 us)",
        0x0113: "Out of connections",
        0x0114: "Vendor ID or product code mismatch / Invalid segment type",
        0x0115: "Device type mismatch",
        0x0116: "Revision mismatch",
        0x0117: "Invalid produced or consumed application path",
        0x0118: "Invalid or inconsistent configuration application path",
        0x0119: "Non-listen-only connection not opened",
        0x011A: "Target object out of connections",
        0x0126: "Invalid configuration size",
        0x0127: "Invalid Originator to Target (O->T) size",
        0x0128: "Invalid Target to Originator (T->O) size",
        0x0129: "Invalid configuration application path",
    }
    if ext_status in descriptions:
        return descriptions[ext_status]
    return f"Unknown extended status"


# --- High-Level Operations ---

def build_output_assembly_104(servo_on: bool = False,
                               servo_off: bool = False,
                               speed_rpm: float = 0.0,
                               accel_rpm_per_s: float = DEFAULT_ACCEL_RPM_PER_S,
                               decel_rpm_per_s: float = DEFAULT_DECEL_RPM_PER_S,
                               operating_mode: int = 0,
                               position_puu: int = 0,
                               torque_percent: float = 0.0,
                               torque_ramp_time_ms: int = 0,
                               starting_index: int = 0,
                               non_cyclic_move_type: int = 0,
                               cyclic_move_type: int = 0,
                               travel_mode: int = 10,
                               position_command_override: bool = False,
                               position_command_overlap: bool = False,
                               captured_position_select: bool = False,
                               start_motion: bool = False,
                               stop_motion: bool = False,
                               fault_reset: bool = False,
                               homing_method: int = 0,
                               home_return_speed_rpm: float = 0.0) -> bytes:
    """Build a 40-byte Kinetix 5100 Output Assembly 104 frame.

    OperatingMode enum (UM004D Table 104/106, confirmed against Input
    Assembly 154 "Operating mode (input)" which mirrors the same values):
        0 = Mode not specified   3 = Home mode     6 = Index mode
        1 = Position mode        4 = Torque mode   7 = ECAM mode
        2 = Speed mode           5 = Gear mode

    Args:
        servo_on: Enable servo (bit 0 of control byte)
        servo_off: Disable servo (bit 1 of control byte)
        speed_rpm: Speed reference in RPM (stored as 0.1 RPM units)
        accel_rpm_per_s: Accel reference in RPM/s (stored as 0.1 RPM/s units)
        decel_rpm_per_s: Decel reference in RPM/s (stored as 0.1 RPM/s units)
        operating_mode: See OperatingMode enum above (use 3 to home, not 6)
        position_puu: Position reference in PUU (bytes 16-19, Position mode)
        torque_percent: Torque reference in % rated (0.1% units, bytes 28-31)
        torque_ramp_time_ms: Torque ramp time in ms (bytes 32-35)
        starting_index: Start index for Index/ECAM modes (byte 36)
        non_cyclic_move_type: Byte 24 non-cyclic move type
        cyclic_move_type: Byte 25 cyclic move type
        travel_mode: Byte 26 travel mode (default 10 = cyclic)
        position_command_override: Byte 27 bit 0
        position_command_overlap: Byte 27 bit 1
        captured_position_select: Byte 27 bit 2
        start_motion: Rising-edge trigger for motion / start homing (bit 4 of control byte)
        stop_motion: Stop motion immediately (bit 2 of control byte)
        fault_reset: Reset drive fault (bit 3 of control byte)
        homing_method: Homing method 0-38 written to byte 3 (UM004D Table 112,
            "Homing Method Values - IO Mode"). Method 34 = define current
            position as origin (no extra wiring needed). Methods 0-33 need a
            limit switch or ORG (home) switch wired. Methods 35-38 use
            torque-based collision detection and require DI.Enable Homing.
        home_return_speed_rpm: Creep speed for return-to-home phase in RPM (bytes 20-23)
    """
    buf = bytearray(40)

    # byte 0: operating_mode (i8) — in I/O mode this is AOP-controlled
    buf[0] = operating_mode & 0xFF

    # byte 1: control bits
    control = 0
    if servo_on:
        control |= (1 << 0)
    if servo_off:
        control |= (1 << 1)
    if stop_motion:
        control |= (1 << 2)
    if fault_reset:
        control |= (1 << 3)
    if start_motion:
        control |= (1 << 4)
    buf[1] = control

    # byte 2: reserved
    # byte 3: homing_method (i8)
    buf[3] = homing_method & 0xFF

    # bytes 4-7: speed_reference (i32, 0.1 RPM)
    speed_raw = int(speed_rpm * 10)
    struct.pack_into("<i", buf, 4, speed_raw)

    # bytes 8-11: accel_reference (i32, 0.1 RPM/s)
    # bytes 12-15: decel_reference (i32, 0.1 RPM/s)
    accel_raw = int(accel_rpm_per_s * 10)
    decel_raw = int(decel_rpm_per_s * 10)
    struct.pack_into("<i", buf, 8, accel_raw)
    struct.pack_into("<i", buf, 12, decel_raw)

    # bytes 16-19: position_reference (i32, PUU)
    struct.pack_into("<i", buf, 16, int(position_puu))
    # bytes 20-23: home_return_speed (i32, 0.1 RPM). EDS Param31 min=1;
    # keep 0 for non-home frames only when the caller left the default unset —
    # Home mode must pass a positive value (see make_home_asm / move home path).
    home_return_raw = int(home_return_speed_rpm * 10)
    if home_return_raw < 0:
        home_return_raw = 0
    struct.pack_into("<i", buf, 20, home_return_raw)
    # byte 24: non_cyclic_move_type
    buf[24] = non_cyclic_move_type & 0xFF
    # byte 25: cyclic_move_type
    buf[25] = cyclic_move_type & 0xFF
    # byte 26: travel_mode — UM004D: 2=non-cyclic, 10=cyclic; 0/1 reserved (A603)
    buf[26] = travel_mode & 0xFF
    # byte 27: flags
    flags = 0
    if position_command_override:
        flags |= (1 << 0)
    if position_command_overlap:
        flags |= (1 << 1)
    if captured_position_select:
        flags |= (1 << 2)
    buf[27] = flags
    # bytes 28-31: torque_reference (i32, 0.1% rated)
    struct.pack_into("<i", buf, 28, int(torque_percent * 10))
    # bytes 32-35: torque_ramp_time (i32, ms). EDS Param34 min=1; a zero value
    # is out of range and is a known trigger for A603 (Invalid I/O command) when
    # the drive validates the full command image (notably Home / Position).
    ramp_ms = int(torque_ramp_time_ms)
    if ramp_ms < 1:
        ramp_ms = 1000
    struct.pack_into("<i", buf, 32, ramp_ms)
    # byte 36: starting_index
    buf[36] = starting_index & 0xFF
    # bytes 37-39: reserved pad

    return bytes(buf)


def parse_input_assembly_154(data: bytes) -> dict:
    """Parse a 52-byte Kinetix 5100 Input Assembly 154 frame."""
    if len(data) < 52:
        return {"error": f"data too short: {len(data)} bytes"}

    status = {
        "status_byte_0": data[0],
        "status_byte_8": data[8],
        "status_byte_9": data[9],
        "diagnostic_sequence_count": struct.unpack_from("<b", data, 1)[0],
        "operating_mode": struct.unpack_from("<b", data, 11)[0],
        "active_index": struct.unpack_from("<b", data, 12)[0],
        "motor_type": struct.unpack_from("<b", data, 15)[0],
        "actual_speed": struct.unpack_from("<i", data, 16)[0],
        "fault_code": struct.unpack_from("<H", data, 20)[0],
        "warning_code": struct.unpack_from("<H", data, 22)[0],
        "actual_position": struct.unpack_from("<i", data, 24)[0],
        "actual_torque": struct.unpack_from("<i", data, 28)[0],
    }
    active_flags = []
    for flag in DRIVE_INPUT_STATUS_FLAGS:
        is_set = bool(data[flag["byte"]] & (1 << flag["bit"]))
        status[flag["field"]] = is_set
        if is_set:
            active_flags.append(flag["field"])
    status["active_status_flags"] = active_flags
    return status


def format_alarm_display_code(code: int) -> str:
    """Format warning/alarm code as the drive display style (Axxx)."""
    return f"A{code & 0x0FFF:03X}"


def get_drive_trip_reason(status: dict) -> Optional[str]:
    """Return a fail-fast trip reason for any active drive alarm/fault state."""
    if "error" in status:
        return status["error"]

    connection_faulted = bool(status.get("connection_faulted", False))
    diagnostic_active = bool(status.get("diagnostic_active", False))
    fault = bool(status.get("fault", False))
    uncertain = bool(status.get("uncertain", False))
    warning_present = bool(status.get("warning_present", False))
    fault_code = int(status.get("fault_code", 0))
    warning_code = int(status.get("warning_code", 0))

    if connection_faulted:
        return f"status flag active: {format_status_flags_summary(status, severity_filter=('fault',))}"
    # Treat fault/alarm as active based on status bits. Some drives may keep
    # non-zero code words latched even when the corresponding present bit is 0.
    if fault:
        detail = format_drive_code_detail(fault_code, prefer_prefix="E")
        return (f"fault active (fault={fault}, fault_code=0x{fault_code:04X}, {detail}; "
                f"{format_status_flags_summary(status)})")
    if diagnostic_active:
        return f"diagnostic flag active: {format_status_flags_summary(status, severity_filter=('diagnostic',))}"
    if uncertain:
        return f"uncertain status flag active: {format_status_flags_summary(status, severity_filter=('warning',))}"
    if warning_present:
        detail = format_drive_code_detail(warning_code, prefer_prefix="A")
        return ("alarm active "
                f"(warning_present={warning_present}, warning_code=0x{warning_code:04X}, "
                f"{detail}; {format_status_flags_summary(status)})")
    return None


def get_hold_fatal_reason(status: dict) -> Optional[str]:
    """Return only truly fatal hold conditions.

    For persistent background hold handoff, warning bits can be transient during
    command ownership changes; do not fail hold purely on warning presence.
    """
    if "error" in status:
        return status["error"]

    connection_faulted = bool(status.get("connection_faulted", False))
    fault = bool(status.get("fault", False))
    fault_code = int(status.get("fault_code", 0))

    if connection_faulted:
        return f"connection faulted: {format_status_flags_summary(status, severity_filter=('fault',))}"
    if fault:
        detail = format_drive_code_detail(fault_code, prefer_prefix="E")
        return (f"fault active (fault_code=0x{fault_code:04X}, {detail}; "
                f"{format_status_flags_summary(status)})")
    return None


def validate_accel_decel_ranges(accel_rpm_per_s: float, decel_rpm_per_s: float) -> Optional[str]:
    """Validate accel/decel ranges used by Kinetix motion commands.

    Units here are RPM/s. The drive ranges are specified in 0.1 RPM/s:
    458..30,000,000 (UM004D / MAx instruction docs) → 45.8..3,000,000 RPM/s.
    """
    accel_raw = int(accel_rpm_per_s * 10)
    decel_raw = int(decel_rpm_per_s * 10)
    if not (458 <= accel_raw <= 30000000):
        return (f"AccelReference out of range: {accel_rpm_per_s} RPM/s "
                f"(valid 45.8..3000000 RPM/s; raw={accel_raw}, limit 458..30000000)")
    if not (458 <= decel_raw <= 30000000):
        return (f"DecelReference out of range: {decel_rpm_per_s} RPM/s "
                f"(valid 45.8..3000000 RPM/s; raw={decel_raw}, limit 458..30000000)")
    return None


def normalize_display_code(value) -> Optional[str]:
    """Normalize a code input to display form like A603 or E60A.

    Kinetix display codes are hex (e.g. E60A, A603), not decimal-only.
    """
    if value is None:
        return None
    if isinstance(value, int):
        digits = f"{value & 0x0FFF:03X}"
        return f"A{digits}"

    text = str(value).strip().upper().replace(" ", "")
    if text.startswith("0X"):
        try:
            return normalize_display_code(int(text, 16))
        except ValueError:
            return None
    if re.fullmatch(r"[AE][0-9A-F]{3}", text):
        return text
    if re.fullmatch(r"[0-9A-F]{1,4}", text):
        try:
            return f"A{int(text, 16):03X}"
        except ValueError:
            return None
    return None


def load_drive_code_table(path: Path = DRIVE_CODES_CSV) -> dict:
    """Load the Kinetix fault/alarm code table from the Rockwell CSV export."""
    global _DRIVE_CODE_TABLE
    if _DRIVE_CODE_TABLE is not None:
        return _DRIVE_CODE_TABLE

    table = {}
    try:
        with open(path, newline="", encoding="utf-8-sig") as f:
            reader = csv.reader(f)
            header_seen = False
            for row in reader:
                if not row:
                    continue
                if not header_seen:
                    header_seen = any(cell.strip() == "Fault Code" for cell in row)
                    continue
                if len(row) < 5:
                    continue
                code = normalize_display_code(row[1])
                if not code:
                    continue
                table[code] = {
                    "fault_type": row[0].strip(),
                    "display": code,
                    "name": row[2].strip(),
                    "description": row[3].strip(),
                    "corrective_action": row[4].strip(),
                    "servo_on": row[5].strip() if len(row) > 5 else "",
                    "servo_off": row[6].strip() if len(row) > 6 else "",
                }
    except OSError as exc:
        print(f"  [WARN] Could not load drive code table {path}: {exc}")

    _DRIVE_CODE_TABLE = table
    return table


def decode_drive_code(value, prefer_prefix: str = "A") -> Optional[dict]:
    """Decode a drive fault/warning word or display code using the local CSV table."""
    table = load_drive_code_table()
    code = normalize_display_code(value)
    if not code:
        return None

    candidates = [code]
    if isinstance(value, int):
        digits = f"{value & 0x0FFF:03X}"
        candidates = [f"{prefer_prefix}{digits}", f"A{digits}", f"E{digits}"]

    for candidate in candidates:
        if candidate in table:
            return table[candidate]
    return None


def first_nonempty_line(text: str) -> str:
    for line in text.splitlines():
        line = line.strip()
        if line:
            return line
    return ""


def format_drive_code_detail(value, prefer_prefix: str = "A") -> str:
    decoded = decode_drive_code(value, prefer_prefix=prefer_prefix)
    if isinstance(value, int):
        fallback = f"{prefer_prefix}{value & 0x0FFF:03X}"
    else:
        fallback = normalize_display_code(value) or str(value)
    if not decoded:
        return fallback
    action = first_nonempty_line(decoded.get("corrective_action", ""))
    suffix = f" Action: {action}" if action else ""
    return f"{decoded['display']}: {decoded.get('name', '').strip()}{suffix}"


def format_status_code_summary(status: dict) -> str:
    """Return compact decoded fault/warning summary for status prints."""
    parts = []
    fault_code = int(status.get("fault_code", 0))
    warning_code = int(status.get("warning_code", 0))
    if fault_code:
        decoded = decode_drive_code(fault_code, prefer_prefix="E")
        label = decoded["display"] if decoded else f"0x{fault_code:04X}"
        name = f" {decoded['name']}" if decoded and decoded.get("name") else ""
        parts.append(f"fault={label}{name}")
    if warning_code:
        decoded = decode_drive_code(warning_code, prefer_prefix="A")
        label = decoded["display"] if decoded else f"0x{warning_code:04X}"
        name = f" {decoded['name']}" if decoded and decoded.get("name") else ""
        parts.append(f"warning={label}{name}")
    return ", ".join(parts) if parts else "fault/warning clear"


def format_status_flags_summary(status: dict, severity_filter: Optional[tuple[str, ...]] = None) -> str:
    """Return active Assembly 154 status flags using EDS field names."""
    flags = []
    for flag in DRIVE_INPUT_STATUS_FLAGS:
        if severity_filter is not None and flag["severity"] not in severity_filter:
            continue
        if status.get(flag["field"], False):
            flags.append(flag["display"])
    if flags:
        return "flags=" + "|".join(flags)
    if severity_filter:
        return "flags=none"
    return "flags=none"


def format_status_fault_summary(status: dict) -> str:
    """Return decoded fault/warning codes plus active diagnostic/fault flags."""
    return f"{format_status_code_summary(status)}; {format_status_flags_summary(status)}"


def strip_eds_comment(line: str) -> str:
    return line.split("$", 1)[0].strip()


def parse_int_field(value: str) -> Optional[int]:
    value = str(value).strip()
    if value == "" or value.upper() == "NULL":
        return None
    try:
        return int(value, 0)
    except ValueError:
        return None


def parse_eds_csv_fields(text: str) -> list[str]:
    """Parse a comma-separated EDS block after comments have been removed."""
    try:
        return next(csv.reader([text], skipinitialspace=True))
    except csv.Error:
        return []


def parse_eds_parameters(path: Path = DEFAULT_EDS_PATH) -> dict:
    """Parse Kinetix EDS Parameter and Group metadata."""
    path = Path(path)
    cache_key = str(path.resolve())
    if cache_key in _EDS_CACHE:
        return _EDS_CACHE[cache_key]

    params: dict[int, dict] = {}
    groups: dict[str, list[int]] = {}
    try:
        lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError as exc:
        print(f"  [FAIL] Could not read EDS {path}: {exc}")
        result = {"params": params, "groups": groups}
        _EDS_CACHE[cache_key] = result
        return result

    i = 0
    while i < len(lines):
        match = re.search(r"\bParam(\d+)\s*=", lines[i])
        if match:
            param_id = int(match.group(1))
            block_lines = []
            i += 1
            while i < len(lines):
                block_lines.append(strip_eds_comment(lines[i]))
                if ";" in lines[i]:
                    break
                i += 1
            block_text = " ".join(block_lines).replace(";", "")
            fields = parse_eds_csv_fields(block_text)
            if len(fields) >= 12:
                params[param_id] = {
                    "id": param_id,
                    "descriptor": parse_int_field(fields[3]),
                    "data_type": parse_int_field(fields[4]),
                    "size": parse_int_field(fields[5]) or 2,
                    "name": fields[6].strip(),
                    "units": fields[7].strip(),
                    "help": fields[8].strip(),
                    "min": parse_int_field(fields[9]),
                    "max": parse_int_field(fields[10]),
                    "default": parse_int_field(fields[11]),
                    "groups": [],
                }
        group_match = re.search(r"\bGroup\d+\s*=", lines[i])
        if group_match:
            block_lines = []
            i += 1
            while i < len(lines):
                block_lines.append(strip_eds_comment(lines[i]))
                if ";" in lines[i]:
                    break
                i += 1
            block_text = " ".join(block_lines).replace(";", "")
            fields = parse_eds_csv_fields(block_text)
            if len(fields) >= 3:
                group_name = fields[0].strip()
                ids = [value for value in (parse_int_field(field) for field in fields[2:]) if value is not None]
                groups[group_name] = ids
                for param_id in ids:
                    if param_id in params:
                        params[param_id]["groups"].append(group_name)
        i += 1

    result = {"params": params, "groups": groups}
    _EDS_CACHE[cache_key] = result
    return result


def decode_parameter_value(raw: bytes, size: int, data_type: Optional[int] = None):
    """Decode a parameter value using CIP elementary data type where possible."""
    if raw is None:
        return None
    size = min(size or len(raw), len(raw))
    data = raw[:size]
    if size == 0:
        return None
    if data_type == 0xC1:  # BOOL
        return bool(data[0])
    if data_type == 0xC2:  # SINT
        return struct.unpack("<b", data[:1])[0]
    if data_type == 0xC3 and size >= 2:  # INT
        return struct.unpack("<h", data[:2])[0]
    if data_type == 0xC4 and size >= 4:  # DINT
        return struct.unpack("<i", data[:4])[0]
    if data_type == 0xCA and size >= 4:  # REAL
        return struct.unpack("<f", data[:4])[0]
    if size == 1:
        return data[0]
    if size == 2:
        return struct.unpack("<H", data[:2])[0]
    if size >= 4:
        return struct.unpack("<I", data[:4])[0]
    return int.from_bytes(data, "little", signed=False)


def encode_parameter_value(value, size: int, data_type: Optional[int] = None) -> bytes:
    """Encode a parameter value using CIP elementary data type where possible."""
    if data_type == 0xC1:
        return struct.pack("<B", 1 if bool(value) else 0)
    if data_type == 0xC2:
        return struct.pack("<b", int(value))
    if data_type == 0xC3:
        return struct.pack("<h", int(value))
    if data_type == 0xC4:
        return struct.pack("<i", int(value))
    if data_type == 0xCA:
        return struct.pack("<f", float(value))
    if size == 1:
        return struct.pack("<B", int(value) & 0xFF)
    if size == 4:
        return struct.pack("<I", int(value) & 0xFFFFFFFF)
    return struct.pack("<H", int(value) & 0xFFFF)


def parse_id_list(text: Optional[str]) -> Optional[list[int]]:
    if not text:
        return None
    ids = []
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        ids.append(int(part, 0))
    return ids


def select_parameter_ids(eds: dict, ids_text: Optional[str] = None, group: Optional[str] = None) -> list[int]:
    params = eds.get("params", {})
    explicit_ids = parse_id_list(ids_text)
    if explicit_ids is not None:
        return [param_id for param_id in explicit_ids if param_id in params]
    if group:
        return [param_id for param_id in eds.get("groups", {}).get(group, []) if param_id in params]
    return sorted(params)


def add_ramp_sample(samples: list[tuple[float, float]], status: dict, timestamp: float):
    if "error" not in status:
        samples.append((timestamp, abs(status["actual_speed"] / 10.0)))


def measured_rate_between(samples: list[tuple[float, float]],
                          high_speed: float,
                          start_fraction: float,
                          end_fraction: float,
                          rising: bool) -> Optional[float]:
    """Compute measured ramp rate between two speed thresholds."""
    if len(samples) < 2 or high_speed <= 0:
        return None
    start_threshold = abs(high_speed * start_fraction)
    end_threshold = abs(high_speed * end_fraction)
    start_sample = None
    for sample in samples:
        speed = sample[1]
        if rising:
            if speed >= start_threshold:
                start_sample = sample
                break
        else:
            if speed <= start_threshold:
                start_sample = sample
                break
    if start_sample is None:
        return None
    for sample in samples:
        if sample[0] <= start_sample[0]:
            continue
        speed = sample[1]
        if rising:
            if speed >= end_threshold:
                dt = sample[0] - start_sample[0]
                return (sample[1] - start_sample[1]) / dt if dt >= RAMP_MIN_WINDOW_S else None
        else:
            if speed <= end_threshold:
                dt = sample[0] - start_sample[0]
                return (start_sample[1] - sample[1]) / dt if dt >= RAMP_MIN_WINDOW_S else None
    return None


def print_rate_verification(label: str, measured: Optional[float], commanded: float, tolerance_percent: float):
    if measured is None or commanded <= 0:
        print(f"  [WARN] {label}: insufficient samples for rate verification")
        return
    error_pct = abs(measured - commanded) / commanded * 100.0
    verdict = "PASS" if error_pct <= tolerance_percent else "WARN"
    print(f"  [{verdict}] {label}: measured={measured:.1f} RPM/s "
          f"commanded={commanded:.1f} RPM/s error={error_pct:.1f}% "
          f"tolerance={tolerance_percent:.1f}%")


def wait_for_user_confirmation(message: str):
    """Block until user confirms a safety-critical action."""
    input(f"  {message} Press Enter to continue or Ctrl+C to abort...")


def parse_move_mode_positionals(operating_mode: int,
                                mode_param_1: Optional[str],
                                mode_param_2: Optional[str],
                                mode_param_3: Optional[str],
                                force_required: bool = False) -> tuple[Optional[dict], Optional[str]]:
    """Parse move mode-specific positional arguments.

    Positional mapping:
      mode 1 (Position): p1=position_puu
      mode 2 (Speed):    p1=speed_rpm
      mode 3 (Home):     p1=speed_rpm, p2=homing_method, p3=home_return_speed_rpm
      mode 4 (Torque):   p1=torque_percent, p2=torque_ramp_ms
      mode 5 (Gear):     p1=speed_rpm
      mode 6 (Index):    p1=starting_index
      mode 7 (ECAM):     p1=starting_index
    """
    if mode_param_1 is None and mode_param_2 is None and mode_param_3 is None:
        if force_required:
            required_msg = {
                1: "Position mode requires positional parameter p1=position_puu.",
                2: "Speed mode requires positional parameter p1=speed_rpm.",
                3: "Home mode requires positional parameter p1=speed_rpm.",
                4: "Torque mode requires positional parameter p1=torque_percent.",
                5: "Gear mode requires positional parameter p1=speed_rpm.",
                6: "Index mode requires positional parameter p1=starting_index.",
                7: "ECAM mode requires positional parameter p1=starting_index.",
            }
            return None, required_msg.get(
                operating_mode, "Selected positional operating mode requires positional parameters.")
        return {}, None

    def as_int(label: str, value: str) -> int:
        try:
            return int(value, 0)
        except ValueError as exc:
            raise ValueError(f"{label} expects integer, got '{value}'") from exc

    def as_float(label: str, value: str) -> float:
        try:
            return float(value)
        except ValueError as exc:
            raise ValueError(f"{label} expects number, got '{value}'") from exc

    def extra_args_error(unused: list[Optional[str]]) -> Optional[str]:
        if any(v is not None for v in unused):
            return "Too many positional mode parameters provided for selected operating mode."
        return None

    values: dict = {}
    try:
        if operating_mode == 1:
            if mode_param_1 is None:
                return None, "Position mode requires positional parameter p1=position_puu."
            values["position_puu"] = as_int("position_puu", mode_param_1)
            err = extra_args_error([mode_param_2, mode_param_3])
            if err:
                return None, err
        elif operating_mode == 2:
            if mode_param_1 is None:
                return None, "Speed mode requires positional parameter p1=speed_rpm."
            values["speed_rpm"] = as_float("speed_rpm", mode_param_1)
            err = extra_args_error([mode_param_2, mode_param_3])
            if err:
                return None, err
        elif operating_mode == 3:
            if mode_param_1 is None:
                return None, ("Home mode requires p1=speed_rpm; optional p2=homing_method, "
                              "p3=home_return_speed_rpm.")
            values["speed_rpm"] = as_float("speed_rpm", mode_param_1)
            if mode_param_2 is not None:
                values["homing_method"] = as_int("homing_method", mode_param_2)
            if mode_param_3 is not None:
                values["home_return_speed_rpm"] = as_float("home_return_speed_rpm", mode_param_3)
        elif operating_mode == 4:
            if mode_param_1 is None:
                return None, "Torque mode requires positional parameter p1=torque_percent."
            values["torque_percent"] = as_float("torque_percent", mode_param_1)
            if mode_param_2 is not None:
                values["torque_ramp_ms"] = as_int("torque_ramp_ms", mode_param_2)
            err = extra_args_error([mode_param_3])
            if err:
                return None, err
        elif operating_mode == 5:
            if mode_param_1 is None:
                return None, "Gear mode requires positional parameter p1=speed_rpm."
            values["speed_rpm"] = as_float("speed_rpm", mode_param_1)
            err = extra_args_error([mode_param_2, mode_param_3])
            if err:
                return None, err
        elif operating_mode in (6, 7):
            if mode_param_1 is None:
                return None, "Index/ECAM mode requires positional parameter p1=starting_index."
            values["starting_index"] = as_int("starting_index", mode_param_1)
            err = extra_args_error([mode_param_2, mode_param_3])
            if err:
                return None, err
        else:
            return None, f"Unsupported operating mode {operating_mode} for positional parsing."
    except ValueError as exc:
        return None, str(exc)

    return values, None


def clear_drive_fault(client, ot_conn_id: int, max_cycles: int = 20, timeout: float = 2.0) -> bool:
    """Clear a drive trip state (fault or warning/alarm) using both the CIP
    FaultReset parameter and the cyclic assembly fault_reset bit
    (rising-edge triggered).

    The Kinetix 5100 EDS defines CIP Parameter Object instance #24 as
    FaultReset. The drive also expects a rising edge on the assembly
    fault_reset bit (byte 1, bit 3). Some hardware faults still require a
    power cycle.

    Returns True when fault and warning indicators are all clear.
    """
    print("\n  --- Clearing Drive Fault/Alarm ---")
    print("  Note: hardware faults may still require power-cycle.")

    # 1. Trigger CIP FaultReset (Parameter Object instance #24, value 1)
    print("  Writing CIP FaultReset parameter (instance 24)...")
    cip_ok = client.write_k5100_parameter(24, 1, size=1)
    print(f"  CIP FaultReset: {'OK' if cip_ok else 'FAIL'}")

    # 2. Pulse the cyclic fault_reset bit (rising-edge triggered)
    print("  Pulsing assembly fault_reset bit (0 -> 1 -> 0)...")
    low_asm = build_output_assembly_104(servo_on=False)
    high_asm = build_output_assembly_104(servo_on=False, fault_reset=True)
    client.exchange_io_frame(ot_conn_id, low_asm, include_run_idle=True, timeout=timeout)
    time.sleep(0.05)
    client.exchange_io_frame(ot_conn_id, high_asm, include_run_idle=True, timeout=timeout)
    time.sleep(0.05)
    client.exchange_io_frame(ot_conn_id, low_asm, include_run_idle=True, timeout=timeout)

    # 3. Monitor until fault+warning indicators are all clear
    status_seen = False
    for i in range(max_cycles):
        to_data = client.exchange_io_frame(ot_conn_id, low_asm, include_run_idle=True, timeout=timeout)
        if to_data is None:
            time.sleep(0.1)
            continue
        status = parse_input_assembly_154(to_data)
        if "error" not in status:
            status_seen = True
            print(f"  Reset attempt {i+1}: fault={status['fault']} "
                  f"fault_code=0x{status['fault_code']:04X} "
                  f"warning_code=0x{status['warning_code']:04X} "
                  f"{format_status_fault_summary(status)}")
            if (not status['fault']
                    and not status['warning_present']
                    and not status.get('connection_faulted', False)
                    and not status.get('diagnostic_active', False)
                    and not status.get('uncertain', False)):
                print("  [OK] Fault/alarm cleared")
                return True
        time.sleep(0.1)
    if not status_seen:
        print("  [WARN] No T->O status received while trying to clear fault/alarm")
    print("  [FAIL] Fault/alarm did not clear. Power-cycle may be required.")
    return False


class EipClient:
    """EtherNet/IP client for interacting with a drive."""

    def __init__(self, host: str = DRIVE_IP, port: int = EIP_PORT):
        self.host = host
        self.port = port
        self.sock: Optional[socket.socket] = None
        self.session_handle: int = 0
        self.verbose: bool = True
        # Persistent UDP socket for Class 1 I/O. Reusing one bound socket across
        # frames keeps port 2222 continuously listening, which avoids ICMP
        # "port unreachable" replies to the drive between frames (a cause of
        # silent Class 1 connection drops during a long-running hold).
        self.io_sock: Optional[socket.socket] = None

    def connect(self) -> bool:
        """Open TCP connection."""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5.0)
            self.sock.connect((self.host, self.port))
            if self.verbose:
                print(f"[OK] TCP connected to {self.host}:{self.port}")
            return True
        except Exception as e:
            print(f"[FAIL] TCP connect failed: {e}")
            return False

    def disconnect(self):
        """Close TCP connection."""
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None
            self.session_handle = 0
        self.close_io_socket()

    def ensure_io_socket(self, timeout: float = 2.0) -> Optional[socket.socket]:
        """Return a persistent UDP socket bound to the Class 1 I/O port.

        Created lazily and reused for the lifetime of the client so the port
        stays continuously bound between frames.
        """
        if self.io_sock is None:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                sock.bind(('0.0.0.0', UDP_PORT))
                self.io_sock = sock
            except Exception as exc:
                if self.verbose:
                    print(f"  [FAIL] Could not open persistent UDP socket: {exc}")
                self.io_sock = None
                return None
        try:
            self.io_sock.settimeout(timeout)
        except Exception:
            pass
        return self.io_sock

    def close_io_socket(self):
        if self.io_sock is not None:
            try:
                self.io_sock.close()
            except Exception:
                pass
            self.io_sock = None

    @staticmethod
    def _extract_connected_assembly(rx_data: bytes) -> Optional[bytes]:
        """Pull the assembly bytes (after the 2-byte seq count) from a CPF frame."""
        for type_id, item_data in parse_cpf(rx_data):
            if type_id == CPF_CONNECTED_DATA and len(item_data) >= 2:
                return item_data[2:]
        return None

    def _send_recv(self, request: bytes, timeout: float = 5.0) -> Optional[bytes]:
        """Send request and receive response over TCP."""
        if not self.sock:
            print("[FAIL] Not connected")
            return None
        try:
            self.sock.settimeout(timeout)
            self.sock.sendall(request)

            if self.verbose:
                cmd_raw = struct.unpack_from("<H", request, 0)[0]
                print(f"  TX {len(request)} bytes, cmd=0x{cmd_raw:04X}")

            # Read encap header first to get data length
            hdr = self._recv_exact(24, timeout)
            if hdr is None:
                print("[FAIL] No response header received")
                return None

            _, dlen, _, _, _ = parse_encap_header(hdr)
            response = bytearray(hdr)

            if dlen > 0:
                data = self._recv_exact(dlen, timeout)
                if data is None:
                    print(f"[FAIL] Incomplete response data (expected {dlen} bytes)")
                    return None
                response += data

            if self.verbose:
                cmd_raw = struct.unpack_from("<H", bytes(response), 0)[0]
                print(f"  RX {len(response)} bytes, cmd=0x{cmd_raw:04X}")

            return bytes(response)

        except socket.timeout:
            print("[FAIL] TCP timeout")
            return None
        except Exception as e:
            print(f"[FAIL] TCP error: {e}")
            return None

    def _recv_exact(self, n: int, timeout: float) -> Optional[bytes]:
        """Receive exactly n bytes."""
        self.sock.settimeout(timeout)
        data = bytearray()
        while len(data) < n:
            try:
                chunk = self.sock.recv(n - len(data))
                if not chunk:
                    return None
                data += chunk
            except socket.timeout:
                return None
        return bytes(data)

    def list_identity(self) -> Optional[dict]:
        """Send ListIdentity request and parse response."""
        request = build_encap_header(ENCAP_LIST_IDENTITY, b'')
        response = self._send_recv(request, timeout=3.0)
        if response is None:
            return None

        cmd, dlen, sh, st, offset = parse_encap_header(response)
        if st != 0:
            print(f"[FAIL] ListIdentity status=0x{st:08X}")
            return None

        cpf_items = parse_cpf(response[offset:offset + dlen])
        for type_id, item_data in cpf_items:
            if type_id == CPF_LIST_IDENTITY_RESPONSE:
                return self._parse_identity_item(item_data)

        print("[FAIL] No identity item in response")
        return None

    def _parse_identity_item(self, data: bytes) -> dict:
        """Parse ListIdentity response data."""
        result = {}
        result["encap_version"] = struct.unpack_from("<H", data, 0)[0]

        # socket address (16 bytes): sin_family(2) + port(2) + ip(4) + zeros(8)
        # bytes 2-3: sin_family, bytes 4-5: sin_port (network order), bytes 6-9: sin_addr, bytes 10-17: zeros
        sin_port = struct.unpack_from(">H", data, 4)[0]  # network byte order
        sin_addr = '.'.join(str(b) for b in data[6:10])
        result["socket_address"] = f"{sin_addr}:{sin_port}"

        # Offsets after 16-byte socket address (starting at byte 18)
        result["vendor_id"] = struct.unpack_from("<H", data, 18)[0]    # bytes 18-19
        result["device_type"] = struct.unpack_from("<H", data, 20)[0]  # bytes 20-21
        result["product_code"] = struct.unpack_from("<H", data, 22)[0] # bytes 22-23
        result["revision"] = f"{data[24]}.{data[25]}"                  # bytes 24-25
        result["status"] = struct.unpack_from("<H", data, 26)[0]       # bytes 26-27
        result["serial_number"] = struct.unpack_from("<I", data, 28)[0] # bytes 28-31

        name_len = data[32]                                            # byte 32
        result["product_name"] = data[33:33 + name_len].decode("ascii", errors="replace")
        result["state"] = data[33 + name_len]
        return result

    def register_session(self) -> bool:
        """Register an EIP session and store the handle."""
        data = le16(1) + le16(0)  # protocol version 1, options 0
        request = build_encap_header(ENCAP_REGISTER_SESSION, data)
        response = self._send_recv(request)
        if response is None:
            return False

        cmd, dlen, sh, st, offset = parse_encap_header(response)
        if st != 0:
            print(f"[FAIL] RegisterSession status=0x{st:08X}")
            return False
        if sh == 0:
            print("[FAIL] RegisterSession returned zero handle")
            return False

        self.session_handle = sh
        print(f"[OK] RegisterSession succeeded, handle=0x{sh:08X}")
        return True

    def exchange_io_frame(self, ot_conn_id: int, assembly_data: bytes,
                          include_run_idle: bool = True,
                          timeout: float = 5.0,
                          reuse_socket: bool = False,
                          drain: bool = False) -> Optional[bytes]:
        """Send one O->T frame and receive the freshest T->O frame over UDP 2222.

        Returns the T->O assembly data (after stripping the seq count), or None.

        reuse_socket: use the client's persistent UDP socket (kept bound between
            frames). Preferred for long-running cyclic I/O such as the servo
            hold, because it avoids per-frame socket churn and keeps the port
            listening so the drive never gets ICMP "port unreachable".
        drain: after the first response, consume any additional queued T->O
            datagrams and return the newest one. Keeps a high-rate loop current
            instead of processing stale buffered frames.
        """
        encap_seq = int(time.time() * 1000) & 0xFFFFFFFF
        cip_seq = (encap_seq & 0xFFFF)

        # Sequenced address item
        addr_data = le32(ot_conn_id) + le32(encap_seq)

        # Connected data item: seq(u16) + optional Run/Idle(u32) + assembly
        data = le16(cip_seq)
        if include_run_idle:
            data += le32(0x00000001)  # Run state
        data += assembly_data

        cpf = build_cpf([
            (CPF_SEQUENCED_ADDRESS, addr_data),
            (CPF_CONNECTED_DATA, data),
        ])

        if self.verbose:
            print(f"  Sending O->T: {len(cpf)} bytes (conn_id=0x{ot_conn_id:08X}, seq={encap_seq})")

        owns_socket = False
        udp_sock: Optional[socket.socket] = None
        try:
            if reuse_socket:
                udp_sock = self.ensure_io_socket(timeout)
                if udp_sock is None:
                    return None
            else:
                udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                udp_sock.settimeout(timeout)
                udp_sock.bind(('0.0.0.0', UDP_PORT))
                owns_socket = True

            udp_sock.sendto(cpf, (self.host, UDP_PORT))

            if self.verbose:
                print(f"  Waiting for T->O on UDP:{UDP_PORT} (timeout {timeout}s)...")
            rx_data, addr = udp_sock.recvfrom(4096)
            latest = self._extract_connected_assembly(rx_data)

            if drain:
                # Grab any newer datagrams already queued so we act on the most
                # recent drive state and keep the OS receive buffer from filling.
                udp_sock.setblocking(False)
                try:
                    while True:
                        try:
                            more, _addr = udp_sock.recvfrom(4096)
                        except (BlockingIOError, OSError):
                            break
                        newer = self._extract_connected_assembly(more)
                        if newer is not None:
                            latest = newer
                finally:
                    try:
                        udp_sock.settimeout(timeout)
                    except Exception:
                        pass

            if self.verbose:
                print(f"  Received T->O: {len(rx_data)} bytes from {addr[0]}:{addr[1]}")
                if latest is not None:
                    print(f"  T->O assembly data={len(latest)} bytes")
                    print(f"  T->O hex: {format_hex(latest[:32])}{'...' if len(latest) > 32 else ''}")

            if latest is None and self.verbose:
                print("  [WARN] No connected data item in T->O response")
                print(f"  Raw response hex: {format_hex(rx_data[:64])}")
            return latest

        except socket.timeout:
            if self.verbose:
                print(f"  [FAIL] T->O UDP timeout ({timeout}s)")
            if reuse_socket:
                # A stale/half-open persistent socket can wedge; drop it so the
                # next call rebinds cleanly.
                self.close_io_socket()
            return None
        except Exception as e:
            if self.verbose:
                print(f"  [FAIL] UDP I/O error: {e}")
            if reuse_socket:
                self.close_io_socket()
            return None
        finally:
            if owns_socket and udp_sock is not None:
                try:
                    udp_sock.close()
                except Exception:
                    pass

    def get_attribute_single(self, class_id: int, instance_id: int,
                             attribute_id: int = 1) -> Optional[bytes]:
        """Send Get Attribute Single explicit message. Returns raw attribute data or None."""
        if self.session_handle == 0:
            print("[FAIL] No active session. Run register-session first.")
            return None

        epath = build_epath(class_id, instance_id, has_attr=True, attr_id=attribute_id)
        cip_request = build_mr_request(CIP_GET_ATTRIBUTE_SINGLE, epath)
        request = build_send_rr_data(cip_request, self.session_handle)

        prev_verbose = self.verbose
        self.verbose = False
        response = self._send_recv(request)
        self.verbose = prev_verbose

        if response is None:
            return None

        cmd, dlen, sh, st, offset = parse_encap_header(response)
        if st != 0:
            print(f"  [FAIL] SendRRData encap status=0x{st:08X}")
            return None

        payload = response[offset:offset + dlen]
        if len(payload) < 6:
            return None
        cpf_items = parse_cpf(payload[6:])
        for type_id, item_data in cpf_items:
            if type_id == CPF_UNCONNECTED_DATA:
                mr = parse_mr_response(item_data)
                if mr.get("is_success"):
                    return mr.get("response_data", b'')
                else:
                    gen = mr.get("general_status", 0xFF)
                    print(f"  [FAIL] GetAttributeSingle: general_status=0x{gen:02X}")
                    return None
        return None

    def set_attribute_single(self, class_id: int, instance_id: int,
                             attribute_id: int, value: bytes) -> bool:
        """Send Set Attribute Single explicit message. Returns True on success."""
        if self.session_handle == 0:
            print("[FAIL] No active session. Run register-session first.")
            return False

        epath = build_epath(class_id, instance_id, has_attr=True, attr_id=attribute_id)
        cip_request = build_mr_request(CIP_SET_ATTRIBUTE_SINGLE, epath, value)
        request = build_send_rr_data(cip_request, self.session_handle)

        prev_verbose = self.verbose
        self.verbose = False
        response = self._send_recv(request)
        self.verbose = prev_verbose

        if response is None:
            return False

        cmd, dlen, sh, st, offset = parse_encap_header(response)
        if st != 0:
            print(f"  [FAIL] SendRRData encap status=0x{st:08X}")
            return False

        payload = response[offset:offset + dlen]
        if len(payload) < 6:
            return False
        cpf_items = parse_cpf(payload[6:])
        for type_id, item_data in cpf_items:
            if type_id == CPF_UNCONNECTED_DATA:
                mr = parse_mr_response(item_data)
                if mr.get("is_success"):
                    return True
                else:
                    gen = mr.get("general_status", 0xFF)
                    print(f"  [FAIL] SetAttributeSingle: general_status=0x{gen:02X}")
                    return False
        return False

    def read_k5100_parameter_raw(self, param_id: int) -> Optional[bytes]:
        """Read raw bytes from a Kinetix 5100 Parameter Object value attribute."""
        return self.get_attribute_single(0x0F, param_id, attribute_id=1)

    def read_k5100_parameter(self, param_id: int, size: Optional[int] = None,
                             data_type: Optional[int] = None):
        """Read a Kinetix 5100 parameter value via CIP Parameter Object (class 0x0F).

        Args:
            param_id: Parameter ID (e.g., 117 for P1.001 Control Mode).
            size: Optional byte size from the EDS descriptor.
            data_type: Optional CIP elementary data type from the EDS descriptor.

        Returns:
            Decoded parameter value, or None on failure.
        """
        data = self.read_k5100_parameter_raw(param_id)
        if data is None:
            return None
        if size is not None or data_type is not None:
            return decode_parameter_value(data, size or len(data), data_type)
        if len(data) >= 2:
            return struct.unpack_from("<H", data, 0)[0]  # Most K5100 params are UINT16
        if len(data) == 1:
            return data[0]
        return None

    def write_k5100_parameter(self, param_id: int, value: int, size: int = 2,
                              data_type: Optional[int] = None) -> bool:
        """Write a Kinetix 5100 parameter value via CIP Parameter Object (class 0x0F).

        Args:
            param_id: Parameter ID (e.g., 117 for P1.001).
            value: Integer value to write.
            size: Byte size (1, 2, or 4). Defaults to 2 (UINT16).
            data_type: Optional CIP elementary data type from the EDS descriptor.

        Returns:
            True on success, False on failure.
        """
        data = encode_parameter_value(value, size, data_type)
        return self.set_attribute_single(0x0F, param_id, attribute_id=1, value=data)

    def check_drive_ready(self) -> dict:
        """Check if the drive is powered on, in I/O mode, and servo-enabled."""
        result = {
            "powered_on": False,
            "io_mode": False,
            "p1_001_value": None,
            "p1_001_name": "Unknown",
            "servo_ready": False,
            "servo_active": False,
            "servo_enabled": False,
        }

        # Check connectivity via ListIdentity (drive responds = powered on)
        prev_verbose = self.verbose
        self.verbose = False
        identity = self.list_identity()
        self.verbose = prev_verbose

        if identity is None:
            print("  [FAIL] Drive not responding. Is it powered on?")
            return result

        result["powered_on"] = True
        print(f"  [OK] Drive powered on: {identity.get('product_name', '?')} "
              f"state={identity.get('state', '?')}")

        # Read P1.001 (Control Mode, param ID 117)
        pmode = self.read_k5100_parameter(117)
        if pmode is not None:
            result["p1_001_value"] = pmode
            mode_names = {
                0x0000: "PT (Position / Pulse Train)",
                0x0001: "PR (Position Register / Indexing)",
                0x0002: "S (Speed / Analog)",
                0x0003: "T (Torque / Analog)",
                0x0004: "Sz (Speed / Internal Registers)",
                0x0005: "Tz (Torque / Internal Registers)",
                0x000A: "PT_PR (Dual: Position/Indexing)",
                0x000B: "PT_S (Dual: Position/Speed)",
                0x000C: "IO (EtherNet/IP I/O Mode)",
                0x000D: "PT_T (Dual: Position/Torque)",
                0x000E: "PR_S (Dual: Indexing/Speed)",
                0x000F: "PR_T (Dual: Indexing/Torque)",
                0x0010: "S_T (Dual: Speed/Torque)",
                0x0101: "CSP (Cyclic Sync Position)",
                0x0102: "CSV (Cyclic Sync Velocity)",
                0x0103: "CST (Cyclic Sync Torque)",
            }
            result["p1_001_name"] = mode_names.get(pmode, f"Unknown (0x{pmode:04X})")
            result["io_mode"] = (pmode == 0x000C)
            print(f"  P1.001 Control Mode = 0x{pmode:04X} ({result['p1_001_name']})")
        else:
            print("  [WARN] Could not read P1.001 via CIP Parameter Object")

        # Read Ready state (Param 44, BOOL) explicitly without touching cyclic I/O.
        ready = self.read_k5100_parameter(44, size=1, data_type=0xC1)
        if ready is None:
            print("  [WARN] Could not read Ready (Param 44) via CIP Parameter Object")
        else:
            result["servo_ready"] = bool(ready)
            print(f"  Servo ready (Param44) = {result['servo_ready']}")

        active = self.read_k5100_parameter(45, size=1, data_type=0xC1)
        if active is None:
            print("  [WARN] Could not read Active (Param 45) via CIP Parameter Object")
        else:
            result["servo_active"] = bool(active)
            print(f"  Servo active (Param45) = {result['servo_active']}")

        # In the Kinetix assembly map, Active represents enabled operating state.
        # Keep "servo_ready" for compatibility, but gate motion on servo_enabled.
        result["servo_enabled"] = bool(result["servo_active"])

        return result

    def unregister_session(self) -> bool:
        """Unregister the current session."""
        if self.session_handle == 0:
            print("[SKIP] No active session")
            return True

        request = build_encap_header(ENCAP_UNREGISTER_SESSION, b'', self.session_handle)
        # UnRegisterSession may not send a reply in some implementations
        try:
            if self.sock:
                self.sock.sendall(request)
            print(f"[OK] UnRegisterSession sent (handle=0x{self.session_handle:08X})")
        except Exception as e:
            print(f"[WARN] UnRegisterSession send error: {e}")
        self.session_handle = 0
        return True

    def forward_open(self, params: Optional[ForwardOpenParams] = None) -> Optional[dict]:
        """Send ForwardOpen and return parsed result."""
        if self.session_handle == 0:
            print("[FAIL] No active session. Run register-session first.")
            return None

        if params is None:
            params = ForwardOpenParams()

        # Build CIP ForwardOpen request
        fo_data = build_forward_open_request(params)
        epath = build_epath(CIP_CLASS_CONNECTION_MANAGER, 1)
        cip_request = build_mr_request(CIP_FORWARD_OPEN, epath, fo_data)

        # Wrap in SendRRData
        request = build_send_rr_data(cip_request, self.session_handle)
        response = self._send_recv(request)
        if response is None:
            return None

        # Parse encap
        cmd, dlen, sh, st, offset = parse_encap_header(response)
        if st != 0:
            print(f"[FAIL] SendRRData encap status=0x{st:08X}")
            return None

        # Parse SendRRData payload to get CIP response
        # Payload format: interface_handle(u32) + timeout(u16) + CPF
        payload = response[offset:offset + dlen]
        if len(payload) < 6:
            print(f"[FAIL] SendRRData payload too short: {len(payload)} bytes")
            return None
        cpf_data = payload[6:]  # skip interface_handle(4) + timeout(2)
        cpf_items = parse_cpf(cpf_data)
        cip_response = None
        for type_id, item_data in cpf_items:
            if type_id == CPF_UNCONNECTED_DATA:
                cip_response = item_data
                break

        if cip_response is None:
            print("[FAIL] No unconnected data item in SendRRData reply")
            return None

        # Parse Message Router response
        mr = parse_mr_response(cip_response)
        if "error" in mr:
            print(f"[FAIL] MR parse error: {mr['error']}")
            return mr

        # Print params used
        # Calculate actual connection sizes for display
        if params.raw_ot_conn_size is not None:
            ot_sent = params.raw_ot_conn_size
        else:
            ot_sent = params.ot_assembly_size + 2 + (4 if params.include_run_idle_header else 0)
        if params.raw_to_conn_size is not None:
            to_sent = params.raw_to_conn_size
        else:
            to_sent = params.to_assembly_size + 2

        # Build network params for debug display
        set_run_idle_bit = params.run_idle_bit_only
        ot_np = make_network_connection_params(
            ot_sent, CONN_TYPE_P2P, PRIORITY_SCHEDULED, set_run_idle_bit)
        to_np = make_network_connection_params(
            to_sent, params.to_connection_type, PRIORITY_SCHEDULED, False)

        flags = []
        if set_run_idle_bit:
            flags.append("RunIdleBit")
        print(f"  Params: config={params.config_instance} O->T={params.ot_instance}"
              f"({params.ot_assembly_size}B+2{'+4' if params.include_run_idle_header else ''}={ot_sent}B)"
              f" T->O={params.to_instance}({params.to_assembly_size}B+2={to_sent}B)"
              f" RPI={params.ot_rpi_us}/{params.to_rpi_us}us"
              f" T->O_type={'P2P' if params.to_connection_type == CONN_TYPE_P2P else 'MCAST'}"
              f" {' '.join(flags)}")
        print(f"  NetworkParams: O->T=0x{ot_np:04X} (size={ot_sent}) T->O=0x{to_np:04X} (size={to_sent})")

        if mr["is_success"]:
            print(f"[OK] ForwardOpen SUCCESS! general_status=0x{mr['general_status']:02X}")

            # Parse ForwardOpen reply data
            fo_reply = parse_forward_open_reply(mr["response_data"])
            fo_reply["mr"] = mr
            fo_reply["params"] = params
            if "error" not in fo_reply:
                print(f"  O->T conn_id=0x{fo_reply.get('ot_connection_id', 0):08X}"
                      f" T->O conn_id=0x{fo_reply.get('to_connection_id', 0):08X}"
                      f" O->T API={fo_reply.get('ot_api_us', 0)}us"
                      f" T->O API={fo_reply.get('to_api_us', 0)}us")
            return fo_reply
        else:
            ext_status = 0
            if len(mr["additional_status"]) >= 2:
                ext_status = struct.unpack_from("<H", mr["additional_status"], 0)[0]

            print(f"[FAIL] ForwardOpen REJECTED: general_status=0x{mr['general_status']:02X}")
            print(f"  Extended status=0x{ext_status:04X} -> {extended_status_description(ext_status)}")
            print(f"  Additional status bytes: {format_hex(mr['additional_status'])}")

            # If there are extra bytes beyond the extended status, they may encode the expected size
            if len(mr["additional_status"]) > 2:
                extra = mr["additional_status"][2:]
                if len(extra) >= 2:
                    expected = struct.unpack_from("<H", extra, 0)[0]
                    print(f"  Hint: drive may expect size = {expected} bytes (from additional status)")

            return {"mr": mr, "extended_status": ext_status}

    def forward_close(self, params: Optional[ForwardOpenParams] = None) -> bool:
        """Send ForwardClose to cleanly tear down a Class 1 connection."""
        if self.session_handle == 0:
            print("[SKIP] No active session, cannot ForwardClose")
            return False

        if params is None:
            params = ForwardOpenParams()

        fc_data = build_forward_close_request(params)
        epath = build_epath(CIP_CLASS_CONNECTION_MANAGER, 1)
        cip_request = build_mr_request(CIP_FORWARD_CLOSE, epath, fc_data)
        request = build_send_rr_data(cip_request, self.session_handle)

        prev_verbose = self.verbose
        self.verbose = False
        response = self._send_recv(request)
        self.verbose = prev_verbose

        if response is None:
            print("[WARN] ForwardClose received no response")
            return False

        cmd, dlen, sh, st, offset = parse_encap_header(response)
        if st != 0:
            print(f"[WARN] ForwardClose encap status=0x{st:08X}")
            return False

        payload = response[offset:offset + dlen]
        if len(payload) < 6:
            print("[WARN] ForwardClose payload too short")
            return False

        cpf_items = parse_cpf(payload[6:])
        for type_id, item_data in cpf_items:
            if type_id == CPF_UNCONNECTED_DATA:
                mr = parse_mr_response(item_data)
                if mr.get("is_success"):
                    print("[OK] ForwardClose succeeded")
                    return True
                else:
                    gen = mr.get("general_status", 0xFF)
                    print(f"[WARN] ForwardClose CIP status=0x{gen:02X}")
                    return False
        print("[WARN] ForwardClose response missing unconnected data item")
        return False


def run_list_identity(client: EipClient) -> bool:
    """Run ListIdentity command."""
    print("\n=== ListIdentity ===")
    if not client.connect():
        # ListIdentity uses UDP broadcast, but we try TCP first since it's simpler
        # Actually, ListIdentity typically uses TCP on port 44818
        return False

    result = client.list_identity()
    if result:
        print(f"  Product: {result.get('product_name', 'N/A')}")
        print(f"  Vendor ID: {result.get('vendor_id', 0)}")
        print(f"  Device Type: {result.get('device_type', 0)}")
        print(f"  Product Code: {result.get('product_code', 0)}")
        print(f"  Revision: {result.get('revision', 'N/A')}")
        print(f"  Serial: 0x{result.get('serial_number', 0):08X}")
        print(f"  State: {result.get('state', 'N/A')}")

    client.disconnect()
    return result is not None


def run_register_session(client: EipClient, store_handle: bool = True) -> bool:
    """Run RegisterSession."""
    print("\n=== RegisterSession ===")
    if not client.connect():
        return False
    return client.register_session()


def run_forward_open(client: EipClient, params: ForwardOpenParams) -> Optional[dict]:
    """Run ForwardOpen with given parameters. Assumes client is connected and session registered."""
    print("\n=== ForwardOpen ===")
    result = client.forward_open(params)
    return result


def run_full_test(client: EipClient, params: ForwardOpenParams) -> dict:
    """Run a full test: connect, register, forward-open, unregister, disconnect."""
    if not client.connect():
        return {"success": False, "stage": "connect"}
    if not client.register_session():
        client.disconnect()
        return {"success": False, "stage": "register"}

    result = client.forward_open(params)
    if result is not None:
        client.forward_close(result.get("params", params))
    client.unregister_session()
    client.disconnect()
    return {"success": result is not None, "stage": "forward_open", "result": result}


def run_servo_state_command(client: EipClient, enable: bool, rpi_us: int = 5000,
                            attempts: int = 8, hold_seconds: float = 0.0):
    """Enable or disable servo as an explicit standalone command.

    For servo-on, this command keeps cyclic I/O alive and keeps ownership
    until the user stops the hold session (Ctrl+C) or timeout is reached.
    """
    action = "Enable" if enable else "Disable"
    print(f"\n=== Servo {action} ===")
    print("  This command only toggles servo state; no travel motion profile is sent.")
    wait_for_user_confirmation(f"{action} servo command acknowledged.")

    # Pre-flight explicit checks first.
    if not client.connect():
        return
    if not client.register_session():
        client.disconnect()
        return
    status = client.check_drive_ready()
    client.unregister_session()
    client.disconnect()

    if not status["powered_on"]:
        print("\n[ABORT] Drive not responding. Check power and Ethernet connection.")
        return
    if not status["io_mode"]:
        print(f"\n[ABORT] Drive is NOT in I/O mode (P1.001=0x{status['p1_001_value']:04X} = {status['p1_001_name']}).")
        print("  Set P1.001 = 0x0C (IO mode), then power-cycle and retry.")
        return

    # If the requested state is already present, skip Class 1 open/close.
    # Opening I/O solely to pulse ServoOff on an already-disabled drive can
    # surface unrelated latched faults (e.g. E60A) without changing servo state.
    already_active = bool(status.get("servo_enabled", status.get("servo_active", False)))
    if enable and already_active:
        print("  [OK] Servo is already enabled (Param45 Active=True). No action taken.")
        return
    if (not enable) and (not already_active):
        print("  [OK] Servo is already disabled (Param45 Active=False). No action taken.")
        return

    # Open cyclic connection and apply servo state.
    if not client.connect():
        return
    if not client.register_session():
        client.disconnect()
        return

    params = ForwardOpenParams()
    params.ot_assembly_size = 40
    params.to_assembly_size = 52
    params.ot_rpi_us = rpi_us
    params.to_rpi_us = rpi_us
    params.raw_ot_conn_size = 40 + 2 + 4
    # docs/LOW_LEVEL_GANTRY_CONTROL.md / EXPECTED_ELECTROMECHANICAL_ASSEMBLY.md
    # indicate Kinetix requires the 32-bit Run/Idle header on O->T cyclic data.
    params.include_run_idle_header = True

    result = client.forward_open(params)
    if result is None:
        client.unregister_session()
        client.disconnect()
        return

    ot_conn_id = result.get("ot_connection_id", 0)
    if ot_conn_id == 0:
        print("[FAIL] ForwardOpen returned zero O->T connection ID")
        client.unregister_session()
        client.disconnect()
        return

    # Seed status once to capture current position for a zero-distance hold frame.
    # This improves "servo-on" hold behavior by commanding a position lock at the
    # current feedback position (no intentional travel move).
    hold_position_puu = 0
    seed = client.exchange_io_frame(
        ot_conn_id, build_output_assembly_104(servo_on=False, servo_off=False),
        include_run_idle=True, timeout=2.0)
    if seed is not None:
        seed_status = parse_input_assembly_154(seed)
        if "error" not in seed_status:
            hold_position_puu = int(seed_status.get("actual_position", 0))

    if enable:
        # Pulse StartMotion once in Position mode at the current feedback position
        # to command a non-traveling hold target.
        pulse_cmd = build_output_assembly_104(
            servo_on=True,
            servo_off=False,
            speed_rpm=5.0,
            accel_rpm_per_s=DEFAULT_ACCEL_RPM_PER_S,
            decel_rpm_per_s=DEFAULT_DECEL_RPM_PER_S,
            operating_mode=1,
            position_puu=hold_position_puu,
            travel_mode=2,
            start_motion=True)
        hold_cmd = build_output_assembly_104(
            servo_on=True,
            servo_off=False,
            speed_rpm=5.0,
            accel_rpm_per_s=DEFAULT_ACCEL_RPM_PER_S,
            decel_rpm_per_s=DEFAULT_DECEL_RPM_PER_S,
            operating_mode=1,
            position_puu=hold_position_puu,
            travel_mode=2,
            start_motion=False)
        print(f"  Hold reference set to current position: {hold_position_puu} PUU")
    else:
        # ServoOff is an explicit control bit in Output Assembly 104 (byte1 bit1).
        pulse_cmd = build_output_assembly_104(
            servo_on=False, servo_off=True, speed_rpm=0.0, operating_mode=0)
        hold_cmd = build_output_assembly_104(
            servo_on=False, servo_off=False, speed_rpm=0.0, operating_mode=0)

    ok = False
    status_seen = False
    active_seen = False
    for i in range(max(1, attempts)):
        cmd = pulse_cmd if i == 0 else hold_cmd
        to_data = client.exchange_io_frame(ot_conn_id, cmd, include_run_idle=True, timeout=2.0)
        if to_data is None:
            continue
        st = parse_input_assembly_154(to_data)
        if "error" in st:
            continue
        status_seen = True
        active_seen = active_seen or bool(st.get("active", False))
        trip_reason = get_drive_trip_reason(st)
        if trip_reason is not None:
            # Disable goal is "Active low". If Active is already clear, treat
            # the command as done and surface the fault as a warning instead of
            # failing the disable confirmation.
            if (not enable) and (not bool(st.get("active", False))):
                print(f"  [WARN] Drive reports a fault/alarm while already inactive: {trip_reason}")
                print("  Servo disable goal met (Active=False). Clear the fault separately if needed.")
                ok = True
                break
            print("  [FAIL] Drive alarm/fault detected during servo state command.")
            print(f"  Reason: {trip_reason}")
            break
        print(f"  Attempt {i+1}: ready={st['ready']} active={st['active']} "
              f"fault={st['fault']} {format_status_fault_summary(st)}")
        if enable:
            # Ready indicates the drive accepted I/O mode control path; Active is
            # preferred evidence of torque state when provided by the device.
            if st["ready"]:
                ok = True
            if st["active"]:
                ok = True
                break
        else:
            # Many drives keep Ready asserted when power is available; Active low
            # is the practical disable confirmation.
            if (not st["active"]) or (not st["ready"]):
                ok = True
                break

    if enable and ok and not active_seen:
        print("  [WARN] Active bit never asserted during enable attempts.")
        print("  If shaft is still free, verify STO/SON wiring and drive enable chain")
        print("  per docs/LOW_LEVEL_GANTRY_CONTROL.md and docs/EXPECTED_ELECTROMECHANICAL_ASSEMBLY.md.")
    if not status_seen:
        print("  [WARN] No valid status frames received while applying servo command.")

    # Keep the Class 1 session alive while servo is enabled so torque hold
    # does not drop when the command exits.
    if enable and ok:
        print("  [OK] Servo enable accepted; maintaining cyclic hold session.")
        if hold_seconds > 0:
            print(f"  Hold timeout: {hold_seconds:.1f}s")
        else:
            print("  Press Ctrl+C to end hold and release ownership.")
        frame_count = 0
        hold_started = time.time()
        try:
            while True:
                if hold_seconds > 0 and (time.time() - hold_started) >= hold_seconds:
                    print("  [OK] Hold timeout reached.")
                    break
                to_data = client.exchange_io_frame(ot_conn_id, hold_cmd, include_run_idle=True, timeout=2.0)
                frame_count += 1
                if to_data is None:
                    continue
                st = parse_input_assembly_154(to_data)
                if "error" in st:
                    continue
                if frame_count % 20 == 0:
                    print(f"  Hold: ready={st['ready']} active={st['active']} "
                          f"fault={st['fault']} {format_status_fault_summary(st)}")
                trip_reason = get_drive_trip_reason(st)
                if trip_reason is not None:
                    print("  [FAIL] Drive alarm/fault detected during servo hold.")
                    print(f"  Reason: {trip_reason}")
                    break
                time.sleep(max(rpi_us / 1000000.0, 0.01))
        except KeyboardInterrupt:
            print("\n  [OK] Hold stop requested by user.")

    client.forward_close(result.get("params", params))
    client.unregister_session()
    client.disconnect()

    if ok and ((not enable) or active_seen):
        print(f"[OK] Servo {action.lower()} command completed.")
    elif ok and enable and (not active_seen):
        print("[WARN] Servo enable command sent, but drive never reported Active=1.")
        print("  Torque hold is not confirmed. Verify STO/SON/enable-chain hardware.")
    else:
        print(f"[WARN] Servo {action.lower()} command not confirmed in {attempts} attempt(s).")

        if enable:
            print("  If the shaft remains free, verify:")
            print("    - STO circuit closed")
            print("    - SON/enable input chain wiring")
            print("    - Drive front-panel state transition on ServoOn")

        return


def try_reacquire_hold(client: "EipClient", ot_conn_id: int,
                       pulse_cmd: bytes, hold_cmd: bytes,
                       retries: int = 8, timeout: float = 2.0) -> tuple[bool, Optional[dict]]:
    """Try to reacquire active hold state without tearing down connection."""
    last_status = None
    for i in range(max(1, retries)):
        cmd = pulse_cmd if i == 0 else hold_cmd
        to_data = client.exchange_io_frame(
            ot_conn_id, cmd, include_run_idle=True, timeout=timeout,
            reuse_socket=True, drain=True)
        if to_data is None:
            continue
        st = parse_input_assembly_154(to_data)
        if "error" in st:
            continue
        last_status = st
        if bool(st.get("active", False)):
            return True, st
    return False, last_status


def _hold_extract_assembly(rx_data: bytes) -> Optional[bytes]:
    for type_id, item_data in parse_cpf(rx_data):
        if type_id == CPF_CONNECTED_DATA and len(item_data) >= 2:
            return item_data[2:]
    return None


def _hold_build_ot_cpf(ot_conn_id: int, assembly_data: bytes) -> bytes:
    encap_seq = int(time.time() * 1000) & 0xFFFFFFFF
    cip_seq = encap_seq & 0xFFFF
    addr_data = le32(ot_conn_id) + le32(encap_seq)
    data = le16(cip_seq) + le32(0x00000001) + assembly_data
    return build_cpf([
        (CPF_SEQUENCED_ADDRESS, addr_data),
        (CPF_CONNECTED_DATA, data),
    ])


def _hold_dual_exchange(udp: socket.socket,
                        axes: list["_HoldAxis"],
                        cmds: list[bytes],
                        timeout: float = 0.5) -> dict[str, bytes]:
    """Send O->T to each axis and demux T->O replies by source IP."""
    for ax, cmd in zip(axes, cmds):
        udp.sendto(_hold_build_ot_cpf(ax.ot_id, cmd), (ax.host, UDP_PORT))

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
        assy = _hold_extract_assembly(rx)
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
            assy = _hold_extract_assembly(rx)
            if assy is not None:
                latest[addr[0]] = assy
    finally:
        try:
            udp.settimeout(timeout)
        except Exception:
            pass
    return latest


@dataclass
class _HoldAxis:
    """Per-drive state for the dual-axis persistent hold worker."""
    name: str
    host: str
    ot_seed: int
    serial: int
    client: Optional["EipClient"] = None
    fo_result: Optional[dict] = None
    fo_params: Optional[ForwardOpenParams] = None
    ot_id: int = 0
    hold_position_puu: int = 0
    hold_cmd: bytes = field(default_factory=bytes)
    pulse_cmd: bytes = field(default_factory=bytes)
    hold_cmd_fallback: bytes = field(default_factory=bytes)
    clear_high_cmd: bytes = field(default_factory=bytes)
    clear_low_cmd: bytes = field(default_factory=bytes)
    settle_cmd: bytes = field(default_factory=bytes)
    steady_hold_cmd: bytes = field(default_factory=bytes)
    using_fallback: bool = False
    active_seen: bool = False
    clean_active_frames: int = 0
    a603_clear_attempts: int = 0
    last_status: Optional[dict] = None
    no_feedback_cycles: int = 0
    fatal_cycles: int = 0

    def snapshot(self) -> dict:
        st = self.last_status or {}
        return {
            "host": self.host,
            "active": bool(st.get("active", False)),
            "ready": bool(st.get("ready", False)),
            "fault": bool(st.get("fault", False)),
            "warning_present": bool(st.get("warning_present", False)),
            "warning_code": int(st.get("warning_code", 0)),
            "hold_position_puu": self.hold_position_puu,
            "profile": ("fallback_position_hold"
                        if self.using_fallback else "neutral_hold"),
            "active_seen": self.active_seen,
            "clean_active_frames": self.clean_active_frames,
        }


def _hold_open_axis(ax: _HoldAxis, rpi_us: int, shared_udp: socket.socket) -> Optional[str]:
    """ForwardOpen one axis onto the shared UDP socket. Returns error string or None."""
    client = EipClient(host=ax.host)
    client.verbose = False
    if not client.connect():
        return f"{ax.name}: TCP connect failed ({ax.host})"
    if not client.register_session():
        client.disconnect()
        return f"{ax.name}: RegisterSession failed ({ax.host})"

    status = client.check_drive_ready()
    client.unregister_session()
    client.disconnect()

    if not status.get("powered_on", False):
        return f"{ax.name}: drive not powered/responding ({ax.host})"
    if not status.get("io_mode", False):
        return f"{ax.name}: not in IO mode P1.001!=0x000C ({ax.host})"

    if not client.connect():
        return f"{ax.name}: TCP reconnect failed ({ax.host})"
    if not client.register_session():
        client.disconnect()
        return f"{ax.name}: RegisterSession failed on reconnect ({ax.host})"

    params = ForwardOpenParams()
    params.ot_assembly_size = 40
    params.to_assembly_size = 52
    params.ot_rpi_us = rpi_us
    params.to_rpi_us = rpi_us
    params.raw_ot_conn_size = 40 + 2 + 4
    params.include_run_idle_header = True
    params.connection_timeout_multiplier = 7
    params.ot_connection_id = ax.ot_seed
    params.connection_serial = ax.serial

    result = forward_open_with_retry(client, params)
    ot_conn_id = int((result or {}).get("ot_connection_id", 0) or 0)
    if ot_conn_id == 0:
        ext = int((result or {}).get("extended_status", 0) or 0)
        try:
            client.unregister_session()
            client.disconnect()
        except Exception:
            pass
        return (f"{ax.name}: ForwardOpen failed "
                f"(extended_status=0x{ext:04X})")

    client.io_sock = shared_udp
    ax.client = client
    ax.fo_result = result
    ax.fo_params = (result or {}).get("params", params)
    ax.ot_id = ot_conn_id
    return None


def _hold_keepalive_cmds(axes: list[_HoldAxis]) -> list[bytes]:
    """Neutral keep-alive images for already-opened axes (pre-arm)."""
    cmds = []
    for ax in axes:
        if ax.steady_hold_cmd:
            cmds.append(ax.steady_hold_cmd)
        elif ax.hold_cmd:
            cmds.append(ax.hold_cmd)
        else:
            cmds.append(build_output_assembly_104(
                servo_on=False, servo_off=False, operating_mode=0,
                travel_mode=10, torque_ramp_time_ms=1000))
    return cmds


def _hold_open_axes_with_keepalive(axes: list[_HoldAxis], rpi_us: int,
                                   shared_udp: socket.socket,
                                   frame_period: float,
                                   token: str) -> Optional[str]:
    """ForwardOpen each axis while streaming O->T to already-open axes."""
    opened: list[_HoldAxis] = []
    stop_ka = threading.Event()
    ka_error: list[str] = []

    def ka_loop():
        while not stop_ka.wait(frame_period):
            live = list(opened)
            if not live:
                continue
            try:
                _hold_dual_exchange(
                    shared_udp, live, _hold_keepalive_cmds(live), timeout=0.2)
            except Exception as exc:
                ka_error.append(str(exc))
                return

    ka_thread = threading.Thread(target=ka_loop, name="hold-keepalive",
                                 daemon=True)
    ka_thread.start()
    try:
        for ax in axes:
            _update_hold_state_for_token(
                token,
                status="starting",
                last_seen=_now_iso(),
                opening=ax.name,
                hosts=[a.host for a in axes])
            err = _hold_open_axis(ax, rpi_us, shared_udp)
            if err:
                return err
            opened.append(ax)
            print(f"[hold worker] {ax.name} Class 1 open ({ax.host})")
            # Immediate frame so the new connection is not idle before the
            # keepalive thread's next tick.
            _hold_dual_exchange(
                shared_udp, opened, _hold_keepalive_cmds(opened), timeout=0.5)
    finally:
        stop_ka.set()
        ka_thread.join(timeout=2.0)
    if ka_error:
        print(f"[hold worker] keepalive warning: {ka_error[0]}")
    return None


def _hold_build_axis_cmds(ax: _HoldAxis) -> None:
    """Build idle / pulse / fallback / A603-clear command images for one axis."""
    ax.hold_cmd = build_output_assembly_104(
        servo_on=True,
        servo_off=False,
        speed_rpm=0.0,
        operating_mode=0,
        travel_mode=10,
        torque_ramp_time_ms=1000,
        start_motion=False,
        stop_motion=False)
    ax.settle_cmd = build_output_assembly_104(
        servo_on=True,
        operating_mode=0,
        travel_mode=10,
        torque_ramp_time_ms=1000)
    ax.clear_high_cmd = build_output_assembly_104(
        servo_on=True,
        fault_reset=True,
        operating_mode=0,
        travel_mode=10,
        torque_ramp_time_ms=1000)
    ax.clear_low_cmd = build_output_assembly_104(
        servo_on=True,
        fault_reset=False,
        operating_mode=0,
        travel_mode=10,
        torque_ramp_time_ms=1000)
    ax.pulse_cmd = build_output_assembly_104(
        servo_on=True,
        servo_off=False,
        speed_rpm=5.0,
        accel_rpm_per_s=DEFAULT_ACCEL_RPM_PER_S,
        decel_rpm_per_s=DEFAULT_DECEL_RPM_PER_S,
        operating_mode=1,
        position_puu=ax.hold_position_puu,
        travel_mode=2,
        torque_ramp_time_ms=1000,
        start_motion=True)
    ax.hold_cmd_fallback = build_output_assembly_104(
        servo_on=True,
        servo_off=False,
        speed_rpm=5.0,
        accel_rpm_per_s=DEFAULT_ACCEL_RPM_PER_S,
        decel_rpm_per_s=DEFAULT_DECEL_RPM_PER_S,
        operating_mode=1,
        position_puu=ax.hold_position_puu,
        travel_mode=2,
        torque_ramp_time_ms=1000,
        start_motion=False)
    ax.steady_hold_cmd = ax.hold_cmd
    ax.using_fallback = False
    ax.active_seen = False
    ax.clean_active_frames = 0
    ax.a603_clear_attempts = 0


def _hold_arm_heartbeat(token: str, axes: list[_HoldAxis], phase: str) -> None:
    agg = _hold_aggregate_state(axes)
    _update_hold_state_for_token(
        token,
        status="arming",
        arm_phase=phase,
        last_seen=_now_iso(),
        **agg)


def _hold_arm_axes(udp: socket.socket, axes: list[_HoldAxis],
                   frame_period: float, token: str) -> Optional[str]:
    """Seed, clear A603, and arm all axes in lockstep so neither starves."""
    for ax in axes:
        assert ax.client is not None

    # Seed both together (servo off) to read actual positions.
    _hold_arm_heartbeat(token, axes, "seed")
    seed_cmds = [
        build_output_assembly_104(
            servo_on=False, servo_off=False, operating_mode=0,
            travel_mode=10, torque_ramp_time_ms=1000)
        for _ in axes
    ]
    seed_map = _hold_dual_exchange(udp, axes, seed_cmds, timeout=2.0)
    seed_active: dict[str, bool] = {}
    for ax in axes:
        seed = seed_map.get(ax.host)
        if seed is not None:
            seed_status = parse_input_assembly_154(seed)
            if "error" not in seed_status:
                ax.last_status = seed_status
                ax.hold_position_puu = int(seed_status.get("actual_position", 0))
                seed_trip = get_drive_trip_reason(seed_status)
                if seed_trip is not None and bool(seed_status.get("fault", False)):
                    return f"{ax.name}: drive fault blocks servo enable: {seed_trip}"
        seed_active[ax.name] = bool((ax.last_status or {}).get("active", False))
        _hold_build_axis_cmds(ax)

    # Settle + A603 clear (same pattern as move/home) before enabling hold.
    _hold_arm_heartbeat(token, axes, "a603-clear")
    settle_deadline = time.time() + 3.0
    while time.time() < settle_deadline:
        cmds = []
        for ax in axes:
            st = ax.last_status or {}
            if (bool(st.get("warning_present", False))
                    and (int(st.get("warning_code", 0)) & 0x0FFF) == 0x0603
                    and ax.a603_clear_attempts < 2):
                ax.a603_clear_attempts += 1
                print(f"[hold worker] {ax.name}: clearing latched A603...")
                for _ in range(3):
                    _hold_dual_exchange(
                        udp, axes,
                        [a.clear_high_cmd for a in axes], timeout=0.5)
                    time.sleep(frame_period)
                for _ in range(4):
                    _hold_dual_exchange(
                        udp, axes,
                        [a.clear_low_cmd for a in axes], timeout=0.5)
                    time.sleep(frame_period)
                cmds = None
                break
            cmds.append(ax.settle_cmd)
        if cmds is None:
            _hold_arm_heartbeat(token, axes, "a603-clear")
            continue
        rx_map = _hold_dual_exchange(udp, axes, cmds, timeout=0.5)
        all_clean = True
        for ax in axes:
            to_data = rx_map.get(ax.host)
            if to_data is None:
                all_clean = False
                continue
            st = parse_input_assembly_154(to_data)
            if "error" in st:
                all_clean = False
                continue
            ax.last_status = st
            if bool(st.get("fault", False)):
                return (f"{ax.name}: drive fault during hold settle: "
                        f"{get_drive_trip_reason(st)}")
            warn = (bool(st.get("warning_present", False))
                    and (int(st.get("warning_code", 0)) & 0x0FFF) == 0x0603)
            if not (bool(st.get("active", False)) and bool(st.get("ready", False))
                    and not warn):
                all_clean = False
        _hold_arm_heartbeat(token, axes, "settle")
        if all_clean:
            break
        time.sleep(frame_period)

    warning_seen: dict[str, bool] = {ax.name: False for ax in axes}
    _hold_arm_heartbeat(token, axes, "enable")

    for i in range(8):
        cmds = []
        for ax in axes:
            if seed_active[ax.name] or i < 4:
                cmds.append(ax.hold_cmd)
            elif i == 4:
                cmds.append(ax.pulse_cmd)
            else:
                cmds.append(ax.hold_cmd_fallback)
        rx_map = _hold_dual_exchange(udp, axes, cmds, timeout=2.0)
        for ax, cmd in zip(axes, cmds):
            to_data = rx_map.get(ax.host)
            if to_data is None:
                continue
            st = parse_input_assembly_154(to_data)
            if "error" in st:
                continue
            ax.last_status = st
            ax.active_seen = ax.active_seen or bool(st.get("active", False))
            warning_seen[ax.name] = (warning_seen[ax.name]
                                     or bool(st.get("warning_present", False)))
            if bool(st.get("active", False)):
                if cmd is ax.hold_cmd_fallback or cmd is ax.pulse_cmd:
                    ax.steady_hold_cmd = ax.hold_cmd_fallback
                    ax.using_fallback = True
                else:
                    ax.steady_hold_cmd = ax.hold_cmd
                    ax.using_fallback = False
        if i % 2 == 0:
            _hold_arm_heartbeat(token, axes, f"enable-{i}")
        time.sleep(frame_period)

    for ax in axes:
        if ax.active_seen:
            continue
        reacquired, reacq_status = _hold_try_reacquire_axis(
            udp, axes, ax, retries=12, timeout=2.0)
        if reacq_status and "error" not in reacq_status:
            ax.last_status = reacq_status
        if reacquired:
            ax.active_seen = True
            ax.steady_hold_cmd = ax.hold_cmd_fallback
            ax.using_fallback = True
            _hold_arm_heartbeat(token, axes, f"reacquire-{ax.name}")
            continue
        last_ready = bool((reacq_status or {}).get("ready", False))
        if last_ready and get_hold_fatal_reason(reacq_status or {}) is None:
            print(f"[hold worker] {ax.name}: Active not asserted yet; "
                  f"continuing on Ready with fallback position hold")
            ax.active_seen = True
            ax.steady_hold_cmd = ax.hold_cmd_fallback
            ax.using_fallback = True
            continue
        trip = get_drive_trip_reason(ax.last_status or {})
        summary = format_status_fault_summary(ax.last_status or {})
        if trip:
            detail = trip
        elif warning_seen[ax.name]:
            detail = f"warning present during hold handoff; {summary}"
        else:
            detail = (f"active bit did not assert "
                      f"(ready={bool((ax.last_status or {}).get('ready', False))}, "
                      f"{summary})")
        return f"{ax.name}: background hold not confirmed: {detail}"
    return None


def _hold_try_reacquire_axis(udp: socket.socket,
                             axes: list[_HoldAxis],
                             target: _HoldAxis,
                             retries: int = 8,
                             timeout: float = 0.5) -> tuple[bool, Optional[dict]]:
    """Pulse one axis while continuing steady hold frames on the others."""
    last_status = None
    for i in range(max(1, retries)):
        cmds = []
        for ax in axes:
            if ax is target:
                cmds.append(ax.pulse_cmd if i == 0 else ax.hold_cmd_fallback)
            else:
                cmds.append(ax.steady_hold_cmd or ax.hold_cmd)
        rx_map = _hold_dual_exchange(udp, axes, cmds, timeout=timeout)
        for ax in axes:
            to_data = rx_map.get(ax.host)
            if to_data is None:
                continue
            st = parse_input_assembly_154(to_data)
            if "error" in st:
                continue
            ax.last_status = st
            if ax is target:
                last_status = st
        if last_status is not None and bool(last_status.get("active", False)):
            return True, last_status
    return False, last_status


def _hold_maybe_promote_idle(ax: _HoldAxis) -> None:
    """After enough clean Active frames, prefer neutral idle hold over fallback."""
    st = ax.last_status or {}
    clean = (bool(st.get("active", False))
             and bool(st.get("ready", False))
             and not bool(st.get("fault", False))
             and not (bool(st.get("warning_present", False))
                      and (int(st.get("warning_code", 0)) & 0x0FFF) == 0x0603))
    if clean:
        ax.clean_active_frames += 1
    else:
        ax.clean_active_frames = 0
        return
    if (ax.using_fallback
            and ax.hold_cmd
            and ax.clean_active_frames >= HOLD_IDLE_PROMOTE_FRAMES):
        ax.steady_hold_cmd = ax.hold_cmd
        ax.using_fallback = False
        print(f"[hold worker] {ax.name}: promoted to neutral idle hold")


def _hold_aggregate_state(axes: list[_HoldAxis]) -> dict:
    """Top-level state fields for callers that still expect a single-drive shape."""
    snaps = {ax.name: ax.snapshot() for ax in axes}
    return {
        "axes": snaps,
        "active": all(bool(s.get("active", False)) for s in snaps.values()),
        "active_seen": all(bool(s.get("active_seen", False)) for s in snaps.values()),
        "ready": all(bool(s.get("ready", False)) for s in snaps.values()),
        "fault": any(bool(s.get("fault", False)) for s in snaps.values()),
        "warning_present": any(bool(s.get("warning_present", False)) for s in snaps.values()),
        "warning_code": next(
            (int(s.get("warning_code", 0)) for s in snaps.values()
             if int(s.get("warning_code", 0))),
            0),
        "hold_position_puu": (snaps.get("X") or {}).get("hold_position_puu", 0),
        "profile": ("fallback_position_hold"
                    if any(ax.using_fallback for ax in axes) else "neutral_hold"),
        "hosts": [ax.host for ax in axes],
    }


def run_background_hold_worker(token: str, rpi_us: int,
                               x_ip: str = DRIVE_IP_X,
                               z_ip: str = DRIVE_IP_Z):
    """Worker process that keeps persistent servo-on hold alive on X and Z.

    Runs detached (see start_background_hold) and owns both drives' Class 1
    connections on shared UDP 2222 until the state file's ``stop`` flag is set
    or its ``token`` is superseded by a newer hold.

    Reliability invariants (do not regress -- torque loss here is silent, i.e.
    no drive fault, see the module docstring):
      * O->T frames must keep flowing at the RPI cadence. Nothing in the loop
        (disk I/O, logging) may block long enough to exceed the connection
        timeout, or the drive drops the connection and releases torque.
      * Use one persistent UDP socket for both axes (demux by source IP) so
        port 2222 stays bound between frames; a fresh per-frame socket triggers
        ICMP port-unreachable and drops the connection.
      * Throttle state-file reads/writes far below the frame rate; only poll the
        stop flag and write the heartbeat periodically.
      * Keepalive already-open axes while ForwardOpening the next axis.
    """
    pid = os.getpid()
    _boost_current_process_priority()
    print(f"=== servo hold worker start pid={pid} token={token[:8]} "
          f"rpi={rpi_us}us X={x_ip} Z={z_ip} ===")
    _update_hold_state_for_token(
        token,
        pid=pid,
        status="starting",
        stop=False,
        supervise=True,
        worker_started_at=_now_iso(),
        rpi_us=int(rpi_us),
        hosts=[x_ip, z_ip])

    def mark_error(message: str):
        print(f"[hold worker] ERROR: {message}")
        _update_hold_state_for_token(
            token,
            status="error",
            last_error=message,
            last_seen=_now_iso())

    axes = [
        _HoldAxis("X", x_ip, ot_seed=0x10000001, serial=0x0001),
        _HoldAxis("Z", z_ip, ot_seed=0x10000002, serial=0x0002),
    ]
    shared_udp: Optional[socket.socket] = None
    stop_requested = False
    frame_period = max(rpi_us / 1_000_000.0, 0.005)

    try:
        shared_udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        shared_udp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            shared_udp.bind(("0.0.0.0", UDP_PORT))
        except OSError as exc:
            mark_error(f"Could not bind UDP {UDP_PORT}: {exc}")
            return

        err = _hold_open_axes_with_keepalive(
            axes, rpi_us, shared_udp, frame_period, token)
        if err:
            mark_error(err)
            return

        _update_hold_state_for_token(
            token,
            status="arming",
            last_seen=_now_iso(),
            hosts=[ax.host for ax in axes])
        err = _hold_arm_axes(shared_udp, axes, frame_period, token)
        if err:
            mark_error(err)
            return
        for ax in axes:
            print(f"[hold worker] {ax.name} hold armed "
                  f"(active_seen={ax.active_seen}, "
                  f"fallback={ax.using_fallback})")

        agg = _hold_aggregate_state(axes)
        _update_hold_state_for_token(
            token,
            status="holding",
            supervise=True,
            last_seen=_now_iso(),
            **agg)

        # Steady-state hold. Keep sending O->T frames at the RPI cadence for
        # both axes; throttle disk I/O far below the frame rate.
        stop_check_interval = 0.1
        heartbeat_interval = 0.3
        state_stale_grace = 5.0

        last_stop_check = 0.0
        last_heartbeat = 0.0
        last_good_state = time.time()

        while True:
            now = time.time()

            if now - last_stop_check >= stop_check_interval:
                last_stop_check = now
                state = _load_hold_state_raw()
                if state is None:
                    if now - last_good_state > state_stale_grace:
                        mark_error("Hold state file unavailable for too long")
                        break
                else:
                    last_good_state = now
                    if state.get("token") != token:
                        break  # superseded by a newer hold session
                    if bool(state.get("stop", False)):
                        stop_requested = True
                        break

            cmds = [ax.steady_hold_cmd for ax in axes]
            rx_map = _hold_dual_exchange(
                shared_udp, axes, cmds, timeout=0.5)

            for ax in axes:
                to_data = rx_map.get(ax.host)
                if to_data is None:
                    ax.no_feedback_cycles += 1
                    ax.clean_active_frames = 0
                    if ax.no_feedback_cycles >= 4:
                        _update_hold_state_for_token(
                            token, status="recovering",
                            last_error=(f"{ax.name}: no T->O feedback; "
                                        f"attempting hold reacquire"),
                            last_seen=_now_iso())
                        reacquired, reacq_status = _hold_try_reacquire_axis(
                            shared_udp, axes, ax, retries=8, timeout=0.5)
                        ax.no_feedback_cycles = 0
                        if reacquired:
                            ax.steady_hold_cmd = ax.hold_cmd_fallback
                            ax.using_fallback = True
                            ax.clean_active_frames = 0
                            if reacq_status:
                                ax.last_status = reacq_status
                            last_heartbeat = 0.0
                    continue

                ax.no_feedback_cycles = 0
                st = parse_input_assembly_154(to_data)
                if "error" in st:
                    continue
                fatal_reason = get_hold_fatal_reason(st)
                if fatal_reason is not None:
                    ax.fatal_cycles += 1
                    ax.clean_active_frames = 0
                    if ax.fatal_cycles >= 20:
                        _update_hold_state_for_token(
                            token, status="recovering",
                            last_error=f"{ax.name}: {fatal_reason}",
                            last_seen=_now_iso())
                        reacquired, reacq_status = _hold_try_reacquire_axis(
                            shared_udp, axes, ax, retries=8, timeout=0.5)
                        ax.fatal_cycles = 0
                        if reacquired:
                            ax.steady_hold_cmd = ax.hold_cmd_fallback
                            ax.using_fallback = True
                            if reacq_status:
                                ax.last_status = reacq_status
                            last_heartbeat = 0.0
                else:
                    ax.fatal_cycles = 0
                    ax.last_status = st
                    ax.active_seen = ax.active_seen or bool(st.get("active", False))
                    # Opportunistic A603 clear during steady hold.
                    if (bool(st.get("warning_present", False))
                            and (int(st.get("warning_code", 0)) & 0x0FFF) == 0x0603
                            and ax.a603_clear_attempts < 4):
                        ax.a603_clear_attempts += 1
                        print(f"[hold worker] {ax.name}: clearing A603 in steady hold...")
                        # Keep heartbeat fresh so the watchdog does not treat
                        # this brief blocking clear as a dead worker.
                        _update_hold_state_for_token(
                            token, status="holding", supervise=True,
                            last_seen=_now_iso(), last_error=f"{ax.name}:a603-clear",
                            **_hold_aggregate_state(axes))
                        for _ in range(2):
                            _hold_dual_exchange(
                                shared_udp, axes,
                                [a.clear_high_cmd or a.steady_hold_cmd for a in axes],
                                timeout=0.5)
                            time.sleep(frame_period)
                        for _ in range(3):
                            _hold_dual_exchange(
                                shared_udp, axes,
                                [a.clear_low_cmd or a.steady_hold_cmd for a in axes],
                                timeout=0.5)
                            time.sleep(frame_period)
                        ax.clean_active_frames = 0
                        last_heartbeat = 0.0
                        continue
                    _hold_maybe_promote_idle(ax)

            if now - last_heartbeat >= heartbeat_interval:
                last_heartbeat = now
                agg = _hold_aggregate_state(axes)
                _update_hold_state_for_token(
                    token,
                    status="holding",
                    supervise=True,
                    last_seen=_now_iso(),
                    **agg)

            time.sleep(frame_period)

        # On an explicit stop, yield ownership cleanly with neutral frames.
        if stop_requested and any(ax.hold_cmd for ax in axes):
            for _ in range(3):
                cmds = [ax.hold_cmd or ax.steady_hold_cmd for ax in axes]
                _hold_dual_exchange(shared_udp, axes, cmds, timeout=1.0)
                time.sleep(frame_period)

    except Exception as exc:
        mark_error(f"{type(exc).__name__}: {exc}")
    finally:
        print(f"=== servo hold worker exit pid={pid} token={token[:8]} "
              f"at={_now_iso()} ===")
        forward_close_ok = True
        any_opened = False
        for ax in axes:
            if ax.client is None:
                continue
            any_opened = True
            ax_ok = False
            try:
                # Detach shared UDP so client close does not close it early.
                ax.client.io_sock = None
                if ax.fo_params is not None:
                    ax_ok = bool(ax.client.forward_close(ax.fo_params))
            except Exception as exc:
                print(f"[hold worker] {ax.name} ForwardClose exception: {exc}")
                ax_ok = False
            try:
                ax.client.unregister_session()
            except Exception:
                pass
            try:
                ax.client.disconnect()
            except Exception:
                pass
            forward_close_ok = forward_close_ok and ax_ok
            print(f"[hold worker] {ax.name} forward_close_ok={ax_ok}")

        if not any_opened:
            forward_close_ok = False

        if shared_udp is not None:
            try:
                shared_udp.close()
            except Exception:
                pass

        # Leave a stopped marker so the handoff waiter knows whether the drives
        # received ForwardClose. Do not clear the file here — the caller that
        # requested the stop clears it after reading forward_close_ok.
        state = _load_hold_state_raw()
        if state and state.get("token") == token:
            if state.get("status") != "error":
                state["status"] = "stopped"
            state["forward_close_ok"] = forward_close_ok
            state["stopped_at"] = _now_iso()
            state["updated_at"] = _now_iso()
            _write_hold_state_raw(state)
        print(f"[hold worker] forward_close_ok={forward_close_ok}")


def _load_watchdog_state() -> Optional[dict]:
    return _read_json_file(HOLD_WATCHDOG_STATE_FILE)


def _write_watchdog_state(state: dict) -> None:
    _write_json_file_atomic(HOLD_WATCHDOG_STATE_FILE, state)


def _clear_watchdog_state() -> None:
    try:
        HOLD_WATCHDOG_STATE_FILE.unlink(missing_ok=True)
    except Exception:
        pass


def get_hold_watchdog_state() -> Optional[dict]:
    state = _load_watchdog_state()
    if not state:
        return None
    if str(state.get("status", "")) in ("stopped", "error"):
        return None
    try:
        pid = int(state.get("pid", 0))
    except Exception:
        pid = 0
    if pid > 0 and not _is_process_alive(pid):
        _clear_watchdog_state()
        return None
    return state


def start_hold_watchdog(rpi_us: int = 5000) -> bool:
    """Ensure the hold supervisor process is running."""
    existing = get_hold_watchdog_state()
    if existing:
        return True

    token = uuid.uuid4().hex
    state = {
        "token": token,
        "pid": 0,
        "rpi_us": int(rpi_us),
        "status": "starting",
        "stop": False,
        "created_at": _now_iso(),
        "updated_at": _now_iso(),
    }
    _write_watchdog_state(state)
    command = [
        sys.executable,
        str(Path(__file__).resolve()),
        "__hold-watchdog",
        "--token",
        token,
        "--rpi",
        str(int(rpi_us)),
    ]
    try:
        if (HOLD_WATCHDOG_LOG_FILE.exists()
                and HOLD_WATCHDOG_LOG_FILE.stat().st_size > HOLD_LOG_MAX_BYTES):
            HOLD_WATCHDOG_LOG_FILE.unlink(missing_ok=True)
    except Exception:
        pass
    try:
        proc = _spawn_detached_python(command, log_path=HOLD_WATCHDOG_LOG_FILE)
    except Exception as exc:
        _clear_watchdog_state()
        print(f"  [WARN] Could not start hold watchdog: {exc}")
        return False
    state["pid"] = int(proc.pid)
    state["status"] = "running"
    state["updated_at"] = _now_iso()
    _write_watchdog_state(state)
    print(f"  [OK] Hold watchdog running (pid={proc.pid}).")
    return True


def stop_hold_watchdog(reason: str = "stop", timeout_s: float = 5.0) -> bool:
    state = get_hold_watchdog_state()
    if not state:
        _clear_watchdog_state()
        return True
    pid = int(state.get("pid", 0))
    state["stop"] = True
    state["stop_reason"] = reason
    state["updated_at"] = _now_iso()
    _write_watchdog_state(state)
    deadline = time.time() + max(timeout_s, 0.1)
    while time.time() < deadline:
        if not _is_process_alive(pid):
            _clear_watchdog_state()
            return True
        time.sleep(0.05)
    return False


def run_hold_watchdog_worker(token: str, rpi_us: int):
    """Restart supervised hold when heartbeat goes stale or Active drops."""
    pid = os.getpid()
    print(f"=== hold watchdog start pid={pid} token={token[:8]} rpi={rpi_us}us ===")
    state = _load_watchdog_state()
    if state and state.get("token") == token:
        state["pid"] = pid
        state["status"] = "running"
        state["updated_at"] = _now_iso()
        _write_watchdog_state(state)

    inactive_ticks = 0
    restart_cooldown_until = 0.0

    try:
        while True:
            wd = _load_watchdog_state()
            if wd is None or wd.get("token") != token or bool(wd.get("stop", False)):
                break
            wd["updated_at"] = _now_iso()
            wd["last_seen"] = _now_iso()
            _write_watchdog_state(wd)

            if _hold_pause_marker_active():
                inactive_ticks = 0
                time.sleep(HOLD_WATCHDOG_POLL_S)
                continue

            hold = _load_hold_state_raw()
            now = time.time()
            need_restart = False
            reason = ""

            if hold is None:
                # No hold file and not paused → revive if we were supervising.
                need_restart = True
                reason = "missing-hold-state"
            else:
                supervise = bool(hold.get("supervise", True))
                status = str(hold.get("status", ""))
                if not supervise or status in ("stopped",):
                    inactive_ticks = 0
                    time.sleep(HOLD_WATCHDOG_POLL_S)
                    continue
                if status == "error":
                    need_restart = True
                    reason = f"hold-error:{hold.get('last_error', '')}"
                elif status in ("holding", "recovering", "arming", "starting"):
                    try:
                        hold_pid = int(hold.get("pid", 0))
                    except Exception:
                        hold_pid = 0
                    if hold_pid > 0 and not _is_process_alive(hold_pid):
                        need_restart = True
                        reason = "hold-process-dead"
                    else:
                        age_s = HOLD_WATCHDOG_STALE_S + 1
                        updated_text = str(hold.get("updated_at", "") or "")
                        if updated_text:
                            try:
                                age_s = (datetime.now()
                                         - datetime.fromisoformat(updated_text)
                                         ).total_seconds()
                            except Exception:
                                pass
                        if status == "holding" and age_s > HOLD_WATCHDOG_STALE_S:
                            need_restart = True
                            reason = f"stale-heartbeat:{age_s:.1f}s"
                        elif status == "holding" and not bool(hold.get("active", False)):
                            inactive_ticks += 1
                            if inactive_ticks >= 3:
                                need_restart = True
                                reason = "active-dropped"
                        else:
                            inactive_ticks = 0

            if need_restart and now >= restart_cooldown_until:
                print(f"[hold watchdog] restarting hold ({reason})")
                restart_cooldown_until = now + 8.0
                try:
                    # Stop any zombie worker/file without clearing pause marker logic.
                    if get_background_hold_state():
                        stop_background_hold(f"watchdog:{reason}", timeout_s=6.0)
                    _clear_hold_state_file()
                    time.sleep(HOLD_OWNERSHIP_SETTLE_CLEAN_S)
                    if not start_background_hold(rpi_us=rpi_us,
                                                 start_watchdog=False):
                        print("[hold watchdog] restart failed")
                except Exception as exc:
                    print(f"[hold watchdog] restart exception: {exc}")
                inactive_ticks = 0

            time.sleep(HOLD_WATCHDOG_POLL_S)
    except Exception as exc:
        print(f"[hold watchdog] ERROR: {type(exc).__name__}: {exc}")
    finally:
        print(f"=== hold watchdog exit pid={pid} at={_now_iso()} ===")
        wd = _load_watchdog_state()
        if wd and wd.get("token") == token:
            wd["status"] = "stopped"
            wd["updated_at"] = _now_iso()
            _write_watchdog_state(wd)


def run_decode_command(code_text: str):
    """Offline display/fault/alarm decoder."""
    if code_text.strip().lower() in ("flags", "status-flags", "fault-flags"):
        print("\n=== Kinetix Input Assembly 154 Status Flags ===")
        for flag in DRIVE_INPUT_STATUS_FLAGS:
            print(f"  byte {flag['byte']:02d} bit {flag['bit']}: "
                  f"{flag['display']} ({flag['severity']}) -> {flag['field']}")
        return

    lookup_value = code_text
    if code_text.strip().lower().startswith("0x"):
        lookup_value = int(code_text, 16)
    decoded = decode_drive_code(lookup_value)
    if decoded is None:
        print(f"\n[WARN] No decoder entry found for {code_text}")
        normalized = normalize_display_code(lookup_value)
        if normalized:
            print(f"  Normalized display form: {normalized}")
        return

    print(f"\n=== Kinetix Code {decoded['display']} ===")
    print(f"  Type: {decoded.get('fault_type', '')}")
    print(f"  Name: {decoded.get('name', '')}")
    if decoded.get("description"):
        print("\n  Description:")
        print(f"  {decoded['description']}")
    if decoded.get("corrective_action"):
        print("\n  Corrective Action:")
        print(f"  {decoded['corrective_action']}")
    if decoded.get("servo_on") or decoded.get("servo_off"):
        print("\n  Reset Behavior:")
        print(f"  Servo On: {decoded.get('servo_on', '')}")
        print(f"  Servo Off: {decoded.get('servo_off', '')}")


def print_eds_summary(eds: dict, path: Path):
    params = eds.get("params", {})
    groups = eds.get("groups", {})
    print(f"\n=== EDS Parse: {path} ===")
    print(f"  Parameters: {len(params)}")
    print(f"  Groups: {len(groups)}")
    if groups:
        print("  Available groups:")
        for group_name in sorted(groups):
            print(f"    {group_name}: {len(groups[group_name])} params")


def run_dump_params_command(client: EipClient, args):
    eds_path = Path(args.eds)
    eds = parse_eds_parameters(eds_path)
    print_eds_summary(eds, eds_path)
    if args.dry_parse:
        return

    param_ids = select_parameter_ids(eds, args.ids, args.group)
    if not param_ids:
        print("  [FAIL] No parameters selected. Check --ids or --group.")
        return

    if not client.connect():
        return
    identity = client.list_identity()
    if not client.register_session():
        client.disconnect()
        return

    print(f"\n=== Dump Parameters ({len(param_ids)} selected) ===")
    snapshot = {
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "drive_identity": identity or {},
        "eds_path": str(eds_path),
        "params": {},
    }
    ok_count = 0
    fail_count = 0
    for param_id in param_ids:
        meta = eds["params"][param_id]
        raw = client.read_k5100_parameter_raw(param_id)
        if raw is None:
            fail_count += 1
            print(f"  [WARN] Param {param_id} {meta['name']}: read failed")
            continue
        value = decode_parameter_value(raw, meta.get("size", len(raw)), meta.get("data_type"))
        snapshot["params"][str(param_id)] = {
            "name": meta.get("name", ""),
            "value": value,
            "raw_hex": raw.hex(),
            "size": meta.get("size"),
            "data_type": meta.get("data_type"),
            "groups": meta.get("groups", []),
        }
        ok_count += 1
        print(f"  [OK] Param {param_id:03d} {meta['name']}: {value}")

    client.unregister_session()
    client.disconnect()

    out_path = Path(args.out) if args.out else Path(f"k5100_params_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json")
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(snapshot, f, indent=2, sort_keys=True)
        f.write("\n")
    print(f"\n[OK] Snapshot written to {out_path}")
    print(f"  Read OK: {ok_count}, failed/skipped: {fail_count}")


def run_restore_params_command(client: EipClient, args):
    snapshot_path = Path(args.snapshot)
    try:
        with open(snapshot_path, "r", encoding="utf-8") as f:
            snapshot = json.load(f)
    except (OSError, json.JSONDecodeError) as exc:
        print(f"\n[FAIL] Could not load snapshot {snapshot_path}: {exc}")
        return

    eds_path = Path(args.eds)
    eds = parse_eds_parameters(eds_path)
    snapshot_params = snapshot.get("params", {})
    if not snapshot_params:
        print("\n[FAIL] Snapshot contains no params.")
        return

    selected_ids = select_parameter_ids(eds, args.ids, args.group)
    if args.ids is None and args.group is None:
        selected_ids = [int(param_id) for param_id in snapshot_params if str(param_id).isdigit()]
    selected_ids = [param_id for param_id in selected_ids if str(param_id) in snapshot_params]
    if not selected_ids:
        print("\n[FAIL] No snapshot parameters selected for restore.")
        return

    if not client.connect():
        return
    if not client.register_session():
        client.disconnect()
        return

    print(f"\n=== Restore Parameter Diff ({len(selected_ids)} selected) ===")
    diffs = []
    for param_id in sorted(selected_ids):
        snap = snapshot_params[str(param_id)]
        meta = eds.get("params", {}).get(param_id, {})
        size = meta.get("size") or snap.get("size") or 2
        data_type = meta.get("data_type") if meta.get("data_type") is not None else snap.get("data_type")
        name = meta.get("name") or snap.get("name") or f"Param{param_id}"
        live = client.read_k5100_parameter(param_id, size=size, data_type=data_type)
        if live is None:
            print(f"  [WARN] Param {param_id:03d} {name}: live read failed")
            continue
        desired = snap.get("value")
        if live != desired:
            diffs.append({
                "id": param_id,
                "name": name,
                "live": live,
                "desired": desired,
                "size": size,
                "data_type": data_type,
            })
            print(f"  DIFF Param {param_id:03d} {name}: live={live} snapshot={desired}")
        else:
            print(f"  SAME Param {param_id:03d} {name}: {live}")

    if not diffs:
        print("\n[OK] Live parameters already match snapshot selection.")
        client.unregister_session()
        client.disconnect()
        return

    if not args.apply:
        print(f"\n[DRY-RUN] {len(diffs)} parameter(s) differ. Re-run with --apply to write.")
        client.unregister_session()
        client.disconnect()
        return

    confirmation = input(f"\nWrite {len(diffs)} differing parameter(s) to the drive? Type 'yes' to continue: ")
    if confirmation.strip().lower() != "yes":
        print("[ABORT] Restore cancelled by user.")
        client.unregister_session()
        client.disconnect()
        return

    ok_count = 0
    fail_count = 0
    for diff in diffs:
        ok = client.write_k5100_parameter(
            diff["id"],
            diff["desired"],
            size=diff["size"],
            data_type=diff["data_type"])
        if ok:
            ok_count += 1
            print(f"  [OK] Wrote Param {diff['id']:03d} {diff['name']} = {diff['desired']}")
        else:
            fail_count += 1
            print(f"  [FAIL] Param {diff['id']:03d} {diff['name']} write failed")

    client.unregister_session()
    client.disconnect()
    print(f"\nRestore complete: OK={ok_count}, FAIL={fail_count}")


# --- Main ---

def main():
    if len(sys.argv) > 1 and sys.argv[1] == "__hold-worker":
        worker_parser = argparse.ArgumentParser(add_help=False)
        worker_parser.add_argument("--token", required=True)
        worker_parser.add_argument("--rpi", type=int, default=5000)
        worker_parser.add_argument("--x-ip", default=DRIVE_IP_X)
        worker_parser.add_argument("--z-ip", default=DRIVE_IP_Z)
        worker_args = worker_parser.parse_args(sys.argv[2:])
        run_background_hold_worker(
            token=getattr(worker_args, "token"),
            rpi_us=getattr(worker_args, "rpi", 5000),
            x_ip=getattr(worker_args, "x_ip", DRIVE_IP_X),
            z_ip=getattr(worker_args, "z_ip", DRIVE_IP_Z))
        return

    if len(sys.argv) > 1 and sys.argv[1] == "__hold-watchdog":
        wd_parser = argparse.ArgumentParser(add_help=False)
        wd_parser.add_argument("--token", required=True)
        wd_parser.add_argument("--rpi", type=int, default=5000)
        wd_args = wd_parser.parse_args(sys.argv[2:])
        run_hold_watchdog_worker(
            token=getattr(wd_args, "token"),
            rpi_us=getattr(wd_args, "rpi", 5000))
        return

    parser = argparse.ArgumentParser(
        description="EtherNet/IP Interactive Test Harness for Kinetix 5100",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  py tools/eip_test.py list-identity
  py tools/eip_test.py register-session
  py tools/eip_test.py forward-open --ot-size 46 --run-idle
  py tools/eip_test.py forward-open --ot-size 46 --run-idle --ot-instance 106
  py tools/eip_test.py unregister-session
  py tools/eip_test.py full-test --ot-size 46 --run-idle
  py tools/eip_test.py move 2 1000 --duration 2
  py tools/eip_test.py move 1 50000 --duration 2
  py tools/eip_test.py move 4 20.0 500 --duration 2
  py tools/eip_test.py --log motion_log.txt move --speed 1.0 --duration 5.0
  py tools/eip_test.py move --stop-strategy speed-zero --speed 1000 --decel 10000
  py tools/eip_test.py servo-on
  py tools/eip_test.py servo-off
  py tools/eip_test.py home
  py tools/eip_test.py home --homing-method 0 --speed-fast 120.0 --speed-slow 12.0 --timeout 60
  py tools/eip_test.py home --param-method 29
  py tools/eip_test.py clear-fault
  py tools/eip_test.py clear-fault --max-cycles 30
  py tools/eip_test.py decode A603
  py tools/eip_test.py decode flags
  py tools/eip_test.py dump-params --dry-parse
  py tools/eip_test.py dump-params --ids 24,29,117 --out k5100_params.json
  py tools/eip_test.py restore-params k5100_params.json --dry-run
        """)
    parser.add_argument("--log", type=str, default=None,
                        help="Save all output to specified file (in addition to stdout)")

    sub = parser.add_subparsers(dest="command", help="Command to run")

    # list-identity
    sub.add_parser("list-identity", help="List drive identity")

    # register-session
    sub.add_parser("register-session", help="Register a session and print handle")

    # unregister-session
    sub.add_parser("unregister-session", help="Unregister a session")

    # forward-open
    fo = sub.add_parser("forward-open", help="Send ForwardOpen with configurable params")

    # -- Assembly instances
    fo.add_argument("--config-instance", type=int, default=191,
                    help="Config assembly instance (default: 191 / 0xBF)")
    fo.add_argument("--ot-instance", type=int, default=104,
                    help="O->T assembly instance (default: 104)")
    fo.add_argument("--to-instance", type=int, default=154,
                    help="T->O assembly instance (default: 154)")

    # -- Sizes
    fo.add_argument("--ot-size", type=int, default=40,
                    help="O->T assembly data size in bytes (default: 40)")
    fo.add_argument("--to-size", type=int, default=52,
                    help="T->O assembly data size in bytes (default: 52)")

    # -- RPI
    fo.add_argument("--ot-rpi", type=int, default=5000,
                    help="O->T RPI in microseconds (default: 5000)")
    fo.add_argument("--to-rpi", type=int, default=5000,
                    help="T->O RPI in microseconds (default: 5000)")

    # -- Flags
    fo.add_argument("--run-idle", action="store_true", default=False,
                    help="Include 4-byte Run/Idle header in O->T (adds 4 bytes)")
    fo.add_argument("--no-run-idle", action="store_true", default=False,
                    help="Explicitly disable Run/Idle header")
    fo.add_argument("--run-idle-bit", action="store_true", default=None,
                    help="Set Run/Idle bit in net params without adding 4 bytes to conn size")
    fo.add_argument("--raw-ot-conn-size", type=int, default=None,
                    help="Override O->T connection size directly (bypass assembly+2+4 calc)")
    fo.add_argument("--raw-to-conn-size", type=int, default=None,
                    help="Override T->O connection size directly (bypass assembly+2 calc)")
    fo.add_argument("--mcast", action="store_true", default=False,
                    help="Use Multicast for T->O (default: Point-to-Point)")
    fo.add_argument("--p2p", action="store_true", default=False,
                    help="Use Point-to-Point for T->O (default)")

    # -- IDs
    fo.add_argument("--vendor-id", type=int, default=1,
                    help="Originator vendor ID (default: 1)")
    fo.add_argument("--serial", type=lambda x: int(x, 0), default=0xCAFEB00D,
                    help="Originator serial (default: 0xCAFEB00D)")

    # io-test
    io = sub.add_parser("io-test", help="Full test with cyclic I/O exchange validation")
    io.add_argument("--ot-size", type=int, default=40)
    io.add_argument("--to-size", type=int, default=52)
    io.add_argument("--ot-rpi", type=int, default=5000)
    io.add_argument("--to-rpi", type=int, default=5000)
    io.add_argument("--vendor-id", type=int, default=1)
    io.add_argument("--serial", type=lambda x: int(x, 0), default=0xCAFEB00D)
    io.add_argument("--count", type=int, default=3, help="Number of I/O exchanges")

    # move
    mv = sub.add_parser(
        "move",
        help="Send motion commands to slowly move the motor",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
OperatingMode map and arguments:
  1 Position:
    Positional:  move 1 <position_puu>
    Flags:       --operating-mode 1 --position-puu <int>
    Uses:        position_puu, accel, decel, duration

  2 Speed:
    Positional:  move 2 <speed_rpm>
    Flags:       --operating-mode 2 --speed <float>
    Uses:        speed_rpm, accel, decel, duration, stop strategy

  3 Home:
    Positional:  move 3 <speed_rpm> [homing_method] [home_return_speed_rpm]
    Flags:       --operating-mode 3 --speed <float> --homing-method <0..38> --home-return-speed <float>
    Uses:        speed_rpm, homing_method, home_return_speed, accel, decel, duration

  4 Torque:
    Positional:  move 4 <torque_percent> [torque_ramp_ms]
    Flags:       --operating-mode 4 --torque-percent <float> --torque-ramp-ms <int>
    Uses:        torque_percent, torque_ramp_ms, accel, decel, duration

  5 Gear:
    Positional:  move 5 <speed_rpm>
    Flags:       --operating-mode 5 --speed <float>
    Uses:        speed_rpm, accel, decel, duration

  6 Index:
    Positional:  move 6 <starting_index>
    Flags:       --operating-mode 6 --starting-index <0..255>
    Uses:        starting_index, accel, decel, duration

  7 ECAM:
    Positional:  move 7 <starting_index>
    Flags:       --operating-mode 7 --starting-index <0..255>
    Uses:        starting_index, accel, decel, duration

Examples:
  py tools/eip_test.py move
  py tools/eip_test.py move 2 1000 --duration 2
  py tools/eip_test.py move 1 50000 --duration 2
  py tools/eip_test.py move 4 20.0 500 --duration 2
  py tools/eip_test.py move 3 60 34 6 --duration 2
  py tools/eip_test.py move --operating-mode 6 --starting-index 1 --duration 2
  py tools/eip_test.py move --operating-mode 2 --speed 1000 --stop-strategy speed-zero --decel-tolerance 2
        """)
    mv.add_argument("operating_mode_pos", nargs="?", type=int, default=None,
                    help="Positional OperatingMode (1..7). If omitted, --operating-mode or "
                         "default 2 is used.")
    mv.add_argument("mode_param_1", nargs="?", default=None,
                    help="Mode positional p1: mode1=position_puu, mode2/5=speed_rpm, "
                         "mode3=speed_rpm, mode4=torque_percent, mode6/7=starting_index.")
    mv.add_argument("mode_param_2", nargs="?", default=None,
                    help="Mode positional p2: mode3=homing_method, mode4=torque_ramp_ms.")
    mv.add_argument("mode_param_3", nargs="?", default=None,
                    help="Mode positional p3: mode3=home_return_speed_rpm.")
    mv.add_argument("--speed", type=float, default=1.0, help="Speed in RPM (default: 1.0)")
    mv.add_argument("--speed-end", type=float, default=None,
                    help="If set, linearly ramp commanded speed from --speed to this "
                         "value over --duration (Speed mode).")
    mv.add_argument("--accel", type=float, default=DEFAULT_ACCEL_RPM_PER_S,
                    help=f"Acceleration in RPM/s (default: {DEFAULT_ACCEL_RPM_PER_S})")
    mv.add_argument("--decel", type=float, default=DEFAULT_DECEL_RPM_PER_S,
                    help=f"Deceleration in RPM/s (default: {DEFAULT_DECEL_RPM_PER_S})")
    mv.add_argument("--duration", type=float, default=3.0, help="Duration in seconds (default: 3.0)")
    mv.add_argument("--rpi", type=int, default=5000, help="RPI in microseconds (default: 5000)")
    mv.add_argument("--operating-mode", type=int, default=None,
                    help="OperatingMode value sent while moving (1..7). "
                         "Positional operating_mode overrides this flag. "
                         "1=Position 2=Speed 3=Home 4=Torque 5=Gear 6=Index 7=ECAM.")
    mv.add_argument("--position-puu", type=int, default=0,
                    help="Position reference in PUU for Position mode (mode 1).")
    mv.add_argument("--torque-percent", type=float, default=0.0,
                    help="Torque reference in percent rated for Torque mode (mode 4).")
    mv.add_argument("--torque-ramp-ms", type=int, default=0,
                    help="Torque ramp time in ms for Torque mode (mode 4).")
    mv.add_argument("--starting-index", type=int, default=0,
                    help="Starting index for Index/ECAM modes (modes 6/7), byte 36.")
    mv.add_argument("--non-cyclic-move-type", type=int, default=0,
                    help="Output assembly byte 24 (Non Cyclic Move Type), default 0.")
    mv.add_argument("--cyclic-move-type", type=int, default=0,
                    help="Output assembly byte 25 (Cyclic Move Type), default 0.")
    mv.add_argument("--travel-mode", type=int, default=10,
                    help="Output assembly byte 26 (Travel Mode), default 10 (cyclic).")
    mv.add_argument("--position-command-override", action="store_true", default=False,
                    help="Set output assembly byte 27 bit 0.")
    mv.add_argument("--position-command-overlap", action="store_true", default=False,
                    help="Set output assembly byte 27 bit 1.")
    mv.add_argument("--captured-position-select", action="store_true", default=False,
                    help="Set output assembly byte 27 bit 2.")
    mv.add_argument("--homing-method", type=int, default=34,
                    help="Homing method for Home mode (mode 3), valid range 0..38.")
    mv.add_argument("--home-return-speed", type=float, default=6.0,
                    help="Home return speed in RPM for Home mode (mode 3).")
    mv.add_argument("--stop-strategy", choices=("stop-bit", "speed-zero", "hybrid"),
                    default="stop-bit",
                    help="Stop behavior: stop-bit pulses stop_motion then holds speed zero; "
                         "speed-zero only commands zero speed; hybrid starts speed-zero and "
                         "escalates to stop bit after 2x estimated stop time (default: stop-bit)")
    mv.add_argument("--decel-tolerance", type=float, default=2.0,
                    help="Allowed measured-vs-commanded accel/decel error percent (default: 2)")

    # home
    hm = sub.add_parser("home", help="Execute homing sequence on the motor")
    hm.add_argument("--homing-method", type=int, default=34,
                    help="Homing method 0-38, EDS Param#29 range (default: 34 = "
                         "define current position as origin; no switches needed). "
                         "0-33 require a limit/ORG switch wired; 35-38 are torque-based "
                         "collision homing and require DI.Enable Homing.")
    hm.add_argument("--operating-mode", type=int, default=3,
                    help="OperatingMode value sent in byte 0 (default: 3 = Home mode). "
                         "1=Position 2=Speed 3=Home 4=Torque 5=Gear 6=Index 7=ECAM.")
    hm.add_argument("--speed-fast", type=float, default=60.0,
                    help="Fast homing speed in RPM (default: 60.0)")
    hm.add_argument("--speed-slow", type=float, default=6.0,
                    help="Slow/creep homing speed in RPM (default: 6.0)")
    hm.add_argument("--accel", type=float, default=DEFAULT_HOME_ACCEL_RPM_PER_S,
                    help=f"Homing acceleration in RPM/s (default: {DEFAULT_HOME_ACCEL_RPM_PER_S})")
    hm.add_argument("--decel", type=float, default=DEFAULT_HOME_DECEL_RPM_PER_S,
                    help=f"Homing deceleration in RPM/s (default: {DEFAULT_HOME_DECEL_RPM_PER_S})")
    hm.add_argument("--rpi", type=int, default=5000,
                    help="RPI in microseconds (default: 5000)")
    hm.add_argument("--timeout", type=float, default=30.0,
                    help="Timeout for homing completion in seconds (default: 30.0)")
    hm.add_argument("--param-method", type=int, default=None,
                    help="Optional: pre-write homing method via CIP Parameter Object "
                         "instance 29 (HomingMethod, verified in EDS) instead of only "
                         "the cyclic assembly byte. Pass 29 to use it.")

    # clear-fault
    cf = sub.add_parser("clear-fault", help="Clear a drive fault via cyclic I/O without power-cycling")
    cf.add_argument("--rpi", type=int, default=5000, help="RPI in microseconds (default: 5000)")
    cf.add_argument("--max-cycles", type=int, default=20,
                    help="Max fault-reset frames to send (default: 20)")

    # servo-on / servo-off
    so = sub.add_parser("servo-on", help="Enable servo and start persistent background hold")
    so.add_argument("--rpi", type=int, default=5000, help="RPI in microseconds (default: 5000)")
    so.add_argument("--attempts", type=int, default=8, help="Max cyclic status attempts (default: 8)")
    so.add_argument("--hold-seconds", type=float, default=0.0,
                    help="Foreground test hold duration in seconds. 0 = background hold mode (default: 0)")
    so.add_argument("--no-watchdog", action="store_true", default=False,
                    help="Do not start the hold watchdog supervisor")
    sf = sub.add_parser("servo-off", help="Disable servo only (no motion command)")
    sf.add_argument("--rpi", type=int, default=5000, help="RPI in microseconds (default: 5000)")
    sf.add_argument("--attempts", type=int, default=8, help="Max cyclic status attempts (default: 8)")

    # hold-setup — Defender exclusion + watchdog sanity
    sub.add_parser(
        "hold-setup",
        help="Apply Windows Defender exclusions for hold/jog paths (Admin recommended)")

    # check-mode
    cm = sub.add_parser("check-mode", help="Check if drive is powered on and in I/O mode")
    # get-param
    gp = sub.add_parser("get-param", help="Read a Kinetix 5100 parameter via CIP")
    gp.add_argument("param_id", type=lambda x: int(x, 0), help="Parameter ID (e.g., 117 or 0x75 for P1.001)")

    # decode
    dec = sub.add_parser("decode", help="Decode a Kinetix display/fault/alarm code offline")
    dec.add_argument("code", help="Display code (A603/E602/603), raw word (0x0603), or flags")

    # dump-params
    dp = sub.add_parser("dump-params", help="Dump Kinetix parameters to a JSON snapshot")
    dp.add_argument("--eds", type=str, default=str(DEFAULT_EDS_PATH),
                    help=f"EDS path (default: {DEFAULT_EDS_PATH})")
    dp.add_argument("--ids", type=str, default=None,
                    help="Comma-separated parameter IDs to dump (e.g. 24,29,117)")
    dp.add_argument("--group", type=str, default=None,
                    help="EDS group name to dump (e.g. Motor)")
    dp.add_argument("--out", type=str, default=None,
                    help="Output JSON path (default: k5100_params_<timestamp>.json)")
    dp.add_argument("--dry-parse", action="store_true", default=False,
                    help="Only parse the EDS and print table/group counts; no drive connection")

    # restore-params
    rp = sub.add_parser("restore-params", help="Diff and optionally restore a JSON parameter snapshot")
    rp.add_argument("snapshot", help="Snapshot JSON created by dump-params")
    rp.add_argument("--eds", type=str, default=str(DEFAULT_EDS_PATH),
                    help=f"EDS path (default: {DEFAULT_EDS_PATH})")
    rp.add_argument("--ids", type=str, default=None,
                    help="Comma-separated parameter IDs to restore from the snapshot")
    rp.add_argument("--group", type=str, default=None,
                    help="EDS group name to restore from the snapshot")
    rp.add_argument("--dry-run", action="store_true", default=True,
                    help="Print diffs only (default)")
    rp.add_argument("--apply", action="store_true", default=False,
                    help="Write differing parameters after confirmation")

    # full-test
    ft = sub.add_parser("full-test", help="Run connect + register + forward-open + unregister")
    ft.add_argument("--config-instance", type=int, default=191)
    ft.add_argument("--ot-instance", type=int, default=104)
    ft.add_argument("--to-instance", type=int, default=154)
    ft.add_argument("--ot-size", type=int, default=40)
    ft.add_argument("--to-size", type=int, default=52)
    ft.add_argument("--ot-rpi", type=int, default=5000)
    ft.add_argument("--to-rpi", type=int, default=5000)
    ft.add_argument("--run-idle", action="store_true", default=False)
    ft.add_argument("--no-run-idle", action="store_true", default=False)
    ft.add_argument("--run-idle-bit", action="store_true", default=None)
    ft.add_argument("--raw-ot-conn-size", type=int, default=None)
    ft.add_argument("--raw-to-conn-size", type=int, default=None)
    ft.add_argument("--mcast", action="store_true", default=False)
    ft.add_argument("--p2p", action="store_true", default=False)
    ft.add_argument("--vendor-id", type=int, default=1)
    ft.add_argument("--serial", type=lambda x: int(x, 0), default=0xCAFEB00D)

    args = parser.parse_args()

    # --- Logging setup ---
    if args.log:
        import datetime
        log_file = open(args.log, 'w', encoding='utf-8')
        log_file.write(f"# EIP Test Log — {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        log_file.write(f"# Command: {' '.join(sys.argv)}\n\n")
        log_file.flush()

        class Tee:
            def __init__(self, *files):
                self.files = files
            def write(self, data):
                for f in self.files:
                    f.write(data)
                    f.flush()
            def flush(self):
                for f in self.files:
                    f.flush()
            def close(self):
                for f in self.files:
                    if f is not sys.__stdout__ and f is not sys.__stderr__:
                        f.close()

        tee = Tee(sys.__stdout__, log_file)
        sys.stdout = tee

    if args.command is None:
        parser.print_help()
        return

    client = EipClient()

    if args.command == "decode":
        run_decode_command(args.code)

    elif args.command == "dump-params":
        run_dump_params_command(client, args)

    elif args.command == "restore-params":
        run_restore_params_command(client, args)

    elif args.command == "list-identity":
        run_list_identity(client)

    elif args.command == "register-session":
        run_register_session(client)

    elif args.command == "unregister-session":
        print("\n=== UnRegisterSession ===")
        # We need an existing connection with a session; interactive use
        print("This command is designed for use within an interactive session.")
        print("Use full-test for complete test cycles.")

    elif args.command == "hold-setup":
        print("\n=== Hold Reliability Setup ===")
        print(f"  Paths under: {SCRIPT_DIR}")
        ensure_hold_defender_exclusion()
        print("  Tip: re-run from an elevated prompt if Add-MpPreference was denied.")
        print("  Watchdog starts automatically with servo-on.")

    elif args.command == "servo-on":
        hold_seconds = float(getattr(args, 'hold_seconds', 0.0))
        rpi_us = int(getattr(args, 'rpi', 5000))
        if hold_seconds > 0:
            run_servo_state_command(
                client,
                enable=True,
                rpi_us=rpi_us,
                attempts=getattr(args, 'attempts', 8),
                hold_seconds=hold_seconds)
        else:
            print("\n=== Servo Enable (Persistent Hold) ===")
            print("  This starts a background hold session and returns immediately.")
            wait_for_user_confirmation("Enable servo hold service acknowledged.")
            ensure_hold_defender_exclusion()
            if start_background_hold(
                    rpi_us=rpi_us,
                    start_watchdog=not bool(getattr(args, "no_watchdog", False))):
                print("[OK] Persistent servo hold is active. Ready for home/move commands.")
            else:
                print("[FAIL] Could not start persistent servo hold.")

    elif args.command == "servo-off":
        _set_hold_supervise(False, reason="servo-off")
        try:
            HOLD_PAUSE_MARKER.write_text(
                f"servo-off\n{_now_iso()}\n", encoding="utf-8")
        except Exception:
            pass
        if get_hold_watchdog_state():
            print("  Stopping hold watchdog...")
            stop_hold_watchdog("servo-off")
        if get_background_hold_state():
            print("  Stopping persistent background hold...")
            if not stop_background_hold("servo-off command", timeout_s=8.0):
                print("  [WARN] Background hold did not stop cleanly in time.")
        _clear_hold_pause_marker()
        run_servo_state_command(
            client,
            enable=False,
            rpi_us=getattr(args, 'rpi', 5000),
            attempts=getattr(args, 'attempts', 8))

    elif args.command == "io-test":
        # io-test: full cycle + I/O exchange
        print("\n=== I/O Test ===")
        params = ForwardOpenParams()
        params.ot_assembly_size = args.ot_size
        params.to_assembly_size = args.to_size
        params.ot_rpi_us = args.ot_rpi
        params.to_rpi_us = args.to_rpi
        params.originator_vendor_id = args.vendor_id
        params.originator_serial = args.serial
        # Key: O->T connection size = 46 (40+2+4), NO Run/Idle bit
        params.raw_ot_conn_size = args.ot_size + 2 + 4  # assembly + seq + RunIdle
        params.include_run_idle_header = False  # Do NOT set Run/Idle bit

        if not client.connect():
            return
        if not client.register_session():
            client.disconnect()
            return

        result = client.forward_open(params)
        if result is None:
            client.unregister_session()
            client.disconnect()
            return

        ot_conn_id = result.get("ot_connection_id", 0)
        if ot_conn_id == 0:
            print("[FAIL] ForwardOpen returned zero O->T connection ID")
            client.unregister_session()
            client.disconnect()
            return

        print(f"\n  Starting cyclic I/O with O->T conn_id=0x{ot_conn_id:08X}")

        # Build idle output assembly (40 bytes of zeros with servo_on=0)
        idle_assembly = bytes(40)

        exchange_count = getattr(args, 'count', 3)
        for i in range(exchange_count):
            print(f"\n  --- Exchange {i+1}/{exchange_count} ---")
            to_assembly = client.exchange_io_frame(
                ot_conn_id, idle_assembly, include_run_idle=True, timeout=5.0)
            if to_assembly is None:
                print(f"  [FAIL] Exchange {i+1} failed")
                break
            if len(to_assembly) >= 52:
                # Parse Kinetix 5100 input assembly 154
                status_byte0 = to_assembly[0]
                status_byte8 = to_assembly[8] if len(to_assembly) > 8 else 0
                status_byte9 = to_assembly[9] if len(to_assembly) > 9 else 0
                op_mode = to_assembly[11] if len(to_assembly) > 11 else -1
                actual_speed = struct.unpack_from("<i", to_assembly, 16)[0] if len(to_assembly) >= 20 else 0
                fault_code = struct.unpack_from("<H", to_assembly, 20)[0] if len(to_assembly) >= 22 else 0
                actual_position = struct.unpack_from("<i", to_assembly, 24)[0] if len(to_assembly) >= 28 else 0

                parsed_status = parse_input_assembly_154(to_assembly)
                run_mode = parsed_status.get("run_mode", False)
                fault = parsed_status.get("fault", False)
                ready = parsed_status.get("ready", False)
                active = parsed_status.get("active", False)

                print(f"  Status: run_mode={run_mode} fault={fault} ready={ready} active={active} "
                      f"{format_status_fault_summary(parsed_status)}")
                fault_detail = format_drive_code_detail(fault_code, prefer_prefix="E") if fault_code else "clear"
                print(f"  OpMode={op_mode} Speed={actual_speed/10:.1f} RPM "
                      f"Pos={actual_position} PUU FaultCode=0x{fault_code:04X} ({fault_detail})")

            if i < exchange_count - 1:
                import time as _time
                _time.sleep(0.1)

        client.forward_close(result.get("params", params))
        client.unregister_session()
        client.disconnect()

    elif args.command == "move":
        # Move: mode-aware cyclic command test (supports OperatingMode 1..7)
        speed_rpm = getattr(args, 'speed', 1.0)
        speed_end_rpm = getattr(args, 'speed_end', None)
        accel_rpm_per_s = getattr(args, 'accel', DEFAULT_ACCEL_RPM_PER_S)
        decel_rpm_per_s = getattr(args, 'decel', DEFAULT_DECEL_RPM_PER_S)
        duration_s = getattr(args, 'duration', 3.0)
        rpi_us = getattr(args, 'rpi', 5000)
        hold_state_before_move = get_background_hold_state()
        resume_hold_after_move = hold_state_before_move is not None
        resume_hold_rpi_us = int(hold_state_before_move.get("rpi_us", rpi_us)) if hold_state_before_move else int(rpi_us)
        hold_paused_for_move = False
        if hold_state_before_move:
            print("  [INFO] Persistent servo hold detected; move will take temporary ownership.")
        operating_mode = (getattr(args, 'operating_mode_pos', None)
                          if getattr(args, 'operating_mode_pos', None) is not None
                          else getattr(args, 'operating_mode', None))
        if operating_mode is None:
            operating_mode = 2
        position_puu = getattr(args, 'position_puu', 0)
        torque_percent = getattr(args, 'torque_percent', 0.0)
        torque_ramp_ms = getattr(args, 'torque_ramp_ms', 0)
        starting_index = getattr(args, 'starting_index', 0)
        non_cyclic_move_type = getattr(args, 'non_cyclic_move_type', 0)
        cyclic_move_type = getattr(args, 'cyclic_move_type', 0)
        travel_mode = getattr(args, 'travel_mode', 10)
        position_command_override = getattr(args, 'position_command_override', False)
        position_command_overlap = getattr(args, 'position_command_overlap', False)
        captured_position_select = getattr(args, 'captured_position_select', False)
        homing_method = getattr(args, 'homing_method', 34)
        home_return_speed_rpm = getattr(args, 'home_return_speed', 6.0)
        stop_strategy = getattr(args, 'stop_strategy', "stop-bit")
        decel_tolerance = getattr(args, 'decel_tolerance', 2.0)

        positional_values, positional_error = parse_move_mode_positionals(
            operating_mode,
            getattr(args, 'mode_param_1', None),
            getattr(args, 'mode_param_2', None),
            getattr(args, 'mode_param_3', None),
            force_required=(getattr(args, 'operating_mode_pos', None) is not None))
        if positional_error is not None:
            print(f"\n[ABORT] Invalid positional move arguments: {positional_error}")
            return
        if positional_values:
            speed_rpm = positional_values.get("speed_rpm", speed_rpm)
            position_puu = positional_values.get("position_puu", position_puu)
            torque_percent = positional_values.get("torque_percent", torque_percent)
            torque_ramp_ms = positional_values.get("torque_ramp_ms", torque_ramp_ms)
            starting_index = positional_values.get("starting_index", starting_index)
            homing_method = positional_values.get("homing_method", homing_method)
            home_return_speed_rpm = positional_values.get("home_return_speed_rpm", home_return_speed_rpm)

        if operating_mode == 3:
            # Bench: OperatingMode echoes 3 with travel_mode=2. travel_mode=10
            # often never leaves a prior Position sub-mode. Values 0/1 → A603.
            if travel_mode != 2:
                print(f"  [INFO] Home mode forcing travel_mode=2 (was {travel_mode}).")
                travel_mode = 2

        if operating_mode not in OPERATING_MODE_NAMES:
            print(f"\n[ABORT] Invalid operating mode: {operating_mode} (valid: 0..7)")
            return
        if not (0 <= starting_index <= 255):
            print(f"\n[ABORT] Invalid starting index: {starting_index} (valid: 0..255)")
            return
        if not (0 <= homing_method <= 38):
            print(f"\n[ABORT] Invalid homing method: {homing_method} (valid: 0..38)")
            return
        if torque_ramp_ms < 0:
            print(f"\n[ABORT] Invalid torque ramp ms: {torque_ramp_ms} (must be >= 0)")
            return
        for field_name, field_value in (
            ("non_cyclic_move_type", non_cyclic_move_type),
            ("cyclic_move_type", cyclic_move_type),
            ("travel_mode", travel_mode),
        ):
            if not (0 <= int(field_value) <= 255):
                print(f"\n[ABORT] Invalid {field_name}: {field_value} (valid: 0..255)")
                return

        range_error = validate_accel_decel_ranges(accel_rpm_per_s, decel_rpm_per_s)
        if range_error is not None:
            print(f"\n[ABORT] Invalid motion command input: {range_error}")
            return

        mode_name = OPERATING_MODE_NAMES.get(operating_mode, f"Unknown({operating_mode})")
        mode_refs = []
        if operating_mode in MODE_USES_SPEED_REFERENCE:
            if speed_end_rpm is not None:
                mode_refs.append(f"speed={speed_rpm}→{speed_end_rpm}RPM (linear ramp)")
            else:
                mode_refs.append(f"speed={speed_rpm}RPM")
        if operating_mode in MODE_USES_POSITION_REFERENCE:
            mode_refs.append(f"position={position_puu}PUU")
        if operating_mode in MODE_USES_TORQUE_REFERENCE:
            mode_refs.append(f"torque={torque_percent}%")
            mode_refs.append(f"torque_ramp={torque_ramp_ms}ms")
        if operating_mode in MODE_USES_INDEX_REFERENCE:
            mode_refs.append(f"starting_index={starting_index}")
        if operating_mode == 3:
            mode_refs.append(f"homing_method={homing_method}")
            mode_refs.append(f"home_return_speed={home_return_speed_rpm}RPM")
        refs_text = ", ".join(mode_refs) if mode_refs else "no specific mode references"
        if operating_mode not in MODE_USES_SPEED_REFERENCE and abs(speed_rpm) > 1e-9:
            print(f"  [INFO] speed={speed_rpm}RPM is ignored for mode {operating_mode} ({mode_name})")
        if operating_mode in MODE_USES_POSITION_REFERENCE and position_puu == 0:
            print("  [WARN] Position mode selected with position reference = 0 PUU")
        if operating_mode in MODE_USES_TORQUE_REFERENCE and abs(torque_percent) < 1e-9:
            print("  [WARN] Torque mode selected with torque reference = 0.0%")
        if operating_mode in MODE_USES_INDEX_REFERENCE and starting_index == 0:
            print("  [WARN] Index/ECAM mode selected with starting_index = 0")

        print(f"\n=== Motion Test: mode={operating_mode} ({mode_name}) for {duration_s}s ===")
        print(f"  Command references: {refs_text}")
        print(f"  Accel: {accel_rpm_per_s} RPM/s, Decel: {decel_rpm_per_s} RPM/s")
        print(f"  Stop strategy: {stop_strategy}, tolerance: {decel_tolerance:.1f}%")
        print(f"  Byte24/25/26: non_cyclic={non_cyclic_move_type}, cyclic={cyclic_move_type}, travel={travel_mode}")
        print("  SAFETY: Ensure the motor is clear and E-Stop is accessible!")
        wait_for_user_confirmation("Pre-flight acknowledged.")

        # Pre-flight: check drive is powered on and in I/O mode
        print("\n  --- Pre-flight Check ---")
        if not client.connect():
            return
        if not client.register_session():
            client.disconnect()
            return
        status = client.check_drive_ready()
        client.unregister_session()
        client.disconnect()

        if not status["powered_on"]:
            print("\n[ABORT] Drive not responding. Check power and Ethernet connection.")
            return
        if not status["io_mode"]:
            print(f"\n[ABORT] Drive is NOT in I/O mode (P1.001=0x{status['p1_001_value']:04X} = {status['p1_001_name']}).")
            print("  Use KNX5100C or the drive keypad to set P1.001 = 0x0C (IO mode),")
            print("  then power-cycle the drive and retry.")
            return
        if not status.get("servo_enabled", status.get("servo_active", False)):
            print("\n[ABORT] Servo is not enabled.")
            print("  Run: py tools/eip_test.py servo-on")
            print("  Then retry the motion command.")
            return

        print("  [OK] Drive is in I/O mode and servo is already enabled.")
        print("  Proceeding with motion test without toggling servo state.\n")

        # Pause hold BEFORE opening a new TCP session. Competing TCP while the
        # hold worker still owns Class 1 can abort its socket (WinError 10053)
        # and leave the drive in ownership conflict 0x0106.
        if resume_hold_after_move and not hold_paused_for_move:
            if not pause_background_hold_for_handoff("handoff:move", timeout_s=8.0):
                print("  [FAIL] Could not pause persistent servo hold.")
                return
            hold_paused_for_move = True

        # Reconnect for ForwardOpen and cyclic I/O
        if not client.connect():
            if hold_paused_for_move:
                print("  Restoring persistent servo hold after connect failure...")
                start_background_hold(rpi_us=resume_hold_rpi_us)
            return
        if not client.register_session():
            client.disconnect()
            if hold_paused_for_move:
                print("  Restoring persistent servo hold after session failure...")
                start_background_hold(rpi_us=resume_hold_rpi_us)
            return

        params = ForwardOpenParams()
        params.ot_assembly_size = 40
        params.to_assembly_size = 52
        params.ot_rpi_us = rpi_us
        params.to_rpi_us = rpi_us
        params.raw_ot_conn_size = 40 + 2 + 4  # 46
        params.include_run_idle_header = True
        params.connection_timeout_multiplier = 7

        result = forward_open_with_retry(client, params)
        if result is None:
            client.unregister_session()
            client.disconnect()
            if hold_paused_for_move:
                print("  Restoring persistent servo hold after handoff failure...")
                time.sleep(HOLD_OWNERSHIP_SETTLE_S)
                start_background_hold(rpi_us=resume_hold_rpi_us)
            return

        ot_conn_id = result.get("ot_connection_id", 0)
        if ot_conn_id == 0:
            print("[FAIL] ForwardOpen returned zero O->T connection ID")
            client.unregister_session()
            client.disconnect()
            if hold_paused_for_move:
                print("  Restoring persistent servo hold after connection ID failure...")
                time.sleep(HOLD_OWNERSHIP_SETTLE_S)
                start_background_hold(rpi_us=resume_hold_rpi_us)
            return

        print(f"  O->T conn_id=0x{ot_conn_id:08X}, RPI={rpi_us}us")

        import time as _time

        client.verbose = False

        # Servo state is intentionally not modified in move command.
        print("  [OK] Servo state is managed independently (no auto enable/disable).")

        def io_x(asm: bytes, timeout: float = 2.0):
            return client.exchange_io_frame(
                ot_conn_id, asm, include_run_idle=True, timeout=timeout,
                reuse_socket=True, drain=True)

        # After hold handoff, ServoOn must see a 0→1 edge on the new Class 1
        # connection before StartMotion. Commanding motion while Active is low
        # is a reliable A603 trigger on this drive.
        print("  Settling servo (mode 0) before motion command...")
        seed_asm = build_output_assembly_104(
            servo_on=False, servo_off=False, operating_mode=0, travel_mode=10,
            torque_ramp_time_ms=1000)
        settle_asm = build_output_assembly_104(
            servo_on=True, servo_off=False, speed_rpm=0.0, operating_mode=0,
            travel_mode=10, start_motion=False, stop_motion=False,
            torque_ramp_time_ms=1000)
        stop_asm = build_output_assembly_104(
            servo_on=True, stop_motion=True, operating_mode=0, travel_mode=10,
            torque_ramp_time_ms=1000)
        clear_low = build_output_assembly_104(
            servo_on=True, operating_mode=0, travel_mode=10, torque_ramp_time_ms=1000)
        clear_high = build_output_assembly_104(
            servo_on=True, fault_reset=True, operating_mode=0, travel_mode=10,
            torque_ramp_time_ms=1000)

        last_status = None
        move_trip_reason = None
        for _ in range(3):
            to_data = io_x(seed_asm)
            if to_data is not None:
                st = parse_input_assembly_154(to_data)
                if "error" not in st:
                    last_status = st
            _time.sleep(rpi_us / 1000000.0)

        settle_deadline = _time.time() + 3.0
        settled_active = False
        a603_clear_attempts = 0
        stop_pulsed = False
        while _time.time() < settle_deadline:
            to_data = io_x(settle_asm)
            if to_data is None:
                _time.sleep(0.05)
                continue
            st = parse_input_assembly_154(to_data)
            if "error" in st:
                continue
            last_status = st
            if bool(st.get("fault", False)):
                move_trip_reason = get_drive_trip_reason(st)
                print("  [FAIL] Drive fault during pre-move settle.")
                print(f"  Reason: {move_trip_reason}")
                break
            if (bool(st.get("warning_present", False))
                    and (int(st.get("warning_code", 0)) & 0x0FFF) == 0x0603
                    and a603_clear_attempts < 2):
                a603_clear_attempts += 1
                print("  [INFO] Clearing latched A603 before motion...")
                for _ in range(3):
                    io_x(clear_high)
                    _time.sleep(0.05)
                for _ in range(4):
                    io_x(clear_low)
                    _time.sleep(0.05)
                continue
            if (bool(st.get("active", False)) and bool(st.get("ready", False))
                    and not bool(st.get("warning_present", False))):
                if bool(st.get("command_in_progress", False)) and not stop_pulsed:
                    stop_pulsed = True
                    print("  [INFO] Canceling leftover CommandInProgress before motion...")
                    for _ in range(6):
                        io_x(stop_asm)
                        _time.sleep(rpi_us / 1000000.0)
                    continue
                settled_active = True
                print("  [OK] Servo Active/Ready after settle.")
                break
            _time.sleep(rpi_us / 1000000.0)

        if move_trip_reason is not None:
            client.forward_close(result.get("params", params))
            client.unregister_session()
            client.disconnect()
            if hold_paused_for_move:
                print("\n  Restoring persistent servo hold...")
                try:
                    if not start_background_hold(rpi_us=resume_hold_rpi_us):
                        print("  [WARN] Could not restore persistent servo hold. Run servo-on.")
                except OSError as exc:
                    print(f"  [WARN] Could not restore persistent servo hold ({exc}). Run servo-on.")
            return

        if not settled_active:
            print("  [WARN] Clean Active/Ready not seen during settle; continuing anyway.")
            if last_status:
                print(f"  Settle status: mode={last_status.get('operating_mode')} "
                      f"active={last_status.get('active')} ready={last_status.get('ready')} "
                      f"{format_status_fault_summary(last_status)}")

        # Clamp torque ramp for modes that do not pass an explicit value.
        if int(torque_ramp_ms) < 1:
            torque_ramp_ms = 1000

        speed_start_rpm = float(speed_rpm) if operating_mode in MODE_USES_SPEED_REFERENCE else 0.0
        speed_command_rpm = speed_start_rpm
        ramp_enabled = (
            operating_mode in MODE_USES_SPEED_REFERENCE
            and speed_end_rpm is not None
            and duration_s > 0.0)
        speed_end_cmd = float(speed_end_rpm) if ramp_enabled else speed_start_rpm
        # When --speed-end is set, use the drive accel profile to ramp from
        # start→end over --duration. Per-frame SpeedReference updates are
        # ignored after the StartMotion edge latches the initial command.
        ramp_accel_rpm_per_s = accel_rpm_per_s
        ramp_start_dwell_s = 0.4
        if ramp_enabled:
            ramp_window_s = max(0.5, duration_s - ramp_start_dwell_s)
            ramp_accel_rpm_per_s = abs(speed_end_cmd - speed_start_rpm) / ramp_window_s
            # Keep at least the EDS minimum (~45.8 RPM/s).
            if ramp_accel_rpm_per_s < 50.0:
                ramp_accel_rpm_per_s = 50.0
            print(f"  Speed ramp: {speed_start_rpm:.1f} → {speed_end_cmd:.1f} RPM over "
                  f"~{ramp_window_s:.1f}s via drive accel={ramp_accel_rpm_per_s:.1f} RPM/s")

        def make_mode_asm(start_motion: bool,
                          stop_motion: bool,
                          speed_value: float,
                          torque_value: Optional[float] = None,
                          accel_value: Optional[float] = None):
            return build_output_assembly_104(
                servo_on=True,
                speed_rpm=speed_value,
                accel_rpm_per_s=accel_rpm_per_s if accel_value is None else accel_value,
                decel_rpm_per_s=decel_rpm_per_s,
                operating_mode=operating_mode,
                position_puu=position_puu,
                torque_percent=torque_percent if torque_value is None else torque_value,
                torque_ramp_time_ms=torque_ramp_ms,
                starting_index=starting_index,
                non_cyclic_move_type=non_cyclic_move_type,
                cyclic_move_type=cyclic_move_type,
                travel_mode=travel_mode,
                position_command_override=position_command_override,
                position_command_overlap=position_command_overlap,
                captured_position_select=captured_position_select,
                start_motion=start_motion,
                stop_motion=stop_motion,
                homing_method=homing_method,
                home_return_speed_rpm=home_return_speed_rpm)

        print(f"\n  --- Commanding mode={operating_mode} ({mode_name}) ---")
        # Preload the mode image with StartMotion low, then hold StartMotion high
        # for several RPIs. A single-frame pulse is often missed (T->O still
        # reflects the prior settle image), so Speed/Position never leave mode 0.
        if ramp_enabled:
            # Phase A: establish start speed, then Phase B: edge to end speed
            # with accel sized for the requested duration.
            speed_command_rpm = speed_start_rpm
            preload_asm = make_mode_asm(start_motion=False, stop_motion=False,
                                        speed_value=speed_start_rpm,
                                        accel_value=ramp_accel_rpm_per_s)
            start_asm = make_mode_asm(start_motion=True, stop_motion=False,
                                      speed_value=speed_start_rpm,
                                      accel_value=ramp_accel_rpm_per_s)
        else:
            preload_asm = make_mode_asm(start_motion=False, stop_motion=False,
                                        speed_value=speed_start_rpm)
            start_asm = make_mode_asm(start_motion=True, stop_motion=False,
                                      speed_value=speed_start_rpm)
        move_asm = preload_asm
        for _ in range(6):
            to_data = io_x(preload_asm)
            if to_data is not None:
                st = parse_input_assembly_154(to_data)
                if "error" not in st:
                    last_status = st
                    trip_reason = get_drive_trip_reason(st)
                    if trip_reason is not None:
                        move_trip_reason = trip_reason
                        print("  [FAIL] Drive alarm/fault during motion preload.")
                        print(f"  Reason: {trip_reason}")
                        break
            _time.sleep(rpi_us / 1000000.0)

        start_time = _time.time()
        exchange_count = 0
        motion_started = False
        motion_complete = False
        move_samples: list[tuple[float, float]] = []
        completion_modes = {1, 3, 6, 7}
        start_motion_frames = 0
        start_motion_hold_frames = 8
        mode_echoed = False
        ramp_phase = "start" if ramp_enabled else "hold"
        # Brief dwell at start speed so the first edge is visible, then re-edge
        # to the end speed for the drive-accel ramp.
        ramp_end_edged = False
        ramp_phase_frames = 0

        while move_trip_reason is None and (_time.time() - start_time) < duration_s:
            elapsed = _time.time() - start_time

            if ramp_enabled:
                if ramp_phase == "start":
                    speed_command_rpm = speed_start_rpm
                    if ramp_phase_frames < start_motion_hold_frames:
                        move_asm = make_mode_asm(start_motion=True, stop_motion=False,
                                                 speed_value=speed_start_rpm,
                                                 accel_value=ramp_accel_rpm_per_s)
                    else:
                        move_asm = make_mode_asm(start_motion=False, stop_motion=False,
                                                 speed_value=speed_start_rpm,
                                                 accel_value=ramp_accel_rpm_per_s)
                    if elapsed >= ramp_start_dwell_s and ramp_phase_frames >= start_motion_hold_frames:
                        ramp_phase = "to_end"
                        ramp_phase_frames = 0
                        print(f"  [OK] Start speed established; commanding end speed "
                              f"{speed_end_cmd:.1f} RPM (accel={ramp_accel_rpm_per_s:.1f}).")
                        continue
                elif ramp_phase == "to_end":
                    speed_command_rpm = speed_end_cmd
                    # Phase-local counters: SM low, then rising-edge SM with end speed.
                    if ramp_phase_frames < 4:
                        move_asm = make_mode_asm(start_motion=False, stop_motion=False,
                                                 speed_value=speed_end_cmd,
                                                 accel_value=ramp_accel_rpm_per_s)
                    elif ramp_phase_frames < 4 + start_motion_hold_frames:
                        move_asm = make_mode_asm(start_motion=True, stop_motion=False,
                                                 speed_value=speed_end_cmd,
                                                 accel_value=ramp_accel_rpm_per_s)
                    else:
                        if not ramp_end_edged:
                            ramp_end_edged = True
                            motion_started = True
                            print("  [OK] End-speed StartMotion edge issued.")
                        move_asm = make_mode_asm(start_motion=False, stop_motion=False,
                                                 speed_value=speed_end_cmd,
                                                 accel_value=ramp_accel_rpm_per_s)
                else:
                    move_asm = make_mode_asm(start_motion=False, stop_motion=False,
                                             speed_value=speed_end_cmd,
                                             accel_value=ramp_accel_rpm_per_s)
            elif start_motion_frames < start_motion_hold_frames:
                move_asm = make_mode_asm(start_motion=True, stop_motion=False,
                                         speed_value=speed_command_rpm)
            else:
                motion_started = True
                move_asm = make_mode_asm(start_motion=False, stop_motion=False,
                                         speed_value=speed_command_rpm)

            to_data = io_x(move_asm)
            if to_data is None:
                print("  [WARN] T->O timeout")
                _time.sleep(0.05)
                continue

            start_motion_frames += 1
            ramp_phase_frames += 1

            status = parse_input_assembly_154(to_data)
            exchange_count += 1
            last_status = status
            add_ramp_sample(move_samples, status, _time.time())

            if "error" not in status:
                trip_reason = get_drive_trip_reason(status)
                if trip_reason is not None:
                    move_trip_reason = trip_reason
                    print("  [FAIL] Drive alarm/fault detected during move. Stopping.")
                    print(f"  Reason: {trip_reason}")
                    break
                if (not mode_echoed
                        and int(status.get("operating_mode", -1)) == operating_mode):
                    mode_echoed = True
                    print(f"  [OK] Drive OperatingMode echoed {operating_mode} ({mode_name}).")
                if operating_mode in completion_modes:
                    command_done = False
                    if (operating_mode == 3
                            and status['homed_status']
                            and status.get('operating_mode') == 3
                            and status.get('active', False)):
                        command_done = True
                    elif operating_mode in (1, 6, 7):
                        command_done = status['at_reference'] or (
                            not status['command_in_progress'] and motion_started and exchange_count > 2)
                    if command_done:
                        print("  [OK] Mode command reached completion condition.")
                        motion_complete = True
                        break

            # Print status every ~500ms
            if exchange_count % 10 == 0:
                if "error" not in status:
                    print(f"  t={elapsed:.1f}s  mode={status['operating_mode']} "
                          f"ready={status['ready']} active={status['active']} "
                          f"cmd={speed_command_rpm:.1f}RPM "
                          f"speed={status['actual_speed']/10:.1f}RPM "
                          f"pos={status['actual_position']}PUU  fault={status['fault']} "
                          f"{format_status_fault_summary(status)}")

            _time.sleep(rpi_us / 1000000.0)

        if (operating_mode in completion_modes) and (not motion_complete):
            print("  [WARN] Completion condition not reached before timeout; initiating stop strategy.")

        # Phase 3: Stop strategy (skip explicit stop when a completion-mode command
        # already reached its done condition).
        print("\n  --- Stopping ---")
        stop_status_count = 0
        initial_stop_speed_rpm = 0.0
        if last_status and "error" not in last_status:
            initial_stop_speed_rpm = abs(last_status['actual_speed'] / 10.0)
        stop_samples: list[tuple[float, float]] = [(_time.time(), initial_stop_speed_rpm)]

        if motion_complete and operating_mode in completion_modes:
            print("  [OK] Completion-mode command finished; skipping explicit stop pulse.")
            stop_ok = True
        else:
            stop_torque_value = 0.0 if operating_mode in MODE_USES_TORQUE_REFERENCE else None
            stop_edge_asm = make_mode_asm(
                start_motion=False, stop_motion=True, speed_value=0.0, torque_value=stop_torque_value)
            stop_hold_asm = make_mode_asm(
                start_motion=False, stop_motion=False, speed_value=0.0, torque_value=stop_torque_value)

            estimated_stop_s = initial_stop_speed_rpm / max(decel_rpm_per_s, 1e-3)
            stop_timeout_s = max(2.0, min(20.0, estimated_stop_s * 4.0 + 1.0))
            print(f"  Decel command: {decel_rpm_per_s:.1f} RPM/s, strategy={stop_strategy} "
                  f"(estimated stop {estimated_stop_s:.2f}s, timeout {stop_timeout_s:.1f}s)")

            stop_started_at = _time.time()
            stop_ok = False
            stop_bit_sent = False
            while (_time.time() - stop_started_at) < stop_timeout_s:
                stop_elapsed = _time.time() - stop_started_at
                if stop_strategy == "stop-bit":
                    stop_frame = stop_edge_asm if not stop_bit_sent else stop_hold_asm
                    stop_bit_sent = True
                elif stop_strategy == "speed-zero":
                    stop_frame = stop_hold_asm
                else:
                    if (not stop_bit_sent) and stop_elapsed >= max(estimated_stop_s * 2.0, 0.25):
                        print("  [WARN] Hybrid stop escalating to stop_motion bit")
                        stop_frame = stop_edge_asm
                        stop_bit_sent = True
                    else:
                        stop_frame = stop_hold_asm

                to_data = io_x(stop_frame)
                if to_data is None:
                    _time.sleep(0.05)
                    continue

                status = parse_input_assembly_154(to_data)
                if "error" in status:
                    _time.sleep(0.05)
                    continue

                last_status = status
                stop_status_count += 1
                add_ramp_sample(stop_samples, status, _time.time())

                trip_reason = get_drive_trip_reason(status)
                if trip_reason is not None:
                    move_trip_reason = trip_reason
                    print("  [FAIL] Drive alarm/fault detected while stopping.")
                    print(f"  Reason: {trip_reason}")
                    break

                speed_abs_rpm = abs(status['actual_speed'] / 10.0)
                if stop_status_count % 5 == 0:
                    print(f"  Stop: speed={status['actual_speed']/10:.1f}RPM "
                          f"stopped={status['stopped']} decel_cmd={decel_rpm_per_s:.1f} "
                          f"{format_status_fault_summary(status)}")

                if status['stopped'] or speed_abs_rpm <= 1.0:
                    print(f"  [OK] Motor stopped (speed={status['actual_speed']/10:.1f}RPM "
                          f"stopped={status['stopped']})")
                    stop_ok = True
                    break

                _time.sleep(max(rpi_us / 1000000.0, 0.01))

            if not stop_ok:
                print(f"  [WARN] Stop not confirmed within {stop_timeout_s:.1f}s; "
                      "continuing to servo disable.")

        print("\n  --- Ramp Verification ---")
        accel_target = ramp_accel_rpm_per_s if ramp_enabled else accel_rpm_per_s
        accel_measured = measured_rate_between(
            move_samples, abs(speed_end_cmd if ramp_enabled else speed_command_rpm),
            0.10, 0.90, rising=True)
        decel_measured = measured_rate_between(
            stop_samples, initial_stop_speed_rpm, 0.90, 0.10, rising=False)
        print_rate_verification("Accel", accel_measured, accel_target, decel_tolerance)
        print_rate_verification("Decel", decel_measured, decel_rpm_per_s, decel_tolerance)

        # Servo state is left unchanged; use servo-off command explicitly.
        print("\n  --- Servo State ---")
        print("  Servo remains under explicit user control (no automatic disable).")

        if last_status and "error" not in last_status:
            print(f"\n  Final: mode={last_status['operating_mode']} "
                  f"ready={last_status['ready']} active={last_status['active']} "
                  f"stopped={last_status['stopped']} speed={last_status['actual_speed']/10:.1f}RPM "
                  f"pos={last_status['actual_position']}PUU "
                  f"{format_status_fault_summary(last_status)}")
        print(f"  Total exchanges: move={exchange_count}, stop={stop_status_count}")

        client.forward_close(result.get("params", params))
        client.unregister_session()
        client.disconnect()
        if hold_paused_for_move:
            print("\n  Restoring persistent servo hold...")
            try:
                if not start_background_hold(rpi_us=resume_hold_rpi_us):
                    print("  [WARN] Could not restore persistent servo hold. Run servo-on.")
            except OSError as exc:
                print(f"  [WARN] Could not restore persistent servo hold ({exc}). Run servo-on.")

    elif args.command == "clear-fault":
        rpi_us = getattr(args, 'rpi', 5000)
        max_cycles = getattr(args, 'max_cycles', 20)

        print("\n=== Clear Drive Fault ===")
        print("  Uses CIP Parameter Object #24 (FaultReset) plus a rising edge")
        print("  on the assembly fault_reset bit (byte 1, bit 3).")
        print("  Note: hardware faults may still require power-cycle.")

        if not client.connect():
            return
        if not client.register_session():
            client.disconnect()
            return

        # Try CIP FaultReset before opening the cyclic connection (some drives
        # reject the write while the Class 1 connection is active).
        print("\n  Pre-ForwardOpen CIP FaultReset attempt...")
        client.write_k5100_parameter(24, 1, size=1)

        params = ForwardOpenParams()
        params.ot_assembly_size = 40
        params.to_assembly_size = 52
        params.ot_rpi_us = rpi_us
        params.to_rpi_us = rpi_us
        params.raw_ot_conn_size = 40 + 2 + 4  # 46
        params.include_run_idle_header = False

        result = client.forward_open(params)
        if result is None:
            client.unregister_session()
            client.disconnect()
            return

        ot_conn_id = result.get("ot_connection_id", 0)
        if ot_conn_id == 0:
            print("[FAIL] ForwardOpen returned zero O->T connection ID")
            client.unregister_session()
            client.disconnect()
            return

        cleared = clear_drive_fault(client, ot_conn_id, max_cycles=max_cycles)

        client.forward_close(result.get("params", params))
        client.unregister_session()
        client.disconnect()

        if cleared:
            print("\n[OK] Fault cleared successfully. Drive ready for motion.")
        else:
            print("\n[FAIL] Could not clear fault. Power-cycle the drive and retry.")

    elif args.command == "home":
        # Home: execute homing sequence.
        # OperatingMode=3 is "Home mode" (UM004D Table 104/106 OperatingMode enum).
        # An earlier version of this tool incorrectly used operating_mode=6
        # ("Index mode") while also setting homing fields, which caused E 602.
        homing_method = getattr(args, 'homing_method', 34)
        operating_mode = getattr(args, 'operating_mode', 3)
        speed_fast = getattr(args, 'speed_fast', 60.0)
        speed_slow = getattr(args, 'speed_slow', 6.0)
        accel_rpm_per_s = getattr(args, 'accel', DEFAULT_HOME_ACCEL_RPM_PER_S)
        decel_rpm_per_s = getattr(args, 'decel', DEFAULT_HOME_DECEL_RPM_PER_S)
        rpi_us = getattr(args, 'rpi', 5000)
        hold_state_before_home = get_background_hold_state()
        resume_hold_after_home = hold_state_before_home is not None
        resume_hold_rpi_us = int(hold_state_before_home.get("rpi_us", rpi_us)) if hold_state_before_home else int(rpi_us)
        hold_paused_for_home = False
        if hold_state_before_home:
            print("  [INFO] Persistent servo hold detected; home will take temporary ownership.")
        timeout_s = getattr(args, 'timeout', 30.0)
        pid_method = getattr(args, 'param_method', None)

        # Validate homing command ranges before touching the drive.
        speed_fast_raw = int(speed_fast * 10)
        speed_slow_raw = int(speed_slow * 10)
        if not (0 <= homing_method <= 38):
            print(f"\n[ABORT] HomingMethod out of range: {homing_method} (valid: 0..38)")
            return
        if not (1 <= speed_fast_raw <= 20000):
            print(f"\n[ABORT] SpeedReference out of range for home mode: {speed_fast} RPM "
                  f"(raw={speed_fast_raw}, valid raw range=1..20000)")
            return
        if not (1 <= speed_slow_raw <= 5000):
            print(f"\n[ABORT] HomeReturnSpeed out of range: {speed_slow} RPM "
                  f"(raw={speed_slow_raw}, valid raw range=1..5000)")
            return
        range_error = validate_accel_decel_ranges(accel_rpm_per_s, decel_rpm_per_s)
        if range_error is not None:
            print(f"\n[ABORT] Invalid homing command input: {range_error}")
            return

        print(f"\n=== Homing Sequence ===")
        print(f"  OperatingMode: {operating_mode} (3=Home), Homing Method: {homing_method}")
        print(f"  Fast: {speed_fast} RPM, Slow: {speed_slow} RPM")
        print(f"  Accel: {accel_rpm_per_s} RPM/s, Decel: {decel_rpm_per_s} RPM/s")
        print(f"  Timeout: {timeout_s}s, RPI: {rpi_us}us")
        if homing_method == 34:
            print("  Method 34: defines current position as origin. No switches required.")
        elif 0 <= homing_method <= 33:
            print("  [NOTE] Methods 0-33 require a limit switch or ORG (home) switch wired")
            print("  to the drive. Homing will not complete without that hardware input.")
        elif 35 <= homing_method <= 38:
            print("  [NOTE] Methods 35-38 use torque-based collision homing and require")
            print("  DI.Enable Homing plus configured P1.087/P1.088 torque-limit params.")
        if pid_method:
            print(f"  Will also pre-write HomingMethod via CIP Parameter Object instance {pid_method}")
        print("  SAFETY: Ensure the motor can move freely and E-Stop is accessible!")
        wait_for_user_confirmation("Pre-flight acknowledged.")

        # Pre-flight: check drive is powered on and in I/O mode
        print("\n  --- Pre-flight Check ---")
        if not client.connect():
            return
        if not client.register_session():
            client.disconnect()
            return
        status = client.check_drive_ready()
        client.unregister_session()
        client.disconnect()

        if not status["powered_on"]:
            print("\n[ABORT] Drive not responding. Check power and Ethernet connection.")
            return
        if not status["io_mode"]:
            print(f"\n[ABORT] Drive is NOT in I/O mode (P1.001=0x{status['p1_001_value']:04X} = {status['p1_001_name']}).")
            print("  Use KNX5100C or the drive keypad to set P1.001 = 0x0C (IO mode),")
            print("  then power-cycle the drive and retry.")
            return
        if not status.get("servo_enabled", status.get("servo_active", False)):
            print("\n[ABORT] Servo is not enabled.")
            print("  Run: py tools/eip_test.py servo-on")
            print("  Then retry the homing command.")
            return

        print("  [OK] Drive is in I/O mode and servo is already enabled.")
        print("  Proceeding with homing sequence without toggling servo state.\n")

        # Pause hold BEFORE opening a new TCP session (see move handoff notes).
        if resume_hold_after_home and not hold_paused_for_home:
            if not pause_background_hold_for_handoff("handoff:home", timeout_s=8.0):
                print("  [FAIL] Could not pause persistent servo hold.")
                return
            hold_paused_for_home = True

        # Reconnect for CIP + ForwardOpen
        if not client.connect():
            if hold_paused_for_home:
                print("  Restoring persistent servo hold after connect failure...")
                start_background_hold(rpi_us=resume_hold_rpi_us)
            return
        if not client.register_session():
            client.disconnect()
            if hold_paused_for_home:
                print("  Restoring persistent servo hold after session failure...")
                start_background_hold(rpi_us=resume_hold_rpi_us)
            return

        # Phase 0: Optionally pre-write HomingMethod via CIP Parameter Object
        # instance 29 (verified in EDS: "Homing_Method", range 0-38, matches
        # Table 112 exactly). This is normally unnecessary since the assembly's
        # byte 3 already carries the same value every cycle.
        if pid_method is not None:
            print("\n  --- Writing Homing Method via CIP ---")
            ok = client.write_k5100_parameter(pid_method, homing_method, size=1)
            print(f"  Instance {pid_method} (HomingMethod) = {homing_method}: {'OK' if ok else 'FAIL'}")

        # ForwardOpen for cyclic I/O
        params = ForwardOpenParams()
        params.ot_assembly_size = 40
        params.to_assembly_size = 52
        params.ot_rpi_us = rpi_us
        params.to_rpi_us = rpi_us
        params.raw_ot_conn_size = 40 + 2 + 4  # 46
        params.include_run_idle_header = True
        params.connection_timeout_multiplier = 7

        result = forward_open_with_retry(client, params)
        if result is None:
            client.unregister_session()
            client.disconnect()
            if hold_paused_for_home:
                print("  Restoring persistent servo hold after handoff failure...")
                time.sleep(HOLD_OWNERSHIP_SETTLE_S)
                start_background_hold(rpi_us=resume_hold_rpi_us)
            return

        ot_conn_id = result.get("ot_connection_id", 0)
        if ot_conn_id == 0:
            print("[FAIL] ForwardOpen returned zero O->T connection ID")
            client.unregister_session()
            client.disconnect()
            if hold_paused_for_home:
                print("  Restoring persistent servo hold after connection ID failure...")
                time.sleep(HOLD_OWNERSHIP_SETTLE_S)
                start_background_hold(rpi_us=resume_hold_rpi_us)
            return

        print(f"  O->T conn_id=0x{ot_conn_id:08X}, RPI={rpi_us}us")

        import time as _time

        # Quiet per-frame UDP chatter; keep phase / status prints.
        client.verbose = False

        # Servo state is intentionally not modified in home command.
        print("  [OK] Servo state is managed independently (no auto enable/disable).")

        # Phase 2: enter Home mode first, THEN rising-edge StartMotion.
        # Pulsing StartMotion in the same frame that first selects OperatingMode=Home
        # produces A603 (Invalid I/O command data) on Kinetix 5100.
        print(f"\n  --- Sending Homing Command via Assembly "
              f"(mode={operating_mode}, method={homing_method}) ---")

        last_status = None
        home_trip_reason = None
        home_mode_seen = False
        home_command_seen = False
        homed = False

        def io_x(asm: bytes, timeout: float = 2.0):
            return client.exchange_io_frame(
                ot_conn_id, asm, include_run_idle=True, timeout=timeout,
                reuse_socket=True, drain=True)

        def make_home_asm(start_motion: bool) -> bytes:
            # Home mode on this drive accepts Travel Mode 2 (non-cyclic). Bench
            # probes showed OperatingMode never echoes 3 with travel_mode=10
            # while a prior Position command was still sticky; travel_mode=2
            # enters Home cleanly. Values 0/1 are reserved (A603).
            home_ret = max(float(speed_slow), 0.1)
            return build_output_assembly_104(
                servo_on=True,
                speed_rpm=max(float(speed_fast), 0.1),
                accel_rpm_per_s=accel_rpm_per_s,
                decel_rpm_per_s=decel_rpm_per_s,
                operating_mode=operating_mode,
                travel_mode=2,
                start_motion=start_motion,
                homing_method=homing_method,
                home_return_speed_rpm=home_ret,
                torque_ramp_time_ms=1000)

        # --- Phase 2a: re-enable after handoff (ServoOn is 0→1 edge) ---
        # Match the hold worker: seed ServoOn=0, then assert ServoOn=1 so Active
        # reappears on the new Class 1 connection before Home mode entry.
        print("  Settling servo (mode 0) before Home mode entry...")
        seed_asm = build_output_assembly_104(
            servo_on=False, servo_off=False, operating_mode=0, travel_mode=10,
            torque_ramp_time_ms=1000)
        settle_asm = build_output_assembly_104(
            servo_on=True,
            servo_off=False,
            speed_rpm=0.0,
            operating_mode=0,
            travel_mode=10,
            start_motion=False,
            stop_motion=False,
            torque_ramp_time_ms=1000)
        stop_asm = build_output_assembly_104(
            servo_on=True,
            stop_motion=True,
            operating_mode=0,
            travel_mode=10,
            torque_ramp_time_ms=1000)
        clear_low = build_output_assembly_104(
            servo_on=True, operating_mode=0, travel_mode=10, torque_ramp_time_ms=1000)
        clear_high = build_output_assembly_104(
            servo_on=True, fault_reset=True, operating_mode=0, travel_mode=10,
            torque_ramp_time_ms=1000)

        for _ in range(3):
            to_data = io_x(seed_asm)
            if to_data is not None:
                st = parse_input_assembly_154(to_data)
                if "error" not in st:
                    last_status = st
            _time.sleep(rpi_us / 1000000.0)

        settle_deadline = _time.time() + 3.0
        settled_active = False
        a603_clear_attempts = 0
        stop_pulsed = False
        while _time.time() < settle_deadline:
            to_data = io_x(settle_asm)
            if to_data is None:
                _time.sleep(0.05)
                continue
            st = parse_input_assembly_154(to_data)
            if "error" in st:
                continue
            last_status = st
            if bool(st.get("fault", False)):
                home_trip_reason = get_drive_trip_reason(st)
                print("  [FAIL] Drive fault during pre-home settle.")
                print(f"  Reason: {home_trip_reason}")
                break
            if (bool(st.get("warning_present", False))
                    and (int(st.get("warning_code", 0)) & 0x0FFF) == 0x0603
                    and a603_clear_attempts < 2):
                a603_clear_attempts += 1
                print("  [INFO] Clearing latched A603 before Home entry...")
                for _ in range(3):
                    io_x(clear_high)
                    _time.sleep(0.05)
                for _ in range(4):
                    io_x(clear_low)
                    _time.sleep(0.05)
                continue
            if (bool(st.get("active", False)) and bool(st.get("ready", False))
                    and not bool(st.get("warning_present", False))):
                if bool(st.get("command_in_progress", False)) and not stop_pulsed:
                    stop_pulsed = True
                    print("  [INFO] Canceling leftover CommandInProgress before Home...")
                    for _ in range(6):
                        io_x(stop_asm)
                        _time.sleep(rpi_us / 1000000.0)
                    continue
                settled_active = True
                print("  [OK] Servo Active/Ready after settle.")
                break
            _time.sleep(rpi_us / 1000000.0)
        if home_trip_reason is None and not settled_active:
            print("  [WARN] Clean Active/Ready not seen during settle; continuing to Home mode.")
            if last_status and bool(last_status.get("warning_present", False)):
                print(f"  Settle warning: {format_status_fault_summary(last_status)}")
            if last_status:
                print(f"  Settle status: mode={last_status.get('operating_mode')} "
                      f"active={last_status.get('active')} "
                      f"ready={last_status.get('ready')} "
                      f"cmd={last_status.get('command_in_progress')} "
                      f"{format_status_fault_summary(last_status)}")

        print("  Preloading Home mode command (StartMotion held low)...")
        home_asm = make_home_asm(start_motion=False)
        preload_deadline = _time.time() + 0.5
        a603_entry_cycles = 0
        while home_trip_reason is None and _time.time() < preload_deadline:
            to_data = io_x(home_asm)
            if to_data is None:
                _time.sleep(0.05)
                continue
            st = parse_input_assembly_154(to_data)
            if "error" in st:
                continue
            last_status = st
            if bool(st.get("fault", False)) or bool(st.get("connection_faulted", False)):
                home_trip_reason = get_drive_trip_reason(st)
                print("  [FAIL] Drive fault while preloading Home mode.")
                print(f"  Reason: {home_trip_reason}")
                break
            if bool(st.get("warning_present", False)) and (
                    int(st.get("warning_code", 0)) & 0x0FFF) == 0x0603:
                a603_entry_cycles += 1
                if a603_entry_cycles >= 10:
                    home_trip_reason = get_drive_trip_reason(st)
                    print("  [FAIL] Persistent A603 while preloading Home mode.")
                    print(f"  Reason: {home_trip_reason}")
                    print("  Hint: check speed/accel/home-return ranges and HomingMethod.")
                    break
            else:
                a603_entry_cycles = 0
            # OperatingMode often stays 0 until StartMotion; echo here is optional.
            if int(st.get("operating_mode", -1)) == operating_mode:
                home_mode_seen = True
            _time.sleep(rpi_us / 1000000.0)

        if home_trip_reason is None:
            print("  Pulsing StartMotion for homing trigger...")
            home_asm = make_home_asm(start_motion=True)
            to_data = io_x(home_asm)
            if to_data is not None:
                st = parse_input_assembly_154(to_data)
                if "error" not in st:
                    last_status = st
                    if int(st.get("operating_mode", -1)) == operating_mode:
                        home_mode_seen = True
            home_asm = make_home_asm(start_motion=False)

            start_time = _time.time()
            a603_cycles = 0
            while (_time.time() - start_time) < timeout_s:
                to_data = io_x(home_asm)
                if to_data is None:
                    print("  [WARN] T->O timeout, retrying...")
                    _time.sleep(0.05)
                    continue

                st = parse_input_assembly_154(to_data)
                last_status = st
                if "error" in st:
                    _time.sleep(rpi_us / 1000000.0)
                    continue

                if bool(st.get("fault", False)) or bool(st.get("connection_faulted", False)):
                    home_trip_reason = get_drive_trip_reason(st)
                    print("  [FAIL] Drive alarm/fault detected during homing. Stopping.")
                    print(f"  Reason: {home_trip_reason}")
                    break

                if bool(st.get("warning_present", False)):
                    wcode = int(st.get("warning_code", 0)) & 0x0FFF
                    if wcode == 0x0603:
                        # Tolerate brief A603 around the StartMotion edge.
                        a603_cycles += 1
                        if a603_cycles >= 8:
                            home_trip_reason = get_drive_trip_reason(st)
                            print("  [FAIL] Persistent A603 during homing. Stopping.")
                            print(f"  Reason: {home_trip_reason}")
                            break
                    else:
                        home_trip_reason = get_drive_trip_reason(st)
                        print("  [FAIL] Drive alarm/fault detected during homing. Stopping.")
                        print(f"  Reason: {home_trip_reason}")
                        break
                else:
                    a603_cycles = 0

                if st.get('operating_mode') == operating_mode:
                    if not home_mode_seen:
                        print(f"  [OK] Drive OperatingMode echoed {operating_mode} (Home).")
                    home_mode_seen = True
                if st.get('command_in_progress', False):
                    home_command_seen = True

                homed = bool(st['homed_status'])
                elapsed = _time.time() - start_time
                if int(elapsed * 2) % 2 == 0:
                    print(f"  t={elapsed:.1f}s  mode={st['operating_mode']} ready={st['ready']} "
                          f"active={st['active']} homed={homed} cmd={st.get('command_in_progress')} "
                          f"speed={st['actual_speed']/10:.1f}RPM  "
                          f"fault={st['fault']} {format_status_fault_summary(st)}")

                # Bench: OperatingMode echoes 3 only after StartMotion. Require
                # that echo plus HomedStatus (method 34 clears then re-asserts it).
                if not home_mode_seen:
                    _time.sleep(rpi_us / 1000000.0)
                    continue

                method34_done = (
                    homing_method == 34
                    and home_mode_seen
                    and bool(st.get("active", False))
                    and homed
                    and home_command_seen)
                other_done = (
                    homing_method != 34
                    and home_mode_seen
                    and homed
                    and home_command_seen
                    and bool(st.get("active", False)))
                if method34_done or other_done:
                    print(f"  [OK] Homing completed at t={elapsed:.1f}s")
                    break

                _time.sleep(rpi_us / 1000000.0)

        if home_trip_reason is not None:
            print("\n  [ABORT] Homing stopped due to drive alarm/fault condition.")
            print(f"  Reason: {home_trip_reason}")
        elif not homed:
            print(f"\n  [WARN] Homing did not complete within {timeout_s}s timeout.")
            if last_status and "error" not in last_status:
                print(f"  Final: ready={last_status['ready']} active={last_status['active']} "
                      f"homed={last_status['homed_status']} fault={last_status['fault']}")
            if homing_method != 34:
                print(f"  Tip: method {homing_method} needs a physical switch/DI input.")
                print("  Try --homing-method 34 first to confirm the trigger mechanism works")
                print("  (it requires no extra wiring and should complete almost instantly).")
            else:
                print("  Tip: method 34 needs no extra hardware — if it still doesn't complete,")
                print("  check that OperatingMode actually reached 3 (input assembly 'mode' field)")
                print("  by inspecting the printed status lines above, and check for warnings.")
            if not (home_mode_seen or home_command_seen):
                print("  Note: home command was never acknowledged (mode/command bits not observed).")
        else:
            print(f"\n  [OK] Homing succeeded (method={homing_method}).")
            if last_status and "error" not in last_status:
                print(f"  Final position: {last_status.get('actual_position', 0)} PUU")

        print("\n  --- Servo State ---")
        print("  Servo remains under explicit user control (no automatic disable).")

        if last_status and "error" not in last_status:
            print(f"\n  Final: homed={homed} speed={last_status['actual_speed']/10:.1f}RPM "
                  f"pos={last_status['actual_position']}PUU "
                  f"{format_status_fault_summary(last_status)}")
        print("  Homing sequence complete.")

        client.forward_close(result.get("params", params))
        client.unregister_session()
        client.disconnect()
        if hold_paused_for_home:
            print("\n  Restoring persistent servo hold...")
            try:
                if not start_background_hold(rpi_us=resume_hold_rpi_us):
                    print("  [WARN] Could not restore persistent servo hold. Run servo-on.")
            except OSError as exc:
                print(f"  [WARN] Could not restore persistent servo hold ({exc}). Run servo-on.")

    elif args.command == "check-mode":
        print("\n=== Drive Readiness Check ===")
        hold_state = get_background_hold_state()
        if hold_state:
            print(f"  Persistent hold service: RUNNING (pid={hold_state.get('pid')}, "
                  f"status={hold_state.get('status', 'unknown')})")
            if hold_state.get("profile"):
                print(f"  Hold profile: {hold_state.get('profile')}")
            if hold_state.get("last_error"):
                print(f"  Hold last error: {hold_state.get('last_error')}")
            if hold_state.get("last_seen"):
                print(f"  Hold heartbeat: {hold_state.get('last_seen')}")
        else:
            print("  Persistent hold service: STOPPED")
        if HOLD_LOG_FILE.exists():
            print(f"  Hold worker log: {HOLD_LOG_FILE}")
        if not client.connect():
            return
        if not client.register_session():
            client.disconnect()
            return

        status = client.check_drive_ready()

        if status["powered_on"]:
            if status["io_mode"]:
                print(f"  Servo ready (Param44): {status.get('servo_ready', False)}")
                print(f"  Servo active (Param45): {status.get('servo_active', False)}")
                if status.get("servo_enabled", status.get("servo_active", False)):
                    print("\n  [OK] Drive is in I/O mode and servo is enabled.")
                else:
                    print("\n  [WARN] Drive is in I/O mode but servo is not enabled.")
                    print("  Run: py tools/eip_test.py servo-on")
            else:
                print(f"\n  [WARN] Drive is NOT in I/O mode.")
                print(f"  Current mode: P1.001 = 0x{status['p1_001_value']:04X} ({status['p1_001_name']})")
                print("  Action: Set P1.001 to 0x0C (IO mode) via KNX5100C or keypad,")
                print("  then power-cycle the drive.")

        client.unregister_session()
        client.disconnect()

    elif args.command == "get-param":
        param_id = getattr(args, 'param_id', 0)
        print(f"\n=== Read Parameter P1.{param_id:03d} (ID={param_id}, 0x{param_id:04X}) ===")
        if not client.connect():
            return
        if not client.register_session():
            client.disconnect()
            return

        value = client.read_k5100_parameter(param_id)
        if value is not None:
            print(f"  Value = {value} (0x{value:04X})")
        else:
            print("  [FAIL] Could not read parameter.")

        client.unregister_session()
        client.disconnect()

    elif args.command in ("forward-open", "full-test"):
        params = ForwardOpenParams()
        params.config_instance = args.config_instance
        params.ot_instance = args.ot_instance
        params.to_instance = args.to_instance
        params.ot_assembly_size = args.ot_size
        params.to_assembly_size = args.to_size
        params.ot_rpi_us = args.ot_rpi
        params.to_rpi_us = args.to_rpi
        params.originator_vendor_id = args.vendor_id
        params.originator_serial = args.serial

        # Raw connection size overrides
        if hasattr(args, 'raw_ot_conn_size') and args.raw_ot_conn_size is not None:
            params.raw_ot_conn_size = args.raw_ot_conn_size
        if hasattr(args, 'raw_to_conn_size') and args.raw_to_conn_size is not None:
            params.raw_to_conn_size = args.raw_to_conn_size

        # Run/Idle: --run-idle enables, --no-run-idle disables, default = False
        if args.run_idle:
            params.include_run_idle_header = True
        elif args.no_run_idle:
            params.include_run_idle_header = False

        # Run/Idle bit-only mode
        if hasattr(args, 'run_idle_bit') and args.run_idle_bit is not None:
            params.run_idle_bit_only = args.run_idle_bit

        # Connection type: --mcast enables multicast, --p2p enables P2P (default P2P)
        if args.mcast:
            params.to_connection_type = CONN_TYPE_MULTICAST
        elif args.p2p:
            params.to_connection_type = CONN_TYPE_P2P

        if args.command == "full-test":
            result = run_full_test(client, params)
            if result["success"]:
                print("\n[PASS] full-test completed successfully")
            else:
                print(f"\n[FAIL] full-test failed at stage: {result['stage']}")
        else:
            # forward-open only
            if not client.connect():
                return
            if not client.register_session():
                client.disconnect()
                return
            result = run_forward_open(client, params)
            if result is not None:
                client.forward_close(result.get("params", params))
            client.unregister_session()
            client.disconnect()

    # Close log file if one was opened
    if args.log:
        try:
            sys.stdout.close()
        except Exception:
            pass
        sys.stdout = sys.__stdout__


if __name__ == "__main__":
    main()

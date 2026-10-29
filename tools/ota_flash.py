#!/usr/bin/env python3
"""WT32-ETH01 Ethernet OTA Firmware Flasher over LAN8720.

Streams a compiled firmware binary (.bin) over the plant Ethernet network to the
WT32 Gantry Controller's Dual-OTA partition manager.

Default path is a direct TCP connection to the gantry, which requires this PC
on the cell switch (`10.42.0.0/24`). `--wireless <Pi IP>` relays through that
supervisor (user `$PI_USER`, password `$PI_PASS`) when the cable is not connected.

Usage:
  python tools/ota_flash.py --destination 10.42.0.100 idf/build/wt32_eth01_gantry.bin
  python tools/ota_flash.py --wireless 10.168.40.1 --destination 10.42.0.100 idf/build/wt32_eth01_gantry.bin

Safety Interlock:
  The firmware disables enabled-but-idle motors itself at START (and again
  before the reboot). It rejects OTA while a move, home or pick is running.
"""

from __future__ import annotations

import argparse
import os
import shutil
import socket
import subprocess
import sys
import tempfile
import threading
import time
from collections.abc import Callable

DEFAULT_HOST = "10.42.0.100"
DEFAULT_PORT = 8032
DEFAULT_PI_HOST = "10.168.40.1"
DEFAULT_PI_USER = os.environ.get("PI_USER", "robotics")
DEFAULT_PASSWORD = os.environ.get("GANTRY_TCP_PASSWORD", "LTU_1932")
CHUNK_SIZE = 4096

LogFn = Callable[[str], None]
ProgressFn = Callable[[int, int, float], None]  # sent_bytes, total_bytes, KiB/s


def read_line(sock: socket.socket, timeout: float = 5.0) -> str:
    sock.settimeout(timeout)
    data = bytearray()
    while True:
        try:
            ch = sock.recv(1)
        except socket.timeout:
            raise TimeoutError("Socket timed out waiting for line response")
        if not ch:
            break
        if ch == b"\n":
            break
        if ch != b"\r":
            data.extend(ch)
    return data.decode("utf-8", errors="replace")


def flash_ota(
    host: str,
    port: int,
    password: str,
    bin_path: str,
    *,
    log: LogFn | None = None,
    progress: ProgressFn | None = None,
    connect_timeout: float = 5.0,
) -> bool:
    """Stream a .bin to the Dual-OTA TCP server (LAN8720, default :8032).

    `log` / `progress` let a GUI consume the same protocol as the CLI. When
    omitted, this prints to stdout/stderr as before.
    """
    to_stdout = log is None

    def emit(msg: str, *, error: bool = False) -> None:
        if log is not None:
            log(msg)
        elif error:
            print(msg, file=sys.stderr)
        else:
            print(msg)

    if not os.path.isfile(bin_path):
        emit(f"[ERROR] Firmware file not found: {bin_path}", error=True)
        return False

    file_size = os.path.getsize(bin_path)
    emit("============================================================")
    emit("=== WT32-ETH01 LAN8720 ETHERNET OTA FLASHER               ===")
    emit("============================================================")
    emit(f"Target Host      : {host}:{port}")
    emit(f"Firmware File    : {bin_path}")
    emit(f"Binary Size      : {file_size:,} bytes ({file_size / 1024:.1f} KB)")
    emit("============================================================")

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(connect_timeout)

    try:
        emit(f"Connecting to {host}:{port}...")
        sock.connect((host, port))
        emit("Connected.")

        sock.sendall(f"AUTH {password}\n".encode("utf-8"))
        resp = read_line(sock, 5.0)
        if not resp.startswith("OK AUTH"):
            emit(f"[ERROR] Authentication failed: {resp}", error=True)
            return False
        emit("Authentication verified (OK).")

        sock.sendall(f"START {file_size}\n".encode("utf-8"))
        resp = read_line(sock, 8.0)
        if not resp.startswith("OK READY"):
            emit(f"[ERROR] OTA initiation rejected by target: {resp}", error=True)
            return False
        emit("Target ready for binary stream.")

        emit("Streaming firmware binary...")
        start_time = time.time()
        sent_bytes = 0

        with open(bin_path, "rb") as f:
            while sent_bytes < file_size:
                chunk = f.read(CHUNK_SIZE)
                if not chunk:
                    break
                sock.sendall(chunk)
                sent_bytes += len(chunk)
                elapsed = time.time() - start_time
                speed_kb = (sent_bytes / 1024.0) / (elapsed if elapsed > 0 else 0.001)
                if progress is not None:
                    progress(sent_bytes, file_size, speed_kb)
                elif to_stdout:
                    pct = (sent_bytes / file_size) * 100.0
                    bar_len = 30
                    filled = int(bar_len * (sent_bytes / file_size))
                    bar = "=" * filled + "-" * (bar_len - filled)
                    print(
                        f"\r[{bar}] {pct:5.1f}% ({sent_bytes / 1024:.1f} / "
                        f"{file_size / 1024:.1f} KB) @ {speed_kb:5.1f} KB/s",
                        end="",
                        flush=True,
                    )

        if to_stdout:
            print()
        emit("All bytes transmitted. Finalizing flash and verifying image...")

        resp = read_line(sock, 15.0)
        if not resp.startswith("OK COMPLETE"):
            emit(f"[ERROR] OTA validation failed on target: {resp}", error=True)
            return False

        elapsed = time.time() - start_time
        avg_speed = (file_size / 1024.0) / (elapsed if elapsed > 0 else 0.001)
        if progress is not None:
            progress(file_size, file_size, avg_speed)
        emit("============================================================")
        emit(f"[SUCCESS] OTA Flash Completed in {elapsed:.2f} s ({avg_speed:.1f} KB/s)")
        emit(f"Target Response  : {resp}")
        emit("Target WT32 is now rebooting into the new firmware slot!")
        emit("============================================================")
        return True

    except Exception as e:
        emit(f"[ERROR] Connection error during OTA flash: {e}", error=True)
        return False
    finally:
        try:
            sock.close()
        except Exception:
            pass


def _local_port_is_listening(port: int) -> bool:
    probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        probe.bind(("127.0.0.1", port))
    except OSError:
        return True
    else:
        return False
    finally:
        probe.close()


def _write_askpass() -> str:
    """Helper OpenSSH runs to read PI_PASS. The password stays in the environment."""
    handle = tempfile.NamedTemporaryFile(
        suffix=".cmd", delete=False, mode="w", encoding="ascii", newline="\r\n"
    )
    exe = sys.executable.replace("%", "%%")
    handle.write("@echo off\n")
    handle.write(f"\"{exe}\" -c \"import os; print(os.environ['PI_PASS'])\"\n")
    handle.close()
    return handle.name


def flash_via_pi(
    host: str,
    port: int,
    password: str,
    bin_path: str,
    *,
    pi_host: str = DEFAULT_PI_HOST,
    pi_user: str | None = None,
    pi_password: str | None = None,
    pi_key: str | None = None,
    log: LogFn | None = None,
    progress: ProgressFn | None = None,
) -> bool:
    """SSH to the Pi and stream the image to the gantry on the cell LAN.

    `pi_password` defaults to `$PI_PASS`. It is never placed on the ssh command line.
    `pi_key` (default `$PI_KEY`) is the private key the Pi trusts; when unset, ssh
    picks its own default key.
    """

    def emit(msg: str, *, error: bool = False) -> None:
        if log is not None:
            log(msg)
        elif error:
            print(msg, file=sys.stderr)
        else:
            print(msg)

    user = pi_user or os.environ.get("PI_USER", DEFAULT_PI_USER)
    secret = pi_password if pi_password is not None else os.environ.get("PI_PASS", "")
    if not secret:
        emit("[ERROR] PI_PASS is not set", error=True)
        return False
    if shutil.which("ssh") is None:
        emit("[ERROR] ssh client not found on PATH", error=True)
        return False

    local_port = 0
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as reserved:
        reserved.bind(("127.0.0.1", 0))
        local_port = int(reserved.getsockname()[1])

    askpass = _write_askpass()
    env = os.environ.copy()
    env["PI_PASS"] = secret
    env["SSH_ASKPASS"] = askpass
    env["SSH_ASKPASS_REQUIRE"] = "force"
    env["DISPLAY"] = "localhost:0"
    stderr_chunks: list[bytes] = []

    key = pi_key if pi_key is not None else os.environ.get("PI_KEY", "")
    key_args: list[str] = []
    if key:
        key = os.path.expanduser(key)
        if not os.path.isfile(key):
            emit(f"[ERROR] SSH key not found: {key}", error=True)
            return False
        key_args = ["-i", key, "-o", "IdentitiesOnly=yes"]

    emit(f"Opening SSH forward {user}@{pi_host} -> {host}:{port}"
         + (f" (key {os.path.basename(key)})" if key else ""))
    proc = subprocess.Popen(
        [
            "ssh",
            "-N",
            *key_args,
            "-o", "ExitOnForwardFailure=yes",
            "-o", "StrictHostKeyChecking=accept-new",
            # The Pi offers password only after a public-key attempt fails.
            "-o", "PreferredAuthentications=publickey,password",
            "-o", "NumberOfPasswordPrompts=1",
            "-o", "ConnectTimeout=15",
            "-L", f"127.0.0.1:{local_port}:{host}:{port}",
            f"{user}@{pi_host}",
        ],
        env=env,
        stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
    )

    def _drain() -> None:
        if proc.stderr is None:
            return
        stderr_chunks.append(proc.stderr.read())

    drain = threading.Thread(target=_drain, daemon=True)
    drain.start()

    try:
        deadline = time.time() + 20.0
        while time.time() < deadline:
            if proc.poll() is not None:
                drain.join(timeout=2.0)
                detail = b"".join(stderr_chunks).decode("utf-8", errors="replace").strip()
                if "Permission denied (password)" in detail:
                    detail += " PI_PASS was sent and the supervisor rejected it for this user."
                emit(
                    "[ERROR] SSH forward to the Pi failed"
                    + (f": {detail}" if detail else ""),
                    error=True,
                )
                return False
            if _local_port_is_listening(local_port):
                break
            time.sleep(0.1)
        else:
            emit("[ERROR] SSH forward did not start listening", error=True)
            return False

        return flash_ota(
            "127.0.0.1",
            local_port,
            password,
            bin_path,
            log=log,
            progress=progress,
            connect_timeout=20.0,
        )
    finally:
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
        drain.join(timeout=2.0)
        try:
            os.unlink(askpass)
        except OSError:
            pass


def main():
    parser = argparse.ArgumentParser(
        description="Stream firmware updates to WT32-ETH01 over LAN8720 Ethernet."
    )
    parser.add_argument("bin", help="Path to compiled firmware .bin file")
    parser.add_argument(
        "--destination",
        metavar="WT32_IP",
        default=DEFAULT_HOST,
        help=f"Gantry plant IP (default: {DEFAULT_HOST})",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=DEFAULT_PORT,
        help=f"Target OTA port (default: {DEFAULT_PORT})",
    )
    parser.add_argument(
        "--password",
        default=DEFAULT_PASSWORD,
        help="Gantry OTA password (default: LTU_1932 or $GANTRY_TCP_PASSWORD)",
    )
    parser.add_argument(
        "--wireless",
        metavar="PI_IP",
        help="Flash through the supervisor at PI_IP when this PC is not on the cell switch "
        "(SSH user $PI_USER or robotics, password from PI_PASS)",
    )
    parser.add_argument(
        "--pi-user",
        default=DEFAULT_PI_USER,
        help="Supervisor SSH user for --wireless (default: $PI_USER or robotics)",
    )
    parser.add_argument(
        "--pi-key",
        default=None,
        help="Private key the supervisor trusts for --wireless, e.g. ~/.ssh/ltukey_s "
        "(default: $PI_KEY, else ssh's default key)",
    )

    args = parser.parse_args()
    if args.wireless:
        success = flash_via_pi(
            args.destination,
            args.port,
            args.password,
            args.bin,
            pi_host=args.wireless,
            pi_user=args.pi_user,
            pi_key=args.pi_key,
        )
    else:
        success = flash_ota(args.destination, args.port, args.password, args.bin)
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()

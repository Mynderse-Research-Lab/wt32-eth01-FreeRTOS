#!/usr/bin/env python3
"""WT32-ETH01 Ethernet OTA Firmware Flasher over LAN8720.

Streams a compiled firmware binary (.bin) over the plant Ethernet network to the
WT32 Gantry Controller's Dual-OTA partition manager.

Usage:
  python tools/eth_ota_flash.py idf/build/wt32_eth01_gantry.bin
  python tools/eth_ota_flash.py --host 192.168.1.100 --port 8032 idf/build/wt32_eth01_gantry.bin

Safety Interlock:
  The firmware automatically rejects OTA attempts if the gantry is ENABLED or
  BUSY to protect industrial actuators and mechanics.
"""

from __future__ import annotations

import argparse
import os
import socket
import sys
import time
from collections.abc import Callable

DEFAULT_HOST = "192.168.1.100"
DEFAULT_PORT = 8032
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
    sock.settimeout(5.0)

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


def main():
    parser = argparse.ArgumentParser(
        description="Stream firmware updates to WT32-ETH01 over LAN8720 Ethernet."
    )
    parser.add_argument("bin", help="Path to compiled firmware .bin file")
    parser.add_argument(
        "--host",
        default=DEFAULT_HOST,
        help=f"Target IP address (default: {DEFAULT_HOST})",
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
        help="Authentication password (default: LTU_1932 or $GANTRY_TCP_PASSWORD)",
    )

    args = parser.parse_args()
    success = flash_ota(args.host, args.port, args.password, args.bin)
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()

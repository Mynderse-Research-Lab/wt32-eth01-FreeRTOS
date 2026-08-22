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

DEFAULT_HOST = "192.168.1.100"
DEFAULT_PORT = 8032
DEFAULT_PASSWORD = os.environ.get("GANTRY_TCP_PASSWORD", "LTU_1932")
CHUNK_SIZE = 4096


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


def flash_ota(host: str, port: int, password: str, bin_path: str) -> bool:
    if not os.path.isfile(bin_path):
        print(f"[ERROR] Firmware file not found: {bin_path}", file=sys.stderr)
        return False

    file_size = os.path.getsize(bin_path)
    print(f"============================================================")
    print(f"=== WT32-ETH01 LAN8720 ETHERNET OTA FLASHER               ===")
    print(f"============================================================")
    print(f"Target Host      : {host}:{port}")
    print(f"Firmware File    : {bin_path}")
    print(f"Binary Size      : {file_size:,} bytes ({file_size / 1024:.1f} KB)")
    print(f"============================================================")

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5.0)

    try:
        print(f"Connecting to {host}:{port}...")
        sock.connect((host, port))
        print("Connected.")

        # 1. Authentication
        sock.sendall(f"AUTH {password}\n".encode("utf-8"))
        resp = read_line(sock, 5.0)
        if not resp.startswith("OK AUTH"):
            print(f"[ERROR] Authentication failed: {resp}", file=sys.stderr)
            return False
        print("Authentication verified (OK).")

        # 2. Pre-flight check & stream start
        sock.sendall(f"START {file_size}\n".encode("utf-8"))
        resp = read_line(sock, 8.0)
        if not resp.startswith("OK READY"):
            print(f"[ERROR] OTA initiation rejected by target: {resp}", file=sys.stderr)
            return False
        print("Target ready for binary stream.")

        # 3. Stream binary chunks
        print("\nStreaming firmware binary...")
        start_time = time.time()
        sent_bytes = 0

        with open(bin_path, "rb") as f:
            while sent_bytes < file_size:
                chunk = f.read(CHUNK_SIZE)
                if not chunk:
                    break
                sock.sendall(chunk)
                sent_bytes += len(chunk)

                # Progress bar
                pct = (sent_bytes / file_size) * 100.0
                elapsed = time.time() - start_time
                speed_kb = (sent_bytes / 1024.0) / (elapsed if elapsed > 0 else 0.001)
                bar_len = 30
                filled = int(bar_len * (sent_bytes / file_size))
                bar = "=" * filled + "-" * (bar_len - filled)
                print(
                    f"\r[{bar}] {pct:5.1f}% ({sent_bytes / 1024:.1f} / {file_size / 1024:.1f} KB) @ {speed_kb:5.1f} KB/s",
                    end="",
                    flush=True,
                )

        print("\nAll bytes transmitted. Finalizing flash and verifying image...")

        # 4. Await verification and reboot confirmation
        resp = read_line(sock, 15.0)
        if not resp.startswith("OK COMPLETE"):
            print(f"[ERROR] OTA validation failed on target: {resp}", file=sys.stderr)
            return False

        elapsed = time.time() - start_time
        avg_speed = (file_size / 1024.0) / (elapsed if elapsed > 0 else 0.001)
        print(f"============================================================")
        print(f"[SUCCESS] OTA Flash Completed in {elapsed:.2f} s ({avg_speed:.1f} KB/s)")
        print(f"Target Response  : {resp}")
        print(f"Target WT32 is now rebooting into the new firmware slot!")
        print(f"============================================================")
        return True

    except Exception as e:
        print(f"\n[ERROR] Connection error during OTA flash: {e}", file=sys.stderr)
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

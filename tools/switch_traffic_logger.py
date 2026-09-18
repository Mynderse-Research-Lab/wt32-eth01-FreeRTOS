#!/usr/bin/env python3
"""Log traffic seen on a selected NIC (console + optional files).

Works on Windows/Linux/macOS with Scapy. On Windows, install Npcap and run an
elevated terminal for raw packet capture.

Important: on an unmanaged switch, this host only sees broadcast/multicast and
unicast frames destined to this NIC. It will NOT see all unicast between other
ports without a TAP/mirror path.
"""

from __future__ import annotations

import argparse
import datetime as dt
import sys
from pathlib import Path
from typing import TextIO

try:
    from scapy.all import Ether, PcapWriter, get_if_list, sniff  # type: ignore
except Exception as exc:  # pragma: no cover - import guard for missing deps
    print("ERROR: Scapy is required. Install with: py -3 -m pip install scapy")
    print(f"Import failure: {exc}")
    raise SystemExit(2)


CELLNET_ETHERTYPE = 0x88B5


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Capture and log packets visible on one NIC."
    )
    p.add_argument("--list-ifaces", action="store_true", help="List capture interfaces and exit")
    p.add_argument("--iface", help="Interface name (required unless --list-ifaces)")
    p.add_argument("--bpf", default="", help='BPF filter, e.g. "ether proto 0x88b5"')
    p.add_argument("--count", type=int, default=0, help="Stop after N packets (0 = infinite)")
    p.add_argument("--pcap", default="", help="Optional pcap output path")
    p.add_argument("--log", default="", help="Optional text log output path")
    p.add_argument("--hex", action="store_true", help="Print packet bytes as hex")
    p.add_argument("--hex-bytes", type=int, default=96, help="Max bytes shown when --hex is set")
    p.add_argument(
        "--no-promisc",
        action="store_true",
        help="Disable promiscuous mode (default is promiscuous)",
    )
    return p.parse_args()


def to_hex(data: bytes, width: int = 16) -> str:
    chunks: list[str] = []
    for i in range(0, len(data), width):
        chunk = data[i : i + width]
        chunks.append(" ".join(f"{b:02X}" for b in chunk))
    return "\n".join(chunks)


class TrafficLogger:
    def __init__(self, text_log: TextIO | None, pcap: PcapWriter | None, dump_hex: bool, hex_bytes: int):
        self.text_log = text_log
        self.pcap = pcap
        self.dump_hex = dump_hex
        self.hex_bytes = max(1, hex_bytes)
        self.seq = 0

    def _write_line(self, line: str) -> None:
        print(line)
        if self.text_log is not None:
            self.text_log.write(line + "\n")
            self.text_log.flush()

    def _cellnet_suffix(self, frame: bytes) -> str:
        if len(frame) < 22:
            return ""
        version = frame[14]
        msg_type = frame[15]
        sender = frame[16]
        seq = frame[17]
        ts_low = int.from_bytes(frame[18:22], "little", signed=False)
        return (
            f" cellnet[v={version} msg=0x{msg_type:02X} sender=0x{sender:02X} "
            f"seq={seq} ts={ts_low}]"
        )

    def on_packet(self, pkt) -> None:  # scapy packet type is dynamic
        if Ether not in pkt:
            return
        self.seq += 1
        raw_bytes = bytes(pkt)
        eth = pkt[Ether]
        now = dt.datetime.fromtimestamp(float(pkt.time)).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        etype = int(eth.type)
        suffix = self._cellnet_suffix(raw_bytes) if etype == CELLNET_ETHERTYPE else ""
        line = (
            f"{now} #{self.seq:07d} len={len(raw_bytes):4d} "
            f"{eth.src} -> {eth.dst} type=0x{etype:04X}{suffix}"
        )
        self._write_line(line)

        if self.dump_hex:
            clipped = raw_bytes[: self.hex_bytes]
            self._write_line(to_hex(clipped))
            if len(raw_bytes) > self.hex_bytes:
                self._write_line(f"... ({len(raw_bytes) - self.hex_bytes} bytes more)")

        if self.pcap is not None:
            self.pcap.write(pkt)


def main() -> int:
    args = parse_args()
    if args.list_ifaces:
        print("Available interfaces:")
        for name in get_if_list():
            print(f"  {name}")
        return 0

    if not args.iface:
        print("ERROR: --iface is required (or use --list-ifaces)")
        return 2

    text_log: TextIO | None = None
    if args.log:
        log_path = Path(args.log)
        log_path.parent.mkdir(parents=True, exist_ok=True)
        text_log = log_path.open("a", encoding="utf-8")

    pcap_writer: PcapWriter | None = None
    if args.pcap:
        pcap_path = Path(args.pcap)
        pcap_path.parent.mkdir(parents=True, exist_ok=True)
        pcap_writer = PcapWriter(str(pcap_path), append=True, sync=True)

    logger = TrafficLogger(text_log, pcap_writer, args.hex, args.hex_bytes)

    print(f"Capturing on iface: {args.iface}")
    print(f"BPF filter: {args.bpf or '(none)'}")
    print("Press Ctrl+C to stop.")
    print(
        "Note: unmanaged switch limits visibility; full port-to-port unicast "
        "needs TAP/mirror."
    )

    try:
        sniff(
            iface=args.iface,
            filter=args.bpf or None,
            prn=logger.on_packet,
            store=False,
            promisc=not args.no_promisc,
            count=max(0, int(args.count)),
        )
    except PermissionError:
        print("ERROR: permission denied. Run an elevated terminal/admin shell.")
        return 1
    except KeyboardInterrupt:
        pass
    finally:
        if text_log is not None:
            text_log.close()
        if pcap_writer is not None:
            pcap_writer.close()

    print("Capture stopped.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


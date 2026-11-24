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
import struct
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
CELLNET_MSG_CONVEYOR_SPEED = 0x02
CELLNET_SENDER_CONVEYOR = 0x02


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
    p.add_argument(
        "--conveyor-only",
        action="store_true",
        help="Only log CellNet conveyor-speed packets (msg=0x02)",
    )
    p.add_argument(
        "--conveyor-sender-id",
        type=lambda x: int(x, 0),
        default=CELLNET_SENDER_CONVEYOR,
        help="Expected sender_id for --conveyor-only (default 0x02)",
    )
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
    def __init__(
        self,
        text_log: TextIO | None,
        pcap: PcapWriter | None,
        dump_hex: bool,
        hex_bytes: int,
        conveyor_only: bool,
        conveyor_sender_id: int,
    ):
        self.text_log = text_log
        self.pcap = pcap
        self.dump_hex = dump_hex
        self.hex_bytes = max(1, hex_bytes)
        self.conveyor_only = conveyor_only
        self.conveyor_sender_id = conveyor_sender_id & 0xFF
        self.seq = 0

    def _write_line(self, line: str) -> None:
        print(line)
        if self.text_log is not None:
            self.text_log.write(line + "\n")
            self.text_log.flush()

    def _cellnet_info(self, frame: bytes) -> dict[str, int] | None:
        if len(frame) < 22:
            return None
        return {
            "version": int(frame[14]),
            "msg_type": int(frame[15]),
            "sender": int(frame[16]),
            "seq": int(frame[17]),
            "ts_low": int.from_bytes(frame[18:22], "little", signed=False),
        }

    def _cellnet_suffix(self, info: dict[str, int]) -> str:
        return (
            f" cellnet[v={info['version']} msg=0x{info['msg_type']:02X} "
            f"sender=0x{info['sender']:02X} seq={info['seq']} ts={info['ts_low']}]"
        )

    def _conveyor_suffix(self, frame: bytes) -> str:
        if len(frame) < 34:
            return " conveyor[short]"
        speed_mm_s, displacement_m, raw_encoder_cnt = struct.unpack_from("<ffi", frame, 22)
        return (
            f" conveyor[speed={speed_mm_s:.2f}mm/s disp={displacement_m:.4f}m "
            f"enc={raw_encoder_cnt}]"
        )

    def on_packet(self, pkt) -> None:  # scapy packet type is dynamic
        if Ether not in pkt:
            return
        raw_bytes = bytes(pkt)
        eth = pkt[Ether]
        now = dt.datetime.fromtimestamp(float(pkt.time)).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        etype = int(eth.type)

        cell_info: dict[str, int] | None = None
        if etype == CELLNET_ETHERTYPE:
            cell_info = self._cellnet_info(raw_bytes)

        if self.conveyor_only:
            if cell_info is None:
                return
            if cell_info["msg_type"] != CELLNET_MSG_CONVEYOR_SPEED:
                return
            if cell_info["sender"] != self.conveyor_sender_id:
                return

        self.seq += 1
        suffix = ""
        if cell_info is not None:
            suffix = self._cellnet_suffix(cell_info)
            if cell_info["msg_type"] == CELLNET_MSG_CONVEYOR_SPEED:
                suffix += self._conveyor_suffix(raw_bytes)
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

    bpf_filter = args.bpf
    if args.conveyor_only and not bpf_filter:
        bpf_filter = "ether proto 0x88b5"

    logger = TrafficLogger(
        text_log,
        pcap_writer,
        args.hex,
        args.hex_bytes,
        args.conveyor_only,
        args.conveyor_sender_id,
    )

    print(f"Capturing on iface: {args.iface}")
    print(f"BPF filter: {bpf_filter or '(none)'}")
    if args.conveyor_only:
        print(
            "Mode: conveyor-only "
            f"(CellNet msg=0x{CELLNET_MSG_CONVEYOR_SPEED:02X}, "
            f"sender=0x{args.conveyor_sender_id & 0xFF:02X})"
        )
    print("Press Ctrl+C to stop.")
    print(
        "Note: unmanaged switch limits visibility; full port-to-port unicast "
        "needs TAP/mirror."
    )

    try:
        sniff(
            iface=args.iface,
            filter=bpf_filter or None,
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


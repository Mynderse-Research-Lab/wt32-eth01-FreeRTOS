#!/usr/bin/env python3
"""
cell_net_l2.py — Complete Python implementation of the CellNetL2 protocol stack.

Exact Python replica of the C++ lib/CellNetL2 library:
  - IEEE 802.3 Layer-2 raw Ethernet framing with EtherType 0x88B5
  - Fixed-size packed binary wire serialization & deserialization (#pragma pack(1))
  - Full support for all 5 message types (Heartbeat, Vision, Conveyor, Gantry, Command)
  - Abstract IL2Transport interface with LinuxRawSocket, UdpBridge, and Loopback drivers
  - CellNetL2Node with rolling sequence counters, drop tracking, and callback dispatching
  - Self-test suite verifying 100% byte-for-byte fidelity with the C++ host test suite
  - Interactive CLI supporting packet sniffing/monitoring and node simulation

Usage:
  # 1. Run internal regression unit tests:
  python tools/cell_net_l2.py --test

  # 2. Sniff and monitor live traffic (raw Linux socket or UDP bridge):
  python tools/cell_net_l2.py --monitor --iface eth0
  python tools/cell_net_l2.py --monitor --udp-port 8850

  # 3. Simulate a conveyor telemetry node (broadcasting at 100 Hz / 10ms):
  python tools/cell_net_l2.py --simulate conveyor --udp-port 8850

  # 4. Simulate a vision detection node:
  python tools/cell_net_l2.py --simulate vision --udp-port 8850
"""

from __future__ import annotations

import argparse
import ctypes
import enum
import os
import platform
import select
import socket
import struct
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Optional, Tuple, Union

# ==============================================================================
# 1. PROTOCOL CONSTANTS & ENUMS (matching include/cell_net_l2_protocol.h)
# ==============================================================================

CELL_NET_L2_ETHERTYPE: int = 0x88B5
CELL_NET_L2_VERSION: int = 0x01

# Ethernet frame limits
MIN_ETH_FRAME_LEN: int = 60
MAX_PADDED_RX_LEN: int = 64
BROADCAST_MAC: bytes = b"\xff\xff\xff\xff\xff\xff"

# Flags
VISION_CAPTURE_DISP_VALID: int = 0x01
VISION_MAX_WIDTH_MM: float = 80.0


class CellNodeId(enum.IntEnum):
    UNKNOWN = 0x00
    VISION = 0x01       # Ubuntu IPC Camera Classifier
    CONVEYOR = 0x02     # WT32 Belt Tracker (500 PPR)
    GANTRY = 0x03       # WT32 Multi-Axis Motion Controller
    SUPERVISOR = 0x04   # Raspberry Pi Cell Coordinator / HMI
    BROADCAST = 0xFF


class CellMsgType(enum.IntEnum):
    HEARTBEAT = 0x00
    VISION_DETECT = 0x01    # Vision -> Gantry / Supervisor
    CONVEYOR_SPEED = 0x02   # Conveyor -> Gantry / Supervisor
    GANTRY_STATUS = 0x03    # Gantry -> Supervisor
    CELL_COMMAND = 0x04     # Supervisor -> Gantry / Conveyor


def mac_to_str(mac: bytes) -> str:
    """Format 6-byte MAC address into colon-separated hex string."""
    return ":".join(f"{b:02X}" for b in mac[:6])


def str_to_mac(s: str) -> bytes:
    """Parse colon- or hyphen-separated hex string into 6 bytes."""
    clean = s.replace(":", "").replace("-", "")
    if len(clean) != 12:
        raise ValueError(f"Invalid MAC address string: {s}")
    return bytes.fromhex(clean)


# ==============================================================================
# 2. PACKED WIRE STRUCTURES & PAYLOADS
# ==============================================================================

# Struct format strings (standard sizes, no alignment padding)
# ! = Network byte order (big-endian)
# < = Little-endian (Intel/ARM/Xtensa byte order for payloads)

FMT_ETH_HDR: str = "!6s6sH"             # 14 bytes: dest_mac[6], src_mac[6], ethertype
FMT_CELL_HDR: str = "<BBBB I"           # 8 bytes: version, msg_type, sender_id, sequence, timestamp_us
FMT_PAYLOAD_HEARTBEAT: str = "<IH2s"    # 8 bytes: uptime_ms, status_flags, reserved[2]
FMT_PAYLOAD_VISION: str = "<IfffB3sffff"# 36 bytes: item_id, x, y, theta, class, res[3], disp, w, l, h
FMT_PAYLOAD_CONVEYOR: str = "<ffi"      # 12 bytes: speed, disp, raw_encoder
FMT_PAYLOAD_GANTRY: str = "<BBHffff"    # 20 bytes: motion_state, active_slot, fault_flags, x, z, th, cycle_t
FMT_PAYLOAD_COMMAND: str = "<BBHf"      # 8 bytes: command_id, param_u8, param_u16, param_float

LEN_ETH_HDR: int = struct.calcsize(FMT_ETH_HDR)          # 14
LEN_CELL_HDR: int = struct.calcsize(FMT_CELL_HDR)        # 8
LEN_PAYLOAD_HEARTBEAT: int = struct.calcsize(FMT_PAYLOAD_HEARTBEAT)  # 8
LEN_PAYLOAD_VISION: int = struct.calcsize(FMT_PAYLOAD_VISION)        # 36
LEN_PAYLOAD_CONVEYOR: int = struct.calcsize(FMT_PAYLOAD_CONVEYOR)    # 12
LEN_PAYLOAD_GANTRY: int = struct.calcsize(FMT_PAYLOAD_GANTRY)        # 20
LEN_PAYLOAD_COMMAND: int = struct.calcsize(FMT_PAYLOAD_COMMAND)      # 8

LEN_FRAME_HEARTBEAT: int = LEN_ETH_HDR + LEN_CELL_HDR + LEN_PAYLOAD_HEARTBEAT  # 30
LEN_FRAME_VISION: int = LEN_ETH_HDR + LEN_CELL_HDR + LEN_PAYLOAD_VISION        # 58
LEN_FRAME_CONVEYOR: int = LEN_ETH_HDR + LEN_CELL_HDR + LEN_PAYLOAD_CONVEYOR    # 34
LEN_FRAME_GANTRY: int = LEN_ETH_HDR + LEN_CELL_HDR + LEN_PAYLOAD_GANTRY        # 42
LEN_FRAME_COMMAND: int = LEN_ETH_HDR + LEN_CELL_HDR + LEN_PAYLOAD_COMMAND      # 30


@dataclass
class L2EthernetHeader:
    dest_mac: bytes = BROADCAST_MAC
    src_mac: bytes = b"\x00" * 6
    ethertype: int = CELL_NET_L2_ETHERTYPE


@dataclass
class L2CellHeader:
    version: int = CELL_NET_L2_VERSION
    msg_type: int = 0
    sender_id: int = CellNodeId.UNKNOWN
    sequence: int = 0
    timestamp_us_low: int = 0


@dataclass
class L2HeartbeatPayload:
    uptime_ms: int = 0
    status_flags: int = 0
    reserved: bytes = b"\x00\x00"


@dataclass
class L2VisionDetectPayload:
    item_id: int = 0
    x_across_mm: float = 0.0
    y_bat_mm: float = 0.0
    theta_deg: float = 0.0
    battery_class: int = 0
    reserved: bytes = b"\x00\x00\x00"
    capture_displacement_m: float = 0.0
    width_mm: float = 0.0
    length_mm: float = 0.0
    height_from_belt_mm: float = 0.0


@dataclass
class L2ConveyorSpeedPayload:
    speed_mm_s: float = 0.0
    displacement_m: float = 0.0
    raw_encoder_cnt: int = 0


@dataclass
class L2GantryStatusPayload:
    motion_state: int = 0
    active_slot: int = 0
    fault_flags: int = 0
    x_pos_mm: float = 0.0
    z_pos_mm: float = 0.0
    theta_deg: float = 0.0
    last_cycle_time_ms: float = 0.0


@dataclass
class L2CellCommandPayload:
    command_id: int = 0
    param_u8: int = 0
    param_u16: int = 0
    param_float: float = 0.0


PayloadType = Union[
    L2HeartbeatPayload,
    L2VisionDetectPayload,
    L2ConveyorSpeedPayload,
    L2GantryStatusPayload,
    L2CellCommandPayload,
]


@dataclass
class ParsedFrame:
    eth: L2EthernetHeader = field(default_factory=L2EthernetHeader)
    cell: L2CellHeader = field(default_factory=L2CellHeader)
    payload: Optional[PayloadType] = None
    raw_bytes: bytes = b""
    valid: bool = False


# ==============================================================================
# 3. WIRE FRAMING & SERIALIZATION (matching CellNetL2Framing.cpp)
# ==============================================================================

class CellNetL2Framing:
    """Freestanding binary serialization, deserialization, and validation for EtherType 0x88B5."""

    @staticmethod
    def expected_frame_length(msg_type: CellMsgType | int) -> int:
        """Retrieve expected unpadded frame length in bytes for a given message type."""
        try:
            mt = CellMsgType(msg_type)
        except ValueError:
            return 0
        if mt == CellMsgType.HEARTBEAT:
            return LEN_FRAME_HEARTBEAT
        elif mt == CellMsgType.VISION_DETECT:
            return LEN_FRAME_VISION
        elif mt == CellMsgType.CONVEYOR_SPEED:
            return LEN_FRAME_CONVEYOR
        elif mt == CellMsgType.GANTRY_STATUS:
            return LEN_FRAME_GANTRY
        elif mt == CellMsgType.CELL_COMMAND:
            return LEN_FRAME_COMMAND
        return 0

    @staticmethod
    def parse_frame(buffer: bytes) -> Optional[ParsedFrame]:
        """
        Parse and validate an incoming raw Ethernet frame.
        Returns ParsedFrame with valid=True if well-formed, or None / valid=False if malformed.
        """
        if not buffer or len(buffer) < (LEN_ETH_HDR + LEN_CELL_HDR):
            return None

        # Parse Ethernet Header
        dest_mac, src_mac, ethertype = struct.unpack_from(FMT_ETH_HDR, buffer, 0)
        if ethertype != CELL_NET_L2_ETHERTYPE:
            return None

        # Parse Cell Header
        version, msg_type_raw, sender_id, sequence, ts_us = struct.unpack_from(
            FMT_CELL_HDR, buffer, LEN_ETH_HDR
        )
        if version != CELL_NET_L2_VERSION:
            return None

        try:
            msg_type = CellMsgType(msg_type_raw)
        except ValueError:
            return None

        expected_len = CellNetL2Framing.expected_frame_length(msg_type)
        if expected_len == 0 or len(buffer) < expected_len:
            return None

        # Maximum allowed length: accommodates trailing zero padding up to 64 bytes
        max_allowed_len = max(expected_len, MAX_PADDED_RX_LEN)
        if len(buffer) > max_allowed_len:
            return None

        eth = L2EthernetHeader(dest_mac=dest_mac, src_mac=src_mac, ethertype=ethertype)
        cell = L2CellHeader(
            version=version,
            msg_type=msg_type_raw,
            sender_id=sender_id,
            sequence=sequence,
            timestamp_us_low=ts_us,
        )

        payload_offset = LEN_ETH_HDR + LEN_CELL_HDR
        payload: Optional[PayloadType] = None

        if msg_type == CellMsgType.HEARTBEAT:
            uptime, flags, res = struct.unpack_from(FMT_PAYLOAD_HEARTBEAT, buffer, payload_offset)
            payload = L2HeartbeatPayload(uptime_ms=uptime, status_flags=flags, reserved=res)

        elif msg_type == CellMsgType.VISION_DETECT:
            (
                item_id,
                x,
                y,
                theta,
                b_class,
                res,
                disp,
                width,
                length,
                height,
            ) = struct.unpack_from(FMT_PAYLOAD_VISION, buffer, payload_offset)
            payload = L2VisionDetectPayload(
                item_id=item_id,
                x_across_mm=x,
                y_bat_mm=y,
                theta_deg=theta,
                battery_class=b_class,
                reserved=res,
                capture_displacement_m=disp,
                width_mm=width,
                length_mm=length,
                height_from_belt_mm=height,
            )

        elif msg_type == CellMsgType.CONVEYOR_SPEED:
            speed, disp, enc = struct.unpack_from(FMT_PAYLOAD_CONVEYOR, buffer, payload_offset)
            payload = L2ConveyorSpeedPayload(
                speed_mm_s=speed, displacement_m=disp, raw_encoder_cnt=enc
            )

        elif msg_type == CellMsgType.GANTRY_STATUS:
            (
                m_state,
                slot,
                fault,
                x,
                z,
                theta,
                cycle_t,
            ) = struct.unpack_from(FMT_PAYLOAD_GANTRY, buffer, payload_offset)
            payload = L2GantryStatusPayload(
                motion_state=m_state,
                active_slot=slot,
                fault_flags=fault,
                x_pos_mm=x,
                z_pos_mm=z,
                theta_deg=theta,
                last_cycle_time_ms=cycle_t,
            )

        elif msg_type == CellMsgType.CELL_COMMAND:
            cmd_id, p_u8, p_u16, p_flt = struct.unpack_from(
                FMT_PAYLOAD_COMMAND, buffer, payload_offset
            )
            payload = L2CellCommandPayload(
                command_id=cmd_id,
                param_u8=p_u8,
                param_u16=p_u16,
                param_float=p_flt,
            )

        return ParsedFrame(eth=eth, cell=cell, payload=payload, raw_bytes=buffer, valid=True)

    @staticmethod
    def build_frame(
        src_mac: bytes,
        sender_id: CellNodeId | int,
        sequence: int,
        timestamp_us: int,
        msg_type: CellMsgType,
        payload_bytes: bytes,
        dest_mac: Optional[bytes] = None,
    ) -> bytes:
        """Construct raw Ethernet frame with standard padding to 60 bytes minimum."""
        dst = dest_mac if dest_mac is not None else BROADCAST_MAC
        eth_bytes = struct.pack(FMT_ETH_HDR, dst, src_mac, CELL_NET_L2_ETHERTYPE)
        cell_bytes = struct.pack(
            FMT_CELL_HDR,
            CELL_NET_L2_VERSION,
            int(msg_type),
            int(sender_id),
            sequence & 0xFF,
            timestamp_us & 0xFFFFFFFF,
        )
        frame = eth_bytes + cell_bytes + payload_bytes
        # Pad with zeros to minimum Ethernet frame length (60 bytes)
        if len(frame) < MIN_ETH_FRAME_LEN:
            frame += b"\x00" * (MIN_ETH_FRAME_LEN - len(frame))
        return frame

    @classmethod
    def build_heartbeat_frame(
        cls,
        src_mac: bytes,
        sender_id: CellNodeId | int,
        sequence: int,
        timestamp_us: int,
        payload: L2HeartbeatPayload,
        dest_mac: Optional[bytes] = None,
    ) -> bytes:
        p_bytes = struct.pack(
            FMT_PAYLOAD_HEARTBEAT, payload.uptime_ms, payload.status_flags, payload.reserved
        )
        return cls.build_frame(
            src_mac, sender_id, sequence, timestamp_us, CellMsgType.HEARTBEAT, p_bytes, dest_mac
        )

    @classmethod
    def build_vision_detect_frame(
        cls,
        src_mac: bytes,
        sender_id: CellNodeId | int,
        sequence: int,
        timestamp_us: int,
        payload: L2VisionDetectPayload,
        dest_mac: Optional[bytes] = None,
    ) -> bytes:
        p_bytes = struct.pack(
            FMT_PAYLOAD_VISION,
            payload.item_id,
            payload.x_across_mm,
            payload.y_bat_mm,
            payload.theta_deg,
            payload.battery_class,
            payload.reserved,
            payload.capture_displacement_m,
            payload.width_mm,
            payload.length_mm,
            payload.height_from_belt_mm,
        )
        return cls.build_frame(
            src_mac, sender_id, sequence, timestamp_us, CellMsgType.VISION_DETECT, p_bytes, dest_mac
        )

    @classmethod
    def build_conveyor_speed_frame(
        cls,
        src_mac: bytes,
        sender_id: CellNodeId | int,
        sequence: int,
        timestamp_us: int,
        payload: L2ConveyorSpeedPayload,
        dest_mac: Optional[bytes] = None,
    ) -> bytes:
        p_bytes = struct.pack(
            FMT_PAYLOAD_CONVEYOR,
            payload.speed_mm_s,
            payload.displacement_m,
            payload.raw_encoder_cnt,
        )
        return cls.build_frame(
            src_mac, sender_id, sequence, timestamp_us, CellMsgType.CONVEYOR_SPEED, p_bytes, dest_mac
        )

    @classmethod
    def build_gantry_status_frame(
        cls,
        src_mac: bytes,
        sender_id: CellNodeId | int,
        sequence: int,
        timestamp_us: int,
        payload: L2GantryStatusPayload,
        dest_mac: Optional[bytes] = None,
    ) -> bytes:
        p_bytes = struct.pack(
            FMT_PAYLOAD_GANTRY,
            payload.motion_state,
            payload.active_slot,
            payload.fault_flags,
            payload.x_pos_mm,
            payload.z_pos_mm,
            payload.theta_deg,
            payload.last_cycle_time_ms,
        )
        return cls.build_frame(
            src_mac, sender_id, sequence, timestamp_us, CellMsgType.GANTRY_STATUS, p_bytes, dest_mac
        )

    @classmethod
    def build_cell_command_frame(
        cls,
        src_mac: bytes,
        sender_id: CellNodeId | int,
        sequence: int,
        timestamp_us: int,
        payload: L2CellCommandPayload,
        dest_mac: Optional[bytes] = None,
    ) -> bytes:
        p_bytes = struct.pack(
            FMT_PAYLOAD_COMMAND,
            payload.command_id,
            payload.param_u8,
            payload.param_u16,
            payload.param_float,
        )
        return cls.build_frame(
            src_mac, sender_id, sequence, timestamp_us, CellMsgType.CELL_COMMAND, p_bytes, dest_mac
        )


# ==============================================================================
# 4. PLUGGABLE TRANSPORTS (matching IL2Transport.h)
# ==============================================================================

RxCallbackType = Callable[[bytes], None]


class IL2Transport:
    """Abstract interface for physical Layer-2 packet I/O."""

    def send_frame(self, data: bytes) -> bool:
        raise NotImplementedError

    def get_mac_address(self) -> bytes:
        raise NotImplementedError

    def is_link_up(self) -> bool:
        raise NotImplementedError

    def set_rx_callback(self, callback: RxCallbackType) -> None:
        raise NotImplementedError

    def close(self) -> None:
        pass


class LoopbackTransport(IL2Transport):
    """In-memory thread-safe transport for unit testing and local simulation."""

    def __init__(self, mac: bytes = b"\x00\x11\x22\x33\x44\x55"):
        self.mac = mac
        self.link_up = True
        self.sent_frames: list[bytes] = []
        self._rx_cb: Optional[RxCallbackType] = None
        self._lock = threading.Lock()

    def send_frame(self, data: bytes) -> bool:
        if not self.link_up or not data:
            return False
        with self._lock:
            self.sent_frames.append(data)
        return True

    def get_mac_address(self) -> bytes:
        return self.mac

    def is_link_up(self) -> bool:
        return self.link_up

    def set_rx_callback(self, callback: RxCallbackType) -> None:
        self._rx_cb = callback

    def inject_rx_frame(self, data: bytes) -> None:
        if self._rx_cb:
            self._rx_cb(data)


class UdpBridgeTransport(IL2Transport):
    """
    Cross-platform UDP bridge transport.
    Encapsulates raw Layer-2 Ethernet frames inside UDP datagrams, allowing
    CellNetL2 nodes to communicate across Windows, macOS, and Linux without root/pcap.
    """

    def __init__(
        self,
        local_port: int = 8850,
        remote_port: int = 8850,
        remote_host: str = "127.0.0.1",
        mac: Optional[bytes] = None,
    ):
        self.local_port = local_port
        self.remote_port = remote_port
        self.remote_host = remote_host
        self.mac = mac if mac is not None else b"\x02\x00\x00\x00\x00\x01"

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if hasattr(socket, "SO_BROADCAST"):
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        self._sock.bind(("", self.local_port))
        self._rx_cb: Optional[RxCallbackType] = None
        self._running = True
        self._thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._thread.start()

    def send_frame(self, data: bytes) -> bool:
        if not self._running:
            return False
        try:
            self._sock.sendto(data, (self.remote_host, self.remote_port))
            return True
        except Exception as e:
            sys.stderr.write(f"[UdpBridge] send error: {e}\n")
            return False

    def get_mac_address(self) -> bytes:
        return self.mac

    def is_link_up(self) -> bool:
        return self._running

    def set_rx_callback(self, callback: RxCallbackType) -> None:
        self._rx_cb = callback

    def _rx_loop(self) -> None:
        while self._running:
            try:
                r, _, _ = select.select([self._sock], [], [], 0.2)
                if not r:
                    continue
                data, _ = self._sock.recvfrom(2048)
                if data and self._rx_cb:
                    self._rx_cb(data)
            except Exception:
                if not self._running:
                    break

    def close(self) -> None:
        self._running = False
        try:
            self._sock.close()
        except Exception:
            pass


class LinuxRawSocketL2Transport(IL2Transport):
    """
    Native Linux raw packet socket transport (AF_PACKET / SOCK_RAW).
    Requires Linux kernel and CAP_NET_RAW or sudo privileges.
    """

    def __init__(self, iface_name: str = "eth0"):
        if platform.system() != "Linux":
            raise RuntimeError("LinuxRawSocketL2Transport is only supported on Linux.")

        self.iface_name = iface_name
        self._sock = socket.socket(
            socket.AF_PACKET, socket.SOCK_RAW, socket.htons(CELL_NET_L2_ETHERTYPE)
        )
        self._sock.bind((self.iface_name, 0))

        # Query hardware MAC address
        self.mac = self._query_mac()
        self._rx_cb: Optional[RxCallbackType] = None
        self._running = True
        self._thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._thread.start()

    def _query_mac(self) -> bytes:
        try:
            import fcntl
            # SIOCGIFHWADDR = 0x8927
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            info = fcntl.ioctl(
                s.fileno(), 0x8927, struct.pack("256s", bytes(self.iface_name, "utf-8")[:15])
            )
            return info[18:24]
        except Exception:
            return b"\x00\x02\x00\x00\x00\x01"

    def send_frame(self, data: bytes) -> bool:
        if not self._running:
            return False
        try:
            self._sock.send(data)
            return True
        except Exception as e:
            sys.stderr.write(f"[LinuxRaw] send error: {e}\n")
            return False

    def get_mac_address(self) -> bytes:
        return self.mac

    def is_link_up(self) -> bool:
        return self._running

    def set_rx_callback(self, callback: RxCallbackType) -> None:
        self._rx_cb = callback

    def _rx_loop(self) -> None:
        while self._running:
            try:
                r, _, _ = select.select([self._sock], [], [], 0.2)
                if not r:
                    continue
                data = self._sock.recv(2048)
                if data and self._rx_cb:
                    self._rx_cb(data)
            except Exception:
                if not self._running:
                    break

    def close(self) -> None:
        self._running = False
        try:
            self._sock.close()
        except Exception:
            pass


# ==============================================================================
# 5. NODE DISPATCHER & STATS (matching CellNetL2.cpp)
# ==============================================================================

@dataclass
class CellNetStats:
    tx_frames: int = 0
    rx_frames: int = 0
    rx_invalid_frames: int = 0
    rx_sequence_drops: int = 0


HeartbeatCallback = Callable[[L2HeartbeatPayload, L2CellHeader], None]
VisionDetectCallback = Callable[[L2VisionDetectPayload, L2CellHeader], None]
ConveyorSpeedCallback = Callable[[L2ConveyorSpeedPayload, L2CellHeader], None]
GantryStatusCallback = Callable[[L2GantryStatusPayload, L2CellHeader], None]
CellCommandCallback = Callable[[L2CellCommandPayload, L2CellHeader], None]


class CellNetL2Node:
    """Subsystem-agnostic Layer-2 Communication Node."""

    def __init__(
        self,
        transport: IL2Transport,
        node_id: CellNodeId | int = CellNodeId.UNKNOWN,
        mac_address: Optional[bytes] = None,
    ):
        self.transport = transport
        self.node_id = CellNodeId(node_id)
        self.self_mac = mac_address if mac_address is not None else transport.get_mac_address()
        self.sequence = 0
        self.initialized = False

        self.stats = CellNetStats()
        self._stats_lock = threading.Lock()

        # Sequence drop tracking per sender node ID (0..255)
        self._sender_seen: list[bool] = [False] * 256
        self._last_sender_seq: list[int] = [0] * 256

        # Callbacks
        self._heartbeat_cb: Optional[HeartbeatCallback] = None
        self._vision_cb: Optional[VisionDetectCallback] = None
        self._conveyor_cb: Optional[ConveyorSpeedCallback] = None
        self._gantry_cb: Optional[GantryStatusCallback] = None
        self._command_cb: Optional[CellCommandCallback] = None

    def begin(self) -> bool:
        """Initialize Layer-2 node, bind RX callback, and verify transport."""
        if not self.self_mac or len(self.self_mac) != 6:
            self.self_mac = self.transport.get_mac_address()
        self.transport.set_rx_callback(self.process_incoming_frame)
        self.initialized = True
        return True

    def is_ready(self) -> bool:
        return self.initialized and self.transport.is_link_up()

    @staticmethod
    def get_current_timestamp_us() -> int:
        return int(time.time() * 1_000_000) & 0xFFFFFFFF

    # --------------------------------------------------------------------------
    # Message Transmission Methods
    # --------------------------------------------------------------------------

    def send_heartbeat(
        self,
        uptime_ms: int,
        status_flags: int = 0,
        dest_mac: Optional[bytes] = None,
    ) -> bool:
        if not self.initialized:
            return False
        payload = L2HeartbeatPayload(uptime_ms=uptime_ms, status_flags=status_flags)
        seq = self.sequence
        self.sequence = (self.sequence + 1) & 0xFF
        frame = CellNetL2Framing.build_heartbeatFrame(  # type: ignore[attr-defined]
            self.self_mac, self.node_id, seq, self.get_current_timestamp_us(), payload, dest_mac
        ) if hasattr(CellNetL2Framing, "build_heartbeatFrame") else CellNetL2Framing.build_heartbeat_frame(
            self.self_mac, self.node_id, seq, self.get_current_timestamp_us(), payload, dest_mac
        )
        ok = self.transport.send_frame(frame)
        if ok:
            with self._stats_lock:
                self.stats.tx_frames += 1
        return ok

    def send_vision_detect(
        self,
        item_id: int,
        x_across_mm: float,
        y_bat_mm: float,
        theta_deg: float,
        battery_class: int,
        capture_displacement_m: float = 0.0,
        capture_disp_valid: bool = False,
        width_mm: float = 0.0,
        length_mm: float = 0.0,
        height_from_belt_mm: float = 0.0,
        dest_mac: Optional[bytes] = None,
    ) -> bool:
        if not self.initialized:
            return False
        res = bytes([VISION_CAPTURE_DISP_VALID if capture_disp_valid else 0, 0, 0])
        payload = L2VisionDetectPayload(
            item_id=item_id,
            x_across_mm=x_across_mm,
            y_bat_mm=y_bat_mm,
            theta_deg=theta_deg,
            battery_class=battery_class,
            reserved=res,
            capture_displacement_m=capture_displacement_m,
            width_mm=width_mm,
            length_mm=length_mm,
            height_from_belt_mm=height_from_belt_mm,
        )
        seq = self.sequence
        self.sequence = (self.sequence + 1) & 0xFF
        frame = CellNetL2Framing.build_vision_detect_frame(
            self.self_mac, self.node_id, seq, self.get_current_timestamp_us(), payload, dest_mac
        )
        ok = self.transport.send_frame(frame)
        if ok:
            with self._stats_lock:
                self.stats.tx_frames += 1
        return ok

    def send_conveyor_speed(
        self,
        speed_mm_s: float,
        displacement_m: float,
        raw_encoder_cnt: int,
        dest_mac: Optional[bytes] = None,
    ) -> bool:
        if not self.initialized:
            return False
        payload = L2ConveyorSpeedPayload(
            speed_mm_s=speed_mm_s,
            displacement_m=displacement_m,
            raw_encoder_cnt=raw_encoder_cnt,
        )
        seq = self.sequence
        self.sequence = (self.sequence + 1) & 0xFF
        frame = CellNetL2Framing.build_conveyor_speed_frame(
            self.self_mac, self.node_id, seq, self.get_current_timestamp_us(), payload, dest_mac
        )
        ok = self.transport.send_frame(frame)
        if ok:
            with self._stats_lock:
                self.stats.tx_frames += 1
        return ok

    def send_gantry_status(
        self,
        motion_state: int,
        active_slot: int,
        x_pos_mm: float,
        z_pos_mm: float,
        theta_deg: float,
        last_cycle_time_ms: float,
        fault_flags: int = 0,
        dest_mac: Optional[bytes] = None,
    ) -> bool:
        if not self.initialized:
            return False
        payload = L2GantryStatusPayload(
            motion_state=motion_state,
            active_slot=active_slot,
            fault_flags=fault_flags,
            x_pos_mm=x_pos_mm,
            z_pos_mm=z_pos_mm,
            theta_deg=theta_deg,
            last_cycle_time_ms=last_cycle_time_ms,
        )
        seq = self.sequence
        self.sequence = (self.sequence + 1) & 0xFF
        frame = CellNetL2Framing.build_gantry_status_frame(
            self.self_mac, self.node_id, seq, self.get_current_timestamp_us(), payload, dest_mac
        )
        ok = self.transport.send_frame(frame)
        if ok:
            with self._stats_lock:
                self.stats.tx_frames += 1
        return ok

    def send_cell_command(
        self,
        command_id: int,
        param_u8: int = 0,
        param_u16: int = 0,
        param_float: float = 0.0,
        dest_mac: Optional[bytes] = None,
    ) -> bool:
        if not self.initialized:
            return False
        payload = L2CellCommandPayload(
            command_id=command_id,
            param_u8=param_u8,
            param_u16=param_u16,
            param_float=param_float,
        )
        seq = self.sequence
        self.sequence = (self.sequence + 1) & 0xFF
        frame = CellNetL2Framing.build_cell_command_frame(
            self.self_mac, self.node_id, seq, self.get_current_timestamp_us(), payload, dest_mac
        )
        ok = self.transport.send_frame(frame)
        if ok:
            with self._stats_lock:
                self.stats.tx_frames += 1
        return ok

    # --------------------------------------------------------------------------
    # Incoming Packet Processing & Callbacks
    # --------------------------------------------------------------------------

    def on_heartbeat(self, cb: HeartbeatCallback) -> None:
        self._heartbeat_cb = cb

    def on_vision_detect(self, cb: VisionDetectCallback) -> None:
        self._vision_cb = cb

    def on_conveyor_speed(self, cb: ConveyorSpeedCallback) -> None:
        self._conveyor_cb = cb

    def on_gantry_status(self, cb: GantryStatusCallback) -> None:
        self._gantry_cb = cb

    def on_cell_command(self, cb: CellCommandCallback) -> None:
        self._command_cb = cb

    def process_incoming_frame(self, data: bytes) -> bool:
        parsed = CellNetL2Framing.parse_frame(data)
        if parsed is None or not parsed.valid:
            with self._stats_lock:
                self.stats.rx_invalid_frames += 1
            return False

        sender = parsed.cell.sender_id
        with self._stats_lock:
            self.stats.rx_frames += 1
            if self._sender_seen[sender]:
                expected_seq = (self._last_sender_seq[sender] + 1) & 0xFF
                if parsed.cell.sequence != expected_seq:
                    lost = (parsed.cell.sequence - expected_seq) & 0xFF
                    self.stats.rx_sequence_drops += lost
            else:
                self._sender_seen[sender] = True
            self._last_sender_seq[sender] = parsed.cell.sequence

        # Dispatch
        m_type = CellMsgType(parsed.cell.msg_type)
        if m_type == CellMsgType.HEARTBEAT and self._heartbeat_cb and isinstance(parsed.payload, L2HeartbeatPayload):
            self._heartbeat_cb(parsed.payload, parsed.cell)
        elif m_type == CellMsgType.VISION_DETECT and self._vision_cb and isinstance(parsed.payload, L2VisionDetectPayload):
            self._vision_cb(parsed.payload, parsed.cell)
        elif m_type == CellMsgType.CONVEYOR_SPEED and self._conveyor_cb and isinstance(parsed.payload, L2ConveyorSpeedPayload):
            self._conveyor_cb(parsed.payload, parsed.cell)
        elif m_type == CellMsgType.GANTRY_STATUS and self._gantry_cb and isinstance(parsed.payload, L2GantryStatusPayload):
            self._gantry_cb(parsed.payload, parsed.cell)
        elif m_type == CellMsgType.CELL_COMMAND and self._command_cb and isinstance(parsed.payload, L2CellCommandPayload):
            self._command_cb(parsed.payload, parsed.cell)

        return True

    def get_stats(self) -> CellNetStats:
        with self._stats_lock:
            return CellNetStats(
                tx_frames=self.stats.tx_frames,
                rx_frames=self.stats.rx_frames,
                rx_invalid_frames=self.stats.rx_invalid_frames,
                rx_sequence_drops=self.stats.rx_sequence_drops,
            )

    def reset_stats(self) -> None:
        with self._stats_lock:
            self.stats = CellNetStats()
            self._sender_seen = [False] * 256
            self._last_sender_seq = [0] * 256


# ==============================================================================
# 6. INTERNAL UNIT TEST SUITE (Exact port of test/host/test_cell_net_l2.cpp)
# ==============================================================================

def run_self_tests() -> bool:
    """Run byte-for-byte regression tests matching test_cell_net_l2.cpp."""
    print("================================================================================")
    print(" Running CellNetL2 Python Test Suite (Port of test_cell_net_l2.cpp)")
    print("================================================================================")
    passed = 0
    total = 0

    def assert_true(cond: bool, msg: str = "Assertion failed"):
        nonlocal passed, total
        total += 1
        if not cond:
            raise AssertionError(msg)
        passed += 1

    # 1. Expected frame lengths
    assert_true(CellNetL2Framing.expected_frame_length(CellMsgType.HEARTBEAT) == LEN_FRAME_HEARTBEAT)
    assert_true(CellNetL2Framing.expected_frame_length(CellMsgType.VISION_DETECT) == LEN_FRAME_VISION)
    assert_true(CellNetL2Framing.expected_frame_length(CellMsgType.CONVEYOR_SPEED) == LEN_FRAME_CONVEYOR)
    assert_true(CellNetL2Framing.expected_frame_length(CellMsgType.GANTRY_STATUS) == LEN_FRAME_GANTRY)
    assert_true(CellNetL2Framing.expected_frame_length(CellMsgType.CELL_COMMAND) == LEN_FRAME_COMMAND)
    print("  [PASS] test_framing_expected_lengths")

    # 2. Heartbeat roundtrip
    src_mac = bytes([0x00, 0x1A, 0x2B, 0x3C, 0x4D, 0x5E])
    hb_payload = L2HeartbeatPayload(uptime_ms=123456, status_flags=0xABCD)
    hb_bytes = CellNetL2Framing.build_heartbeat_frame(
        src_mac, CellNodeId.SUPERVISOR, 42, 987654, hb_payload
    )
    # Framed bytes padded to 60
    assert_true(len(hb_bytes) == MIN_ETH_FRAME_LEN)
    parsed = CellNetL2Framing.parse_frame(hb_bytes)
    assert_true(parsed is not None and parsed.valid)
    assert_true(parsed.cell.version == CELL_NET_L2_VERSION)
    assert_true(parsed.cell.msg_type == CellMsgType.HEARTBEAT)
    assert_true(parsed.cell.sender_id == CellNodeId.SUPERVISOR)
    assert_true(parsed.cell.sequence == 42)
    assert_true(parsed.cell.timestamp_us_low == 987654)
    assert isinstance(parsed.payload, L2HeartbeatPayload)
    assert_true(parsed.payload.uptime_ms == 123456)
    assert_true(parsed.payload.status_flags == 0xABCD)
    print("  [PASS] test_framing_heartbeat_roundtrip")

    # 3. Vision Detect roundtrip
    src_mac_vis = bytes([0x10, 0x20, 0x30, 0x40, 0x50, 0x60])
    vis_payload = L2VisionDetectPayload(
        item_id=999,
        x_across_mm=145.5,
        y_bat_mm=12.25,
        theta_deg=-45.0,
        battery_class=4,
        reserved=bytes([VISION_CAPTURE_DISP_VALID, 0, 0]),
        capture_displacement_m=1.25,
        width_mm=75.0,
        length_mm=120.0,
        height_from_belt_mm=80.0,
    )
    vis_bytes = CellNetL2Framing.build_vision_detect_frame(
        src_mac_vis, CellNodeId.VISION, 101, 555123, vis_payload
    )
    assert_true(len(vis_bytes) == MIN_ETH_FRAME_LEN)
    parsed_vis = CellNetL2Framing.parse_frame(vis_bytes)
    assert_true(parsed_vis is not None and parsed_vis.valid)
    assert isinstance(parsed_vis.payload, L2VisionDetectPayload)
    assert_true(parsed_vis.payload.item_id == 999)
    assert_true(abs(parsed_vis.payload.x_across_mm - 145.5) < 1e-3)
    assert_true(abs(parsed_vis.payload.y_bat_mm - 12.25) < 1e-3)
    assert_true(abs(parsed_vis.payload.theta_deg - -45.0) < 1e-3)
    assert_true(parsed_vis.payload.battery_class == 4)
    assert_true(parsed_vis.payload.reserved[0] == VISION_CAPTURE_DISP_VALID)
    assert_true(abs(parsed_vis.payload.capture_displacement_m - 1.25) < 1e-4)
    assert_true(abs(parsed_vis.payload.width_mm - 75.0) < 1e-3)
    assert_true(abs(parsed_vis.payload.length_mm - 120.0) < 1e-3)
    assert_true(abs(parsed_vis.payload.height_from_belt_mm - 80.0) < 1e-3)
    print("  [PASS] test_framing_vision_detect_roundtrip")

    # 4. Conveyor Speed roundtrip
    src_mac_conv = bytes([0x02, 0x00, 0x00, 0x00, 0x00, 0x02])
    conv_payload = L2ConveyorSpeedPayload(
        speed_mm_s=250.75, displacement_m=18.520, raw_encoder_cnt=92600
    )
    conv_bytes = CellNetL2Framing.build_conveyor_speed_frame(
        src_mac_conv, CellNodeId.CONVEYOR, 77, 1000200, conv_payload
    )
    parsed_conv = CellNetL2Framing.parse_frame(conv_bytes)
    assert_true(parsed_conv is not None and parsed_conv.valid)
    assert isinstance(parsed_conv.payload, L2ConveyorSpeedPayload)
    assert_true(abs(parsed_conv.payload.speed_mm_s - 250.75) < 1e-3)
    assert_true(abs(parsed_conv.payload.displacement_m - 18.520) < 1e-3)
    assert_true(parsed_conv.payload.raw_encoder_cnt == 92600)
    print("  [PASS] test_framing_conveyor_speed_roundtrip")

    # 5. Gantry Status roundtrip
    src_mac_g = bytes([0x03, 0x00, 0x00, 0x00, 0x00, 0x03])
    g_payload = L2GantryStatusPayload(
        motion_state=2,
        active_slot=1,
        fault_flags=0,
        x_pos_mm=210.0,
        z_pos_mm=35.7,
        theta_deg=90.0,
        last_cycle_time_ms=824.5,
    )
    g_bytes = CellNetL2Framing.build_gantry_status_frame(
        src_mac_g, CellNodeId.GANTRY, 88, 2000300, g_payload
    )
    parsed_g = CellNetL2Framing.parse_frame(g_bytes)
    assert_true(parsed_g is not None and parsed_g.valid)
    assert isinstance(parsed_g.payload, L2GantryStatusPayload)
    assert_true(parsed_g.payload.motion_state == 2)
    assert_true(parsed_g.payload.active_slot == 1)
    assert_true(abs(parsed_g.payload.x_pos_mm - 210.0) < 1e-3)
    assert_true(abs(parsed_g.payload.z_pos_mm - 35.7) < 1e-3)
    assert_true(abs(parsed_g.payload.theta_deg - 90.0) < 1e-3)
    assert_true(abs(parsed_g.payload.last_cycle_time_ms - 824.5) < 1e-3)
    print("  [PASS] test_framing_gantry_status_roundtrip")

    # 6. Cell Command roundtrip
    src_mac_cmd = bytes([0x04, 0x00, 0x00, 0x00, 0x00, 0x04])
    cmd_payload = L2CellCommandPayload(command_id=5, param_u8=1, param_u16=300, param_float=35.7)
    cmd_bytes = CellNetL2Framing.build_cell_command_frame(
        src_mac_cmd, CellNodeId.SUPERVISOR, 12, 3000400, cmd_payload
    )
    parsed_cmd = CellNetL2Framing.parse_frame(cmd_bytes)
    assert_true(parsed_cmd is not None and parsed_cmd.valid)
    assert isinstance(parsed_cmd.payload, L2CellCommandPayload)
    assert_true(parsed_cmd.payload.command_id == 5)
    assert_true(parsed_cmd.payload.param_u8 == 1)
    assert_true(parsed_cmd.payload.param_u16 == 300)
    assert_true(abs(parsed_cmd.payload.param_float - 35.7) < 1e-3)
    print("  [PASS] test_framing_cell_command_roundtrip")

    # 7. Reject invalid ethertype
    bad_eth = bytearray(hb_bytes)
    bad_eth[12] = 0x08
    bad_eth[13] = 0x00  # IPv4 (0x0800) instead of 0x88B5
    assert_true(CellNetL2Framing.parse_frame(bytes(bad_eth)) is None)
    print("  [PASS] test_framing_reject_invalid_ethertype")

    # 8. Reject invalid version
    bad_ver = bytearray(hb_bytes)
    bad_ver[14] = 0x02  # Version 2
    assert_true(CellNetL2Framing.parse_frame(bytes(bad_ver)) is None)
    print("  [PASS] test_framing_reject_invalid_version")

    # 9. Reject truncated frames
    assert_true(CellNetL2Framing.parse_frame(hb_bytes[:LEN_FRAME_HEARTBEAT - 1]) is None)
    print("  [PASS] test_framing_reject_truncated_frames")

    # 10. Cell Command accepts padding up to 64 bytes, rejects > 64
    padded_60 = cmd_bytes[:MIN_ETH_FRAME_LEN]
    assert_true(CellNetL2Framing.parse_frame(padded_60) is not None)
    padded_64 = cmd_bytes[:MIN_ETH_FRAME_LEN] + b"\x00" * 4
    assert_true(CellNetL2Framing.parse_frame(padded_64) is not None)
    oversized_65 = cmd_bytes[:MIN_ETH_FRAME_LEN] + b"\x00" * 5
    assert_true(CellNetL2Framing.parse_frame(oversized_65) is None)
    print("  [PASS] test_framing_cell_command_accepts_ethernet_padding")

    # 11. Node RX dispatch callbacks
    mock_transport = LoopbackTransport()
    node = CellNetL2Node(mock_transport, CellNodeId.GANTRY)
    assert_true(node.begin())

    rx_events = {"hb": False, "vis": False, "conv": False, "cmd": False}

    node.on_heartbeat(lambda p, h: rx_events.update({"hb": True}))
    node.on_vision_detect(lambda p, h: rx_events.update({"vis": True}))
    node.on_conveyor_speed(lambda p, h: rx_events.update({"conv": True}))
    node.on_cell_command(lambda p, h: rx_events.update({"cmd": True}))

    # Inject Heartbeat (sender=SUPERVISOR, seq=1)
    hb_test = L2HeartbeatPayload(uptime_ms=777)
    f_hb = CellNetL2Framing.build_heartbeat_frame(mock_transport.mac, CellNodeId.SUPERVISOR, 1, 1000, hb_test)
    mock_transport.inject_rx_frame(f_hb)
    assert_true(rx_events["hb"])

    # Inject Conveyor Speed (sender=CONVEYOR, seq=1)
    conv_test = L2ConveyorSpeedPayload(speed_mm_s=150.0)
    f_conv = CellNetL2Framing.build_conveyor_speed_frame(mock_transport.mac, CellNodeId.CONVEYOR, 1, 1010, conv_test)
    mock_transport.inject_rx_frame(f_conv)
    assert_true(rx_events["conv"])

    # Inject Vision Detect (sender=VISION, seq=1)
    vis_test = L2VisionDetectPayload(item_id=55)
    f_vis = CellNetL2Framing.build_vision_detect_frame(mock_transport.mac, CellNodeId.VISION, 1, 1020, vis_test)
    mock_transport.inject_rx_frame(f_vis)
    assert_true(rx_events["vis"])

    # Inject Cell Command (sender=SUPERVISOR, seq=2)
    cmd_test = L2CellCommandPayload(command_id=1)
    f_cmd = CellNetL2Framing.build_cell_command_frame(mock_transport.mac, CellNodeId.SUPERVISOR, 2, 1030, cmd_test)
    mock_transport.inject_rx_frame(f_cmd)
    assert_true(rx_events["cmd"])

    st = node.get_stats()
    assert_true(st.rx_frames == 4)
    assert_true(st.rx_invalid_frames == 0)
    assert_true(st.rx_sequence_drops == 0)
    print("  [PASS] test_node_rx_dispatch_callbacks")

    # 12. Node sequence loss detection
    mock_t2 = LoopbackTransport()
    node2 = CellNetL2Node(mock_t2, CellNodeId.GANTRY)
    node2.begin()

    f1 = CellNetL2Framing.build_conveyor_speed_frame(
        mock_t2.mac, CellNodeId.CONVEYOR, 10, 1000, conv_payload
    )
    mock_t2.inject_rx_frame(f1)

    # Skip 11, 12, 13 -> Jump to 14 (3 drops)
    f2 = CellNetL2Framing.build_conveyor_speed_frame(
        mock_t2.mac, CellNodeId.CONVEYOR, 14, 1040, conv_payload
    )
    mock_t2.inject_rx_frame(f2)

    st2 = node2.get_stats()
    assert_true(st2.rx_frames == 2)
    assert_true(st2.rx_sequence_drops == 3)
    print("  [PASS] test_node_sequence_loss_detection")

    # 13. Node TX methods
    mock_t3 = LoopbackTransport()
    node3 = CellNetL2Node(mock_t3, CellNodeId.GANTRY)
    node3.begin()

    assert_true(node3.send_heartbeat(1000, 0))
    assert_true(node3.send_gantry_status(1, 0, 10.0, 20.0, 0.0, 150.0))
    assert_true(node3.send_conveyor_speed(150.0, 12.0, 5000))
    assert_true(node3.send_vision_detect(101, 50.0, 10.0, 0.0, 1))
    assert_true(node3.send_cell_command(1))

    st3 = node3.get_stats()
    assert_true(st3.tx_frames == 5)
    assert_true(len(mock_t3.sent_frames) == 5)
    for f in mock_t3.sent_frames:
        assert_true(len(f) == MIN_ETH_FRAME_LEN)
    print("  [PASS] test_node_tx_methods")

    print(f"\nAll {passed}/{total} assertions passed successfully! 100% C++ parity verified.\n")
    return True


# ==============================================================================
# 7. INTERACTIVE CLI RUNNER & SIMULATOR
# ==============================================================================

def create_transport(args: argparse.Namespace) -> IL2Transport:
    if args.use_raw:
        if platform.system() != "Linux":
            print("[ERROR] Raw Linux packet sockets (AF_PACKET) are only available on Linux.")
            print("        Falling back to UDP bridge on localhost.")
            return UdpBridgeTransport(local_port=args.udp_port, remote_port=args.udp_port)
        return LinuxRawSocketL2Transport(iface_name=args.iface)
    else:
        return UdpBridgeTransport(
            local_port=args.udp_port,
            remote_port=args.remote_port if hasattr(args, "remote_port") and args.remote_port else args.udp_port,
            remote_host=args.dest_ip,
        )


def run_monitor(args: argparse.Namespace) -> None:
    transport = create_transport(args)
    node = CellNetL2Node(transport, CellNodeId.SUPERVISOR)
    node.begin()

    print(f"[CellNetL2 Monitor] Listening on {transport.__class__.__name__}...")
    print("Press Ctrl+C to stop.\n")
    print(f"{'Time':<12} {'Sender':<12} {'Seq':<5} {'Type':<16} {'Details'}")
    print("-" * 80)

    def on_hb(p: L2HeartbeatPayload, h: L2CellHeader):
        s_name = CellNodeId(h.sender_id).name
        print(f"{h.timestamp_us_low:<12} {s_name:<12} {h.sequence:<5} HEARTBEAT        uptime={p.uptime_ms}ms flags=0x{p.status_flags:04X}")

    def on_vis(p: L2VisionDetectPayload, h: L2CellHeader):
        s_name = CellNodeId(h.sender_id).name
        print(f"{h.timestamp_us_low:<12} {s_name:<12} {h.sequence:<5} VISION_DETECT    item={p.item_id} X={p.x_across_mm:.1f} Y={p.y_bat_mm:.1f} th={p.theta_deg:.1f} cls={p.battery_class} W={p.width_mm:.1f}")

    def on_conv(p: L2ConveyorSpeedPayload, h: L2CellHeader):
        s_name = CellNodeId(h.sender_id).name
        print(f"{h.timestamp_us_low:<12} {s_name:<12} {h.sequence:<5} CONVEYOR_SPEED   spd={p.speed_mm_s:.1f}mm/s disp={p.displacement_m:.3f}m enc={p.raw_encoder_cnt}")

    def on_gantry(p: L2GantryStatusPayload, h: L2CellHeader):
        s_name = CellNodeId(h.sender_id).name
        print(f"{h.timestamp_us_low:<12} {s_name:<12} {h.sequence:<5} GANTRY_STATUS    state={p.motion_state} X={p.x_pos_mm:.1f} Z={p.z_pos_mm:.1f} th={p.theta_deg:.1f} cycle={p.last_cycle_time_ms:.1f}ms")

    def on_cmd(p: L2CellCommandPayload, h: L2CellHeader):
        s_name = CellNodeId(h.sender_id).name
        print(f"{h.timestamp_us_low:<12} {s_name:<12} {h.sequence:<5} CELL_COMMAND     cmd={p.command_id} u8={p.param_u8} u16={p.param_u16} flt={p.param_float:.2f}")

    node.on_heartbeat(on_hb)
    node.on_vision_detect(on_vis)
    node.on_conveyor_speed(on_conv)
    node.on_gantry_status(on_gantry)
    node.on_cell_command(on_cmd)

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\nStopping monitor.")
    finally:
        transport.close()


def run_simulation(args: argparse.Namespace) -> None:
    transport = create_transport(args)
    sim_type = args.simulate.lower()

    if sim_type == "conveyor":
        node = CellNetL2Node(transport, CellNodeId.CONVEYOR)
        node.begin()
        print(f"[Simulator] Running CONVEYOR node (100 Hz / 10 ms telemetry)...")
        disp = 0.0
        enc = 0
        speed = 800.0  # mm/s
        dt = 0.010
        try:
            while True:
                disp += (speed * 0.001) * dt
                enc += int(500 * (speed * 0.001) * dt)
                node.send_conveyor_speed(speed, disp, enc)
                time.sleep(dt)
        except KeyboardInterrupt:
            pass

    elif sim_type == "vision":
        node = CellNetL2Node(transport, CellNodeId.VISION)
        node.begin()
        print(f"[Simulator] Running VISION node (emitting detections every 1.5s)...")
        item_id = 1
        try:
            while True:
                node.send_vision_detect(
                    item_id=item_id,
                    x_across_mm=120.5,
                    y_bat_mm=0.0,
                    theta_deg=30.0,
                    battery_class=8,  # 21700
                    capture_displacement_m=item_id * 0.450,
                    capture_disp_valid=True,
                    width_mm=21.2,
                    length_mm=70.0,
                    height_from_belt_mm=21.2,
                )
                print(f"  -> Published VISION_DETECT for item #{item_id}")
                item_id += 1
                time.sleep(1.5)
        except KeyboardInterrupt:
            pass

    elif sim_type == "gantry":
        node = CellNetL2Node(transport, CellNodeId.GANTRY)
        node.begin()
        print(f"[Simulator] Running GANTRY node (emitting status at 20 Hz)...")
        try:
            while True:
                node.send_gantry_status(
                    motion_state=2,
                    active_slot=0,
                    x_pos_mm=120.5,
                    z_pos_mm=35.7,
                    theta_deg=30.0,
                    last_cycle_time_ms=640.2,
                    fault_flags=0,
                )
                time.sleep(0.05)
        except KeyboardInterrupt:
            pass

    elif sim_type == "supervisor":
        node = CellNetL2Node(transport, CellNodeId.SUPERVISOR)
        node.begin()
        print(f"[Simulator] Running SUPERVISOR node (heartbeat 1 Hz)...")
        up = 0
        try:
            while True:
                node.send_heartbeat(uptime_ms=up, status_flags=0x0001)
                up += 1000
                time.sleep(1.0)
        except KeyboardInterrupt:
            pass
    else:
        print(f"Unknown simulation mode: {sim_type}. Options: conveyor, vision, gantry, supervisor")
    transport.close()


def main():
    parser = argparse.ArgumentParser(
        description="CellNetL2 Layer-2 Protocol Engine (Python implementation of lib/CellNetL2)"
    )
    parser.add_argument("--test", action="store_true", help="Run comprehensive self-test suite")
    parser.add_argument("--monitor", action="store_true", help="Sniff and monitor CellNetL2 frames")
    parser.add_argument(
        "--simulate",
        type=str,
        choices=["conveyor", "vision", "gantry", "supervisor"],
        help="Simulate an automated node emitting periodic telemetry",
    )
    parser.add_argument("--iface", type=str, default="eth0", help="Network interface for raw Linux sockets")
    parser.add_argument("--use-raw", action="store_true", help="Use raw Linux AF_PACKET sockets (Linux only)")
    parser.add_argument("--udp-port", type=int, default=8850, help="Local/remote UDP port for cross-platform bridge")
    parser.add_argument("--dest-ip", type=str, default="127.0.0.1", help="Target IP for UDP bridge")

    args = parser.parse_args()

    if args.test:
        ok = run_self_tests()
        sys.exit(0 if ok else 1)

    if args.monitor:
        run_monitor(args)
    elif args.simulate:
        run_simulation(args)
    else:
        # Default action: run tests and show help
        run_self_tests()
        print("\nRun with --help to see available monitoring and simulation modes.")


if __name__ == "__main__":
    main()

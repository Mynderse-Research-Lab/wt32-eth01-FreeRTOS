#!/usr/bin/env python3
"""
example_cell_net_l2.py — Example usage of the CellNetL2 protocol stack.

Demonstrates closed-cell industrial communication between:
  1. Conveyor Belt Tracker (WT32-ETH01, Node ID 0x02): broadcasts 100 Hz belt telemetry.
  2. Gantry Motion Controller (WT32-ETH01, Node ID 0x03): tracks belt, publishes multi-axis status.
  3. Cell Supervisor / HMI (Raspberry Pi, Node ID 0x04): emits heartbeats and supervisory commands.

Key features demonstrated:
  - Setting up node transports (cross-platform UDP bridge or native Linux raw packet sockets).
  - Registering typed callbacks for conveyor speed, gantry status, cell commands, and heartbeats.
  - Microsecond timestamping and rolling sequence tracking.
  - Link health monitoring (TX/RX frame counts, drop detection, invalid frame filtering).

Usage:
  # 1. Run the self-contained live multi-node interaction demo:
  python tools/example_cell_net_l2.py

  # 2. Run as an independent Gantry Motion Controller in one terminal:
  python tools/example_cell_net_l2.py --role gantry --port 8850

  # 3. Run as a Conveyor Belt Broadcaster (100 Hz) in a second terminal:
  python tools/example_cell_net_l2.py --role conveyor --target-port 8850

  # 4. Run as a Supervisor / HMI console in a third terminal:
  python tools/example_cell_net_l2.py --role supervisor --target-port 8850
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

# Ensure tools directory is on sys.path
sys.path.insert(0, str(Path(__file__).resolve().parent))

from cell_net_l2 import (
    CellMsgType,
    CellNetL2Node,
    CellNodeId,
    L2CellCommandPayload,
    L2CellHeader,
    L2ConveyorSpeedPayload,
    L2GantryStatusPayload,
    L2HeartbeatPayload,
    LinuxRawSocketL2Transport,
    UdpBridgeTransport,
    mac_to_str,
)


# ==============================================================================
# 1. GANTRY CONTROLLER NODE (WT32-ETH01, Node ID 0x03)
# ==============================================================================

def run_gantry_controller(port: int = 8850, iface: str | None = None, use_raw: bool = False):
    """
    Simulates the WT32-ETH01 Gantry Motion Controller:
      - Listens to 100 Hz conveyor telemetry to track the belt odometer in real-time.
      - Receives cell commands (START, STOP, PARK, SET_GRIPPER) from the Supervisor.
      - Periodically broadcasts its multi-axis position (X, Z, Theta) and status.
    """
    print("=" * 80)
    print(f" [GANTRY CONTROLLER] Starting Node ID: {CellNodeId.GANTRY} (WT32-ETH01)")
    print("=" * 80)

    # 1. Initialize transport
    if use_raw and iface:
        print(f" Transport: Linux Raw Socket (AF_PACKET / EtherType 0x88B5) on '{iface}'")
        transport = LinuxRawSocketL2Transport(iface_name=iface)
    else:
        print(f" Transport: UDP Bridge on localhost port {port}")
        transport = UdpBridgeTransport(local_port=port, remote_port=port)

    # 2. Bind node
    gantry_node = CellNetL2Node(transport, node_id=CellNodeId.GANTRY)
    gantry_node.begin()
    print(f" Hardware MAC Address : {mac_to_str(gantry_node.self_mac)}")
    print(f" Node Link Ready      : {gantry_node.is_ready()}\n")

    # Internal motion state
    current_x_mm = 0.0
    current_z_mm = 35.7  # Safe staging ceiling
    current_theta_deg = 0.0
    motion_state = 0     # 0=IDLE, 1=HOMING, 2=INTERCEPTING, 3=GRIP_COMMANDED, 4=FAULT
    last_belt_disp_m = 0.0

    # Callback A: Process high-speed conveyor odometer telemetry (100 Hz)
    def on_conveyor_telemetry(payload: L2ConveyorSpeedPayload, header: L2CellHeader):
        nonlocal last_belt_disp_m
        last_belt_disp_m = payload.displacement_m
        sender_name = CellNodeId(header.sender_id).name
        print(
            f"  [CONVEYOR RX] From {sender_name} | Seq: {header.sequence:>3} | "
            f"Speed: {payload.speed_mm_s:6.1f} mm/s | "
            f"Disp: {payload.displacement_m:7.3f} m | "
            f"Enc: {payload.raw_encoder_cnt:>7} cnt | "
            f"Timestamp: {header.timestamp_us_low} us"
        )

    # Callback B: Process supervisor commands
    def on_supervisor_command(payload: L2CellCommandPayload, header: L2CellHeader):
        nonlocal motion_state, current_x_mm, current_z_mm
        sender_name = CellNodeId(header.sender_id).name
        cmd_names = {1: "START", 2: "STOP", 3: "PAUSE", 4: "CLEAR_FAULT", 5: "PARK", 6: "SET_GRIPPER"}
        cmd_str = cmd_names.get(payload.command_id, f"UNKNOWN_{payload.command_id}")

        print(
            f"\n  *** [COMMAND RX] From {sender_name} ***\n"
            f"      Command     : {cmd_str} (ID {payload.command_id})\n"
            f"      Param U8    : {payload.param_u8}\n"
            f"      Param U16   : {payload.param_u16}\n"
            f"      Param Float : {payload.param_float:.2f}\n"
            f"      Sequence    : {header.sequence}\n"
        )

        if payload.command_id == 1:    # START
            motion_state = 2           # INTERCEPTING
        elif payload.command_id == 5:  # PARK
            motion_state = 0           # IDLE
            current_x_mm = 0.0
            current_z_mm = 35.7
        elif payload.command_id == 6:  # SET_GRIPPER
            print(f"      -> Gripper solenoid toggled: {'CLOSE' if payload.param_u8 == 1 else 'OPEN'}")

    # Callback C: Supervisor heartbeats
    def on_heartbeat(payload: L2HeartbeatPayload, header: L2CellHeader):
        sender_name = CellNodeId(header.sender_id).name
        print(f"  [HEARTBEAT RX] From {sender_name} | Uptime: {payload.uptime_ms} ms | Flags: 0x{payload.status_flags:04X}")

    gantry_node.on_conveyor_speed(on_conveyor_telemetry)
    gantry_node.on_cell_command(on_supervisor_command)
    gantry_node.on_heartbeat(on_heartbeat)

    print(" Gantry Node listening. Periodically publishing GANTRY_STATUS (20 Hz)...")
    print(" Press Ctrl+C to stop.\n")

    try:
        last_pub_t = time.time()
        while True:
            time.sleep(0.05)  # 20 Hz loop
            now = time.time()
            if now - last_pub_t >= 0.050:
                last_pub_t = now
                # Publish Gantry Multi-Axis Status
                gantry_node.send_gantry_status(
                    motion_state=motion_state,
                    active_slot=0,
                    x_pos_mm=current_x_mm,
                    z_pos_mm=current_z_mm,
                    theta_deg=current_theta_deg,
                    last_cycle_time_ms=640.2,
                    fault_flags=0,
                )
    except KeyboardInterrupt:
        print("\n Gantry Controller shutting down...")
    finally:
        transport.close()


# ==============================================================================
# 2. CONVEYOR BELT TRACKER NODE (WT32-ETH01, Node ID 0x02)
# ==============================================================================

def run_conveyor_broadcaster(target_port: int = 8850, my_port: int = 8851, iface: str | None = None, use_raw: bool = False):
    """
    Simulates the WT32-ETH01 Conveyor Tracker:
      - Reads a 500-PPR hardware quadrature encoder on the belt drive roller.
      - Broadcasts real-time belt linear speed and cumulative displacement at 100 Hz (10 ms period).
    """
    print("=" * 80)
    print(f" [CONVEYOR TRACKER] Starting Node ID: {CellNodeId.CONVEYOR} (WT32-ETH01)")
    print("=" * 80)

    if use_raw and iface:
        transport = LinuxRawSocketL2Transport(iface_name=iface)
    else:
        transport = UdpBridgeTransport(local_port=my_port, remote_port=target_port)

    node = CellNetL2Node(transport, node_id=CellNodeId.CONVEYOR)
    node.begin()

    speed_mm_s = 800.0  # Production sorting speed: 800 mm/s
    displacement_m = 0.0
    encoder_cnt = 0
    dt_s = 0.010        # 10 ms interval (100 Hz cadence)

    print(f" Broadcasting Conveyor Telemetry at 100 Hz to port {target_port}...")
    print(" Press Ctrl+C to stop.\n")

    try:
        step = 0
        while True:
            # Advance simulated belt displacement
            displacement_m += (speed_mm_s * 0.001) * dt_s
            encoder_cnt += int(500 * (speed_mm_s * 0.001) * dt_s)

            # Transmit Layer-2 CONVEYOR_SPEED Frame (Message Type 0x02)
            node.send_conveyor_speed(
                speed_mm_s=speed_mm_s,
                displacement_m=displacement_m,
                raw_encoder_cnt=encoder_cnt,
            )

            step += 1
            if step % 100 == 0:  # Status log once per second
                stats = node.get_stats()
                print(f"  [CONVEYOR TX] Dispatched {stats.tx_frames:>5} frames | Displacement: {displacement_m:6.2f} m | Speed: {speed_mm_s} mm/s")

            time.sleep(dt_s)
    except KeyboardInterrupt:
        print("\n Conveyor Tracker stopped.")
    finally:
        transport.close()


# ==============================================================================
# 3. SUPERVISOR / HMI NODE (Raspberry Pi, Node ID 0x04)
# ==============================================================================

def run_supervisor_console(target_port: int = 8850, my_port: int = 8852, iface: str | None = None, use_raw: bool = False):
    """
    Simulates the Raspberry Pi Cell Supervisor & HMI:
      - Emits periodic Heartbeats (1 Hz).
      - Listens to both Conveyor telemetry and Gantry motion status.
      - Emits supervisory Cell Commands (START, STOP, PARK, SET_GRIPPER).
    """
    print("=" * 80)
    print(f" [CELL SUPERVISOR] Starting Node ID: {CellNodeId.SUPERVISOR} (Raspberry Pi)")
    print("=" * 80)

    if use_raw and iface:
        transport = LinuxRawSocketL2Transport(iface_name=iface)
    else:
        transport = UdpBridgeTransport(local_port=my_port, remote_port=target_port)

    supervisor_node = CellNetL2Node(transport, node_id=CellNodeId.SUPERVISOR)
    supervisor_node.begin()

    # Listen to Gantry Status
    def on_gantry_status(payload: L2GantryStatusPayload, header: L2CellHeader):
        state_names = {0: "IDLE", 1: "HOMING", 2: "INTERCEPTING", 3: "GRIP_COMMANDED", 4: "FAULT"}
        st_name = state_names.get(payload.motion_state, f"UNKNOWN_{payload.motion_state}")
        print(
            f"  [GANTRY STATUS RX] State: {st_name:<14} | "
            f"X: {payload.x_pos_mm:5.1f} mm | Z: {payload.z_pos_mm:5.1f} mm | "
            f"Theta: {payload.theta_deg:5.1f} deg | Cycle: {payload.last_cycle_time_ms:5.1f} ms"
        )

    # Listen to Conveyor Telemetry
    def on_conveyor_speed(payload: L2ConveyorSpeedPayload, header: L2CellHeader):
        print(f"  [CONVEYOR RX] Speed: {payload.speed_mm_s:5.1f} mm/s | Disp: {payload.displacement_m:6.3f} m")

    supervisor_node.on_gantry_status(on_gantry_status)
    supervisor_node.on_conveyor_speed(on_conveyor_speed)

    print(" Supervisor ready. Emitting Heartbeats and interactive commands.")
    print(" Commands available: start, stop, park, grip_on, grip_off\n")

    uptime_ms = 0
    try:
        # Example sequence of commands
        print(" -> [Supervisor] Sending Command 1: START")
        supervisor_node.send_cell_command(command_id=1)
        time.sleep(1.0)

        print(" -> [Supervisor] Sending Command 6: SET_GRIPPER (Close)")
        supervisor_node.send_cell_command(command_id=6, param_u8=1)
        time.sleep(1.0)

        print(" -> [Supervisor] Sending Command 6: SET_GRIPPER (Open)")
        supervisor_node.send_cell_command(command_id=6, param_u8=0)
        time.sleep(1.0)

        print(" -> [Supervisor] Sending Command 5: PARK")
        supervisor_node.send_cell_command(command_id=5, param_float=35.7)
        time.sleep(1.0)

        # Loop heartbeats
        while True:
            supervisor_node.send_heartbeat(uptime_ms=uptime_ms, status_flags=0x0001)
            uptime_ms += 1000
            time.sleep(1.0)

    except KeyboardInterrupt:
        print("\n Supervisor stopped.")
    finally:
        transport.close()


# ==============================================================================
# 4. SELF-CONTAINED MULTI-NODE INTERACTION DEMO
# ==============================================================================

def run_multi_node_interactive_demo():
    """
    Self-contained live demo:
    Spawns all 3 non-vision sorting cell nodes communicating simultaneously in real-time:
      - Conveyor Tracker (Node 0x02): Transmits 100 Hz belt telemetry
      - Gantry Controller (Node 0x03): Receives belt odometer, publishes status at 20 Hz
      - Supervisor / HMI (Node 0x04): Issues START, SET_GRIPPER, and PARK commands
    """
    print("\n" + "#" * 80)
    print(" CellNetL2 Industrial Sorting Cell Communication Demo")
    print(" Nodes: Conveyor Tracker (0x02) + Gantry Motion (0x03) + Supervisor (0x04)")
    print("#" * 80 + "\n")

    # Setup transports on loopback ports
    # Gantry listens on 8850, sends to 8851
    gantry_tx = UdpBridgeTransport(local_port=8850, remote_port=8851)
    gantry = CellNetL2Node(gantry_tx, node_id=CellNodeId.GANTRY)
    gantry.begin()

    # Conveyor & Supervisor send on 8851, transmit to 8850
    remote_tx = UdpBridgeTransport(local_port=8851, remote_port=8850)
    conveyor = CellNetL2Node(remote_tx, node_id=CellNodeId.CONVEYOR)
    conveyor.begin()

    supervisor = CellNetL2Node(remote_tx, node_id=CellNodeId.SUPERVISOR)
    supervisor.begin()

    # Metrics container
    received = {"conveyor_telemetry": 0, "commands": 0, "heartbeats": 0, "gantry_status": 0}

    # Gantry callbacks
    def gantry_on_conveyor(payload: L2ConveyorSpeedPayload, header: L2CellHeader):
        received["conveyor_telemetry"] += 1
        print(
            f"  [Gantry <- Conveyor] Speed: {payload.speed_mm_s:5.1f} mm/s | "
            f"Disp: {payload.displacement_m:6.3f} m | Enc: {payload.raw_encoder_cnt} cnt"
        )

    def gantry_on_command(payload: L2CellCommandPayload, header: L2CellHeader):
        received["commands"] += 1
        cmd_names = {1: "START", 2: "STOP", 5: "PARK", 6: "SET_GRIPPER"}
        c_name = cmd_names.get(payload.command_id, str(payload.command_id))
        print(f"  [Gantry <- Supervisor] Executing Command: {c_name} (u8={payload.param_u8}, flt={payload.param_float})")

    def gantry_on_heartbeat(payload: L2HeartbeatPayload, header: L2CellHeader):
        received["heartbeats"] += 1
        print(f"  [Gantry <- Supervisor] Heartbeat | Uptime: {payload.uptime_ms} ms")

    # Supervisor callbacks
    def supervisor_on_gantry_status(payload: L2GantryStatusPayload, header: L2CellHeader):
        received["gantry_status"] += 1
        print(
            f"  [Supervisor <- Gantry] Status | State: {payload.motion_state} | "
            f"X: {payload.x_pos_mm:.1f} mm | Z: {payload.z_pos_mm:.1f} mm | Theta: {payload.theta_deg:.1f} deg"
        )

    gantry.on_conveyor_speed(gantry_on_conveyor)
    gantry.on_cell_command(gantry_on_command)
    gantry.on_heartbeat(gantry_on_heartbeat)
    supervisor.on_gantry_status(supervisor_on_gantry_status)

    time.sleep(0.05)

    # 1. Conveyor begins transmitting high-speed telemetry frames
    print("--- 1. Conveyor Tracker transmitting 100 Hz belt odometer frames ---")
    for i in range(1, 11):
        conveyor.send_conveyor_speed(
            speed_mm_s=800.0,
            displacement_m=0.008 * i,
            raw_encoder_cnt=i * 20,
        )
        time.sleep(0.010)

    # 2. Supervisor sends command to Gantry
    print("\n--- 2. Supervisor emitting cell commands to Gantry ---")
    supervisor.send_heartbeat(uptime_ms=5000, status_flags=0x0001)
    time.sleep(0.02)

    supervisor.send_cell_command(command_id=1)  # START
    time.sleep(0.02)

    supervisor.send_cell_command(command_id=6, param_u8=1)  # SET_GRIPPER (Pneumatic Close)
    time.sleep(0.02)

    supervisor.send_cell_command(command_id=5, param_float=35.7)  # PARK at Safe-Z
    time.sleep(0.02)

    # 3. Gantry transmits its multi-axis status
    print("\n--- 3. Gantry Controller publishing motion status ---")
    gantry.send_gantry_status(
        motion_state=2,  # INTERCEPTING
        active_slot=0,
        x_pos_mm=125.0,
        z_pos_mm=35.7,
        theta_deg=45.0,
        last_cycle_time_ms=640.2,
    )
    time.sleep(0.05)

    # 4. Display Stats
    g_stats = gantry.get_stats()
    c_stats = conveyor.get_stats()
    s_stats = supervisor.get_stats()

    print("\n" + "=" * 80)
    print(" Live Interaction Summary:")
    print(f"   Conveyor Telemetry Dispatched : {c_stats.tx_frames:>3} frames")
    print(f"   Supervisor Frames Dispatched  : {s_stats.tx_frames:>3} frames")
    print(f"   Gantry Frames Received        : {g_stats.rx_frames:>3} frames")
    print(f"   Gantry Sequence Drops         : {g_stats.rx_sequence_drops:>3} drops")
    print(f"   Gantry Invalid Frames         : {g_stats.rx_invalid_frames:>3} invalid")
    print("=" * 80)

    gantry_tx.close()
    remote_tx.close()

    assert g_stats.rx_frames == 14, f"Expected 14 frames, got {g_stats.rx_frames}"
    assert g_stats.rx_sequence_drops == 0, f"Expected 0 drops, got {g_stats.rx_sequence_drops}"
    print("\n[SUCCESS] Industrial sorting cell communication verified with 100% data integrity!\n")


def main():
    parser = argparse.ArgumentParser(
        description="CellNetL2 Industrial Communication Example (Conveyor + Gantry + Supervisor)"
    )
    parser.add_argument(
        "--role",
        type=str,
        choices=["demo", "gantry", "conveyor", "supervisor"],
        default="demo",
        help="Node role to run: 'demo' (interactive 3-node simulation), 'gantry', 'conveyor', or 'supervisor'",
    )
    parser.add_argument("--port", type=int, default=8850, help="Local listening UDP port (default: 8850)")
    parser.add_argument("--target-port", type=int, default=8850, help="Target destination UDP port (default: 8850)")
    parser.add_argument("--iface", type=str, default=None, help="Physical Linux Ethernet NIC (e.g. eth0)")
    parser.add_argument("--use-raw", action="store_true", help="Use raw Linux AF_PACKET sockets instead of UDP bridge")

    args = parser.parse_args()

    if args.role == "demo":
        run_multi_node_interactive_demo()
    elif args.role == "gantry":
        run_gantry_controller(port=args.port, iface=args.iface, use_raw=args.use_raw)
    elif args.role == "conveyor":
        run_conveyor_broadcaster(target_port=args.target_port, my_port=args.port + 1, iface=args.iface, use_raw=args.use_raw)
    elif args.role == "supervisor":
        run_supervisor_console(target_port=args.target_port, my_port=args.port + 2, iface=args.iface, use_raw=args.use_raw)


if __name__ == "__main__":
    main()

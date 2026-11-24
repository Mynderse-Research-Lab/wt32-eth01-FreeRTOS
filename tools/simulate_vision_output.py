"""Simulated vision node: random battery detections as CellNet L2 VISION_DETECT frames.

Default transport is raw L2 Ethernet (EtherType 0x88B5) on --iface, so the
gantry on the cell switch receives the frames. Run on the Lab-Pi (needs root
or CAP_NET_RAW for the raw socket):

    sudo python3 simulate_vision_output.py --iface eth0 --no-plot
    sudo python3 simulate_vision_output.py --iface eth0 --no-plot --class 5   # 9V only

Each detection is stamped with the live conveyor displacement taken from the
conveyor's CONVEYOR_SPEED frames on the same wire; the gantry rejects a
detection without it. --udp keeps the old UDP bridge to the MuJoCo sim.

Frame sent (include/cell_net_l2_protocol.h, the firmware vision contract):
  x_across_mm  across the belt, from the camera-image centre line
  y_bat_mm     along the belt, from the camera centre
  theta_deg    long axis, 0 along +X (across), 90 along +Y (downstream), + about +Z up
  battery_class firmware id (5 = 9V, 11 laptop, 12 Li-ion pack, 13 power tool)
"""

import argparse
import math
import os
import random
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cell_net_l2 import CellNetL2Node, CellNodeId, LinuxRawSocketL2Transport, UdpBridgeTransport  # noqa: E402

# [class name, sim class id, length (longest face) mm, width mm, height from belt mm]
# sim ids: 1 = 9V, 2 = pack of cylindrical cells, 3 = laptop, 4 = power tool
CLASSES = [["nine_volt_flatface", 1, 47.7, 26.6, 17.3],
           ["nine_volt_shortface", 1, 47.7, 17.3, 26.6],
           ["nine_volt_bottomface", 1, 26.6, 17.3, 47.7],
           ["pack", 2, 115.14, 68.3, 26.7],
           ["pack_shortface", 2, 115.14, 26.7, 68.3],
           ["laptop", 3, 207.9, 50.3, 21.3],
           ["power_tool_makita", 4, 111.7, 73.8, 49.6],
           ["power_tool_makita_lenface", 4, 111.7, 49.6, 73.8],
           ["power_tool_makita_shortface", 4, 73.8, 49.6, 111.7],
           ["power_tool_NiCd_24V", 4, 148.4, 92.1, 53],
           ["power_tool_NiCd_24V_shortface", 4, 92.1, 53, 148.4],
           ["power_tool_NiCd_24V_lenface", 4, 148.4, 53, 92.1],
           ["power_tool_rigid", 4, 117.8, 84.7, 43.3],
           ["power_tool_rigid_lenface", 4, 117.8, 43.3, 78.4],
           ["power_tool_rigid_shortface", 4, 78.4, 43.3, 117.8],
           ["power_tool_single_source", 4, 145.1, 78.7, 66.8],
           ["power_tool_single_source_lenface", 4, 145.1, 66.8, 78.7],
           ["power_tool_single_source_shortface", 4, 78.7, 66.8, 145.1],
           ["power_tool_gtv", 4, 124.3, 79.6, 53.2],
           ["power_tool_gtv_lenface", 4, 124.3, 53.2, 79.6],
           ["power_tool_gtv_shortface", 4, 79.6, 53.2, 124.3],
           ["power_tool_ryobi", 4, 97.2, 76.2, 54.1],
           ["power_tool_ryobi_lenface", 4, 97.2, 54.1, 76.2],
           ["power_tool_ryobi_shortface", 4, 76.2, 54.1, 97.2],
           ["power_tool_diehard", 4, 136.2, 77.4, 56.4],
           ["power_tool_diehard_lenface", 4, 136.2, 56.4, 77.4],
           ["power_tool_diehard_shortface", 4, 77.4, 56.4, 136.2]]

# Sim class id -> firmware battery_class (include/pick_intercept.h kDevProfiles).
FIRMWARE_CLASS = {1: 5, 2: 12, 3: 11, 4: 13}

# Camera FOV: 14.5 in by 10.875 in; x is parallel to the gantry beam (across
# the belt), y runs along the belt.
X_MAX_MM = 10.875 * 25.4  # 276.225
Y_MAX_MM = 14.5 * 25.4    # 368.3
POT_ANGLES_RAD = [math.radians(0.25 * i) for i in range(int(360 / 0.25) + 1)]

# A detection is stamped only from a conveyor frame younger than this.
CONVEYOR_MAX_AGE_S = 0.5
# Below this belt speed the simulation pauses: nothing reaches the gripper,
# so detections would only fill the gantry's 16-slot part tracker.
BELT_STOPPED_MM_S = 5.0


class ConveyorClock:
    """Latest CONVEYOR_SPEED frame; extrapolated to the capture instant."""

    def __init__(self):
        self._lock = threading.Lock()
        self._disp_m = 0.0
        self._speed_mm_s = 0.0
        self._rx_s = None

    def on_frame(self, payload, _header):
        with self._lock:
            self._disp_m = payload.displacement_m
            self._speed_mm_s = payload.speed_mm_s
            self._rx_s = time.monotonic()

    def displacement_now(self):
        """(displacement_m, age_s) or (None, age_s) when stale or never seen."""
        with self._lock:
            if self._rx_s is None:
                return None, None
            age = time.monotonic() - self._rx_s
            if age > CONVEYOR_MAX_AGE_S:
                return None, age
            return self._disp_m + self._speed_mm_s * age * 0.001, age

    def speed_mm_s(self):
        """Latest belt speed, or None before the first conveyor frame."""
        with self._lock:
            return None if self._rx_s is None else self._speed_mm_s


def choose_center_coords(x_max_mm, y_max_mm, length, width, theta):
    x_c = random.randint(0, int(x_max_mm))
    y_c = random.randint(0, int(y_max_mm))
    # orientation: 0 rad = longest face parallel to the x axis
    xp1 = x_c - ((length / 2) * math.cos(theta)) + ((width / 2) * math.sin(theta))
    yp1 = y_c - ((length / 2) * math.sin(theta)) - ((width / 2) * math.cos(theta))
    xp2 = x_c - ((length / 2) * math.cos(theta)) - ((width / 2) * math.sin(theta))
    yp2 = y_c - ((length / 2) * math.sin(theta)) + ((width / 2) * math.cos(theta))
    xp3 = x_c + ((length / 2) * math.cos(theta)) + ((width / 2) * math.sin(theta))
    yp3 = y_c + ((length / 2) * math.sin(theta)) - ((width / 2) * math.cos(theta))
    xp4 = x_c + ((length / 2) * math.cos(theta)) - ((width / 2) * math.sin(theta))
    yp4 = y_c + ((length / 2) * math.sin(theta)) + ((width / 2) * math.cos(theta))
    return [[x_c, y_c], [xp1, yp1], [xp2, yp2], [xp3, yp3], [xp4, yp4]]


def to_firmware_frame(x_c, y_c, angle_rad):
    """Image coordinates to the firmware vision contract.

    The image angle runs from +x (across the belt) toward +y (downstream), which
    is the firmware reference too, so theta is the angle folded into [-180, 180).
    """
    x_across = x_c - X_MAX_MM / 2.0
    y_bat = y_c - Y_MAX_MM / 2.0
    theta = math.degrees(angle_rad)
    theta = (theta + 180.0) % 360.0 - 180.0
    return x_across, y_bat, theta


def class_pool(firmware_ids):
    """CLASSES entries whose firmware battery_class is in firmware_ids (all when None)."""
    if not firmware_ids:
        return list(CLASSES)
    return [c for c in CLASSES if FIRMWARE_CLASS[c[1]] in firmware_ids]


def parse_class_ids(values):
    """--class 5 12 or --class 5,12 -> {5, 12}; rejects ids this script cannot send."""
    valid = sorted(set(FIRMWARE_CLASS.values()))
    ids = set()
    for v in values or []:
        for part in str(v).split(","):
            part = part.strip()
            if not part:
                continue
            if not part.isdigit() or int(part) not in valid:
                raise SystemExit(f"--class: '{part}' is not one of {valid} "
                                 "(5 = 9V, 11 = laptop, 12 = Li-ion pack, 13 = power tool)")
            ids.add(int(part))
    return ids


def make_detection(pool=CLASSES):
    sim_class = random.choice(pool)
    _, class_id, length, width, height = sim_class
    orientation = random.choice(POT_ANGLES_RAD) * random.choice([-1, 1])
    while True:
        corners = choose_center_coords(X_MAX_MM, Y_MAX_MM, length, width, orientation)
        if all(0 <= x <= X_MAX_MM and 0 <= y <= Y_MAX_MM for x, y in corners[1:]):
            return sim_class, orientation, corners


class Plot:
    def __init__(self):
        import matplotlib.pyplot as plt
        self.plt = plt
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(6, 4))
        plt.show(block=False)

    def alive(self):
        return self.plt.fignum_exists(self.fig.number)

    def draw(self, corners, item_id):
        ax = self.ax
        ax.clear()
        order = [1, 2, 4, 3, 1]
        ax.plot([corners[j][1] for j in order], [corners[j][0] for j in order],
                color="black", marker="o")
        ax.plot(corners[0][1], corners[0][0], color="black", marker="o")
        ax.set_title(f"Vision Output - Item {item_id}")
        ax.set_xlim(0, Y_MAX_MM)
        ax.set_ylim(0, X_MAX_MM)
        ax.set_aspect("equal")
        self.fig.canvas.draw_idle()

    def pause(self, s):
        self.plt.pause(s)

    def close(self):
        self.plt.close(self.fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--iface", default="eth0", help="cell-switch interface for raw L2 (default eth0)")
    ap.add_argument("--udp", action="store_true",
                    help="UDP bridge to the MuJoCo sim on 127.0.0.1:8850 instead of raw L2")
    ap.add_argument("--interval", type=float, default=0.5, help="seconds between detections")
    ap.add_argument("--count", type=int, default=0, help="stop after this many sends (0 = forever)")
    ap.add_argument("--no-plot", action="store_true", help="headless (no matplotlib window)")
    ap.add_argument("--class", dest="classes", nargs="+", metavar="ID",
                    help="firmware classes to generate, e.g. --class 5 or --class 5,12 "
                         "(5 = 9V, 11 = laptop, 12 = Li-ion pack, 13 = power tool; default all)")
    args = ap.parse_args()
    class_ids = parse_class_ids(args.classes)
    pool = class_pool(class_ids)

    if args.udp:
        transport = UdpBridgeTransport(local_port=8851, remote_port=8850, remote_host="127.0.0.1")
    else:
        transport = LinuxRawSocketL2Transport(iface_name=args.iface)
    node = CellNetL2Node(transport, node_id=CellNodeId.VISION)
    clock = ConveyorClock()
    node.on_conveyor_speed(clock.on_frame)
    node.begin()

    plot = None
    if not args.no_plot and (os.environ.get("DISPLAY") or sys.platform == "win32"):
        plot = Plot()
    print(f"vision sim on {'UDP bridge' if args.udp else args.iface}, every {args.interval:.2f} s, "
          f"classes {sorted(class_ids) if class_ids else 'all'} ({len(pool)} object types)",
          flush=True)

    item_id = 0
    sent = 0
    stale_noted = False
    paused = False
    try:
        while (plot is None or plot.alive()) and (args.count == 0 or sent < args.count):
            start = time.perf_counter()
            sim_class, orientation, corners = make_detection(pool)
            name, class_id, length, width, height = sim_class
            disp_m, age_s = clock.displacement_now()
            speed = clock.speed_mm_s()
            belt_stopped = speed is not None and abs(speed) < BELT_STOPPED_MM_S
            if belt_stopped != paused:
                paused = belt_stopped
                print(f"belt {'stopped' if paused else 'moving'} ({speed:.1f} mm/s); simulation "
                      f"{'paused' if paused else 'resumed'}", flush=True)
            if paused:
                pass
            elif disp_m is None and not args.udp:
                if not stale_noted:
                    print("no fresh CONVEYOR_SPEED frame on the wire; holding detections "
                          "(the gantry rejects a detection without capture displacement)", flush=True)
                    stale_noted = True
            else:
                stale_noted = False
                item_id += 1
                x_across, y_bat, theta = to_firmware_frame(corners[0][0], corners[0][1], orientation)
                ok = node.send_vision_detect(
                    item_id=item_id,
                    x_across_mm=float(x_across),
                    y_bat_mm=float(y_bat),
                    theta_deg=float(theta),
                    battery_class=FIRMWARE_CLASS[class_id],
                    capture_displacement_m=float(disp_m or 0.0),
                    capture_disp_valid=disp_m is not None,
                    width_mm=float(width),
                    length_mm=float(length),
                    height_from_belt_mm=float(height),
                )
                sent += 1
                print(f"#{item_id} {name} class {FIRMWARE_CLASS[class_id]} x {x_across:+.1f} "
                      f"y {y_bat:+.1f} th {theta:+.2f} w {width} l {length} h {height} "
                      f"disp {disp_m if disp_m is not None else float('nan'):.4f} m "
                      f"{'sent' if ok else 'SEND FAILED'}", flush=True)
                if plot is not None:
                    plot.draw(corners, item_id)
            remaining = max(0.001, args.interval - (time.perf_counter() - start))
            if plot is not None:
                plot.pause(remaining)
            else:
                time.sleep(remaining)
    except KeyboardInterrupt:
        pass
    finally:
        transport.close()
        if plot is not None:
            plot.close()


if __name__ == "__main__":
    main()

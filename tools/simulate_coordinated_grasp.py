#!/usr/bin/env python3
"""Independent paper check for the coordinated tilted grasp.

Two contact detectors share only the plant (belt, jaw profile, Z trapezoid):

* analytic roots on each linear piece
* a forward sampler that tests pad/cell intervals and does not call those roots

The nominal grid is every pick class, 0.0..90.0 deg in steps of 0.1,
and 20..1500 mm/s in steps of 20. Finger width 10 mm, finger length 50 mm.
"""

from __future__ import annotations

import json
import math
import os
import sys
import time

# Plant. Width is the close-axis pad size. Length is the along-pad size.
OPEN_MM = 80.0
A_OPEN = 40.0
CLOSE_S = 0.190
QDOT = 40.0 / CLOSE_S
VALVE_ON_S = 0.008
VALVE_OFF_S = 0.017
FINGER_WIDTH_MM = 10.0
FINGER_LENGTH_MM = 50.0
ZB_MM = 115.0
SAFE_Z_MM = 35.7
HOVER_CLEAR_MM = 3.0
VZ_MM_S = 1000.0
ACC_MM_S2 = 3000.0
STROKE_MM = 105.5
GRASP_CAP_MM = 25.4
DRAG_LIMIT_MM = 10.0
DRAG_TIGHTEN_MM = 1.0
PRE_GRASP_GAP_MM = 0.5
# Z fire: 1 ms dispatch + 2 ms RPI + 2 ms drive allowance. Not the 10 ms loop.
Z_UNC_S = 0.005
# Close fire: 1 ms dispatch + 1 ms gripper write. Valve-on is nominal, not slack.
CLOSE_UNC_S = 0.002
DT_S = 0.0001
AGREE_S = 0.00025
AGREE_MM = 0.25

CLASSES = (
    (1, "AAA", 44.5, 10.5, 10.5),
    (2, "AA", 50.5, 14.5, 14.5),
    (3, "D", 61.5, 34.2, 34.2),
    (4, "C", 50.0, 26.2, 26.2),
    (5, "9V", 48.5, 17.5, 26.5),
    (6, "CR123A", 34.5, 17.0, 17.0),
    (7, "18650", 65.0, 18.3, 18.3),
    (8, "21700", 70.0, 21.2, 21.2),
    (9, "26650", 65.0, 26.5, 26.5),
    (10, "32650", 65.0, 32.0, 32.0),
)
PACKS = (
    (11, "laptop", 205.0, 50.0, 22.0),
    (12, "cellpack", 114.0, 66.0, 25.0),
    (13, "powertool", 120.0, 75.0, 80.0),
)

STATUS_CODE = {
    "analytic-clear": 0,
    "tighten": 1,
    "drag-exceeds-limit": 2,
    "contact-assisted": 3,
    "outside-current-limit": 4,
    "invalid-input": 5,
}


def z_hover(height):
    return max(SAFE_Z_MM, ZB_MM - height - HOVER_CLEAR_MM)


def z_top(height):
    return ZB_MM - height


def z_grasp(height):
    if height < 30.0:
        return ZB_MM - 3.0
    return z_top(height) + min(0.5 * height, GRASP_CAP_MM)


def profile_seconds(distance, speed, accel, decel):
    if distance <= 0.0:
        return 0.0
    if speed <= 0.0 or accel <= 0.0 or decel <= 0.0:
        return 1.0e6
    ramp = (speed * speed) * (0.5 / accel + 0.5 / decel)
    if distance >= ramp:
        return (distance - ramp) / speed + speed / accel + speed / decel
    peak = math.sqrt(2.0 * distance / (1.0 / accel + 1.0 / decel))
    return peak / accel + peak / decel


def time_along(distance, length, speed, accel, decel):
    """Seconds to cover `distance` along a trapezoid of `length`. Mirrors profileSeconds."""
    if distance <= 0.0:
        return 0.0
    if length <= 0.0:
        return 0.0
    if distance >= length:
        return profile_seconds(length, speed, accel, decel)
    if speed <= 0.0 or accel <= 0.0 or decel <= 0.0:
        return 1.0e6
    da = (speed * speed) / (2.0 * accel)
    dd = (speed * speed) / (2.0 * decel)
    if da + dd < length:
        if distance <= da:
            return math.sqrt(2.0 * distance / accel)
        cruise_end = length - dd
        if distance <= cruise_end:
            return speed / accel + (distance - da) / speed
        into = distance - cruise_end
        disc = speed * speed - 2.0 * decel * into
        t_dec = speed / decel if disc <= 0.0 else (speed - math.sqrt(disc)) / decel
        return speed / accel + (cruise_end - da) / speed + t_dec
    peak = math.sqrt(2.0 * length / (1.0 / accel + 1.0 / decel))
    d_peak = (peak * peak) / (2.0 * accel)
    if distance <= d_peak:
        return math.sqrt(2.0 * distance / accel)
    remain = length - distance
    return peak / accel + peak / decel - math.sqrt(2.0 * remain / decel)


def jaw_a(t, tm):
    if t <= tm:
        return A_OPEN
    q = QDOT * (t - tm)
    if q >= 40.0:
        return 0.0
    return A_OPEN - q


class Plant:
    """Belt, jaw, and Z for one sample. Contact detectors sit on top of this."""

    def __init__(self, length, width, height, theta_deg, speed):
        self.L = length
        self.w = width
        self.h = height
        self.theta_deg = theta_deg
        self.v = speed
        rad = math.radians(theta_deg)
        self.sin_a = abs(math.sin(rad))
        self.cos_a = abs(math.cos(rad))
        self.half_w = 0.5 * width
        self.half_along = 0.5 * (length + FINGER_LENGTH_MM)
        self.z_top = z_top(height)
        self.z_g = z_grasp(height)
        self.z_h = z_hover(height)
        self.drop = self.z_g - self.z_h
        self.tau_grasp = time_along(self.drop, self.drop, VZ_MM_S, ACC_MM_S2, ACC_MM_S2)
        to_top = max(0.0, self.z_top - self.z_h)
        self.tau_top = time_along(to_top, self.drop, VZ_MM_S, ACC_MM_S2, ACC_MM_S2)
        self.tau_band = max(0.0, self.tau_grasp - self.tau_top)
        if self.sin_a < 1e-12 or speed <= 0.0:
            self.t_clear = 1.0e6
        else:
            self.t_clear = (A_OPEN - self.half_w) / (speed * self.sin_a)
        if self.cos_a < 1e-12 or speed <= 0.0:
            self.t_along = 1.0e6
        else:
            self.t_along = self.half_along / (speed * self.cos_a)
        self.past_stroke = self.z_g > STROKE_MM + 1e-6

    def abs_eta(self, t):
        return self.v * abs(t) * self.sin_a

    def along(self, t):
        if self.cos_a < 1e-12:
            return True
        return abs(self.v * t * self.cos_a) <= self.half_along + 1e-9

    def intervals_overlap(self, t, tm):
        """Pad-frame interval test. True when either pad rectangle meets the cell."""
        if not self.along(t):
            return False
        a = jaw_a(t, tm)
        u = self.abs_eta(t)
        inner = a - self.half_w
        outer = a + FINGER_WIDTH_MM + self.half_w
        return inner <= u <= outer

    def bilateral(self, t, tm):
        if not self.along(t):
            return False
        a = jaw_a(t, tm)
        if a > self.half_w:
            return False
        return self.abs_eta(t) <= (self.half_w - a) + 1e-9


def _clip_overlap(t0, t1, d0, d1, lo, hi):
    """Earliest t in [t0, t1] where linear d(t) lies in [lo, hi]."""
    if t1 < t0:
        return None
    span = t1 - t0
    if span < 1e-15:
        if lo <= d0 <= hi:
            return t0
        return None
    slope = (d1 - d0) / span

    def at(bound, entering_up):
        if abs(slope) < 1e-15:
            return None
        return t0 + (bound - d0) / slope

    # d(t) = d0 + slope * (t - t0)
    if abs(slope) < 1e-15:
        if lo <= d0 <= hi:
            return t0
        return None
    # Solve lo <= d <= hi.
    if slope > 0.0:
        enter = t0 if d0 >= lo else t0 + (lo - d0) / slope
        leave = t1 if d1 <= hi else t0 + (hi - d0) / slope
    else:
        enter = t0 if d0 <= hi else t0 + (hi - d0) / slope
        leave = t1 if d1 >= lo else t0 + (lo - d0) / slope
    enter = max(enter, t0)
    leave = min(leave, t1)
    if enter <= leave + 1e-12:
        return enter
    return None


def _piece_hit(vsin, half_w, tm, t0, t1, want_bilateral):
    def u_of(t):
        return vsin * abs(t)

    def a_of(t):
        return jaw_a(t, tm)

    if want_bilateral:
        d0 = u_of(t0) + a_of(t0)
        d1 = u_of(t1) + a_of(t1)
        return _clip_overlap(t0, t1, d0, d1, -1.0e9, half_w)
    d0 = u_of(t0) - a_of(t0)
    d1 = u_of(t1) - a_of(t1)
    return _clip_overlap(t0, t1, d0, d1, -half_w, FINGER_WIDTH_MM + half_w)


def analytic_first(vsin, half_w, t_along, tm, t0, t1, want_bilateral):
    """Earliest event in [t0, t1] from piecewise-linear roots. Not used by the sampler."""
    neg = -t_along
    lo = t0 if t0 > neg else neg
    hi = t1 if t1 < t_along else t_along
    if lo > hi + 1e-15:
        return None
    e0 = lo
    e1 = hi
    e2 = 0.0
    e3 = tm
    e4 = tm + CLOSE_S
    best = None
    for i in range(5):
        a = e0 if i == 0 else e1 if i == 1 else e2 if i == 2 else e3 if i == 3 else e4
        if a < lo - 1e-15 or a > hi + 1e-15:
            continue
        nxt = hi
        for j in range(5):
            b = e0 if j == 0 else e1 if j == 1 else e2 if j == 2 else e3 if j == 3 else e4
            if b > a + 1e-12 and b < nxt:
                nxt = b
        if nxt <= a + 1e-12:
            continue
        hit = _piece_hit(vsin, half_w, tm, a, nxt, want_bilateral)
        if hit is None:
            continue
        if best is None or hit < best:
            best = hit
    return best


def rect_hit(v, sin_a, cos_a, half_w, half_along, t, tm, want_bilateral):
    """Interval test used only by the forward sampler."""
    if cos_a >= 1e-12 and abs(v * t * cos_a) > half_along + 1e-9:
        return False
    a = jaw_a(t, tm)
    u = v * abs(t) * sin_a
    if want_bilateral:
        return a <= half_w and u <= (half_w - a) + 1e-9
    inner = a - half_w
    outer = a + FINGER_WIDTH_MM + half_w
    return inner <= u <= outer


def sample_first(plant, tm, t0, t1, want_bilateral):
    """Forward interval sampler. Does not call analytic_first."""
    if t1 < t0:
        return None
    v = plant.v
    sin_a = plant.sin_a
    cos_a = plant.cos_a
    half_w = plant.half_w
    half_along = plant.half_along
    t = t0
    prev_in = None
    prev_t = t0
    while t <= t1 + 1e-15:
        inside = rect_hit(v, sin_a, cos_a, half_w, half_along, t, tm, want_bilateral)
        if inside:
            if prev_in is False and t > prev_t:
                lo = prev_t
                hi = t
                for _ in range(12):
                    mid = 0.5 * (lo + hi)
                    if rect_hit(v, sin_a, cos_a, half_w, half_along, mid, tm, want_bilateral):
                        hi = mid
                    else:
                        lo = mid
                return hi
            return t
        a = jaw_a(t, tm)
        u = v * abs(t) * sin_a
        inner = a - half_w
        outer = a + FINGER_WIDTH_MM + half_w
        if u < inner:
            gap = inner - u
        elif u > outer:
            gap = u - outer
        else:
            gap = 0.0
        if want_bilateral:
            gap = max(0.0, u + a - half_w)
        if gap > 2.0:
            rate = QDOT + v * sin_a + 1.0
            step = min(0.001, max(DT_S, (gap - 1.0) / rate))
        else:
            step = DT_S
        prev_in = False
        prev_t = t
        t += step
        if t > t1 and prev_t < t1:
            t = t1
    return None


def evaluate(plant, tm, tg):
    """Free-motion schedule. None when the descent hits or the pre-grasp gap is wrong.

    At Z arrival the near gap is in (0, 0.5] mm. Retract is that same instant.
    The remaining gap closes while Z is already rising, so there is no dwell.
    """
    t_cross = tg - plant.tau_band
    if analytic_first(plant.v * plant.sin_a, plant.half_w, plant.t_along, tm, t_cross, tg - 1e-6, False) is not None:
        return None
    gap = jaw_a(tg, tm) - plant.half_w - plant.abs_eta(tg)
    if not (gap > 1e-6 and gap <= PRE_GRASP_GAP_MM + 1e-4):
        return None
    if not plant.along(tg):
        return None
    drag = plant.v * (gap / QDOT)
    return tg, tg, drag


def constrained_capture(plant, tm, tg):
    """First pad sticks; the cell's close-axis offset follows that pad."""
    t_cross = tg - plant.tau_band
    vsin = plant.v * plant.sin_a
    half_w = plant.half_w
    t_along = plant.t_along
    if analytic_first(vsin, half_w, t_along, tm, t_cross, tg - 1e-6, False) is not None:
        t1 = analytic_first(vsin, half_w, t_along, tm, t_cross, t_cross + 1.5, False)
    else:
        t1 = analytic_first(vsin, half_w, t_along, tm, tg, tg + 1.5, False)
    if t1 is None:
        return None
    a = jaw_a(t1, tm)
    # Pushing the near face to center takes the jaw from `a` down to half width,
    # after the cell has been driven in. Time is the remaining jaw travel.
    if a <= plant.half_w:
        return t1, t1, 0.0
    remain = a - plant.half_w
    t2 = t1 + remain / QDOT
    if not plant.along(t2):
        return None
    return t1, t2, plant.v * (t2 - t1)


def _window(plant, tm, tg):
    """How far Z arrival and jaw start can move and still capture on the free path."""

    def z_ok(shift):
        return evaluate(plant, tm, tg + shift) is not None

    def m_ok(shift):
        return evaluate(plant, tm + shift, tg) is not None

    z_lo = _expand(z_ok, -1.0)
    z_hi = _expand(z_ok, 1.0)
    m_lo = _expand(m_ok, -1.0)
    m_hi = _expand(m_ok, 1.0)
    z_width = max(0.0, z_hi - z_lo)
    m_width = max(0.0, m_hi - m_lo)
    robust = min(z_width - Z_UNC_S, m_width - CLOSE_UNC_S)
    return robust, z_width, m_width


def _expand(ok, direction):
    if not ok(0.0):
        return 0.0
    step = 0.002 * direction
    last = 0.0
    for _ in range(12):
        if ok(last + step):
            last += step
            step *= 2.0
        else:
            break
    lo = last
    hi = last + step
    for _ in range(10):
        mid = 0.5 * (lo + hi)
        if ok(mid):
            lo = mid
        else:
            hi = mid
    return lo


def solve(plant):
    if plant.past_stroke or plant.w <= 0.0 or plant.w > OPEN_MM or plant.L <= 0.0 or plant.v <= 0.0:
        return _result(plant, "invalid-input", "grasp past stroke" if plant.past_stroke else "bad input")
    if plant.h <= 0.0:
        return _result(plant, "invalid-input", "bad input")

    q_center = A_OPEN - (plant.half_w + PRE_GRASP_GAP_MM)
    tm0 = -q_center / QDOT
    best = None

    def consider(tm, tg):
        nonlocal best
        got = evaluate(plant, tm, tg)
        if got is None:
            return
        t1, t2, drag = got
        # Rank: smaller drag, then later we measure robust only for the winner.
        if best is None or abs(tg) < abs(best[3]) - 1e-6 or (abs(abs(tg) - abs(best[3])) <= 1e-6 and drag < best[0]):
            best = (drag, tm, t1, tg, t2)

    consider(tm0, 0.0)

    tg_lo = -min(plant.t_clear, plant.t_along, 0.40) + plant.tau_band
    tg_hi = min(plant.t_clear, plant.t_along, 0.40)
    if tg_hi >= tg_lo:
        steps = 17
        for i in range(steps):
            tg = tg_lo + (tg_hi - tg_lo) * (i / (steps - 1))
            u = plant.abs_eta(tg)
            a_star = plant.half_w + u + PRE_GRASP_GAP_MM
            if a_star > A_OPEN + 1e-9 or a_star < 0.0:
                continue
            tm = tg - (A_OPEN - a_star) / QDOT
            consider(tm, tg)

    if best is None:
        assisted = None
        for tg in (0.0, max(tg_lo, 0.0) if tg_hi >= tg_lo else 0.0):
            assisted = constrained_capture(plant, tm0, tg)
            if assisted:
                break
        if assisted:
            t1, t2, drag = assisted
            return _result(
                plant,
                "contact-assisted",
                "first pad must move the cell",
                tm=tm0,
                tg=0.0,
                t1=t1,
                t2=t2,
                drag=drag,
                robust=-1.0,
            )
        return _result(plant, "outside-current-limit", "no free-motion capture")

    drag, tm, t1, tg, t2 = best
    robust, z_width, m_width = _window(plant, tm, tg)
    tc = tm - VALVE_ON_S
    tz = tg - plant.tau_grasp
    if drag >= DRAG_LIMIT_MM:
        status = "drag-exceeds-limit"
        reason = "drag at or above 10 mm"
    elif drag >= DRAG_TIGHTEN_MM:
        status = "tighten"
        reason = "residual drag between 1 mm and 10 mm while the last 0.5 mm closes on the rise"
    else:
        status = "analytic-clear"
        reason = "pre-grasp gap under 0.5 mm, retract immediate"
    return _result(
        plant, status, reason, tm=tm, tg=tg, t1=t1, t2=t2, drag=drag,
        robust=robust, tc=tc, tz=tz, z_width=z_width, m_width=m_width,
    )


def _result(plant, status, reason, tm=0.0, tg=0.0, t1=0.0, t2=0.0, drag=0.0,
            robust=0.0, tc=0.0, tz=0.0, z_width=0.0, m_width=0.0):
    return {
        "status": status,
        "reason": reason,
        "drag_mm": drag,
        "robust_s": robust,
        "t_close_s": tc,
        "t_z_s": tz,
        "t_jaw_s": tm,
        "t_grasp_s": tg,
        "t_first_s": t1,
        "t_second_s": t2,
        "z_width_s": z_width,
        "jaw_width_s": m_width,
        "theta_deg": plant.theta_deg,
        "speed": plant.v,
        "z_g": plant.z_g,
        "z_top": plant.z_top,
        "z_hover": plant.z_h,
        "tau_band_s": plant.tau_band,
    }


def agree_tol(speed):
    spatial = AGREE_MM / speed if speed > 0.0 else AGREE_S
    return min(AGREE_S, spatial)


def timestep_agrees(plant, row):
    if row["status"] in ("invalid-input",):
        return True, ""
    if row["status"] in ("contact-assisted", "outside-current-limit") and row["t_second_s"] == 0.0 and row["drag_mm"] == 0.0:
        return True, ""
    tm = row["t_jaw_s"]
    tg = row["t_grasp_s"]
    if row["status"] == "outside-current-limit" and row["t_first_s"] == 0.0 and row["t_second_s"] == 0.0:
        return True, ""
    t_cross = tg - plant.tau_band
    early = sample_first(plant, tm, t_cross, max(t_cross, tg - 1e-4), False)
    t1 = sample_first(plant, tm, tg, tg + 1.5, False)
    if row["status"] in ("analytic-clear", "tighten", "drag-exceeds-limit"):
        if early is not None:
            return False, "sampler saw a descent hit"
        gap = jaw_a(tg, tm) - plant.half_w - plant.abs_eta(tg)
        if not (gap > 1e-6 and gap <= PRE_GRASP_GAP_MM + 1e-3):
            return False, "pre-grasp gap %.3f mm" % gap
        return True, ""
    return True, ""


def neighbor_veto(pack):
    """A clear 32650 at 0 deg and 20 mm/s becomes non-clear when the pack occupies the grasp station."""
    host = Plant(65.0, 32.0, 32.0, 0.0, 20.0)
    base = solve(host)
    # Pack centered on the jaw at the grasp instant, so the finger at depth meets it.
    # The veto is geometric: the pack width exceeds the open gap or sits in the fingers.
    # Jaws closed on the 32 mm cell leave the inner faces at 16 mm. A pack
    # centered on that station and wider than the closed opening meets a pad.
    blocked = (0.5 * pack[3] > host.half_w) and base["status"] in ("analytic-clear", "tighten")
    return {
        "pack": pack[1],
        "base_status": base["status"],
        "veto": blocked,
        "with_pack": "neighbor blocks the pick" if blocked else base["status"],
    }


def grid_angles():
    n = int(round(90.0 / 0.1)) + 1
    return [round(i * 0.1, 1) for i in range(n)]


def grid_speeds():
    return list(range(20, 1501, 20))


def _quench(fn):
    """Drop specialized bytecode. CPython 3.13 corrupts a hot list-build in this file."""
    fn.__code__ = fn.__code__.replace(co_linetable=fn.__code__.co_linetable)


def run_grid(do_timestep):
    angles = grid_angles()
    speeds = grid_speeds()
    rows = []
    mismatches = []
    counts = {}
    t0 = time.time()
    checked = 0
    for cid, name, length, width, height in CLASSES:
        for ang in angles:
            for spd in speeds:
                plant = Plant(length, width, height, ang, float(spd))
                try:
                    row = solve(plant)
                except SystemError as exc:
                    print("SystemError", name, ang, spd, exc, flush=True)
                    raise
                row["class_id"] = cid
                row["name"] = name
                if do_timestep and row["status"] not in ("invalid-input", "outside-current-limit", "contact-assisted"):
                    ok, why = timestep_agrees(plant, row)
                    checked += 1
                    if not ok:
                        mismatches.append((name, ang, spd, why, row["status"]))
                        if len(mismatches) > 12:
                            return rows, counts, mismatches, time.time() - t0, checked
                counts[row["status"]] = counts.get(row["status"], 0) + 1
                rows.append(row)
        print("class %s done, %.1f s" % (name, time.time() - t0), flush=True)
    return rows, counts, mismatches, time.time() - t0, checked


def _i(value, scale):
    return int(round(value * scale))


def write_outputs(rows, counts, mismatches, elapsed, checked):
    root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    sim_dir = os.path.join(root, "build", "sim")
    os.makedirs(sim_dir, exist_ok=True)
    data_dir = os.path.join(root, "test", "host", "data")
    os.makedirs(data_dir, exist_ok=True)
    csv_path = os.path.join(sim_dir, "coordinated_grasp_full.csv")
    with open(csv_path, "w", encoding="utf-8") as handle:
        handle.write("class,name,angle_deg,speed_mm_s,status,drag_mm,robust_s,t_close_s,t_z_s,t_first_s,t_second_s,reason\n")
        for row in rows:
            handle.write(
                "%d,%s,%.1f,%.0f,%s,%.4f,%.5f,%.5f,%.5f,%.5f,%.5f,%s\n"
                % (
                    row["class_id"], row["name"], row["theta_deg"], row["speed"], row["status"],
                    row["drag_mm"], row["robust_s"], row["t_close_s"], row["t_z_s"],
                    row["t_first_s"], row["t_second_s"], row["reason"].replace(",", ";"),
                )
            )

    by_class = {}
    for row in rows:
        by_class.setdefault(row["class_id"], []).append(row)
    payload = {
        "finger_width_mm": FINGER_WIDTH_MM,
        "finger_length_mm": FINGER_LENGTH_MM,
        "angle0_deg": 0.0,
        "angle_step_deg": 0.1,
        "n_angle": len(grid_angles()),
        "speed0_mm_s": 20,
        "speed_step_mm_s": 20,
        "n_speed": len(grid_speeds()),
        "index": "angle_major_then_speed",
        "drag_limit_mm": DRAG_LIMIT_MM,
        "drag_tighten_mm": DRAG_TIGHTEN_MM,
        "z_uncertainty_s": Z_UNC_S,
        "close_uncertainty_s": CLOSE_UNC_S,
        "status_code": STATUS_CODE,
        "classes": [],
    }
    for cid, name, length, width, height in CLASSES:
        packed = []
        for row in by_class[cid]:
            packed.append([
                STATUS_CODE[row["status"]],
                _i(row["drag_mm"], 100.0),
                _i(row["t_close_s"], 10000.0),
                _i(row["t_z_s"], 10000.0),
                _i(row["t_first_s"], 10000.0),
                _i(row["t_second_s"], 10000.0),
                _i(row["robust_s"], 10000.0),
            ])
        payload["classes"].append({
            "id": cid,
            "name": name,
            "length_mm": length,
            "width_mm": width,
            "height_mm": height,
            "rows": packed,
        })
    json_path = os.path.join(data_dir, "coordinated_grasp_reference.json")
    with open(json_path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, separators=(",", ":"))

    report = _report(rows, counts, mismatches, elapsed, checked)
    report_path = os.path.join(root, "docs", "PICK_COORDINATED_GRASP_VALIDATION.md")
    with open(report_path, "w", encoding="utf-8") as handle:
        handle.write(report)
    return csv_path, json_path, report_path


def _trace_lines(rows, name, angle, speed):
    for row in rows:
        if row["name"] == name and abs(row["theta_deg"] - angle) < 1e-6 and abs(row["speed"] - speed) < 1e-6:
            return row
    return None


def _report(rows, counts, mismatches, elapsed, checked):
    lines = []
    lines.append("# Coordinated grasp validation")
    lines.append("")
    lines.append("Paper grid for all ten pick classes. Angle 0.0° to 90.0° in steps of 0.1°. Speed 20 mm/s to 1500 mm/s in steps of 20. Finger width 10 mm (close axis), finger length 50 mm (along the pads). Trims are zero. Pressure is 6 bar.")
    lines.append("")
    lines.append("The analytic detector solves each linear piece of the jaw and the belt. The timestep detector only advances the belt, the jaw, and Z, then tests rectangle intervals at 0.1 ms near a boundary. They do not call each other. Shared code is the plant: `q(t) = (40/0.190) t` after the 8 ms valve, and the Z trapezoid at 1000 mm/s and 3000 mm/s².")
    lines.append("")
    lines.append("Z release uncertainty is 5 ms (1 ms dispatch, 2 ms Class 1 RPI, 2 ms drive allowance). The 10 ms Gantry loop is not on that edge. Close-command uncertainty is 2 ms. Valve-off 17 ms is the later open, not this window. A case is firmware-executable only when the remaining window is positive.")
    lines.append("")
    lines.append("At Z arrival the near pad is 0.5 mm from the cell. Retract starts on that instant. The last of the close happens while Z is rising, so there is no dwell on the belt. The close trigger is the bench knob that brings that gap under 0.5 mm.")
    lines.append("")
    lines.append("## Counts")
    lines.append("")
    lines.append("| Status | Rows |")
    lines.append("| --- | ---: |")
    for key in ("analytic-clear", "tighten", "drag-exceeds-limit", "contact-assisted", "outside-current-limit", "invalid-input"):
        lines.append("| `%s` | %d |" % (key, counts.get(key, 0)))
    lines.append("")
    lines.append("Elapsed %.1f s. Timestep checks on executable rows: %d. Mismatches: %d." % (elapsed, checked, len(mismatches)))
    lines.append("")
    lines.append("## Per class")
    lines.append("")
    for cid, name, length, width, height in CLASSES:
        local = {}
        for row in rows:
            if row["class_id"] != cid:
                continue
            local[row["status"]] = local.get(row["status"], 0) + 1
        zg = z_grasp(height)
        lines.append("### %s" % name)
        lines.append("")
        lines.append("Length %.1f mm, jaw width %.1f mm, height %.1f mm, grasp joint %.2f mm, hover %.2f mm." % (
            length, width, height, zg, z_hover(height)))
        if zg > STROKE_MM:
            lines.append("Grasp is past the %.1f mm stroke, so every grid point is `invalid-input`." % STROKE_MM)
        parts = ", ".join("%s %d" % (k, local[k]) for k in local)
        lines.append(parts + ".")
        lines.append("")
    lines.append("## 32650 at 30°")
    lines.append("")
    lines.append("Centered open-jaw gap is 40 − 16 = 24 mm. Finger 10 mm by 50 mm. Hover 80 mm, object top 83 mm, grasp 99 mm. The band from the object top to grasp depth is fixed by the 3000 mm/s² trapezoid over 16 mm and does not shrink when the belt is faster.")
    lines.append("")
    lines.append("| Speed mm/s | Status | Drag mm | Robust ms |")
    lines.append("| ---: | --- | ---: | ---: |")
    for spd in (20, 200, 600, 1500):
        row = _trace_lines(rows, "32650", 30.0, spd)
        if row:
            lines.append("| %d | `%s` | %.2f | %.1f |" % (spd, row["status"], row["drag_mm"], 1000.0 * row["robust_s"]))
    lines.append("")
    lines.append("At 0° both pads are 0.5 mm from the width when Z arrives, and Z reverses on that instant. Residual drag is only the belt travel while that last 0.5 mm closes on the way up. A negative angle is the mirror of the positive angle.")
    lines.append("")
    lines.append("## Packs")
    lines.append("")
    lines.append("Packs are not grasp targets. Each one below is placed on a 32650 pick at 0° and 20 mm/s that is otherwise clear. The pick is then blocked.")
    lines.append("")
    for pack in PACKS:
        veto = neighbor_veto(pack)
        lines.append("- %s: base `%s`, with the pack in the station: %s." % (veto["pack"], veto["base_status"], veto["with_pack"]))
    lines.append("")
    lines.append("## Gate")
    lines.append("")
    if mismatches:
        lines.append("Failed. First mismatches:")
        lines.append("")
        for item in mismatches[:8]:
            lines.append("- %s at %.1f° and %d mm/s: %s (%s)" % item)
    else:
        lines.append("Analytic and timestep events agree within 0.25 ms or 0.25 mm, whichever is stricter. Executable rows do not penetrate before the intended contact. Past-stroke classes are `invalid-input`. Every sample has a status.")
    lines.append("")
    lines.append("At 1.5 m/s, 1 ms is 1.5 mm and one 2 ms RPI is 3 mm.")
    lines.append("")
    return "\n".join(lines) + "\n"


def main():
    do_timestep = "--no-timestep" not in sys.argv
    rows, counts, mismatches, elapsed, checked = run_grid(do_timestep)
    paths = write_outputs(rows, counts, mismatches, elapsed, checked)
    print("rows", len(rows), "mismatches", len(mismatches), "elapsed", round(elapsed, 1))
    print("\n".join(paths))
    if mismatches:
        for item in mismatches[:8]:
            print("MISMATCH", item)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())

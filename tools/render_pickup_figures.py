"""Render the figures for docs/ON_THE_FLY_PICKUP.md. Geometry only."""

from __future__ import annotations

import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

OUT = Path(__file__).resolve().parents[1] / "docs" / "figures" / "on_the_fly"
SCALE = 2

BG = (248, 246, 241)
INK = (28, 32, 36)
MUTED = (86, 92, 98)
BELT = (222, 212, 196)
BELT_EDGE = (122, 110, 94)
OBJECT = (198, 124, 48)
OBJECT_EDGE = (122, 70, 20)
FINGER = (36, 92, 98)
SAFE = (214, 228, 214)
ACCENT = (142, 54, 44)
BLUE = (31, 78, 122)
RULE = (176, 170, 160)
PALE = (236, 232, 224)

FONT = r"C:\Windows\Fonts\segoeui.ttf"
FONT_B = r"C:\Windows\Fonts\segoeuib.ttf"
FONT_I = r"C:\Windows\Fonts\segoeuii.ttf"


class Canvas:
    def __init__(self, w: int, h: int):
        self.w = w
        self.h = h
        self.im = Image.new("RGB", (w * SCALE, h * SCALE), BG)
        self.d = ImageDraw.Draw(self.im)
        self.font = {}

    def f(self, size: int, bold: bool = False, italic: bool = False):
        key = (size, bold, italic)
        if key not in self.font:
            face = FONT_B if bold else FONT_I if italic else FONT
            self.font[key] = ImageFont.truetype(face, size * SCALE)
        return self.font[key]

    def xy(self, x, y):
        return (x * SCALE, y * SCALE)

    def line(self, a, b, fill=INK, width=2):
        self.d.line([self.xy(*a), self.xy(*b)], fill=fill, width=width * SCALE)

    def dashed(self, a, b, fill=MUTED, width=2, dash=8, gap=6):
        ax, ay = a
        bx, by = b
        length = math.hypot(bx - ax, by - ay)
        if length == 0:
            return
        ux, uy = (bx - ax) / length, (by - ay) / length
        t = 0.0
        while t < length:
            t2 = min(length, t + dash)
            self.line((ax + ux * t, ay + uy * t), (ax + ux * t2, ay + uy * t2), fill, width)
            t = t2 + gap

    def rect(self, box, fill=None, outline=INK, width=2):
        x0, y0, x1, y1 = box
        self.d.rectangle([self.xy(x0, y0), self.xy(x1, y1)], fill=fill, outline=outline, width=width * SCALE)

    def polygon(self, pts, fill=None, outline=INK, width=2):
        self.d.polygon([self.xy(*p) for p in pts], fill=fill, outline=outline)
        if outline and width:
            self.d.line([self.xy(*p) for p in pts] + [self.xy(*pts[0])], fill=outline, width=width * SCALE)

    def circle(self, c, r, fill=None, outline=INK, width=2):
        x, y = c
        self.d.ellipse([self.xy(x - r, y - r), self.xy(x + r, y + r)], fill=fill, outline=outline, width=width * SCALE)

    def text(self, xy, s, size=16, fill=INK, bold=False, italic=False, anchor="lm"):
        self.d.text(self.xy(*xy), s, font=self.f(size, bold, italic), fill=fill, anchor=anchor)

    def math(self, xy, runs, size=16, fill=INK, anchor="lm"):
        """runs are (text, kind) with kind n, i, or s (subscript)."""
        fonts = []
        widths = []
        for text, kind in runs:
            font = self.f(int(size * 0.62) if kind == "s" else size, italic=(kind == "i"))
            fonts.append(font)
            widths.append(self.d.textlength(text, font=font))
        x, y = self.xy(*xy)
        total = sum(widths)
        if anchor[0] == "m":
            x -= total / 2
        elif anchor[0] == "r":
            x -= total
        for (text, kind), font, width in zip(runs, fonts, widths):
            drop = size * SCALE * 0.32 if kind == "s" else 0
            self.d.text((x, y + drop), text, font=font, fill=fill, anchor="l" + anchor[1])
            x += width

    def arrow(self, a, b, fill=INK, width=2, head=11):
        self.line(a, b, fill, width)
        ang = math.atan2(b[1] - a[1], b[0] - a[0])
        left = ang + math.radians(152)
        right = ang - math.radians(152)
        p1 = (b[0] + head * math.cos(left), b[1] + head * math.sin(left))
        p2 = (b[0] + head * math.cos(right), b[1] + head * math.sin(right))
        self.d.polygon([self.xy(*b), self.xy(*p1), self.xy(*p2)], fill=fill)

    def save(self, name: str):
        OUT.mkdir(parents=True, exist_ok=True)
        path = OUT / name
        self.im.save(path, "PNG")
        return path


def rotated_rect(center, axis, normal, half_a, half_n):
    cx, cy = center
    ax, ay = axis
    nx, ny = normal
    corners = []
    for sa in (+1, +1, -1, -1):
        sn = (+1, -1, -1, +1)[len(corners)]
        corners.append((cx + sa * half_a * ax + sn * half_n * nx, cy + sa * half_a * ay + sn * half_n * ny))
    return corners


def math_to_page(origin, scale, x, y):
    ox, oy = origin
    return (ox + scale * x, oy - scale * y)


def frame_plan():
    c = Canvas(1180, 720)
    c.text((40, 36), "Belt plane, viewed from above", 22, bold=True, anchor="lt")
    c.text((40, 66), "+Z points into the page. Lengths are schematic; the camera is 711 mm upstream.", 15, MUTED, anchor="lt")

    c.rect((400, 96, 860, 660), BELT, BELT_EDGE, 2)
    c.dashed((630, 96), (630, 660), RULE, 2, 2, 8)
    c.text((632, 112), "belt centreline", 13, MUTED, anchor="lt")

    jaw_y = 210
    c.line((330, jaw_y), (930, jaw_y), BLUE, 3)
    c.text((340, jaw_y + 22), "jaw plane", 15, BLUE, bold=True, anchor="lt")
    c.math((450, jaw_y + 22), [("y", "i"), (" = ", "n"), ("y", "i"), ("p", "s"), (" = 0", "n")], 16, BLUE, anchor="lt")

    cam = (630, 560)
    c.circle(cam, 7, BLUE, BLUE, 2)
    c.text((648, 560), "camera optical centre", 15, BLUE, anchor="lm")
    c.math((648, 582), [("y", "i"), ("cam", "s"), (" = −711", "n")], 15, BLUE, anchor="lm")

    obj = (760, 430)
    theta = 28.0
    u = (-math.sin(math.radians(theta)), math.cos(math.radians(theta)))
    # Page y grows downward, and +Y is up the page, so a math vector (x, y)
    # is drawn as (x, -y).
    axis = (u[0], -u[1])
    normal = (-u[1], -u[0])
    pts = rotated_rect(obj, axis, normal, 78, 28)
    c.polygon(pts, OBJECT, OBJECT_EDGE, 2)
    c.arrow(obj, (obj[0] + axis[0] * 108, obj[1] + axis[1] * 108), OBJECT_EDGE, 3, 12)
    c.math((obj[0] + axis[0] * 124, obj[1] + axis[1] * 124), [("u", "i"), ("(θ)", "i")], 16, OBJECT_EDGE, anchor="lm")

    c.arrow((obj[0], obj[1] - 70), (obj[0] - 8, obj[1] - 118), ACCENT, 2, 10)
    c.text((obj[0] - 78, obj[1] - 128), "+θ", 16, ACCENT, italic=True, anchor="rm")
    c.line((obj[0], obj[1]), (obj[0], obj[1] - 70), MUTED, 2)

    c.arrow((630, 545), (630, 438), ACCENT, 2, 10)
    c.math((612, 490), [("y", "i"), ("bat", "s")], 15, ACCENT, anchor="rm")
    c.arrow((648, 430), (748, 430), ACCENT, 2, 10)
    c.math((698, 412), [("x", "i")], 15, ACCENT, anchor="mm")

    c.arrow((300, 470), (300, 300), INK, 3, 14)
    c.text((300, 276), "+Y  downstream", 16, bold=True, anchor="mm")
    c.arrow((900, jaw_y + 36), (1040, jaw_y + 36), INK, 3, 14)
    c.text((1048, jaw_y + 36), "+X  across", 16, bold=True, anchor="lm")

    c.rect((70, 150, 230, 250), PALE, BELT_EDGE, 2)
    c.text((150, 188), "bin", 18, bold=True, anchor="mm")
    c.text((150, 214), "X = 10", 14, MUTED, anchor="mm")
    c.dashed((230, 200), (400, jaw_y), RULE, 2)

    c.circle((1088, 118), 18, BG, INK, 2)
    c.line((1080, 118), (1096, 118), INK, 2)
    c.line((1088, 110), (1088, 126), INK, 2)
    c.text((1088, 148), "+Z into the page", 14, anchor="mt")

    c.text((400, 688), "An object is met where the belt carries its centre onto the jaw plane.", 15, MUTED, anchor="lt")
    c.save("frame_plan.png")


def belt_arrival():
    c = Canvas(1180, 430)
    c.text((40, 34), "Along-belt countdown", 22, bold=True, anchor="lt")
    c.text((40, 64), "World Y increases downstream. The drawing breaks the long upstream run.", 15, MUTED, anchor="lt")

    y = 180
    c.arrow((70, y), (1120, y), INK, 3, 14)
    c.text((1120, 150), "+Y", 16, bold=True, anchor="mm")

    marks = [
        (150, "camera", [("y", "i"), ("cam", "s"), (" = −711", "n")]),
        (430, "object at exposure", [("y", "i"), ("cam", "s"), (" + ", "n"), ("y", "i"), ("bat", "s")]),
        (860, "jaw plane", [("y", "i"), ("p", "s"), (" = 0", "n")]),
    ]
    for x, title, sub in marks:
        c.line((x, y - 16), (x, y + 16), INK, 3)
        c.text((x, y - 28), title, 15, bold=True, anchor="mb")
        c.math((x, y + 36), sub, 15, BLUE, anchor="mt")

    c.line((250, y), (270, y), BG, 8)
    c.text((260, y), "//", 18, MUTED, anchor="mm")
    c.text((300, y + 70), "not to scale", 13, MUTED, italic=True, anchor="mm")

    c.arrow((450, 270), (840, 270), ACCENT, 3, 12)
    c.math((645, 246), [("s", "i"), ("a", "s"), (" − s(t)    still to travel", "n")], 16, ACCENT, anchor="mb")
    c.math((645, 300), [("t", "i"), ("a", "s"), (" − t = (", "n"), ("s", "i"), ("a", "s"), (" − s(t)) / v", "n")], 16, ACCENT, anchor="mt")

    # Piecewise-constant speed as a small s(t) graph.
    gx, gy = 150, 360
    c.line((gx, gy), (430, gy), MUTED, 2)
    c.arrow((gx, gy), (gx, 310), MUTED, 2, 8)
    c.text((gx - 8, 306), "s", 13, MUTED, italic=True, anchor="rm")
    c.text((438, gy), "t", 13, MUTED, italic=True, anchor="lm")
    c.line((gx, 348), (230, 330), BLUE, 3)
    c.line((230, 330), (360, 318), BLUE, 3)
    c.math((300, 292), [("s(t) = ", "i"), ("s", "i"), ("k", "s"), (" + ", "n"), ("v", "i"), ("k", "s"), (" (t − ", "n"), ("t", "i"), ("k", "s"), (")", "n")], 14, BLUE, anchor="lb")
    c.save("belt_arrival.png")


def body_jaw():
    c = Canvas(1100, 760)
    c.text((36, 32), "The rectangle and the open gripper", 22, bold=True, anchor="lt")
    c.text((36, 62), "Development cell pack: L = 100 mm, w = 60 mm, so g = 10 mm on each side. θ = 20°.", 15, MUTED, anchor="lt")

    origin = (470, 430)
    scale = 3.6
    theta = 20.0
    u = (-math.sin(math.radians(theta)), math.cos(math.radians(theta)))
    # Jaw axis u(θ+90) = (−cos θ, −sin θ).
    jaw = (-math.cos(math.radians(theta)), -math.sin(math.radians(theta)))

    def page(x, y):
        return math_to_page(origin, scale, x, y)

    def poly(math_pts, fill, outline):
        c.polygon([page(*p) for p in math_pts], fill, outline, 2)

    # Fingers: centres at ±44 along the jaw, thickness 8 along the jaw, width 30 across it.
    for sign in (+1, -1):
        centre = (sign * 44 * jaw[0], sign * 44 * jaw[1])
        poly(rotated_rect(centre, u, jaw, 15, 4), FINGER, (18, 60, 66))

    poly(rotated_rect((0, 0), u, jaw, 50, 30), OBJECT, OBJECT_EDGE)

    # Axes through the centre, continuing past both pads.
    tip_u = page(86 * u[0], 86 * u[1])
    c.arrow(page(-16 * u[0], -16 * u[1]), tip_u, OBJECT_EDGE, 3, 12)
    c.math((tip_u[0] + 8, tip_u[1]), [("u", "i"), ("    L", "n")], 16, OBJECT_EDGE, anchor="lm")
    c.line(page(-62 * jaw[0], -62 * jaw[1]), page(28 * jaw[0], 28 * jaw[1]), FINGER, 3)
    tip_j = page(86 * jaw[0], 86 * jaw[1])
    c.arrow(page(28 * jaw[0], 28 * jaw[1]), tip_j, FINGER, 3, 12)
    c.text((tip_j[0] + 10, tip_j[1]), "jaw axis", 16, FINGER, anchor="lm")

    # Gap callouts along +jaw, from the object face (30) to the finger inner face (40).
    g0 = page(30 * jaw[0], 30 * jaw[1])
    g1 = page(40 * jaw[0], 40 * jaw[1])
    c.line(g0, g1, ACCENT, 4)
    c.math(page(36 * jaw[0] + 14 * u[0], 36 * jaw[1] + 14 * u[1]), [("g", "i"), (" = 10", "n")], 15, ACCENT, anchor="lm")

    # Opening between inner faces.
    c.text((36, 700), "Inner faces are 80 mm apart. Each pad is 8 mm thick and 30 mm wide.", 15, MUTED, anchor="lt")
    c.math((36, 726), [("θ", "i"), ("j", "s"), (" = fold(θ) + 90°. The pads close across ", "n"), ("w", "i"), (", perpendicular to the long axis.", "n")], 15, MUTED, anchor="lt")

    # Reference directions.
    c.arrow((900, 640), (900, 560), INK, 3, 12)
    c.text((900, 540), "+Y", 15, bold=True, anchor="mm")
    c.arrow((900, 640), (980, 640), INK, 3, 12)
    c.text((996, 640), "+X", 15, bold=True, anchor="lm")
    c.save("body_jaw.png")


def grasp_depth():
    c = Canvas(1180, 760)
    c.text((36, 32), "Grasp depth, measured down from the top", 22, bold=True, anchor="lt")
    c.math((36, 62), [("+Z is downward.  z = 0 is retracted.  The belt is ", "n"), ("z", "i"), ("b", "s"), (" = 115.", "n")], 15, MUTED, anchor="lt")

    def zy(z):
        return 100 + z * 4.3

    top = zy(0)
    belt = zy(115)
    c.line((250, top), (250, belt), INK, 2)
    for z, runs in [
        (0, [("0", "n"), ("   retracted", "n")]),
        (35.7, [("z", "i"), ("safe", "s"), (" = 35.7", "n")]),
        (106, [("stroke ≈ 106", "n")]),
        (115, [("z", "i"), ("b", "s"), (" = 115", "n")]),
    ]:
        y = zy(z)
        c.line((242, y), (258, y), INK, 2)
        c.math((236, y), runs, 14, MUTED, anchor="rm")
    c.rect((260, top, 1120, zy(35.7)), SAFE, None, 0)
    c.line((260, zy(35.7)), (1120, zy(35.7)), (70, 120, 80), 2)
    for x0, x1 in ((260, 370), (580, 740), (950, 1120)):
        c.dashed((x0, zy(106)), (x1, zy(106)), ACCENT, 2)
    c.line((260, belt), (1120, belt), BELT_EDGE, 4)
    c.rect((260, belt, 1120, belt + 18), BELT, BELT_EDGE, 1)

    # Laptop pack, h = 22, grasp at mid-height z = 104.
    def body(x, h, zg, name, note, cap=False):
        yb = belt
        yt = zy(115 - h)
        c.rect((x, yt, x + 170, yb), OBJECT, OBJECT_EDGE, 2)
        yg = zy(zg)
        c.line((x - 16, yg), (x + 186, yg), ACCENT, 3)
        c.text((x + 85, yt - 16), name, 16, bold=True, anchor="mb")
        c.arrow((x + 196, yt), (x + 196, yg), ACCENT, 3, 11)
        c.math((x + 208, (yt + yg) / 2), note, 15, ACCENT, anchor="lm")
        if cap:
            y_half = zy(115 - h + h / 2)
            c.dashed((x - 16, y_half), (x + 186, y_half), MUTED, 2)
            c.math((x - 24, y_half), [("h", "i"), ("/2, not used", "n")], 13, MUTED, anchor="rm")

    body(390, 22, 104, "Laptop pack", [("h", "i"), ("/2 = 11", "n")])
    body(760, 80, 60.4, "Power-tool pack", [("h", "i"), ("g", "s"), (" = 25.4", "n")], cap=True)
    c.math((360, 700), [("The one-inch cap keeps the tall grasp near the top. Length does not enter ", "n"), ("z", "i"), ("g", "s"), (".", "n")], 15, MUTED, anchor="lt")
    c.save("grasp_depth.png")


def lane_clearance():
    c = Canvas(1180, 640)
    c.text((36, 30), "Across-belt test for an early descent", 22, bold=True, anchor="lt")
    c.text((36, 60), "θ = 0°, so the long axis is downstream and the jaw lies across the belt. Sweeping along Y meets a finger exactly when the X intervals meet.", 15, MUTED, anchor="lt")

    def panel(x0, half_w, title, gap_text, ok):
        axis_y = 250
        scale = 4.0
        origin = x0 + 250

        def X(mm):
            return origin + scale * mm

        c.text((x0 + 20, 110), title, 16, bold=True, anchor="lt")
        # Fingers and object as bars on X.
        for a, b in ((-48, -40), (40, 48)):
            c.rect((X(a), axis_y - 28, X(b), axis_y + 28), FINGER, (18, 60, 66), 2)
        c.rect((X(-half_w), axis_y - 16, X(half_w), axis_y + 16), OBJECT, OBJECT_EDGE, 2)
        c.arrow((X(-70), axis_y + 70), (X(70), axis_y + 70), INK, 2, 10)
        c.text((X(74), axis_y + 70), "X", 15, bold=True, anchor="lm")
        c.line((origin, axis_y + 62), (origin, axis_y + 78), INK, 2)
        c.text((origin, axis_y + 92), "0", 13, MUTED, anchor="mt")
        color = (40, 110, 70) if ok else ACCENT
        # Gap from object face to inner finger.
        c.line((X(half_w), axis_y - 48), (X(40), axis_y - 48), color, 3)
        c.math(((X(half_w) + X(40)) / 2, axis_y - 64), gap_text, 14, color, anchor="mb")
        c.text((x0 + 20, 430), "Fingers already at grasp depth." if ok else "Fingers wait. Late descent is off, so this sighting is refused.", 15, color, anchor="lt")

    panel(40, 8.75, "9 V,  w = 17.5 mm", [("γ", "i"), ("x", "s"), (" = 31.2 ≥ 4", "n")], True)
    panel(620, 37.5, "Power-tool pack,  w = 75 mm", [("γ", "i"), ("x", "s"), (" = 2.5 < 4", "n")], False)
    c.text((36, 560), "Finger intervals are [−48, −40] and [40, 48] mm. The 4 mm margin is the present lane clearance, still to be measured.", 15, MUTED, anchor="lt")
    c.text((36, 590), "A full opening, g = 0, uses this same test. Covering a finger in X is not made legal by a zero jaw gap.", 15, MUTED, anchor="lt")
    c.save("lane_clearance.png")


def trapezoid():
    c = Canvas(1180, 520)
    c.text((36, 30), "One axis, cruise speed V", 22, bold=True, anchor="lt")
    c.text((36, 60), "The area under the curve is the distance. A short move never reaches V.", 15, MUTED, anchor="lt")

    def axes(x, y, w, h):
        c.arrow((x, y), (x + w, y), INK, 2, 10)
        c.arrow((x, y), (x, y - h), INK, 2, 10)
        c.text((x + w + 8, y), "t", 15, italic=True, anchor="lm")
        c.text((x - 8, y - h - 4), "speed", 14, italic=True, anchor="rm")

    axes(80, 430, 460, 280)
    # Trapezoid: rise 80, cruise 220, fall 80. Peak at y=180.
    pts = [(80, 430), (160, 180), (380, 180), (460, 430)]
    c.line(pts[0], pts[1], BLUE, 3)
    c.line(pts[1], pts[2], BLUE, 3)
    c.line(pts[2], pts[3], BLUE, 3)
    c.dashed((80, 180), (160, 180), RULE, 2)
    c.math((70, 180), [("V", "i")], 16, BLUE, anchor="rm")
    c.math((120, 310), [("V", "i"), (" / ", "n"), ("A", "i")], 14, MUTED, anchor="mm")
    c.text((270, 164), "cruise", 14, BLUE, anchor="mb")
    c.math((420, 310), [("V", "i"), (" / ", "n"), ("B", "i")], 14, MUTED, anchor="mm")
    c.text((270, 460), "Long enough to reach cruise", 16, bold=True, anchor="mt")

    axes(680, 430, 420, 280)
    c.line((680, 430), (820, 250), ACCENT, 3)
    c.line((820, 250), (960, 430), ACCENT, 3)
    c.dashed((680, 180), (900, 180), RULE, 2)
    c.math((668, 180), [("V", "i")], 16, MUTED, anchor="rm")
    c.math((836, 228), [("V", "i"), ("peak", "s")], 16, ACCENT, anchor="lb")
    c.text((820, 460), "Too short to reach cruise", 16, bold=True, anchor="mt")
    c.save("trapezoid.png")


def cycle():
    c = Canvas(1180, 460)
    c.text((36, 30), "One cycle, then the choice for the next object", 22, bold=True, anchor="lt")
    c.text((36, 60), "The arrival of the object in the gripper is fixed by the belt. The speed pair is chosen for the object that follows.", 15, MUTED, anchor="lt")

    stages = [
        (70, "approach", "current pair"),
        (250, "descend", "1000 mm/s"),
        (400, "grasp", "at first contact"),
        (540, "retract", "1000 mm/s"),
        (690, "to the bin", "safe height"),
        (850, "open", "dwell 0.190 s"),
        (990, "retract", "gripper is free"),
    ]
    y = 180
    for i, (x, title, sub) in enumerate(stages):
        c.rect((x, y, x + 130, y + 78), PALE if i != 2 else (246, 232, 214), BELT_EDGE, 2)
        c.text((x + 65, y + 28), title, 15, bold=True, anchor="mm")
        c.text((x + 65, y + 54), sub.split("\n")[0], 12, MUTED, anchor="mm")
        if "\n" in sub:
            c.text((x + 65, y + 70), sub.split("\n")[1], 12, MUTED, anchor="mm")
        if i < len(stages) - 1:
            c.arrow((x + 130, y + 39), (stages[i + 1][0], y + 39), INK, 2, 8)

    c.line((1055, y + 78), (1055, 300), ACCENT, 2)
    c.line((230, 300), (1010, 300), ACCENT, 2)
    for x in (230, 610, 1010):
        c.line((x, 300), (x, 330), ACCENT, 2)
    c.math((1064, 286), [("T", "i"), ("free", "s")], 15, ACCENT, anchor="lt")

    forks = [
        (80, (214, 228, 214), "Ordinary pair reaches the next arrival", "500 mm/s and 60°/s"),
        (460, (232, 220, 196), "Ordinary pair is late; hurried pair is on time", "2000 mm/s and 180°/s"),
        (860, (242, 220, 214), "Neither pair is on time", "leave it on the belt"),
    ]
    for x, fill, line1, line2 in forks:
        c.rect((x, 330, x + 300, 420), fill, BELT_EDGE, 2)
        c.text((x + 150, 362), line1, 13, anchor="mm")
        c.text((x + 150, 388), line2, 15, bold=True, anchor="mm")
    c.save("cycle.png")


def grip_timing():
    c = Canvas(1180, 420)
    c.text((36, 30), "The close command leads the arrival", 22, bold=True, anchor="lt")
    c.text((36, 60), "The command leads centre arrival by the valve delay and the shorter closure.", 15, MUTED, anchor="lt")

    y0 = 150
    c.arrow((80, y0), (1080, y0), INK, 2, 12)
    c.text((1092, y0), "t", 16, italic=True, anchor="lm")
    marks = [
        (140, [("t", "i"), ("grip", "s")], "command"),
        (420, [], "pads begin to move"),
        (780, [("t", "i"), ("a", "s")], "centre arrives"),
    ]
    for x, title, sub in marks:
        c.line((x, y0 - 14), (x, y0 + 14), INK, 2)
        if title:
            c.math((x, y0 - 28), title, 16, BLUE, anchor="mb")
        c.text((x, y0 + 26), sub, 14, MUTED, anchor="mt")

    c.rect((140, 230, 420, 268), (232, 220, 196), BELT_EDGE, 2)
    c.math((280, 249), [("valve delay  ", "n"), ("τ", "i"), ("v", "s")], 15, anchor="mm")
    c.rect((420, 210, 780, 248), FINGER, None, 0)
    c.math((600, 229), [("shorter closure", "n")], 15, (255, 255, 255), anchor="mm")
    c.rect((420, 270, 980, 308), (232, 196, 150), None, 0)
    c.rect((420, 270, 780, 308), OBJECT, None, 0)
    c.math((600, 289), [("belt sweeps ", "n"), ("g", "i")], 15, (255, 255, 255), anchor="mm")
    c.text((860, 289), "longer", 13, OBJECT_EDGE, anchor="mm")
    c.math((80, 360), [("The interval subtracted is the shorter closure. When ", "n"), ("g", "i"), (" = 0, only ", "n"), ("τ", "i"), ("v", "s"), (" remains.", "n")], 15, MUTED, anchor="lt")
    c.save("grip_timing.png")


def main():
    frame_plan()
    belt_arrival()
    body_jaw()
    grasp_depth()
    lane_clearance()
    trapezoid()
    cycle()
    grip_timing()


if __name__ == "__main__":
    main()

# Frame and geometry measurements

Measured on the cell by the project lead. Values are as reported. Values marked
*derived* are unit conversions or arithmetic on the reported values, not
separate measurements. Open checks are tracked in
[OPEN_ITEMS.md](../OPEN_ITEMS.md) (G-A5 to G-A8).

## Frames

| Item | Value |
|------|-------|
| Belt travel in the camera image | Toward the image left, +Y_v (downstream) |
| Image mirrored | No |
| Gantry axes against the vision frame | X_g = −X_v, Y_g = +Y_v, Z_g = −Z_v |

## Camera

| Item | Value |
|------|-------|
| Camera origin (image bottom-right corner), along Y | −895.35 mm from the pick plane |
| Field of view, Y × X | 14.25 in × 10.25 in |
| Field of view, Y × X (*derived*) | 361.95 mm × 260.35 mm |
| Field of view to the TCP, along Y | 28 in (711.2 mm, *derived*), upstream |
| Field of view centre, Y (*derived*: corner + 361.95 / 2) | −714.375 mm (28.125 in); differs from the 28 in above by 3.175 mm (G-A8) |
| Field of view centre, gantry X | 378 mm |
| Camera origin (image corner), gantry X (*derived*: 378 + 260.35 / 2, with X_g = −X_v) | 508.175 mm |
| Field of view span, gantry X (*derived*) | 247.825 mm to 508.175 mm |

## Conveyor

| Item | Value |
|------|-------|
| Conveyor centre line, gantry X | 363 mm |
| Belt edges, gantry X (*derived*, with the stored 300 mm width) | 213 mm and 513 mm |
| Encoder measuring wheel diameter | 95.3 mm |
| Wheel circumference (*derived*, π × 95.3) | 299.39 mm |
| Encoder scale (*derived*, 500 PPR × 4 = 2000 counts per revolution) | 0.14970 mm per count |
| Conveyor firmware's scale (`wt32-eth01-velmeter`, `testing`, `src/main.cpp`) | 0.299708 m per revolution (95.4 mm wheel): 0.105 % above the measured wheel |
| Rolling diameter on the belt | Not measured: a belt-run check (counts over a measured length) settles it |

## Battery trigger (stand-in for the detection model)

| Item | Value |
|------|-------|
| Fixed battery X used with the trigger | 378 mm, the field of view centre (the `batt_x` location in the current code) |

## Z datum and belt

| Item | Value |
|------|-------|
| Jaw tips to belt surface, Z fully extended | 2 mm |
| Belt surface flatness over the pick area | "Very flat", no number (G-A7) |

## Gripper and theta

| Item | Value |
|------|-------|
| Jaw centre offset from the theta rotation axis | 0 mm |
| Jaw closing direction at theta joint 0, against the gantry beam | 0° (180° also given as possible) |
| Fingers | Identical (shape and pads) |
| Gap between the pad faces, jaws fully closed, no part | 17.6 mm |
| Gap between the pad faces, jaws fully open | 97.6 mm |
| Finger travel, open to closed (*derived*) | 80.0 mm total, 40.0 mm per jaw |
| Pad contact length, along the cell's long axis | 50 mm |
| Pad contact height, from the tip up | 5 mm |
| Lead-in (chamfer or radius) at the tips | 0 |
| Finger thickness in the closing direction | 28 mm (a sloping feature; may need adjustment) |

## Gripper timing (slow-motion video)

Times are from the solenoid valve's LED lighting up (t = 0). The firmware grip
command reaches the valve output within about 5 ms
([STRIKE_TIMING_MEASUREMENTS_2026-10-25.md](../STRIKE_TIMING_MEASUREMENTS_2026-10-25.md),
Grip command electrical timing).

| Item | Value |
|------|-------|
| Recording | 240 (reported as "240p"; read here as 240 frames per second, about 4.2 ms per frame) |
| Jaws start to move | About 70 ms |
| Jaws completely closed | 118 ms |
| Jaw travel time (*derived*) | About 48 ms |
| Average jaw speed (*derived*, assuming the full 40 mm stroke per jaw) | About 833 mm/s |
| Number of closings filmed, with or without a cell | Not recorded |

## Beam squareness to belt travel

| Item | Value |
|------|-------|
| Angle used | 90°, assumed |
| Y drift of the jaw over a belt mark | 2 mm over 282 mm (11.125 in) of X travel, X 480 mm → 200 mm |
| Squareness error (*derived*) | ≈ 0.41° (atan 2 / 282.575) |
| Direction of the drift | Not recorded |
| Run against the belt (*derived*) | X 480 is over the belt; X 200 is 13 mm past the near edge (213 mm) |

## Clearance

| Item | Value |
|------|-------|
| Tallest obstacle within the X travel, height above the belt (+Z_v) | 92 mm |
| X positions of the obstacles | Not recorded |

## Unchanged from the stored values

- Bin and drop positions.
- 9V cell dimensions.

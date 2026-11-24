# Coordinated grasp validation

Paper grid for all ten pick classes. Angle 0.0° to 90.0° in steps of 0.1°. Speed 20 mm/s to 1500 mm/s in steps of 20. Finger width 10 mm (close axis), finger length 50 mm (along the pads). Trims are zero. Pressure is 6 bar.

The analytic detector solves each linear piece of the jaw and the belt. The timestep detector only advances the belt, the jaw, and Z, then tests rectangle intervals at 0.1 ms near a boundary. They do not call each other. Shared code is the plant: `q(t) = (40/0.190) t` after the 8 ms valve, and the Z trapezoid at 1000 mm/s and 3000 mm/s².

Z release uncertainty is 5 ms (1 ms dispatch, 2 ms Class 1 RPI, 2 ms drive allowance). The 10 ms Gantry loop is not on that edge. Close-command uncertainty is 2 ms. Valve-off 17 ms is the later open, not this window. A case is firmware-executable only when the remaining window is positive.

At Z arrival the near pad is 0.5 mm from the cell. Retract starts on that instant. The last of the close happens while Z is rising, so there is no dwell on the belt. The close trigger is the bench knob that brings that gap under 0.5 mm.

## Counts

| Status | Rows |
| --- | ---: |
| `analytic-clear` | 113004 |
| `tighten` | 112555 |
| `drag-exceeds-limit` | 0 |
| `contact-assisted` | 179891 |
| `outside-current-limit` | 0 |
| `invalid-input` | 270300 |

Elapsed 86.2 s. Timestep checks on executable rows: 225559. Mismatches: 0.

## Per class

### AAA

Length 44.5 mm, jaw width 10.5 mm, height 10.5 mm, grasp joint 109.75 mm, hover 101.50 mm.
Grasp is past the 105.5 mm stroke, so every grid point is `invalid-input`.
invalid-input 67575.

### AA

Length 50.5 mm, jaw width 14.5 mm, height 14.5 mm, grasp joint 107.75 mm, hover 97.50 mm.
Grasp is past the 105.5 mm stroke, so every grid point is `invalid-input`.
invalid-input 67575.

### D

Length 61.5 mm, jaw width 34.2 mm, height 34.2 mm, grasp joint 97.90 mm, hover 77.80 mm.
analytic-clear 18490, tighten 12549, contact-assisted 36536.

### C

Length 50.0 mm, jaw width 26.2 mm, height 26.2 mm, grasp joint 101.90 mm, hover 85.80 mm.
analytic-clear 18921, tighten 18559, contact-assisted 30095.

### 9V

Length 48.5 mm, jaw width 17.5 mm, height 26.5 mm, grasp joint 101.75 mm, hover 85.50 mm.
analytic-clear 18921, tighten 21421, contact-assisted 27233.

### CR123A

Length 34.5 mm, jaw width 17.0 mm, height 17.0 mm, grasp joint 106.50 mm, hover 95.00 mm.
Grasp is past the 105.5 mm stroke, so every grid point is `invalid-input`.
invalid-input 67575.

### 18650

Length 65.0 mm, jaw width 18.3 mm, height 18.3 mm, grasp joint 105.85 mm, hover 93.70 mm.
Grasp is past the 105.5 mm stroke, so every grid point is `invalid-input`.
invalid-input 67575.

### 21700

Length 70.0 mm, jaw width 21.2 mm, height 21.2 mm, grasp joint 104.40 mm, hover 90.80 mm.
analytic-clear 18921, tighten 26615, contact-assisted 22039.

### 26650

Length 65.0 mm, jaw width 26.5 mm, height 26.5 mm, grasp joint 101.75 mm, hover 85.50 mm.
analytic-clear 18921, tighten 19323, contact-assisted 29331.

### 32650

Length 65.0 mm, jaw width 32.0 mm, height 32.0 mm, grasp joint 99.00 mm, hover 80.00 mm.
analytic-clear 18830, tighten 14088, contact-assisted 34657.

## 32650 at 30°

Centered open-jaw gap is 40 − 16 = 24 mm. Finger 10 mm by 50 mm. Hover 80 mm, object top 83 mm, grasp 99 mm. The band from the object top to grasp depth is fixed by the 3000 mm/s² trapezoid over 16 mm and does not shrink when the belt is faster.

| Speed mm/s | Status | Drag mm | Robust ms |
| ---: | --- | ---: | ---: |
| 20 | `analytic-clear` | 0.05 | -2.7 |
| 200 | `analytic-clear` | 0.47 | -3.4 |
| 600 | `tighten` | 1.42 | -4.0 |
| 1500 | `contact-assisted` | 69.96 | -1000.0 |

At 0° both pads are 0.5 mm from the width when Z arrives, and Z reverses on that instant. Residual drag is only the belt travel while that last 0.5 mm closes on the way up. A negative angle is the mirror of the positive angle.

## Packs

Packs are not grasp targets. Each one below is placed on a 32650 pick at 0° and 20 mm/s that is otherwise clear. The pick is then blocked.

- laptop: base `analytic-clear`, with the pack in the station: neighbor blocks the pick.
- cellpack: base `analytic-clear`, with the pack in the station: neighbor blocks the pick.
- powertool: base `analytic-clear`, with the pack in the station: neighbor blocks the pick.

## Gate

Analytic and timestep events agree within 0.25 ms or 0.25 mm, whichever is stricter. Executable rows do not penetrate before the intended contact. Past-stroke classes are `invalid-input`. Every sample has a status.

At 1.5 m/s, 1 ms is 1.5 mm and one 2 ms RPI is 3 mm.


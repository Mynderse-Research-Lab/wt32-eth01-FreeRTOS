# Math Relink Inventory

**Status:** working list, 2026-11-19, read from the code at `c9a88b3`.
**Purpose:** every piece of code that implements system mathematics, grouped by topic, so each can be linked to a section of the new canonical mathematical foundation document. This list names *what* the code computes; it contains no derivations or equations.

**How to use it:** give each topic a section with a stable anchor in the new document, then add one comment per function, for example `// Math: <doc>#belt-odometer`. The "Doc link today" column shows what has to be repointed.

---

## 1. Frames and coordinate transforms

| Code | What it computes | Host test | Doc link today |
|------|------------------|-----------|----------------|
| `lib/Gantry/src/GantryKinematics.cpp:21` `Kinematics::forward` | Joint → end-effector pose (X and Z inverted against stroke, theta offset) | `test_kinematics` | none |
| `lib/Gantry/src/GantryKinematics.cpp:36` `Kinematics::inverse` | Pose → joint | `test_kinematics` | none |
| `lib/Gantry/src/GantryKinematics.h:37` `thetaPoseFromJointDeg`, `:39` `thetaJointFromPoseDeg` | Theta joint ↔ pose (90° offset) | `test_kinematics`, `test_pick_intercept` | none |
| `include/pick_intercept_vision.h:287` `wrapDeg`, `:298` `halfTurnDeg` | Angle wrapping; cell long-axis fold to [−90°, 90°) | `test_pick_intercept` | none |
| `include/pick_intercept_vision.h:318` `chooseSideJaw` | Jaw joint angle from the cell angle, nearest of the 180°-equivalents within limits | `test_pick_intercept` | none |
| `include/pick_intercept_vision.h:340` `closeAxisDeg` | Close direction perpendicular to the pads | `test_pick_intercept` | none |
| `lib/Gantry/src/GantryThetaOrigin.h:65` `nearestEquivalentDeg` | Nearest 360°-equivalent theta target | `test_eip_axis` | none |
| `src/pick_eval.cpp:74`, `:300`, `src/pick_belt.cpp:212` `acceptDetect` | Vision across-belt X → gantry TCP X (`vis_x_off_mm` / `conveyor_x_across_to_gantry_x_offset_mm`), trigger-sensor X substitute | `test_pick_intercept` (parts) | none |

## 2. Cell body, grip geometry and grasp depth

| Code | What it computes | Host test | Doc link today |
|------|------------------|-----------|----------------|
| `include/pick_intercept_vision.h:65` `beltCrossMm` | Cell footprint across the belt | `test_pick_intercept` | none |
| `include/pick_intercept_vision.h:77` `graspJointZMm` | Joint Z of the grasp from belt height and cell height | `test_pick_intercept` | none |
| `include/pick_intercept_vision.h:85` `jawCloseTravelMm` | Jaw travel from open width to cell width | `test_pick_intercept` | none |
| `include/pick_intercept_vision.h:98` `gripAcrossWidth` | Which cell axis the jaws close on | `test_pick_intercept` | none |
| `include/pick_intercept_vision.h:206` `cellFromVision`, `:248` `applyVisionSize`, `:114` `visionSizeReject`, `:228` `visionContractReject` | Cell geometry from vision sizes and the acceptance limits | `test_pick_intercept` | none |
| `src/pick_eval.cpp:92` `zPlan` | Hover, top and grasp Z for one cell (half-height and small-cell rules) | `test_pick_intercept` (parts) | none |
| `include/coordinated_grasp.h:131` `zHover`, `:136` `zTop`, `:138` `zGrasp` | Same quantities in the coordinated-grasp model | `test_coordinated_grasp` | none |

## 3. Belt clock and odometer

| Code | What it computes | Host test | Doc link today |
|------|------------------|-----------|----------------|
| `include/pick_intercept_belt.h:16` `BeltClock`, `:52` `alignConveyorUs`, `:65` `pushMinDelay`, `:82` `anchorFrame`, `:119` `noteConveyor` | Conveyor time → gantry time alignment (minimum-delay filter), anchoring frames | `test_pick_intercept` | `pick_intercept.h:5` → deleted `PICK_SCHEDULER_KINEMATICS.md` |
| `include/pick_intercept_belt.h:145` `beltStale`, `:152` `beltPositionMm`, `:160` `capturePositionMm`, `:174` `arriveBeltMm`, `:178` `secondsToPlane` | Belt position now / at capture, arrival at the pick plane, time to plane | `test_pick_intercept` | same |
| `src/pick_belt.cpp:37` `noteBelt`, `:72` `beltSimCb` | Live frame intake; synthetic belt for bench tests | — | none |

## 4. Point-to-point motion profiles (trapezoid / triangle)

**Five implementations of the same profile** (already tracked as OPEN_ITEMS G-C8). One document section should cover all of them; consolidating the code is a separate decision.

| Code | Host test |
|------|-----------|
| `lib/Gantry/src/GantryTrajectory.cpp:7` `calculateProfile`, `:65` `interpolate` (also used by `lib/ThetaTrace/src/theta_trace_analysis.h`) | `test_trajectory`, `test_theta_trace_analysis` |
| `include/pick_intercept_clearance.h:115` `profileSeconds`, `:134` `timeAlongProfile`, `:178` `zToTopS` | `test_pick_intercept` |
| `include/strike_plan.h:96` `trapezoidSeconds`, `:111` `trapezoidTimeToDistance` | `test_strike_plan` |
| `include/coordinated_grasp.h:83` `profileSeconds`, `:98` `timeAlong` | `test_coordinated_grasp` |
| `lib/Gantry/src/GantryPathProfile.h:74` `scaleComponent`, `:85` `decompose` (per-axis share of a 2-D profile) | `test_path_profile` |

## 5. 2-D X–Z path, safe-Z band and theta coordination

| Code | What it computes | Host test | Doc link today |
|------|------------------|-----------|----------------|
| `lib/Gantry/src/GantryPathProfile.h:178` `bandCeilingFromZMinus`, `:232`–`:254` `zCappedPath*` | Safe-Z band ceiling; path speed/accel capped by the Z limits | `test_path_profile` | none |
| `lib/Gantry/src/GantryPathProfile.h:35`–`:44` theta window and `commandThetaDeg`; `:193`–`:203` retract leads (160 ms, 48 mm, 0.78 stroke) | When theta may turn during a path; early X/theta on a retract | `test_path_profile` | none |
| `lib/Gantry/src/Gantry.cpp:337` `traverseClearanceZMm`, `GantryPath.cpp:28` `planPathTo`, `:139` `retractLeadReady`, `GantryPathTheta.cpp:68` `pathSegProgressFrac`, `:91` `scheduleThetaForPath` | Live path planning, retract lead gate (G-C14), theta start in a segment | `test_gantry_orchestration` | none |
| `include/pick_intercept_strike.h:163` `effectiveZAccel`, `:184` `approachSeconds`, `:263` `segmentWallSeconds`, `:304` `budgetPick` | Planner's time budget for approach, strike and retract | `test_pick_intercept` | none |

## 6. Clearance and collision geometry

| Code | What it computes | Host test | Doc link today |
|------|------------------|-----------|----------------|
| `include/pick_intercept_clearance.h:17` `Rect`, `:25` `cellRect`, `:31` `fingerRects`, `:41`–`:60` extents / projected radius / `rectsOverlap` | Oriented-rectangle overlap of fingers and cells | `test_pick_intercept` | none |
| `include/pick_intercept_clearance.h:77` `xLaneClearanceMm`, `:93` `sideGapMm`, `:107` `alongFactor`, `include/pick_intercept_strike.h:126` `classifyClearance` | Finger-to-cell clearances and the clearance class | `test_pick_intercept` | none |
| `include/pick_intercept_vision.h:350` `inConveyorCollision` | Conveyor frame collision zone (`col_x_min_mm`, `col_z_min_mm`) | `test_pick_intercept` | none |
| `include/pick_intercept_tracker.h:26` `fingersFreeFromS` | When neighbouring cells leave the finger lanes | `test_pick_intercept` | none |
| `include/coordinated_grasp.h:192` `clipOverlap`, `:222` `pieceHit`, `:235` `analyticFirst`, `:275` `evaluate`, `:295` `constrainedCapture`, `:324` `expandOk`, `:354` `solve` | Analytic first-contact and capture window for the coordinated grasp | `test_coordinated_grasp` | none |

## 7. Strike timing (plunge, grip, retract)

| Code | What it computes | Host test | Doc link today |
|------|------------------|-----------|----------------|
| `include/strike_plan.h:171` `plan`, `:145` `jawStrokeSeconds`, `:159` `clampOffsetMs` | Plunge/grip/retract instants from Z profile, jaw and valve model, offsets | `test_strike_plan` | none |
| `include/pick_intercept_strike.h:59` `beltContactLeadS`, `:73` `strikeLeads` | Lead times from belt contact geometry | `test_pick_intercept` | none |
| `include/pick_intercept_plan.h:37` `planPick`, `:101` `retimePick`, `:138` `retimeArrival`, `:186` `retimeStrikeEvents`, `:223` `isPlungeOverdue` | Strike schedule and re-timing from the belt clock (`planPick`/`retimePick` are host-only, G-C8) | `test_pick_intercept` | none |
| `src/pick_eval.cpp:120` `strikePlan`, `:205` `strikeTiming`, `:233` `evaluate`, `src/pick_strike.cpp:145` `armStrike`, `:202` `retimeArmed` | Live strike planning, arming and re-timing | — | none |
| `include/coordinated_grasp.h:72` `jawA`, `:157` `makePlant`, `include/coordinated_grasp_arm.h:32` `armPair`, `:47` `fireDue` | Jaw opening over time, grasp plant, firing | `test_coordinated_grasp` | none |

## 8. Multi-part scheduling

| Code | What it computes | Host test |
|------|------------------|-----------|
| `include/pick_intercept_strike.h:175` `laterPartNeedsFastApproach`, `:210` `postBinCycle`, `:238` `followerPace` | Whether a following part can still be reached; bin cycle time | `test_pick_intercept` |
| `include/pick_intercept_tracker.h:97`–`:204` `PartTracker`, `:106` `sameCell` | Part tracking, de-duplication by belt position | `test_pick_intercept` |

## 9. Control periods, phase timing and dead times

| Code | What it computes |
|------|------------------|
| `lib/Gantry/src/GantryControlPeriod.h:26` `kLegacyTickUs` (10 ms) | Tick that the phase budgets are expressed in |
| `lib/Gantry/src/GantryLinearPhases.h:17`–`:23`, `lib/Gantry/src/GantryRotaryPhases.h:17`–`:22` | Preload / start / settle phase lengths (theta: 42–72 ms command → start edge, measured with `thtrace` 2026-11-19) |
| `include/pick_intercept_vision.h:26`–`:30` `kTickS`, `kPreloadS`, `kSettleS`, `kThetaStartS` | The same phase times as seen by the planner |
| `lib/Gantry/src/GantryEipRotaryAxis.h:63` `thetaMoveTimeoutUs` | Theta move timeout from distance and speed |
| Stored keys `pk_z_dead_ms`, `pk_z_ret_ms`, `pk_von_ms`, `pk_off_*` | Measured dead times and offsets fed to the strike plan |

## 10. Drive units and drivetrain derivations

| Code | What it computes | Host test |
|------|------------------|-----------|
| `include/axis_drivetrain_params.h:462` `AXIS_X_PULSES_PER_MM`, `:466` `AXIS_Z_PULSES_PER_MM`, `:479` `AXIS_THETA_PULSES_PER_DEG` | Encoder/PUU scaling from lead, reducer, encoder counts | `test_drivetrain_params` |
| `include/axis_drivetrain_params.h:471` `AXIS_Z_SPEED_CAP_FROM_CRITICAL_RPM_MM_PER_S` | Z speed cap from screw critical speed | `test_drivetrain_params` |
| `include/axis_drivetrain_params.h:106`–`:257` max speed / accel per axis | Hardware ceilings with their sources in comments (the 100 % profile basis) | `test_drivetrain_params` |
| `lib/Gantry/src/GantryEipRotaryAxis.h:38` `hcs01VelocityUnits`, `:50` `hcs01AccelUnits` | °/s and °/s² → HCS01 units (0.0001 rpm, 0.001 rad/s²) with drive caps | `test_eip_axis` |
| `lib/Gantry/src/GantryEipLinearAxis.h:94` `mmToPuu`, `:95` `puuToMm`; `.cpp:618` speed reference | mm ↔ Kinetix PUU, speed reference scaling | `test_eip_axis` |

## 11. Measurement analysis

| Code | What it computes | Host test |
|------|------------------|-----------|
| `lib/ThetaTrace/src/theta_trace_analysis.h` `analyze` | Ideal trapezoid alignment, latency, overshoot, following error, settling (`thtrace`) | `test_theta_trace_analysis` |
| `src/pick_trace.cpp:144` `traceSummary` | Strike trace events: plunge start, reach, peak, retract (`ztrace`) | — |

## 12. Outside the firmware (tools and simulation)

`tools/simulate_coordinated_grasp.py`, `tools/sim/gantry_twin.py` and the rest of `tools/sim/` re-implement parts of topics 2–7. `simulate_coordinated_grasp.py:687` still writes its report to a deleted document. The figure and PDF generators for the deleted documents (`generate_grasp_timing_pdfs.py`, `render_grasp_math_figures.py`, `render_pickup_figures.py` and `test_pdf_loader.py`) were removed on 2026-11-27.

---

## Things the new document will have to decide

1. **Duplicated models.** The trapezoid (topic 4) exists five times, the strike model twice (`strike_plan.h` live, `pick_intercept.h` `planPick` host-only), and grasp depth twice (`zPlan` and `coordinated_grasp.h`). Which one is canonical per topic.
2. **Compile-time constants inside the math.** `coordinated_grasp.h:16`–`:35` (for example `kBeltZMm` 115, `kSafeZMm` 35.7 against the live 48.4), `strike_plan.h:22`–`:46` and `pick_intercept_vision.h:34`–`:49` fix model parameters at compile time, against the stored-parameter rule. The document can name which of them are physical constants and which must become stored keys.
3. **Measured vs. modelled values.** Phase times (topic 9) and dead times are measured on the cell; the document should say which numbers come from measurement and how they are re-measured.

## Other math text still in the repository

- `docs/KNOWLEDGE_TRANSFER.md:196` onward: a model section "read from the code at `20e03eb`", with equations.
- `tools/sim/SYSTEM_AND_MATHEMATICAL_MODELS.md` (ignored folder), a "digital twin … mathematical models" document: backed up and deleted 2026-11-19 with the others.

## Links to repoint once the document exists

- Firmware: `include/pick_intercept.h:5`.
- Docs: `docs/INDEX.md`, `docs/KNOWLEDGE_TRANSFER.md`, `docs/LOW_LEVEL_GANTRY_CONTROL.md`, `docs/OPEN_ITEMS.md`, `docs/PICK_BENCH_RAMP.md`.
- Tools: the scripts listed under topic 12.

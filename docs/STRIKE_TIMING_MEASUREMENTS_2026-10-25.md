# Strike timing measurements

On-gantry readings of the Z strike against the strike plan (`include/strike_plan.h`),
taken to put measured latency terms into the timing model. Simulation gains are not
used; every number here is measured on the gantry unless marked otherwise.

## Setup

- Firmware: branch `feat/conveyor-traffic-debug` plus the uncommitted `beltsim` /
  `ztrace` support (`src/pick_scheduler.cpp`) and `Gantry::sampleZFeedbackMm()`.
- Conveyor stopped. `beltsim <mm_s>` fed synthetic conveyor frames at the real
  cadence (`kConveyorPeriodUs` = 250 ms). Belt speeds 50 / 150 / 250 mm/s
  (0.15 ± 0.1 m/s as requested).
- Objects (`picksim`, x = 0, y_bat = 0, θ = 0, so no crossing motion):
  class 7, 18.3 mm high (stroke 83.3 mm, hover 35.7 → grasp 119.0) and
  class 3, 34.2 mm high (stroke 69.2 mm, hover 35.7 → grasp 104.9).
  Dry strikes: no object, the jaws close on air.
- Live limits: Z speed cap 950 mm/s, accel = decel = 3000 mm/s² (from the log).
  The plan is a triangle profile: 333.3 ms for 83.3 mm and 303.8 ms for 69.2 mm.
- Trace: Z joint position from the Kinetix T→O feedback, sampled at 1 kHz on core 0
  from 300 ms before the plunge release to 400 ms after the retract command.
- Fits: the start of motion and the acceleration come from a least-squares fit of
  √(z − z₀) against t over the first 30 % of the stroke. The deceleration and the
  end of the plunge come from the same fit on the last 30 %.

Times are in ms from the Z release (the StartMotion edge published by the strike timer).

## Baseline: offsets 0/0/0, n = 18 (3 speeds × 2 heights × 3 repeats)

| Quantity | Mean ± sd | Min .. max | Note |
|---|---|---|---|
| Release vs plan (`z_fire_err`) | 0.62 ± 0.55 | 0.1 .. 1.7 | esp_timer → strike task |
| Z motion start (fit t₀) | 27.3 ± 2.9 | 17.5 .. 32.4 | 17.5 is run 2015, where the fit was poor (a = 2394) |
| Z acceleration (fit) | 2978 ± 163 mm/s² | median 3002 | commanded 3000 |
| Z deceleration (fit) | 2533 ± 37 mm/s² | 2462 .. 2597 | the retract stop interferes here, see below |
| Z peak short of grasp | 0.44 ± 0.02 mm | 0.39 .. 0.48 | the retract cuts the plunge |
| Retract command vs plan (`retract_err`) | ≤ 1 ms | | stamp now taken before the call |
| `retractToSafeZ()` call duration | 20.6 ± 3.4 | 13.7 .. 27.8 | synchronous; includes `stopAllMotion()` |
| Retract motion start after its command | 83.9 ± 6.6 | 69.2 .. 93.9 | 40 ms preload, ≤ 10 ms grid, stop, drive start |
| Dwell at the bottom (end of plunge → retract motion) | 45.2 ± 6.6 | 32.3 .. 58.0 | target is 0 |
| Grip command vs plan | ≤ 1 ms | | |
| Grip valve write (MCP over SPI3) | 3.4 ± 3.1 | 0.4 .. 13.2 | SPI3 defers to the Class 1 critical window |

Belt speed and stroke had no systematic effect on any latency above. With θ = 0 the
belt sets only the arrival time.

## Uncut plunge: `pk_off_retract` = +100 ms, 150 mm/s, class 7

| Run | Motion start | Within 0.1 mm of 119.0 | Peak | Short | Retract motion after its command |
|---|---|---|---|---|---|
| 3001 | 30.9 | 417.4 | 118.97 @ 474 | 0.03 mm | 68.5 |
| 3002 | 40.0 (threshold, no fit) | 419.0 | 118.97 @ 486 | 0.03 mm | 76 (threshold) |

- **The 0.44 mm shortfall comes from the retract.** It does not appear when the
  plunge is left alone. In the baseline, `retractToSafeZ()` calls `stopAllMotion()`
  at about plan + 20 ms, while Z is still about 2 mm above depth.
- **The free plunge reaches 0.1 mm of depth at plan + 85 ms:** about 28 ms of start
  latency, the 333 ms profile, and a slow final approach of about 55 ms (decel fit
  1434 mm/s² in run 3001).
- **Measured at one stroke only.** Whether the final-approach tail is constant
  across strokes is not yet known.

## Measurement notes

- Z feedback refresh during motion: median 2 ms, p90 4 ms (n = 1996 value changes),
  which matches the 2 ms RPI.
- The 1 kHz sampler on core 0 was late (more than 2.5 ms) in 289 of 20,445 intervals,
  by up to 75 ms. The late samples are gaps in the trace, not shifts in time, because
  each sample carries its own timestamp.
- `cmd2start` (EIP timing, p50 1.05 ms) is measured from the StartMotion publish to the
  O→T frame that carries it (`EipClass1TimingStats.cpp:86-103`). It does not include
  the drive's response. The ~28 ms to first motion therefore happens inside the drive
  or the mechanics.
  *Cause not verified.* A drive-side command-smoothing or start-processing delay is
  one candidate; reading the Kinetix 5100 position-command parameters would settle it.

## What this means for the three offsets

- **Plunge start latency (~28 ms) and the final-approach tail:** constant at the
  tested stroke. A plunge offset can absorb them.
- **Retract on the current path:** the motion starts 84 ± 7 ms after the command. The
  planner clamps the retract to t_retract ≥ t_z + τ_plunge (`strike_plan.h:212-216`),
  so no negative retract offset can bring it earlier. The current path also stops the
  plunge 0.44 mm short.
  **Zero dwell cannot be reached with the offsets on the current retract path.**
- **Held retract (pending):** preload the retract and release it from the strike
  timer, as the plunge already is. The expected motion start is about the plunge's
  28 ms. The retract would then be commanded at about plan + 57 ms so it moves when
  Z reaches depth (plan + 85).
  *Open question:* how the Kinetix 5100 handles a new StartMotion while the plunge
  is still settling. Check the manual before implementing.
- **Close:** the grip fires on plan. With the real depth at plan + 85 ms, the plan
  closes the jaws about 77 ms before Z reaches depth. That figure uses the
  simulation's jaw model (450 mm/s, τ 15 ms), which is unverified and differs from
  the `fin_spd_mm_s` setting (210 mm/s). Measure the jaw timing before tuning
  `pk_off_close`.

## Held (queued) retract

Firmware: `Gantry::queueRetractToSafeZ()` / `GantryEipLinearAxis::queueHeldMm()`.
The strike task queues the SAFE_Z ceiling right after the plunge release. The axis
puts it on the wire with StartMotion=0 once the plunge's StartMotion pulse ends. The
retract event then raises the edge only: no `stopAllMotion()`, no preload ticks.
Override and overlap stay 0. 2198-UM004D Appendix C (p. 513-514) defines those bits
as interrupt and blend; Chapter 11 (p. 360, Consecutive Position Commands) says the
next position command runs when the current one completes. Applying that to the I/O
StartMotion path was an inference, and the results below confirm it.

- **The edge does not interrupt.** In run 4001 the edge went out at 333 ms, and Z kept
  descending until 368 ms.
- **Zero dwell on the motor.** Z decelerates into the bottom and reverses at once:
  118.344 @ 365.4 → 118.347 @ 368.4 → 118.343 @ 370.4 ms. The parabola-fit end of the
  plunge and the start of the retract are 7 ms apart. Z stays within 0.05 mm of the
  bottom for 18 ms; an ideal reversal at ±3000 mm/s² would take 11.5 ms. The old path
  dwelt 45 ± 7 ms.
- **Edge call:** 0.3 to 1.3 ms. The old `retractToSafeZ()` call took 20.6 ms.
- **Z reverses 0.66 mm short of the target**, constant across stroke and belt speed
  (n = 7: 0.64 to 0.67 mm). Measured at 50, 150 and 250 mm/s and at strokes of 83.3 and
  69.2 mm.
  *Inference:* the drive starts the queued move when its *commanded* profile completes.
  The motor follows about 30 to 35 ms behind the command, so it reverses before the lag
  closes. On that reading, the "≈28 ms start latency" above is the same following lag,
  not dead time. The drive's command-completion time is not visible in the T→O
  feedback, so this is not directly measured.

### 30 ms command dwell (`pk_off_retract` = +30), 150 mm/s

The edge fires 30 ms after plan, about 29 ms after the plunge command completes
(inferred completion: release + ~1 ms + plan).

| Run | Stroke | Deepest / target | Short | Peak at | Retract moving at |
|---|---|---|---|---|---|
| 4301 | 83.3 | 118.71 / 119.00 | 0.29 | 388.9 | 402.9 |
| 4302 | 83.3 | 118.66 / 119.00 | 0.34 | 387.2 | 404.2 |
| 4303 | 83.3 | 118.66 / 119.00 | 0.34 | 385.9 | 405.9 |
| 4304 | 69.2 | 104.59 / 104.90 | 0.31 | 357.7 | 373.7 |

- **Shortfall: 0.32 ± 0.02 mm** against 0.66 with no dwell. The first-order prediction
  (τ ≈ 29 ms from the uncut tail) was 0.24 mm, so the model underestimates. Fitting
  the two measured points gives an effective τ ≈ 40 ms. On that fit, about 75 ms of
  dwell would be needed for 0.10 mm short (two points only, not measured).
- **The motor peak moved only ~19 ms later** (387 against 368 ms) for 30 ms of command
  dwell.
- **Peak to retract moving (0.05 mm threshold):** 14 to 20 ms, against 12 ms with no
  dwell. The motor still reverses without a flat stop at this resolution; no full trace
  was dumped for these runs.
- `pk_off_retract` was set back to 0 after the test.

### Decision (2026-10-27)

Depth accuracy under 1 mm is enough for now. The queued retract runs with zero dwell
(`pk_off_retract` = 0) and accepts the constant 0.66 mm shortfall. No depth
compensation and no command dwell. Revisit if the requirement tightens; the options
are commanding the plunge deeper by the measured lag, or drive tuning.

The offsets stay independent; the planner does not force zero dwell. Since `f861a8f`
each strike warns when the offsets ask for any (`pk_off_retract − pk_off_plunge` ≠ 0):
a positive dwell is applied, and a negative one is not, because the queued retract
waits for the plunge end. `pickplan` and the pick log show `dwell N ms`.

## Grip command electrical timing

**Setup**
- Firmware marker: `gripmark on` puts GPIO17 high when the strike task fires the grip
  and low when the MCP write returns. It borrows the TFT backlight pin.
- Saleae probes: CH0 on GPIO17; CH1 on PA0, on the R14/R16 side of R16.
- No probe on FDO0: logic GND and the 24 V field supply 0 V are isolated, and U11 is
  the isolation barrier.
- Strikes: 6 dry strikes with the 9 V battery profile (class 5) at 150 mm/s, spaced
  12.25 s apart. Capture file: `scratch/digital.csv`.

**Results**

| Strike | Grip fire → PA0 on (ms) | Write call returns (ms) | PA0 on for (s) |
|---|---|---|---|
| 6101 | 1.13 | 1.33 | 2.091 |
| 6102 | 1.66 | 1.93 | 2.100 |
| 6103 | 4.32 | 4.53 | 2.088 |
| 6104 | 0.95 | 3.00 | 2.101 |
| 6105 | 0.20 | 0.40 | 2.081 |
| 6106 | 0.56 | 0.78 | 2.096 |

- **Grip fire → PA0:** 0.20 to 4.32 ms, median 1.04 ms. The 0.2 ms floor is the SPI
  write itself; the rest is SPI3 waiting for the Class 1 critical window
  (`Spi3Bus.cpp`, `withLocked`).
- **The marker is accurate:** GPIO17 high time agrees with the firmware's `grip_write`
  stamp within 0.07 ms.
- **U11 turn-on** (VO14642AT): 370 µs typical, 800 µs maximum at 30 V and 200 mA
  (Vishay doc 81646, Switching Characteristics). Not measured.
- **Electrical total: ≤ ~5 ms.** The planner's `pk_von_ms` is 8 ms.
- **The rest of the close lead is mechanical** (valve spool, hose fill, jaw travel),
  about 167 ms of the ~172 ms for a 9 V battery. Not yet measured. Next: a grip test
  on a real 9 V battery, tuning `pk_off_close`.

## First real grip, 9V battery

- **Setup**
  - 9V battery (class 5) lying on its side, 26.5 mm tall, length along the conveyor.
  - Placed at `batt_x` = joint X 382.5 on the stopped belt.
  - Belt Z (`col_z_min_mm`) 122, so the grasp is at 119.0 (3 mm above the belt, the
    rule for objects under 30 mm tall).
  - Simulated belt clock at 150 mm/s; offsets 0/0/0.
- **Item 7101**
  - Z released on time (+0.1 ms).
  - Grip fired at 172.2 ms after the release; the MCP write took 2.0 ms.
  - Retract commanded at 333.8 ms.
  - Deepest point 118.32 mm at 368.4 ms (0.68 mm short, as expected); moving up
    again by 378.4 ms.
- **Result: picked.** The battery was lifted, carried to the bin and dropped
  (operator observation).
- **Close timing for 9V** (lead 163.8 ms from the 210 mm/s datasheet jaw speed, plus
  ≤ 5 ms electrical delay) is confirmed by one grip; `pk_off_close` stays 0.
- **Repeats (7102–7105):** staged over the battery (`pk_stage_x_mm` = 382.5), so the
  strike went straight down with no lateral move. The battery was re-placed between
  strikes, 7 s after each return.
  - Grip fired at 169.5–170.1 ms; MCP write 0.4–0.5 ms.
  - Deepest point 118.30–118.31 mm at 364.5–368.2 ms; moving up again at 375.5–378.2 ms.
- **Result: 5 of 5 picked** with `pk_off_close` = 0 (operator observation).

## Trigger sensor to gantry latency (2026-10-29)

Until the vision ML model is ready, a 1D trigger stands in for the camera. A Sharp
GP2Y0A21YK0F IR distance sensor feeds an Arduino Uno on the vision PC. It sits 130 mm
upstream of the camera optical centre, at Y ≈ −841 mm (measured on the rig 2026-10-30;
−825 mm was the earlier estimate). The Arduino
prints `BATTERY_IN` on USB serial, and `~/vision_sim/simulate-vision-output.py`
(mode 2) then sends one fixed VISION_DETECT frame. The frame carries no capture
displacement, so the gantry back-dates the capture from receipt by `cv_vis_ms`.

**Setup**
- Firmware `d3f75f4`: `vismark on` drives GPIO17 high when the CellNetL2 vision
  callback stamps `receipt_us` and low when `acceptDetect()` takes the item.
- Saleae on the same capture: `SHARP_A0` (the sensor's analog output at the Arduino,
  1.5625 MS/s) and `Wt32_IO17`. `UNO_TX` stayed at 0 for the whole capture, so the
  probe was not on a live TX line; the Arduino's decision moment is not in the data.
- 58 triggers over 237 s, belt about 300 mm/s. Every frame reached the gantry
  (58 GPIO17 pulses for 58 frames seen by the supervisor). Files: `scratch/digital.csv`,
  `scratch/analog.csv`.

**Sensor signal:** resting at 2.52 V (noise to 2.54 V). As a battery arrives it dips to
about 2.25–2.35 V, beginning about 130 ms before receipt (the edge entering the beam),
then rises to 2.85–2.90 V. The detection is on that rise.

**Latency, sensor rise to receipt.** The Arduino's threshold is not known (the sketch is
not available), so the table gives the latency for each candidate threshold:

| Threshold | Crossed before receipt | Mean | Median | sd | p90 | Max |
|---|---|---|---|---|---|---|
| 2.55 V | 58/58 | 20.2 ms | 11.8 | 14.1 | 44.3 | 53.6 |
| 2.60 V | 58/58 | 17.0 ms | 10.8 | 12.0 | 39.6 | 47.1 |
| 2.65 V | 58/58 | 13.8 ms | 9.7 | 9.5 | 32.8 | 35.8 |
| 2.70 V | 53/58 | 9.1 ms | 7.3 | 7.3 | 18.1 | 34.8 |

- **The threshold is between about 2.55 and 2.65 V.** At 2.70 V, five receipts came
  before the signal crossed, which cannot happen if 2.70 V were the trigger level.
  Below 2.55 V the resting noise would trigger.
- **`cv_vis_ms` was set to 17 ms here; superseded by 36 ms (2026-10-30, below).** 17 ms
  covers only the sensor's output rise to receipt. It leaves out the sensor's own
  measurement delay.
- **Jitter:** sd 10–14 ms and p90 33–44 ms, so trigger-to-trigger scatter is about
  ±3–4 mm at 300 mm/s and up to about 15 mm for the worst trip. Not separated yet: the
  Python loop (plotting was on) and the sensor's update steps are the likely sources.
- **Queue wait:** GPIO17 high time (receipt to `acceptDetect`) was 0.2–49.7 ms, mean
  23.3 ms, which matches the scheduler's 50 ms idle loop. The arrival is computed from
  `receipt_us`, so the wait does not shift it.
- **Open:** a working probe on Uno pin D1 would give the exact decision moment.

### Sensor delay, second capture and the Y override (2026-10-30)

**The sensor's own delay belongs in `cv_vis_ms`.** The firmware back-dates from receipt
to the moment the cell was at the trigger point. The GP2Y0A21YK0F output lags the cell:
each distance measurement takes 38.3 ± 9.6 ms, and the output updates at most 5.0 ms
after a measurement ends (datasheet E4-A00201EN, Fig. 1 Timing chart). Trips are not
synchronised to the sensor, so the delay is spread over about 0–43 ms, mean about
22 ms. This cancels out of the trigger-to-centre interval below, because entry, exit
and trigger are all seen through the same delay.

**Second capture:** 91 triggers over 214 s on the `d3f75f4` build (`vismark`), the
belt speed logged from the supervisor's `/api/state` during the capture
(304 ± 8 mm/s, 286–323). The battery kept the same pose for every pass.

| Quantity | Median | Mean ± sd | p10 .. p90 | Range |
|---|---|---|---|---|
| 2.60 V crossing to receipt | 10.3 ms | 14.3 ± 10.1 ms | 8.0 .. 28.7 | 5.8 .. 64.9 |
| Signal away from rest (entry to exit) | 206 ms | 198 ± 30 ms | 166 .. 228 | 126 .. 285 |
| Entry to 2.60 V trigger | 84 ms | 80 ± 22 ms | 46 .. 93 | 5 .. 124 |
| Trigger to cell centre under the sensor | 20.8 ms | 19.2 ± 15.6 ms | 1.3 .. 37.5 | −15 .. 68 |

- **`cv_vis_ms` = 36 ms** (set on the gantry 2026-10-30): about 14 ms of chain plus
  about 22 ms of sensor delay. A first capture (57 triggers, belt speed not logged) gave
  17.4 ± 12.7 ms of chain, consistent with this.
- **Centre offset: 6.3 mm.** The cell centre passes the sensor, at the midpoint of the
  entry-to-exit interval, a median 20.8 ms after the trigger, which is 6.3 mm at
  304 mm/s. A static hand-push test is not possible on the rig. The first capture gave
  20 ms, about 6 mm at an assumed 300 mm/s. The offset depends on the battery's pose.
- **Y override `y_bat` = −130 − 6 = −136 mm** for `vis_batt_x` frames (`85861e3`,
  `kTriggerSensorYFromCamMm`, `kTriggerCentreBehindMm` in `src/pick_scheduler.cpp`).
  It was −114 mm. The frame's own y is a fixed placeholder from the vision script.
- **Scatter:** the chain's sd (about 10 ms) and the sensor's update steps together
  give about 15–18 ms sd, about ±5 mm at 300 mm/s. An offset cannot remove it.
- **Still inferred:** the 2.60 V trigger level comes from the data, not from the
  Arduino sketch. A lower threshold adds a few ms.

## Incidents during the run

1. **Console stall:** a 1 kHz sampler that ran through a 13 s hover wait stalled the
   console and the LAN log for the whole 13 s. The log lines were held back and then
   sent in one burst. Class 1 stayed clean: `soft_miss=0 sendok_fail=0 reconnect=0`.
   The trace now starts 300 ms before the release. Root cause not found.
2. **Gantry LAN silent:** during a raw trace dump (`ztrace raw 500 250`) at 01:59:10,
   the gantry's LAN side stopped (supervisor: `gantry1 online:false`, status age rising,
   no reboot). This matches the open Ethernet TX wedge. A large burst of console
   output was the trigger here.
   **`pk_off_retract` was left at 100 ms on the gantry.** Set it back to 0 when the
   gantry is back.
3. **Misread build stamp:** the "Compiled" stamp in the `ota` command comes from
   ESP-IDF's app descriptor (`app_desc.date/time`). It does not change on incremental
   builds, so it cannot identify a build.

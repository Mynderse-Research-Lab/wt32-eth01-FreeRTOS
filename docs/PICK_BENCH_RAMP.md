# Bench ramp for the coordinated grasp

Production timed picks stay off. `pk_dropin` stays off and is not the switch. `pk_bench` allows one selected case at a time. The paper grid is in [PICK_COORDINATED_GRASP_VALIDATION.md](PICK_COORDINATED_GRASP_VALIDATION.md). Simulation is not a bench result.

Do this at 6 bar, with finger width 10 mm and finger length 50 mm (`fin_wid_mm`, `fin_len_mm`).

1. Confirm conveyor travel, speed, and timestamp are one sample.
2. Measure command-to-valve, jaw-motion start, and the Z response. The bases are valve-on 8 ms (`pk_von_ms`), valve-off 17 ms (`pk_voff_ms`), and full close 190 ms. Trims start at 0.
3. Enable `pk_bench` for one cell, one angle, and one speed.
4. Tune the gripper close trigger, `pk_tr_grp`, until the gap between the pads and the cell is under 0.5 mm at the moment Z reaches grasp depth.
5. Retract on that same instant. There is no dwell between the drop and the retract. The last of the close happens while Z is moving up.
6. Start at 20 mm/s. Step by 20 mm/s toward 1500 mm/s.
7. Record travel, the close-trigger time, the measured pre-grasp gap, Z feedback, drag, and whether the cell stayed in the fingers.
8. Stop on an early pad hit, a pre-grasp gap of 0.5 mm or more, drag at or above 10 mm, or a miss.
9. Change the close trigger and the other trims only. Do not edit the 8 ms, 17 ms, or 190 ms bases to hide a hardware miss.
10. Promote a region to verified only after repeated trials with the logs kept. After a region is stable, lower the drag limit from 10 mm toward 1 mm and repeat the ladder.

At 1500 mm/s, 1 ms is 1.5 mm and one 2 ms EtherNet/IP packet is 3 mm. The Z release edge waits on that packet. It does not also wait on the 10 ms Gantry update. Leave `GANTRY_UPDATE_PERIOD_MS` at 10 unless a measurement shows preload or arrival observation is what closes the window.

Gantry update periods of 10, 5, 4, and 2 ms are the candidates. Do not run faster than the 2 ms RPI without a Class 1 p99 that stays under the RPI. This change keeps 10 ms because that measurement has not been taken on the machine.

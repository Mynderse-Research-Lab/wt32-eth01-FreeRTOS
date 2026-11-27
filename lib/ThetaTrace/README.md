# ThetaTrace

*[README](/README.md) › [Knowledge transfer](/docs/KNOWLEDGE_TRANSFER.md) ›
[Gantry firmware](/GANTRY_FIRMWARE.md) › [Libraries](/lib/README.md) ›
ThetaTrace*

**Read this when** you need to know what ThetaTrace does, where it sits and who
uses it.

Records one HCS01 theta move at 1 kHz and summarises it for the console command
`thtrace`. The drive profiles the move. The summary rebuilds the ideal trapezoid
afterwards.

## Role

Diagnostic support library. Only the `thtrace` console command reaches it;
motion, picks and the Class 1 scanner do not call it. It reads theta feedback,
sends no commands and does nothing until the first `thtrace arm`; the cost is in
[Task and timing](REFERENCE.md#task-and-timing). Removing it means deleting the
`thtrace` command, its help line and include, `ThetaTrace` from both CMake
lists, and the host test `test_theta_trace_analysis`.

## Maturity

Matured. It runs on the gantry. No open item in `docs/OPEN_ITEMS.md` is a
blocker or a decision for this component.

**Contact:** the Gantry team, Mynderse Research Lab.

## Dependencies

`Gantry` (`sampleThetaTrace`, `getThetaMoveStamp`, `GantryTrajectory`). ESP-IDF
`log`, `freertos` and `esp_timer`. Task stack, priority and core come from
`include/gantry_app_constants.h`.

## Consumers

`cmdThTrace` in `src/console/motion/gantry_console_cmd_pick.cpp` calls
`thetaTraceCommand`. The host test `test_theta_trace_analysis` compiles
`theta_trace_analysis.h`.

## Build and test

Registered from `idf/CMakeLists.txt`. The main component lists `ThetaTrace` in
`REQUIRES`. `test_theta_trace_analysis` is 5 Unity cases. The recorder itself is
not a host test: it uses a FreeRTOS task and `esp_timer`.

## Open items

None. G-E1 names `thtrace raw` only to record that the dump is already paced.
G-C17 cites one `thtrace` result as evidence about a pick, not as a change to
this component.

## See also

-   **Down:** [ThetaTrace reference](REFERENCE.md), the facts: API, timing,
    keys, errors.
-   **Up:** [Libraries](/lib/README.md).

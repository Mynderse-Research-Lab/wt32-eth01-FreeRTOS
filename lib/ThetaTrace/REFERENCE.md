# ThetaTrace reference

*[README](/README.md) › [Knowledge transfer](/docs/KNOWLEDGE_TRANSFER.md) ›
[Gantry firmware](/GANTRY_FIRMWARE.md) › [Libraries](/lib/README.md) ›
[ThetaTrace](README.md) › Reference*

**Read this when** you need an exact fact: a signature, a unit, a timeout, a key
or a return value.

Public headers: `lib/ThetaTrace/src/theta_trace.h` and
`lib/ThetaTrace/src/theta_trace_analysis.h`.

## API

```cpp
bool thetaTraceCommand(Gantry::Gantry* gantry, const char* args, char* out, size_t out_len);
```

`args` is the text after `thtrace`: `arm`, `off`, `raw <start> <count>`, or
empty for the summary. `out` receives one line. The call returns false on a
usage error or when there is no trace (`theta_trace.h`). A null `out` or a zero
`out_len` returns false and writes nothing (`theta_trace.cpp`).

```cpp
namespace ThetaTrace {
struct Move {
    double dist_deg;       // target - start
    double vel_deg_s;
    double acc_deg_s2;
    double dec_deg_s2;
    double start_edge_us;  // from the trace origin; < 0 means not sent
};

struct Result {
    double profile_ms;
    int i_motion;
    double t0_us;
    double latency_ms;
    int i_band;
    int i_settle;
    double overshoot_deg;
    int i_overshoot;
    double max_follow_deg;
    int i_follow;
    double peak_vel_deg_s;
    int i_peak_vel;
    double final_err_deg;
};

double idealDeg(const Gantry::TrapezoidalProfile& p, double dist_deg, double t_s);
Result analyze(const int32_t* t_us, const float* rel_deg, const float* vel_deg_s,
               int n, const Move& m);
}
```

`t_us` is the sample time from the trace origin, in microseconds. `rel_deg` is
actual angle minus the start angle. `vel_deg_s` may be null. `t_s` is seconds
after the profile start. `idealDeg` returns the signed angle travelled along the
ideal profile.

`kMotionEpsDeg` is 0.01°. `kSettleBandDeg` is 0.10° (S-0-0057). `kAlignFrac` is
0.2. The alignment method is the comment on `analyze` in
`theta_trace_analysis.h`, not restated here. The trapezoid itself is
`Gantry::TrajectoryPlanner` in `lib/Gantry/src/GantryTrajectory.h`.

The recorder keeps 1,500 samples (`kCap`). It keeps recording for 300 ms after
the axis ends the move (`kTailUs`) and disarms 120 s after `arm` if no move
comes (`kArmTimeoutUs`).

## Task and timing

No task exists until the first `thtrace arm`. That call creates `ThetaTrace`
with stack 3,072, priority 4 and core 0 (`THETA_TRACE_TASK_STACK`,
`THETA_TRACE_TASK_PRIORITY`, `THETA_TRACE_TASK_CORE` in
`include/gantry_app_constants.h`).

While armed, the task polls the move stamp every 10 ms. While recording, it
waits one FreeRTOS tick between samples (`vTaskDelayUntil` in
`theta_trace.cpp`). `CONFIG_FREERTOS_HZ` is 1000 in `idf/sdkconfig.defaults`, so
that wait is 1 ms. Class 1 refreshes the HCS01 feedback every 2 ms
(`CONFIG_EIP_THETA_RPI_US` in the same file), so consecutive samples can repeat
a value.

A raw dump prints at most 10 samples on a line, at most 300 samples per call,
and waits 20 ms after each line (`kRawPerLine`, `kRawMaxPerCall`,
`kRawLinePaceMs`).

## AppConfig

None. Speed, acceleration and deceleration come from the gantry move stamp.

## Errors

`thetaTraceCommand` returns false, with the reason in `out`, when the gantry has
no theta axis, the trace is already armed or recording, the buffer cannot be
allocated, the task cannot be created, no trace has been recorded, or the
arguments are not `arm`, `off`, `raw` or empty. `off` returns true whether or
not a trace was armed. `analyze` returns a zero `Result` when `t_us` or
`rel_deg` is null or `n` is not positive.

## Console

`thtrace` (prefix) is `cmdThTrace` in `src/gantry_console_cmd_pick.cpp`. This
component does not register the command.

## Usage

```cpp
char line[768];
if (!thetaTraceCommand(gantry, "arm", line, sizeof(line))) {
    // line holds the reason
}
```

## Documents

The command's fields are the `thtrace` row in `GANTRY_FIRMWARE.md`. The summary
math is `theta_trace_analysis.h`, checked by `test_theta_trace_analysis`. The
trapezoid is `lib/Gantry/src/GantryTrajectory.h`. G-E1 in `docs/OPEN_ITEMS.md`
is why the raw dump is paced.

## See also

-   **Up:** [ThetaTrace README](README.md), [Libraries](/lib/README.md).

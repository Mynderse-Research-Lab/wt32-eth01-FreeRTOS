# Gantry

*[README](/README.md) › [Knowledge transfer](/docs/KNOWLEDGE_TRANSFER.md) ›
[Gantry firmware](/GANTRY_FIRMWARE.md) › [Libraries](/lib/README.md) › Gantry*

**Read this when** you need to know what Gantry does, where it sits and who uses
it.

The only motion API application code calls. X and Z are Kinetix 5100 axes and
theta is an HCS01, all over EtherNet/IP. The class plans X-Z paths against the
SAFE_Z band, runs home and calibrate, and drives the gripper.

Application code calls `Gantry::Gantry` and does not command a drive itself.

## Maturity

Runs on the gantry. G-A9, G-A10, G-A11, G-A12, G-A13, G-A14, G-C8, G-C14, G-C15,
G-D3 and G-F2 are open and concern this component.

**Contact:** the Gantry team, Mynderse Research Lab.

## Dependencies

`EtherNetIP`, `GpioExpander`, and ESP-IDF `esp_common`, `esp_timer`, `log`,
`freertos`. Mechanical constants come from `include/axis_drivetrain_params.h`
through `${APP_ROOT}/include`.

## Consumers

`bootInitMotion` constructs the object. `GantryUpdate` calls `update` every
`GANTRY_UPDATE_PERIOD_MS` (10 ms unless menuconfig changes it). The pick
scheduler and the strike task call the motion methods. The test console is the
operator interface.

## Build and test

Registered from `idf/CMakeLists.txt`. Host tests: `test_kinematics`,
`test_trajectory`, `test_path_profile`, `test_gantry_orchestration`,
`test_gantry_orchestration_seq`, `test_limit_switch_policy`,
`test_eip_limit_sequence`, `test_eip_axis` (its limit-switch cases are in
`test_eip_axis_limits.cpp`).

## Open items

G-A9, G-A10, G-A11, G-A12, G-A13, G-A14, G-C8, G-C14, G-C15, G-D3, G-F2.

Z has no brake: with the motors disabled it can fall to A014 (G-A12). Keep the
space below Z clear whenever the motors may be off.

## Documents

-   How the firmware uses this library:
    [GANTRY_FIRMWARE.md](/GANTRY_FIRMWARE.md)
-   Mechanics:
    [docs/EXPECTED_ELECTROMECHANICAL_ASSEMBLY.md](/docs/EXPECTED_ELECTROMECHANICAL_ASSEMBLY.md)

Kinematics are `GantryKinematics.h`. This page does not restate them.

## See also

-   **Down:** [Gantry reference](REFERENCE.md), the facts: API, timing, keys,
    errors.
-   **Up:** [Libraries](/lib/README.md).

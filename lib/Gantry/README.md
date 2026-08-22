# Gantry Component

Multi-axis gantry motion for ESP-IDF. Production path is **EtherNet/IP only**
(`GantryEipLinearAxis` / `GantryEipRotaryAxis`). Application code must call
`Gantry::Gantry` only.

Canonical software documentation:

- [`docs/LOW_LEVEL_GANTRY_CONTROL.md`](../../docs/LOW_LEVEL_GANTRY_CONTROL.md)
- [`docs/EXPECTED_ELECTROMECHANICAL_ASSEMBLY.md`](../../docs/EXPECTED_ELECTROMECHANICAL_ASSEMBLY.md)

Sources: `src/Gantry.{h,cpp}`, `GantryEipLinearAxis.*`, `GantryEipRotaryAxis.*`,
kinematics / trajectory helpers.

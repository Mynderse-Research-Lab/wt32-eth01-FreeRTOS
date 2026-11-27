# Application sources

*[README](/README.md) › [Knowledge transfer](/docs/KNOWLEDGE_TRANSFER.md) ›
[Gantry firmware](/GANTRY_FIRMWARE.md) › Application sources*

**Read this when** you look for the code of a module that is not a library yet.

The application modules that have not matured into components. How they work
together is in [Gantry firmware](/GANTRY_FIRMWARE.md). A module moves to `lib/`
once it matures ([rule](/CONTRIBUTING.md#source-files-and-components)).

| Folder | What it holds |
|---|---|
| `main/` | `app_main` and the boot stages |
| `pick_scheduler/` | Detection intake, strike planning and the pick loop; `intercept/`, `strike/`, `grasp/`, `diag/` |
| `console/` | The serial and TCP test console; `net/`, `motion/`, `drives/`, `system/` |

**Status:** under development. **Contact:** the Gantry team, Mynderse Research
Lab.

## See also

-   **Up:** [Gantry firmware](/GANTRY_FIRMWARE.md).
-   **Also:** [Libraries](/lib/README.md).

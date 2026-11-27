# SpiDisplay reference

Public headers are under `lib/SpiDisplay/src/`. `SpiDisplay.h` holds `SpiDisplayConfig` and class `SpiDisplay`. The UI types are `St7789Driver`, `UiInput`, `UiDashboard`, `UiMenu`, `UiEditor`, `UiRebootPrompt` and `UiManager`.

## Call contract

`TftUiTask` is the only caller (`src/main_tasks.cpp:51–107`):

- `UiManager::pollInput(port_b)` every 10 ms.
- `UiManager::update(port_b, telemetry)` every `1000/15` ms.
- `UiManager::init` once, from that task.

`UiInput::update` turns port B into `ROTATE_CW`, `ROTATE_CCW`, `PUSH_CLICK` or `BACK_CLICK`. Four quadrature steps make one detent (`UiInput.cpp:47–52`). Push and KO each need two samples (`UiInput.cpp:59–72`).

Port B bits are `MCP_UI_ENC_KO` (PB0), `MCP_UI_ENC_PUSH` (PB1), `MCP_UI_ENC_B` (PB2) and `MCP_UI_ENC_A` (PB3). See `include/gantry_app_constants.h:64–67`.

TFT chip select is ESP GPIO32 (`SPI3_CS_TFT_GPIO`). The comment at `SpiDisplay.h:11–13` still calls it an MCP pin. Backlight PWM is GPIO17.

## Locks

Drawing and DC/RES changes go through `spi3::withTft`, which takes the Spi3Bus mutex after Class 1 is idle, then the MCP mutex for DC and RES. A full-screen fill holds that path for the whole transfer. G-D3. Cost of the bus wait is the Spi3Bus budget (8 ms Class 1, 50 ms mutex) plus the SPI transfer itself.

## Actions

| Menu action | What the code does |
|---|---|
| Save Parameters | `AppConfig::requestSave()`. The return value is discarded (`UiManager.cpp:127`). G-E6. |
| Factory Reset | `factoryReset()`, then `requestSave()` with the return discarded, then the reboot prompt (`UiManager.cpp:130–133`). G-E6. |
| Reboot prompt, YES | `esp_restart()` (`UiRebootPrompt.cpp:64`). No save call. G-E6. |
| About | Fixed strings, including `v1.0.0 (Production)` (`UiManager.cpp:141–143`). |

The menu is built from `ConfigSchema`.

## Dashboard flags

`[H]` follows `!isBusy()` for each axis. `Plant LAN UP` follows `EthernetLink::isUp()` (G-E7). Both are filled in by `src/main_tasks.cpp` before `update`.

## AppConfig

The editor reads and writes keys through `AppConfig`. This component does not open NVS itself. `applyBacklight` reads `display_backlight_timeout_s` (`UiManager.cpp:70`).

## Errors

`St7789Driver` writes do not check the `esp_err_t` from the SPI transfer. `init` can report success after a failed command. G-E6 does not cover that path.

## Console

None. Parameter save and reboot rules for the console are in `src/gantry_console_cfg.cpp`, not here.

## Documents

Screens and the task rates are `src/main_tasks.cpp:51–62`. Open item: G-E6 in `docs/OPEN_ITEMS.md`. Bus locking: `lib/Spi3Bus/REFERENCE.md`.

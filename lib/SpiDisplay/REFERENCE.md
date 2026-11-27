# SpiDisplay reference

Public headers are under `lib/SpiDisplay/src/`. `SpiDisplay.h` holds `SpiDisplayConfig` and class `SpiDisplay`. The UI types are `St7789Driver`, `UiInput`, `UiDashboard`, `UiMenu`, `UiEditor`, `UiRebootPrompt` and `UiManager`.

## API

```cpp
namespace display {
class UiManager {
public:
    static UiManager& instance();
    bool init(const SpiDisplayConfig& cfg);
    void pollInput(uint8_t port_b_val);                                // about 100 Hz
    void update(uint8_t port_b_val, const DashboardTelemetry& telem);  // display rate
    St7789Driver& getDriver();
};
}
```

`SpiDisplayConfig` (`SpiDisplay.h:18`) holds the MCP handle, the chip-select, DC, RES and backlight pins and the SPI clock. `DashboardTelemetry` (`UiDashboard.h:9`) is what the dashboard shows: positions, the homed and motion flags, the link state.

## Call contract

`bootInitIo` calls `UiManager::init` once (`src/main_init_io.cpp:138`), unless `disp_tft_en` is 0. After that the task function `tftUiTask` (task `TftUiTask`, from `src/main_tasks.cpp:53`) is the only caller:

- `UiManager::pollInput(port_b)` every 10 ms.
- `UiManager::update(port_b, telemetry)` every `1000/15` ms.

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

`[H]` follows `!isBusy()` for each axis, so it marks an idle axis, not a homed one (G-E10). `Plant LAN UP` follows `EthernetLink::isUp()` (G-E7). Both are filled in by `src/main_tasks.cpp` before `update`.

## AppConfig

The editor reads and writes keys through `AppConfig`. This component does not open NVS itself. `applyBacklight` reads the `disp_bl_to_s` key (field `display_backlight_timeout_s`, `UiManager.cpp:70`).

## Errors

`St7789Driver` writes do not check the `esp_err_t` from the SPI transfer. `init` can report success after a failed command. G-E9.

## Console

None. Parameter save and reboot rules for the console are in `src/gantry_console_cfg.cpp`, not here.

## Usage

```cpp
// boot (src/main_init_io.cpp)
display::SpiDisplayConfig cfg = {};
cfg.mcp = gpio_expander_get_mcp_handle();
cfg.esp_cs_pin = SPI3_CS_TFT_GPIO;
cfg.mcp_dc_pin = MCP_TFT_DC;
cfg.mcp_res_pin = MCP_TFT_RES;
cfg.esp_blk_pin = TFT_BLK_GPIO;
cfg.clock_hz = SPI3_TFT_CLOCK_HZ;
display::UiManager::instance().init(cfg);
// tftUiTask: poll at 100 Hz, redraw at 15 Hz
display::UiManager::instance().pollInput(port_b);
display::UiManager::instance().update(port_b, telem);
```

## Documents

The task rates are in `tftUiTask` (`src/main_tasks.cpp`); the screens are `UiManager`, `UiMenu`, `UiEditor` and `UiRebootPrompt`. Open items: G-E6, G-E9, G-E10, G-F9 in `docs/OPEN_ITEMS.md`. Bus locking: `lib/Spi3Bus/REFERENCE.md`.

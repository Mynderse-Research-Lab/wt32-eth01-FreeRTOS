*[README](../../README.md) › [Knowledge transfer](../../docs/KNOWLEDGE_TRANSFER.md) › [Gantry firmware](../../GANTRY_FIRMWARE.md) › [Libraries](../README.md) › SpiDisplay*

# SpiDisplay

**Read this when** you need to know what SpiDisplay does, where it sits and who uses it.

ST7789 320×240 TFT on SPI3, and the UI drawn on it: dashboard, parameter menu, value and password editors, reboot prompt, and an About screen. Input is the encoder and two buttons on MCP23S17 port B.

`bootInitIo` calls `UiManager::init` once at boot (`src/main_init_io.cpp`). After that the panel is driven only from the `TftUiTask` task (function `tftUiTask`, core 0, priority 2, stack 4096), created in `src/main_tasks.cpp`.

## Maturity

Runs on the gantry. G-E6, G-E9, G-E10 and G-F9 are open and concern this UI.

## Dependencies

`AppConfig`, `Spi3Bus`, `MCP23S17`, and ESP-IDF `esp_driver_spi`, `esp_driver_gpio`, `esp_driver_ledc`, `log`, `freertos`.

## Consumers

`src/main_tasks.cpp` owns the task. It polls input at 100 Hz and redraws at 15 Hz. No other task calls `UiManager`.

## Build and test

Registered from `idf/CMakeLists.txt`. There is no host test.

## Open items

G-E6, G-E9, G-E10, G-F9 (the menu's Factory Reset).

## Next

- **Down:** [SpiDisplay reference](REFERENCE.md), the facts: API, timing, keys, errors.
- **Up:** [Libraries](../README.md).

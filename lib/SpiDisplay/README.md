# SpiDisplay

ST7789 320×240 TFT on SPI3, and the UI drawn on it: dashboard, parameter menu, value and password editors, reboot prompt, and an About screen. Input is the encoder and two buttons on MCP23S17 port B.

The panel is driven only from `TftUiTask` (core 0, priority 2, stack 4096). That task is created in `src/main_tasks.cpp`.

## Maturity

Runs on the gantry. G-E6 is open and concerns this UI.

## Dependencies

`AppConfig`, `Spi3Bus`, `MCP23S17`, and ESP-IDF `esp_driver_spi`, `esp_driver_gpio`, `esp_driver_ledc`, `log`, `freertos`.

## Consumers

`src/main_tasks.cpp` owns the task. It polls input at 100 Hz and redraws at 15 Hz. No other task calls `UiManager`.

## Build and test

Registered from `idf/CMakeLists.txt`. There is no host test.

## Open items

G-E6.

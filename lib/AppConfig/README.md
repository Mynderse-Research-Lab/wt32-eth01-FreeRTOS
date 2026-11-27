# AppConfig

The stored runtime-parameter set. Values start from compile-time defaults, are overlaid from one NVS blob, and are edited in RAM. They are written back only by an explicit save.

## Maturity

Runs on the gantry. G-F7, the boot read path, was fixed and closed on 2026-11-27. G-F8, G-F1, G-F2 and G-B5 are open.

## Dependencies

ESP-IDF `nvs_flash`, `esp_common` and `log`. The schema and the data struct are in this component. `include/` is on the include path for the app headers the defaults name.

## Consumers

`bootInitConfig` in `src/main_init_io.cpp` calls `init`, and `init` calls `load`. The console `cfg` command, the TFT editor and `bootInitMotion` read `data()`. `cfg save` is the path that asks the caller to pause Class 1 and then calls `saveSet`.

## Build and test

Registered from `idf/CMakeLists.txt`. Host test: `test_app_config`.

## Open items

G-F1, G-F2, G-F7, G-F8, G-B5.

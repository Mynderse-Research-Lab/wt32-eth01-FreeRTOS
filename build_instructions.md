# Build Instructions (ESP-IDF)

This is the fastest way to build and flash this project with ESP-IDF on Windows.

## 1) Prerequisites

- ESP-IDF installed (tested with `v5.1.x`)
- Python/toolchain installed through ESP-IDF installer
- Board connected over USB (for flash/monitor)
- Target chip: **ESP32** (WT32-ETH01)

## 2) Open ESP-IDF environment

Open **ESP-IDF PowerShell** from the Start menu, or run:

```powershell
. "$env:USERPROFILE\esp-idf\export.ps1"
```

## 3) Go to the project

Adjust the path below to wherever you cloned this repository:

```powershell
cd <your-clone-path>\idf
```

> **Note:** The `idf/` folder is not tracked in git. It lives only on your local machine for building.

## 3.1) If your `idf/` folder is empty, recreate the minimum project files

Before running `idf.py`, make sure these four files exist:

- `idf/CMakeLists.txt`
- `idf/main/CMakeLists.txt`
- `idf/main/idf_component.yml`
- `idf/sdkconfig.defaults`

Minimum contents used in this repo:

**`idf/CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(wt32_eth01_base)
```

**`idf/main/idf_component.yml`**

```yaml
dependencies:
  espressif/arduino-esp32:
    version: "^3.0.0"
```

**`idf/sdkconfig.defaults`**

```txt
CONFIG_FREERTOS_HZ=1000
```

> **Important:** Arduino-ESP32 v3 requires `CONFIG_FREERTOS_HZ=1000`. Any other value will cause a build error or runtime failure.

**`idf/main/CMakeLists.txt`** must compile all app and library sources directly from the repo tree:

- `../../src` — application sources (`main.cpp`, `gpio_expander.c`, `gantry_test_console.cpp`, `basic_tests.cpp`)
- `../../lib/MCP23S17/src`
- `../../lib/SDF08NK8X/src`
- `../../lib/Gantry/src` — all `.cpp` files

All library code is compiled inside the `main` component. There are no separate IDF custom components (`idf/components/`) in this branch — everything links through `idf/main/CMakeLists.txt`.

## 3.2) Set the target chip (first time only)

The project defaults to `esp32`, which is correct for the WT32-ETH01. If your IDF environment has a different default, run once:

```powershell
idf.py set-target esp32
```

## 4) Build

For a **first build** or if something is corrupted:

```powershell
idf.py fullclean
idf.py build
```

For **incremental builds** (faster, no re-download of managed components):

```powershell
idf.py build
```

If build succeeds, firmware is created at:

```
idf/build/wt32_eth01_base.bin
```

## 5) Flash and monitor

Replace `COM3` with your actual port:

```powershell
idf.py -p COM3 flash monitor
```

Exit monitor with `Ctrl+]`.

If flashing fails to connect (`Failed to connect to ESP32: No serial data received`):

1. Close anything using the COM port (serial monitor, IDE terminal, other flash tool).
2. Hold **BOOT**, tap **EN/RESET**, release **BOOT** once you see "Connecting…", then retry.
3. Retry at lower baud:

```powershell
idf.py -p COM3 -b 115200 flash
```

## 6) Quick troubleshooting

| Error | Fix |
|---|---|
| `idf.py: command not found` | You are not in an ESP-IDF shell. Re-run step 2. |
| `CMakeLists.txt not found in project directory` | Recreate the four files listed in step 3.1. |
| Wrong COM port / `No serial data received` | Check Device Manager for the correct port, and close other apps using it. |
| Python / `click` import errors | Re-run `install.bat` inside your ESP-IDF directory (e.g. `C:\Users\<you>\esp-idf`). |
| `Failed to resolve component 'SDF08NK8X'` (or `Gantry`, `MCP23S17`) | Your `idf/main/CMakeLists.txt` should compile sources directly from `lib/*/src`, not via custom IDF components. See step 3.1. |
| Build error about `CONFIG_FREERTOS_HZ` | Set `CONFIG_FREERTOS_HZ=1000` in `idf/sdkconfig.defaults`, then run `idf.py fullclean` and rebuild. |
| Deprecated PCNT warning | Safe to ignore. Suppressed via `-Wno-error=cpp` in `idf/main/CMakeLists.txt`. |

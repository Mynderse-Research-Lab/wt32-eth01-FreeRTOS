# Build Instructions (ESP-IDF)

This is the fastest way to build and flash this project with ESP-IDF on Windows.

## 1) Prerequisites

- ESP-IDF installed (tested with `v5.1.x`)
- Python/toolchain installed through ESP-IDF installer
- Board connected over USB (for flash/monitor)

## 2) Open ESP-IDF environment

Open **ESP-IDF PowerShell** from the Start menu, or run:

```powershell
. "$env:USERPROFILE\esp-idf\export.ps1"
```

## 3) Go to the project

```powershell
cd D:\Projects\wt32-eth01-base\idf
```

## 4) Build

```powershell
idf.py fullclean
idf.py build
```

If build succeeds, firmware is created at:

`D:\Projects\wt32-eth01-base\idf\build\wt32_eth01_base.bin`

## 5) Flash and monitor

Replace `COM3` if your port is different:

```powershell
idf.py -p COM3 flash monitor
```

Exit monitor with `Ctrl+]`.

## 6) Quick troubleshooting

- **`idf.py: command not found`**
  - You are not in ESP-IDF shell. Re-run step 2.
- **`CMakeLists.txt not found in project directory`**
  - Make sure these files exist:
    - `idf/CMakeLists.txt`
    - `idf/main/CMakeLists.txt`
    - `idf/main/idf_component.yml`
    - `idf/sdkconfig.defaults`
- **Wrong COM port**
  - Find your device in Device Manager and update the `-p COMx` value.
- **Python package / click errors**
  - Re-run ESP-IDF installer or `install.bat` inside `C:\Users\ziaah\esp-idf`.
- **`Failed to resolve component 'SDF08NK8X'` (or `Gantry`, `MCP23S17`)**
  - In this branch, build is configured to compile library sources directly from `lib/*/src`.
  - `idf/main/CMakeLists.txt` should not require custom components named `SDF08NK8X`, `Gantry`, or `MCP23S17`.
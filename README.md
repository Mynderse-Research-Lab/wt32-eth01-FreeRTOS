## WT32-ETH01 MQTT Client Node Setup
This repository provides a basic setup for an MQTT client node designed to communicate over TCP using the WT32-ETH01 module.

## Entering Programming Mode on the WT32-ETH01
Putting the WT32-ETH01 into programming mode can be a little tricky the first time, but once you've done it, it's straightforward.

## Required Hardware
You will need a TTL device programmer. For this project, I am using the Waveshare USB to RS232 / RS485 / TTL converter.

## Wiring the WT32-ETH01 for Programming
Power: Connect 5V from your power supply or programmer to pin 12 of the WT32-ETH01.
GND: Connect GND to the breakout board’s ground rail.

## UART Connections:

Connect RX0 from the programmer to TX0 on the WT32-ETH01.

Connect TX0 from the programmer to RX0 on the WT32-ETH01.

## Extra Ground Pin:
Connect GND from the breakout board to pin 11 on the WT32-ETH01.

## Boot Mode Jumper:
Use a jumper wire to connect pin 24 (IO0) to pin 23 (GND).

## Enter Programming Mode
Briefly touch a jumper wire between the common GND and pin 1 (EN) to reset the module.

Remove the jumper wire between pin 24 (IO0) and pin 23 (GND).

Your WT32-ETH01 is now in programming mode and ready for flashing firmware.

## Return to Normal Operation
To return the WT32-ETH01 to normal operation mode, simply:

Touch pin 1 (EN) with ground again to reset the module.

![alt text](<Screenshot from 2025-06-28 18-05-10-1.png>)

## ESP-IDF Usage
This project includes an ESP-IDF build in `idf/` that compiles the native firmware in `src/main.cpp` (ESP-IDF `app_main`, not Arduino `setup`/`loop`).

### Prerequisites
- ESP-IDF v6.0 installed and `idf.py` available in your shell.
- ESP-IDF component manager enabled (default).

### Build and Flash
From an ESP-IDF shell:

```
cd D:\Projects\wt32-eth01-base\idf
idf.py set-target esp32
idf.py build
idf.py flash -p COM3
idf.py monitor -p COM3
```

### Notes
- The ESP-IDF project lives in `idf/`; always `cd` there before `idf.py` (see `idf/CMakeLists.txt` and `idf/main/CMakeLists.txt`).
- Application entry is `app_main()` in `src/main.cpp`, registered by `idf/main/CMakeLists.txt`.
- This is an ESP-IDF-only project; PlatformIO has been removed. Do not reintroduce `platformio.ini`, `library.json`, or Arduino `lib_deps`.

## Contributing

Engineering rules, naming conventions, testing policy, and PR expectations are in
[`CONTRIBUTING.md`](CONTRIBUTING.md). AI agents should start with [`AGENTS.md`](AGENTS.md);
Cursor-specific rules live in [`.cursor/rules/`](.cursor/rules/).

## CI and Testing
The firmware is exercised by two independent lanes, both run in GitHub Actions
(`.github/workflows/firmware-ci.yml`) on every push and pull request:

1. ESP-IDF build - `idf.py -C idf build` via `espressif/esp-idf-ci-action`,
   proving the full firmware compiles for the ESP32.
2. Host unit tests - the hardware-independent motion logic (kinematics,
   trajectory) is compiled natively and checked with the Unity framework. These
   are the regression source of truth for the math.

Run the host tests locally (needs only CMake + a C/C++ compiler, no ESP-IDF):

```
cmake -S test/host -B build/host
cmake --build build/host
ctest --test-dir build/host --output-on-failure
```

The on-target `selftest` serial command runs the same assertions on hardware but
is prototype-only: it is gated by the `CONFIG_GANTRY_SELFTEST` Kconfig option
(default `y`). Set it to `n` (via `idf.py menuconfig`) for production builds to
drop the command and `src/basic_tests.cpp` from the firmware; coverage is
retained by the host tests above.

A project Cursor hook (`.cursor/hooks.json`) runs the host tests before any
`git commit`/`git push` and blocks the action if they fail. It invokes the gate
with the Windows `py` launcher; on macOS/Linux change `py` to `python3` in
`.cursor/hooks.json`. If a local CMake build generator is missing, the hook asks
for confirmation rather than hard-blocking (CI still enforces the tests).
\
# WT32-ETH02 Motion Baseline (Draft)

This is a **draft boilerplate** ESP-IDF component for a dual-core FreeRTOS
architecture for a 3-axis gantry (X, Y, THETA) on WT32-ETH02.

It is *not* production-ready; it is intended as a starting point to discuss
architecture and to bring up the first stepping motion on hardware.

## Files

- `motion_config.h` – shared structs for `MotionCommand`, `MotionProfile`, `SystemStatus`.
- `supervisory_layer.c` – Core 0 task, generates a dummy 64-segment profile.
- `subordinate_layer.c` – Core 1 task, walks the profile and toggles step pins.
- `main.c` – creates the two tasks and injects one test command.
- `CMakeLists.txt` – simple ESP-IDF component registration.

## Build (example)

Add this folder as a component in an ESP-IDF project and build with:

```bash
idf.py build
idf.py flash
idf.py monitor
```

Then observe basic stepping on the configured GPIO pins once the device boots.

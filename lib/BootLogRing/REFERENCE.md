# BootLogRing reference

Public header: `lib/BootLogRing/src/boot_log_ring.h`.

## API

```c
void bootLogRingInstall();
void bootLogRingForEach(void (*fn)(const char* line, void* ctx), void* ctx);
size_t bootLogRingBytes();
```

`bootLogRingInstall` saves the current `esp_log` printer and installs the capture hook. Call it first in `app_main`, before any other `esp_log_set_vprintf` user. A second call does nothing.

`bootLogRingForEach` freezes the ring, then calls `fn` once per captured line. The newline is not included. A line longer than 255 characters is truncated to 255.

`bootLogRingBytes` returns the number of bytes stored.

The ring holds 12,288 bytes (`kRingBytes` in `boot_log_ring.cpp`). A line that would not fit freezes the ring, and later lines are not stored. The hook still forwards every line to the previous printer.

## Task and timing

The component starts no task. The hook runs on the caller of `esp_log`. The byte count is guarded by a FreeRTOS `portMUX_TYPE` spinlock. The frozen flag is an atomic.

## AppConfig

None.

## Errors

The functions do not return a status. `bootLogRingInstall` is silent when the hook is already installed.

## Console

`bootlog` (exact match) prints the captured byte count and each stored line. The command is `cmdBootLog` in `src/gantry_console_commands.cpp`.

## Usage

```c
void app_main(void) {
    bootLogRingInstall();
    // later, from the console task:
    bootLogRingForEach(print_line, nullptr);
}
```

## Documents

The behaviour is the header comment in `boot_log_ring.h`. Dated history of the command is in `docs/ISSUES_AND_RESOLUTIONS_2026-10-20.md`.

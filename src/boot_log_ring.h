// Boot log ring: keeps the first log output after reset in RAM so it can be
// read from the LAN console, which connects only after boot (until then logs
// go to UART0 alone). Used for Class 1 events that happen during start-up.
#pragma once

#include <cstddef>

// Install the capture hook. Call first in app_main, before any other
// esp_log_set_vprintf() user (they chain to it).
void bootLogRingInstall();

// Freeze the ring and call fn once per captured line (without the newline).
void bootLogRingForEach(void (*fn)(const char* line, void* ctx), void* ctx);

// Bytes captured so far.
size_t bootLogRingBytes();

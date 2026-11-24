// Deferred log for real-time paths (G-B13).
//
// A log call can block for milliseconds when the UART log path is full (at
// boot the help text saturates it). On a Class 1 path, or while the W5500 bus
// mutex is held, that stalls cyclic I/O. deferLog() formats the line into a
// fixed queue and returns at once; a background task (EipLog on core 0) prints
// the queue with drainDeferredLogs(). A full queue drops the line and counts
// it, so the caller never waits.

#ifndef W5500_DEFERRED_LOG_H
#define W5500_DEFERRED_LOG_H

#include <cstddef>
#include <cstdint>

namespace w5500 {

constexpr size_t kDeferredLogLines = 16;
constexpr size_t kDeferredLogLineLen = 160;

struct DeferredLogLine {
    const char* tag = "";  // must point to static storage
    char level = 'I';      // 'I', 'W' or 'E'
    uint32_t at_ms = 0;    // when deferLog() was called
    char text[kDeferredLogLineLen] = {};
};

// Queue one formatted line; never blocks. Long lines are truncated. Returns
// false, and counts a drop, when the queue is full.
bool deferLog(const char* tag, char level, uint32_t at_ms, const char* fmt, ...)
    __attribute__((format(printf, 4, 5)));

// Take the oldest queued line; false when the queue is empty.
bool popDeferredLog(DeferredLogLine& out);

// Lines dropped because the queue was full, since boot (or the last reset).
uint32_t deferredLogDrops();

// Print every queued line with ESP_LOG, and a line for any new drops. Call
// only from a task that may block (ESP_PLATFORM; a no-op on the host).
size_t drainDeferredLogs();

// Host tests only: empty the queue and zero the drop count.
void resetDeferredLog();

}  // namespace w5500

#endif  // W5500_DEFERRED_LOG_H

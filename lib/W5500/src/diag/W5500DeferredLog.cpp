// Deferred log for real-time paths (G-B13). See W5500DeferredLog.h.

#include "W5500DeferredLog.h"

#include <cstdarg>
#include <cstdio>
#include <cstring>

#ifdef ESP_PLATFORM
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#else
#include <mutex>
#endif

namespace w5500 {

namespace {

DeferredLogLine g_lines[kDeferredLogLines];
size_t g_head = 0;  // next line to pop
size_t g_count = 0;
uint32_t g_drops = 0;

#ifdef ESP_PLATFORM
// A spinlock, not a mutex: callers may hold the W5500 bus mutex or run the
// Class 1 loop, and the critical section is only a 160-byte copy.
portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
struct Lock {
    Lock() { portENTER_CRITICAL(&g_mux); }
    ~Lock() { portEXIT_CRITICAL(&g_mux); }
};
#else
std::mutex g_mutex;
struct Lock {
    std::lock_guard<std::mutex> guard{g_mutex};
};
#endif

}  // namespace

bool deferLog(const char* tag, char level, uint32_t at_ms, const char* fmt, ...) {
    // Format outside the lock: vsnprintf is the slow part.
    char text[kDeferredLogLineLen];
    va_list args;
    va_start(args, fmt);
    std::vsnprintf(text, sizeof(text), fmt, args);
    va_end(args);

    Lock lock;
    if (g_count == kDeferredLogLines) {
        ++g_drops;
        return false;
    }
    DeferredLogLine& line = g_lines[(g_head + g_count) % kDeferredLogLines];
    line.tag = (tag != nullptr) ? tag : "";
    line.level = level;
    line.at_ms = at_ms;
    std::memcpy(line.text, text, sizeof(line.text));
    ++g_count;
    return true;
}

bool popDeferredLog(DeferredLogLine& out) {
    Lock lock;
    if (g_count == 0) {
        return false;
    }
    out = g_lines[g_head];
    g_head = (g_head + 1) % kDeferredLogLines;
    --g_count;
    return true;
}

uint32_t deferredLogDrops() {
    Lock lock;
    return g_drops;
}

size_t drainDeferredLogs() {
#ifdef ESP_PLATFORM
    static uint32_t reported_drops = 0;
    size_t printed = 0;
    DeferredLogLine line;
    while (popDeferredLog(line)) {
        switch (line.level) {
            case 'E':
                ESP_LOGE(line.tag, "%s (at %lu ms)", line.text,
                         static_cast<unsigned long>(line.at_ms));
                break;
            case 'W':
                ESP_LOGW(line.tag, "%s (at %lu ms)", line.text,
                         static_cast<unsigned long>(line.at_ms));
                break;
            default:
                ESP_LOGI(line.tag, "%s (at %lu ms)", line.text,
                         static_cast<unsigned long>(line.at_ms));
                break;
        }
        ++printed;
    }
    const uint32_t drops = deferredLogDrops();
    if (drops != reported_drops) {
        ESP_LOGW("W5500Log", "%lu deferred log line(s) dropped (queue full)",
                 static_cast<unsigned long>(drops - reported_drops));
        reported_drops = drops;
    }
    return printed;
#else
    return 0;
#endif
}

void resetDeferredLog() {
    Lock lock;
    g_head = 0;
    g_count = 0;
    g_drops = 0;
}

}  // namespace w5500

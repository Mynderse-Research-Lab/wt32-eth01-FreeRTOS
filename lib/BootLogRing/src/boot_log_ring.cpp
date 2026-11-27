#include "boot_log_ring.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#include <atomic>
#include <cstdarg>
#include <cstdio>
#include <cstring>

namespace {

constexpr size_t kRingBytes = 12288;

char g_ring[kRingBytes];
size_t g_len = 0;                  // guarded by g_mux
std::atomic<bool> g_frozen{false};
portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
vprintf_like_t g_next = nullptr;

int ringVprintf(const char* fmt, va_list args) {
    if (!g_frozen.load(std::memory_order_relaxed)) {
        char line[256];
        va_list copy;
        va_copy(copy, args);
        const int n = vsnprintf(line, sizeof(line), fmt, copy);
        va_end(copy);
        if (n > 0) {
            const size_t len = (static_cast<size_t>(n) < sizeof(line))
                                   ? static_cast<size_t>(n)
                                   : sizeof(line) - 1;
            portENTER_CRITICAL(&g_mux);
            if (g_len + len <= kRingBytes) {
                std::memcpy(g_ring + g_len, line, len);
                g_len += len;
            } else {
                g_frozen.store(true, std::memory_order_relaxed);
            }
            portEXIT_CRITICAL(&g_mux);
        }
    }
    return (g_next != nullptr) ? g_next(fmt, args) : vprintf(fmt, args);
}

}  // namespace

void bootLogRingInstall() {
    if (g_next != nullptr) return;
    g_next = esp_log_set_vprintf(ringVprintf);
}

void bootLogRingForEach(void (*fn)(const char* line, void* ctx), void* ctx) {
    g_frozen.store(true, std::memory_order_relaxed);
    size_t len = 0;
    portENTER_CRITICAL(&g_mux);
    len = g_len;
    portEXIT_CRITICAL(&g_mux);
    char line[256];
    size_t start = 0;
    for (size_t i = 0; i <= len; ++i) {
        if (i == len || g_ring[i] == '\n') {
            size_t n = i - start;
            if (n > sizeof(line) - 1) n = sizeof(line) - 1;
            if (n > 0) {
                std::memcpy(line, g_ring + start, n);
                line[n] = '\0';
                fn(line, ctx);
            }
            start = i + 1;
        }
    }
}

size_t bootLogRingBytes() {
    portENTER_CRITICAL(&g_mux);
    const size_t len = g_len;
    portEXIT_CRITICAL(&g_mux);
    return len;
}

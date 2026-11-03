#include "W5500BusMutex.h"

#include "W5500Poll.h"

#include <cstring>

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

namespace w5500 {

namespace {

void currentTaskName(char* out, size_t len) {
#ifdef ESP_PLATFORM
    const char* name = pcTaskGetName(nullptr);
    std::strncpy(out, name != nullptr ? name : "?", len - 1);
#else
    std::strncpy(out, "host", len - 1);
#endif
    out[len - 1] = '\0';
}

// The statistics have their own short lock, so reading them never holds the
// bus mutex. take() used to lock the bus mutex from the priority-2 EipLog task
// on core 0; preempted there by network work, it held off Class 1 for 30 ms
// and more and dropped the drives (G-B6, measured 2026-11-03).
#ifdef ESP_PLATFORM
portMUX_TYPE g_stats_mux = portMUX_INITIALIZER_UNLOCKED;

struct StatsGuard {
    StatsGuard() { portENTER_CRITICAL(&g_stats_mux); }
    ~StatsGuard() { portEXIT_CRITICAL(&g_stats_mux); }
};
#else
std::mutex g_stats_mutex;

struct StatsGuard {
    StatsGuard() { g_stats_mutex.lock(); }
    ~StatsGuard() { g_stats_mutex.unlock(); }
};
#endif

}  // namespace

void BusMutex::lock() {
    const int64_t t0 = pollNowUs();
    m_.lock();
    acquired_us_ = pollNowUs();
    const int64_t wait = acquired_us_ - t0;
    StatsGuard guard;
    if (wait > static_cast<int64_t>(stats_.max_wait_us)) {
        stats_.max_wait_us = static_cast<uint32_t>(wait);
        currentTaskName(stats_.waiter, sizeof(stats_.waiter));
    }
}

void BusMutex::unlock() {
    const int64_t hold = pollNowUs() - acquired_us_;
    {
        StatsGuard guard;
        if (hold > static_cast<int64_t>(stats_.max_hold_us)) {
            stats_.max_hold_us = static_cast<uint32_t>(hold);
            currentTaskName(stats_.holder, sizeof(stats_.holder));
        }
    }
    m_.unlock();
}

BusHoldStats BusMutex::take() {
    StatsGuard guard;
    const BusHoldStats out = stats_;
    stats_ = BusHoldStats{};
    return out;
}

BusMutex& spiBusMutex() {
    static BusMutex m;
    return m;
}

}  // namespace w5500

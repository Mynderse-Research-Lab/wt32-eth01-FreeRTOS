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

}  // namespace

void BusMutex::lock() {
    const int64_t t0 = pollNowUs();
    m_.lock();
    acquired_us_ = pollNowUs();
    const int64_t wait = acquired_us_ - t0;
    if (wait > static_cast<int64_t>(stats_.max_wait_us)) {
        stats_.max_wait_us = static_cast<uint32_t>(wait);
        currentTaskName(stats_.waiter, sizeof(stats_.waiter));
    }
}

void BusMutex::unlock() {
    const int64_t hold = pollNowUs() - acquired_us_;
    if (hold > static_cast<int64_t>(stats_.max_hold_us)) {
        stats_.max_hold_us = static_cast<uint32_t>(hold);
        currentTaskName(stats_.holder, sizeof(stats_.holder));
    }
    m_.unlock();
}

BusHoldStats BusMutex::take() {
    std::lock_guard<std::mutex> lock(m_);
    const BusHoldStats out = stats_;
    stats_ = BusHoldStats{};
    return out;
}

BusMutex& spiBusMutex() {
    static BusMutex m;
    return m;
}

}  // namespace w5500

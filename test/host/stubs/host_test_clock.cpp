#include "host_test_clock.h"

#include "esp_timer.h"

namespace {

int64_t g_now_us = 0;

}  // namespace

namespace HostTest {

void resetClock() { g_now_us = 0; }

void setUs(int64_t us) { g_now_us = us; }

void advanceUs(int64_t us) { g_now_us += us; }

void advanceMs(uint32_t ms) {
    g_now_us += static_cast<int64_t>(ms) * 1000LL;
}

int64_t nowUs() { return g_now_us; }

}  // namespace HostTest

extern "C" int64_t esp_timer_get_time(void) {
    return HostTest::nowUs();
}

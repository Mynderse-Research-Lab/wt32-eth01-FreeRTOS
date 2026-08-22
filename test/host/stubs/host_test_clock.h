/**
 * @file host_test_clock.h
 * @brief Settable monotonic clock for host tests of Gantry timing.
 */
#pragma once

#include <cstdint>

namespace HostTest {

void resetClock();
void setUs(int64_t us);
void advanceMs(uint32_t ms);
void advanceUs(int64_t us);
int64_t nowUs();

}  // namespace HostTest

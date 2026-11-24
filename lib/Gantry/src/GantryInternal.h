/**
 * @file GantryInternal.h
 * @brief Private helpers shared by the Gantry*.cpp files that implement
 *        the Gantry class. Not part of the library interface; include it
 *        only from those files.
 */
#pragma once

#include <cstdint>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {
// Arduino-compat helpers, pure ESP-IDF implementation.
static inline unsigned long gantry_millis() {
    return (unsigned long)(esp_timer_get_time() / 1000LL);
}
static inline void gantry_delay(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}
}  // namespace

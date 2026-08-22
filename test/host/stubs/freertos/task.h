#pragma once

#include <freertos/FreeRTOS.h>
#include "host_test_clock.h"

static inline void vTaskDelay(TickType_t ticks) {
    HostTest::advanceMs(static_cast<uint32_t>(ticks));
}

static inline TickType_t xTaskGetTickCount(void) {
    return static_cast<TickType_t>(HostTest::nowUs() / 1000LL);
}

typedef void* TaskHandle_t;
typedef void (*TaskFunction_t)(void*);

#define tskNO_AFFINITY (-1)

static inline BaseType_t xTaskCreatePinnedToCore(TaskFunction_t, const char*,
                                                 uint32_t, void*, UBaseType_t,
                                                 TaskHandle_t*, BaseType_t) {
    return pdFAIL;
}

static inline void vTaskDelete(TaskHandle_t) {}

static inline BaseType_t xTaskDelayUntil(TickType_t* last_wake,
                                         TickType_t period) {
    if (last_wake == nullptr) {
        return pdFALSE;
    }
    *last_wake += period;
    HostTest::advanceMs(static_cast<uint32_t>(period));
    return pdTRUE;
}

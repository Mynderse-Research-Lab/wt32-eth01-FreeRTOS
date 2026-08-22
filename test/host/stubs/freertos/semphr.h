#pragma once

#include <freertos/FreeRTOS.h>
#include <stdint.h>

typedef void* SemaphoreHandle_t;

static inline SemaphoreHandle_t xSemaphoreCreateMutex(void) {
    return (SemaphoreHandle_t)(uintptr_t)1;
}

static inline BaseType_t xSemaphoreTake(SemaphoreHandle_t, TickType_t) {
    return pdTRUE;
}

static inline BaseType_t xSemaphoreGive(SemaphoreHandle_t) { return pdTRUE; }

static inline void vSemaphoreDelete(SemaphoreHandle_t) {}

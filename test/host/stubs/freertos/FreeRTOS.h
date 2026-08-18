#pragma once

#include <stdint.h>

typedef uint32_t TickType_t;
typedef int32_t BaseType_t;
typedef uint32_t UBaseType_t;

#define pdTRUE 1
#define pdFALSE 0
#define pdPASS pdTRUE
#define pdFAIL pdFALSE
#define portTICK_PERIOD_MS 1

#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))

#pragma once

#include <stdio.h>

#define ESP_LOGE(tag, fmt, ...) \
    ((void)(tag), (void)printf("[E] " fmt "\n", ##__VA_ARGS__))
#define ESP_LOGW(tag, fmt, ...) ((void)(tag))
#define ESP_LOGI(tag, fmt, ...) ((void)(tag))
#define ESP_LOGD(tag, fmt, ...) ((void)(tag))
#define ESP_LOGV(tag, fmt, ...) ((void)(tag))

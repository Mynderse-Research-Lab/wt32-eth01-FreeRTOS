/**
 * @file AppConfigInternal.h
 * @brief Declarations shared by the AppConfig*.cpp files. Private to this
 *        library; include it only from those files.
 */
#pragma once

#include "AppConfig.h"
#include "MemoryStorage.h"
#include "NvsStorage.h"
#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <ctime>
#include <utility>

#if defined(ESP_PLATFORM)
#include "sdkconfig.h"
#endif

namespace {

// NVS keys are at most 15 characters. Old cv_jaw1_ms keys are not read.
constexpr int kPickClassSlots = 10;
const char* const kPickJawKeys[kPickClassSlots] = {
    "jaw_aaa_ms", "jaw_aa_ms", "jaw_d_ms", "jaw_c_ms", "jaw_9v_ms",
    "jaw_cr123_ms", "jaw_18650_ms", "jaw_21700_ms", "jaw_26650_ms", "jaw_32650_ms",
};

}  // namespace

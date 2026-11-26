/**
 * @file EipMultiScannerInternal.h
 * @brief Log tag shared by the multi-axis scanner files.
 *
 * Private to this library.
 */
#pragma once

namespace eip {
#ifdef ESP_PLATFORM
static const char* kTag = "EipMulti";
#endif
}  // namespace eip

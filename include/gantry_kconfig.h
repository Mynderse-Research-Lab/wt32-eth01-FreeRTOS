/**
 * @file gantry_kconfig.h
 * @brief Optional sdkconfig.h bridge for kinematic macros (ESP-IDF builds).
 *
 * Host unit tests do not have sdkconfig.h; they keep the hardcoded defaults
 * in axis_*_params.h. Firmware builds pull floats/ints from menuconfig.
 */
#ifndef GANTRY_KCONFIG_H
#define GANTRY_KCONFIG_H

#if defined(__has_include)
#  if __has_include("sdkconfig.h")
#    include "sdkconfig.h"
#  endif
#elif defined(ESP_PLATFORM)
#  include "sdkconfig.h"
#endif

#endif /* GANTRY_KCONFIG_H */

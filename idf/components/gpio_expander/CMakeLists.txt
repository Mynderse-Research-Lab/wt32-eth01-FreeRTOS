# gpio_expander component. Implementation lives at ../../../src/gpio_expander.c
# and its header at ../../../include/gpio_expander.h. We keep those at the repo
# root so the PlatformIO layout is preserved, and point the ESP-IDF component
# build at them by absolute path.

set(APP_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../..")

idf_component_register(
    SRCS
        "${APP_ROOT}/src/gpio_expander.c"
    INCLUDE_DIRS
        "${APP_ROOT}/include"
    REQUIRES
        MCP23S17
        esp_common
        log
)

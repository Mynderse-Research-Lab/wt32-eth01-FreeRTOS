# GpioExpander – MCP23S17 pin wrapper used by the gantry and the field I/O.

set(APP_ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")

idf_component_register(
    SRCS
        "src/gpio_expander.c"
    INCLUDE_DIRS
        "src"
        "${APP_ROOT}/include"
    REQUIRES
        MCP23S17
        log
        esp_common
)

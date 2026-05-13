# Main application component for the WT32-ETH01 gantry controller.
#
# Sources live in the repo's top-level ../src/ and headers in ../include/ so
# the PlatformIO layout is preserved. They are pulled in by absolute path.

set(APP_ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")

idf_component_register(
    SRCS
        "${APP_ROOT}/src/main.cpp"
        "${APP_ROOT}/src/gantry_test_console.cpp"
        "${APP_ROOT}/src/basic_tests.cpp"
    INCLUDE_DIRS
        "${APP_ROOT}/include"
        "${APP_ROOT}/src"
    REQUIRES
        Gantry
        PulseMotor
        MCP23S17
        gpio_expander
        driver
        esp_timer
        log
        freertos
)

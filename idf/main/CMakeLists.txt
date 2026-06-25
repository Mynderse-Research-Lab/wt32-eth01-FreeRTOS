# Main application component for the WT32-ETH01 gantry controller.
#
# Sources live in the repo's top-level ../src/ and headers in ../include/ at the
# repo root. They are pulled in by absolute path.

set(APP_ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")

set(APP_SRCS
    "${APP_ROOT}/src/main.cpp"
    "${APP_ROOT}/src/pick_scheduler.cpp"
    "${APP_ROOT}/src/gantry_test_console.cpp"
)

# basic_tests.cpp backs the on-target "selftest" console command. The same
# assertions run host-side in CI (test/host/), so it is only compiled in when
# CONFIG_GANTRY_SELFTEST is enabled (default y; set n for production builds).
if(CONFIG_GANTRY_SELFTEST)
    list(APPEND APP_SRCS "${APP_ROOT}/src/basic_tests.cpp")
endif()

idf_component_register(
    SRCS
        ${APP_SRCS}
    INCLUDE_DIRS
        "${APP_ROOT}/include"
        "${APP_ROOT}/src"
    REQUIRES
        Gantry
        MqttBridge
        PulseMotor
        MCP23S17
        gpio_expander
        esp_driver_gpio
        esp_timer
        log
        freertos
)

# ESP-IDF v6 treats #warning as -Werror=cpp. When enabled via menuconfig (or
# idf.py -DCONFIG_GANTRY_SUPPRESS_GEOMETRY_WARNING=y in CI), silence the
# intentional dev-rig geometry reminder in main.cpp only.
if(CONFIG_GANTRY_SUPPRESS_GEOMETRY_WARNING)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE GANTRY_SUPPRESS_GEOMETRY_WARNING)
endif()

# ESP-IDF v6 enables -Werror by default; keep geometry placeholder reminders as
# warnings instead of hard errors so CI can compile while still surfacing them.
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error=cpp)

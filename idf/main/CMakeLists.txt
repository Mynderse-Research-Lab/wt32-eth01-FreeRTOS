# Main application component for the WT32-ETH01 gantry controller.
#
# Sources live in the repo's top-level ../src/ and headers in ../include/ at the
# repo root. They are pulled in by absolute path.

set(APP_ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")

set(APP_SRCS
    "${APP_ROOT}/src/main.cpp"
    "${APP_ROOT}/src/pick_scheduler.cpp"
    "${APP_ROOT}/src/pick_belt.cpp"
    "${APP_ROOT}/src/pick_bench.cpp"
    "${APP_ROOT}/src/pick_console.cpp"
    "${APP_ROOT}/src/pick_eval.cpp"
    "${APP_ROOT}/src/pick_motion.cpp"
    "${APP_ROOT}/src/pick_run.cpp"
    "${APP_ROOT}/src/pick_strike.cpp"
    "${APP_ROOT}/src/pick_trace.cpp"
    "${APP_ROOT}/src/theta_trace.cpp"
    "${APP_ROOT}/src/gantry_cellnet_ota.cpp"
    "${APP_ROOT}/src/gantry_test_console.cpp"
    "${APP_ROOT}/src/gantry_console_bringup.cpp"
    "${APP_ROOT}/src/gantry_console_cfg.cpp"
    "${APP_ROOT}/src/gantry_console_cip.cpp"
    "${APP_ROOT}/src/gantry_console_cmd_motion.cpp"
    "${APP_ROOT}/src/gantry_console_cmd_pick.cpp"
    "${APP_ROOT}/src/gantry_console_cmd_profile.cpp"
    "${APP_ROOT}/src/gantry_console_commands.cpp"
    "${APP_ROOT}/src/gantry_console_drive_diag.cpp"
    "${APP_ROOT}/src/gantry_console_drv.cpp"
    "${APP_ROOT}/src/gantry_console_homing.cpp"
    "${APP_ROOT}/src/gantry_console_io.cpp"
    "${APP_ROOT}/src/gantry_console_session.cpp"
    "${APP_ROOT}/src/gantry_console_status.cpp"
    "${APP_ROOT}/src/gantry_console_testcycle.cpp"
    "${APP_ROOT}/src/gantry_console_thetapath.cpp"
    "${APP_ROOT}/src/gantry_net_console.cpp"
    "${APP_ROOT}/src/gantry_net_console_auth.cpp"
    "${APP_ROOT}/src/gantry_ota.cpp"
    "${APP_ROOT}/src/gpio_expander.c"
    "${APP_ROOT}/src/boot_log_ring.cpp"
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
        CellNetL2
        EthernetLink
        EtherNetIP
        MCP23S17
        Spi3Bus
        SpiDisplay
        AppConfig
        nvs_flash
        esp_driver_spi
        esp_driver_gpio
        esp_driver_ledc
        esp_driver_uart
        esp_timer
        log
        freertos
        lwip
        esp_netif
        app_update
        esp_partition
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

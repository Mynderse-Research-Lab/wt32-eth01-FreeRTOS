# ThetaTrace – 1 kHz trace of one HCS01 theta move, for the console command thtrace.

set(APP_ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")

idf_component_register(
    SRCS
        "src/theta_trace.cpp"
    INCLUDE_DIRS
        "src"
        "${APP_ROOT}/include"
    REQUIRES
        Gantry
        log
        freertos
        esp_timer
)

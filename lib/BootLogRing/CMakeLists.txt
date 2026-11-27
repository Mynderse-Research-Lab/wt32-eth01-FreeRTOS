# BootLogRing – the first log lines after reset, kept in RAM for the console.

idf_component_register(
    SRCS
        "src/boot_log_ring.cpp"
    INCLUDE_DIRS
        "src"
    REQUIRES
        log
        freertos
)

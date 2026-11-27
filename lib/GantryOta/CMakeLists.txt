# GantryOta – dual-OTA flash, the TCP server, and CellNet OTA-only mode.

set(APP_ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")

idf_component_register(
    SRCS
        "src/gantry_ota.cpp"
        "src/gantry_ota_tcp.cpp"
        "src/gantry_cellnet_ota.cpp"
    INCLUDE_DIRS
        "src"
        "${APP_ROOT}/include"
    REQUIRES
        Gantry
        CellNetL2
        EthernetLink
        app_update
        esp_app_format
        esp_partition
        esp_eth
        esp_system
        esp_timer
        freertos
        log
        lwip
        hal
        soc
)

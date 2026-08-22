set(APP_ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")

idf_component_register(
    SRCS
        "src/EthernetLink.cpp"
    INCLUDE_DIRS
        "src"
        "${APP_ROOT}/include"
    REQUIRES
        esp_eth
        esp_netif
        esp_event
        freertos
        log
        lan87xx
        nvs_flash
    PRIV_REQUIRES
        esp_driver_gpio
)

set(APP_ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")

idf_component_register(
    SRCS
        "src/EthernetLink.cpp"
        "src/MqttBridge.cpp"
    INCLUDE_DIRS
        "src"
        "${APP_ROOT}/include"
    REQUIRES
        esp_eth
        esp_event
        esp_netif
        esp_timer
        freertos
        log
        mqtt
    PRIV_REQUIRES
        esp_driver_gpio
)

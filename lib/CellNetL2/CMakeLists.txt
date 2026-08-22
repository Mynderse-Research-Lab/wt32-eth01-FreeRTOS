idf_component_register(
    SRCS
        "src/CellNetL2Framing.cpp"
        "src/CellNetL2.cpp"
        "src/EspEthL2Transport.cpp"
    INCLUDE_DIRS
        "src"
        "../../include"
    REQUIRES
        esp_eth
        esp_netif
        esp_timer
        log
        freertos
)


# Top-level ESP-IDF project for the WT32-ETH01 gantry controller.
#
# All drive control is EtherNet/IP over W5500 SPI. MCP23S17 and step/direction
# removed per 2026-07 refactor.
#
# Build:   idf.py -C idf build
# Flash:   idf.py -C idf -p COM3 flash monitor

cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/../lib/Gantry"
    "${CMAKE_CURRENT_LIST_DIR}/../lib/MqttBridge"
    "${CMAKE_CURRENT_LIST_DIR}/../lib/EtherNetIP"
    "${CMAKE_CURRENT_LIST_DIR}/../lib/W5500"
    "${CMAKE_CURRENT_LIST_DIR}/../lib/I2cDisplay"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(wt32_eth01_gantry)

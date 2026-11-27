# Top-level ESP-IDF project for the WT32-ETH01 gantry controller.
#
# Drive control: EtherNet/IP over W5500 (SPI2).
# SPI3 shared: MCP23S17 (Field I/O + TFT control) + Amazon SPI TFT.
#
# Build:   idf.py -C idf build
# Flash:   idf.py -C idf -p COM3 flash monitor

cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/../lib/Gantry"
    "${CMAKE_CURRENT_LIST_DIR}/../lib/CellNetL2"
    "${CMAKE_CURRENT_LIST_DIR}/../lib/EthernetLink"
    "${CMAKE_CURRENT_LIST_DIR}/../lib/EtherNetIP"
    "${CMAKE_CURRENT_LIST_DIR}/../lib/W5500"
    "${CMAKE_CURRENT_LIST_DIR}/../lib/MCP23S17"
    "${CMAKE_CURRENT_LIST_DIR}/../lib/Spi3Bus"
    "${CMAKE_CURRENT_LIST_DIR}/../lib/SpiDisplay"
    "${CMAKE_CURRENT_LIST_DIR}/../lib/AppConfig"
    "${CMAKE_CURRENT_LIST_DIR}/../lib/BootLogRing"
)

# Compile-time defaults come from sdkconfig.defaults alone. The generated
# sdkconfig lives in the build tree and is rebuilt on every configure, so a
# leftover menuconfig edit cannot override them. Runtime values live in NVS.
set(SDKCONFIG "${CMAKE_BINARY_DIR}/sdkconfig")
file(REMOVE "${SDKCONFIG}")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(wt32_eth01_gantry)

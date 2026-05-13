# Top-level ESP-IDF project for the WT32-ETH01 gantry controller.
#
# Layout:
#   idf/                      <- this folder (the ESP-IDF project root)
#     CMakeLists.txt          <- this file
#     main/                   <- main app component (pulls src/*.cpp)
#     components/             <- project-private components (gpio_expander)
#   ../src/                   <- application sources (main.cpp, console, etc.)
#   ../include/               <- application headers (pin map, params)
#   ../lib/PulseMotor/        <- PulseMotor component
#   ../lib/Gantry/            <- Gantry component
#   ../lib/MCP23S17/          <- MCP23S17 SPI expander component
#
# Build:   idf.py -C idf build
# Flash:   idf.py -C idf -p COM3 flash monitor

cmake_minimum_required(VERSION 3.16)

# Point ESP-IDF at the repo's "lib/" dir so every library under lib/<Name>/
# with a CMakeLists.txt is picked up as an ESP-IDF component.
set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/../lib/MCP23S17"
    "${CMAKE_CURRENT_LIST_DIR}/../lib/PulseMotor"
    "${CMAKE_CURRENT_LIST_DIR}/../lib/Gantry"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(wt32_eth01_gantry)

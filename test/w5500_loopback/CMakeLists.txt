# Standalone ESP-IDF project for W5500 MACRAW loopback validation.
#
# Build: idf.py -C test/w5500_loopback build
# Flash: idf.py -C test/w5500_loopback -p COM3 flash monitor
#
# This project is isolated from the main firmware. It only depends on
# lib/W5500/.  No MCP23S17, Gantry, or EtherNet/IP are pulled in.

cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/../../lib/W5500"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(w5500_loopback)

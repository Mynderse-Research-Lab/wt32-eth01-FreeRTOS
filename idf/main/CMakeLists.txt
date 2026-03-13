set(SRC_DIR "${CMAKE_CURRENT_LIST_DIR}/../../src")
set(PROJECT_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/../../include")
set(SDF08NK8X_DIR "${CMAKE_CURRENT_LIST_DIR}/../../lib/SDF08NK8X/src")
set(GANTRY_DIR "${CMAKE_CURRENT_LIST_DIR}/../../lib/Gantry/src")
set(MCP23S17_DIR "${CMAKE_CURRENT_LIST_DIR}/../../lib/MCP23S17/src")

idf_component_register(
  SRCS
    "${SRC_DIR}/basic_tests.cpp"
    "${SRC_DIR}/gantry_test_console.cpp"
    "${SRC_DIR}/gpio_expander.c"
    "${SRC_DIR}/main.cpp"
  INCLUDE_DIRS
    "."
    "${PROJECT_INCLUDE_DIR}"
    "${SRC_DIR}"
    "${SDF08NK8X_DIR}"
    "${GANTRY_DIR}"
    "${MCP23S17_DIR}"
  REQUIRES
    espressif__arduino-esp32
    SDF08NK8X
    Gantry
    MCP23S17
)

target_compile_definitions(${COMPONENT_LIB} PRIVATE
  ETH_CLK_MODE=ETH_CLOCK_GPIO17_OUT
  ETH_PHY_POWER=16
  ETH_TYPE=ETH_PHY_LAN8720
  ETH_ADDR=1
  ETH_MDC_PIN=23
  ETH_MDIO_PIN=18
  ARDUINO_RUNNING_CORE=1
  SDF08NK8X_USE_FREERTOS=0
  SDF08NK8X_DEBUG=1
)

# Suppress deprecated PCNT warning treated as error by -Werror=cpp
target_compile_options(${COMPONENT_LIB} PRIVATE
  -Wno-error=cpp
)

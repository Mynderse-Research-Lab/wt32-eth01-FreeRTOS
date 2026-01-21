set(SRC_DIR "${CMAKE_CURRENT_LIST_DIR}/../../src")
set(SDF08NK8X_DIR "${CMAKE_CURRENT_LIST_DIR}/../../lib/SDF08NK8X/src")
set(GANTRY_DIR "${CMAKE_CURRENT_LIST_DIR}/../../lib/Gantry/src")

idf_component_register(
  SRCS
    "app_main.cpp"
    "${SRC_DIR}/main.cpp"
  INCLUDE_DIRS
    "."
    "${SRC_DIR}"
    "${SDF08NK8X_DIR}"
    "${GANTRY_DIR}"
  REQUIRES
    espressif__arduino-esp32
    SDF08NK8X
    Gantry
    mqtt
)

target_compile_definitions(${COMPONENT_LIB} PRIVATE
  ETH_CLK_MODE=ETH_CLOCK_GPIO17_OUT
  ETH_PHY_POWER=16
  ETH_TYPE=ETH_PHY_LAN8720
  ETH_ADDR=1
  ETH_MDC_PIN=23
  ETH_MDIO_PIN=18
  ARDUINO_RUNNING_CORE=1
  ELEGANTOTA_USE_ASYNC_WEBSERVER=1
  SDF08NK8X_USE_FREERTOS=0
  SDF08NK8X_DEBUG=1
)

# Suppress deprecated PCNT warning treated as error by -Werror=cpp
target_compile_options(${COMPONENT_LIB} PRIVATE
  -Wno-error=cpp
)

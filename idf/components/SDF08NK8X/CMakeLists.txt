set(SRC_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../lib/SDF08NK8X/src")
set(PROJECT_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../include")
set(MCP23S17_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../lib/MCP23S17/src")

idf_component_register(
  SRCS
    "${SRC_DIR}/SDF08NK8X.cpp"
  INCLUDE_DIRS
    "${SRC_DIR}"
    "${PROJECT_INCLUDE_DIR}"
    "${MCP23S17_DIR}"
  REQUIRES
    espressif__arduino-esp32
)

target_compile_definitions(${COMPONENT_LIB} PRIVATE
  SDF08NK8X_USE_FREERTOS=0
  SDF08NK8X_DEBUG=0
)

# Suppress deprecated PCNT warning treated as error by -Werror=cpp
target_compile_options(${COMPONENT_LIB} PRIVATE
  -Wno-error=cpp
)

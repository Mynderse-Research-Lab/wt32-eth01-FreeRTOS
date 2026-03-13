set(SRC_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../lib/MCP23S17/src")

idf_component_register(
  SRCS
    "${SRC_DIR}/MCP23S17.cpp"
  INCLUDE_DIRS
    "${SRC_DIR}"
  REQUIRES
    driver
)

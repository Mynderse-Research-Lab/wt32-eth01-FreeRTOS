set(SRC_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../lib/SDF08NK8X/src")

idf_component_register(
  SRCS
    "${SRC_DIR}/SDF08NK8X.cpp"
  INCLUDE_DIRS
    "${SRC_DIR}"
  REQUIRES
    espressif__arduino-esp32
)

target_compile_definitions(${COMPONENT_LIB} PRIVATE
  SDF08NK8X_USE_FREERTOS=0
  SDF08NK8X_DEBUG=0
)

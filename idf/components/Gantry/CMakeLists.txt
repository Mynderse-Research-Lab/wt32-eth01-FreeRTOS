set(GANTRY_SRC_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../lib/Gantry/src")

idf_component_register(
  SRCS
    "${GANTRY_SRC_DIR}/Gantry.cpp"
    "${GANTRY_SRC_DIR}/GantryConfig.cpp"
    "${GANTRY_SRC_DIR}/GantryKinematics.cpp"
    "${GANTRY_SRC_DIR}/GantryTrajectory.cpp"
  INCLUDE_DIRS
    "${GANTRY_SRC_DIR}"
  REQUIRES
    espressif__arduino-esp32
    SDF08NK8X
)

# Suppress deprecated warnings treated as errors
target_compile_options(${COMPONENT_LIB} PRIVATE
  -Wno-error=cpp
)

set(GANTRY_SRC_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../lib/Gantry/src")
set(PROJECT_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../include")
set(MCP23S17_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../lib/MCP23S17/src")

idf_component_register(
  SRCS
    "${GANTRY_SRC_DIR}/Gantry.cpp"
    "${GANTRY_SRC_DIR}/GantryAxisStepper.cpp"
    "${GANTRY_SRC_DIR}/GantryConfig.cpp"
    "${GANTRY_SRC_DIR}/GantryEndEffector.cpp"
    "${GANTRY_SRC_DIR}/GantryKinematics.cpp"
    "${GANTRY_SRC_DIR}/GantryRotaryServo.cpp"
    "${GANTRY_SRC_DIR}/GantryTrajectory.cpp"
  INCLUDE_DIRS
    "${GANTRY_SRC_DIR}"
    "${PROJECT_INCLUDE_DIR}"
    "${MCP23S17_DIR}"
  REQUIRES
    espressif__arduino-esp32
    SDF08NK8X
)

# Suppress deprecated warnings treated as errors
target_compile_options(${COMPONENT_LIB} PRIVATE
  -Wno-error=cpp
)

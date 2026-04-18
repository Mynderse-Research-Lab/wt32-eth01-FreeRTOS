# Gantry component - multi-axis gantry control layered on top of PulseMotor.
#
# Public headers live in src/. The Gantry component pulls the per-axis
# mechanical parameter headers (axis_drivetrain_params.h, axis_pulse_motor_params.h)
# from the app-level include/ dir.

set(APP_ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")

idf_component_register(
    SRCS
        "src/Gantry.cpp"
        "src/GantryConfig.cpp"
        "src/GantryEndEffector.cpp"
        "src/GantryKinematics.cpp"
        "src/GantryLimitSwitch.cpp"
        "src/GantryPulseMotorLinearAxis.cpp"
        "src/GantryPulseMotorRotaryAxis.cpp"
        "src/GantryTrajectory.cpp"
    INCLUDE_DIRS
        "src"
        "${APP_ROOT}/include"
    REQUIRES
        PulseMotor
        gpio_expander
        driver
        esp_common
        esp_timer
        log
        freertos
)

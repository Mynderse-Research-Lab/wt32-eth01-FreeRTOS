# Gantry component - multi-axis gantry control over EtherNet/IP.
#
# Public headers live in src/. The Gantry component pulls the per-axis
# mechanical parameter headers (axis_drivetrain_params.h, axis_pulse_motor_params.h)
# from the app-level include/ dir.

set(APP_ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")

idf_component_register(
    SRCS
        "src/Gantry.cpp"
        "src/GantryAxisState.cpp"
        "src/homing/GantryEipBringUp.cpp"
        "src/homing/GantryEipHomeCal.cpp"
        "src/homing/GantryHoming.cpp"
        "src/motion/GantryMotion.cpp"
        "src/motion/GantryPath.cpp"
        "src/motion/GantryPathTheta.cpp"
        "src/motion/GantrySequential.cpp"
        "src/GantryConfig.cpp"
        "src/io/GantryEndEffector.cpp"
        "src/kinematics/GantryKinematics.cpp"
        "src/io/GantryLimitSwitch.cpp"
        "src/linear_axis/GantryEipLinearAxis.cpp"
        "src/linear_axis/GantryEipLinearAxisPhases.cpp"
        "src/linear_axis/GantryEipLinearAxisAlarm.cpp"
        "src/rotary_axis/GantryEipRotaryAxis.cpp"
        "src/rotary_axis/GantryEipRotaryAxisUpdate.cpp"
        "src/motion/GantryTrajectory.cpp"
    INCLUDE_DIRS
        "src"
        "src/motion"
        "src/homing"
        "src/kinematics"
        "src/linear_axis"
        "src/rotary_axis"
        "src/io"
        "${APP_ROOT}/include"
    REQUIRES
        EtherNetIP
        GpioExpander
        esp_common
        esp_timer
        log
        freertos
)

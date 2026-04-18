# PulseMotor component - generic pulse+direction motor driver.
#
# Exposes src/PulseMotor.h as a public header. Consumers should
# REQUIRES pulse_motor.
#
# Build flags:
#   PULSE_MOTOR_USE_FREERTOS (default 1) - guard driver with a mutex
#   PULSE_MOTOR_DEBUG        (default 1) - enable ESP_LOGD trace

idf_component_register(
    SRCS
        "src/PulseMotor.cpp"
    INCLUDE_DIRS
        "src"
    REQUIRES
        driver
        esp_common
        esp_timer
        log
        freertos
        gpio_expander
)

# Preserve the build flags that platformio.ini used to set.
target_compile_definitions(${COMPONENT_LIB} PUBLIC
    PULSE_MOTOR_USE_FREERTOS=0
    PULSE_MOTOR_DEBUG=1
)


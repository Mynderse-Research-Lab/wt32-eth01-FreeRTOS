#ifndef GANTRY_APP_CONSTANTS_H
#define GANTRY_APP_CONSTANTS_H

// MCP23S17 is mandatory for this application.
#define APP_USE_MCP23S17 1

// MCP23S17 SPI pins (direct ESP32 GPIO)
#define PIN_SPI_MISO 4
#define PIN_SPI_MOSI 12
#define PIN_SPI_SCLK 14
#define PIN_SPI_CS 5

// MCP23S17 pin assignments (0-15)
// X-axis signals are on Port A (0-7), Y-axis signals are on Port B (8-15).
// Available/unused: GPA6(6)
#define PIN_X_DIR 0
#define PIN_X_ENABLE 1
#define PIN_X_LIMIT_MIN 2
#define PIN_X_LIMIT_MAX 3
#define PIN_X_ALARM_STATUS 4
#define PIN_X_ALARM_RESET 5
#define PIN_GRIPPER 7
#define PIN_Y_DIR 8
#define PIN_Y_ENABLE 9
#define PIN_Y_LIMIT_MIN 10
#define PIN_Y_LIMIT_MAX 11
#define PIN_Y_ALARM_STATUS 12
#define PIN_Y_ALARM_RESET 13
#define PIN_THETA_LIMIT_MIN 14
#define PIN_THETA_LIMIT_MAX 15

// Direct ESP32 GPIO pins (hardware peripheral requirements)
// NOTE:
// - Encoder A/B must stay on direct GPIO for PCNT.
// - X/Y pulse and Theta PWM stay on direct GPIO for LEDC/PWM generation.
#define PIN_X_PULSE 32
#define PIN_Y_PULSE 33
#define PIN_X_ENC_A 35
#define PIN_X_ENC_B 36
#define PIN_Y_ENC_A 39
// GPIO2 is a boot strapping pin; keep LOW/floating during reset.
#define PIN_Y_ENC_B 2
// GPIO15 is a boot strapping pin; keep HIGH during reset for normal boot logs.
#define PIN_THETA_PWM 15

// Hardware peripheral channel allocation
#define X_PULSE_LEDC_CHANNEL 0
#define Y_PULSE_LEDC_CHANNEL 1
#define THETA_PWM_LEDC_CHANNEL 2
#define X_ENCODER_PCNT_UNIT 0
#define Y_ENCODER_PCNT_UNIT 1

// MCP23S17 defaults
#define MCP23S17_DEVICE_ADDRESS 0x00
#define MCP23S17_CLOCK_HZ 10000000
#define MCP_DEBUG_CMDS 1

// Gantry motion defaults
#define GANTRY_HOMING_SPEED_PPS 6000
#define GANTRY_ENCODER_PPR 6000
#define GANTRY_MAX_PULSE_FREQ 10000
#define GANTRY_X_MIN_MM 0.0f
#define GANTRY_X_MAX_MM 200.0f
#define GANTRY_Y_STEPS_PER_MM 200.0f
#define GANTRY_Y_MIN_MM 0.0f
#define GANTRY_Y_MAX_MM 200.0f
#define GANTRY_Y_MAX_SPEED_MMPS 100.0f
#define GANTRY_Y_ACCEL_MMPS2 500.0f
#define GANTRY_Y_DECEL_MMPS2 500.0f
#define GANTRY_THETA_MIN_DEG -90.0f
#define GANTRY_THETA_MAX_DEG 90.0f
#define GANTRY_THETA_MIN_PULSE_US 1000
#define GANTRY_THETA_MAX_PULSE_US 2000
#define GANTRY_SAFE_Y_MM 150.0f

// Task defaults
#define GANTRY_UPDATE_TASK_STACK 4096
#define GANTRY_UPDATE_TASK_PRIORITY 5
#define GANTRY_UPDATE_TASK_CORE 1
#define CONSOLE_TASK_STACK 4096
#define CONSOLE_TASK_PRIORITY 1
#define CONSOLE_TASK_CORE 0

#endif  // GANTRY_APP_CONSTANTS_H

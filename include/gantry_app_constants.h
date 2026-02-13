#ifndef GANTRY_APP_CONSTANTS_H
#define GANTRY_APP_CONSTANTS_H

// Temporary hardware mode switch for app-level testing.
// 0 = No MCP23S17 connected (direct WT32 GPIO only for critical X-axis signals)
// 1 = MCP23S17 connected and used for extended IO
#define APP_USE_MCP23S17 0

// MCP23S17 SPI pins (direct ESP32 GPIO)
#define PIN_SPI_MISO 19
#define PIN_SPI_MOSI 23
#define PIN_SPI_SCLK 18
#define PIN_SPI_CS 5

// MCP23S17 pin assignments (0-15)
#define PIN_DIR 1
#define PIN_ENABLE 2
#define PIN_LIMIT_MIN 3
#define PIN_LIMIT_MAX 4
#define PIN_Y_STEP 5
#define PIN_Y_DIR 6
#define PIN_Y_ENABLE 7
#define PIN_GRIPPER 8
#define PIN_LED 9
#define PIN_X_POS_REACHED 10
#define PIN_X_BRAKE_STATUS 11
#define PIN_X_ALARM_STATUS 12
#define PIN_X_ALARM_RESET 13
#define PIN_X_CWCCW_PROHIB 14
#define PIN_X_PULSE_INHIB 15

// Direct ESP32 GPIO pins
#define PIN_PULSE 32
#define PIN_ENC_A 35
#define PIN_ENC_B 36
#define PIN_THETA_PWM 13

// Direct WT32 test pins for X-axis (used when APP_USE_MCP23S17 == 0)
// These intentionally mirror SDF08NK8X-Driver-library test mappings.
#define PIN_X_PULSE_DIRECT 2
#define PIN_X_DIR_DIRECT 12
#define PIN_X_ENABLE_DIRECT 15
#define PIN_X_ALARM_DIRECT 17
// ARST output enabled for testing.
#define PIN_X_ALARM_RESET_DIRECT 32
#define PIN_X_LIMIT_MIN_DIRECT 33
#define PIN_X_LIMIT_MAX_DIRECT 5

// MCP23S17 defaults
#define MCP23S17_DEVICE_ADDRESS 0x00
#define MCP23S17_CLOCK_HZ 10000000

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

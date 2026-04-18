#ifndef GANTRY_APP_CONSTANTS_H
#define GANTRY_APP_CONSTANTS_H

#include "axis_pulse_motor_params.h"
#include "axis_drivetrain_params.h"

// MCP23S17 is mandatory for this application.
#define APP_USE_MCP23S17 1

// Known-good MCP23S17 SPI mapping/clock validated on WT32-ETH01 + MIKROE-951.
#define MCP23S17_SPI_CS_PIN 15
#define MCP23S17_SPI_MISO_PIN 35
#define MCP23S17_SPI_MOSI_PIN 12
#define MCP23S17_SPI_SCLK_PIN 5
#define MCP23S17_SPI_CLOCK_HZ_WORKING 1000000

// Backward-compatible aliases (legacy name style used in older modules/docs).
#define PIN_SPI_CS MCP23S17_SPI_CS_PIN
#define PIN_SPI_MISO MCP23S17_SPI_MISO_PIN
#define PIN_SPI_MOSI MCP23S17_SPI_MOSI_PIN
#define PIN_SPI_SCLK MCP23S17_SPI_SCLK_PIN

// MCP23S17 pin assignments (0-15)
// X-axis signals are on Port A (0-7), Y-axis and Theta signals are on Port B (8-15).
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
#define PIN_THETA_DIR 14
#define PIN_THETA_ENABLE 15

// Direct ESP32 GPIO pins (hardware peripheral requirements)
// NOTE:
// - Encoder A/B must stay on direct GPIO for PCNT.
// - X/Y pulse and Theta PWM stay on direct GPIO for LEDC/PWM generation.
// IMPORTANT: avoid driving GPIO32/33 as pulse outputs on this hardware setup.
// For Ethernet compatibility, avoid GPIO17 pulse usage (ETH clock output).
#define PIN_X_PULSE 14
#define PIN_Y_PULSE 2
// PIN_X_ENC_A: GPIO4 (general-purpose header pin). Previously GPIO34 which is
// not routed to any WT32-ETH01 header pad, and before that GPIO35 which shares
// a trace with MCP23S17 SPI MISO. GPIO4 has no strap/Ethernet conflict and
// PCNT can route any GPIO via the input matrix.
#define PIN_X_ENC_A 4
#define PIN_X_ENC_B 36
#define PIN_Y_ENC_A 39
#define PIN_Y_ENC_B 32
#define PIN_THETA_PULSE 0
#define PIN_THETA_ENC_A -1
#define PIN_THETA_ENC_B -1

// Hardware peripheral channel allocation
#define X_PULSE_LEDC_CHANNEL 0
#define Y_PULSE_LEDC_CHANNEL 1
#define THETA_PULSE_LEDC_CHANNEL 2
#define X_ENCODER_PCNT_UNIT 0
#define Y_ENCODER_PCNT_UNIT 1
#define THETA_ENCODER_PCNT_UNIT 2

// MCP23S17 defaults
#define MCP23S17_DEVICE_ADDRESS 0x00
#define MCP23S17_CLOCK_HZ 10000000
// Keep MCP diagnostics available by default; set to 0 for lean production console.
#define MCP_DEBUG_CMDS 1

// Backwards-compatible aliases for existing call sites
#define GANTRY_HOMING_SPEED_PPS AXIS_X_HOMING_SPEED_PPS
#define GANTRY_ENCODER_PPR AXIS_X_ENCODER_PPR
#define GANTRY_MAX_PULSE_FREQ AXIS_X_MAX_PULSE_FREQ_HZ
#define GANTRY_X_MIN_MM AXIS_X_TRAVEL_MIN_MM
#define GANTRY_X_MAX_MM AXIS_X_TRAVEL_MAX_MM
#define GANTRY_Y_STEPS_PER_MM AXIS_Y_PULSES_PER_MM
#define GANTRY_Y_MIN_MM AXIS_Y_TRAVEL_MIN_MM
#define GANTRY_Y_MAX_MM AXIS_Y_TRAVEL_MAX_MM
#define GANTRY_Y_MAX_SPEED_MMPS AXIS_Y_MAX_SPEED_MM_PER_S
#define GANTRY_Y_ACCEL_MMPS2 AXIS_Y_ACCEL_MM_PER_S2
#define GANTRY_Y_DECEL_MMPS2 AXIS_Y_DECEL_MM_PER_S2
#define GANTRY_THETA_MIN_DEG AXIS_THETA_TRAVEL_MIN_DEG
#define GANTRY_THETA_MAX_DEG AXIS_THETA_TRAVEL_MAX_DEG
#define GANTRY_SAFE_Y_MM GANTRY_SAFE_Y_HEIGHT_MM

// Task defaults
#define GANTRY_UPDATE_TASK_STACK 4096
#define GANTRY_UPDATE_TASK_PRIORITY 5
#define GANTRY_UPDATE_TASK_CORE 1
#define CONSOLE_TASK_STACK 4096
#define CONSOLE_TASK_PRIORITY 1
#define CONSOLE_TASK_CORE 0

#endif  // GANTRY_APP_CONSTANTS_H

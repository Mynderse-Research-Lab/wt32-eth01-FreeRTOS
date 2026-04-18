#ifndef GANTRY_APP_CONSTANTS_H
#define GANTRY_APP_CONSTANTS_H

/**
 * @file gantry_app_constants.h
 * @brief Pin map, hardware-peripheral channel allocation, task parameters.
 *
 * Motor/driver electrical tuning lives in axis_pulse_motor_params.h.
 * Mechanical drivetrain / travel / motion caps live in axis_drivetrain_params.h.
 * This header owns ONLY what is unique to this board + firmware build:
 *   - MCP23S17 bus & logical pin assignments
 *   - Direct ESP32 GPIOs that must stay on hardware peripherals (LEDC/PCNT)
 *   - LEDC channel / PCNT unit allocation
 *   - FreeRTOS task stacks, priorities, and core affinities
 */

// MCP23S17 is mandatory for this application.
#define APP_USE_MCP23S17 1

// Known-good MCP23S17 SPI mapping/clock validated on WT32-ETH01 + MIKROE-951.
#define MCP23S17_SPI_CS_PIN               15
#define MCP23S17_SPI_MISO_PIN             35
#define MCP23S17_SPI_MOSI_PIN             12
#define MCP23S17_SPI_SCLK_PIN              5
#define MCP23S17_SPI_CLOCK_HZ_WORKING  1000000

// Backward-compatible aliases (legacy name style used in older modules/docs).
#define PIN_SPI_CS   MCP23S17_SPI_CS_PIN
#define PIN_SPI_MISO MCP23S17_SPI_MISO_PIN
#define PIN_SPI_MOSI MCP23S17_SPI_MOSI_PIN
#define PIN_SPI_SCLK MCP23S17_SPI_SCLK_PIN

// ============================================================================
// MCP23S17 logical pin assignments (0-15)
// ============================================================================
// Port A (0..7) - X-axis signals + gripper
#define PIN_X_DIR             0
#define PIN_X_ENABLE          1
#define PIN_X_LIMIT_MIN       2
#define PIN_X_LIMIT_MAX       3
#define PIN_X_ALARM_STATUS    4
#define PIN_X_ALARM_RESET     5
// PA6 (6) is currently unassigned (available).
#define PIN_GRIPPER           7

// Port B (8..15) - Y-axis signals + theta pulse-train control
#define PIN_Y_DIR             8
#define PIN_Y_ENABLE          9
#define PIN_Y_LIMIT_MIN      10
#define PIN_Y_LIMIT_MAX      11
#define PIN_Y_ALARM_STATUS   12
#define PIN_Y_ALARM_RESET   13
// Theta axis pulse-train control lines.
// Previously pins 14 and 15 were PIN_THETA_LIMIT_MIN / PIN_THETA_LIMIT_MAX for
// a PWM hobby-servo configuration. With theta moving to pulse-train control
// (custom driver for SCHUNK ERD 04-40-D-H-N), these become DIR / ENABLE lines.
// The ERD has unlimited rotation, so inductive limit switches are not used;
// firmware clamps rotation to AXIS_THETA_HARD_LIMIT_MIN/MAX_DEG (cable
// management), and soft limits derived at boot-time calibration may be
// tighter.
#define PIN_THETA_DIR        14
#define PIN_THETA_ENABLE     15

// ============================================================================
// Direct ESP32 GPIO pins (hardware peripheral requirements)
// ============================================================================
// - Encoder A/B must stay on direct GPIO for PCNT.
// - Pulse outputs must stay on direct GPIO for LEDC.
// - Avoid driving GPIO32/33 as pulse outputs on this hardware setup.
// - For Ethernet compatibility, avoid GPIO17 pulse usage (ETH clock output).
#define PIN_X_PULSE          14
#define PIN_Y_PULSE           2
// PIN_X_ENC_A: GPIO4 (general-purpose header pin). Previously GPIO34 which is
// not routed to any WT32-ETH01 header pad, and before that GPIO35 which shares
// a trace with MCP23S17 SPI MISO. GPIO4 has no strap/Ethernet conflict and
// PCNT can route any GPIO via the input matrix.
#define PIN_X_ENC_A           4
#define PIN_X_ENC_B          36
#define PIN_Y_ENC_A          39
#define PIN_Y_ENC_B          32
// Theta pulse output. GPIO0 is a boot-strapping pin; keep HIGH during reset
// for normal boot. The custom ERD pulse-train driver tolerates this because
// commissioning releases the pin to LEDC only after reset completes.
#define PIN_THETA_PULSE       0

// Theta encoder pins: not yet wired in this hardware revision. The custom
// ERD driver terminates HIPERFACE internally; only the echoed pulse/dir
// feedback (if the driver provides one) would be routed back. Leave
// unassigned for now - the DriverConfig simply carries -1 for these.
#define PIN_THETA_ENC_A      -1
#define PIN_THETA_ENC_B      -1

// Legacy alias: previous PWM-hobby-servo build used PIN_THETA_PWM.
// Keep the alias for any consumer that still spells it that way; schedule
// removal for the next cleanup pass once all references are updated.
#define PIN_THETA_PWM         PIN_THETA_PULSE

// ============================================================================
// Hardware peripheral channel allocation
// ============================================================================
#define X_PULSE_LEDC_CHANNEL     0
#define Y_PULSE_LEDC_CHANNEL     1
#define THETA_PULSE_LEDC_CHANNEL 2
// Legacy alias (previous PWM-hobby-servo build).
#define THETA_PWM_LEDC_CHANNEL   THETA_PULSE_LEDC_CHANNEL

#define X_ENCODER_PCNT_UNIT      0
#define Y_ENCODER_PCNT_UNIT      1
#define THETA_ENCODER_PCNT_UNIT  2

// MCP23S17 defaults
#define MCP23S17_DEVICE_ADDRESS 0x00
#define MCP23S17_CLOCK_HZ 10000000
// Keep MCP diagnostics available by default; set to 0 for lean production console.
#define MCP_DEBUG_CMDS 1

// ============================================================================
// FreeRTOS task parameters
// ============================================================================
#define GANTRY_UPDATE_TASK_STACK    4096
#define GANTRY_UPDATE_TASK_PRIORITY    5
#define GANTRY_UPDATE_TASK_CORE        1
#define CONSOLE_TASK_STACK          4096
#define CONSOLE_TASK_PRIORITY          1
#define CONSOLE_TASK_CORE              0

#endif  // GANTRY_APP_CONSTANTS_H

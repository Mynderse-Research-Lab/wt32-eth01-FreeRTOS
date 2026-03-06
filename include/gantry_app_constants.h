#ifndef GANTRY_APP_CONSTANTS_H
#define GANTRY_APP_CONSTANTS_H

// MCP23S17 SPI pins (direct ESP32 GPIO)
// Chosen to avoid WT32-ETH01 Ethernet PHY pins (18, 19, 23 are NOT available).
#define PIN_SPI_MISO 4    // GPIO 4  — general purpose, no boot constraints
#define PIN_SPI_MOSI 15   // GPIO 15 — default pull-up keeps HIGH at boot (required)
#define PIN_SPI_SCLK 14   // GPIO 14 — general purpose, no boot constraints
#define PIN_SPI_CS 5      // GPIO 5  — exposed on header

// MCP23S17 pin assignments (0-15)
// Port A (pins 0–7)
#define PIN_DIR 1              // PA1 — X-axis direction (Servo SIGN/DIR)
#define PIN_ENABLE 2           // PA2 — X-axis servo enable (SON)
#define PIN_LIMIT_X_MIN 3     // PA3 — X-axis home limit switch (active low, pull-up)
#define PIN_LIMIT_X_MAX 4     // PA4 — X-axis end limit switch (active low, pull-up)
#define PIN_Y_STEP 5           // PA5 — Y-axis stepper pulse
#define PIN_Y_DIR 6            // PA6 — Y-axis stepper direction
#define PIN_Y_ENABLE 7         // PA7 — Y-axis stepper enable

// Port B (pins 8–15)
#define PIN_GRIPPER 8          // PB0 — End-effector gripper control
#define PIN_LIMIT_Y_MIN 9     // PB1 — Y-axis home limit switch (active low, pull-up)
#define PIN_LIMIT_Y_MAX 10    // PB2 — Y-axis end limit switch (active low, pull-up)
#define PIN_X_ALARM_STATUS 11 // PB3 — X-axis alarm status (Drive OUT3, active low)
#define PIN_Y_ALARM_STATUS 12 // PB4 — Y-axis alarm status (Drive OUT3, active low)
#define PIN_X_ALARM_RESET 13  // PB5 — X-axis alarm reset (Drive IN1/ARST)
#define PIN_Y_ALARM_RESET 14  // PB6 — Y-axis alarm reset (Drive IN1/ARST)
#define PIN_X_PULSE_INHIB 15  // PB7 — X-axis pulse inhibit (Drive IN5/INH)

// Direct ESP32 GPIO pins (exposed on WT32-ETH01 headers)
#define PIN_PULSE 32      // GPIO 32 — LEDC pulse output (labelled CFG)
#define PIN_ENC_A 35      // GPIO 35 — PCNT input (input-only)
#define PIN_ENC_B 36      // GPIO 36 — PCNT input (input-only)
#define PIN_THETA_PWM 2   // GPIO 2  — LEDC PWM output (exposed as IO2)

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

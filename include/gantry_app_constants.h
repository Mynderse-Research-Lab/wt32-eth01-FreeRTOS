#ifndef GANTRY_APP_CONSTANTS_H
#define GANTRY_APP_CONSTANTS_H

/**
 * @file gantry_app_constants.h
 * @brief Pin map, hardware-peripheral channel allocation, task parameters.
 *
 * All drive control is EtherNet/IP over W5500 SPI. MCP23S17 and step/direction
 * removed per 2026-07 refactor. I2C display uses GPIO4/14; gripper and W5500
 * RST relocated to free those lines.
 */

// ============================================================================
// W5500 SPI Ethernet (WIZ850io) — EtherNet/IP daisy-chain to drives
// ============================================================================
#define W5500_SPI_HOST     2        // HSPI_HOST (same bus MCP23S17 used)
#define W5500_CS_GPIO     15
#define W5500_INT_GPIO    33        // Interrupt (optional; polled mode without it)
#define W5500_RST_GPIO    32        // HW reset (relocated; GPIO14 is I2C SCL)
#define W5500_MOSI_GPIO   12
#define W5500_MISO_GPIO   35
#define W5500_SCLK_GPIO    5
#define W5500_SCLK_HZ     20000000  // 20 MHz conservative (max 80 MHz)

// ============================================================================
// I2C display (SSD1306/SH1106-class) — stub driver in lib/I2cDisplay
// ============================================================================
#define I2C_DISPLAY_SDA_GPIO   4
#define I2C_DISPLAY_SCL_GPIO  14
#define I2C_DISPLAY_ADDR    0x3C
#define I2C_DISPLAY_SCL_HZ  400000

// ============================================================================
// Gripper (direct GPIO — was MCP PA7, then GPIO4; GPIO4 now I2C SDA)
// ============================================================================
#define PIN_GRIPPER      17

// ============================================================================
// FreeRTOS task parameters
// ============================================================================
#define GANTRY_UPDATE_TASK_STACK    4096
#define GANTRY_UPDATE_TASK_PRIORITY    5
#define GANTRY_UPDATE_TASK_CORE        1
#define CONSOLE_TASK_STACK          4096
#define CONSOLE_TASK_PRIORITY          1
#define CONSOLE_TASK_CORE              0

#define PICK_SCHEDULER_TASK_STACK   4096
#define PICK_SCHEDULER_TASK_PRIORITY   4
#define PICK_SCHEDULER_TASK_CORE       1

#endif  // GANTRY_APP_CONSTANTS_H

#ifndef GANTRY_APP_CONSTANTS_H
#define GANTRY_APP_CONSTANTS_H

/**
 * @file gantry_app_constants.h
 * @brief Pin map, hardware-peripheral channel allocation, task parameters.
 *
 * All drive control is EtherNet/IP over W5500 SPI. MCP23S17 and step/direction
 * removed per 2026-07 refactor. Only W5500 SPI pins, gripper GPIO, and
 * FreeRTOS task parameters remain.
 */

// ============================================================================
// W5500 SPI Ethernet (WIZ850io) — EtherNet/IP daisy-chain to drives
// ============================================================================
#define W5500_SPI_HOST     2        // HSPI_HOST (same bus MCP23S17 used)
#define W5500_CS_GPIO     15
#define W5500_INT_GPIO    33        // Interrupt (optional; polled mode without it)
#define W5500_RST_GPIO    14        // HW reset (GPIO 17 is RMII CLK for LAN8720)
#define W5500_MOSI_GPIO   12
#define W5500_MISO_GPIO   35
#define W5500_SCLK_GPIO    5
#define W5500_SCLK_HZ     20000000  // 20 MHz conservative (max 80 MHz)

// ============================================================================
// Gripper (direct GPIO — was MCP PA7, reassigned to GPIO4)
// ============================================================================
// GPIO4: fewest alternate functions (ADC2_CH0, TOUCH0, RTC_GPIO10).
// No strapping, JTAG, or EMAC conflicts.
#define PIN_GRIPPER       4

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

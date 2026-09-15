#ifndef GANTRY_APP_CONSTANTS_H
#define GANTRY_APP_CONSTANTS_H

/**
 * @file gantry_app_constants.h
 * @brief Pin map, peripheral channels, task parameters.
 *
 * Drive control: EtherNet/IP over W5500 (SPI2) — highest priority vs SPI3.
 * SPI3: MCP23S17 (default CS) + TFT (CS/DC/RES on MCP; BLK hardwired).
 * Free ESP ADC: GPIO12, 32, 33, 39 (see pinout.csv).
 */

// ============================================================================
// W5500 SPI Ethernet (WIZ850io) — EtherNet/IP daisy-chain (SPI2 / HSPI)
// Pinout per WT32-ETH01-IM-REV02-PCB schematic:
//   SCLK: GPIO12
//   MISO: GPIO5
//   MOSI: GPIO15
//   CS:   GPIO33
//   RST:  MCP PB7
// ============================================================================
#define W5500_SPI_HOST     1        // SPI2_HOST
#define W5500_CS_GPIO     33        // WT32-ETH01-IM-REV02-PCB schematic
#define W5500_INT_GPIO    (-1)      // Unused (Class 1 polled)
#define W5500_RST_GPIO    (-1)      // Hardware RST via MCP PB7 (not ESP GPIO)
#define W5500_MOSI_GPIO   15        // WT32-ETH01-IM-REV02-PCB schematic
#define W5500_MISO_GPIO    5        // WT32-ETH01-IM-REV02-PCB schematic
#define W5500_SCLK_GPIO   12        // WT32-ETH01-IM-REV02-PCB schematic
// ESP-IDF v6 full-duplex on GPIO-matrix pins max ~26.67 MHz (80/3).
// Use 10 MHz — reliable over GPIO matrix and board traces; 20 MHz can be set via Kconfig if verified.
#if defined(CONFIG_EIP_W5500_SPI_HZ)
#define W5500_SCLK_HZ     CONFIG_EIP_W5500_SPI_HZ
#else
#define W5500_SCLK_HZ     10000000
#endif

// ============================================================================
// SPI3 shared bus — MCP23S17 (default) + TFT (software CS on MCP)
// ============================================================================
#define SPI3_HOST_ID           2    // SPI3_HOST / VSPI
#define SPI3_SCLK_GPIO        14
#define SPI3_MOSI_GPIO         4
#define SPI3_MISO_GPIO        36
#define SPI3_CS_MCP_GPIO       2    // Idle HIGH; boot strap safe; only ESP CS on SPI3
#define SPI3_MCP_CLOCK_HZ   1000000
#define SPI3_TFT_CLOCK_HZ  20000000
#define MCP23S17_HW_ADDR    0x00    // A0=A1=A2=GND → opcode addr 0

// SPI3 CS pins: MCP23S17 uses GPIO2.
#define SPI3_CS_TFT_GPIO      32    // Unique ESP32 GPIO for TFT CS
#define TFT_BLK_GPIO          17    // Unique ESP32 GPIO for TFT Backlight PWM

// MCP23S17 logical pins 0..15 (Port A 0..7, Port B 8..15)
#define MCP_FIELD_DOUT0        0    // PA0 — gripper (Field 24 V OUT0)
#define MCP_FIELD_DOUT1        1
#define MCP_FIELD_DOUT2        2
#define MCP_FIELD_DOUT3        3
#define MCP_FIELD_DIN0         4    // PA4
#define MCP_FIELD_DIN1         5
#define MCP_FIELD_DIN2         6
#define MCP_FIELD_DIN3         7

// Port B: Disp1 / UI / W5500 RST (WT32-ETH01-IM-REV02-PCB schematic)
#define MCP_UI_ENC_KO          8    // PB0 — module KO / key out (input)
#define MCP_UI_ENC_PUSH        9    // PB1 — encoder push button (input)
#define MCP_UI_ENC_B          10    // PB2 — encoder B (input)
#define MCP_UI_ENC_A          11    // PB3 — encoder A (input)
#define MCP_TFT_BLK           12    // PB4 — Unused (Moved to ESP32 GPIO17 for PWM)
#define MCP_TFT_DC            13    // PB5 — TFT data/command (output)
#define MCP_TFT_RES           14    // PB6 — TFT reset (output)
#define MCP_W5500_RST         15    // PB7 — WIZ850io RSTn (active low)

#define FIELD_24V_DOUT_COUNT   4
#define FIELD_24V_DIN_COUNT    4

// Free ESP ADC inputs (isolator → 0..3.3 V): GPIO17, 32, 35, 39
#define ADC_FREE_GPIO_0       17
#define ADC_FREE_GPIO_1       32
#define ADC_FREE_GPIO_2       35
#define ADC_FREE_GPIO_3       39

// ============================================================================
// Gripper = Field 24 V DOUT0 (MCP PA0)
// ============================================================================
#define PIN_GRIPPER      MCP_FIELD_DOUT0

// ============================================================================
// FreeRTOS task parameters
// Class 1 scanner priority is set in EipScannerTask (above gantry/SPI3 users).
// ============================================================================
#define GANTRY_UPDATE_TASK_STACK    4096
#define GANTRY_UPDATE_TASK_PRIORITY    5
#define GANTRY_UPDATE_TASK_CORE        1
#define CONSOLE_TASK_STACK          4096
// Above TftUiTask (2): UART/TCP must preempt SPI3 UI redraws under Class 1 load.
#define CONSOLE_TASK_PRIORITY          3
#define CONSOLE_TASK_CORE              0
#define TFT_UI_TASK_STACK           4096
#define TFT_UI_TASK_PRIORITY           2
#define TFT_UI_TASK_CORE               0

#define PICK_SCHEDULER_TASK_STACK   4096
#define PICK_SCHEDULER_TASK_PRIORITY   4
#define PICK_SCHEDULER_TASK_CORE       1

#endif  // GANTRY_APP_CONSTANTS_H

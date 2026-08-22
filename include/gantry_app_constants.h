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
// ESP32 enum: SPI1=0 (flash), SPI2_HOST=1, SPI3_HOST=2 — do not use literal "2" for SPI2.
// ============================================================================
#define W5500_SPI_HOST     1        // SPI2_HOST
#define W5500_CS_GPIO     15        // Required for VDM frame edges (not hardwireable)
#define W5500_INT_GPIO    (-1)      // Unused (Class 1 polled)
#define W5500_RST_GPIO    (-1)      // Hardware RST via MCP PB7 (not ESP GPIO)
#define W5500_MOSI_GPIO   17        // Non-ADC; frees GPIO12 for ADC
#define W5500_MISO_GPIO   35
#define W5500_SCLK_GPIO    5        // Non-ADC
// ESP-IDF v6 full-duplex on GPIO-matrix pins max ~26.67 MHz (80/3).
// Use 20 MHz (80/4) — reliable; chip datasheet allows up to 80 MHz on short traces.
#if defined(CONFIG_EIP_W5500_SPI_HZ)
#define W5500_SCLK_HZ     CONFIG_EIP_W5500_SPI_HZ
#else
#define W5500_SCLK_HZ     20000000
#endif

// ============================================================================
// SPI3 shared bus — MCP23S17 (default) + TFT (software CS on MCP)
// ============================================================================
#define SPI3_HOST_ID           2    // SPI3_HOST / VSPI
#define SPI3_SCLK_GPIO        14
#define SPI3_MOSI_GPIO         4
#define SPI3_MISO_GPIO        36
#define SPI3_CS_MCP_GPIO       2    // Idle HIGH; boot strap safe; only ESP CS on SPI3
#define SPI3_MCP_CLOCK_HZ  10000000
#define SPI3_TFT_CLOCK_HZ  20000000
#define MCP23S17_HW_ADDR    0x00    // A0=A1=A2=GND → opcode addr 0

// MCP23S17 logical pins 0..15 (Port A 0..7, Port B 8..15)
#define MCP_FIELD_DOUT0        0    // PA0 — gripper (Field 24 V OUT0)
#define MCP_FIELD_DOUT1        1
#define MCP_FIELD_DOUT2        2
#define MCP_FIELD_DOUT3        3
#define MCP_FIELD_DIN0         4    // PA4
#define MCP_FIELD_DIN1         5
#define MCP_FIELD_DIN2         6
#define MCP_FIELD_DIN3         7
#define MCP_TFT_DC             8    // PB0
#define MCP_TFT_RES            9    // PB1
#define MCP_TFT_CS            10    // PB2 — TFT chip select (idle HIGH)
#define MCP_TFT_BLK           (-1)  // Hardwired ON (+5V); not MCP-driven
#define MCP_UI_ENC_A          11    // PB3
#define MCP_UI_ENC_B          12    // PB4
#define MCP_UI_ENC_PUSH       13    // PB5
#define MCP_UI_ENC_KO         14    // PB6 — module KO / key (input)
#define MCP_W5500_RST         15    // PB7 — WIZ850io RSTn (active low)

#define FIELD_24V_DOUT_COUNT   4
#define FIELD_24V_DIN_COUNT    4

// Free ESP ADC inputs (isolator → 0..3.3 V): GPIO12, 32, 33, 39
#define ADC_FREE_GPIO_0       12
#define ADC_FREE_GPIO_1       32
#define ADC_FREE_GPIO_2       33
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
#define CONSOLE_TASK_PRIORITY          1
#define CONSOLE_TASK_CORE              0

#define PICK_SCHEDULER_TASK_STACK   4096
#define PICK_SCHEDULER_TASK_PRIORITY   4
#define PICK_SCHEDULER_TASK_CORE       1

#endif  // GANTRY_APP_CONSTANTS_H

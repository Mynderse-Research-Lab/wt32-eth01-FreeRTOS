/**
 * @file main.cpp
 * @brief Gantry control application for WT32-ETH01 with W5500 EtherNet/IP.
 *
 * Coordinate convention (firmware-wide, as of 2026-08):
 *   X = horizontal traverse (across belt), Y = along-belt (no gantry actuator;
 *   conveyor downstream = +Y), Z = vertical (+Z = down). Joint Z=0 is A015
 *   retract datum; physical offset: GANTRY_Z_DATUM_OFFSET_ABOVE_BED_MM
 *   (axis_drivetrain_params.h).
 *
 * FreeRTOS application with:
 * - W5500 SPI Ethernet for EtherNet/IP drive control (X, Z)
 * - LAN8720 RMII for MQTT bridge
 * - SPI3: MCP (Field, TFT CS/DC/RES/BLK, W5500 RST, encoder) + TFT stub
 * - End-effector: SCHUNK gripper on Field DOUT0 (MCP PA0)
 * - Free ESP ADC GPIOs 12/32/33/39; Class 1 priority over SPI3
 * - Interactive serial console (gantry_test_console)
 * - Periodic gantry update task at 100 Hz
 *
 * Pin assignments live in gantry_app_constants.h / pinout.csv.
 */

// Ask axis_drivetrain_params.h to emit its deployment-time reminders in this
// TU only.
#define AXIS_DRIVETRAIN_PARAMS_EMIT_WARNINGS

#include "main_internal.h"

#include "AppConfig.h"
#include "axis_drivetrain_params.h"
#include "boot_log_ring.h"
#include "esp_log.h"
#include "ethernet_app_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gantry_ota.h"
#include "gantry_test_console.h"
#include "sdkconfig.h"

// ---------------------------------------------------------------------------
// app_main
// ---------------------------------------------------------------------------
extern "C" void app_main(void) {
    // CellNet OTA (N-2): after REBOOT_TO_OTA, run OTA-only mode and nothing else.
    uint16_t ota_session = 0;
    if (gantryOtaCellNetModeRequested(&ota_session)) {
        gantryOtaRunCellNetMode(ota_session);  // does not return
        return;
    }
    // Before the console task starts. Registered tables are searched first.
    size_t pick_n = 0;
    gantryConsoleRegisterCommands(gantryConsolePickCommands(&pick_n), pick_n);
#if CONFIG_GANTRY_SELFTEST
    size_t selftest_n = 0;
    gantryConsoleRegisterCommands(gantryConsoleSelfTestCommands(&selftest_n), selftest_n);
#endif
    // First: keep start-up logs readable from the LAN console ('bootlog').
    bootLogRingInstall();
    ESP_LOGI(TAG, "\n========================================");
    ESP_LOGI(TAG, "WT32-ETH01 Gantry Controller (EIP over W5500)");
#if defined(CONFIG_EIP_AXIS_THETA)
    ESP_LOGI(TAG, "EIP line: WT32 -> X -> Z -> Theta HCS01 (%s)",
             CONFIG_EIP_TARGET_IP_THETA);
#else
    ESP_LOGI(TAG, "EIP line: WT32 -> X -> Z (Theta gated off; PC uplink exclusive)");
#endif
    ESP_LOGI(TAG, "========================================\n");

    bootInitConfig();
    bootInitIo();
    const BootMotion motion = bootInitMotion();
    if (motion.gantry == nullptr) {
        return;
    }
    Gantry::Gantry& gantry = *motion.gantry;

    // ------------------------------------------------------------------
    // FreeRTOS tasks first — do not block console / motion behind network wait.
    // ------------------------------------------------------------------
    static Network::EthernetLink ethernetLink;
    static CellNet::EspEthL2Transport l2Transport;
    static CellNet::CellNetL2Node l2Node(l2Transport, CellNodeId::GANTRY);

    GantryTestConsoleConfig* consoleCfg =
        bootStartTasks(gantry, motion.w5500_hal, ethernetLink, l2Node);
    if (consoleCfg == nullptr) {
        return;
    }

    // ------------------------------------------------------------------
    // LAN8720 first so TCP console and Layer-2 link come online.
    // ------------------------------------------------------------------
    const bool ethUp =
        bootStartNetwork(gantry, *consoleCfg, ethernetLink, l2Transport, l2Node);

    ESP_LOGI(TAG, "All tasks created successfully (ETH %s, L2 %s)",
             ethUp ? "up" : "down", l2Node.isReady() ? "active" : "offline");

    // Confirm healthy boot to cancel OTA rollback timer/watchdog
    gantryOtaConfirmBootValid();
    Config::AppConfig::instance().commitPending();

#if CONSOLE_UART_ENABLE
    ESP_LOGI(TAG, "System ready - type 'help' (UART and/or TCP %d)", CONSOLE_TCP_PORT);
    gantryTestPrintHelp();
#else
    ESP_LOGI(TAG, "System ready - connect plant PC to TCP %d (e.g. tools/lan_debug_ui.py)",
             CONSOLE_TCP_PORT);
#endif

    vTaskDelete(nullptr);
}

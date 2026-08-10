/**
 * @file main.cpp
 * @brief Gantry control application for WT32-ETH01 with W5500 EtherNet/IP.
 *
 * Coordinate convention (firmware-wide, as of 2026-05):
 *   X = horizontal traverse (across belt), Y = along-belt (no gantry actuator;
 *   conveyor downstream = -Y), Z = vertical (+Z = up). Joint Z=0 is homing datum;
 *   physical bed offset: GANTRY_Z_DATUM_OFFSET_ABOVE_BED_MM (axis_drivetrain_params.h).
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

#include "Gantry.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "gantry_test_console.h"
#include "gantry_net_console.h"
#include "gantry_app_constants.h"
#include "axis_drivetrain_params.h"
#include "mqtt_topics.h"
#include "ethernet_app_config.h"
#include "MqttBridge.h"
#include "pick_scheduler.h"
#include "Spi3Bus.h"
#include "SpiDisplay.h"
#include "MCP23S17.h"
#include "gpio_expander.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

#if CONFIG_EIP_SCANNER_ENABLED
#include "W5500.h"
#include "W5500SpiHal.h"
#include "EipScannerTask.h"
#include "EipProcessImage.h"
#include "EipSocketW5500.h"
#include "GantryEipLinearAxis.h"
#if defined(CONFIG_EIP_AXIS_THETA)
#include "GantryEipRotaryAxis.h"
#endif
#endif

static const char* TAG = "GantryApp";

// ---------------------------------------------------------------------------
// Task parameter structures
// ---------------------------------------------------------------------------
struct UpdateTaskConfig {
    Gantry::Gantry* gantry;
};

// ---------------------------------------------------------------------------
// Gantry periodic update task (100 Hz on Core 1)
// ---------------------------------------------------------------------------
void gantryUpdateTask(void* param) {
    auto* cfg = static_cast<UpdateTaskConfig*>(param);
    if (cfg == nullptr || cfg->gantry == nullptr) {
        ESP_LOGE(TAG, "Invalid Gantry update task config");
        vTaskDelete(nullptr);
        return;
    }

    const TickType_t updateInterval = pdMS_TO_TICKS(10);
    ESP_LOGI(TAG, "Gantry update task started (100 Hz)");

    while (1) {
        cfg->gantry->update();
        vTaskDelay(updateInterval);
    }
}

// ---------------------------------------------------------------------------
// app_main
// ---------------------------------------------------------------------------
extern "C" void app_main(void) {
    ESP_LOGI(TAG, "\n========================================");
    ESP_LOGI(TAG, "WT32-ETH01 Gantry Controller (EIP over W5500)");
    ESP_LOGI(TAG, "EIP line: WT32 -> X -> Z (Theta unpowered; PC uplink exclusive)");
    ESP_LOGI(TAG, "========================================\n");

    // SPI3 shared bus (MCP default client) + MCP23S17 Field/UI + TFT stub
    static display::SpiDisplay spiDisplay;
    if (!spi3::init()) {
        ESP_LOGW(TAG, "SPI3 bus init failed — continuing without MCP/TFT");
    } else {
        mcp23s17_config_t mcpCfg = {};
        mcpCfg.spi_host = spi3::host();
        mcpCfg.cs_pin = static_cast<gpio_num_t>(SPI3_CS_MCP_GPIO);
        mcpCfg.miso_pin = static_cast<gpio_num_t>(SPI3_MISO_GPIO);
        mcpCfg.mosi_pin = static_cast<gpio_num_t>(SPI3_MOSI_GPIO);
        mcpCfg.sclk_pin = static_cast<gpio_num_t>(SPI3_SCLK_GPIO);
        mcpCfg.device_address = MCP23S17_HW_ADDR;
        mcpCfg.clock_speed_hz = SPI3_MCP_CLOCK_HZ;
        mcpCfg.skip_bus_init = true;
        if (!gpio_expander_init(&mcpCfg)) {
            ESP_LOGW(TAG, "MCP23S17 init failed — Field I/O unavailable");
        } else if (gpio_expander_configure_field_and_ui() != ESP_OK) {
            ESP_LOGW(TAG, "MCP Field/UI pin configure failed");
        } else {
            display::SpiDisplayConfig dispCfg = {};
            dispCfg.mcp = gpio_expander_get_mcp_handle();
            dispCfg.mcp_cs_pin = MCP_TFT_CS;
            dispCfg.mcp_dc_pin = MCP_TFT_DC;
            dispCfg.mcp_res_pin = MCP_TFT_RES;
            dispCfg.mcp_blk_pin = MCP_TFT_BLK;
            dispCfg.clock_hz = SPI3_TFT_CLOCK_HZ;
            if (!spiDisplay.begin(dispCfg)) {
                ESP_LOGW(TAG, "SpiDisplay stub begin failed — MCP Field I/O still live");
            } else {
                (void)spiDisplay.refreshStub();
            }
        }
    }

#if CONFIG_EIP_SCANNER_ENABLED
    // --- W5500 init (must outlive scanner task; app_main deletes itself) ---
    static W5500 w5500;
    W5500Config w5500Cfg = {};
    w5500Cfg.spi_host  = W5500_SPI_HOST;
    w5500Cfg.cs_gpio   = W5500_CS_GPIO;
    w5500Cfg.int_gpio  = W5500_INT_GPIO;
    w5500Cfg.rst_gpio  = W5500_RST_GPIO;  // -1; hardware RST via MCP PB7
    w5500Cfg.rst_set_level = gpio_expander_w5500_rst_set_level;
    w5500Cfg.rst_ctx   = nullptr;
    w5500Cfg.mosi_gpio = W5500_MOSI_GPIO;
    w5500Cfg.miso_gpio = W5500_MISO_GPIO;
    w5500Cfg.sclk_gpio = W5500_SCLK_GPIO;
    w5500Cfg.sclk_hz   = W5500_SCLK_HZ;
    ESP_LOGI(TAG, "W5500 pins: MOSI=%d MISO=%d SCLK=%d CS=%d RST=MCP_PB7 @ %d Hz",
             w5500Cfg.mosi_gpio, w5500Cfg.miso_gpio, w5500Cfg.sclk_gpio,
             w5500Cfg.cs_gpio, w5500Cfg.sclk_hz);
    if (!w5500.init(w5500Cfg)) {
        ESP_LOGE(TAG, "FATAL: W5500 init failed (check MOSI17/SCLK5/CS15/MISO35 + RST MCP PB7)");
        return;
    }
    ESP_LOGI(TAG, "W5500 initialized (version 0x%02X)", w5500.getVersion());

    // --- EIP process images (one per drive) ---
    static eip::EipProcessImage eipImageX;
    static eip::EipProcessImage eipImageZ;
#if defined(CONFIG_EIP_AXIS_THETA)
    static eip::EipProcessImage eipImageTheta;
#endif

    // Kinetix assembly 104 speed/accel/decel refs are 0.1 RPM (or 0.1 RPM/s).
    // Linear mm/s -> motor RPM: rpm = mm_s * i / lead * 60;
    // ref = rpm * 10 = 600 * i * mm_s / lead.
#if defined(CONFIG_EIP_AXIS_X)
    const double xSpeedRefPerMmS =
        600.0 * static_cast<double>(AXIS_X_MOTOR_REDUCER_RATIO) /
        static_cast<double>(AXIS_X_LEAD_MM_PER_REV);
    auto xAxis = std::make_unique<Gantry::GantryEipLinearAxis>(
        eipImageX, Gantry::EipLinearAxisConfig{
            CONFIG_EIP_AXIS_X_PUU_PER_MM, xSpeedRefPerMmS, xSpeedRefPerMmS,
            xSpeedRefPerMmS, AXIS_X_LEAD_MM_PER_REV});
    ESP_LOGI(TAG, "X axis over EIP (Kinetix 5100, %.1f PUU/mm, speed_ref/mm_s=%.3f), target %s",
             CONFIG_EIP_AXIS_X_PUU_PER_MM, xSpeedRefPerMmS, CONFIG_EIP_TARGET_IP_X);
#else
    auto xAxis = std::unique_ptr<Gantry::GantryLinearAxis>(nullptr);
#endif

#if defined(CONFIG_EIP_AXIS_Z)
    const double zSpeedRefPerMmS =
        600.0 * static_cast<double>(AXIS_Z_MOTOR_REDUCER_RATIO) /
        static_cast<double>(AXIS_Z_LEAD_MM_PER_REV);
    auto zAxis = std::make_unique<Gantry::GantryEipLinearAxis>(
        eipImageZ, Gantry::EipLinearAxisConfig{
            CONFIG_EIP_AXIS_Z_PUU_PER_MM, zSpeedRefPerMmS, zSpeedRefPerMmS,
            zSpeedRefPerMmS, AXIS_Z_LEAD_MM_PER_REV});
    ESP_LOGI(TAG, "Z axis over EIP (Kinetix 5100, %.1f PUU/mm, speed_ref/mm_s=%.3f), target %s",
             CONFIG_EIP_AXIS_Z_PUU_PER_MM, zSpeedRefPerMmS, CONFIG_EIP_TARGET_IP_Z);
#else
    auto zAxis = std::unique_ptr<Gantry::GantryLinearAxis>(nullptr);
#endif

#if defined(CONFIG_EIP_AXIS_THETA)
    auto thetaAxis = std::make_unique<Gantry::GantryEipRotaryAxis>(
        eipImageTheta, Gantry::EipRotaryAxisConfig{
            (double)AXIS_THETA_ENCODER_PPR / 360.0,
            (int32_t)AXIS_THETA_MAX_SPEED_DEG_PER_S * (int32_t)(AXIS_THETA_ENCODER_PPR / 360),
            (int32_t)AXIS_THETA_ACCEL_DEG_PER_S2 * (int32_t)(AXIS_THETA_ENCODER_PPR / 360),
            (int32_t)AXIS_THETA_DECEL_DEG_PER_S2 * (int32_t)(AXIS_THETA_ENCODER_PPR / 360)
        });
    ESP_LOGI(TAG, "Theta axis over EIP (HCS01, %.1f PUU/deg)", (double)AXIS_THETA_ENCODER_PPR / 360.0);
#else
    auto thetaAxis = std::unique_ptr<Gantry::GantryRotaryAxis>(nullptr);
#endif

    static Gantry::Gantry gantry(std::move(xAxis), std::move(zAxis),
        std::move(thetaAxis), PIN_GRIPPER);

    // Seed joint-limit envelope with mechanical hard limits.
    gantry.setJointLimits(AXIS_X_HARD_LIMIT_MIN_MM,     AXIS_X_HARD_LIMIT_MAX_MM,
                          AXIS_Z_HARD_LIMIT_MIN_MM,     AXIS_Z_HARD_LIMIT_MAX_MM,
                          AXIS_THETA_HARD_LIMIT_MIN_DEG, AXIS_THETA_HARD_LIMIT_MAX_DEG);
    gantry.setZAxisLimits(AXIS_Z_HARD_LIMIT_MIN_MM, AXIS_Z_HARD_LIMIT_MAX_MM);
    gantry.setThetaLimits(AXIS_THETA_HARD_LIMIT_MIN_DEG, AXIS_THETA_HARD_LIMIT_MAX_DEG);
    gantry.setSafeZHeight(GANTRY_SAFE_Z_HEIGHT_MM);

    ESP_LOGI(TAG, "Initializing gantry...");
    if (!gantry.begin()) {
        ESP_LOGE(TAG, "ERROR: Gantry initialization failed!");
        return;
    }
    ESP_LOGI(TAG, "OK Gantry initialized");
    // Defer gantry.enable() until Class 1 + GantryUpdate are running.
    // Boot enable before the scanner wastes the ServoOn edge (A603 on first move).
    ESP_LOGI(TAG, "Motors idle — run 'enable' after Class 1 is online");
#else
    // Non-EIP build: placeholder gantry, skip all init.
    static Gantry::Gantry gantry(
        std::unique_ptr<Gantry::GantryLinearAxis>(nullptr),
        std::unique_ptr<Gantry::GantryLinearAxis>(nullptr),
        std::unique_ptr<Gantry::GantryRotaryAxis>(nullptr),
        PIN_GRIPPER);
#endif

    // ------------------------------------------------------------------
    // EIP scanner tasks (before MQTT so daisy-chain stays alive even if
    // LAN8720/MQTT fails)
    // ------------------------------------------------------------------
#if CONFIG_EIP_SCANNER_ENABLED
    // W5500 link status adapter for scanner tasks
    class W5500LinkStatusAdapter : public eip::ILinkStatus {
        W5500& w5500_;
    public:
        explicit W5500LinkStatusAdapter(W5500& w) : w5500_(w) {}
        bool isUp() const override { return w5500_.isLinkUp(); }
    };
    static W5500LinkStatusAdapter w5500LinkStatus(w5500);
    static W5500SpiHal w5500Hal(w5500);

    eip::startScannerTask(w5500, w5500Hal, w5500LinkStatus,
#if defined(CONFIG_EIP_AXIS_X)
                          &eipImageX,
#else
                          nullptr,
#endif
#if defined(CONFIG_EIP_AXIS_Z)
                          &eipImageZ,
#else
                          nullptr,
#endif
#if defined(CONFIG_EIP_AXIS_THETA)
                          &eipImageTheta
#else
                          nullptr
#endif
    );
#endif

    // ------------------------------------------------------------------
    // FreeRTOS tasks first — do not block console / motion behind MQTT wait.
    // Construct Bridge before tasks take its address; start() runs after.
    // ------------------------------------------------------------------
    static MqttBridge::EthernetLink ethernetLink;
    static MqttBridge::Bridge mqttBridge(&ethernetLink);

    BaseType_t result;
    static PickSchedulerTaskConfig pickCfg = { &gantry, &mqttBridge };
    result = xTaskCreatePinnedToCore(
        pickSchedulerTask, "PickScheduler",
        PICK_SCHEDULER_TASK_STACK, &pickCfg,
        PICK_SCHEDULER_TASK_PRIORITY, nullptr, PICK_SCHEDULER_TASK_CORE);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "FATAL: Failed to create PickScheduler task");
        gantry.disable();
        return;
    }
    static UpdateTaskConfig updateCfg = { &gantry };
    result = xTaskCreatePinnedToCore(
        gantryUpdateTask, "GantryUpdate",
        GANTRY_UPDATE_TASK_STACK, &updateCfg,
        GANTRY_UPDATE_TASK_PRIORITY, nullptr, GANTRY_UPDATE_TASK_CORE);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create Gantry Update task!");
    }

    static GantryTestConsoleConfig consoleCfg = {};
    consoleCfg.gantry                 = &gantry;
    consoleCfg.limit_min_pin          = -1;   // Endstops drive-managed (EIP)
    consoleCfg.limit_max_pin          = -1;
    consoleCfg.use_mcp23s17           = (gpio_expander_get_mcp_handle() != nullptr);
    consoleCfg.limit_switches_active  = false;

    result = xTaskCreatePinnedToCore(
        gantryTestConsoleTask, "SerialCmd",
        CONSOLE_TASK_STACK, &consoleCfg,
        CONSOLE_TASK_PRIORITY, nullptr, CONSOLE_TASK_CORE);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create Serial task!");
    }

    // ------------------------------------------------------------------
    // LAN8720 first so TCP console works even if MQTT broker is down.
    // Bridge::start() reuses EthernetLink (idempotent start()).
    // ------------------------------------------------------------------
    bool ethUp = false;
    if (ethernetLink.start() && ethernetLink.waitForUp(ETH_IP_WAIT_TIMEOUT_MS)) {
        ethUp = true;
        gantryNetConsoleStart(&consoleCfg);
        ESP_LOGI(TAG, "Net console listening on TCP %d (plant / LAN8720)",
                 CONSOLE_TCP_PORT);
    } else {
        ESP_LOGW(TAG, "LAN8720 not up — UART console only; net console skipped");
    }

    // ------------------------------------------------------------------
    // MQTT bridge (non-fatal — eth may already be up; EIP/console already live)
    // ------------------------------------------------------------------
    bool mqttReady = false;
    if (mqttBridge.start(MQTT_GANTRY_ID_DEFAULT)) {
        mqttReady = true;
        (void)mqttBridge.publishStatusJson("{\"state\":\"LINK_INIT\",\"source\":\"main\"}");
    } else {
        ESP_LOGW(TAG, "MQTT bridge failed to start — EIP and console will still work; "
                 "pick scheduling unavailable until LAN8720 link is restored.");
    }

    ESP_LOGI(TAG, "All tasks created successfully (ETH %s, MQTT %s)",
             ethUp ? "up" : "down", mqttReady ? "online" : "offline");
    ESP_LOGI(TAG, "System ready - type 'help' for commands (UART and/or TCP %d)",
             CONSOLE_TCP_PORT);
    gantryTestPrintHelp();

    vTaskDelete(nullptr);
}

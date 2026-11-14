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

#include "Gantry.h"
#include "GantryControlPeriod.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "gantry_test_console.h"
#include "gantry_net_console.h"
#include "gantry_app_constants.h"
#include "gantry_ota.h"
#include "axis_drivetrain_params.h"
#include "ethernet_app_config.h"
#include "CellNetL2.h"
#include "EspEthL2Transport.h"
#include "EthernetLink.h"
#include "esp_netif_sntp.h"
#include "pick_scheduler.h"
#include "boot_log_ring.h"
#include "Spi3Bus.h"
#include "SpiDisplay.h"
#include "UiManager.h"
#include "AppConfig.h"
#include "gantry_runtime_params.h"
#include "nvs_flash.h"
#include "MCP23S17.h"
#include "esp_timer.h"
#include "gpio_expander.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

#if CONFIG_EIP_SCANNER_ENABLED
#include "W5500.h"
#include "W5500Socket.h"
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

    const TickType_t updateInterval = pdMS_TO_TICKS(GANTRY_UPDATE_PERIOD_MS);
    Gantry::setControlPeriodUs(static_cast<uint32_t>(GANTRY_UPDATE_PERIOD_MS) * 1000u);
    ESP_LOGI(TAG, "Gantry update task started (%d ms)", GANTRY_UPDATE_PERIOD_MS);
    TickType_t lastWakeTime = xTaskGetTickCount();

    while (1) {
        cfg->gantry->update();
        vTaskDelayUntil(&lastWakeTime, updateInterval);
    }
}

// ---------------------------------------------------------------------------
// TFT UI: encoder polled ~100 Hz; display redraw 15 Hz (Core 0)
// ---------------------------------------------------------------------------
struct UiTaskConfig {
    Gantry::Gantry* gantry;
    Network::EthernetLink* eth;
};

void tftUiTask(void* param) {
    auto* cfg = static_cast<UiTaskConfig*>(param);
    display::DashboardTelemetry telem = {};
    // Quadrature needs >> detent rate: 15 Hz display undersamples the knob.
    const TickType_t inputInterval = pdMS_TO_TICKS(10);       // 100 Hz
    const TickType_t drawInterval = pdMS_TO_TICKS(1000 / 15);  // 15 Hz
    TickType_t lastWakeTime = xTaskGetTickCount();
    TickType_t lastDrawTime = lastWakeTime - drawInterval;  // draw on first pass

    ESP_LOGI(TAG, "TFT UI task started (input 100 Hz, draw 15 Hz)");

    while (1) {
        // Skip SPI3 while Class 1 critical — brief miss OK at 100 Hz input rate.
        if (!spi3_class1_critical_active()) {
            uint8_t port_b = gpio_expander_read_port_b();
            const TickType_t now = xTaskGetTickCount();
            const bool draw_due = (now - lastDrawTime) >= drawInterval;

            if (draw_due) {
                if (cfg && cfg->gantry) {
                    auto joints = cfg->gantry->getCurrentJointConfig();
                    telem.x_mm = joints.x;
                    telem.z_mm = joints.z;
                    telem.theta_deg = joints.theta;
                    telem.x_homed = !cfg->gantry->isBusy();
                    telem.z_homed = !cfg->gantry->isBusy();
                    telem.theta_homed = cfg->gantry->isThetaDriveOriginAligned();
                    telem.gripper_open = !cfg->gantry->isGripperActive();
                    telem.uptime_s =
                        static_cast<uint32_t>(esp_timer_get_time() / 1000000ULL);
                    telem.dev_unlocked =
                        Config::AppConfig::instance().isDeveloperModeUnlocked();
                    telem.motion_state = cfg->gantry->isBusy() ? "BUSY" : "IDLE";
                }

                if (cfg && cfg->eth) {
                    telem.lan_link = cfg->eth->isUp();
                    esp_netif_ip_info_t ip_info;
                    if (cfg->eth->getNetif() &&
                        esp_netif_get_ip_info(cfg->eth->getNetif(), &ip_info) ==
                            ESP_OK) {
                        esp_ip4addr_ntoa(&ip_info.ip, telem.lan_ip,
                                         sizeof(telem.lan_ip));
                    } else {
                        std::strncpy(
                            telem.lan_ip,
                            Config::AppConfig::instance().data().eth_static_ip,
                            sizeof(telem.lan_ip));
                    }
                }

                display::UiManager::instance().update(port_b, telem);
                lastDrawTime = now;
            } else {
                display::UiManager::instance().pollInput(port_b);
            }
        }

        vTaskDelayUntil(&lastWakeTime, inputInterval);
    }
}

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

    // Initialize NVS storage for runtime application parameters
    // Stored parameters only revert on an operator command, so unreadable NVS
    // is never erased here. 'cfg factory' erases it and starts a new set.
    const esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_OK) {
        Config::AppConfig::instance().init();
    }
    const Config::ConfigSetInfo active_set = Config::AppConfig::instance().activeSet();
    if (active_set.damaged) {
        ESP_LOGE(TAG, "Stored parameter set is damaged: running on compile-time defaults. "
                      "It is kept until 'cfg factory' or a parameter file reload");
    } else if (active_set.valid) {
        ESP_LOGI(TAG, "Parameter set #%lu loaded (%u parameters)",
                 static_cast<unsigned long>(active_set.seq),
                 static_cast<unsigned>(active_set.param_count));
    } else {
        ESP_LOGE(TAG, "Parameter set not stored (NVS %s): running on compile-time defaults, "
                      "nothing persists. 'cfg factory' erases NVS and starts over",
                 esp_err_to_name(nvs_err));
    }

    // cfg log_uart_async (reboot): without a UART driver every log line
    // busy-waits on UART0 at 115200 (~22 ms per 250-char line) while holding
    // stdout. On 2026-10-20 that stalled the Class 1 task 20-40 ms whenever it
    // logged behind a Gantry log burst, and X/theta timed out. With the driver
    // a line is copied into the TX ring and the UART drains it in the
    // background; a full ring still blocks, so it is sized for long bursts.
    if (Config::AppConfig::instance().data().log_uart_async != 0) {
        constexpr int kLogUartTxBuf = 16384;
        const esp_err_t uerr = uart_driver_install(
            static_cast<uart_port_t>(CONFIG_ESP_CONSOLE_UART_NUM), 256, kLogUartTxBuf,
            0, nullptr, 0);
        if (uerr == ESP_OK) {
            uart_vfs_dev_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);
            ESP_LOGW(TAG, "Log UART buffered (%d B TX ring, cfg log_uart_async=1)",
                     kLogUartTxBuf);
        } else {
            ESP_LOGW(TAG, "Log UART driver install failed: %s", esp_err_to_name(uerr));
        }
    }

    // Enable WT32 LAN8720 crystal (GPIO16) before other bring-up so REFCLK and
    // RJ45 LEDs can come up; EthernetLink::start() will re-assert if needed.
    if (!Network::EthernetLink::enablePhyOscillator()) {
        ESP_LOGW(TAG, "LAN8720 crystal enable failed — plant ETH / TCP :2323 may be unavailable");
    }

    // Diagnostic electrical line probe for SPI3 lines (CS=GPIO2, MISO=GPIO36)
    {
        gpio_config_t t_cs = {};
        t_cs.pin_bit_mask = (1ULL << SPI3_CS_MCP_GPIO);
        t_cs.mode = GPIO_MODE_INPUT_OUTPUT;
        t_cs.pull_up_en = GPIO_PULLUP_ENABLE;
        gpio_config(&t_cs);
        gpio_set_level(static_cast<gpio_num_t>(SPI3_CS_MCP_GPIO), 1);
        int cs_mcp_hi = gpio_get_level(static_cast<gpio_num_t>(SPI3_CS_MCP_GPIO));
        gpio_set_level(static_cast<gpio_num_t>(SPI3_CS_MCP_GPIO), 0);
        int cs_mcp_lo = gpio_get_level(static_cast<gpio_num_t>(SPI3_CS_MCP_GPIO));
        gpio_set_level(static_cast<gpio_num_t>(SPI3_CS_MCP_GPIO), 1);
        gpio_reset_pin(static_cast<gpio_num_t>(SPI3_CS_MCP_GPIO));

        gpio_config_t t_miso = {};
        t_miso.pin_bit_mask = (1ULL << SPI3_MISO_GPIO);
        t_miso.mode = GPIO_MODE_INPUT;
        gpio_config(&t_miso);
        int miso36_lvl = gpio_get_level(static_cast<gpio_num_t>(SPI3_MISO_GPIO));
        gpio_reset_pin(static_cast<gpio_num_t>(SPI3_MISO_GPIO));

        ESP_LOGI(TAG, "SPI3 line probe: CS_MCP(GPIO%d) 1->%d 0->%d | MISO(GPIO%d) level=%d",
                 SPI3_CS_MCP_GPIO, cs_mcp_hi, cs_mcp_lo, SPI3_MISO_GPIO, miso36_lvl);
    }

    // SPI3 shared bus (MCP default client) + MCP23S17 Field/UI + ST7789 TFT
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
            dispCfg.mcp_cs_pin = -1; // Shared hardware CS via ESP32 GPIO2
            dispCfg.esp_cs_pin = SPI3_CS_TFT_GPIO;
            dispCfg.mcp_dc_pin = MCP_TFT_DC;
            dispCfg.mcp_res_pin = MCP_TFT_RES;
            dispCfg.mcp_blk_pin = -1; // Moved to ESP32 for native PWM
            dispCfg.esp_blk_pin = TFT_BLK_GPIO;
            dispCfg.clock_hz = SPI3_TFT_CLOCK_HZ;
            // disp_tft_en=0: Class 1 dropout experiment - no TFT traffic on
            // SPI3 (MCP Field I/O and the gripper still run).
            if (Config::AppConfig::instance().data().display_tft_enable == 0) {
                // The TFT driver normally parks its CS; without it GPIO32
                // floats and the panel can sit selected on the shared SPI3
                // bus (MISO contention with the MCP23S17). Deselect it here.
                gpio_config_t tft_cs = {};
                tft_cs.pin_bit_mask = (1ULL << SPI3_CS_TFT_GPIO);
                tft_cs.mode = GPIO_MODE_OUTPUT;
                gpio_config(&tft_cs);
                gpio_set_level(static_cast<gpio_num_t>(SPI3_CS_TFT_GPIO), 1);
                ESP_LOGW(TAG, "TFT UI disabled (cfg disp_tft_en=0) - no display traffic on SPI3; TFT CS held high");
            } else if (!display::UiManager::instance().init(dispCfg)) {
                ESP_LOGW(TAG, "UiManager ST7789 display begin failed");
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
    {
        // cfg eip_scope_gpio (reboot): Class 1 exchange marker for a logic
        // analyzer. Only GPIO17 (TFT backlight), and only with the TFT off.
        const int32_t scope = Config::AppConfig::instance().data().eip_scope_gpio;
        if (scope == TFT_BLK_GPIO &&
            Config::AppConfig::instance().data().display_tft_enable == 0) {
            eip::setScopeMarkerGpio(scope);
        } else if (scope >= 0) {
            ESP_LOGW(TAG, "cfg eip_scope_gpio=%ld ignored (needs %d with disp_tft_en=0)",
                     (long)scope, TFT_BLK_GPIO);
        }
    }
    if (Config::AppConfig::instance().data().eip_spi_dma == 0) {
        w5500Cfg.use_dma = false;
        ESP_LOGW(TAG, "W5500 SPI2 without DMA (cfg eip_spi_dma=0)");
    }
    if (Config::AppConfig::instance().data().eip_txwr_split != 0) {
        w5500::socketSetTxWrSplit(true);
        ESP_LOGW(TAG, "W5500 TX_WR written as two 1-byte writes (cfg eip_txwr_split=1)");
    }
    {
        // cfg eip_spi_khz (reboot): SPI2 clock experiment for the dropouts.
        const int32_t khz = Config::AppConfig::instance().data().eip_w5500_spi_khz;
        if (khz >= 1000 && khz <= 20000 && khz * 1000 != W5500_SCLK_HZ) {
            w5500Cfg.sclk_hz = khz * 1000;
            ESP_LOGW(TAG, "W5500 SPI clock from cfg eip_spi_khz: %ld kHz", (long)khz);
        }
    }

    ESP_LOGI(TAG, "W5500 pins: MOSI=%d MISO=%d SCLK=%d CS=%d RST=MCP_PB7 @ %d Hz",
             w5500Cfg.mosi_gpio, w5500Cfg.miso_gpio, w5500Cfg.sclk_gpio,
             w5500Cfg.cs_gpio, w5500Cfg.sclk_hz);

    bool w5500_ok = w5500.init(w5500Cfg);
    uint8_t post_rst_level = gpio_expander_read(MCP_W5500_RST);
    if (!w5500_ok) {
        ESP_LOGE(TAG, "W5500 init failed (post-init RST level=%d). Continuing boot: UI and console remain active.",
                 post_rst_level);
    } else {
        ESP_LOGI(TAG, "W5500 initialized (version 0x%02X)", w5500.getVersion());
    }

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
    const auto& params = Config::AppConfig::instance().data();
    const double xSpeedRefPerMmS =
        600.0 * static_cast<double>(params.axis_x_motor_reducer_ratio) /
        static_cast<double>(params.axis_x_lead_mm_per_rev);
    auto xAxis = std::make_unique<Gantry::GantryEipLinearAxis>(
        eipImageX, Gantry::EipLinearAxisConfig{
            CONFIG_EIP_AXIS_X_PUU_PER_MM, xSpeedRefPerMmS, xSpeedRefPerMmS,
            xSpeedRefPerMmS, params.axis_x_lead_mm_per_rev});
    ESP_LOGI(TAG, "X axis over EIP (Kinetix 5100, %.1f PUU/mm, speed_ref/mm_s=%.3f), target %s",
             CONFIG_EIP_AXIS_X_PUU_PER_MM, xSpeedRefPerMmS, CONFIG_EIP_TARGET_IP_X);
#else
    auto xAxis = std::unique_ptr<Gantry::GantryLinearAxis>(nullptr);
#endif

#if defined(CONFIG_EIP_AXIS_Z)
    const double zSpeedRefPerMmS =
        600.0 * static_cast<double>(params.axis_z_motor_reducer_ratio) /
        static_cast<double>(params.axis_z_lead_mm_per_rev);
    auto zAxis = std::make_unique<Gantry::GantryEipLinearAxis>(
        eipImageZ, Gantry::EipLinearAxisConfig{
            CONFIG_EIP_AXIS_Z_PUU_PER_MM, zSpeedRefPerMmS, zSpeedRefPerMmS,
            zSpeedRefPerMmS, params.axis_z_lead_mm_per_rev,
            /*invert_direction=*/true});
    ESP_LOGI(TAG,
             "Z axis over EIP (Kinetix 5100, %.1f PUU/mm, invert_dir=1, "
             "speed_ref/mm_s=%.3f), target %s",
             CONFIG_EIP_AXIS_Z_PUU_PER_MM, zSpeedRefPerMmS, CONFIG_EIP_TARGET_IP_Z);
#else
    auto zAxis = std::unique_ptr<Gantry::GantryLinearAxis>(nullptr);
#endif

#if defined(CONFIG_EIP_AXIS_THETA)
    // AXIS_THETA_* accel/decel are placeholders until IndraWorks numbers exist.
    // Defaults go out in drive units: S-0-0259 scales with PUU/deg; S-0-0260 /
    // S-0-0359 are 0.001 rad/s2 and do not.
    const double thetaPuuPerDeg = CONFIG_EIP_AXIS_THETA_PUU_PER_DEG;
    auto thetaAxis = std::make_unique<Gantry::GantryEipRotaryAxis>(
        eipImageTheta, Gantry::eipRotaryAxisConfigFromDeg(
            thetaPuuPerDeg, params.axis_theta_max_speed_deg_per_s,
            params.axis_theta_accel_deg_per_s2, params.axis_theta_decel_deg_per_s2));
    ESP_LOGI(TAG,
             "Theta axis over EIP (HCS01 101/102, %.1f PUU/deg), CIP/FKM %s "
             "(eng HTTP is typically .22)",
             thetaPuuPerDeg, CONFIG_EIP_TARGET_IP_THETA);
#else
    auto thetaAxis = std::unique_ptr<Gantry::GantryRotaryAxis>(nullptr);
#endif

    static Gantry::Gantry gantry(std::move(xAxis), std::move(zAxis),
        std::move(thetaAxis), PIN_GRIPPER);

    // Seed joint-limit envelope with mechanical hard limits.
    gantry.setJointLimits(params.axis_x_hard_limit_min_mm, params.axis_x_hard_limit_max_mm,
                          params.axis_z_hard_limit_min_mm, params.axis_z_hard_limit_max_mm,
                          params.axis_theta_hard_limit_min_deg,
                          params.axis_theta_hard_limit_max_deg);
    gantry.setZAxisLimits(params.axis_z_hard_limit_min_mm, params.axis_z_hard_limit_max_mm);
    gantry.setThetaLimits(params.axis_theta_hard_limit_min_deg,
                          params.axis_theta_hard_limit_max_deg);
    // cfg geom_safe_z_mm (applied at boot): the pick scheduler's hover, staging
    // and bin poses use the same value, so they sit inside the traverse band.
    gantry.setSafeZHeight(params.gantry_safe_z_height_mm);
    gantry.setRuntimeParams(gantryRuntimeParams(params));
    gantryConsoleLoadStoredProfile();

    ESP_LOGI(TAG, "Initializing gantry...");
    if (!gantry.begin()) {
        ESP_LOGE(TAG, "ERROR: Gantry initialization failed!");
        return;
    }
    ESP_LOGI(TAG, "OK Gantry initialized");
#if defined(CONFIG_EIP_ENDSTOP_FROM_DRIVE)
    gantry.configureDriveManagedLimits();
#endif
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

    if (w5500_ok) {
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
    } else {
        ESP_LOGW(TAG, "W5500 is offline: skipping EIP scanner task start");
    }
#endif

    // ------------------------------------------------------------------
    // FreeRTOS tasks first — do not block console / motion behind network wait.
    // ------------------------------------------------------------------
    static Network::EthernetLink ethernetLink;
    static CellNet::EspEthL2Transport l2Transport;
    static CellNet::CellNetL2Node l2Node(l2Transport, CellNodeId::GANTRY);

    BaseType_t result;
    static PickSchedulerTaskConfig pickCfg = { &gantry, &l2Node };
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

    static UiTaskConfig uiCfg = { &gantry, &ethernetLink };
    if (Config::AppConfig::instance().data().display_tft_enable != 0) {
        result = xTaskCreatePinnedToCore(
            tftUiTask, "TftUiTask",
            TFT_UI_TASK_STACK, &uiCfg,
            TFT_UI_TASK_PRIORITY, nullptr, TFT_UI_TASK_CORE);
        if (result != pdPASS) {
            ESP_LOGW(TAG, "Failed to create TftUiTask");
        }
    }

    static GantryTestConsoleConfig consoleCfg = {};
    consoleCfg.gantry                 = &gantry;
    consoleCfg.limit_min_pin          = -1;   // Endstops drive-managed (EIP)
    consoleCfg.limit_max_pin          = -1;
    consoleCfg.use_mcp23s17           = (gpio_expander_get_mcp_handle() != nullptr);
#if CONFIG_EIP_SCANNER_ENABLED
    consoleCfg.w5500_hal              = w5500_ok ? &w5500Hal : nullptr;
#else
    consoleCfg.w5500_hal              = nullptr;
#endif

#if CONSOLE_UART_ENABLE
    result = xTaskCreatePinnedToCore(
        gantryTestConsoleTask, "SerialCmd",
        CONSOLE_TASK_STACK, &consoleCfg,
        CONSOLE_TASK_PRIORITY, nullptr, CONSOLE_TASK_CORE);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create Serial task!");
    }
#else
    ESP_LOGI(TAG, "UART console disabled — GPIO1/3 free; use TCP :%d",
             CONSOLE_TCP_PORT);
#endif

    // ------------------------------------------------------------------
    // LAN8720 first so TCP console and Layer-2 link come online.
    // ------------------------------------------------------------------
    bool ethUp = false;
    if (ethernetLink.start() && ethernetLink.waitForUp(ETH_IP_WAIT_TIMEOUT_MS)) {
        ethUp = true;
        gantryNetConsoleStart(&consoleCfg);
        ESP_LOGI(TAG, "Net console listening on TCP %d (plant / LAN8720)",
                 CONSOLE_TCP_PORT);
        gantryOtaStartServer(&gantry, 8032, CONSOLE_TCP_PASSWORD);
        ESP_LOGI(TAG, "OTA server listening on TCP 8032 (plant / LAN8720)");

        // Initialize High-Speed OSI Layer-2 Cell Network transceiver
        if (l2Transport.attachEthHandle(ethernetLink.getEthHandle(), ethernetLink.getNetif()) == ESP_OK &&
            l2Node.begin()) {
            ESP_LOGI(TAG, "CellNet OSI Layer-2 bus ACTIVE (EtherType 0x%04X, Node 0x%02X)",
                     CELL_NET_L2_ETHERTYPE, static_cast<uint8_t>(CellNodeId::GANTRY));
        } else {
            ESP_LOGW(TAG, "CellNet Layer-2 initialization failed");
        }
    } else {
#if CONSOLE_UART_ENABLE
        ESP_LOGW(TAG, "LAN8720 not up — UART console only; net console skipped");
#else
        ESP_LOGW(TAG, "LAN8720 not up — net console skipped (no UART fallback)");
#endif
    }

    if (ethUp) {
        esp_sntp_config_t sntp = ESP_NETIF_SNTP_DEFAULT_CONFIG(
            Config::AppConfig::instance().data().ntp_server);
        if (esp_netif_sntp_init(&sntp) != ESP_OK) {
            ESP_LOGW(TAG, "NTP client did not start; stored sets will show no clock");
        }
    }

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

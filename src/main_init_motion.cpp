/**
 * @file main_init_motion.cpp
 * @brief Boot stage for motion: the W5500, the EIP process images, the X, Z
 *        and theta axes, the Gantry and its limits, then the EIP scanner task.
 */

#include "main_internal.h"

#include "AppConfig.h"
#include "Gantry.h"
#include "esp_log.h"
#include "gantry_app_constants.h"
#include "gantry_runtime_params.h"
#include "gantry_test_console.h"
#include "gpio_expander.h"
#include "sdkconfig.h"

#include <memory>

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

BootMotion bootInitMotion() {
    BootMotion out;
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
        return out;
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

    out.gantry = &gantry;
#if CONFIG_EIP_SCANNER_ENABLED
    out.w5500_hal = w5500_ok ? &w5500Hal : nullptr;
#endif
    return out;
}

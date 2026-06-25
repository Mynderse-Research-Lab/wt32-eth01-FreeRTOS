/**
 * @file main.cpp
 * @brief Gantry control application for WT32-ETH01 with MCP23S17 IO expansion.
 *
 * Coordinate convention (firmware-wide, as of 2026-05):
 *   X = horizontal traverse (across belt), Y = along-belt (no gantry actuator;
 *   conveyor downstream = -Y), Z = vertical (+Z = up). Joint Z=0 is homing datum;
 *   physical bed offset: GANTRY_Z_DATUM_OFFSET_ABOVE_BED_MM (axis_drivetrain_params.h).
 *
 * FreeRTOS application with:
 * - MCP23S17 SPI GPIO expander initialization
 * - X-axis pulse-train servo control (Allen-Bradley Kinetix 5100 + SCHUNK Beta 100-ZRS belt)
 * - Z-axis pulse-train servo control (Allen-Bradley Kinetix 5100 + SCHUNK Beta 80-SRS ballscrew)
 * - Theta-axis pulse-train rotary control (custom driver + SCHUNK ERD 04-40-D-H-N)
 * - End-effector: SCHUNK KGG 100-80 pneumatic gripper
 * - Interactive serial console (gantry_test_console)
 * - Periodic gantry update task at 100 Hz
 *
 * Pin assignments live in gantry_app_constants.h.
 * Motor/driver electrical tuning lives in axis_pulse_motor_params.h.
 * Mechanical / drivetrain / envelope tuning lives in axis_drivetrain_params.h.
 */

// Ask axis_drivetrain_params.h to emit its deployment-time reminders in this
// TU only. This keeps the geometry-freeze warning (and any future
// single-point-of-truth reminders) to ONE copy per build instead of one per
// translation unit that transitively pulls the header in. MUST be defined
// before any include that may transitively pull axis_drivetrain_params.h
// (e.g. "Gantry.h" -> "GantryConfig.h" -> "axis_drivetrain_params.h").
#define AXIS_DRIVETRAIN_PARAMS_EMIT_WARNINGS

#include "Gantry.h"
#include "PulseMotor.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "gpio_expander.h"
#include "MCP23S17.h"
#include "gantry_test_console.h"
#include "gantry_app_constants.h"
#include "axis_pulse_motor_params.h"
#include "axis_drivetrain_params.h"
#include "mqtt_topics.h"
#include "MqttBridge.h"
#include "pick_scheduler.h"

#if CONFIG_EIP_SCANNER_ENABLED
#include "EipScannerTask.h"
#endif

static const char* TAG = "GantryApp";

// Pulse pins must run on direct ESP32 GPIOs (LEDC). Encode them so the
// gpio_expander-aware pin path treats them as direct GPIOs.
static const int PIN_X_PULSE_EXP     = GPIO_EXPANDER_DIRECT_PIN(PIN_X_PULSE);
static const int PIN_Z_PULSE_EXP     = GPIO_EXPANDER_DIRECT_PIN(PIN_Z_PULSE);
static const int PIN_THETA_PULSE_EXP = GPIO_EXPANDER_DIRECT_PIN(PIN_THETA_PULSE);

static void initDirectOutputs(void) {
    // PIN_Z_PULSE (GPIO2) is a strapping pin; pre-seed it LOW on the raw ESP32
    // GPIO before LEDC binds the channel, so the line is in a known state
    // through reset.
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << PIN_Z_PULSE);
    io_conf.mode         = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type    = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)PIN_Z_PULSE, 0));
}

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
// MCP23S17 initialization helper
// ---------------------------------------------------------------------------
static bool initMcp23s17(void) {
    ESP_LOGI(TAG, "Initializing MCP23S17 GPIO expander...");
    mcp23s17_config_t mcp_config = {};
    mcp_config.spi_host       = SPI2_HOST;
    mcp_config.cs_pin         = (gpio_num_t)MCP23S17_SPI_CS_PIN;
    mcp_config.miso_pin       = (gpio_num_t)MCP23S17_SPI_MISO_PIN;
    mcp_config.mosi_pin       = (gpio_num_t)MCP23S17_SPI_MOSI_PIN;
    mcp_config.sclk_pin       = (gpio_num_t)MCP23S17_SPI_SCLK_PIN;
    mcp_config.device_address = MCP23S17_DEVICE_ADDRESS;
    mcp_config.clock_speed_hz = MCP23S17_SPI_CLOCK_HZ_WORKING;
    ESP_LOGI(TAG, "MCP SPI config: CS=%d MISO=%d MOSI=%d SCLK=%d CLK=%lu",
             static_cast<int>(mcp_config.cs_pin),
             static_cast<int>(mcp_config.miso_pin),
             static_cast<int>(mcp_config.mosi_pin),
             static_cast<int>(mcp_config.sclk_pin),
             static_cast<unsigned long>(mcp_config.clock_speed_hz));

    if (!gpio_expander_init(&mcp_config)) {
        ESP_LOGE(TAG, "FATAL: Failed to initialize MCP23S17; cannot continue.");
        return false;
    }
    ESP_LOGI(TAG, "MCP23S17 initialized successfully");

    // Defensive per-pin seeding (DIR/EN/ALM/ARST/GRIPPER low, limit inputs
    // with pull-up) is now owned by Gantry::Gantry::preparePinsForBoot(), so
    // the application layer no longer reaches past the Gantry abstraction to
    // poke individual MCP pins. See lib/Gantry/docs/ARCHITECTURE_FLOW.md
    // invariant 6.
    return true;
}

// ---------------------------------------------------------------------------
// Build PulseMotor DriverConfig / DrivetrainConfig per axis
// ---------------------------------------------------------------------------

static PulseMotor::DriverConfig makeXDriverConfig() {
    PulseMotor::DriverConfig cfg;
    cfg.pulse_pin        = PIN_X_PULSE_EXP;
    cfg.dir_pin          = PIN_X_DIR;
    cfg.enable_pin       = PIN_X_ENABLE;
    cfg.alarm_reset_pin  = PIN_X_ALARM_RESET;
    cfg.alarm_pin        = PIN_X_ALARM_STATUS;
    cfg.encoder_a_pin    = PIN_X_ENC_A;
    cfg.encoder_b_pin    = PIN_X_ENC_B;

    // Limits are enforced at the Gantry layer; driver sees no limit inputs.
    cfg.limit_min_pin    = -1;
    cfg.limit_max_pin    = -1;

    cfg.pulse_mode       = PulseMotor::PulseMode::PULSE_DIRECTION;
    cfg.encoder_ppr      = AXIS_X_ENCODER_PPR;
    cfg.max_pulse_freq   = AXIS_X_MAX_PULSE_FREQ_HZ;
    cfg.gear_numerator   = AXIS_X_GEAR_NUMERATOR;
    cfg.gear_denominator = AXIS_X_GEAR_DENOMINATOR;
    cfg.invert_dir_pin   = AXIS_X_INVERT_DIR != 0;
    cfg.invert_output_logic = AXIS_X_INVERT_OUTPUT_LOGIC != 0;
    cfg.ledc_channel     = X_PULSE_LEDC_CHANNEL;
    cfg.ledc_resolution  = AXIS_X_LEDC_RESOLUTION_BITS;
    cfg.pcnt_unit        = X_ENCODER_PCNT_UNIT;
    cfg.enable_encoder_feedback = (AXIS_X_ENCODER_FEEDBACK_ENABLED != 0);
    cfg.homing_speed_pps = AXIS_X_HOMING_SPEED_PPS;
    cfg.limit_debounce_cycles    = AXIS_X_LIMIT_DEBOUNCE_CYCLES;
    cfg.limit_sample_interval_ms = AXIS_X_LIMIT_SAMPLE_INTERVAL_MS;
    return cfg;
}

static PulseMotor::DrivetrainConfig makeXDrivetrainConfig() {
    PulseMotor::DrivetrainConfig dt;
    dt.type                   = (PulseMotor::DrivetrainType)AXIS_X_DRIVETRAIN;
    dt.belt_lead_mm_per_rev   = AXIS_X_LEAD_MM_PER_REV;
    dt.encoder_ppr            = AXIS_X_ENCODER_PPR;
    dt.motor_reducer_ratio    = AXIS_X_MOTOR_REDUCER_RATIO;
    return dt;
}

static PulseMotor::DriverConfig makeZDriverConfig() {
    PulseMotor::DriverConfig cfg;
    cfg.pulse_pin        = PIN_Z_PULSE_EXP;
    cfg.dir_pin          = PIN_Z_DIR;
    cfg.enable_pin       = PIN_Z_ENABLE;
    cfg.alarm_reset_pin  = PIN_Z_ALARM_RESET;
    cfg.alarm_pin        = PIN_Z_ALARM_STATUS;
    cfg.encoder_a_pin    = PIN_Z_ENC_A;
    cfg.encoder_b_pin    = PIN_Z_ENC_B;

    cfg.limit_min_pin    = -1;
    cfg.limit_max_pin    = -1;

    cfg.pulse_mode       = PulseMotor::PulseMode::PULSE_DIRECTION;
    cfg.encoder_ppr      = AXIS_Z_ENCODER_PPR;
    cfg.max_pulse_freq   = AXIS_Z_MAX_PULSE_FREQ_HZ;
    cfg.gear_numerator   = AXIS_Z_GEAR_NUMERATOR;
    cfg.gear_denominator = AXIS_Z_GEAR_DENOMINATOR;
    cfg.invert_dir_pin   = AXIS_Z_INVERT_DIR != 0;
    cfg.invert_output_logic = AXIS_Z_INVERT_OUTPUT_LOGIC != 0;
    cfg.ledc_channel     = Z_PULSE_LEDC_CHANNEL;
    cfg.ledc_resolution  = AXIS_Z_LEDC_RESOLUTION_BITS;
    cfg.pcnt_unit        = Z_ENCODER_PCNT_UNIT;
    cfg.enable_encoder_feedback = (AXIS_Z_ENCODER_FEEDBACK_ENABLED != 0);
    cfg.homing_speed_pps = AXIS_Z_HOMING_SPEED_PPS;
    cfg.limit_debounce_cycles    = AXIS_Z_LIMIT_DEBOUNCE_CYCLES;
    cfg.limit_sample_interval_ms = AXIS_Z_LIMIT_SAMPLE_INTERVAL_MS;
    return cfg;
}

static PulseMotor::DrivetrainConfig makeZDrivetrainConfig() {
    PulseMotor::DrivetrainConfig dt;
    dt.type                    = (PulseMotor::DrivetrainType)AXIS_Z_DRIVETRAIN;
    dt.ballscrew_lead_mm       = AXIS_Z_LEAD_MM_PER_REV;
    dt.ballscrew_critical_rpm  = AXIS_Z_CRITICAL_RPM;
    dt.encoder_ppr             = AXIS_Z_ENCODER_PPR;
    dt.motor_reducer_ratio     = AXIS_Z_MOTOR_REDUCER_RATIO;
    return dt;
}

static PulseMotor::DriverConfig makeThetaDriverConfig() {
    PulseMotor::DriverConfig cfg;
    cfg.pulse_pin        = PIN_THETA_PULSE_EXP;
    cfg.dir_pin          = PIN_THETA_DIR;
    cfg.enable_pin       = PIN_THETA_ENABLE;
    cfg.alarm_reset_pin  = -1;             // not wired in this revision
    cfg.alarm_pin        = -1;             // not wired in this revision
    cfg.encoder_a_pin    = PIN_THETA_ENC_A;
    cfg.encoder_b_pin    = PIN_THETA_ENC_B;

    cfg.limit_min_pin    = -1;
    cfg.limit_max_pin    = -1;

    cfg.pulse_mode       = PulseMotor::PulseMode::PULSE_DIRECTION;
    cfg.encoder_ppr      = AXIS_THETA_ENCODER_PPR;
    cfg.max_pulse_freq   = AXIS_THETA_MAX_PULSE_FREQ_HZ;
    cfg.gear_numerator   = AXIS_THETA_GEAR_NUMERATOR;
    cfg.gear_denominator = AXIS_THETA_GEAR_DENOMINATOR;
    cfg.invert_dir_pin   = AXIS_THETA_INVERT_DIR != 0;
    cfg.invert_output_logic = AXIS_THETA_INVERT_OUTPUT_LOGIC != 0;
    cfg.ledc_channel     = THETA_PULSE_LEDC_CHANNEL;
    cfg.ledc_resolution  = AXIS_THETA_LEDC_RESOLUTION_BITS;
    cfg.pcnt_unit        = THETA_ENCODER_PCNT_UNIT;
    // Theta encoder not wired yet; enable_encoder_feedback false until the
    // custom driver's pulse/dir-feedback lines are routed.
    cfg.enable_encoder_feedback = (PIN_THETA_ENC_A >= 0 && PIN_THETA_ENC_B >= 0);
    cfg.homing_speed_pps = AXIS_THETA_HOMING_SPEED_PPS;
    cfg.limit_debounce_cycles    = AXIS_THETA_LIMIT_DEBOUNCE_CYCLES;
    cfg.limit_sample_interval_ms = AXIS_THETA_LIMIT_SAMPLE_INTERVAL_MS;
    return cfg;
}

static PulseMotor::DrivetrainConfig makeThetaDrivetrainConfig() {
    PulseMotor::DrivetrainConfig dt;
    dt.type                = (PulseMotor::DrivetrainType)AXIS_THETA_DRIVETRAIN;
    dt.output_gear_ratio   = AXIS_THETA_OUTPUT_GEAR_RATIO;
    dt.encoder_ppr         = AXIS_THETA_ENCODER_PPR;
    dt.motor_reducer_ratio = AXIS_THETA_MOTOR_REDUCER_RATIO;
    return dt;
}

// ---------------------------------------------------------------------------
// app_main
// ---------------------------------------------------------------------------
extern "C" void app_main(void) {
    ESP_LOGI(TAG, "\n========================================");
    ESP_LOGI(TAG, "WT32-ETH01 Gantry Controller (PulseMotor)");
    ESP_LOGI(TAG, "========================================\n");

    if (!initMcp23s17()) {
        return;
    }

    // ------------------------------------------------------------------
    // Build per-axis configs
    // ------------------------------------------------------------------
    PulseMotor::DriverConfig     xDrv = makeXDriverConfig();
    PulseMotor::DrivetrainConfig xDt  = makeXDrivetrainConfig();
    PulseMotor::DriverConfig     zDrv = makeZDriverConfig();
    PulseMotor::DrivetrainConfig zDt  = makeZDrivetrainConfig();
    PulseMotor::DriverConfig     tDrv = makeThetaDriverConfig();
    PulseMotor::DrivetrainConfig tDt  = makeThetaDrivetrainConfig();

    // ------------------------------------------------------------------
    // Boot-time pin seeding (defensive; idempotent).
    //
    //   initDirectOutputs()        - pre-seeds PIN_Z_PULSE (GPIO2, strap)
    //                                low on direct ESP GPIO before LEDC
    //                                takes over. Stays in main because it
    //                                is chip-peripheral setup, not Gantry
    //                                business.
    //   Gantry::preparePinsForBoot - MCP-routed DIR/EN/ALM_RESET/GRIPPER
    //                                seeding; replaces the old per-pin
    //                                gpio_expander_* block. See
    //                                ARCHITECTURE_FLOW.md invariant 6.
    // ------------------------------------------------------------------
    initDirectOutputs();
    Gantry::Gantry::preparePinsForBoot(xDrv, zDrv, tDrv, PIN_GRIPPER);

    // ------------------------------------------------------------------
    // Create Gantry instance
    // ------------------------------------------------------------------
    static Gantry::Gantry gantry(xDrv, xDt, zDrv, zDt, tDrv, tDt, PIN_GRIPPER);

    // X-axis limit switches (via MCP23S17)
    gantry.setLimitPins(PIN_X_LIMIT_MIN, PIN_X_LIMIT_MAX);

    // Seed the joint-limit envelope with the MECHANICAL hard limits from
    // axis_drivetrain_params.h. Soft limits derived from the homing /
    // calibration sweep will override these on boot-reset via
    // Gantry::calibrate() and the homing task.
    gantry.setJointLimits(AXIS_X_HARD_LIMIT_MIN_MM,     AXIS_X_HARD_LIMIT_MAX_MM,
                          AXIS_Z_HARD_LIMIT_MIN_MM,     AXIS_Z_HARD_LIMIT_MAX_MM,
                          AXIS_THETA_HARD_LIMIT_MIN_DEG, AXIS_THETA_HARD_LIMIT_MAX_DEG);
    gantry.setZAxisLimits(AXIS_Z_HARD_LIMIT_MIN_MM, AXIS_Z_HARD_LIMIT_MAX_MM);
    gantry.setThetaLimits(AXIS_THETA_HARD_LIMIT_MIN_DEG, AXIS_THETA_HARD_LIMIT_MAX_DEG);
    gantry.setSafeZHeight(GANTRY_SAFE_Z_HEIGHT_MM);

    // ------------------------------------------------------------------
    // Initialize and enable
    // ------------------------------------------------------------------
    ESP_LOGI(TAG, "Initializing gantry...");
    if (!gantry.begin()) {
        ESP_LOGE(TAG, "ERROR: Gantry initialization failed!");
        ESP_LOGE(TAG, "Check pin connections and try again.");
        return;
    }
    ESP_LOGI(TAG, "OK Gantry initialized");

    gantry.enable();
    ESP_LOGI(TAG, "OK Motors enabled");

    // ------------------------------------------------------------------
    // FreeRTOS tasks
    // ------------------------------------------------------------------
    BaseType_t result;
    static MqttBridge::EthernetLink ethernetLink;
    static MqttBridge::Bridge mqttBridge(&ethernetLink);
    if (!mqttBridge.start(MQTT_GANTRY_ID_DEFAULT)) {
        ESP_LOGE(TAG, "FATAL: MQTT bridge failed to start; halting application startup.");
        gantry.disable();
        return;
    }
    (void)mqttBridge.publishStatusJson("{\"state\":\"LINK_INIT\",\"source\":\"main\"}");

#if CONFIG_EIP_SCANNER_ENABLED
    eip::startScannerTask();
#endif

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
    consoleCfg.limit_min_pin          = PIN_X_LIMIT_MIN;
    consoleCfg.limit_max_pin          = PIN_X_LIMIT_MAX;
    consoleCfg.use_mcp23s17           = true;
    consoleCfg.limit_switches_active  = true;
    consoleCfg.x_pulse_pin            = PIN_X_PULSE;
    consoleCfg.x_dir_pin              = PIN_X_DIR;
    consoleCfg.x_enable_pin           = PIN_X_ENABLE;
    consoleCfg.x_alarm_pin            = PIN_X_ALARM_STATUS;
    consoleCfg.x_alarm_reset_pin      = PIN_X_ALARM_RESET;
    consoleCfg.z_alarm_pin            = PIN_Z_ALARM_STATUS;
    consoleCfg.z_alarm_reset_pin      = PIN_Z_ALARM_RESET;
    consoleCfg.x_encoder_a_pin        = PIN_X_ENC_A;
    consoleCfg.x_encoder_b_pin        = PIN_X_ENC_B;
    consoleCfg.z_pulse_pin            = PIN_Z_PULSE;
    consoleCfg.z_encoder_a_pin        = PIN_Z_ENC_A;
    consoleCfg.z_encoder_b_pin        = PIN_Z_ENC_B;
    consoleCfg.x_pulse_ledc_channel    = X_PULSE_LEDC_CHANNEL;
    consoleCfg.z_pulse_ledc_channel    = Z_PULSE_LEDC_CHANNEL;
    consoleCfg.theta_pulse_ledc_channel = THETA_PULSE_LEDC_CHANNEL;
    consoleCfg.theta_pulse_pin         = PIN_THETA_PULSE;

    result = xTaskCreatePinnedToCore(
        gantryTestConsoleTask, "SerialCmd",
        CONSOLE_TASK_STACK, &consoleCfg,
        CONSOLE_TASK_PRIORITY, nullptr, CONSOLE_TASK_CORE);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create Serial task!");
    }

    ESP_LOGI(TAG, "All tasks created successfully");
    ESP_LOGI(TAG, "System ready - type 'help' for commands");
    gantryTestPrintHelp();

    vTaskDelete(nullptr);
}

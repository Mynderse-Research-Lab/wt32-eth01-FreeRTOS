/**
 * @file main.cpp
 * @brief Gantry control application for WT32-ETH01 with MCP23S17 IO expansion.
 *
 * FreeRTOS application with:
 * - MCP23S17 SPI GPIO expander initialization
 * - X-axis pulse-train servo control (Allen-Bradley Kinetix 5100 + SCHUNK Beta 100-ZRS belt)
 * - Y-axis pulse-train servo control (Allen-Bradley Kinetix 5100 + SCHUNK Beta 80-SRS ballscrew)
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
#include "driver/gpio.h"
#include "gpio_expander.h"
#include "MCP23S17.h"
#include "gantry_test_console.h"
#include "gantry_app_constants.h"
#include "axis_pulse_motor_params.h"
#include "axis_drivetrain_params.h"

static const char* TAG = "GantryApp";

// Pulse pins must run on direct ESP32 GPIOs (LEDC). Encode them so the
// gpio_expander-aware pin path treats them as direct GPIOs.
static const int PIN_X_PULSE_EXP     = GPIO_EXPANDER_DIRECT_PIN(PIN_X_PULSE);
static const int PIN_Y_PULSE_EXP     = GPIO_EXPANDER_DIRECT_PIN(PIN_Y_PULSE);
static const int PIN_THETA_PULSE_EXP = GPIO_EXPANDER_DIRECT_PIN(PIN_THETA_PULSE);

static void initDirectOutputs(void) {
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << PIN_Y_PULSE);
    io_conf.mode         = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type    = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)PIN_Y_PULSE, 0));
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

    // Outputs
    gpio_expander_set_direction(PIN_X_DIR,          true);
    gpio_expander_set_direction(PIN_X_ENABLE,       true);
    initDirectOutputs();
    gpio_expander_set_direction(PIN_Y_DIR,          true);
    gpio_expander_set_direction(PIN_Y_ENABLE,       true);
    gpio_expander_set_direction(PIN_GRIPPER,        true);
    gpio_expander_set_direction(PIN_X_ALARM_RESET,  true);
    gpio_expander_set_direction(PIN_Y_ALARM_RESET,  true);
    gpio_expander_set_direction(PIN_THETA_DIR,      true);
    gpio_expander_set_direction(PIN_THETA_ENABLE,   true);

    // Inputs (with pull-ups)
    gpio_expander_set_direction(PIN_X_LIMIT_MIN,    false);
    gpio_expander_set_pullup(PIN_X_LIMIT_MIN,       true);
    gpio_expander_set_direction(PIN_X_LIMIT_MAX,    false);
    gpio_expander_set_pullup(PIN_X_LIMIT_MAX,       true);
    gpio_expander_set_direction(PIN_Y_LIMIT_MIN,    false);
    gpio_expander_set_pullup(PIN_Y_LIMIT_MIN,       true);
    gpio_expander_set_direction(PIN_Y_LIMIT_MAX,    false);
    gpio_expander_set_pullup(PIN_Y_LIMIT_MAX,       true);
    gpio_expander_set_direction(PIN_X_ALARM_STATUS, false);
    gpio_expander_set_pullup(PIN_X_ALARM_STATUS,    true);
    gpio_expander_set_direction(PIN_Y_ALARM_STATUS, false);
    gpio_expander_set_pullup(PIN_Y_ALARM_STATUS,    true);

    // Safe initial output states
    gpio_expander_write(PIN_X_DIR,         0);
    gpio_expander_write(PIN_X_ENABLE,      0);
    gpio_expander_write(PIN_Y_DIR,         0);
    gpio_expander_write(PIN_Y_ENABLE,      0);
    gpio_expander_write(PIN_GRIPPER,       0);
    gpio_expander_write(PIN_X_ALARM_RESET, 0);
    gpio_expander_write(PIN_Y_ALARM_RESET, 0);
    gpio_expander_write(PIN_THETA_DIR,     0);
    gpio_expander_write(PIN_THETA_ENABLE,  0);

    ESP_LOGI(TAG, "MCP23S17 pins configured");
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
    cfg.enable_encoder_feedback = true;
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

static PulseMotor::DriverConfig makeYDriverConfig() {
    PulseMotor::DriverConfig cfg;
    cfg.pulse_pin        = PIN_Y_PULSE_EXP;
    cfg.dir_pin          = PIN_Y_DIR;
    cfg.enable_pin       = PIN_Y_ENABLE;
    cfg.alarm_reset_pin  = PIN_Y_ALARM_RESET;
    cfg.alarm_pin        = PIN_Y_ALARM_STATUS;
    cfg.encoder_a_pin    = PIN_Y_ENC_A;
    cfg.encoder_b_pin    = PIN_Y_ENC_B;

    cfg.limit_min_pin    = -1;
    cfg.limit_max_pin    = -1;

    cfg.pulse_mode       = PulseMotor::PulseMode::PULSE_DIRECTION;
    cfg.encoder_ppr      = AXIS_Y_ENCODER_PPR;
    cfg.max_pulse_freq   = AXIS_Y_MAX_PULSE_FREQ_HZ;
    cfg.gear_numerator   = AXIS_Y_GEAR_NUMERATOR;
    cfg.gear_denominator = AXIS_Y_GEAR_DENOMINATOR;
    cfg.invert_dir_pin   = AXIS_Y_INVERT_DIR != 0;
    cfg.invert_output_logic = AXIS_Y_INVERT_OUTPUT_LOGIC != 0;
    cfg.ledc_channel     = Y_PULSE_LEDC_CHANNEL;
    cfg.ledc_resolution  = AXIS_Y_LEDC_RESOLUTION_BITS;
    cfg.pcnt_unit        = Y_ENCODER_PCNT_UNIT;
    cfg.enable_encoder_feedback = true;
    cfg.homing_speed_pps = AXIS_Y_HOMING_SPEED_PPS;
    cfg.limit_debounce_cycles    = AXIS_Y_LIMIT_DEBOUNCE_CYCLES;
    cfg.limit_sample_interval_ms = AXIS_Y_LIMIT_SAMPLE_INTERVAL_MS;
    return cfg;
}

static PulseMotor::DrivetrainConfig makeYDrivetrainConfig() {
    PulseMotor::DrivetrainConfig dt;
    dt.type                    = (PulseMotor::DrivetrainType)AXIS_Y_DRIVETRAIN;
    dt.ballscrew_lead_mm       = AXIS_Y_LEAD_MM_PER_REV;
    dt.ballscrew_critical_rpm  = AXIS_Y_CRITICAL_RPM;
    dt.encoder_ppr             = AXIS_Y_ENCODER_PPR;
    dt.motor_reducer_ratio     = AXIS_Y_MOTOR_REDUCER_RATIO;
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
    PulseMotor::DriverConfig     yDrv = makeYDriverConfig();
    PulseMotor::DrivetrainConfig yDt  = makeYDrivetrainConfig();
    PulseMotor::DriverConfig     tDrv = makeThetaDriverConfig();
    PulseMotor::DrivetrainConfig tDt  = makeThetaDrivetrainConfig();

    // ------------------------------------------------------------------
    // Create Gantry instance
    // ------------------------------------------------------------------
    static Gantry::Gantry gantry(xDrv, xDt, yDrv, yDt, tDrv, tDt, PIN_GRIPPER);

    // X-axis limit switches (via MCP23S17)
    gantry.setLimitPins(PIN_X_LIMIT_MIN, PIN_X_LIMIT_MAX);

    // Seed the joint-limit envelope with the MECHANICAL hard limits from
    // axis_drivetrain_params.h. Soft limits derived from the homing /
    // calibration sweep will override these on boot-reset via
    // Gantry::calibrate() and the homing task.
    gantry.setJointLimits(AXIS_X_HARD_LIMIT_MIN_MM,     AXIS_X_HARD_LIMIT_MAX_MM,
                          AXIS_Y_HARD_LIMIT_MIN_MM,     AXIS_Y_HARD_LIMIT_MAX_MM,
                          AXIS_THETA_HARD_LIMIT_MIN_DEG, AXIS_THETA_HARD_LIMIT_MAX_DEG);
    gantry.setYAxisLimits(AXIS_Y_HARD_LIMIT_MIN_MM, AXIS_Y_HARD_LIMIT_MAX_MM);
    gantry.setThetaLimits(AXIS_THETA_HARD_LIMIT_MIN_DEG, AXIS_THETA_HARD_LIMIT_MAX_DEG);
    gantry.setSafeYHeight(GANTRY_SAFE_Y_HEIGHT_MM);

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
    static UpdateTaskConfig updateCfg = { &gantry };
    BaseType_t result = xTaskCreatePinnedToCore(
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
    consoleCfg.y_alarm_pin            = PIN_Y_ALARM_STATUS;
    consoleCfg.y_alarm_reset_pin      = PIN_Y_ALARM_RESET;
    consoleCfg.x_encoder_a_pin        = PIN_X_ENC_A;
    consoleCfg.x_encoder_b_pin        = PIN_X_ENC_B;
    consoleCfg.y_pulse_pin            = PIN_Y_PULSE;
    consoleCfg.y_encoder_a_pin        = PIN_Y_ENC_A;
    consoleCfg.y_encoder_b_pin        = PIN_Y_ENC_B;
    consoleCfg.x_pulse_ledc_channel   = X_PULSE_LEDC_CHANNEL;
    consoleCfg.y_pulse_ledc_channel   = Y_PULSE_LEDC_CHANNEL;
    consoleCfg.theta_pwm_pin          = PIN_THETA_PULSE;  // legacy field name; now a pulse pin

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

/**
 * @file main.cpp
 * @brief Gantry control application for WT32-ETH01 with MCP23S17 IO expansion
 *
 * FreeRTOS application with:
 * - MCP23S17 SPI GPIO expander initialization
 * - Pulse-train axis control via PulseMotor driver (direct pulse GPIO, control via MCP)
 * - Three-axis pulse-train configuration (X, Y, Theta)
 * - End-effector (gripper) on MCP23S17
 * - Interactive serial console (gantry_test_console)
 * - Periodic gantry update task at 100 Hz
 *
 * Pin assignments are defined in gantry_app_constants.h and match the
 * MCP23S17_INTEGRATION.md pin table.
 */

#include "Gantry.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "gpio_expander.h"
#include "MCP23S17.h"
#include "gantry_test_console.h"
#include "gantry_app_constants.h"

static const char *TAG = "GantryApp";
static const int PIN_X_PULSE_EXP = GPIO_EXPANDER_DIRECT_PIN(PIN_X_PULSE);
static const int PIN_Y_PULSE_EXP = GPIO_EXPANDER_DIRECT_PIN(PIN_Y_PULSE);

static void initDirectOutputs(void) {
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << PIN_Y_PULSE) | (1ULL << PIN_THETA_PULSE);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)PIN_Y_PULSE, 0));
}

// ---------------------------------------------------------------------------
// Task parameter structures
// ---------------------------------------------------------------------------
struct UpdateTaskConfig {
    Gantry::Gantry *gantry;
};

// ---------------------------------------------------------------------------
// Gantry periodic update task (100 Hz on Core 1)
// ---------------------------------------------------------------------------
void gantryUpdateTask(void *param) {
    auto *cfg = static_cast<UpdateTaskConfig *>(param);
    if (cfg == nullptr || cfg->gantry == nullptr) {
        ESP_LOGE(TAG, "Invalid Gantry update task config");
        vTaskDelete(nullptr);
        return;
    }

    const TickType_t updateInterval = pdMS_TO_TICKS(10);  // 100 Hz
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
    mcp_config.spi_host = SPI2_HOST;
    mcp_config.cs_pin = (gpio_num_t)MCP23S17_SPI_CS_PIN;
    mcp_config.miso_pin = (gpio_num_t)MCP23S17_SPI_MISO_PIN;
    mcp_config.mosi_pin = (gpio_num_t)MCP23S17_SPI_MOSI_PIN;
    mcp_config.sclk_pin = (gpio_num_t)MCP23S17_SPI_SCLK_PIN;
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

    // ---- Output pins ----
    gpio_expander_set_direction(PIN_X_DIR, true);
    gpio_expander_set_direction(PIN_X_ENABLE, true);
    initDirectOutputs();
    gpio_expander_set_direction(PIN_Y_DIR, true);
    gpio_expander_set_direction(PIN_Y_ENABLE, true);
    gpio_expander_set_direction(PIN_THETA_DIR, true);
    gpio_expander_set_direction(PIN_THETA_ENABLE, true);
    gpio_expander_set_direction(PIN_GRIPPER, true);
    gpio_expander_set_direction(PIN_X_ALARM_RESET, true);
    gpio_expander_set_direction(PIN_Y_ALARM_RESET, true);

    // ---- Input pins with pull-ups ----
    gpio_expander_set_direction(PIN_X_LIMIT_MIN, false);
    gpio_expander_set_pullup(PIN_X_LIMIT_MIN, true);
    gpio_expander_set_direction(PIN_X_LIMIT_MAX, false);
    gpio_expander_set_pullup(PIN_X_LIMIT_MAX, true);
    gpio_expander_set_direction(PIN_Y_LIMIT_MIN, false);
    gpio_expander_set_pullup(PIN_Y_LIMIT_MIN, true);
    gpio_expander_set_direction(PIN_Y_LIMIT_MAX, false);
    gpio_expander_set_pullup(PIN_Y_LIMIT_MAX, true);
    gpio_expander_set_direction(PIN_X_ALARM_STATUS, false);
    gpio_expander_set_pullup(PIN_X_ALARM_STATUS, true);
    gpio_expander_set_direction(PIN_Y_ALARM_STATUS, false);
    gpio_expander_set_pullup(PIN_Y_ALARM_STATUS, true);

    // ---- Safe initial output states ----
    gpio_expander_write(PIN_X_DIR, 0);
    gpio_expander_write(PIN_X_ENABLE, 0);      // servo disabled
    // Direct Y pulse (GPIO2) is initialized in initDirectOutputs().
    gpio_expander_write(PIN_Y_DIR, 0);
    gpio_expander_write(PIN_Y_ENABLE, 0);      // stepper disabled
    gpio_expander_write(PIN_THETA_DIR, 0);
    gpio_expander_write(PIN_THETA_ENABLE, 0);
    gpio_expander_write(PIN_GRIPPER, 0);       // gripper open
    gpio_expander_write(PIN_X_ALARM_RESET, 0);
    gpio_expander_write(PIN_Y_ALARM_RESET, 0);

    ESP_LOGI(TAG, "MCP23S17 pins configured");
    return true;
}

// ---------------------------------------------------------------------------
// app_main — ESP-IDF entry point
// ---------------------------------------------------------------------------
extern "C" void app_main(void) {
    ESP_LOGI(TAG, "\n========================================");
    ESP_LOGI(TAG, "WT32-ETH01 Gantry Controller");
    ESP_LOGI(TAG, "MCP23S17 IO Expansion Mode");
    ESP_LOGI(TAG, "========================================\n");

    // ------------------------------------------------------------------
    // 1) Initialize MCP23S17 GPIO expander (required for all IO)
    // ------------------------------------------------------------------
    if (!initMcp23s17()) {
        return;  // cannot proceed without IO expander
    }

    // ------------------------------------------------------------------
    // 2) Configure pulse-train motor drivers
    //    Pulse pins on direct ESP32 GPIO (LEDC); control/status via MCP where routed
    // ------------------------------------------------------------------
    PulseMotor::DriverConfig xConfig;
    PulseMotor::DriverConfig yConfig;
    PulseMotor::DriverConfig thetaConfig;

    // X-axis outputs
    xConfig.pulse_pin = PIN_X_PULSE_EXP;
    xConfig.dir_pin = PIN_X_DIR;
    xConfig.enable_pin = PIN_X_ENABLE;
    xConfig.alarm_reset_pin = PIN_X_ALARM_RESET;

    // X-axis inputs
    xConfig.alarm_pin = PIN_X_ALARM_STATUS;
    xConfig.in_position_pin = -1;
    xConfig.brake_pin = -1;

    // X-axis encoder (PCNT hardware)
    xConfig.encoder_a_pin = PIN_X_ENC_A;
    xConfig.encoder_b_pin = PIN_X_ENC_B;
    xConfig.enable_encoder_feedback = true;
    xConfig.pcnt_unit = X_ENCODER_PCNT_UNIT;

    // X-axis limit switches are handled by Gantry layer
    xConfig.limit_min_pin = -1;
    xConfig.limit_max_pin = -1;
    xConfig.homing_speed_pps = AXIS_X_HOMING_SPEED_PPS;

    // X-axis motor parameters
    xConfig.encoder_ppr = AXIS_X_ENCODER_PPR;
    xConfig.max_pulse_freq = AXIS_X_MAX_PULSE_FREQ_HZ;
    xConfig.gear_numerator = AXIS_X_GEAR_NUMERATOR;
    xConfig.gear_denominator = AXIS_X_GEAR_DENOMINATOR;
    xConfig.invert_dir_pin = AXIS_X_INVERT_DIR;
    xConfig.invert_output_logic = AXIS_X_INVERT_OUTPUT_LOGIC;
    xConfig.ledc_resolution = AXIS_X_LEDC_RESOLUTION_BITS;
    xConfig.limit_debounce_cycles = AXIS_X_LIMIT_DEBOUNCE_CYCLES;
    xConfig.limit_sample_interval_ms = AXIS_X_LIMIT_SAMPLE_INTERVAL_MS;

    // Y-axis outputs and inputs (Kinetix 5100 pulse-train)
    yConfig.pulse_pin = PIN_Y_PULSE_EXP;
    yConfig.dir_pin = PIN_Y_DIR;
    yConfig.enable_pin = PIN_Y_ENABLE;
    yConfig.alarm_reset_pin = PIN_Y_ALARM_RESET;
    yConfig.alarm_pin = PIN_Y_ALARM_STATUS;
    yConfig.encoder_a_pin = PIN_Y_ENC_A;
    yConfig.encoder_b_pin = PIN_Y_ENC_B;
    yConfig.enable_encoder_feedback = false;
    yConfig.pcnt_unit = Y_ENCODER_PCNT_UNIT;
    yConfig.encoder_ppr = AXIS_Y_ENCODER_PPR;
    yConfig.max_pulse_freq = AXIS_Y_MAX_PULSE_FREQ_HZ;
    yConfig.gear_numerator = AXIS_Y_GEAR_NUMERATOR;
    yConfig.gear_denominator = AXIS_Y_GEAR_DENOMINATOR;
    yConfig.invert_dir_pin = AXIS_Y_INVERT_DIR;
    yConfig.invert_output_logic = AXIS_Y_INVERT_OUTPUT_LOGIC;
    yConfig.ledc_resolution = AXIS_Y_LEDC_RESOLUTION_BITS;
    yConfig.limit_debounce_cycles = AXIS_Y_LIMIT_DEBOUNCE_CYCLES;
    yConfig.limit_sample_interval_ms = AXIS_Y_LIMIT_SAMPLE_INTERVAL_MS;

    // Theta-axis pulse-train driver
    thetaConfig.pulse_pin = GPIO_EXPANDER_DIRECT_PIN(PIN_THETA_PULSE);
    thetaConfig.dir_pin = PIN_THETA_DIR;
    thetaConfig.enable_pin = PIN_THETA_ENABLE;
    thetaConfig.alarm_pin = -1;
    thetaConfig.alarm_reset_pin = -1;
    thetaConfig.encoder_a_pin = PIN_THETA_ENC_A;
    thetaConfig.encoder_b_pin = PIN_THETA_ENC_B;
    thetaConfig.enable_encoder_feedback = false;
    thetaConfig.pcnt_unit = THETA_ENCODER_PCNT_UNIT;
    thetaConfig.encoder_ppr = AXIS_THETA_ENCODER_PPR;
    thetaConfig.max_pulse_freq = AXIS_THETA_MAX_PULSE_FREQ_HZ;
    thetaConfig.gear_numerator = AXIS_THETA_GEAR_NUMERATOR;
    thetaConfig.gear_denominator = AXIS_THETA_GEAR_DENOMINATOR;
    thetaConfig.invert_dir_pin = AXIS_THETA_INVERT_DIR;
    thetaConfig.invert_output_logic = AXIS_THETA_INVERT_OUTPUT_LOGIC;
    thetaConfig.ledc_resolution = AXIS_THETA_LEDC_RESOLUTION_BITS;

    // ------------------------------------------------------------------
    // 3) Create Gantry instance and configure all axes
    // ------------------------------------------------------------------
    static Gantry::Gantry gantry(xConfig, yConfig, thetaConfig, PIN_GRIPPER);

    // X-axis soft-limit switches (MCP23S17)
    gantry.setLimitPins(PIN_X_LIMIT_MIN, PIN_X_LIMIT_MAX);

    // Drivetrains (per-axis mm/deg <-> pulse conversion)
    PulseMotor::DrivetrainConfig xDt;
    xDt.type = PulseMotor::DrivetrainType::BELT;
    xDt.belt_lead_mm_per_rev = AXIS_X_BELT_LEAD_MM_PER_REV;
    xDt.encoder_ppr = AXIS_X_ENCODER_PPR;
    xDt.motor_reducer_ratio = AXIS_X_MOTOR_REDUCER_RATIO;
    gantry.setXDrivetrain(xDt);

    PulseMotor::DrivetrainConfig yDt;
    yDt.type = PulseMotor::DrivetrainType::BALLSCREW;
    yDt.lead_mm = AXIS_Y_BALLSCREW_LEAD_MM;
    yDt.encoder_ppr = AXIS_Y_ENCODER_PPR;
    yDt.motor_reducer_ratio = AXIS_Y_MOTOR_REDUCER_RATIO;
    gantry.setYDrivetrain(yDt);

    PulseMotor::DrivetrainConfig thetaDt;
    thetaDt.type = PulseMotor::DrivetrainType::ROTARY_DIRECT;
    thetaDt.output_gear_ratio = AXIS_THETA_OUTPUT_GEAR_RATIO;
    thetaDt.encoder_ppr = AXIS_THETA_ENCODER_PPR;
    thetaDt.motor_reducer_ratio = AXIS_THETA_MOTOR_REDUCER_RATIO;
    gantry.setThetaDrivetrain(thetaDt);

    // Joint-space travel limits (used by move-command validation)
    gantry.setJointLimits(AXIS_X_TRAVEL_MIN_MM, AXIS_X_TRAVEL_MAX_MM,
                          AXIS_Y_TRAVEL_MIN_MM, AXIS_Y_TRAVEL_MAX_MM,
                          AXIS_THETA_TRAVEL_MIN_DEG, AXIS_THETA_TRAVEL_MAX_DEG);

    // Safe Y height for pick-and-place sequences
    gantry.setSafeYHeight(GANTRY_SAFE_Y_HEIGHT_MM);

    // ------------------------------------------------------------------
    // 4) Initialize and enable the gantry
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
    // 5) Create FreeRTOS tasks
    // ------------------------------------------------------------------

    // 5a) Gantry periodic update (Core 1, high priority)
    static UpdateTaskConfig updateCfg = {&gantry};
    BaseType_t result = xTaskCreatePinnedToCore(
        gantryUpdateTask,
        "GantryUpdate",
        GANTRY_UPDATE_TASK_STACK,
        &updateCfg,
        GANTRY_UPDATE_TASK_PRIORITY,
        NULL,
        GANTRY_UPDATE_TASK_CORE
    );
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create Gantry Update task!");
    }

    // 5b) Interactive serial console (Core 0, low priority)
    static GantryTestConsoleConfig consoleCfg = {};
    consoleCfg.gantry = &gantry;
    consoleCfg.limit_min_pin = PIN_X_LIMIT_MIN;
    consoleCfg.limit_max_pin = PIN_X_LIMIT_MAX;
    consoleCfg.use_mcp23s17 = true;
    consoleCfg.limit_switches_active = true;
    consoleCfg.x_pulse_pin = PIN_X_PULSE;
    consoleCfg.x_dir_pin = xConfig.dir_pin;
    consoleCfg.x_enable_pin = xConfig.enable_pin;
    consoleCfg.x_alarm_pin = PIN_X_ALARM_STATUS;
    consoleCfg.x_alarm_reset_pin = PIN_X_ALARM_RESET;
    consoleCfg.y_alarm_pin = PIN_Y_ALARM_STATUS;
    consoleCfg.y_alarm_reset_pin = PIN_Y_ALARM_RESET;
    consoleCfg.x_encoder_a_pin = PIN_X_ENC_A;
    consoleCfg.x_encoder_b_pin = PIN_X_ENC_B;
    consoleCfg.y_pulse_pin = PIN_Y_PULSE;
    consoleCfg.y_encoder_a_pin = PIN_Y_ENC_A;
    consoleCfg.y_encoder_b_pin = PIN_Y_ENC_B;
    consoleCfg.x_pulse_ledc_channel = X_PULSE_LEDC_CHANNEL;
    consoleCfg.y_pulse_ledc_channel = Y_PULSE_LEDC_CHANNEL;
    consoleCfg.theta_pwm_pin = -1;

    result = xTaskCreatePinnedToCore(
        gantryTestConsoleTask,
        "SerialCmd",
        CONSOLE_TASK_STACK,
        &consoleCfg,
        CONSOLE_TASK_PRIORITY,
        NULL,
        CONSOLE_TASK_CORE
    );
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create Serial task!");
    }

    ESP_LOGI(TAG, "All tasks created successfully");
    ESP_LOGI(TAG, "System ready — type 'help' for commands");
    ESP_LOGI(TAG, "");
    gantryTestPrintHelp();

    // Main task exits; FreeRTOS tasks handle everything
    vTaskDelete(NULL);
}

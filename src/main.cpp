/**
 * @file main.cpp
 * @brief Example application for Gantry library
 * @version 1.0.0
 * 
 * This example demonstrates basic usage of the Gantry library with:
 * - X-axis control via SDF08NK8X servo driver
 * - Y-axis control via step/dir stepper
 * - Theta-axis control via PWM servo
 * - End-effector (gripper) control
 * - Serial command interface for interactive testing
 * 
 * Based on configuration patterns from SDF08NK8X-Driver-library branch
 */

#include "Gantry.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/pcnt.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "gpio_expander.h"
#include "MCP23S17.h"
#include "gantry_test_console.h"
#include "gantry_app_constants.h"

static const char *TAG = "GantryExample";

struct UpdateTaskConfig {
    Gantry::Gantry *gantry;
};

/**
 * @brief Gantry update task - Calls gantry.update() at 100 Hz
 */
void gantryUpdateTask(void *param) {
    auto *cfg = static_cast<UpdateTaskConfig *>(param);
    if (cfg == nullptr || cfg->gantry == nullptr) {
        ESP_LOGE(TAG, "Invalid Gantry update task config");
        vTaskDelete(nullptr);
        return;
    }

    const TickType_t updateInterval = pdMS_TO_TICKS(10);  // 100 Hz
    ESP_LOGI(TAG, "Gantry update task started");

    while (1) {
        cfg->gantry->update();
        vTaskDelay(updateInterval);
    }
}

/**
 * @brief ESP-IDF app_main entry point
 */
extern "C" void app_main(void) {
    ESP_LOGI(TAG, "\n========================================");
    ESP_LOGI(TAG, "Gantry Library Example");
    ESP_LOGI(TAG, "========================================\n");

    const int activePulsePin = APP_USE_MCP23S17 ? PIN_PULSE : PIN_X_PULSE_DIRECT;
    const int activeLimitMinPin = APP_USE_MCP23S17 ? PIN_LIMIT_MIN : PIN_X_LIMIT_MIN_DIRECT;
    const int activeLimitMaxPin = APP_USE_MCP23S17 ? PIN_LIMIT_MAX : PIN_X_LIMIT_MAX_DIRECT;
    
    // ========================================================================
    // Initialize MCP23S17 GPIO Expander
    // ========================================================================
    if (APP_USE_MCP23S17) {
        ESP_LOGI(TAG, "Initializing MCP23S17 GPIO expander...");
        mcp23s17_config_t mcp_config = {};
        mcp_config.spi_host = SPI2_HOST;
        mcp_config.cs_pin = (gpio_num_t)PIN_SPI_CS;
        mcp_config.miso_pin = (gpio_num_t)PIN_SPI_MISO;
        mcp_config.mosi_pin = (gpio_num_t)PIN_SPI_MOSI;
        mcp_config.sclk_pin = (gpio_num_t)PIN_SPI_SCLK;
        mcp_config.device_address = MCP23S17_DEVICE_ADDRESS;
        mcp_config.clock_speed_hz = MCP23S17_CLOCK_HZ;

        if (!gpio_expander_init(&mcp_config)) {
            ESP_LOGE(TAG, "Failed to initialize MCP23S17!");
            ESP_LOGE(TAG, "System will continue but GPIO operations may fail.");
        } else {
            ESP_LOGI(TAG, "MCP23S17 initialized successfully");

            // Configure MCP23S17 pins
            // Output pins
            gpio_expander_set_direction(activePulsePin, true);
            gpio_expander_set_direction(PIN_DIR, true);
            gpio_expander_set_direction(PIN_ENABLE, true);
            gpio_expander_set_direction(PIN_Y_STEP, true);
            gpio_expander_set_direction(PIN_Y_DIR, true);
            gpio_expander_set_direction(PIN_Y_ENABLE, true);
            gpio_expander_set_direction(PIN_GRIPPER, true);
            gpio_expander_set_direction(PIN_LED, true);
            gpio_expander_set_direction(PIN_X_ALARM_RESET, true);
            gpio_expander_set_direction(PIN_X_CWCCW_PROHIB, true);
            gpio_expander_set_direction(PIN_X_PULSE_INHIB, true);

            // Input pins with pull-ups
            gpio_expander_set_direction(PIN_LIMIT_MIN, false);
            gpio_expander_set_pullup(PIN_LIMIT_MIN, true);
            gpio_expander_set_direction(PIN_LIMIT_MAX, false);
            gpio_expander_set_pullup(PIN_LIMIT_MAX, true);
            gpio_expander_set_direction(PIN_X_POS_REACHED, false);
            gpio_expander_set_pullup(PIN_X_POS_REACHED, true);
            gpio_expander_set_direction(PIN_X_BRAKE_STATUS, false);
            gpio_expander_set_pullup(PIN_X_BRAKE_STATUS, true);
            gpio_expander_set_direction(PIN_X_ALARM_STATUS, false);
            gpio_expander_set_pullup(PIN_X_ALARM_STATUS, true);

            // Initialize outputs to safe states
            gpio_expander_write(activePulsePin, 0);
            gpio_expander_write(PIN_DIR, 0);
            gpio_expander_write(PIN_ENABLE, 0);  // Disabled
            gpio_expander_write(PIN_Y_STEP, 0);
            gpio_expander_write(PIN_Y_DIR, 0);
            gpio_expander_write(PIN_Y_ENABLE, 0);  // Disabled
            gpio_expander_write(PIN_GRIPPER, 0);  // Open
            gpio_expander_write(PIN_LED, 0);
            gpio_expander_write(PIN_X_ALARM_RESET, 0);
            gpio_expander_write(PIN_X_CWCCW_PROHIB, 0);
            gpio_expander_write(PIN_X_PULSE_INHIB, 0);

            ESP_LOGI(TAG, "MCP23S17 pins configured");
        }
    } else {
        ESP_LOGW(TAG, "MCP23S17 disabled: using direct WT32 GPIO test mapping");
        gpio_expander_init(nullptr);
        gpio_expander_set_direction(activePulsePin, true);
        gpio_expander_set_direction(PIN_X_DIR_DIRECT, true);
        gpio_expander_set_direction(PIN_X_ENABLE_DIRECT, true);
        if (PIN_X_ALARM_RESET_DIRECT >= 0) {
            gpio_expander_set_direction(PIN_X_ALARM_RESET_DIRECT, true);
        }
        gpio_expander_set_direction(PIN_X_ALARM_DIRECT, false);
        gpio_expander_set_pullup(PIN_X_ALARM_DIRECT, true);
        gpio_expander_set_direction(activeLimitMinPin, false);
        gpio_expander_set_pullup(activeLimitMinPin, true);
        gpio_expander_set_direction(activeLimitMaxPin, false);
        gpio_expander_set_pullup(activeLimitMaxPin, true);
    }
    
    // ========================================================================
    // Configure X-axis servo driver
    // ========================================================================
    BergerdaServo::DriverConfig xConfig;
    
    // Pin configuration
    // All basic digital control/status lines are routed via MCP23S17.
    xConfig.output_pin_nos[6] = activePulsePin;
    xConfig.output_pin_nos[7] = APP_USE_MCP23S17 ? PIN_DIR : PIN_X_DIR_DIRECT;
    xConfig.output_pin_nos[0] = APP_USE_MCP23S17 ? PIN_ENABLE : PIN_X_ENABLE_DIRECT;
    xConfig.output_pin_nos[1] = APP_USE_MCP23S17 ? PIN_X_ALARM_RESET : PIN_X_ALARM_RESET_DIRECT;
    xConfig.output_pin_nos[2] = APP_USE_MCP23S17 ? PIN_X_CWCCW_PROHIB : -1;
    xConfig.output_pin_nos[4] = APP_USE_MCP23S17 ? PIN_X_PULSE_INHIB : -1;

    // Drive status inputs via MCP23S17
    xConfig.input_pin_nos[0] = APP_USE_MCP23S17 ? PIN_X_POS_REACHED : -1;
    xConfig.input_pin_nos[1] = APP_USE_MCP23S17 ? PIN_X_BRAKE_STATUS : -1;
    xConfig.input_pin_nos[2] = APP_USE_MCP23S17 ? PIN_X_ALARM_STATUS : PIN_X_ALARM_DIRECT;
    
    // Encoder pins (must stay on direct ESP32 GPIO for PCNT)
    xConfig.input_pin_nos[3] = PIN_ENC_A;    // Encoder A+ (ESP32 GPIO 35)
    xConfig.input_pin_nos[4] = PIN_ENC_B;    // Encoder B+ (ESP32 GPIO 36)
    xConfig.enable_encoder_feedback = true;
    xConfig.pcnt_unit = PCNT_UNIT_0;
    
    // Limit switches are required for any active X-axis mode.
    xConfig.limit_min_pin = activeLimitMinPin;
    xConfig.limit_max_pin = activeLimitMaxPin;
    xConfig.homing_speed_pps = GANTRY_HOMING_SPEED_PPS;
    
    // Motor parameters
    xConfig.encoder_ppr = GANTRY_ENCODER_PPR;
    xConfig.max_pulse_freq = GANTRY_MAX_PULSE_FREQ;
    
    // Control mode
    xConfig.pulse_mode = BergerdaServo::PulseMode::PULSE_DIRECTION;
    xConfig.control_mode = BergerdaServo::ControlMode::POSITION;
    
    // Create gantry instance (gripper on MCP23S17 PB0)
    static Gantry::Gantry gantry(xConfig, APP_USE_MCP23S17 ? PIN_GRIPPER : -1);
    
    // Configure limit switches (always required)
    gantry.setLimitPins(activeLimitMinPin, activeLimitMaxPin);
    
    // Configure Y-axis stepper on MCP23S17
    if (APP_USE_MCP23S17) {
        gantry.setYAxisPins(PIN_Y_STEP, PIN_Y_DIR, PIN_Y_ENABLE);  // MCP23S17 PA5, PA6, PA7
        gantry.setYAxisStepsPerMm(GANTRY_Y_STEPS_PER_MM);
        gantry.setYAxisLimits(GANTRY_Y_MIN_MM, GANTRY_Y_MAX_MM);
        gantry.setYAxisMotionLimits(GANTRY_Y_MAX_SPEED_MMPS, GANTRY_Y_ACCEL_MMPS2, GANTRY_Y_DECEL_MMPS2);
    }
    
    // Configure theta servo
    gantry.setThetaServo(PIN_THETA_PWM, 0);  // LEDC channel 0
    gantry.setThetaLimits(GANTRY_THETA_MIN_DEG, GANTRY_THETA_MAX_DEG);
    gantry.setThetaPulseRange(GANTRY_THETA_MIN_PULSE_US, GANTRY_THETA_MAX_PULSE_US);
    
    // Set safe height
    gantry.setSafeYHeight(GANTRY_SAFE_Y_MM);
    
    // Initialize gantry
    ESP_LOGI(TAG, "Initializing gantry...");
    if (!gantry.begin()) {
        ESP_LOGE(TAG, "ERROR: Gantry initialization failed!");
        ESP_LOGE(TAG, "Check pin connections and try again.");
        return;
    }
    ESP_LOGI(TAG, "OK Gantry initialized");
    
    // Enable motors
    gantry.enable();
    ESP_LOGI(TAG, "OK Motors enabled");
    
    // Create FreeRTOS tasks
    static UpdateTaskConfig updateCfg = {&gantry};
    static GantryTestConsoleConfig consoleCfg = {};
    consoleCfg.gantry = &gantry;
    consoleCfg.limit_min_pin = activeLimitMinPin;
    consoleCfg.limit_max_pin = activeLimitMaxPin;
    consoleCfg.use_mcp23s17 = APP_USE_MCP23S17;
    consoleCfg.limit_switches_active = true;
    consoleCfg.x_pulse_pin = xConfig.output_pin_nos[6];
    consoleCfg.x_dir_pin = xConfig.output_pin_nos[7];
    consoleCfg.x_enable_pin = xConfig.output_pin_nos[0];
    consoleCfg.x_alarm_pin = xConfig.input_pin_nos[2];
    consoleCfg.x_alarm_reset_pin = xConfig.output_pin_nos[1];
    consoleCfg.x_encoder_a_pin = xConfig.input_pin_nos[3];
    consoleCfg.x_encoder_b_pin = xConfig.input_pin_nos[4];
    consoleCfg.theta_pwm_pin = PIN_THETA_PWM;

    // Gantry update task - Core 1, high priority
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

    // Serial command task - Core 0, low priority
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
    ESP_LOGI(TAG, "System ready");
    ESP_LOGI(TAG, "");
    gantryTestPrintHelp();
    
    // Main task can exit - FreeRTOS tasks handle everything
    vTaskDelete(NULL);
}
 
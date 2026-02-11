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
    
    // ========================================================================
    // Initialize MCP23S17 GPIO Expander
    // ========================================================================
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
        gpio_expander_set_direction(PIN_PULSE, true);
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
        gpio_expander_write(PIN_PULSE, 0);
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
    
    // ========================================================================
    // Configure X-axis servo driver
    // ========================================================================
    BergerdaServo::DriverConfig xConfig;
    
    // Pin configuration
    // All basic digital control/status lines are routed via MCP23S17.
    xConfig.output_pin_nos[6] = PIN_PULSE;   // PULSE (direct ESP32 GPIO via LEDC)
    xConfig.output_pin_nos[7] = PIN_DIR;     // DIR (MCP23S17 PA1)
    xConfig.output_pin_nos[0] = PIN_ENABLE;  // ENABLE (MCP23S17 PA2)
    xConfig.output_pin_nos[1] = PIN_X_ALARM_RESET;  // ARST (MCP23S17 PB5)
    xConfig.output_pin_nos[2] = PIN_X_CWCCW_PROHIB; // CW/CCW prohibition (MCP23S17 PB6)
    xConfig.output_pin_nos[4] = PIN_X_PULSE_INHIB;  // INH (MCP23S17 PB7)

    // Drive status inputs via MCP23S17
    xConfig.input_pin_nos[0] = PIN_X_POS_REACHED;   // OUT1/POS_REACHED (PB2)
    xConfig.input_pin_nos[1] = PIN_X_BRAKE_STATUS;  // OUT2/BRAKE (PB3)
    xConfig.input_pin_nos[2] = PIN_X_ALARM_STATUS;  // OUT3/ALM (PB4)
    
    // Encoder pins (must stay on direct ESP32 GPIO for PCNT)
    xConfig.input_pin_nos[3] = PIN_ENC_A;    // Encoder A+ (ESP32 GPIO 35)
    xConfig.input_pin_nos[4] = PIN_ENC_B;    // Encoder B+ (ESP32 GPIO 36)
    xConfig.enable_encoder_feedback = true;
    xConfig.pcnt_unit = PCNT_UNIT_0;
    
    // Limit switch pins (MCP23S17 PA3, PA4)
    xConfig.limit_min_pin = PIN_LIMIT_MIN;   // MCP23S17 PA3
    xConfig.limit_max_pin = PIN_LIMIT_MAX;   // MCP23S17 PA4
    xConfig.homing_speed_pps = GANTRY_HOMING_SPEED_PPS;
    
    // Motor parameters
    xConfig.encoder_ppr = GANTRY_ENCODER_PPR;
    xConfig.max_pulse_freq = GANTRY_MAX_PULSE_FREQ;
    
    // Control mode
    xConfig.pulse_mode = BergerdaServo::PulseMode::PULSE_DIRECTION;
    xConfig.control_mode = BergerdaServo::ControlMode::POSITION;
    
    // Create gantry instance (gripper on MCP23S17 PB0)
    static Gantry::Gantry gantry(xConfig, PIN_GRIPPER);  // MCP23S17 PB0
    
    // Configure limit switches
    gantry.setLimitPins(PIN_LIMIT_MIN, PIN_LIMIT_MAX);
    
    // Configure Y-axis stepper on MCP23S17
    gantry.setYAxisPins(PIN_Y_STEP, PIN_Y_DIR, PIN_Y_ENABLE);  // MCP23S17 PA5, PA6, PA7
    gantry.setYAxisStepsPerMm(GANTRY_Y_STEPS_PER_MM);
    gantry.setYAxisLimits(GANTRY_Y_MIN_MM, GANTRY_Y_MAX_MM);
    gantry.setYAxisMotionLimits(GANTRY_Y_MAX_SPEED_MMPS, GANTRY_Y_ACCEL_MMPS2, GANTRY_Y_DECEL_MMPS2);
    
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
    static GantryTestConsoleConfig consoleCfg = {&gantry, PIN_LIMIT_MIN, PIN_LIMIT_MAX};

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
 
/**
 * @file main.cpp
 * @brief Example application for Gantry library
 * @version 1.0.0
 * 
 * This example demonstrates basic usage of the Gantry library with:
 * - X-axis control via SDF08NK8X servo driver
 * - Y-axis control via SDF08NK8X servo driver
 * - Theta-axis control via PWM servo
 * - End-effector (gripper) control
 * - Serial command interface for interactive testing
 * 
 * Based on configuration patterns from SDF08NK8X-Driver-library branch
 */

#include "Gantry.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "gpio_expander.h"
#include "MCP23S17.h"
#include "gantry_test_console.h"
#include "gantry_app_constants.h"

static const char *TAG = "GantryExample";

#ifndef DIAG_SKIP_X_ARST_INIT_WRITE
#define DIAG_SKIP_X_ARST_INIT_WRITE 0
#endif

#ifndef DIAG_SKIP_Y_ARST_INIT_WRITE
#define DIAG_SKIP_Y_ARST_INIT_WRITE 0
#endif

static const char* resetReasonToString(esp_reset_reason_t reason) {
    switch (reason) {
        case ESP_RST_UNKNOWN: return "UNKNOWN";
        case ESP_RST_POWERON: return "POWERON";
        case ESP_RST_EXT: return "EXTERNAL_PIN";
        case ESP_RST_SW: return "SOFTWARE";
        case ESP_RST_PANIC: return "PANIC";
        case ESP_RST_INT_WDT: return "INT_WDT";
        case ESP_RST_TASK_WDT: return "TASK_WDT";
        case ESP_RST_WDT: return "WDT";
        case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
        case ESP_RST_BROWNOUT: return "BROWNOUT";
        case ESP_RST_SDIO: return "SDIO";
        default: return "UNMAPPED";
    }
}

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
    const esp_reset_reason_t resetReason = esp_reset_reason();
    ESP_LOGW(TAG, "Boot reset reason: %s (%d)", resetReasonToString(resetReason), (int)resetReason);
    ESP_LOGW(TAG, "Strapping snapshot at app_main entry: GPIO2=%d GPIO12=%d GPIO15=%d",
             gpio_get_level((gpio_num_t)2),
             gpio_get_level((gpio_num_t)12),
             gpio_get_level((gpio_num_t)15));

    uint32_t bootStep = 0;
    auto logStep = [&](const char *message) {
        bootStep++;
        ESP_LOGI(TAG, "[BOOT-STEP %lu] %s", (unsigned long)bootStep, message);
    };
    auto checkErr = [&](const char *op, esp_err_t err) -> bool {
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "[BOOT-STEP %lu] %s failed: %s",
                     (unsigned long)bootStep, op, esp_err_to_name(err));
            return false;
        }
        return true;
    };

    ESP_LOGI(TAG, "\n========================================");
    ESP_LOGI(TAG, "Gantry Library Example");
    ESP_LOGI(TAG, "========================================\n");

    const int activePulsePin = PIN_X_PULSE;
    const int activeLimitMinPin = PIN_X_LIMIT_MIN;
    const int activeLimitMaxPin = PIN_X_LIMIT_MAX;
    
    // ========================================================================
    // Initialize MCP23S17 GPIO Expander
    // ========================================================================
    logStep("Initializing MCP23S17 GPIO expander");
    mcp23s17_config_t mcp_config = {};
    mcp_config.spi_host = SPI2_HOST;
    mcp_config.cs_pin = (gpio_num_t)PIN_SPI_CS;
    mcp_config.miso_pin = (gpio_num_t)PIN_SPI_MISO;
    mcp_config.mosi_pin = (gpio_num_t)PIN_SPI_MOSI;
    mcp_config.sclk_pin = (gpio_num_t)PIN_SPI_SCLK;
    mcp_config.device_address = MCP23S17_DEVICE_ADDRESS;
    mcp_config.clock_speed_hz = MCP23S17_CLOCK_HZ;

    if (!gpio_expander_init(&mcp_config)) {
        ESP_LOGE(TAG, "FATAL: Failed to initialize MCP23S17!");
        return;
    }
    ESP_LOGI(TAG, "MCP23S17 initialized successfully");

    // Configure MCP23S17 pins with fine-grained step diagnostics.
    const uint8_t outputPins[] = {
        PIN_X_DIR, PIN_X_ENABLE, PIN_Y_DIR, PIN_Y_ENABLE,
        PIN_GRIPPER, PIN_X_ALARM_RESET, PIN_Y_ALARM_RESET
    };
    for (uint8_t pin : outputPins) {
        logStep("MCP set output direction");
        if (!checkErr("gpio_expander_set_direction(output)",
                      gpio_expander_set_direction(pin, true))) {
            return;
        }
    }

    const uint8_t inputPullupPins[] = {
        PIN_X_LIMIT_MIN, PIN_X_LIMIT_MAX, PIN_Y_LIMIT_MIN, PIN_Y_LIMIT_MAX,
        PIN_X_ALARM_STATUS, PIN_Y_ALARM_STATUS, PIN_THETA_LIMIT_MIN, PIN_THETA_LIMIT_MAX
    };
    for (uint8_t pin : inputPullupPins) {
        logStep("MCP set input direction");
        if (!checkErr("gpio_expander_set_direction(input)",
                      gpio_expander_set_direction(pin, false))) {
            return;
        }
        logStep("MCP enable pull-up");
        if (!checkErr("gpio_expander_set_pullup(true)",
                      gpio_expander_set_pullup(pin, true))) {
            return;
        }
    }

    struct InitialOutputState {
        uint8_t pin;
        uint8_t level;
    };
    const InitialOutputState initialStates[] = {
        {PIN_X_DIR, 0},
        {PIN_X_ENABLE, 0},  // Disabled
        {PIN_Y_DIR, 0},
        {PIN_Y_ENABLE, 0},  // Disabled
        {PIN_GRIPPER, 0},   // Open
        {PIN_X_ALARM_RESET, 0},
        {PIN_Y_ALARM_RESET, 0},
    };
    for (const auto &s : initialStates) {
        if (s.pin == PIN_X_ALARM_RESET && DIAG_SKIP_X_ARST_INIT_WRITE) {
            logStep("MCP skip initial output write (X alarm reset)");
            continue;
        }
        if (s.pin == PIN_Y_ALARM_RESET && DIAG_SKIP_Y_ARST_INIT_WRITE) {
            logStep("MCP skip initial output write (Y alarm reset)");
            continue;
        }
        logStep("MCP write initial output level");
        if (!checkErr("gpio_expander_write(initial)",
                      gpio_expander_write(s.pin, s.level))) {
            return;
        }
    }

    ESP_LOGI(TAG, "MCP23S17 pins configured");

    // Configure direct WT32 pulse outputs (LEDC-capable routing)
    gpio_config_t pulse_io_conf = {};
    pulse_io_conf.pin_bit_mask = (1ULL << PIN_X_PULSE) | (1ULL << PIN_Y_PULSE);
    pulse_io_conf.mode = GPIO_MODE_OUTPUT;
    pulse_io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    pulse_io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    pulse_io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&pulse_io_conf);
    gpio_set_level((gpio_num_t)PIN_X_PULSE, 0);
    gpio_set_level((gpio_num_t)PIN_Y_PULSE, 0);
    
    // ========================================================================
    // Configure X-axis servo driver
    // ========================================================================
    BergerdaServo::DriverConfig xConfig;
    
    // Pin configuration
    // All basic digital control/status lines are routed via MCP23S17.
    xConfig.output_pin_nos[6] = activePulsePin;
    xConfig.output_pin_nos[7] = PIN_X_DIR;
    xConfig.output_pin_nos[0] = PIN_X_ENABLE;
    xConfig.output_pin_nos[1] = PIN_X_ALARM_RESET;
    xConfig.output_pin_nos[2] = -1;
    xConfig.output_pin_nos[4] = -1;
    xConfig.ledc_channel = X_PULSE_LEDC_CHANNEL;
    xConfig.ledc_pulse_pin = activePulsePin;

    // Drive status inputs via MCP23S17
    xConfig.input_pin_nos[0] = -1;
    xConfig.input_pin_nos[1] = -1;
    xConfig.input_pin_nos[2] = PIN_X_ALARM_STATUS;
    
    // Encoder pins (must stay on direct ESP32 GPIO for PCNT)
    xConfig.input_pin_nos[3] = PIN_X_ENC_A;    // Encoder A+ (ESP32 GPIO 35)
    xConfig.input_pin_nos[4] = PIN_X_ENC_B;    // Encoder B+ (ESP32 GPIO 36)
    xConfig.enable_encoder_feedback = true;
    xConfig.pcnt_unit = X_ENCODER_PCNT_UNIT;
    
    // Limit logic is handled in Gantry library (reusable limit objects for all axes).
    xConfig.limit_min_pin = -1;
    xConfig.limit_max_pin = -1;
    xConfig.homing_speed_pps = GANTRY_HOMING_SPEED_PPS;
    
    // Motor parameters
    xConfig.encoder_ppr = GANTRY_ENCODER_PPR;
    xConfig.max_pulse_freq = GANTRY_MAX_PULSE_FREQ;
    
    // Control mode
    xConfig.pulse_mode = BergerdaServo::PulseMode::PULSE_DIRECTION;
    xConfig.control_mode = BergerdaServo::ControlMode::POSITION;
    xConfig.invert_dir_pin = true;
    
    // Configure Y-axis servo driver (same hardware/control path as X-axis)
    BergerdaServo::DriverConfig yConfig;
    yConfig.output_pin_nos[6] = PIN_Y_PULSE;
    yConfig.output_pin_nos[7] = PIN_Y_DIR;
    yConfig.output_pin_nos[0] = PIN_Y_ENABLE;
    yConfig.output_pin_nos[1] = PIN_Y_ALARM_RESET;
    yConfig.output_pin_nos[2] = -1;
    yConfig.output_pin_nos[4] = -1;
    yConfig.ledc_channel = Y_PULSE_LEDC_CHANNEL;
    yConfig.ledc_pulse_pin = PIN_Y_PULSE;
    yConfig.input_pin_nos[0] = -1;
    yConfig.input_pin_nos[1] = -1;
    yConfig.input_pin_nos[2] = PIN_Y_ALARM_STATUS;
    yConfig.input_pin_nos[3] = PIN_Y_ENC_A;
    yConfig.input_pin_nos[4] = PIN_Y_ENC_B;
    yConfig.enable_encoder_feedback = true;
    yConfig.pcnt_unit = Y_ENCODER_PCNT_UNIT;
    yConfig.limit_min_pin = PIN_Y_LIMIT_MIN;
    yConfig.limit_max_pin = PIN_Y_LIMIT_MAX;
    yConfig.homing_speed_pps = GANTRY_HOMING_SPEED_PPS;
    yConfig.encoder_ppr = GANTRY_ENCODER_PPR;
    yConfig.max_pulse_freq = GANTRY_MAX_PULSE_FREQ;
    yConfig.pulse_mode = BergerdaServo::PulseMode::PULSE_DIRECTION;
    yConfig.control_mode = BergerdaServo::ControlMode::POSITION;
    yConfig.invert_dir_pin = true;

    // Create gantry instance (X and Y both using SDF08NK8X servo drivers)
    static Gantry::Gantry gantry(xConfig, yConfig, PIN_GRIPPER);
    
    // Configure limit switches (always required)
    gantry.setLimitPins(activeLimitMinPin, activeLimitMaxPin);
    
    ESP_LOGI(TAG, "Y encoder configured on GPIO %d/%d using PCNT unit %d",
             PIN_Y_ENC_A, PIN_Y_ENC_B, Y_ENCODER_PCNT_UNIT);
    ESP_LOGI(TAG, "Pulse LEDC channel allocation: X=%d, Y=%d (reserved), Theta=%d",
             X_PULSE_LEDC_CHANNEL, Y_PULSE_LEDC_CHANNEL, THETA_PWM_LEDC_CHANNEL);

    // Configure theta servo (dedicated LEDC channel separate from X/Y pulse channels)
    gantry.setThetaServo(PIN_THETA_PWM, THETA_PWM_LEDC_CHANNEL);
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
    consoleCfg.use_mcp23s17 = true;
    consoleCfg.limit_switches_active = true;
    consoleCfg.x_pulse_pin = xConfig.output_pin_nos[6];
    consoleCfg.x_dir_pin = xConfig.output_pin_nos[7];
    consoleCfg.x_enable_pin = xConfig.output_pin_nos[0];
    consoleCfg.x_alarm_pin = xConfig.input_pin_nos[2];
    consoleCfg.x_alarm_reset_pin = xConfig.output_pin_nos[1];
    consoleCfg.y_alarm_pin = PIN_Y_ALARM_STATUS;
    consoleCfg.y_alarm_reset_pin = PIN_Y_ALARM_RESET;
    consoleCfg.x_encoder_a_pin = xConfig.input_pin_nos[3];
    consoleCfg.x_encoder_b_pin = xConfig.input_pin_nos[4];
    consoleCfg.y_pulse_pin = yConfig.output_pin_nos[6];
    consoleCfg.y_encoder_a_pin = yConfig.input_pin_nos[3];
    consoleCfg.y_encoder_b_pin = yConfig.input_pin_nos[4];
    consoleCfg.x_pulse_ledc_channel = X_PULSE_LEDC_CHANNEL;
    consoleCfg.y_pulse_ledc_channel = Y_PULSE_LEDC_CHANNEL;
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

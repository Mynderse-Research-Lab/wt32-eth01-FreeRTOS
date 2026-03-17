/**
 * @file main.cpp
 * @brief Gantry control application for WT32-ETH01 with MCP23S17 IO expansion
 *
 * FreeRTOS application with:
 * - MCP23S17 SPI GPIO expander initialization
 * - X-axis servo control via SDF08NK8X driver (pulse on direct GPIO, control via MCP)
 * - Y-axis stepper control via MCP23S17 pins
 * - Theta-axis PWM servo on direct GPIO
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
    io_conf.pin_bit_mask = (1ULL << PIN_Y_PULSE);
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
    // 2) Configure X-axis servo driver (SDF08NK8X)
    //    Pulse pin on direct ESP32 GPIO (LEDC); everything else via MCP
    // ------------------------------------------------------------------
    BergerdaServo::DriverConfig xConfig;

    // Outputs routed via MCP23S17
    xConfig.output_pin_nos[6] = PIN_X_PULSE_EXP;    // direct ESP32 GPIO pulse out (LEDC)
    xConfig.output_pin_nos[7] = PIN_X_DIR;          // MCP direction
    xConfig.output_pin_nos[0] = PIN_X_ENABLE;       // MCP enable (SON)
    xConfig.output_pin_nos[1] = PIN_X_ALARM_RESET;   // MCP PA5 (ARST)
    xConfig.output_pin_nos[4] = -1;                  // INH not routed in current pin map

    // Inputs routed via MCP23S17
    xConfig.input_pin_nos[2] = PIN_X_ALARM_STATUS;   // MCP PA4 (ALM)

    // Encoder on direct ESP32 GPIO (PCNT hardware)
    xConfig.input_pin_nos[3] = PIN_X_ENC_A;           // GPIO 35
    xConfig.input_pin_nos[4] = PIN_X_ENC_B;           // GPIO 36
    xConfig.enable_encoder_feedback = true;
    xConfig.pcnt_unit = X_ENCODER_PCNT_UNIT;

    // Limit switch handling is done in the Gantry library (not driver-internal)
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

    // ------------------------------------------------------------------
    // 3) Create Gantry instance and configure all axes
    // ------------------------------------------------------------------
    static Gantry::Gantry gantry(xConfig, PIN_GRIPPER);

    // X-axis limit switches (MCP23S17)
    gantry.setLimitPins(PIN_X_LIMIT_MIN, PIN_X_LIMIT_MAX);

    // Y-axis stepper (Y pulse direct GPIO via expander encoding, DIR/EN on MCP Port B)
    gantry.setYAxisPins(PIN_Y_PULSE_EXP, PIN_Y_DIR, PIN_Y_ENABLE);
    gantry.setYAxisStepsPerMm(GANTRY_Y_STEPS_PER_MM);
    gantry.setYAxisLimits(GANTRY_Y_MIN_MM, GANTRY_Y_MAX_MM);
    gantry.setYAxisMotionLimits(GANTRY_Y_MAX_SPEED_MMPS, GANTRY_Y_ACCEL_MMPS2, GANTRY_Y_DECEL_MMPS2);

    // Theta servo (direct ESP32 GPIO, LEDC channel from constants)
    gantry.setThetaServo(PIN_THETA_PWM, THETA_PWM_LEDC_CHANNEL);
    gantry.setThetaLimits(GANTRY_THETA_MIN_DEG, GANTRY_THETA_MAX_DEG);
    gantry.setThetaPulseRange(GANTRY_THETA_MIN_PULSE_US, GANTRY_THETA_MAX_PULSE_US);

    // Safe Y height for pick-and-place sequences
    gantry.setSafeYHeight(GANTRY_SAFE_Y_MM);

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
    consoleCfg.x_dir_pin = xConfig.output_pin_nos[7];
    consoleCfg.x_enable_pin = xConfig.output_pin_nos[0];
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
    consoleCfg.theta_pwm_pin = PIN_THETA_PWM;

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

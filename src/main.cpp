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
#include <inttypes.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <stddef.h>

// ============================================================================
// Pin Definitions - MCP23S17 GPIO Expander Mapping
// ============================================================================
// All digital GPIO operations use MCP23S17 (pins 0-15)
// PWM and encoder pins remain on direct ESP32 GPIO (16+)

// MCP23S17 SPI pins (direct ESP32 GPIO)
#define PIN_SPI_MISO  19  // SPI MISO
#define PIN_SPI_MOSI  23  // SPI MOSI
#define PIN_SPI_SCLK  18  // SPI SCLK
#define PIN_SPI_CS    5   // SPI Chip Select

// MCP23S17 Pin Assignments (Port A: 0-7, Port B: 8-15)
// X-axis pins (on MCP23S17)
#define PIN_PULSE      0   // MCP23S17 Port A Pin 0: Pulse signal (output)
#define PIN_DIR        1   // MCP23S17 Port A Pin 1: Direction signal (output)
#define PIN_ENABLE     2   // MCP23S17 Port A Pin 2: Servo enable SON (output)
#define PIN_LIMIT_MIN  3   // MCP23S17 Port A Pin 3: Home limit switch (input, pullup)
#define PIN_LIMIT_MAX  4   // MCP23S17 Port A Pin 4: End limit switch (input, pullup)

// Y-axis pins (on MCP23S17)
#define PIN_Y_STEP     5   // MCP23S17 Port A Pin 5: Y-axis step pin (output)
#define PIN_Y_DIR      6   // MCP23S17 Port A Pin 6: Y-axis direction pin (output)
#define PIN_Y_ENABLE   7   // MCP23S17 Port A Pin 7: Y-axis enable pin (output)

// End-effector and LED pins (on MCP23S17 Port B)
#define PIN_GRIPPER    8   // MCP23S17 Port B Pin 0: Gripper control pin (output)
#define PIN_LED        9   // MCP23S17 Port B Pin 1: Status LED (output)
// Port B pins 2-7 available for future use

// Direct ESP32 GPIO pins (PWM and encoder - must stay on ESP32)
#define PIN_ENC_A     35  // Direct GPIO: Encoder A+ (input only, PCNT)
#define PIN_ENC_B     36  // Direct GPIO: Encoder B+ (input only, PCNT)
#define PIN_THETA_PWM 13  // Direct GPIO: Theta PWM servo pin (LEDC output)

// Helper macros to convert MCP23S17 pins to GPIO expander format
// MCP23S17 pins are 0-15, direct GPIO pins are 16+
#define MCP_PIN(pin) (pin)           // MCP23S17 pin (0-15)
#define ESP_PIN(pin) ((pin) + 16)    // Direct ESP32 GPIO pin (16+)

static const char *TAG = "GantryExample";

// Gantry system instance
static Gantry::Gantry* g_gantry = nullptr;

// Serial logging helpers (similar to SDF08NK8X example)
#define LOG_PRINT(value) do { \
    ESP_LOGI(TAG, "%s", value); \
} while (0)

#define LOG_PRINTLN(value) do { \
    ESP_LOGI(TAG, "%s", value); \
} while (0)

#define LOG_PRINTF(...) do { \
    ESP_LOGI(TAG, __VA_ARGS__); \
} while (0)

// Forward declarations
void printHelp();
void printStatus();
void printLimits();
void processCommand(const char* cmd);

/**
 * @brief Gantry update task - Calls gantry.update() periodically
 */
void gantryUpdateTask(void *param) {
    (void)param;
    
    const TickType_t updateInterval = pdMS_TO_TICKS(10);  // 100 Hz
    
    // Wait for gantry system to be initialized
    while (g_gantry == nullptr) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    ESP_LOGI(TAG, "Gantry update task started");
    
    while (1) {
        if (g_gantry != nullptr) {
            g_gantry->update();
        }
        vTaskDelay(updateInterval);
    }
    
    vTaskDelete(nullptr);
}

/**
 * @brief Serial command processing task
 */
void serialTask(void *param) {
    (void)param;
    
    char inputLine[256];
    size_t inputIndex = 0;
    
    ESP_LOGI(TAG, "Serial task started");
    ESP_LOGI(TAG, "Type 'help' for commands");
    
    while (1) {
        // Read serial input character by character
        int c = getchar();
        if (c >= 0) {
            if (c == '\n' || c == '\r' || c == ';') {
                if (inputIndex > 0) {
                    inputLine[inputIndex] = '\0';
                    ESP_LOGI(TAG, "[RX] %s", inputLine);
                    processCommand(inputLine);
                    inputIndex = 0;
                }
            } else if (c >= 32 && c <= 126 && inputIndex < sizeof(inputLine) - 1) {
                inputLine[inputIndex++] = c;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    vTaskDelete(nullptr);
}

/**
 * @brief Print help message
 */
void printHelp() {
    LOG_PRINTLN("========================================");
    LOG_PRINTLN("Gantry Library Example - Commands:");
    LOG_PRINTLN("========================================");
    LOG_PRINTLN("  help                 - show this help");
    LOG_PRINTLN("  status               - print gantry status");
    LOG_PRINTLN("  limits               - read limit switches");
    LOG_PRINTLN("  enable               - enable motors");
    LOG_PRINTLN("  disable              - disable motors");
    LOG_PRINTLN("  home                 - home X-axis");
    LOG_PRINTLN("  calibrate            - calibrate X-axis length");
    LOG_PRINTLN("  move <x> <y> <t>     - move to position (mm, mm, deg)");
    LOG_PRINTLN("  grip <0|1>           - control gripper (0=open, 1=close)");
    LOG_PRINTLN("  stop                 - stop all motion");
    LOG_PRINTLN("");
}

/**
 * @brief Print gantry status
 */
void printStatus() {
    if (g_gantry == nullptr) {
        LOG_PRINTLN("ERROR: Gantry not initialized");
        return;
    }
    
    ESP_LOGI(TAG, "=== Gantry Status ===");
    ESP_LOGI(TAG, "X Position: %d pulses", g_gantry->getXEncoder());
    ESP_LOGI(TAG, "Y Position: %d mm", g_gantry->getCurrentY());
    ESP_LOGI(TAG, "Theta: %d deg", g_gantry->getCurrentTheta());
    ESP_LOGI(TAG, "Busy: %s", g_gantry->isBusy() ? "Yes" : "No");
    ESP_LOGI(TAG, "Alarm: %s", g_gantry->isAlarmActive() ? "Yes" : "No");
    
    // Get current joint configuration
    Gantry::JointConfig current = g_gantry->getCurrentJointConfig();
    ESP_LOGI(TAG, "Joint Config: x=%.1f y=%.1f theta=%.1f",
             current.x, current.y, current.theta);
    
    // Get current end-effector pose
    Gantry::EndEffectorPose pose = g_gantry->getCurrentEndEffectorPose();
    ESP_LOGI(TAG, "End-Effector: x=%.1f y=%.1f z=%.1f theta=%.1f",
             pose.x, pose.y, pose.z, pose.theta);
}

/**
 * @brief Print limit switch status
 */
void printLimits() {
    if (g_gantry == nullptr) {
        LOG_PRINTLN("ERROR: Gantry not initialized");
        return;
    }
    
    ESP_LOGI(TAG, "=== Limit Switches ===");
    uint8_t limit_min = gpio_expander_read(PIN_LIMIT_MIN);
    uint8_t limit_max = gpio_expander_read(PIN_LIMIT_MAX);
    ESP_LOGI(TAG, "X_LS_MIN (MCP23S17 PA%d / Home): %s", PIN_LIMIT_MIN,
             limit_min == 0 ? "ACTIVE (LOW)" : "open (HIGH)");
    ESP_LOGI(TAG, "X_LS_MAX (MCP23S17 PA%d / End):  %s", PIN_LIMIT_MAX,
             limit_max == 0 ? "ACTIVE (LOW)" : "open (HIGH)");
}

/**
 * @brief Process serial command
 */
void processCommand(const char* cmd) {
    if (cmd == nullptr || strlen(cmd) == 0) {
        return;
    }
    
    // Convert to lowercase for comparison
    char cmdLower[256];
    strncpy(cmdLower, cmd, sizeof(cmdLower) - 1);
    cmdLower[sizeof(cmdLower) - 1] = '\0';
    for (int i = 0; cmdLower[i]; i++) {
        cmdLower[i] = tolower(cmdLower[i]);
    }
    
    if (g_gantry == nullptr && strcmp(cmdLower, "help") != 0) {
        LOG_PRINTLN("ERROR: Gantry not initialized");
     return;
   }
    
    if (strcmp(cmdLower, "help") == 0 || strcmp(cmdLower, "?") == 0) {
        printHelp();
    } else if (strcmp(cmdLower, "status") == 0) {
        printStatus();
    } else if (strcmp(cmdLower, "limits") == 0) {
        printLimits();
    } else if (strcmp(cmdLower, "enable") == 0) {
        g_gantry->enable();
        LOG_PRINTLN("OK Motors enabled");
    } else if (strcmp(cmdLower, "disable") == 0) {
        g_gantry->disable();
        LOG_PRINTLN("OK Motors disabled");
    } else if (strcmp(cmdLower, "home") == 0) {
        ESP_LOGI(TAG, "Starting homing sequence...");
        g_gantry->home();
        while (g_gantry->isBusy()) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        ESP_LOGI(TAG, "OK Homing complete");
    } else if (strcmp(cmdLower, "calibrate") == 0) {
        ESP_LOGI(TAG, "Starting calibration...");
        int len = g_gantry->calibrate();
        if (len > 0) {
            ESP_LOGI(TAG, "OK Calibrated length: %d mm", len);
   } else {
            LOG_PRINTLN("ERROR: Calibration failed");
        }
    } else if (strncmp(cmdLower, "move", 4) == 0) {
        float x = 0.0f, y = 0.0f, theta = 0.0f;
        int parsed = sscanf(cmd, "move %f %f %f", &x, &y, &theta);
        if (parsed < 3) {
            LOG_PRINTLN("ERROR: Usage: move <x_mm> <y_mm> <theta_deg>");
            return;
        }
        
        Gantry::JointConfig target;
        target.x = x;
        target.y = y;
        target.theta = theta;
        
        ESP_LOGI(TAG, "Moving to: x=%.1f y=%.1f theta=%.1f", x, y, theta);
        Gantry::GantryError result = g_gantry->moveTo(target, 50, 30);
        if (result == Gantry::GantryError::OK) {
            LOG_PRINTLN("OK Move command accepted");
            while (g_gantry->isBusy()) {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            LOG_PRINTLN("OK Move complete");
   } else {
            ESP_LOGE(TAG, "ERROR: Move failed: %d", (int)result);
        }
    } else if (strncmp(cmdLower, "grip", 4) == 0) {
        int value = 0;
        int parsed = sscanf(cmd, "grip %d", &value);
        if (parsed < 1) {
            LOG_PRINTLN("ERROR: Usage: grip <0|1>");
            return;
        }
        g_gantry->grip(value != 0);
        ESP_LOGI(TAG, "OK Gripper %s", value ? "closed" : "opened");
    } else if (strcmp(cmdLower, "stop") == 0) {
        g_gantry->disable();
        g_gantry->enable();  // Re-enable to reset state
        LOG_PRINTLN("OK Stop requested");
    } else {
        ESP_LOGE(TAG, "ERROR: Unknown command: %s", cmd);
        ESP_LOGI(TAG, "Type 'help' for available commands");
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
    mcp_config.device_address = 0x00;  // A0=A1=A2=GND (address 0x20)
    mcp_config.clock_speed_hz = 10000000;  // 10 MHz
    
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
        
        // Input pins with pull-ups
        gpio_expander_set_direction(PIN_LIMIT_MIN, false);
        gpio_expander_set_pullup(PIN_LIMIT_MIN, true);
        gpio_expander_set_direction(PIN_LIMIT_MAX, false);
        gpio_expander_set_pullup(PIN_LIMIT_MAX, true);
        
        // Initialize outputs to safe states
        gpio_expander_write(PIN_PULSE, 0);
        gpio_expander_write(PIN_DIR, 0);
        gpio_expander_write(PIN_ENABLE, 0);  // Disabled
        gpio_expander_write(PIN_Y_STEP, 0);
        gpio_expander_write(PIN_Y_DIR, 0);
        gpio_expander_write(PIN_Y_ENABLE, 0);  // Disabled
        gpio_expander_write(PIN_GRIPPER, 0);  // Open
        gpio_expander_write(PIN_LED, 0);
        
        ESP_LOGI(TAG, "MCP23S17 pins configured");
    }
    
    // ========================================================================
    // Configure X-axis servo driver
    // ========================================================================
    BergerdaServo::DriverConfig xConfig;
    
    // Pin configuration
    // NOTE: SDF08NK8X library expects direct GPIO pin numbers.
    // For MCP23S17 support, library modifications are required.
    // Currently using MCP23S17 pin numbers (0-15) - library needs GPIO abstraction layer.
    xConfig.output_pin_nos[6] = PIN_PULSE;   // PULSE (MCP23S17 PA0)
    xConfig.output_pin_nos[7] = PIN_DIR;     // DIR (MCP23S17 PA1)
    xConfig.output_pin_nos[0] = PIN_ENABLE;  // ENABLE (MCP23S17 PA2)
    
    // Encoder pins (must stay on direct ESP32 GPIO for PCNT)
    xConfig.input_pin_nos[3] = PIN_ENC_A;    // Encoder A+ (ESP32 GPIO 35)
    xConfig.input_pin_nos[4] = PIN_ENC_B;    // Encoder B+ (ESP32 GPIO 36)
    xConfig.enable_encoder_feedback = true;
    xConfig.pcnt_unit = PCNT_UNIT_0;
    
    // Limit switch pins (MCP23S17 PA3, PA4)
    // NOTE: Library needs modification to use gpio_expander_read() instead of gpio_get_level()
    xConfig.limit_min_pin = PIN_LIMIT_MIN;   // MCP23S17 PA3
    xConfig.limit_max_pin = PIN_LIMIT_MAX;   // MCP23S17 PA4
    xConfig.homing_speed_pps = 6000;
    
    // Motor parameters
    xConfig.encoder_ppr = 6000;
    xConfig.max_pulse_freq = 10000;
    xConfig.max_speed_pps = 10000;
    xConfig.max_accel_pps2 = 5000;
    
    // Control mode
    xConfig.pulse_mode = BergerdaServo::PulseMode::PULSE_DIRECTION;
    xConfig.control_mode = BergerdaServo::ControlMode::POSITION;
    
    // Create gantry instance
    // NOTE: Gripper pin is on MCP23S17 PB0 (pin 8)
    // Library needs modification to use gpio_expander_write() instead of digitalWrite()
    static Gantry::Gantry gantry(xConfig, PIN_GRIPPER);  // MCP23S17 PB0
    g_gantry = &gantry;
    
    // Configure limit switches
    gantry.setLimitPins(PIN_LIMIT_MIN, PIN_LIMIT_MAX);
    
    // Configure Y-axis stepper
    // NOTE: GantryAxisStepper library expects direct GPIO pin numbers.
    // For MCP23S17 support, library modifications are required.
    // Currently using MCP23S17 pin numbers (5-7) - library needs GPIO abstraction layer.
    gantry.setYAxisPins(PIN_Y_STEP, PIN_Y_DIR, PIN_Y_ENABLE);  // MCP23S17 PA5, PA6, PA7
    gantry.setYAxisStepsPerMm(200.0f);  // 200 steps/mm (adjust to your setup)
    gantry.setYAxisLimits(0.0f, 200.0f);  // 0-200mm travel
    gantry.setYAxisMotionLimits(100.0f, 500.0f, 500.0f);  // Max speed, accel, decel
    
    // Configure theta servo
    gantry.setThetaServo(PIN_THETA_PWM, 0);  // LEDC channel 0
    gantry.setThetaLimits(-90.0f, 90.0f);
    gantry.setThetaPulseRange(1000, 2000);  // Standard servo range
    
    // Set safe height
    gantry.setSafeYHeight(150.0f);  // Safe height for X-axis travel
    
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
    // Gantry update task - Core 1, high priority
  BaseType_t result = xTaskCreatePinnedToCore(
    gantryUpdateTask,
    "GantryUpdate",
        4096,
        NULL,
        5,
    NULL,
        1  // Core 1
  );
  if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create Gantry Update task!");
  }

    // Serial command task - Core 0, low priority
  result = xTaskCreatePinnedToCore(
        serialTask,
        "SerialCmd",
        4096,
    NULL,
        1,
    NULL,
        0  // Core 0
  );
  if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create Serial task!");
    }
    
    ESP_LOGI(TAG, "All tasks created successfully");
    ESP_LOGI(TAG, "System ready");
    ESP_LOGI(TAG, "");
    printHelp();
    
    // Main task can exit - FreeRTOS tasks handle everything
  vTaskDelete(NULL);
 }
 
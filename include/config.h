#ifndef CONFIG_H
#define CONFIG_H

#include <ETH.h>

// ========== Logging Helper Macros ==========
#define LOG_INFO(tag, format, ...) \
    Serial.printf("[%6lu][Core%d][%s] " format "\n", (unsigned long)millis(), \
                  (int)xPortGetCoreID(), tag, ##__VA_ARGS__)

#define LOG_WARN(tag, format, ...) \
    Serial.printf("[%6lu][Core%d][%s] WARN: " format "\n", (unsigned long)millis(), \
                  (int)xPortGetCoreID(), tag, ##__VA_ARGS__)

#define LOG_ERROR(tag, format, ...) \
    Serial.printf("[%6lu][Core%d][%s] ERROR: " format "\n", (unsigned long)millis(), \
                  (int)xPortGetCoreID(), tag, ##__VA_ARGS__)
static const char *TAG_SYS = "System";
static const char *TAG_ETH = "Ethernet";
static const char *TAG_MQTT = "MQTT";
static const char *TAG_GANTRY = "Gantry";
// ========== FreeRTOS Task Configuration ==========
#define GANTRY_TASK_STACK_SIZE 4096
#define GANTRY_TASK_PRIORITY 5
#define GANTRY_TASK_CORE 1

#define MQTT_TASK_STACK_SIZE 4096
#define MQTT_TASK_PRIORITY 1
#define MQTT_TASK_CORE 0

#define BLINK_TASK_STACK_SIZE 1024
#define BLINK_TASK_PRIORITY 1
#define BLINK_TASK_CORE 0

#define GANTRY_UPDATE_TASK_STACK_SIZE 2048
#define GANTRY_UPDATE_TASK_PRIORITY 3
#define GANTRY_UPDATE_TASK_CORE 1

#define COMMAND_QUEUE_SIZE 10
#define GANTRY_UPDATE_INTERVAL_MS 10

// ========== System Constants ==========
#define LED_PIN 15                      // Heartbeat LED pin
#define INTERCEPT_ITERATIONS 3          // Number of iterations for intercept calculation
#define HEARTBEAT_INTERVAL_MS 5000      // MQTT heartbeat publish interval

// ========== Pin Definitions (WT32-ETH01 Safe) ==========
#define PIN_SERVO_PULSE 2       // Output
#define PIN_SERVO_DIR 12         // Output
#define PIN_SERVO_ENABLE 39     // Output
#define PIN_LIMIT_MIN 33        // Input Pullup (X Home)
#define PIN_LIMIT_MAX 5        // Input Pullup (X End)
#define PIN_ALARM 17            // Output
#define PIN_ENC_A 35            // Input Only
#define PIN_ENC_B 36            // Input Only
#define PIN_GRIPPER 32          // Output for Gripper

// ========== MQTT Configuration ==========
#define MQTT_HOST "192.168.2.1"
#define MQTT_PORT 1883
#define MQTT_CLIENT_ID "esp32-ether-01"
#define MQTT_PUB_TOPIC "lab/outbox"
#define MQTT_SUB_TOPIC "lab/gantry/cmd"

// ---------- Command Layout ----------
struct GantryCommand {
    int32_t x;                          // Item X position when detected (mm)
    int32_t y;                          // Target Y position (mm)
    int32_t theta;                      // Target theta angle (degrees)
    uint32_t speed;                     // Gantry speed (mm/s)
    float conveyor_speed_mm_per_s;      // Conveyor belt speed (mm/s)
    uint32_t detection_timestamp_ms;    // When item was detected
    uint32_t grip_delay_ms;             // Gripper actuation time (ms)
    bool home;
    bool calibrate;
  };

// ========== Pick-and-Place Configuration ==========
#define PICK_ZONE_MIN_X_MM 0.0f        // Start of reachable pick window
#define PICK_ZONE_MAX_X_MM 400.0f      // End of reachable pick window
#define DEFAULT_GRIP_DELAY_MS 100      // Default gripper delay (was 500ms)
#define MAX_GANTRY_SPEED_MM_S 500      // Maximum gantry speed
#define DEFAULT_ACCEL_MM_S2 2000       // Default acceleration
#define DEFAULT_DCCEL_MM_S2 2000       // Default deceleration

// ---------- FreeRTOS Handles ----------
QueueHandle_t commandQueue = nullptr;
TaskHandle_t gantryTaskHandle = nullptr;
TaskHandle_t mqttTaskHandle = nullptr;
TaskHandle_t blinkTaskHandle = nullptr;
TaskHandle_t gantryUpdateTaskHandle = nullptr;
TimerHandle_t mqttReconnectTimer = nullptr;
// ---------- Network Config ----------
const IPAddress local_IP(192, 168, 2, 2);
const IPAddress gateway(192, 168, 2, 1);
const IPAddress subnet(255, 255, 255, 0);
const IPAddress dns(8, 8, 8, 8);

#endif // CONFIG_H

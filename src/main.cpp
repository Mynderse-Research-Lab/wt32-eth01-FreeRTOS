#include "Gantry.h"
#include <AsyncMqttClient.h>
#include <ETH.h>
#include <Ticker.h>
#include <driver/pcnt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/timers.h>
#include <cmath>  // For fabs()

using Gantry::Gantry;

// ---------- Logging Helper ----------
#define LOG_INFO(tag, format, ...)                                             \
  Serial.printf("[%6lu][Core%d][%s] " format "\n", millis(), xPortGetCoreID(), \
                tag, ##__VA_ARGS__)
#define LOG_WARN(tag, format, ...)                                             \
  Serial.printf("[%6lu][Core%d][%s] WARN: " format "\n", millis(),             \
                xPortGetCoreID(), tag, ##__VA_ARGS__)
#define LOG_ERROR(tag, format, ...)                                            \
  Serial.printf("[%6lu][Core%d][%s] ERROR: " format "\n", millis(),            \
                xPortGetCoreID(), tag, ##__VA_ARGS__)

static const char *TAG_SYS = "System";
static const char *TAG_ETH = "Ethernet";
static const char *TAG_MQTT = "MQTT";
static const char *TAG_GANTRY = "Gantry";

// ---------- FreeRTOS Task Configuration ----------
#define GANTRY_TASK_STACK_SIZE     4096
#define GANTRY_TASK_PRIORITY       5
#define GANTRY_TASK_CORE           1

#define MQTT_TASK_STACK_SIZE       4096
#define MQTT_TASK_PRIORITY         1
#define MQTT_TASK_CORE             0

#define BLINK_TASK_STACK_SIZE      1024
#define BLINK_TASK_PRIORITY        1
#define BLINK_TASK_CORE            0

#define GANTRY_UPDATE_TASK_STACK_SIZE 2048
#define GANTRY_UPDATE_TASK_PRIORITY  3
#define GANTRY_UPDATE_TASK_CORE      1

#define COMMAND_QUEUE_SIZE         10
#define GANTRY_UPDATE_INTERVAL_MS   10

// ---------- Pin Definitions (WT32-ETH01 Safe) ----------
#define PIN_SERVO_PULSE 2   // Output
#define PIN_SERVO_DIR 4     // Output
#define PIN_SERVO_ENABLE 12 // Output
#define PIN_ENC_A 35        // Input Only
#define PIN_ENC_B 36        // Input Only
#define PIN_GRIPPER 33      // Output for Gripper
#define PIN_LIMIT_MIN 14    // Input Pullup (X Home)
#define PIN_LIMIT_MAX 32    // Input Pullup (X End)

// ---------- MQTT Config ----------
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

// ---------- Pick-and-Place Configuration ----------
#define PICK_ZONE_MIN_X_MM      50.0f    // Start of reachable pick window
#define PICK_ZONE_MAX_X_MM      400.0f   // End of reachable pick window
#define DEFAULT_GRIP_DELAY_MS   100      // Default gripper delay (was 500ms)
#define MAX_GANTRY_SPEED_MM_S   500      // Maximum gantry speed
#define DEFAULT_ACCEL_MM_S2     2000     // Default acceleration

// ---------- FreeRTOS Handles ----------
QueueHandle_t commandQueue = nullptr;
TaskHandle_t gantryTaskHandle = nullptr;
TaskHandle_t mqttTaskHandle = nullptr;
TaskHandle_t blinkTaskHandle = nullptr;
TaskHandle_t gantryUpdateTaskHandle = nullptr;
TimerHandle_t mqttReconnectTimer = nullptr;

// ---------- Network Config ----------
IPAddress local_IP(192, 168, 2, 2);
IPAddress gateway(192, 168, 2, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(8, 8, 8, 8);

AsyncMqttClient mqttClient;
volatile bool netUp = false;
Ticker mqttReconnectTicker;  // Keep Ticker for compatibility with AsyncMqttClient

// ---------- Objects ----------
BergerdaServo::DriverConfig xConfig;
Gantry::Gantry *gantrySystem = nullptr;

// ---------- MQTT Functions ----------
void onMqttConnect(bool sessionPresent) {
  LOG_INFO(TAG_MQTT, "Connected. Session: %d", sessionPresent);
  mqttClient.subscribe(MQTT_SUB_TOPIC, 1);
  mqttClient.publish(MQTT_PUB_TOPIC, 1, true, "online");
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  LOG_WARN(TAG_MQTT, "Disconnected (Reason: %d)", (int)reason);
  if (netUp) {
    // Use Ticker for compatibility with AsyncMqttClient
    mqttReconnectTicker.once(2, []() { mqttClient.connect(); });
  }
}

void onMqttMessage(char *topic, char *payload,
                   AsyncMqttClientMessageProperties properties, size_t len,
                   size_t index, size_t total) {
  // Basic Parsing: Expecting JSON-like string
  // "x:1000,y:200,t:90,s:5000,h:1,c:1"
  char buf[128];
  size_t copyLen = (len < sizeof(buf) - 1) ? len : (sizeof(buf) - 1);
  memcpy(buf, payload, copyLen);
  buf[copyLen] = '\0';

  LOG_INFO(TAG_MQTT, "Msg Received: %s", buf);

  // Initialize with defaults
  GantryCommand cmd = {
    .x = 0,
    .y = 0,
    .theta = 0,
    .speed = 200,  // Default 200 mm/s
    .conveyor_speed_mm_per_s = 0.0f,
    .detection_timestamp_ms = millis(),  // Default to now
    .grip_delay_ms = DEFAULT_GRIP_DELAY_MS,
    .home = false,
    .calibrate = false
  };

  // Parse command fields
  // Format: "x:100,y:50,t:0,s:200,v:50,g:100,h:0,c:0"
  char *xPtr = strstr(buf, "x:");
  if (xPtr)
    cmd.x = atoi(xPtr + 2);

  char *yPtr = strstr(buf, "y:");
  if (yPtr)
    cmd.y = atoi(yPtr + 2);

  char *tPtr = strstr(buf, "t:");
  if (tPtr)
    cmd.theta = atoi(tPtr + 2);

  char *sPtr = strstr(buf, "s:");
  if (sPtr)
    cmd.speed = atoi(sPtr + 2);
  
  // Conveyor velocity (v:)
  char *vPtr = strstr(buf, "v:");
  if (vPtr)
    cmd.conveyor_speed_mm_per_s = atof(vPtr + 2);
  
  // Grip delay in ms (g:)
  char *gPtr = strstr(buf, "g:");
  if (gPtr)
    cmd.grip_delay_ms = atoi(gPtr + 2);

  char *hPtr = strstr(buf, "h:");
  if (hPtr)
    cmd.home = (atoi(hPtr + 2) == 1);

  char *cPtr = strstr(buf, "c:");
  if (cPtr)
    cmd.calibrate = (atoi(cPtr + 2) == 1);

  // Enqueue command
  // For real-time tracking, prioritize NEW commands over OLD queued ones
  if (commandQueue) {
    // If queue is full, clear it to make room for latest position data
    if (uxQueueSpacesAvailable(commandQueue) == 0) {
      LOG_WARN(TAG_MQTT, "Queue full - clearing stale commands");
      xQueueReset(commandQueue);
    }
    
    if (xQueueSend(commandQueue, &cmd, 0) != pdTRUE) {
      LOG_ERROR(TAG_MQTT, "Queue send failed!");
    } else {
      LOG_INFO(TAG_MQTT, "Queued: x=%d v=%.1f", cmd.x, cmd.conveyor_speed_mm_per_s);
    }
  }
}

void EthEvent(WiFiEvent_t event) {
  switch (event) {
  case ARDUINO_EVENT_ETH_START:
    ETH.setHostname("esp32-eth");
    break;
  case ARDUINO_EVENT_ETH_CONNECTED:
    LOG_INFO(TAG_ETH, "Link Up");
    break;
  case ARDUINO_EVENT_ETH_GOT_IP:
    LOG_INFO(TAG_ETH, "IP: %s", ETH.localIP().toString().c_str());
    netUp = true;
    mqttClient.connect();
    break;
  case ARDUINO_EVENT_ETH_DISCONNECTED:
    LOG_WARN(TAG_ETH, "Link Down");
    netUp = false;
    mqttReconnectTicker.detach();
    break;
  default:
    break;
  }
}

// ============================================================================
// Conveyor Interception Helper
// ============================================================================

/**
 * @brief Calculate intercept position for moving conveyor
 * 
 * The item moves on the conveyor while the gantry travels to it.
 * We need to predict where the item WILL BE when the gantry arrives.
 * 
 * @param item_x_mm Item position when detected (mm)
 * @param conveyor_speed Conveyor belt speed (mm/s, positive = increasing X)
 * @param latency_ms Time since detection (ms)
 * @param gantry_x Current gantry position (mm)
 * @param gantry_speed Gantry travel speed (mm/s)
 * @return Intercept X position (mm), or -1 if unreachable
 */
float calculateInterceptPosition(float item_x_mm, float conveyor_speed,
                                  uint32_t latency_ms, float gantry_x,
                                  float gantry_speed) {
    // Compensate for network/processing latency
    float item_current_x = item_x_mm + conveyor_speed * (latency_ms / 1000.0f);
    
    // If conveyor is stationary, target current position
    if (fabs(conveyor_speed) < 0.1f) {
        return item_current_x;
    }
    
    // Iterative interception calculation (converges in 2-3 iterations)
    float intercept_x = item_current_x;
    for (int i = 0; i < 3; i++) {
        float distance = fabs(intercept_x - gantry_x);
        float travel_time = distance / gantry_speed;  // seconds
        intercept_x = item_current_x + conveyor_speed * travel_time;
    }
    
    // Check if intercept is within pick zone
    if (intercept_x < PICK_ZONE_MIN_X_MM || intercept_x > PICK_ZONE_MAX_X_MM) {
        LOG_WARN(TAG_GANTRY, "Intercept %.1f out of pick zone [%.1f, %.1f]",
                 intercept_x, PICK_ZONE_MIN_X_MM, PICK_ZONE_MAX_X_MM);
        return -1.0f;  // Unreachable
    }
    
    return intercept_x;
}

// ============================================================================
// FreeRTOS Tasks
// ============================================================================

/**
 * @brief Blink LED task - Heartbeat indicator
 * @param param Task parameter (unused)
 */
void blinkTask(void *param) {
  (void)param;  // Unused parameter
  
  pinMode(15, OUTPUT);
  const TickType_t delayTicks = pdMS_TO_TICKS(500);
  
  while (1) {
    digitalWrite(15, !digitalRead(15));
    vTaskDelay(delayTicks);
  }
  
  // Should never reach here
  vTaskDelete(NULL);
}

/**
 * @brief Gantry command processing task
 * @param param Task parameter (unused)
 * 
 * Processes commands from MQTT queue and executes gantry operations.
 * Runs on Core 1 with high priority for real-time motion control.
 */
void gantryTask(void *param) {
  (void)param;  // Unused parameter
  
  LOG_INFO(TAG_GANTRY, "Task Started");

  // 1. Configure X Axis Pins
  xConfig.output_pin_nos[6] = PIN_SERVO_PULSE;
  xConfig.output_pin_nos[7] = PIN_SERVO_DIR;
  xConfig.output_pin_nos[0] = PIN_SERVO_ENABLE;
  xConfig.input_pin_nos[3] = PIN_ENC_A;
  xConfig.input_pin_nos[4] = PIN_ENC_B;
  xConfig.enable_encoder_feedback = true;
  xConfig.pcnt_unit = PCNT_UNIT_0;
  
  // Configure limit switch pins in driver config
  xConfig.limit_min_pin = PIN_LIMIT_MIN;
  xConfig.limit_max_pin = PIN_LIMIT_MAX;

  // 2. Initialize Gantry
  static Gantry::Gantry gantry(xConfig, PIN_GRIPPER);
  gantry.setLimitPins(PIN_LIMIT_MIN, PIN_LIMIT_MAX);
  gantrySystem = &gantry;

  if (!gantry.begin()) {
    LOG_ERROR(TAG_GANTRY, "Init Failed!");
    vTaskDelete(NULL);
     return;
   }
  LOG_INFO(TAG_GANTRY, "Initialized");

  gantry.enable();

  GantryCommand cmd;
  const TickType_t busyCheckDelay = pdMS_TO_TICKS(GANTRY_UPDATE_INTERVAL_MS);

  // 3. Command Processing Loop
  while (1) {
    // Block until command available (infinite timeout)
    if (xQueueReceive(commandQueue, &cmd, portMAX_DELAY) == pdTRUE) {

      if (cmd.home) {
        LOG_INFO(TAG_GANTRY, "Executing Homing...");
        gantry.home();
        
        // Wait for homing to complete
        while (gantry.isBusy()) {
          vTaskDelay(busyCheckDelay);
        }
        LOG_INFO(TAG_GANTRY, "Homing Complete");
        
      } else if (cmd.calibrate) {
        LOG_INFO(TAG_GANTRY, "Executing Calibration...");
        int len = gantry.calibrate();
        if (len > 0) {
          LOG_INFO(TAG_GANTRY, "Calibrated Length: %d mm", len);
   } else {
          LOG_ERROR(TAG_GANTRY, "Calibration Failed");
        }
        
      } else {
        // ============================================================
        // PICK-AND-PLACE FROM MOVING CONVEYOR
        // ============================================================
        
        // Check for alarm before movement
        if (gantry.isAlarmActive()) {
          LOG_ERROR(TAG_GANTRY, "Alarm active - movement blocked");
          continue;
        }
        
        // Calculate latency since item was detected
        uint32_t latency_ms = millis() - cmd.detection_timestamp_ms;
        
        // Get current gantry position (in mm)
        float gantry_x_mm = gantry.getXEncoder() / gantry.getPulsesPerMm();
        
        // Calculate intercept position for moving conveyor
        float target_x_mm;
        if (cmd.conveyor_speed_mm_per_s != 0.0f) {
          // Moving conveyor - calculate interception point
          target_x_mm = calculateInterceptPosition(
            (float)cmd.x,
            cmd.conveyor_speed_mm_per_s,
            latency_ms,
            gantry_x_mm,
            (float)cmd.speed
          );
          
          if (target_x_mm < 0) {
            LOG_WARN(TAG_GANTRY, "Item unreachable - skipping");
            continue;
          }
          
          LOG_INFO(TAG_GANTRY, "Intercept: item@%.1f + %.1fmm/s -> target=%.1f (latency=%ums)",
                   (float)cmd.x, cmd.conveyor_speed_mm_per_s, target_x_mm, latency_ms);
   } else {
          // Static target
          target_x_mm = (float)cmd.x;
          LOG_INFO(TAG_GANTRY, "Static target: X=%.1f Y=%d T=%d Speed=%u",
                   target_x_mm, cmd.y, cmd.theta, cmd.speed);
        }
        
        // Clamp speed to maximum
        uint32_t actual_speed = cmd.speed;
        if (actual_speed > MAX_GANTRY_SPEED_MM_S) {
          actual_speed = MAX_GANTRY_SPEED_MM_S;
        }
        
        // Execute move to intercept position
        gantry.moveTo((int32_t)target_x_mm, cmd.y, cmd.theta, actual_speed);

        // Wait for motion to complete
        while (gantry.isBusy()) {
          vTaskDelay(busyCheckDelay);
        }

        // Grip sequence with configurable delay
        if (cmd.grip_delay_ms > 0) {
          LOG_INFO(TAG_GANTRY, "Gripping (delay=%ums)...", cmd.grip_delay_ms);
          gantry.grip(true);
          vTaskDelay(pdMS_TO_TICKS(cmd.grip_delay_ms));
          gantry.grip(false);
        }
      }
      LOG_INFO(TAG_GANTRY, "Cmd Done");
    }
  }
  
  // Should never reach here
  vTaskDelete(NULL);
}

/**
 * @brief Gantry update task - Calls gantry.update() periodically
 * @param param Task parameter (unused)
 * 
 * Runs continuously to update gantry state (limit debouncing, etc.)
 * Runs on Core 1 with medium priority.
 */
void gantryUpdateTask(void *param) {
  (void)param;  // Unused parameter
  
  const TickType_t updateInterval = pdMS_TO_TICKS(GANTRY_UPDATE_INTERVAL_MS);
  
  // Wait for gantry system to be initialized
  while (gantrySystem == nullptr) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  
  LOG_INFO(TAG_GANTRY, "Update Task Started");
  
  while (1) {
    if (gantrySystem != nullptr) {
      gantrySystem->update();
    }
    vTaskDelay(updateInterval);
  }
  
  // Should never reach here
  vTaskDelete(NULL);
}

/**
 * @brief MQTT loop task - Publishes status periodically
 * @param param Task parameter (unused)
 * 
 * Publishes heartbeat and gantry status to MQTT broker.
 * Runs on Core 0 with low priority.
 */
void mqttLoopTask(void *param) {
  (void)param;  // Unused parameter
  
  const TickType_t publishInterval = pdMS_TO_TICKS(5000);
  static uint32_t count = 0;
  
  while (1) {
    if (netUp && mqttClient.connected()) {
      // Heartbeat message
      char msg[64];
      snprintf(msg, sizeof(msg), "Count: %u", count++);
      mqttClient.publish(MQTT_PUB_TOPIC, 0, false, msg);

      // Gantry status message
      if (gantrySystem != nullptr) {
        char sMsg[128];
        snprintf(sMsg, sizeof(sMsg), "X:%d Y:%d Th:%d Busy:%d Alarm:%d",
                 gantrySystem->getXEncoder(), 
                 gantrySystem->getCurrentY(),
                 gantrySystem->getCurrentTheta(),
                 gantrySystem->isBusy() ? 1 : 0,
                 gantrySystem->isAlarmActive() ? 1 : 0);
        mqttClient.publish("lab/gantry", 0, false, sMsg);
      }
    }
    vTaskDelay(publishInterval);
  }
  
  // Should never reach here
  vTaskDelete(NULL);
}

// ============================================================================
// Setup / Loop
// ============================================================================

/**
 * @brief Arduino setup function - Initializes system and creates FreeRTOS tasks
 */
void setup() {
  Serial.begin(115200);
  delay(1000);  // Allow serial to stabilize
  LOG_INFO(TAG_SYS, "Booting...");

  // 1. Create Command Queue
  commandQueue = xQueueCreate(COMMAND_QUEUE_SIZE, sizeof(GantryCommand));
  if (commandQueue == NULL) {
    LOG_ERROR(TAG_SYS, "Queue Create Failed!");
    // Continue anyway - queue creation failure is logged
  } else {
    LOG_INFO(TAG_SYS, "Command queue created (size: %d)", COMMAND_QUEUE_SIZE);
  }

  // 2. Start Ethernet
  WiFi.onEvent(EthEvent);
  ETH.begin(1, 16, 23, 18, ETH_PHY_LAN8720, ETH_CLOCK_GPIO17_OUT);
  ETH.config(local_IP, gateway, subnet, dns);
  LOG_INFO(TAG_SYS, "Ethernet initialized");

  // 3. Setup MQTT
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setClientId(MQTT_CLIENT_ID);
  LOG_INFO(TAG_SYS, "MQTT client configured");

  // 4. Create FreeRTOS Tasks
  // Gantry task - High priority, Core 1 (real-time motion control)
  BaseType_t result = xTaskCreatePinnedToCore(
    gantryTask,
    "Gantry",
    GANTRY_TASK_STACK_SIZE,
    NULL,
    GANTRY_TASK_PRIORITY,
    &gantryTaskHandle,
    GANTRY_TASK_CORE
  );
  if (result != pdPASS) {
    LOG_ERROR(TAG_SYS, "Failed to create Gantry task!");
  }

  // Gantry update task - Medium priority, Core 1 (periodic updates)
  result = xTaskCreatePinnedToCore(
    gantryUpdateTask,
    "GantryUpdate",
    GANTRY_UPDATE_TASK_STACK_SIZE,
    NULL,
    GANTRY_UPDATE_TASK_PRIORITY,
    &gantryUpdateTaskHandle,
    GANTRY_UPDATE_TASK_CORE
  );
  if (result != pdPASS) {
    LOG_ERROR(TAG_SYS, "Failed to create Gantry Update task!");
  }

  // MQTT loop task - Low priority, Core 0 (network I/O)
  result = xTaskCreatePinnedToCore(
    mqttLoopTask,
    "MqttLoop",
    MQTT_TASK_STACK_SIZE,
    NULL,
    MQTT_TASK_PRIORITY,
    &mqttTaskHandle,
    MQTT_TASK_CORE
  );
  if (result != pdPASS) {
    LOG_ERROR(TAG_SYS, "Failed to create MQTT task!");
  }

  // Blink task - Low priority, Core 0 (heartbeat LED)
  result = xTaskCreatePinnedToCore(
    blinkTask,
    "Blink",
    BLINK_TASK_STACK_SIZE,
    NULL,
    BLINK_TASK_PRIORITY,
    &blinkTaskHandle,
    BLINK_TASK_CORE
  );
  if (result != pdPASS) {
    LOG_ERROR(TAG_SYS, "Failed to create Blink task!");
  }

  LOG_INFO(TAG_SYS, "All tasks created successfully");
  LOG_INFO(TAG_SYS, "System ready");
}

/**
 * @brief Arduino loop function
 * 
 * In FreeRTOS-based applications, loop() is typically empty or minimal.
 * All work is done in FreeRTOS tasks.
 */
void loop() {
  // FreeRTOS tasks handle all functionality
  // This function should not block or perform heavy work
  // Delete this task to free up resources
  vTaskDelete(NULL);
 }
 
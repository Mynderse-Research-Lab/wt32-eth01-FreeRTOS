#include "Gantry.h"
#include <AsyncMqttClient.h>
#include <ETH.h>
#include <Ticker.h>
#include <driver/pcnt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/timers.h>

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
  int32_t x;
  int32_t y;
  int32_t theta;
  uint32_t speed;
  bool home;
  bool calibrate;
};

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

  GantryCommand cmd = {0, 0, 0, 5000, false, false}; // Defaults

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

  char *hPtr = strstr(buf, "h:");
  if (hPtr)
    cmd.home = (atoi(hPtr + 2) == 1);

  char *cPtr = strstr(buf, "c:");
  if (cPtr)
    cmd.calibrate = (atoi(cPtr + 2) == 1);

  // Enqueue
  if (commandQueue) {
    if (xQueueSend(commandQueue, &cmd, 0) != pdTRUE) {
      LOG_ERROR(TAG_MQTT, "Queue Full! Dropped command.");
    } else {
      LOG_INFO(TAG_MQTT, "QueuedCmd");
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
  const TickType_t gripDelay = pdMS_TO_TICKS(500);

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
        // Normal Move
        LOG_INFO(TAG_GANTRY, "Moving: X=%d Y=%d T=%d Speed=%u", 
                 cmd.x, cmd.y, cmd.theta, cmd.speed);
        
        // Check for alarm before movement
        if (gantry.isAlarmActive()) {
          LOG_ERROR(TAG_GANTRY, "Alarm active - movement blocked");
        } else {
          gantry.moveTo(cmd.x, cmd.y, cmd.theta, cmd.speed);

          // Wait for motion to complete
          while (gantry.isBusy()) {
            vTaskDelay(busyCheckDelay);
          }

          // Simple Pick Sequence (gripper open/close)
          // Only if speed > 0 (indicates active movement command)
          if (cmd.speed > 0) {
            LOG_INFO(TAG_GANTRY, "Gripping...");
            gantry.grip(true);
            vTaskDelay(gripDelay);
            gantry.grip(false);
          }
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

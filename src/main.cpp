#include "Gantry.h"
#include <ETH.h>
#include <Ticker.h>
#include <driver/pcnt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/timers.h>
#include <cmath>  // For fabs()
#include <inttypes.h>  // For PRId32, PRIu32
#include "config.h"

// ESP-IDF MQTT Client
#include "mqtt_client.h"

using Gantry::Gantry;

// ESP-IDF MQTT client handle
esp_mqtt_client_handle_t mqttClient = nullptr;
volatile bool netUp = false;
volatile bool mqttConnected = false;
Ticker mqttReconnectTicker;

// ---------- Objects ----------
BergerdaServo::DriverConfig xConfig;
Gantry::Gantry *gantrySystem = nullptr;

// ---------- MQTT Message Handler ----------
static void handleMqttMessage(const char *data, int data_len) {
  // Basic Parsing: Expecting JSON-like string
  // "x:1000,y:200,t:90,s:5000,h:1,c:1"
  char buf[128];
  size_t copyLen = (data_len < (int)sizeof(buf) - 1) ? data_len : (sizeof(buf) - 1);
  memcpy(buf, data, copyLen);
  buf[copyLen] = '\0';
  
  // Warn if message was truncated
  if (data_len >= (int)sizeof(buf)) {
    LOG_WARN(TAG_MQTT, "Message truncated from %d to %d bytes", data_len, (int)sizeof(buf) - 1);
  }

  LOG_INFO(TAG_MQTT, "Msg Received: %s", buf);

  // Initialize with defaults
  uint32_t timestamp_ms = millis();  // Cache timestamp once
  GantryCommand cmd = {
    .x = 0,
    .y = 0,
    .theta = 0,
    .speed = 200,  // Default 200 mm/s
    .conveyor_speed_mm_per_s = 0.0f,
    .detection_timestamp_ms = timestamp_ms,
    .grip_delay_ms = DEFAULT_GRIP_DELAY_MS,
    .home = false,
    .calibrate = false
  };

  // Optimized single-pass parsing
  // Format: "x:100,y:50,t:0,s:200,v:50,g:100,h:0,c:0"
  char *token = strtok(buf, ",");
  while (token != NULL) {
    // Parse each key:value pair
    if (token[0] && token[1] == ':') {
      int32_t value = atoi(token + 2);
      switch (token[0]) {
        case 'x': cmd.x = value; break;
        case 'y': cmd.y = value; break;
        case 't': cmd.theta = value; break;
        case 's': cmd.speed = value; break;
        case 'v': cmd.conveyor_speed_mm_per_s = atof(token + 2); break;
        case 'g': cmd.grip_delay_ms = value; break;
        case 'h': cmd.home = (value == 1); break;
        case 'c': cmd.calibrate = (value == 1); break;
        default: break;  // Ignore unknown parameters
      }
    }
    token = strtok(NULL, ",");
  }

  // Enqueue command with FIFO behavior
  // For real-time tracking, prioritize NEW commands over OLD queued ones
  if (commandQueue) {
    // If queue is full, drop oldest command to make room (FIFO)
    if (uxQueueSpacesAvailable(commandQueue) == 0) {
      GantryCommand old_cmd;
      if (xQueueReceive(commandQueue, &old_cmd, 0) == pdTRUE) {
        LOG_WARN(TAG_MQTT, "Queue full - dropped oldest: x=%" PRId32, old_cmd.x);
      }
    }
    
    if (xQueueSend(commandQueue, &cmd, 0) != pdTRUE) {
      LOG_ERROR(TAG_MQTT, "Queue send failed!");
    } else {
      LOG_INFO(TAG_MQTT, "Queued: x=%" PRId32 " v=%.1f", cmd.x, cmd.conveyor_speed_mm_per_s);
    }
  }
}

// ---------- ESP-IDF MQTT Event Handler ----------
static void mqttEventHandler(void *handler_args, esp_event_base_t base, 
                              int32_t event_id, void *event_data) {
  esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
  
  switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
      LOG_INFO(TAG_MQTT, "Connected to broker");
      mqttConnected = true;
      // Subscribe to command topic
      esp_mqtt_client_subscribe(mqttClient, MQTT_SUB_TOPIC, 1);
      // Publish online status
      esp_mqtt_client_publish(mqttClient, MQTT_PUB_TOPIC, "online", 0, 1, 1);
      break;
      
    case MQTT_EVENT_DISCONNECTED:
      LOG_WARN(TAG_MQTT, "Disconnected from broker");
      mqttConnected = false;
      // Reconnect will be handled automatically by esp_mqtt_client
      break;
      
    case MQTT_EVENT_SUBSCRIBED:
      LOG_INFO(TAG_MQTT, "Subscribed to %s", MQTT_SUB_TOPIC);
      break;
      
    case MQTT_EVENT_DATA:
      // Handle incoming message
      if (event->data_len > 0) {
        handleMqttMessage(event->data, event->data_len);
      }
      break;
      
    case MQTT_EVENT_ERROR:
      LOG_ERROR(TAG_MQTT, "MQTT error occurred");
      if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
        LOG_ERROR(TAG_MQTT, "TCP transport error");
      }
      break;
      
    default:
      break;
  }
}

// ---------- MQTT Client Initialization ----------
static void initMqttClient() {
  char uri[64];
  snprintf(uri, sizeof(uri), "mqtt://%s:%d", MQTT_HOST, MQTT_PORT);
  
  esp_mqtt_client_config_t mqtt_cfg = {};
  mqtt_cfg.broker.address.uri = uri;
  mqtt_cfg.credentials.client_id = MQTT_CLIENT_ID;
  mqtt_cfg.network.reconnect_timeout_ms = 2000;
  mqtt_cfg.session.keepalive = 60;
  
  mqttClient = esp_mqtt_client_init(&mqtt_cfg);
  if (mqttClient == nullptr) {
    LOG_ERROR(TAG_MQTT, "Failed to create MQTT client");
    return;
  }
  
  esp_mqtt_client_register_event(mqttClient, MQTT_EVENT_ANY, mqttEventHandler, nullptr);
  LOG_INFO(TAG_MQTT, "MQTT client initialized");
}

static void startMqttClient() {
  if (mqttClient != nullptr) {
    esp_mqtt_client_start(mqttClient);
    LOG_INFO(TAG_MQTT, "MQTT client started");
  }
}

void EthEvent(arduino_event_id_t event) {
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
    // Start MQTT client when network is ready
    startMqttClient();
    break;
  case ARDUINO_EVENT_ETH_DISCONNECTED:
    LOG_WARN(TAG_ETH, "Link Down");
    netUp = false;
    mqttConnected = false;
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
    // Validate gantry speed to prevent division by zero
    if (gantry_speed <= 0.0f) {
        LOG_ERROR(TAG_GANTRY, "Invalid gantry speed: %.1f", gantry_speed);
        return -1.0f;  // Error condition
    }
    
    // Compensate for network/processing latency
    float item_current_x = item_x_mm + conveyor_speed * (latency_ms / 1000.0f);
    
    // If conveyor is stationary, target current position
    if (fabs(conveyor_speed) < 0.1f) {
        return item_current_x;
    }
    
    // Iterative interception calculation (converges in 2-3 iterations)
    float intercept_x = item_current_x;
    for (int i = 0; i < INTERCEPT_ITERATIONS; i++) {
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
  
  pinMode(LED_PIN, OUTPUT);
  const TickType_t delayTicks = pdMS_TO_TICKS(500);
  
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
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
          
          LOG_INFO(TAG_GANTRY, "Intercept: item@%.1f + %.1fmm/s -> target=%.1f (latency=%" PRIu32 "ms)",
                   (float)cmd.x, cmd.conveyor_speed_mm_per_s, target_x_mm, latency_ms);
   } else {
          // Static target
          target_x_mm = (float)cmd.x;
          LOG_INFO(TAG_GANTRY, "Static target: X=%.1f Y=%" PRId32 " T=%" PRId32 " Speed=%" PRIu32,
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
          LOG_INFO(TAG_GANTRY, "Gripping (delay=%" PRIu32 "ms)...", cmd.grip_delay_ms);
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
  
  const TickType_t publishInterval = pdMS_TO_TICKS(HEARTBEAT_INTERVAL_MS);
  static uint32_t count = 0;
  
  while (1) {
    if (netUp && mqttConnected && mqttClient != nullptr) {
      // Heartbeat message
      char msg[64];
      snprintf(msg, sizeof(msg), "Count: %lu", (unsigned long)count++);
      esp_mqtt_client_publish(mqttClient, MQTT_PUB_TOPIC, msg, 0, 0, 0);

      // Gantry status message
      if (gantrySystem != nullptr) {
        char sMsg[128];
        snprintf(sMsg, sizeof(sMsg), "X:%d Y:%d Th:%d Busy:%d Alarm:%d",
                 gantrySystem->getXEncoder(), 
                 gantrySystem->getCurrentY(),
                 gantrySystem->getCurrentTheta(),
                 gantrySystem->isBusy() ? 1 : 0,
                 gantrySystem->isAlarmActive() ? 1 : 0);
        esp_mqtt_client_publish(mqttClient, "lab/gantry", sMsg, 0, 0, 0);
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
  Network.onEvent(EthEvent);
  ETH.begin(ETH_PHY_LAN8720, ETH_ADDR, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_PHY_POWER, ETH_CLK_MODE);
  ETH.config(local_IP, gateway, subnet, dns);
  LOG_INFO(TAG_SYS, "Ethernet initialized");

  // 3. Setup MQTT (ESP-IDF native client)
  initMqttClient();
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
 
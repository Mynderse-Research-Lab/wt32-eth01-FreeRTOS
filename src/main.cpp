#include "SDF08NK8X.h"
#include <AsyncMqttClient.h>
#include <ETH.h>
#include <Ticker.h>

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
static const char *TAG_MOTION = "Motion";

// ---------- Pin Definitions (WT32-ETH01 Safe) ----------
// Avoid: 16, 17, 18, 23 (MDC/MDIO/CLK/PWR)
// Avoid: 19, 21, 22, 25, 26, 27 (EMAC/RMII)
// Available: 2, 4, 12, 14, 15, 32, 33, 35(I), 36(I), 39(I)

#define PIN_SERVO_PULSE 2   // Output
#define PIN_SERVO_DIR 4     // Output
#define PIN_SERVO_ENABLE 12 // Output
#define PIN_ENC_A 35        // Input Only
#define PIN_ENC_B 36        // Input Only

// ---------- MQTT Config ----------
#define MQTT_HOST "192.168.2.1"
#define MQTT_PORT 1883
#define MQTT_CLIENT_ID "esp32-ether-01"
#define MQTT_PUB_TOPIC "lab/outbox"

// ---------- Network Config ----------
IPAddress local_IP(192, 168, 2, 2);
IPAddress gateway(192, 168, 2, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(8, 8, 8, 8);

AsyncMqttClient mqttClient;
bool netUp = false;
Ticker mqttReconnectTimer;
Ticker ethRetryTimer;

// ---------- Objects ----------
BergerdaServo::DriverConfig servoConfig;
BergerdaServo::ServoDriver *servoDriver = nullptr;

// ---------- MQTT Functions ----------
void onMqttConnect(bool sessionPresent) {
  LOG_INFO(TAG_MQTT, "Connected. Session: %d", sessionPresent);
  mqttClient.publish(MQTT_PUB_TOPIC, 1, true, "online");
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  LOG_WARN(TAG_MQTT, "Disconnected (Reason: %d)", (int)reason);
  if (netUp)
    mqttReconnectTimer.once(2, []() { mqttClient.connect(); });
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
    mqttReconnectTimer.detach();
    break;
  default:
    break;
  }
}

// ---------- FreeRTOS Tasks ----------

void blinkTask(void *param) {
  pinMode(15,
          OUTPUT); // WT32-ETH01 often has no onboard LED, using IO15 as status
  while (1) {
    digitalWrite(15, !digitalRead(15));
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void motionTask(void *param) {
  LOG_INFO(TAG_MOTION, "Task Started");

  // 1. Configure Safe Pins
  servoConfig.output_pin_nos[6] = PIN_SERVO_PULSE;
  servoConfig.output_pin_nos[7] = PIN_SERVO_DIR;
  servoConfig.output_pin_nos[0] = PIN_SERVO_ENABLE;

  // Encoder (Input Only pins are fine)
  servoConfig.input_pin_nos[3] = PIN_ENC_A;
  servoConfig.input_pin_nos[4] = PIN_ENC_B;
  servoConfig.enable_encoder_feedback = true;
  servoConfig.pcnt_unit = PCNT_UNIT_0;

  // 2. Initialize Driver
  static BergerdaServo::ServoDriver driver(servoConfig);
  servoDriver = &driver; // Export pointer if needed globally

  if (!driver.initialize()) {
    LOG_ERROR(TAG_MOTION, "Driver Init Failed!");
    vTaskDelete(NULL);
  }
  LOG_INFO(TAG_MOTION, "Driver Initialized");

  // 3. Enable & Loop
  driver.enable();

  while (1) {
    static bool toggle = false;
    uint32_t target_pos = toggle ? 10000 : 0;
    toggle = !toggle;

    LOG_INFO(TAG_MOTION, "Moving to %u", target_pos);
    driver.moveToPosition(target_pos, 8000, 3000, 3000);

    // Blocking wait for move to complete (sampling feedback)
    while (driver.isMotionActive()) {
      BergerdaServo::DriveStatus status = driver.getStatus();
      // Logging here can be spammy, maybe skip or throttle
      // LOG_INFO(TAG_MOTION, "Pos:%u", status.current_position);
      vTaskDelay(pdMS_TO_TICKS(100));
    }

    LOG_INFO(TAG_MOTION, "Move Complete. Enc: %d", driver.getEncoderPosition());
    vTaskDelay(pdMS_TO_TICKS(2000)); // Hold for 2s
  }
}

void mqttLoopTask(void *param) {
  while (1) {
    if (netUp && mqttClient.connected()) {
      // Periodic publish example
      static uint32_t count = 0;
      char msg[64];
      snprintf(msg, sizeof(msg), "Count: %u", count++);
      mqttClient.publish(MQTT_PUB_TOPIC, 0, false, msg);

      // Also publish servo status if available
      if (servoDriver) {
        BergerdaServo::DriveStatus s = servoDriver->getStatus();
        char sMsg[64];
        snprintf(sMsg, sizeof(sMsg), "Pos:%u Enc:%d", s.current_position,
                 s.encoder_position);
        mqttClient.publish("lab/servo", 0, false, sMsg);
        LOG_INFO(TAG_MQTT, "Published status: %s", sMsg);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5000)); // Publish every 5s
  }
}

// ---------- Setup / Loop ----------

void setup() {
  Serial.begin(115200);
  LOG_INFO(TAG_SYS, "Booting...");

  // 1. Start Ethernet
  WiFi.onEvent(EthEvent);
  ETH.begin(1, 16, 23, 18, ETH_PHY_LAN8720, ETH_CLOCK_GPIO17_OUT);
  ETH.config(local_IP, gateway, subnet, dns);

  // 2. Setup MQTT
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setClientId(MQTT_CLIENT_ID);

  // 3. Create Tasks
  // Motion Task on Core 1 (App Core) - High Priority, Deterministic
  xTaskCreatePinnedToCore(motionTask, "Motion", 4096, NULL, 5, NULL, 1);

  // MQTT & Blink on Core 0 (Pro Core) - Handles WiFi/ETH stack
  xTaskCreatePinnedToCore(mqttLoopTask, "MqttLoop", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(blinkTask, "Blink", 1024, NULL, 1, NULL, 0);

  LOG_INFO(TAG_SYS, "Tasks created. Core pinning applied.");
}

void loop() {
  // Main loop unused in FreeRTOS
  vTaskDelete(NULL);
}

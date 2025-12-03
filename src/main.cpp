#include <ETH.h>
#include <AsyncMqttClient.h>
#include <Ticker.h>


unsigned long lastPublish = 0;

// ---------- MQTT (Mosquitto) ----------
#define MQTT_HOST        "192.168.2.1"
#define MQTT_PORT        1883             // 1883 (no TLS), 8883 (TLS)
#define MQTT_USERNAME    ""               // set to "" if anonymous
#define MQTT_PASSWORD    ""               // set to "" if anonymous
#define MQTT_CLIENT_ID   "esp32-ether-01"
#define MQTT_SUB_TOPIC   "lab/inbox"
#define MQTT_PUB_TOPIC   "lab/outbox"

// Static IP (no router/DHCP)
IPAddress local_IP(192, 168, 2, 2);   // ESP32's static IP
IPAddress gateway(192, 168, 2, 1);    // set to PC/broker if no router
IPAddress subnet(255, 255, 255, 0);   // subnet mask
IPAddress dns(8, 8, 8, 8);            // DNS (not needed if using raw IPs)

// ---------- Reconnect timers ----------
Ticker mqttReconnectTimer;
Ticker ethRetryTimer;

AsyncMqttClient mqttClient;
bool netUp = false;

// ---------- MQTT callbacks ----------
void onMqttConnect(bool sessionPresent) {
  Serial.println("[MQTT] Connected.");
  mqttClient.subscribe(MQTT_SUB_TOPIC, 1);
  mqttClient.publish(MQTT_PUB_TOPIC, 1, true, "online (ethernet)");
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.printf("[MQTT] Disconnected (%d).\n", (int)reason);
  if (netUp) mqttReconnectTimer.once(2, [](){ mqttClient.connect(); });
}

void onMqttMessage(char* topic, char* payload,
                   AsyncMqttClientMessageProperties properties,
                   size_t len, size_t index, size_t total) {
  Serial.print("[MQTT] ");
  Serial.print(topic);
  Serial.print(" => ");
  for (size_t i = 0; i < len; i++) Serial.print((char)payload[i]);
  Serial.println();
}

// ---------- Ethernet events ----------
void EthEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      Serial.println("[ETH] start");
      ETH.setHostname("esp32-eth");
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.println("[ETH] link up");
      
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.printf("[ETH] IP: %s\n", ETH.localIP().toString().c_str());
      netUp = true;
      mqttClient.connect();
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("[ETH] link down");
      netUp = false;
      mqttReconnectTimer.detach();
      ethRetryTimer.once(2, [](){
        ETH.begin(
          /*phy_addr*/1, /*power*/16, /*mdc*/23, /*mdio*/18,
          ETH_PHY_LAN8720, ETH_CLOCK_GPIO17_OUT
        );
        
        if (!ETH.config(local_IP, gateway, subnet, dns)) {
          Serial.println("[ETH] static IP re-config FAILED");
        } else {
          Serial.printf("[ETH] static IP re-set: %s\n", ETH.localIP().toString().c_str());
        }
      });
      break;
    case ARDUINO_EVENT_ETH_STOP:
      Serial.println("[ETH] stop");
      netUp = false;
      break;
    default: break;
  }
}

void setup() {
  Serial.begin(115200);

  // ETH event hook
  WiFi.onEvent(EthEvent);

  // MQTT setup
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  if (strlen(MQTT_USERNAME)) mqttClient.setCredentials(MQTT_USERNAME, MQTT_PASSWORD);
  mqttClient.setClientId(MQTT_CLIENT_ID);
  mqttClient.setKeepAlive(30);   // seconds
  mqttClient.setCleanSession(true);
  mqttClient.setWill(MQTT_PUB_TOPIC, 1, true, "offline");

  // Start Ethernet (WT32-ETH01 / LAN8720 typical pins/clock)
  ETH.begin(
    /*phy_addr*/1, /*power*/16, /*mdc*/23, /*mdio*/18,
    ETH_PHY_LAN8720, ETH_CLOCK_GPIO17_OUT
  );

  // apply static IP right after begin (fixes config error 20487)
  delay(50); // tiny settle
  if (!ETH.config(local_IP, gateway, subnet, dns)) {
    Serial.println("[ETH] static IP config FAILED");
  } else {
    Serial.printf("[ETH] static IP set: %s\n", ETH.localIP().toString().c_str());
  }


}

void loop() {
   

  // Publish every 5 seconds
  if (millis() - lastPublish > 1000) {
    lastPublish = millis();
    if (mqttClient.connected()) {
      static int count = 0;
      count = count + 1;
      String msg = "Hello from loop() with OTA#" + String(count);
      mqttClient.publish("lab/outbox", 0, false, msg.c_str());
      Serial.println("[MQTT] Published: Hello from loop()");
    } else {
      Serial.println("[MQTT] Not connected, skipping publish");
    }
  }
}

#include <Arduino.h>
#include <ETH.h>
#include <PubSubClient.h> //-- Ensure libs are added to PlatformIO file -- //

// ----- Required for WT32-ETH01 -----
#define ETH_ADDR        1 // PHY Address on the RMII bus
#define ETH_POWER_PIN   5 // GPIO pin used to enable power to LAN8720 chip
#define ETH_MDC_PIN     23 // MDC Management Data Clock
#define ETH_MDIO_PIN    18 // MDIO Management Data IO
#define ETH_TYPE        ETH_PHY_LAN8720 // Type of Ethernet PHY
#define ETH_CLK_MODE    ETH_CLOCK_GPIO17_OUT // 50 MHz clock required by LAN8720
// -- Make sure you have these var are in the platformIO.init file-- they are
// required before compile time -- ///

WiFiClient ethClient; // Making a Ethernet Client
PubSubClient mqttClient(ethClient); //mqtt client using ethernet

const char* mqtt_server = "10.0.0.25"; // The mqtt broker's IP
const int mqtt_port = 1883; // Default port

void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      Serial.println("ETH Started");
      ETH.setHostname("wt32-eth01");
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.println("ETH Connected");
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.print("ETH IP Address: ");
      Serial.println(ETH.localIP());
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("ETH Disconnected");
      break;
    case ARDUINO_EVENT_ETH_STOP:
      Serial.println("ETH Stopped");
      break;
    default:
      break;
  }
}

void reconnectMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (mqttClient.connect("WT32Client")) {
      Serial.println("connected");
      mqttClient.subscribe("wt32/topic"); // Optional
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      delay(2000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);  // Give the serial some time to initialize

  Serial.println("ESP32 is up and running!");
  
  // --- Building the Ethernet Connection -- //
  WiFi.onEvent(WiFiEvent); //Register Ethernet Event handler

  ETH.begin(
    ETH_ADDR,
    ETH_POWER_PIN,
    ETH_MDC_PIN,
    ETH_MDIO_PIN,
    ETH_TYPE,
    ETH_CLK_MODE
  );    
  
  mqttClient.setServer(mqtt_server,mqtt_port);

}

void loop() {
  if (!mqttClient.connected()){
    reconnectMQTT();
  }

  mqttClient.loop();  // Keep connection alive

  // Publish a message every 5 seconds
  static unsigned long lastMsg = 0;
  if (millis() - lastMsg > 5000) {
    lastMsg = millis();
    mqttClient.publish("wt32/topic", "Hello from WT32-ETH01");
    Serial.println("Message sent!");
  }

Serial.print("Current ETH IP: ");
Serial.println(ETH.localIP());

}


#include <ETH.h>
#include <PubSubClient.h>
#include "config.hpp"

// MQTT client
WiFiClient ethClient;
PubSubClient client(ethClient);

// Event: Print IP once obtained
void WiFiEvent(WiFiEvent_t event) {
  if (event == ARDUINO_EVENT_ETH_GOT_IP) {
    Serial.print("IP: ");
    Serial.println(ETH.localIP());
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("WT32Client")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" retrying in 2s");
      delay(2000);
    }
  }
}

void setup() {

  // Setting up serial comms
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting ETH...");

  // Setting power pins to turn on the LAN8720A chip
  pinMode(ETH_POWER_PIN, OUTPUT);
  digitalWrite(ETH_POWER_PIN, HIGH);
  delay(100);


  WiFi.onEvent(WiFiEvent);

  // Setting up ethernet comms
  ETH.begin(ETH_ADDR, 
            ETH_POWER_PIN,
            ETH_MDC_PIN, 
            ETH_MDIO_PIN, 
            ETH_TYPE, 
            ETH_CLK_MODE
  );
  
  IPAddress local_ip(192, 168, 2, 2);
  IPAddress gateway(192, 168, 2, 1); 
  IPAddress subnet(255, 255, 255, 0);
  ETH.config(local_ip, gateway, subnet);

  client.setServer(MQTT_SERVER, MQTT_PORT);
}

void loop() {
  
  if (!client.connected()) {
    reconnect();
  }

  client.loop();

  // Send a message every 5 seconds
  static unsigned long lastMsg = 0;
  if (millis() - lastMsg > 5000) {
    lastMsg = millis();
    String msg = "hello from WT32";
    Serial.print("Publishing: ");
    Serial.println(msg);
    client.publish(MQTT_TOPIC_TEST, msg.c_str());
  }
}



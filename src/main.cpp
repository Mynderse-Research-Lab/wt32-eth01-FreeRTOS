#include <ETH.h>
#include <PubSubClient.h>
#include "config.hpp"
#include "mqtt_utils.hpp"

// MQTT client
WiFiClient ethClient;
PubSubClient client(ethClient);

void setup() {

  // Setting up serial comms
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting ETH...");

  enableLAN8720A();

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
  // checks connection
  if (!client.connected()) {
    reconnect();
  }

  // Maintains the connection to the broker-- call often
  client.loop();

  // Send a message every 5 seconds
  sanityCheck();

}



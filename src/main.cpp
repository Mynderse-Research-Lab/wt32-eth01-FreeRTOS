#include <ETH.h>

#define ETH_ADDR        1
#define ETH_POWER_PIN   16   // Set only once
#define ETH_MDC_PIN     23
#define ETH_MDIO_PIN    18
#define ETH_TYPE        ETH_PHY_LAN8720
#define ETH_CLK_MODE    ETH_CLOCK_GPIO17_OUT

void WiFiEvent(WiFiEvent_t event) {
  if (event == ARDUINO_EVENT_ETH_GOT_IP) {
    Serial.print("IP: ");
    Serial.println(ETH.localIP());
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting ETH...");

  pinMode(ETH_POWER_PIN, OUTPUT);
  digitalWrite(ETH_POWER_PIN, HIGH);
  delay(100);

  WiFi.onEvent(WiFiEvent);  // Register event callback

  // Initialize Ethernet driver first
  ETH.begin(
    ETH_ADDR,
    ETH_POWER_PIN,
    ETH_MDC_PIN,
    ETH_MDIO_PIN,
    ETH_TYPE,
    ETH_CLK_MODE
  );

  // Now assign static IP configuration
  IPAddress local_ip(192, 168, 2, 2);
  IPAddress gateway(192, 168, 2, 1);  // Your laptop
  IPAddress subnet(255, 255, 255, 0);
  ETH.config(local_ip, gateway, subnet);
}


void loop() {
  delay(1000);
}


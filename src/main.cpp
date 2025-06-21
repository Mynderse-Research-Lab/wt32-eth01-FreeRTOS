#include <Arduino.h>
#include <ETH.h>

void WiFiEvent(WiFiEvent_t event) {
  if (event == SYSTEM_EVENT_ETH_GOT_IP) {
    Serial.print("Got IP: ");
    Serial.println(ETH.localIP());
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);  // Give the serial some time to initialize

  Serial.println("ESP32 is up and running!");
  
  WiFi.onEvent(WiFiEvent);  // Register after Serial.begin
  ETH.begin();              // Initialize Ethernet
}

void loop() {
  delay(1000);
}


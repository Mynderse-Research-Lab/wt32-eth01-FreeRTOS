#pragma once

#include <ETH.h>
#include <WiFi.h>
#include <PubSubClient.h>

extern PubSubClient client;

// Event Handler for Ethernet IP
void WiFiEvent(WiFiEvent_t event);

// Reconnect logic for MQTT
void reconnect();

void connectionTest();

void enableLAN8720A();
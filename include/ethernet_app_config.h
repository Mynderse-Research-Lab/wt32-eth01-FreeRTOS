#ifndef ETHERNET_APP_CONFIG_H
#define ETHERNET_APP_CONFIG_H

// Lab example (ADSB-PI-base broker.env):
//   Broker Pi:  192.168.1.5   (MQTT_BROKER_URI_DEFAULT in mqtt_topics.h)
//   WT32:       192.168.1.100
//   Netmask:    255.255.255.0
//   Gateway:    192.168.1.5   (Pi on same switch; no router required)

#define ETH_USE_STATIC_IP           1

#define ETH_STATIC_IP_OCTET_1       192
#define ETH_STATIC_IP_OCTET_2       168
#define ETH_STATIC_IP_OCTET_3       1
#define ETH_STATIC_IP_OCTET_4       100

#define ETH_STATIC_GW_OCTET_1       192
#define ETH_STATIC_GW_OCTET_2       168
#define ETH_STATIC_GW_OCTET_3       1
#define ETH_STATIC_GW_OCTET_4       5

#define ETH_STATIC_NETMASK_OCTET_1  255
#define ETH_STATIC_NETMASK_OCTET_2  255
#define ETH_STATIC_NETMASK_OCTET_3  255
#define ETH_STATIC_NETMASK_OCTET_4  0

// How long start() waits for a usable IP (static or DHCP).
#define ETH_IP_WAIT_TIMEOUT_MS      30000

#endif  // ETHERNET_APP_CONFIG_H

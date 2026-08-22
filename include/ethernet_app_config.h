#ifndef ETHERNET_APP_CONFIG_H
#define ETHERNET_APP_CONFIG_H

/**
 * @file ethernet_app_config.h
 * @brief LAN8720 plant Ethernet + TCP console settings.
 *
 * Firmware: values come from idf.py menuconfig
 *   - "LAN8720 plant Ethernet"
 *   - "TCP gantry console (LAN8720)"
 * Host builds without sdkconfig.h keep the defaults below.
 */

#include "gantry_kconfig.h"

/* ---- LAN8720 addressing ------------------------------------------------- */

#if defined(CONFIG_ETH_USE_STATIC_IP)
#define ETH_USE_STATIC_IP 1
#else
#define ETH_USE_STATIC_IP 0
#endif

#ifdef CONFIG_ETH_STATIC_IP
#define ETH_STATIC_IP CONFIG_ETH_STATIC_IP
#else
#define ETH_STATIC_IP "192.168.1.100"
#endif

#ifdef CONFIG_ETH_STATIC_GATEWAY
#define ETH_STATIC_GATEWAY CONFIG_ETH_STATIC_GATEWAY
#else
#define ETH_STATIC_GATEWAY "192.168.1.5"
#endif

#ifdef CONFIG_ETH_STATIC_NETMASK
#define ETH_STATIC_NETMASK CONFIG_ETH_STATIC_NETMASK
#else
#define ETH_STATIC_NETMASK "255.255.255.0"
#endif

#ifdef CONFIG_ETH_IP_WAIT_TIMEOUT_MS
#define ETH_IP_WAIT_TIMEOUT_MS CONFIG_ETH_IP_WAIT_TIMEOUT_MS
#else
#define ETH_IP_WAIT_TIMEOUT_MS 3000
#endif

/* ---- TCP gantry console ------------------------------------------------- */

#if defined(CONFIG_CONSOLE_TCP_ENABLE)
#define CONSOLE_TCP_ENABLE 1
#else
#define CONSOLE_TCP_ENABLE 0
#endif

#if defined(CONFIG_CONSOLE_UART_ENABLE)
#define CONSOLE_UART_ENABLE 1
#else
#define CONSOLE_UART_ENABLE 0
#endif

#ifdef CONFIG_CONSOLE_TCP_PORT
#define CONSOLE_TCP_PORT CONFIG_CONSOLE_TCP_PORT
#else
#define CONSOLE_TCP_PORT 2323
#endif

#if defined(CONFIG_CONSOLE_TCP_AUTH_ENABLE)
#define CONSOLE_TCP_AUTH_ENABLE 1
#else
#define CONSOLE_TCP_AUTH_ENABLE 0
#endif

#ifdef CONFIG_CONSOLE_TCP_PASSWORD
#define CONSOLE_TCP_PASSWORD CONFIG_CONSOLE_TCP_PASSWORD
#else
#define CONSOLE_TCP_PASSWORD "LTU_1932"
#endif

#ifdef CONFIG_CONSOLE_TCP_AUTH_MAX_TRIES
#define CONSOLE_TCP_AUTH_MAX_TRIES CONFIG_CONSOLE_TCP_AUTH_MAX_TRIES
#else
#define CONSOLE_TCP_AUTH_MAX_TRIES 3
#endif

#ifdef CONFIG_CONSOLE_TCP_AUTH_REMEMBER_S
#define CONSOLE_TCP_AUTH_REMEMBER_S CONFIG_CONSOLE_TCP_AUTH_REMEMBER_S
#else
#define CONSOLE_TCP_AUTH_REMEMBER_S 600
#endif

#ifdef CONFIG_CONSOLE_TCP_AUTH_REMEMBER_MAX
#define CONSOLE_TCP_AUTH_REMEMBER_MAX CONFIG_CONSOLE_TCP_AUTH_REMEMBER_MAX
#else
#define CONSOLE_TCP_AUTH_REMEMBER_MAX 4
#endif

/* ---- LAN ESP_LOG stream (TCP tee) --------------------------------------- */

#if defined(CONFIG_CONSOLE_TCP_LOG_ENABLE)
#define CONSOLE_TCP_LOG_ENABLE 1
#else
#define CONSOLE_TCP_LOG_ENABLE 0
#endif

/** Severity thresholds for filtering teed ESP_LOG lines (lower = stricter). */
#define CONSOLE_TCP_LOG_SEV_ERR  1
#define CONSOLE_TCP_LOG_SEV_WARN 2
#define CONSOLE_TCP_LOG_SEV_INFO 3
#define CONSOLE_TCP_LOG_SEV_DBUG 4

#if defined(CONFIG_CONSOLE_TCP_LOG_LEVEL_ERR)
#define CONSOLE_TCP_LOG_LEVEL CONSOLE_TCP_LOG_SEV_ERR
#elif defined(CONFIG_CONSOLE_TCP_LOG_LEVEL_WARN)
#define CONSOLE_TCP_LOG_LEVEL CONSOLE_TCP_LOG_SEV_WARN
#elif defined(CONFIG_CONSOLE_TCP_LOG_LEVEL_DBUG)
#define CONSOLE_TCP_LOG_LEVEL CONSOLE_TCP_LOG_SEV_DBUG
#else
#define CONSOLE_TCP_LOG_LEVEL CONSOLE_TCP_LOG_SEV_INFO
#endif

#endif  // ETHERNET_APP_CONFIG_H

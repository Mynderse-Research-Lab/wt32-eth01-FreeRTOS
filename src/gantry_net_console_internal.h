/**
 * @file gantry_net_console_internal.h
 * @brief Declarations shared by the gantry_net_console*.cpp files that make
 *        up the TCP console. Private to src/; include it only from those files.
 */
#pragma once

#include "ethernet_app_config.h"

#include <cstddef>
#include <cstdint>

[[maybe_unused]] static const char *TAG = "NetConsole";

#if CONSOLE_TCP_ENABLE

namespace GantryNetConsoleImpl {

constexpr size_t kLineCap = 256;

// gantry_net_console.cpp
bool sendAll(int fd, const char *text, bool wait);

// gantry_net_console_auth.cpp
bool authenticateClient(int fd, uint32_t peer_addr_nbo);

}  // namespace GantryNetConsoleImpl

#endif  // CONSOLE_TCP_ENABLE

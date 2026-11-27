#ifndef GANTRY_NET_CONSOLE_H
#define GANTRY_NET_CONSOLE_H

#include "gantry_test_console.h"

/**
 * Start the LAN8720 TCP line console (port CONSOLE_TCP_PORT).
 * Call after EthernetLink has IP. Single client; second connect is rejected.
 * No-op when CONSOLE_TCP_ENABLE is 0.
 */
void gantryNetConsoleStart(const GantryTestConsoleConfig *cfg);

/** One line of TCP send counters since boot (netcon). */
void gantryNetConsoleStats(char *out, size_t out_len);

/**
 * Called by a command that wants to restart. From the NetConsole task it
 * returns true and the console sends the prompt, closes the client and then
 * restarts. From any other task it returns false and the caller restarts.
 */
bool gantryNetConsoleDeferRestart(void);

#endif  // GANTRY_NET_CONSOLE_H

/**
 * @file gantry_console_commands.cpp
 * @brief Gantry test console: command dispatch.
 *
 * One of the gantry_console_*.cpp files that make up the test console;
 * gantry_console_internal.h declares what they share.
 */

#include "gantry_console_internal.h"

namespace GantryConsoleImpl {

namespace {

using Cfg = GantryTestConsoleConfig;
using Match = GantryConsoleMatch;
using ConsoleCommand = GantryConsoleCommand;

constexpr size_t kMaxRegisteredTables = 4;
const GantryConsoleCommand* g_registered[kMaxRegisteredTables] = {};
size_t g_registeredCount[kMaxRegisteredTables] = {};
size_t g_registeredTables = 0;

bool commandMatches(const ConsoleCommand &c, const char *cmdLower) {
  const size_t n = strlen(c.word);
  switch (c.match) {
    case Match::kExact:
      return strcmp(cmdLower, c.word) == 0;
    case Match::kPrefix:
      return strncmp(cmdLower, c.word, n) == 0;
    case Match::kWord:
      return strncmp(cmdLower, c.word, n) == 0 &&
             (cmdLower[n] == '\0' || cmdLower[n] == ' ');
    case Match::kWordWs:
      return strncmp(cmdLower, c.word, n) == 0 &&
             (cmdLower[n] == '\0' || cmdLower[n] == ' ' || cmdLower[n] == '\t');
  }
  return false;
}

// Commands that may run while test_cycle or test_theta_path is running.
bool allowedWhileTestCycleRuns(const char *cmdLower) {
  return !(strcmp(cmdLower, "stop") != 0 &&
           strcmp(cmdLower, "status") != 0 &&
           strcmp(cmdLower, "calibrated") != 0 &&
           strcmp(cmdLower, "faults") != 0 &&
           strcmp(cmdLower, "alarms") != 0 &&
           strcmp(cmdLower, "eiptiming") != 0 &&
           strcmp(cmdLower, "gaptrace") != 0 &&
           strcmp(cmdLower, "bootlog") != 0 &&
           strncmp(cmdLower, "ethlink", 7) != 0 &&
           strcmp(cmdLower, "limits") != 0 &&
           strcmp(cmdLower, "puuinfo") != 0 &&
           strncmp(cmdLower, "puucal", 6) != 0 &&
           strncmp(cmdLower, "puu ", 4) != 0 &&
           strncmp(cmdLower, "thetalim", 8) != 0 &&
           strncmp(cmdLower, "speed", 5) != 0 &&
           strncmp(cmdLower, "accel", 5) != 0 &&
           strncmp(cmdLower, "rangelimit", 10) != 0 &&
           strncmp(cmdLower, "livepos", 7) != 0 &&
           strncmp(cmdLower, "axislog", 7) != 0);
}

void cmdBootTime(const GantryTestConsoleConfig *, const char *, const char *) {
  const Network::EthernetBootTimes bt = Network::ethernetBootTimes();
  ESP_LOGI(TAG, "boottime reset=%d (ms since app start, -1 = not reached): eth_start=%lld "
                "eth_started=%lld phy_link_up=%lld got_ip=%lld",
           static_cast<int>(esp_reset_reason()),
           bt.start_us < 0 ? -1LL : static_cast<long long>(bt.start_us / 1000),
           bt.eth_started_us < 0 ? -1LL : static_cast<long long>(bt.eth_started_us / 1000),
           bt.link_up_us < 0 ? -1LL : static_cast<long long>(bt.link_up_us / 1000),
           bt.got_ip_us < 0 ? -1LL : static_cast<long long>(bt.got_ip_us / 1000));
}

void cmdFlashStall(const GantryTestConsoleConfig *cfg, const char *, const char *cmdLower) {
  char mode[8] = {};
  int writes = 96;
  const int got = sscanf(cmdLower + 11, "%7s %d", mode, &writes);
  const bool erase = strcmp(mode, "erase") == 0;
  char line[200];
  if (got < 1 || (!erase && strcmp(mode, "write") != 0)) {
    ESP_LOGE(TAG, "usage: flashstall erase|write [n]");
    return;
  }
  if (gantryOtaFlashStallTest(cfg->gantry, erase, writes, line, sizeof(line)) != ESP_OK) {
    ESP_LOGE(TAG, "ERROR: %s", line);
    return;
  }
  ESP_LOGI(TAG, "%s", line);
}

void cmdNetCon(const GantryTestConsoleConfig *, const char *, const char *) {
  char line[480];
  gantryNetConsoleStats(line, sizeof(line));
  ESP_LOGI(TAG, "%s", line);
}

void cmdBootLog(const GantryTestConsoleConfig *, const char *, const char *) {
  ESP_LOGI(TAG, "=== boot log (%u bytes captured from reset) ===",
           static_cast<unsigned>(bootLogRingBytes()));
  bootLogRingForEach(
      [](const char *line, void *) { ESP_LOGI(TAG, "boot| %s", line); }, nullptr);
}

// First match wins, so order matters where one word is a prefix of
// another. Registered tables are searched first: gripmark is registered
// there, ahead of grip in this table.
const ConsoleCommand kCommands[] = {
#if MCP_DEBUG_CMDS
    {"mcp_pin_mode", Match::kPrefix,
     [](const Cfg *, const char *cmd, const char *) {
       runMcpPinModeCommand(cmd);
     }},
    {"mcp_dump", Match::kPrefix,
     [](const Cfg *, const char *cmd, const char *) {
       runMcpDumpCommand(cmd);
     }},
    {"mcp_reg", Match::kPrefix,
     [](const Cfg *, const char *cmd, const char *) {
       runMcpRegCommand(cmd);
     }},
    {"field_dout", Match::kPrefix,
     [](const Cfg *, const char *cmd, const char *) {
       runFieldDoutCommand(cmd);
     }},
    {"field_din", Match::kPrefix,
     [](const Cfg *, const char *cmd, const char *) {
       runFieldDinCommand(cmd);
     }},
#endif  // MCP_DEBUG_CMDS
    {"gpio_drive", Match::kPrefix,
     [](const Cfg *, const char *cmd, const char *) {
       runGpioDriveCommand(cmd);
     }},
    {"ota", Match::kPrefix,
     [](const Cfg *, const char *, const char *) {
       gantryOtaPrintStatus();
     }},
    {"status", Match::kExact,
     [](const Cfg *cfg, const char *, const char *) {
       printStatus(cfg->gantry);
     }},
    {"calibrated", Match::kExact,
     [](const Cfg *cfg, const char *, const char *) {
       printWorkspaceCalibrated(cfg->gantry);
     }},
    {"faults", Match::kExact,
     cmdFaults},
    {"alarms", Match::kExact,
     cmdFaults},
    {"puuinfo", Match::kExact,
     [](const Cfg *cfg, const char *, const char *) {
       printPuuInfo(cfg->gantry);
     }},
    {"puucal", Match::kPrefix,
     [](const Cfg *cfg, const char *cmd, const char *) {
       runPuuCalCommand(cfg->gantry, cmd);
     }},
    {"puu ", Match::kPrefix,
     [](const Cfg *cfg, const char *cmd, const char *) {
       runPuuSetCommand(cfg->gantry, cmd);
     }},
    {"thetalim", Match::kPrefix,
     [](const Cfg *cfg, const char *cmd, const char *) {
       runThetaLimCommand(cfg->gantry, cmd);
     }},
    {"autotune", Match::kPrefix,
     [](const Cfg *cfg, const char *cmd, const char *) {
       runAutotuneCommand(cfg, cmd);
     }},
    {"ethlink", Match::kPrefix,
     [](const Cfg *cfg, const char *, const char *cmdLower) {
       runEthLinkCommand(cfg, cmdLower);
     }},
    {"drv", Match::kWord,
     [](const Cfg *cfg, const char *cmd, const char *) {
       runDrvCommand(cfg, cmd);
     }},
    {"limits", Match::kExact,
     [](const Cfg *cfg, const char *, const char *) {
       printLimits(cfg);
     }},
    {"pins", Match::kExact,
     [](const Cfg *cfg, const char *, const char *) {
       printActivePins(cfg);
     }},
    {"enable", Match::kExact,
     cmdEnable},
    {"disable", Match::kExact,
     cmdDisable},
    {"home", Match::kWordWs,
     [](const Cfg *cfg, const char *, const char *cmdLower) {
       dispatchPerAxisCommand(cfg, cmdLower, "home", runHomeForAxis);
     }},
    {"test_cycle", Match::kExact,
     cmdTestCycle},
    {"test_theta_path", Match::kExact,
     cmdTestThetaPath},
    {"calibrate", Match::kWordWs,
     [](const Cfg *cfg, const char *, const char *cmdLower) {
       dispatchPerAxisCommand(cfg, cmdLower, "calibrate", runCalibrateForAxis);
     }},
    {"speed", Match::kPrefix,
     cmdSpeed},
    {"accel", Match::kPrefix,
     cmdAccel},
    {"units", Match::kPrefix,
     cmdUnits},
    {"rangelimit", Match::kPrefix,
     cmdRangeLimit},
    {"livepos", Match::kPrefix,
     cmdLivePos},
    {"axislog", Match::kPrefix,
     cmdAxisLog},
    {"move", Match::kPrefix,
     cmdMove},
    {"boottime", Match::kExact,
     cmdBootTime},
    {"flashstall ", Match::kPrefix,
     cmdFlashStall},
    {"grip", Match::kPrefix,
     cmdGrip},
    {"netcon", Match::kExact,
     cmdNetCon},
    {"eiptiming", Match::kExact,
     [](const Cfg *, const char *, const char *) {
       eip::dumpClass1TimingStats();
     }},
    {"gaptrace", Match::kExact,
     [](const Cfg *, const char *, const char *) {
       eip::printLastClass1GapReport();
     }},
    {"bootlog", Match::kExact,
     cmdBootLog},
    {"stop", Match::kExact,
     cmdStop},
    {"alarmreset", Match::kExact,
     cmdAlarmReset},
    {"arst", Match::kExact,
     cmdAlarmReset},
};

}  // namespace

void registerConsoleCommands(const GantryConsoleCommand* table, size_t n) {
    if (table == nullptr || n == 0) {
        return;
    }
    if (g_registeredTables >= kMaxRegisteredTables) {
        ESP_LOGE(TAG, "console command registration full");
        return;
    }
    g_registered[g_registeredTables] = table;
    g_registeredCount[g_registeredTables] = n;
    ++g_registeredTables;
}

void processCommand(const GantryTestConsoleConfig *cfg, const char *cmd) {
  if (cfg == nullptr || cmd == nullptr || strlen(cmd) == 0) {
    return;
  }

  char cmdLower[256];
  strncpy(cmdLower, cmd, sizeof(cmdLower) - 1);
  cmdLower[sizeof(cmdLower) - 1] = '\0';
  for (int i = 0; cmdLower[i]; i++) {
    cmdLower[i] = static_cast<char>(tolower(cmdLower[i]));
  }

  const bool cfg_cmd = cmdTokenIs(cmd, "cfg");
  if (cfg->gantry == nullptr && strcmp(cmdLower, "help") != 0 &&
      strcmp(cmdLower, "?") != 0 && !cfg_cmd) {
    ESP_LOGE(TAG, "Gantry not initialized");
    return;
  }

  if (strcmp(cmdLower, "help") == 0 || strcmp(cmdLower, "?") == 0) {
    gantryTestPrintHelp();
    return;
  }
  if (cfg_cmd) {
    runCfgCommand(cfg, cmd);
    return;
  }
  if (g_testCycleInProgress && !allowedWhileTestCycleRuns(cmdLower)) {
    ESP_LOGE(TAG,
             "ERROR: test_cycle/test_theta_path running — 'stop' to abort, "
             "'status' to monitor");
    return;
  }
  for (size_t i = 0; i < g_registeredTables; ++i) {
    for (size_t j = 0; j < g_registeredCount[i]; ++j) {
      const ConsoleCommand& c = g_registered[i][j];
      if (commandMatches(c, cmdLower)) {
        c.run(cfg, cmd, cmdLower);
        return;
      }
    }
  }
  for (const ConsoleCommand &c : kCommands) {
    if (commandMatches(c, cmdLower)) {
      c.run(cfg, cmd, cmdLower);
      return;
    }
  }
  ESP_LOGE(TAG, "ERROR: Unknown command: %s", cmd);
  ESP_LOGI(TAG, "Type 'help' for available commands");
}

}  // namespace GantryConsoleImpl

void gantryConsoleRegisterCommands(const GantryConsoleCommand* table, size_t n) {
    GantryConsoleImpl::registerConsoleCommands(table, n);
}

/**
 * @file gantry_test_console.cpp
 * @brief Gantry test console: log tee, line entry, help and the console task.
 *
 * One of the gantry_console_*.cpp files that make up the test console;
 * gantry_console_internal.h declares what they share.
 */

#include "gantry_console_internal.h"

namespace GantryConsoleImpl {

static SemaphoreHandle_t g_consoleCmdMutex = nullptr;
// Guards the sink/reply pointers and tee install state; held for microseconds.
static SemaphoreHandle_t g_teeMutex = nullptr;

static GantryConsoleReplyFn g_replyFn = nullptr;
static void *g_replyCtx = nullptr;
static GantryConsoleReplyFn g_sessionFn = nullptr;
static void *g_sessionCtx = nullptr;
static vprintf_like_t g_prevVprintf = nullptr;
static bool g_teeInstalled = false;

static int consoleTeeVprintf(const char *fmt, va_list args);

/** ESP-IDF log letter after optional ANSI color (E/W/I/D/V). */
static char espLogLetter(const char *s) {
  while (*s == '\033') {
    ++s;
    if (*s == '[') {
      ++s;
      while (*s != '\0' && *s != 'm') {
        ++s;
      }
      if (*s == 'm') {
        ++s;
      }
    }
  }
  while (*s == ' ' || *s == '\t') {
    ++s;
  }
  const char c = *s;
  if (c == 'E' || c == 'W' || c == 'I' || c == 'D' || c == 'V') {
    return c;
  }
  return 0;
}

static int espLogSeverity(char letter) {
  switch (letter) {
    case 'E':
      return CONSOLE_TCP_LOG_SEV_ERR;
    case 'W':
      return CONSOLE_TCP_LOG_SEV_WARN;
    case 'I':
      return CONSOLE_TCP_LOG_SEV_INFO;
    case 'D':
    case 'V':
      return CONSOLE_TCP_LOG_SEV_DBUG;
    default:
      return CONSOLE_TCP_LOG_SEV_INFO;
  }
}

static bool lanLogAllowsLine(const char *line) {
  if (!CONSOLE_TCP_LOG_ENABLE) {
    return false;
  }
  return espLogSeverity(espLogLetter(line)) <= CONSOLE_TCP_LOG_LEVEL;
}

static void installTeeIfNeeded() {
  if (g_teeInstalled) {
    return;
  }
  g_prevVprintf = esp_log_set_vprintf(consoleTeeVprintf);
  g_teeInstalled = true;
}

static void uninstallTeeIfIdle() {
  if (!g_teeInstalled) {
    return;
  }
  if (g_sessionFn != nullptr || g_replyFn != nullptr) {
    return;
  }
  esp_log_set_vprintf(g_prevVprintf != nullptr ? g_prevVprintf : vprintf);
  g_prevVprintf = nullptr;
  g_teeInstalled = false;
}

// Publishes a (fn, ctx) pair for the unlocked readers in consoleTeeVprintf:
// a reader that sees a non-null fn also sees its ctx. Call with g_teeMutex held.
static void setFnCtx(GantryConsoleReplyFn *fn_slot, void **ctx_slot, GantryConsoleReplyFn fn,
              void *ctx) {
    if (fn != nullptr) {
        __atomic_store_n(ctx_slot, ctx, __ATOMIC_RELAXED);
        __atomic_store_n(fn_slot, fn, __ATOMIC_RELEASE);
    } else {
        __atomic_store_n(fn_slot, fn, __ATOMIC_RELEASE);
        __atomic_store_n(ctx_slot, ctx, __ATOMIC_RELAXED);
    }
}

static int consoleTeeVprintf(const char *fmt, va_list args) {
  char buf[512];
  va_list copy;
  va_copy(copy, args);
  int n = vsnprintf(buf, sizeof(buf), fmt, copy);
  va_end(copy);
  if (n > 0) {
    buf[sizeof(buf) - 1] = '\0';
    // Any task logs here, unlocked: load each pointer once, so a detach
    // between the test and the call can never make it call a null pointer.
    // Writers store ctx before a non-null fn; a stale ctx is caught by
    // sendAll's live-fd check.
    const GantryConsoleReplyFn session_fn = __atomic_load_n(&g_sessionFn, __ATOMIC_ACQUIRE);
    if (session_fn != nullptr) {
      if (lanLogAllowsLine(buf)) {
        session_fn(__atomic_load_n(&g_sessionCtx, __ATOMIC_RELAXED), buf);
      }
    } else {
      const GantryConsoleReplyFn reply_fn = __atomic_load_n(&g_replyFn, __ATOMIC_ACQUIRE);
      if (reply_fn != nullptr) {
        reply_fn(__atomic_load_n(&g_replyCtx, __ATOMIC_RELAXED), buf);
      }
    }
  }
  if (g_prevVprintf != nullptr) {
    return g_prevVprintf(fmt, args);
  }
  return vprintf(fmt, args);
}

// The SerialCmd and NetConsole tasks both get here first; a guarded static
// (thread-safe in ESP-IDF, components/cxx/cxx_guards.cpp) creates each mutex
// exactly once, where a check-then-create could make two.
static void ensureConsoleMutex() {
  static const bool created = [] {
    g_consoleCmdMutex = xSemaphoreCreateMutex();
    g_teeMutex = xSemaphoreCreateMutex();
    return true;
  }();
  (void)created;
}

}  // namespace GantryConsoleImpl

using namespace GantryConsoleImpl;

// The sink swap takes only the short tee lock, never g_consoleCmdMutex: a UART
// command holds that for its whole run (calibrate all: up to 120 s), which
// stalled a TCP connect or disconnect behind it. Loggers read the sink pair
// unlocked; sendAll's live-fd check drops a line a swap leaves stale.
void gantryConsoleAttachLogSink(GantryConsoleReplyFn fn, void *ctx) {
    ensureConsoleMutex();
    if (g_teeMutex != nullptr) {
        xSemaphoreTake(g_teeMutex, portMAX_DELAY);
    }
    setFnCtx(&g_sessionFn, &g_sessionCtx, fn, ctx);
    if (fn != nullptr) {
        installTeeIfNeeded();
    } else {
        uninstallTeeIfIdle();
    }
    if (g_teeMutex != nullptr) {
        xSemaphoreGive(g_teeMutex);
    }
}

void gantryConsoleDetachLogSink(void) {
  gantryConsoleAttachLogSink(nullptr, nullptr);
}

void gantryConsoleProcessLine(const GantryTestConsoleConfig *cfg, const char *line,
                              GantryConsoleReplyFn reply_fn, void *reply_ctx) {
  ensureConsoleMutex();
  if (g_consoleCmdMutex != nullptr) {
    xSemaphoreTake(g_consoleCmdMutex, portMAX_DELAY);
  }

  if (g_teeMutex != nullptr) {
    xSemaphoreTake(g_teeMutex, portMAX_DELAY);
  }
  const bool use_temp_tee = (reply_fn != nullptr && g_sessionFn == nullptr);
  setFnCtx(&g_replyFn, &g_replyCtx, reply_fn, reply_ctx);
  if (use_temp_tee) {
    installTeeIfNeeded();
  }
  if (g_teeMutex != nullptr) {
    xSemaphoreGive(g_teeMutex);
  }

  processCommand(cfg, line);

  if (g_teeMutex != nullptr) {
    xSemaphoreTake(g_teeMutex, portMAX_DELAY);
  }
  setFnCtx(&g_replyFn, &g_replyCtx, nullptr, nullptr);
  if (use_temp_tee) {
    uninstallTeeIfIdle();
  }
  if (g_teeMutex != nullptr) {
    xSemaphoreGive(g_teeMutex);
  }

  if (g_consoleCmdMutex != nullptr) {
    xSemaphoreGive(g_consoleCmdMutex);
  }
}

void gantryTestPrintHelp() {
  ESP_LOGI(TAG, "========================================");
  ESP_LOGI(TAG, "Gantry Library Example - Commands:");
  ESP_LOGI(TAG, "========================================");
  ESP_LOGI(TAG, "  help                 - show this help");
  ESP_LOGI(TAG, "  status               - print gantry status");
  ESP_LOGI(TAG, "  calibrated           - query workspace calibrated latch "
                "(Yes until a drive loses position or the ESP reboots)");
  ESP_LOGI(TAG, "  faults | alarms      - decode X/Z Kinetix and theta HCS01 diag");
  ESP_LOGI(TAG, "  puuinfo              - print X/Z PUU/mm and theta PUU/deg");
  ESP_LOGI(TAG, "  eiptiming            - dump Class 1 latency p50/p99 (exchange/ot/cycle/cmd2start)");
  ESP_LOGI(TAG, "  gaptrace             - last Class 1 gap >= 10 ms: breakdown + CPU ms per task");
  ESP_LOGI(TAG, "  bootlog              - print the first ~12 KB of log output since reset");
  ESP_LOGI(TAG, "  ethlink <x|z|t> [1|2] [clear] - drive port speed/duplex/negotiation + error counters");
  ESP_LOGI(TAG, "  drv read [t|x|z|all] | drv hcs <get|set|cmd> ... | drv k5100 <get|set> ...");
  ESP_LOGI(TAG, "  puu t <scale>        - set live theta PUU/deg (re-run home t)");
  ESP_LOGI(TAG, "  puucal <x|z|t> c m   - suggest (x/z) or apply (t) PUU scale from commanded vs measured");
  ESP_LOGI(TAG, "  thetalim <min> <max> - set theta software joint limits (deg); "
                "clamped to captured envelope after home t");
#if GANTRY_CONSOLE_AUTOTUNE
  ESP_LOGI(TAG, "  autotune [theta|t]   - run or guide driver-level inertia auto-tuning (ERD-04/HCS01 C1800)");
#else
  ESP_LOGI(TAG, "  autotune             - DISABLED this version (IndraWorks / KNX5100C / hcs01_eng.py)");
#endif
  ESP_LOGI(TAG, "  ota                  - print Dual-OTA partition status, image state, and version");
  ESP_LOGI(TAG, "  limits               - read limit switches");
  ESP_LOGI(TAG, "  pins                 - print active pin configuration");
#if MCP_DEBUG_CMDS
  ESP_LOGI(TAG, "  mcp_pin_mode p m     - force MCP pin mode (m=inpu|in|out0|out1)");
  ESP_LOGI(TAG, "  mcp_dump <a|b>       - dump MCP IOCON/dir/pullup/olat/gpio");
  ESP_LOGI(TAG, "  mcp_reg <r|w> r [v]  - raw MCP register read/write (hex/dec)");
  ESP_LOGI(TAG, "  field_dout <0..3> v  - set Field 24 V DOUT (0=gripper PA0)");
  ESP_LOGI(TAG, "  field_din            - read Field DIN + encoder + W5500 INT");
#endif  // MCP_DEBUG_CMDS
  ESP_LOGI(TAG, "  gpio_drive g v       - drive direct ESP32 GPIO g to v (0|1)");
  ESP_LOGI(TAG, "  enable               - enable motors");
  ESP_LOGI(TAG, "  disable              - disable motors");
  ESP_LOGI(TAG, "  home [x|z|t|all]     - home (EIP X/Z: seek A014/PL; t=HIPERFACE "
                "soft-home; all=Z then X then t; X needs SAFE_Z band)");
  ESP_LOGI(TAG, "  calibrate [x|z|t|all] - calibrate; 'all' = EIP bring-up "
                "(Z-/A015 switch-clear = 0 -> X home/cal -> X=%.0f -> Z+/A014 -> SAFE_Z "
                "-> theta soft-home)",
           (double)GANTRY_CAL_X_PARK_MM);
  ESP_LOGI(TAG, "  units <mm|in>        - set linear input/output units");
  ESP_LOGI(TAG, "  speed <v> [deg/s]    - set 2-D path speed (resultant) and optional theta deg/s");
  ESP_LOGI(TAG, "  accel <a> [d] [ta] [td] - path accel/decel; optional theta accel/decel (deg/s2)");
  ESP_LOGI(TAG, "  rangelimit <0|1>     - enable/disable path speed+accel/decel range clamps");
  ESP_LOGI(TAG, "  livepos <hz>         - set LIVE POS periodic rate (0=off, default off)");
  ESP_LOGI(TAG, "  axislog <hz>         - set per-axis MOVE periodic rate (0=off; START/END always on)");
  ESP_LOGI(TAG, "  move <x> <z> <t>     - move to (x_linear, z_linear, theta_deg); +Z=down, z=A015 retract");
  ESP_LOGI(TAG, "  grip <0|1>           - control gripper (0=open, 1=close)");
  ESP_LOGI(TAG, "  picksim <x> <y> <t> <class> <id> <w> <len> <h> - synthetic pick; class 1-10, packs 11-13");
  ESP_LOGI(TAG, "  pickplan <x> <y> <t> <class> <id> <w> <len> <h> - dry-run the vision contract");
  ESP_LOGI(TAG, "  beltsim <mm_s>                - synthetic conveyor clock (0 = off)");
  ESP_LOGI(TAG, "  ztrace [raw <start> <count>]  - last strike's 1 kHz Z trace");
  ESP_LOGI(TAG, "  thtrace [arm|off|raw <start> <count>] - 1 kHz trace of the next theta move");
  ESP_LOGI(TAG, "  netcon                        - TCP console send counters");
  ESP_LOGI(TAG, "  gripmark on|off               - grip timing marker on GPIO17 (TFT backlight off)");
  ESP_LOGI(TAG, "  vismark on|off                - vision frame receipt marker on GPIO17 (TFT backlight off)");
  ESP_LOGI(TAG, "  netstat [reset]               - CellNet RX counters and EMAC drop counters");
  ESP_LOGI(TAG, "  boottime                      - Ethernet start / PHY link-up / IP times since app start");
  ESP_LOGI(TAG, "  flashstall erase|write [n]    - erase, then n back-to-back 4 KB writes, passive slot from 1.5 MB (motors off)");
  ESP_LOGI(TAG, "  beltcal [near|far|cnvyr near|cnvyr far|batt|z|apply [center]] - belt/conveyor/battery X");
  ESP_LOGI(TAG, "  pickcover [class]    - pick or skip per cell angle (5 deg steps, 0 across, 90 along the belt)");
  ESP_LOGI(TAG, "  pickcal valve <ms> | jaw <class> <ms, 0=derived> | jawoff <deg> | cam mark | cam apply");
  ESP_LOGI(TAG, "  cfg                  - list settings (same keys as the panel menu)");
  ESP_LOGI(TAG, "  cfg <key> [value]    - read or set a setting; developer keys need unlock");
  ESP_LOGI(TAG, "  cfg unlock <pw>      - unlock developer settings");
  ESP_LOGI(TAG, "  cfg lock             - lock developer settings");
  ESP_LOGI(TAG, "  cfg passwd <old> <new> - change developer password (unlocked)");
  ESP_LOGI(TAG, "  cfg factory          - factory reset stored settings");
  ESP_LOGI(TAG, "  cfg reboot           - restart (apply reboot-required settings)");
    ESP_LOGI(TAG, "  test_cycle           - 8-stage holistic test: arm, bring-up, bounds check, legs A-F, pick+place grip, dynamics, theta G-I, telemetry");
  ESP_LOGI(TAG, "  test_theta_path      - combined in-band X+Z+theta (25-75 "
                "window) using live thetalim-safe dtheta; enable+bring-up first");
  ESP_LOGI(TAG, "  stop                 - stop all motion");
  ESP_LOGI(TAG, "  alarmreset | arst    - pulse EIP FaultReset / HCS01 C0500 bit5");
#if CONFIG_GANTRY_SELFTEST
  ESP_LOGI(TAG, "  selftest             - run basic math/config tests");
#endif
  ESP_LOGI(TAG, "");
}

void gantryConsoleLoadStoredProfile() {
  g_moveSpeedMmPerS = static_cast<uint32_t>(stored().gantry_default_speed_mm_per_s);
  g_moveSpeedDegPerS = static_cast<uint32_t>(stored().gantry_default_speed_deg_per_s);
  g_moveAccelMmPerS2 = static_cast<uint32_t>(stored().gantry_default_accel_mm_per_s2);
  g_moveDecelMmPerS2 = static_cast<uint32_t>(stored().gantry_default_decel_mm_per_s2);
  g_moveAccelDegPerS2 = static_cast<uint32_t>(stored().gantry_default_accel_deg_per_s2);
  g_moveDecelDegPerS2 = static_cast<uint32_t>(stored().gantry_default_decel_deg_per_s2);
}

void gantryTestConsoleTask(void *param) {
  auto *cfg = static_cast<GantryTestConsoleConfig *>(param);

  char inputLine[256];
  size_t inputIndex = 0;

  Config::AppConfig::instance().setSaveHandler([cfg]() { return saveParameters(cfg); });
  ESP_LOGI(TAG, "Serial task started");
  ESP_LOGI(TAG, "Type 'help' for commands");
  startWorkspaceUncalibrated(cfg->gantry);

  while (1) {
    // Drain UART before status polling so a typed line is not held behind
    // Gantry mutex / CTRL FLIP work after Class 1 starts.
    bool got_char = false;
    for (;;) {
      const int c = getchar();
      if (c < 0) {
        break;
      }
      got_char = true;
      if (c == '\n' || c == '\r' || c == ';') {
        if (inputIndex > 0) {
          inputLine[inputIndex] = '\0';
          ESP_LOGI(TAG, "[RX] %s", inputLine);
          gantryConsoleProcessLine(cfg, inputLine);
          inputIndex = 0;
        }
      } else if (c >= 32 && c <= 126 && inputIndex < sizeof(inputLine) - 1) {
        inputLine[inputIndex++] = static_cast<char>(c);
      }
    }

    if (!got_char) {
      monitorControlVariableFlips(cfg);
      logLiveMotionState(cfg);
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

#include "gantry_net_console.h"
#include "gantry_net_console_internal.h"

#include "ethernet_app_config.h"
#include "gantry_app_constants.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lwip/sockets.h"

#include <cerrno>
#include <cstdio>
#include <cstring>

#if CONSOLE_TCP_ENABLE

namespace GantryNetConsoleImpl {

struct NetConsoleCtx {
  const GantryTestConsoleConfig *cfg;
};

// Direct console-loop sends (prompt, banner, auth) retry this long on a full
// socket. The supervisor ends a command on the prompt; a dropped prompt left
// it "busy" until STOP (2026-10-27).
constexpr TickType_t kConsoleSendWaitTicks = pdMS_TO_TICKS(500);

SemaphoreHandle_t g_sendMutex = nullptr;
TaskHandle_t g_consoleTask = nullptr;
// The client whose socket the log sink feeds; -1 when none. Set and cleared
// under g_sendMutex, so a log line from a logger that read the sink just
// before a detach never reaches a closed fd or a later client reusing it.
int g_liveFd = -1;
// Set by gantryNetConsoleDeferRestart from the console task's own command.
bool g_restartPending = false;

// Send outcomes since boot, read with netcon (console "busy" hunt, 2026-10-27).
struct SendStats {
    uint32_t waited_sends = 0;
    uint32_t waited_short = 0;   // console-loop text not fully sent in time
    uint32_t log_skipped = 0;    // log text skipped: another send in progress
    uint32_t log_short = 0;      // log text cut by a full socket
    uint32_t log_stale = 0;      // log text for a client no longer attached
    uint32_t log_sends = 0;
    uint32_t max_wait_ms = 0;
    int last_errno = 0;
};
SendStats g_stats;

// Last received command lines: arrival time and whether the prompt that ends
// them went out in full. Tells a lost command from a lost or misread reply.
constexpr int kRxRing = 8;
struct RxEntry {
    uint32_t at_ms = 0;
    uint32_t done_ms = 0;
    bool prompt_ok = false;
    char line[24] = {};
};
RxEntry g_rx[kRxRing];
uint32_t g_rx_count = 0;

// lwIP send with MSG_DONTWAIT writes part or none of the text when the socket
// buffer is full. wait=true (console task: prompt, banner, command output)
// finishes every byte within kConsoleSendWaitTicks. Log lines from any other
// task never wait, so motion tasks never stall behind a slow PC: one that
// finds the socket busy or full is dropped. With newlib ESP-IDF holds the
// stdout lock across the tee (log_format_text.c), so a console-task wait would
// also hold other loggers; this build uses picolibc, which takes no such lock.
// True when every byte went out.
bool sendAll(int fd, const char *text, bool wait) {
    if (fd < 0 || text == nullptr) {
        return false;
    }
    const size_t n = std::strlen(text);
    if (n == 0) {
        return true;
    }
    if (wait) {
        ++g_stats.waited_sends;
    } else {
        ++g_stats.log_sends;
    }
    const TickType_t start = xTaskGetTickCount();
    if (g_sendMutex != nullptr &&
        xSemaphoreTake(g_sendMutex, wait ? kConsoleSendWaitTicks : 0) != pdTRUE) {
        if (wait) {
            ++g_stats.waited_short;
        } else {
            ++g_stats.log_skipped;
        }
        return false;
    }
    if (!wait && fd != g_liveFd) {
        ++g_stats.log_stale;
        if (g_sendMutex != nullptr) {
            xSemaphoreGive(g_sendMutex);
        }
        return false;
    }
    size_t off = 0;
    while (off < n) {
        const ssize_t w = send(fd, text + off, n - off, MSG_DONTWAIT);
        if (w > 0) {
            off += static_cast<size_t>(w);
            continue;
        }
        if (w < 0) {
            g_stats.last_errno = errno;
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                break;
            }
        }
        if (!wait || xTaskGetTickCount() - start >= kConsoleSendWaitTicks) {
            break;
        }
        vTaskDelay(1);
    }
    if (off < n) {
        if (wait) {
            ++g_stats.waited_short;
        } else {
            ++g_stats.log_short;
        }
    }
    if (wait) {
        const uint32_t ms = pdTICKS_TO_MS(xTaskGetTickCount() - start);
        if (ms > g_stats.max_wait_ms) {
            g_stats.max_wait_ms = ms;
        }
    }
    if (g_sendMutex != nullptr) {
        xSemaphoreGive(g_sendMutex);
    }
    return off == n;
}

// Waits out any log send in flight, so close() never runs under one.
void setLiveFd(int fd) {
    if (g_sendMutex != nullptr) {
        xSemaphoreTake(g_sendMutex, portMAX_DELAY);
    }
    g_liveFd = fd;
    if (g_sendMutex != nullptr) {
        xSemaphoreGive(g_sendMutex);
    }
}

// The console task's own command output waits for its turn like the prompt:
// skipped, a reply arrived as "[TCP RX] cmd" + prompt with no answer while a
// pick task was logging (2026-10-27). Other tasks hold the lock for one
// non-blocking send, so the wait is normally microseconds.
void replyToSocket(void *ctx, const char *text) {
  int fd = static_cast<int>(reinterpret_cast<intptr_t>(ctx));
  (void)sendAll(fd, text, /*wait=*/xTaskGetCurrentTaskHandle() == g_consoleTask);
}

void sendBanner(int fd) {
  const char *log_line;
#if CONSOLE_TCP_LOG_ENABLE
  static const char *const kLevelNames[] = {"?", "ERR", "WARN", "INFO", "DBUG"};
  const int lvl = CONSOLE_TCP_LOG_LEVEL;
  const char *name =
      (lvl >= 1 && lvl <= 4) ? kLevelNames[lvl] : "?";
  char log_buf[64];
  std::snprintf(log_buf, sizeof(log_buf),
                "LAN log ON (min %s). Same commands as UART.", name);
  log_line = log_buf;
#else
  log_line = "LAN log OFF (commands only). Same commands as UART.";
#endif
  char buf[320];
  std::snprintf(buf, sizeof(buf),
                "WT32 gantry console (TCP %d)%s\r\n"
                "%s\r\n"
                "Type help (or logout to disconnect)\r\n> ",
                CONSOLE_TCP_PORT,
#if CONSOLE_TCP_AUTH_ENABLE
                " — authenticated"
#else
                " — auth disabled"
#endif
                ,
                log_line);
  sendAll(fd, buf, /*wait=*/true);
}

void serveClient(const GantryTestConsoleConfig *cfg, int client_fd,
                 uint32_t peer_addr_nbo) {
  if (g_sendMutex == nullptr) {
    g_sendMutex = xSemaphoreCreateMutex();
  }
  g_consoleTask = xTaskGetCurrentTaskHandle();
  if (!authenticateClient(client_fd, peer_addr_nbo)) {
    close(client_fd);
    ESP_LOGI(TAG, "TCP client closed (auth failed or disconnect)");
    return;
  }

  setLiveFd(client_fd);
  void *ctx_fd = reinterpret_cast<void *>(static_cast<intptr_t>(client_fd));
  // Banner first, sink second: a log line ahead of the banner pushed its
  // prompt past the supervisor's banner read, and that prompt then ended the
  // first command's reply early (OPEN_ITEMS S-6, 2026-11-15).
  sendBanner(client_fd);
  if (CONSOLE_TCP_LOG_ENABLE) {
    gantryConsoleAttachLogSink(replyToSocket, ctx_fd);
    ESP_LOGI(TAG, "TCP LAN log sink attached (min level=%d)", CONSOLE_TCP_LOG_LEVEL);
  } else {
    ESP_LOGI(TAG, "TCP LAN logging disabled (commands only)");
  }

  char line[kLineCap];
  size_t len = 0;
  char chunk[64];

  while (true) {
    const int n = recv(client_fd, chunk, sizeof(chunk), 0);
    if (n <= 0) {
      break;
    }
    for (int i = 0; i < n; ++i) {
      const char c = chunk[i];
      if (c == '\n' || c == '\r' || c == ';') {
        if (len > 0) {
          line[len] = '\0';
          if (std::strcmp(line, "logout") == 0 || std::strcmp(line, "exit") == 0 ||
              std::strcmp(line, "quit") == 0) {
            sendAll(client_fd, "Bye\r\n", /*wait=*/true);
            gantryConsoleDetachLogSink();
            setLiveFd(-1);
            close(client_fd);
            ESP_LOGI(TAG, "TCP client logged out");
            return;
          }
          RxEntry &rx = g_rx[g_rx_count % kRxRing];
          ++g_rx_count;
          rx = RxEntry{};
          rx.at_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
          std::strncpy(rx.line, line, sizeof(rx.line) - 1);
          ESP_LOGI(TAG, "[TCP RX] %s", line);
          gantryConsoleProcessLine(cfg, line, replyToSocket, ctx_fd);
          rx.prompt_ok = sendAll(client_fd, "\r\n> ", /*wait=*/true);
          rx.done_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
          len = 0;
          if (g_restartPending) {
            // The prompt ends the supervisor's exchange; the close sends a
            // FIN, so its next command reconnects instead of waiting.
            gantryConsoleDetachLogSink();
            setLiveFd(-1);
            close(client_fd);
            ESP_LOGI(TAG, "TCP client closed for restart");
            vTaskDelay(pdMS_TO_TICKS(300));
            esp_restart();
          }
        }
      } else if (c >= 32 && c <= 126 && len + 1 < kLineCap) {
        line[len++] = c;
      }
    }
  }

  gantryConsoleDetachLogSink();
  setLiveFd(-1);
  close(client_fd);
  ESP_LOGI(TAG, "TCP client disconnected");
}

void netConsoleTask(void *param) {
  auto *ctx = static_cast<NetConsoleCtx *>(param);
  if (ctx == nullptr || ctx->cfg == nullptr) {
    ESP_LOGE(TAG, "Invalid net console config");
    vTaskDelete(nullptr);
    return;
  }

  const int listen_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
  if (listen_fd < 0) {
    ESP_LOGE(TAG, "socket() failed: errno=%d", errno);
    vTaskDelete(nullptr);
    return;
  }

  int yes = 1;
  (void)setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

  sockaddr_in addr = {};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(CONSOLE_TCP_PORT);

  if (bind(listen_fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
    ESP_LOGE(TAG, "bind(:%d) failed: errno=%d", CONSOLE_TCP_PORT, errno);
    close(listen_fd);
    vTaskDelete(nullptr);
    return;
  }
  if (listen(listen_fd, 1) != 0) {
    ESP_LOGE(TAG, "listen() failed: errno=%d", errno);
    close(listen_fd);
    vTaskDelete(nullptr);
    return;
  }

  ESP_LOGI(TAG, "Listening on TCP *:%d (LAN8720, auth %s)", CONSOLE_TCP_PORT,
#if CONSOLE_TCP_AUTH_ENABLE
           "enabled"
#else
           "disabled"
#endif
  );

  while (true) {
    sockaddr_in peer = {};
    socklen_t peer_len = sizeof(peer);
    const int client_fd =
        accept(listen_fd, reinterpret_cast<sockaddr *>(&peer), &peer_len);
    if (client_fd < 0) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    char ipstr[16];
    inet_ntoa_r(peer.sin_addr, ipstr, sizeof(ipstr));
    ESP_LOGI(TAG, "Client connected from %s", ipstr);
    serveClient(ctx->cfg, client_fd, peer.sin_addr.s_addr);
  }
}

}  // namespace GantryNetConsoleImpl

using namespace GantryNetConsoleImpl;

void gantryNetConsoleStart(const GantryTestConsoleConfig *cfg) {
  if (cfg == nullptr) {
    ESP_LOGE(TAG, "gantryNetConsoleStart: null cfg");
    return;
  }
  static NetConsoleCtx ctx = {};
  ctx.cfg = cfg;
  const BaseType_t ok = xTaskCreatePinnedToCore(
      netConsoleTask, "NetConsole", CONSOLE_TASK_STACK, &ctx, CONSOLE_TASK_PRIORITY,
      nullptr, CONSOLE_TASK_CORE);
  if (ok != pdPASS) {
    ESP_LOGE(TAG, "Failed to create NetConsole task");
  }
}

void gantryNetConsoleStats(char *out, size_t out_len) {
    int len = snprintf(out, out_len,
                       "netcon waited=%lu short=%lu max_wait=%lu ms | log=%lu skipped=%lu "
                       "short=%lu stale=%lu | last_errno=%d | rx=%lu",
                       static_cast<unsigned long>(g_stats.waited_sends),
                       static_cast<unsigned long>(g_stats.waited_short),
                       static_cast<unsigned long>(g_stats.max_wait_ms),
                       static_cast<unsigned long>(g_stats.log_sends),
                       static_cast<unsigned long>(g_stats.log_skipped),
                       static_cast<unsigned long>(g_stats.log_short),
                       static_cast<unsigned long>(g_stats.log_stale), g_stats.last_errno,
                       static_cast<unsigned long>(g_rx_count));
    // Newest first (the first is this netcon, its prompt still open), so a
    // truncated log line still keeps the commands just before it.
    const uint32_t first = g_rx_count > kRxRing ? g_rx_count - kRxRing : 0;
    for (uint32_t i = g_rx_count;
         i > first && len > 0 && static_cast<size_t>(len) < out_len; --i) {
        const RxEntry &e = g_rx[(i - 1) % kRxRing];
        len += snprintf(out + len, out_len - len, " | %lu '%s' +%lums %s",
                        static_cast<unsigned long>(e.at_ms), e.line,
                        static_cast<unsigned long>(e.done_ms ? e.done_ms - e.at_ms : 0),
                        e.done_ms ? (e.prompt_ok ? "ok" : "PROMPT-SHORT") : "open");
    }
}

bool gantryNetConsoleDeferRestart(void) {
    if (g_consoleTask == nullptr || xTaskGetCurrentTaskHandle() != g_consoleTask) {
        return false;
    }
    g_restartPending = true;
    return true;
}

#else  // !CONSOLE_TCP_ENABLE

void gantryNetConsoleStart(const GantryTestConsoleConfig *) {}

bool gantryNetConsoleDeferRestart(void) {
    return false;
}

void gantryNetConsoleStats(char *out, size_t out_len) {
    snprintf(out, out_len, "netcon disabled");
}

#endif  // CONSOLE_TCP_ENABLE

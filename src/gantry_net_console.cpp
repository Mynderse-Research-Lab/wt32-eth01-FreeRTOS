#include "gantry_net_console.h"

#include "ethernet_app_config.h"
#include "gantry_app_constants.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"

#include <cerrno>
#include <cstdio>
#include <cstring>

static const char *TAG = "NetConsole";

#if CONSOLE_TCP_ENABLE

namespace {

constexpr size_t kLineCap = 256;

struct NetConsoleCtx {
  const GantryTestConsoleConfig *cfg;
};

void sendAll(int fd, const char *text) {
  if (fd < 0 || text == nullptr) {
    return;
  }
  const size_t n = std::strlen(text);
  if (n == 0) {
    return;
  }
  // Non-blocking: never stall Class 1 / motion behind a slow PC.
  (void)send(fd, text, n, MSG_DONTWAIT);
}

void replyToSocket(void *ctx, const char *text) {
  int fd = static_cast<int>(reinterpret_cast<intptr_t>(ctx));
  sendAll(fd, text);
}

bool passwordsEqual(const char *attempt, const char *expected) {
  if (attempt == nullptr || expected == nullptr) {
    return false;
  }
  const size_t a = std::strlen(attempt);
  const size_t e = std::strlen(expected);
  if (a != e) {
    return false;
  }
  // Constant-time-ish compare for equal-length strings (lab auth, not HSMs).
  unsigned char diff = 0;
  for (size_t i = 0; i < e; ++i) {
    diff |= static_cast<unsigned char>(attempt[i] ^ expected[i]);
  }
  return diff == 0;
}

/** Read one line (ends on \\n/\\r/;). Returns false on disconnect/error. */
bool recvLine(int fd, char *out, size_t cap) {
  size_t len = 0;
  char chunk[64];
  while (true) {
    const int n = recv(fd, chunk, sizeof(chunk), 0);
    if (n <= 0) {
      return false;
    }
    for (int i = 0; i < n; ++i) {
      const char c = chunk[i];
      if (c == '\n' || c == '\r' || c == ';') {
        if (len > 0) {
          out[len] = '\0';
          return true;
        }
        // Ignore bare CR/LF before first character.
        continue;
      }
      if (c >= 32 && c <= 126 && len + 1 < cap) {
        out[len++] = c;
      }
    }
  }
}

#if CONSOLE_TCP_AUTH_ENABLE

constexpr size_t kRememberSlots = CONSOLE_TCP_AUTH_REMEMBER_MAX;

struct RememberedPeer {
  uint32_t addr_nbo = 0;  // IPv4 in network byte order; 0 = empty
  int64_t last_auth_us = 0;
};

RememberedPeer g_remembered[kRememberSlots] = {};

int64_t nowUs() { return esp_timer_get_time(); }

bool peerRecentlyAuthed(uint32_t addr_nbo) {
  if (addr_nbo == 0) {
    return false;
  }
  const int64_t now = nowUs();
  const int64_t ttl_us =
      static_cast<int64_t>(CONSOLE_TCP_AUTH_REMEMBER_S) * 1000000LL;
  for (size_t i = 0; i < kRememberSlots; ++i) {
    if (g_remembered[i].addr_nbo != addr_nbo) {
      continue;
    }
    if ((now - g_remembered[i].last_auth_us) <= ttl_us) {
      return true;
    }
    g_remembered[i].addr_nbo = 0;
    return false;
  }
  return false;
}

void rememberPeerAuth(uint32_t addr_nbo) {
  if (addr_nbo == 0) {
    return;
  }
  const int64_t now = nowUs();
  // Update existing slot or fill an empty one.
  size_t empty = kRememberSlots;
  size_t oldest = 0;
  int64_t oldest_ts = g_remembered[0].last_auth_us;
  for (size_t i = 0; i < kRememberSlots; ++i) {
    if (g_remembered[i].addr_nbo == addr_nbo) {
      g_remembered[i].last_auth_us = now;
      return;
    }
    if (g_remembered[i].addr_nbo == 0 && empty == kRememberSlots) {
      empty = i;
    }
    if (g_remembered[i].last_auth_us < oldest_ts) {
      oldest_ts = g_remembered[i].last_auth_us;
      oldest = i;
    }
  }
  const size_t slot = (empty < kRememberSlots) ? empty : oldest;
  g_remembered[slot].addr_nbo = addr_nbo;
  g_remembered[slot].last_auth_us = now;
}

bool authenticateClient(int fd, uint32_t peer_addr_nbo) {
  if (peerRecentlyAuthed(peer_addr_nbo)) {
    rememberPeerAuth(peer_addr_nbo);  // sliding window
    ESP_LOGI(TAG, "Auth skipped (recent IP, TTL %d s)", CONSOLE_TCP_AUTH_REMEMBER_S);
    sendAll(fd, "OK authenticated (recent IP)\r\n");
    return true;
  }

  sendAll(fd, "Authentication required. Enter password:\r\nPassword: ");

  for (int try_i = 1; try_i <= CONSOLE_TCP_AUTH_MAX_TRIES; ++try_i) {
    char attempt[kLineCap];
    if (!recvLine(fd, attempt, sizeof(attempt))) {
      ESP_LOGW(TAG, "Auth: client disconnected");
      return false;
    }
    if (passwordsEqual(attempt, CONSOLE_TCP_PASSWORD)) {
      rememberPeerAuth(peer_addr_nbo);
      ESP_LOGI(TAG, "Auth OK");
      sendAll(fd, "OK authenticated\r\n");
      return true;
    }
    ESP_LOGW(TAG, "Auth failed (try %d/%d)", try_i, CONSOLE_TCP_AUTH_MAX_TRIES);
    if (try_i >= CONSOLE_TCP_AUTH_MAX_TRIES) {
      sendAll(fd, "ERROR: too many failures\r\n");
      return false;
    }
    sendAll(fd, "ERROR: bad password\r\nPassword: ");
  }
  return false;
}
#else
bool authenticateClient(int fd, uint32_t) {
  (void)fd;
  return true;
}
#endif

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
  sendAll(fd, buf);
}

void serveClient(const GantryTestConsoleConfig *cfg, int client_fd,
                 uint32_t peer_addr_nbo) {
  if (!authenticateClient(client_fd, peer_addr_nbo)) {
    close(client_fd);
    ESP_LOGI(TAG, "TCP client closed (auth failed or disconnect)");
    return;
  }

  void *ctx_fd = reinterpret_cast<void *>(static_cast<intptr_t>(client_fd));
  if (CONSOLE_TCP_LOG_ENABLE) {
    gantryConsoleAttachLogSink(replyToSocket, ctx_fd);
    ESP_LOGI(TAG, "TCP LAN log sink attached (min level=%d)", CONSOLE_TCP_LOG_LEVEL);
  } else {
    ESP_LOGI(TAG, "TCP LAN logging disabled (commands only)");
  }
  sendBanner(client_fd);

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
            sendAll(client_fd, "Bye\r\n");
            gantryConsoleDetachLogSink();
            close(client_fd);
            ESP_LOGI(TAG, "TCP client logged out");
            return;
          }
          ESP_LOGI(TAG, "[TCP RX] %s", line);
          gantryConsoleProcessLine(cfg, line, replyToSocket, ctx_fd);
          sendAll(client_fd, "\r\n> ");
          len = 0;
        }
      } else if (c >= 32 && c <= 126 && len + 1 < kLineCap) {
        line[len++] = c;
      }
    }
  }

  gantryConsoleDetachLogSink();
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

}  // namespace

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

#else  // !CONSOLE_TCP_ENABLE

void gantryNetConsoleStart(const GantryTestConsoleConfig *) {}

#endif  // CONSOLE_TCP_ENABLE

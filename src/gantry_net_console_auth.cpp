/**
 * @file gantry_net_console_auth.cpp
 * @brief TCP console login: the password check and the cache of peers
 *        that logged in recently.
 *
 * One of the gantry_net_console*.cpp files; gantry_net_console_internal.h
 * holds what they share.
 */

#include "gantry_net_console_internal.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "lwip/sockets.h"

#include <cstring>

#if CONSOLE_TCP_ENABLE

namespace GantryNetConsoleImpl {

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

/** Read one line (ends on \\n/\\r/;). timeout_ms > 0 stops the wait. */
bool recvLine(int fd, char *out, size_t cap, int timeout_ms = 0) {
  size_t len = 0;
  char chunk[64];
  const int64_t deadline_us =
      timeout_ms > 0 ? esp_timer_get_time() + static_cast<int64_t>(timeout_ms) * 1000 : 0;
  while (true) {
    if (timeout_ms > 0) {
      const int64_t left_us = deadline_us - esp_timer_get_time();
      if (left_us <= 0) {
        return false;
      }
      fd_set rfds;
      FD_ZERO(&rfds);
      FD_SET(fd, &rfds);
      timeval tv{};
      tv.tv_sec = static_cast<long>(left_us / 1000000);
      tv.tv_usec = static_cast<long>(left_us % 1000000);
      if (select(fd + 1, &rfds, nullptr, nullptr, &tv) <= 0) {
        return false;
      }
    }
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
    sendAll(fd, "OK authenticated (recent IP)\r\n", /*wait=*/true);
    return true;
  }

  sendAll(fd, "Authentication required. Enter password:\r\nPassword: ", /*wait=*/true);

  for (int try_i = 1; try_i <= CONSOLE_TCP_AUTH_MAX_TRIES; ++try_i) {
    char attempt[kLineCap];
    if (!recvLine(fd, attempt, sizeof(attempt), 15000)) {
      ESP_LOGW(TAG, "Auth: no password within 15 s");
      return false;
    }
    if (passwordsEqual(attempt, CONSOLE_TCP_PASSWORD)) {
      rememberPeerAuth(peer_addr_nbo);
      ESP_LOGI(TAG, "Auth OK");
      sendAll(fd, "OK authenticated\r\n", /*wait=*/true);
      return true;
    }
    ESP_LOGW(TAG, "Auth failed (try %d/%d)", try_i, CONSOLE_TCP_AUTH_MAX_TRIES);
    if (try_i >= CONSOLE_TCP_AUTH_MAX_TRIES) {
      sendAll(fd, "ERROR: too many failures\r\n", /*wait=*/true);
      return false;
    }
    sendAll(fd, "ERROR: bad password\r\nPassword: ", /*wait=*/true);
  }
  return false;
}
#else
bool authenticateClient(int fd, uint32_t) {
  (void)fd;
  return true;
}
#endif

}  // namespace GantryNetConsoleImpl

#endif  // CONSOLE_TCP_ENABLE

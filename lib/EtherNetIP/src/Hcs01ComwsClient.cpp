/**
 * @file Hcs01ComwsClient.cpp
 * @brief Implementation of the HCS01 COMWS HTTP client.
 */

#include "Hcs01ComwsClient.h"
#include <cstring>
#include <cstdio>
#include <cstdlib>

#ifdef ESP_PLATFORM
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
static const char* TAG = "Hcs01Comws";
#else
#define ESP_LOGI(tag, ...)
#define ESP_LOGW(tag, ...)
#define ESP_LOGE(tag, ...)
#endif

#ifndef ESP_PLATFORM
#include <thread>
#include <chrono>
static void delayMs(uint32_t ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}
#else
static void delayMs(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}
#endif

namespace hcs01 {

Hcs01ComwsClient::Hcs01ComwsClient(eip::ITcpClient& tcp) : tcp_(tcp), connected_(false) {
    std::memset(host_, 0, sizeof(host_));
}

bool Hcs01ComwsClient::connect(const char* host) {
    std::strncpy(host_, host, sizeof(host_) - 1);
    if (!tcp_.isConnected()) {
        if (!tcp_.connect(host_, kHttpPort)) {
            ESP_LOGE(TAG, "Failed to connect to %s:%d", host_, kHttpPort);
            return false;
        }
    }
    connected_ = true;

    char login_path[128];
    std::snprintf(login_path, sizeof(login_path), "/login.cgi?name=administrator&passwd=%s", kAdminPasswdHash);
    char out_body[128] = {0};
    if (!httpGet(login_path, out_body, sizeof(out_body))) {
        ESP_LOGE(TAG, "Login failed");
        return false;
    }
    return true;
}

void Hcs01ComwsClient::disconnect() {
    if (connected_) {
        tcp_.close();
        connected_ = false;
    }
}

void Hcs01ComwsClient::normalizeIdn(const char* idn, char* out, size_t max_len) {
    if (!idn || !out || max_len == 0) return;
    std::strncpy(out, idn, max_len - 1);
    out[max_len - 1] = '\0';
    
    int dot_count = 0;
    for (const char* p = idn; *p; ++p) {
        if (*p == '.') dot_count++;
    }
    
    if (dot_count < 2) {
        std::strncat(out, ".0.0", max_len - std::strlen(out) - 1);
    }
}

uint16_t Hcs01ComwsClient::parseStatusWord(const char* raw) {
    if (!raw) return 0;
    
    // Skip leading whitespace
    while (*raw == ' ' || *raw == '\t' || *raw == '\r' || *raw == '\n') raw++;
    
    if (std::strncmp(raw, "0b", 2) == 0 || std::strncmp(raw, "0B", 2) == 0) {
        return static_cast<uint16_t>(std::strtol(raw + 2, nullptr, 2));
    } else if (std::strncmp(raw, "0x", 2) == 0 || std::strncmp(raw, "0X", 2) == 0) {
        return static_cast<uint16_t>(std::strtol(raw + 2, nullptr, 16));
    } else {
        return static_cast<uint16_t>(std::strtol(raw, nullptr, 10));
    }
}

bool Hcs01ComwsClient::httpGet(const char* path, char* out_body, size_t max_len) {
    if (!connected_) return false;
    if (!tcp_.isConnected()) {
        if (!tcp_.connect(host_, kHttpPort)) {
            connected_ = false;
            return false;
        }
    }

    char req[256];
    int req_len = std::snprintf(req, sizeof(req), 
        "GET %s HTTP/1.0\r\n"
        "Host: %s\r\n"
        "Connection: close\r\n"
        "\r\n", 
        path, host_);

    if (tcp_.send(reinterpret_cast<const uint8_t*>(req), req_len) != req_len) {
        return false;
    }

    char resp[1024];
    size_t total_recv = 0;
    while (total_recv < sizeof(resp) - 1) {
        ssize_t n = tcp_.recv(reinterpret_cast<uint8_t*>(resp + total_recv), sizeof(resp) - 1 - total_recv, 1000);
        if (n <= 0) break;
        total_recv += n;
    }
    resp[total_recv] = '\0';

    char* body = std::strstr(resp, "\r\n\r\n");
    if (body) {
        body += 4;
        std::strncpy(out_body, body, max_len - 1);
        out_body[max_len - 1] = '\0';
        return true;
    }
    
    return false;
}

bool Hcs01ComwsClient::httpPost(const char* path, const char* post_data, char* out_body, size_t max_len) {
    if (!connected_) return false;
    if (!tcp_.isConnected()) {
        if (!tcp_.connect(host_, kHttpPort)) {
            connected_ = false;
            return false;
        }
    }

    char req[512];
    int data_len = std::strlen(post_data);
    int req_len = std::snprintf(req, sizeof(req), 
        "POST %s HTTP/1.0\r\n"
        "Host: %s\r\n"
        "Content-Type: application/x-www-form-urlencoded\r\n"
        "Content-Length: %d\r\n"
        "Connection: close\r\n"
        "\r\n"
        "%s", 
        path, host_, data_len, post_data);

    if (tcp_.send(reinterpret_cast<const uint8_t*>(req), req_len) != req_len) {
        return false;
    }

    char resp[1024];
    size_t total_recv = 0;
    while (total_recv < sizeof(resp) - 1) {
        ssize_t n = tcp_.recv(reinterpret_cast<uint8_t*>(resp + total_recv), sizeof(resp) - 1 - total_recv, 1000);
        if (n <= 0) break;
        total_recv += n;
    }
    resp[total_recv] = '\0';

    char* body = std::strstr(resp, "\r\n\r\n");
    if (body) {
        body += 4;
        std::strncpy(out_body, body, max_len - 1);
        out_body[max_len - 1] = '\0';
        return true;
    }
    
    return false;
}

bool Hcs01ComwsClient::getVar(const char* idn, char* out_buf, size_t max_len) {
    char norm[64];
    normalizeIdn(idn, norm, sizeof(norm));
    
    char path[128];
    std::snprintf(path, sizeof(path), "/getvar.cgi?var1=0,1,%s", norm);
    
    char body[256];
    if (!httpGet(path, body, sizeof(body))) {
        return false;
    }
    
    char* eq = std::strchr(body, '=');
    if (eq) {
        std::strncpy(out_buf, eq + 1, max_len - 1);
    } else {
        std::strncpy(out_buf, body, max_len - 1);
    }
    out_buf[max_len - 1] = '\0';
    
    // Trim trailing whitespace/newlines
    size_t len = std::strlen(out_buf);
    while (len > 0 && (out_buf[len - 1] == '\r' || out_buf[len - 1] == '\n' || out_buf[len - 1] == ' ')) {
        out_buf[len - 1] = '\0';
        len--;
    }
    
    return true;
}

bool Hcs01ComwsClient::setVar(const char* idn, const char* value) {
    char norm[64];
    normalizeIdn(idn, norm, sizeof(norm));
    
    char post_data[256];
    std::snprintf(post_data, sizeof(post_data), "var=0,1,%s&value=%s", norm, value);
    
    char body[128];
    return httpPost("/setvar.cgi", post_data, body, sizeof(body));
}

bool Hcs01ComwsClient::runCommand(const char* idn, uint32_t timeout_ms) {
    char norm[64];
    normalizeIdn(idn, norm, sizeof(norm));
    
    char op[128];
    char st[128];
    
    size_t len = std::strlen(norm);
    if (len > 2 && norm[len-2] == '.' && norm[len-1] == '7') {
        std::strncpy(op, norm, sizeof(op));
        std::strncpy(st, norm, sizeof(st));
        st[std::strlen(st)-1] = '8';
    } else {
        std::snprintf(op, sizeof(op), "%s.7", norm);
        std::snprintf(st, sizeof(st), "%s.8", norm);
    }
    
    setVar(op, "0b0000000000000000");
    setVar(op, "0b0000000000000011");
    
    uint32_t elapsed = 0;
    while (elapsed < timeout_ms) {
        char status_str[32] = {0};
        if (getVar(st, status_str, sizeof(status_str))) {
            uint16_t sw = parseStatusWord(status_str);
            bool set_bit = (sw & kCmdStatusSet);
            bool enabled = (sw & kCmdStatusEnabled);
            bool executing = (sw & kCmdStatusExecuting);
            bool error = (sw & kCmdStatusError);
            
            if (error) {
                char diag[128] = {0};
                getVar("S-0-0095.0.0", diag, sizeof(diag));
                ESP_LOGE(TAG, "Command %s error: %s", norm, diag);
                setVar(op, "0b0000000000000000");
                return false;
            }
            
            if (set_bit && enabled && !executing) {
                setVar(op, "0b0000000000000000");
                return true;
            }
        }
        
        delayMs(300);
        elapsed += 300;
    }
    
    setVar(op, "0b0000000000000000");
    return false;
}

TuningResult Hcs01ComwsClient::runAutotune(float damping, float travel_deg, bool save_nv) {
    TuningResult result;
    
    // Procedure C1800 is P-0-0162
    if (!runCommand("P-0-0162.0.0")) {
        return result;
    }
    
    getVar("P-0-4010.0.0", result.load_inertia, sizeof(result.load_inertia));
    getVar("S-0-0100.0.0", result.kp, sizeof(result.kp));
    getVar("S-0-0101.0.0", result.tn, sizeof(result.tn));
    getVar("S-0-0104.0.0", result.kv, sizeof(result.kv));
    getVar("S-0-0348.0.0", result.ka, sizeof(result.ka));
    
    if (save_nv) {
        // Command C2200 is S-0-0264
        runCommand("S-0-0264.0.0");
    }
    
    result.success = true;
    return result;
}

} // namespace hcs01

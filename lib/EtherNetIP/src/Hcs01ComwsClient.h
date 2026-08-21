/**
 * @file Hcs01ComwsClient.h
 * @brief HCS01 Service Tool COMWS HTTP client.
 */

#ifndef HCS01_COMWS_CLIENT_H
#define HCS01_COMWS_CLIENT_H

#include <cstdint>
#include <cstddef>
#include "EipTransport.h"

namespace hcs01 {

// Default engineering IP
constexpr const char* kDefaultEngIp = "192.168.1.22";
constexpr uint16_t kHttpPort = 80;

// MD5 hash of admin password (from hcs01_comws.py)
constexpr const char* kAdminPasswdHash = "526545e524f08d95e0514ab5fe3aac46";

// Procedure command status bits (from .8 element)
constexpr uint8_t kCmdStatusSet = 0x01;
constexpr uint8_t kCmdStatusEnabled = 0x02;
constexpr uint8_t kCmdStatusExecuting = 0x04;
constexpr uint8_t kCmdStatusError = 0x08;

struct TuningResult {
    bool success = false;
    char load_inertia[32] = {}; // P-0-4010.0.0
    char kp[32] = {};            // S-0-0100.0.0
    char tn[32] = {};            // S-0-0101.0.0
    char kv[32] = {};            // S-0-0104.0.0
    char ka[32] = {};            // S-0-0348.0.0
};

class Hcs01ComwsClient {
public:
    explicit Hcs01ComwsClient(eip::ITcpClient& tcp);

    // Connect to HTTP port and login
    bool connect(const char* host = kDefaultEngIp);

    // Read an IDN value. Returns the value string in out_buf.
    bool getVar(const char* idn, char* out_buf, size_t max_len);

    // Write an IDN value.
    bool setVar(const char* idn, const char* value);

    // Execute a SERCOS procedure command (e.g., P-0-0162 for C1800).
    // Polls .8 status until completion or timeout.
    bool runCommand(const char* idn, uint32_t timeout_ms = 30000);

    // Full autotune sequence: configure, C1800, read results, C2200 backup.
    TuningResult runAutotune(float damping = 1.0f, float travel_deg = 45.0f, bool save_nv = true);

    void disconnect();

    // Normalizes IDN: ensure .0.0 suffix
    static void normalizeIdn(const char* idn, char* out, size_t max_len);
    
    // Parse a status word from COMWS (decimal, 0x, 0b)
    static uint16_t parseStatusWord(const char* raw);

private:
    // Send an HTTP request and receive the response body
    bool httpGet(const char* path, char* out_body, size_t max_len);
    bool httpPost(const char* path, const char* post_data, char* out_body, size_t max_len);
    
    eip::ITcpClient& tcp_;
    char host_[32] = {};
    bool connected_ = false;
};

} // namespace hcs01

#endif // HCS01_COMWS_CLIENT_H

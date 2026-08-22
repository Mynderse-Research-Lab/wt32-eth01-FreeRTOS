// W5500 MACRAW Loopback Validation Test
//
// Standalone ESP-IDF app. Connect the WIZ850io module to WT32-ETH01:
//
//   WIZ850io J1          WT32-ETH01
//   ───────────          ──────────
//   Pin 1  GND    →      GND
//   Pin 2  GND    →      GND
//   Pin 3  MOSI   →      GPIO17
//   Pin 4  SCLK   →      GPIO5
//   Pin 5  SCSn   →      GPIO15  (VDM: must toggle per SPI frame)
//   Pin 6  INTn   →      (unused; poll mode)
//
//   WIZ850io J2          WT32-ETH01
//   ───────────          ──────────
//   Pin 1  GND    →      GND
//   Pin 2  3V3D   →      3V3
//   Pin 3  3V3D   →      3V3
//   Pin 4  NC     →      (unconnected)
//   Pin 5  RSTn   →      MCP23S17 PB7 (via firmware rst_set_level)
//   Pin 6  MISO   →      GPIO35
//
// Connect WIZ850io RJ45 to a PC running Wireshark to verify the EtherCAT
// broadcast frame is captured on the wire.
//
// Build:  idf.py -C test/w5500_loopback build
// Flash:  idf.py -C test/w5500_loopback -p COM3 flash monitor

#include "W5500.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <cstring>

static const char* TAG = "loopback";

// --- Pin map (WT32-ETH01 with MCP23S17 removed, W5500 on VSPI) ---

static constexpr int W5500_SPI_HOST  = 1;       // SPI2_HOST
static constexpr int W5500_CS_GPIO   = 15;      // VDM requires CS frame edges
static constexpr int W5500_INT_GPIO  = -1;       // polled; INT unused in production
static constexpr int W5500_MOSI_GPIO = 17;      // matches production gantry_app_constants.h
static constexpr int W5500_RST_GPIO  = -1;       // production: MCP PB7 via rst_set_level

static constexpr int W5500_MISO_GPIO = 35;
static constexpr int W5500_SCLK_GPIO = 5;
static constexpr int W5500_SCLK_HZ   = 20000000; // matches production default

// --- EtherCAT broadcast frame ---
// Ethernet header: dst=ff:ff:ff:ff:ff:ff, src=00:11:22:33:44:55, EtherType=0x88A4
// EtherCAT datagram header: 1 datagram, cmd=APRD (ignore), len=0x000A, etc.
static const uint8_t ECAT_FRAME[] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,  // dest MAC (broadcast)
    0x00, 0x11, 0x22, 0x33, 0x44, 0x55,  // src MAC (placeholder)
    0x88, 0xA4,                            // EtherType: EtherCAT
    // EtherCAT datagram (minimal dummy payload)
    0x06, 0x00,       // Length = 6 (type + cmd + idx + addr)
    0x00,              // Reserved
    0x01,              // 1 datagram following
    // Datagram header
    0x02,              // Cmd: APRD
    0x00,              // Idx: auto-increment addressing
    0x00, 0x00,       // Address
    0x0A, 0x00,       // Length = 10 (WKC + data)
    0x00, 0x00,       // IRQ (unused)
    // Working counter placeholder + data (10 bytes)
    0x00, 0x00,       // WKC
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
static constexpr uint16_t ECAT_FRAME_LEN = sizeof(ECAT_FRAME);

// =============================================================================
// Test stage helpers
// =============================================================================

static bool testInit(W5500& w5500, const W5500Config& cfg) {
    ESP_LOGI(TAG, "=== Stage 1: W5500 Init ===");
    if (!w5500.init(cfg)) {
        ESP_LOGE(TAG, "FAIL: W5500 init failed. Check wiring and power.");
        return false;
    }
    ESP_LOGI(TAG, "PASS: W5500 init succeeded.");
    return true;
}

static bool testVersion(W5500& w5500) {
    ESP_LOGI(TAG, "=== Stage 2: Chip Version ===");
    uint8_t ver = w5500.getVersion();
    ESP_LOGI(TAG, "VERSIONR = 0x%02X (expected 0x04)", ver);
    if (ver != 0x04) {
        ESP_LOGE(TAG, "FAIL: Unexpected chip version. Check SPI wiring.");
        return false;
    }
    ESP_LOGI(TAG, "PASS: Chip version OK.");
    return true;
}

static bool testMacAddress(W5500& w5500) {
    ESP_LOGI(TAG, "=== Stage 3: MAC Address ===");
    uint8_t mac[6];
    w5500.getMacAddress(mac);
    ESP_LOGI(TAG, "MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    // A zero MAC likely means SPI isn't reading back correctly
    bool allZero = true;
    for (int i = 0; i < 6; i++) {
        if (mac[i] != 0) { allZero = false; break; }
    }
    if (allZero) {
        ESP_LOGE(TAG, "FAIL: MAC is all zeros. SPI read may be broken.");
        return false;
    }
    ESP_LOGI(TAG, "PASS: MAC address read OK.");
    return true;
}

static bool testLinkStatus(W5500& w5500) {
    ESP_LOGI(TAG, "=== Stage 4: PHY Link ===");
    // Poll link status with timeout (PHY auto-negotiation takes ~2-3 seconds)
    const int maxTries = 60;  // 60 * 100ms = 6s timeout
    for (int i = 0; i < maxTries; i++) {
        bool link = w5500.isLinkUp();
        ESP_LOGI(TAG, "  try %d: link=%s", i + 1, link ? "UP" : "down");
        if (link) {
            ESP_LOGI(TAG, "PASS: PHY link up.");
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    ESP_LOGW(TAG, "WARN: PHY link not up after %d ms. Check cable and switch.",
             maxTries * 100);
    ESP_LOGW(TAG, "Frame send will likely fail without link.");
    return false;  // Non-fatal — test can still try sending
}

static bool testMacRawOpen(W5500& w5500) {
    ESP_LOGI(TAG, "=== Stage 5: MACRAW Socket Open ===");
    if (!w5500.openMacRaw(0)) {
        ESP_LOGE(TAG, "FAIL: Could not open MACRAW socket 0.");
        return false;
    }
    ESP_LOGI(TAG, "PASS: Socket 0 in MACRAW mode.");
    return true;
}

static bool testFrameSend(W5500& w5500) {
    ESP_LOGI(TAG, "=== Stage 6: EtherCAT Frame Send ===");
    ESP_LOGI(TAG, "Sending %d-byte EtherCAT broadcast frame (EtherType 0x88A4)...",
             ECAT_FRAME_LEN);

    int sent = w5500.sendFrame(ECAT_FRAME, ECAT_FRAME_LEN);
    if (sent < 0) {
        ESP_LOGE(TAG, "FAIL: sendFrame returned %d (TX error or timeout).", sent);
        ESP_LOGE(TAG, "Check: cable connected? switch/link partner powered?");
        return false;
    }
    if (sent != ECAT_FRAME_LEN) {
        ESP_LOGW(TAG, "WARN: sent %d bytes, expected %d", sent, ECAT_FRAME_LEN);
    }
    ESP_LOGI(TAG, "PASS: Frame sent (%d bytes). Verify with Wireshark.", sent);
    return true;
}

static void testFrameRecv(W5500& w5500) {
    ESP_LOGI(TAG, "=== Stage 7: Incoming Frame Check ===");
    uint8_t buf[1536];
    int n = w5500.recvFrame(buf, sizeof(buf));
    if (n > 0) {
        ESP_LOGI(TAG, "Received %d-byte frame (EtherType of first bytes: 0x%02X%02X)",
                 n, buf[12], buf[13]);
    } else if (n == 0) {
        ESP_LOGI(TAG, "No frames received (expected on quiet network).");
    } else {
        ESP_LOGW(TAG, "recvFrame returned %d (error)", n);
    }
}

// =============================================================================
// Entry point
// =============================================================================

extern "C" void app_main() {
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  W5500 MACRAW Loopback Validation Test");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "");

    W5500 w5500;
    W5500Config cfg = {};
    cfg.spi_host  = W5500_SPI_HOST;
    cfg.cs_gpio   = W5500_CS_GPIO;
    cfg.int_gpio  = W5500_INT_GPIO;
    cfg.rst_gpio  = W5500_RST_GPIO;
    cfg.mosi_gpio = W5500_MOSI_GPIO;
    cfg.miso_gpio = W5500_MISO_GPIO;
    cfg.sclk_gpio = W5500_SCLK_GPIO;
    cfg.sclk_hz   = W5500_SCLK_HZ;

    int passed = 0;
    int total  = 0;

    // Stage 1: Init
    total++;
    if (testInit(w5500, cfg)) passed++;

    // Stage 2: Version
    total++;
    if (testVersion(w5500)) passed++;

    // Stage 3: MAC
    total++;
    if (testMacAddress(w5500)) passed++;

    // Stage 4: Link
    total++;
    bool linkUp = testLinkStatus(w5500);
    if (linkUp) passed++;

    // Stage 5: MACRAW open
    total++;
    if (testMacRawOpen(w5500)) passed++;

    // Stage 6: Send
    total++;
    if (testFrameSend(w5500)) passed++;

    // Stage 7: Recv (advisory only, not scored)
    testFrameRecv(w5500);

    // Cleanup
    w5500.deinit();

    // Summary
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  RESULTS: %d / %d stages passed", passed, total);
    ESP_LOGI(TAG, "========================================");

    if (passed == total) {
        ESP_LOGI(TAG, "ALL TESTS PASSED. W5500 MACRAW is operational.");
        ESP_LOGI(TAG, "Ready for Phase 2: SOEM integration.");
    } else {
        ESP_LOGE(TAG, "SOME TESTS FAILED. Review logs above.");
        ESP_LOGE(TAG, "Common issues:");
        ESP_LOGE(TAG, "  - Wrong GPIO wiring (double-check MOSI/MISO/SCLK/CS)");
        ESP_LOGE(TAG, "  - Missing 3.3V power on J2 pins 2+3");
        ESP_LOGE(TAG, "  - SPI bus conflict (remove any other SPI device)");
        ESP_LOGE(TAG, "  - RSTn not driven high (needs GPIO17 wired)");
        ESP_LOGE(TAG, "  - Boot strap: GPIO15 MUST be HIGH and GPIO12 LOW at boot");
    }
}

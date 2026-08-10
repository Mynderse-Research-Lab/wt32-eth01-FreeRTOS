#ifndef W5500_H
#define W5500_H

#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

// ==========================================================================
// W5500 SPI Frame Construction Helpers (host-testable, no ESP-IDF deps)
// ==========================================================================

// Block Select Bits for the control byte.
// BSB[4:0] selects the register block; the control byte places them at bits
// 7:3.  Use makeCtrlByte() below to build the full control byte.

constexpr uint8_t kBlockCommon()        { return 0x00; }
constexpr uint8_t kBlockSocketReg(uint8_t sock) { return static_cast<uint8_t>(sock * 4 + 1); }
constexpr uint8_t kBlockSocketTxBuf(uint8_t sock) { return static_cast<uint8_t>(sock * 4 + 2); }
constexpr uint8_t kBlockSocketRxBuf(uint8_t sock) { return static_cast<uint8_t>(sock * 4 + 3); }

constexpr uint8_t kRwRead  = 0;
constexpr uint8_t kRwWrite = 1;

// Build the 8-bit control byte: (BSB << 3) | (RWB << 2) | OM.
// OM = Variable Data Length Mode (0) is the only mode used here.
constexpr uint8_t makeCtrlByte(uint8_t blockSelect, uint8_t rw) {
    return static_cast<uint8_t>((blockSelect << 3) | (rw << 2));
}

// ==========================================================================
// W5500 Register Map
// ==========================================================================

// -- Common registers (address within common block) --
constexpr uint16_t REG_MR        = 0x0000;   // Mode Register
constexpr uint16_t REG_GAR       = 0x0001;   // Gateway Address (4 bytes)
constexpr uint16_t REG_SUBR      = 0x0005;   // Subnet Mask (4 bytes)
constexpr uint16_t REG_SHAR      = 0x0009;   // Source MAC (6 bytes)
constexpr uint16_t REG_SIPR      = 0x000F;   // Source IP (4 bytes)
constexpr uint16_t REG_SIR       = 0x0017;   // Socket Interrupt Register
constexpr uint16_t REG_PHYCFGR   = 0x002E;   // PHY Configuration Register
constexpr uint16_t REG_VERSIONR  = 0x0039;   // Chip version (expect 0x04)

// Mode Register bits
constexpr uint8_t MR_RST  = 0x80;   // Software reset (self-clearing)
constexpr uint8_t MR_WOL  = 0x20;   // Wake on LAN
constexpr uint8_t MR_PB   = 0x10;   // Ping block

// -- Socket N registers (offset within socket register block) --
constexpr uint16_t Sn_MR      = 0x0000;   // Mode
constexpr uint16_t Sn_CR      = 0x0001;   // Command
constexpr uint16_t Sn_IR      = 0x0002;   // Interrupt
constexpr uint16_t Sn_SR      = 0x0003;   // Status
constexpr uint16_t Sn_PORT    = 0x0004;   // Source port (2 bytes)
constexpr uint16_t Sn_DIPR    = 0x000C;   // Destination IP (4 bytes)
constexpr uint16_t Sn_DPORT   = 0x0010;   // Destination port (2 bytes)
constexpr uint16_t Sn_TX_FSR  = 0x0020;   // TX free size (2 bytes)
constexpr uint16_t Sn_TX_RD   = 0x0022;   // TX read pointer (2 bytes)
constexpr uint16_t Sn_TX_WR   = 0x0024;   // TX write pointer (2 bytes)
constexpr uint16_t Sn_RX_RSR  = 0x0026;   // RX received size (2 bytes)
constexpr uint16_t Sn_RX_RD       = 0x0028;   // RX read pointer (2 bytes)
constexpr uint16_t Sn_RXBUF_SIZE  = 0x001E;   // RX buffer size (1 byte, KB)
constexpr uint16_t Sn_TXBUF_SIZE  = 0x001F;   // TX buffer size (1 byte, KB)

// -- Socket Mode bits --
constexpr uint8_t Sn_MR_CLOSE  = 0x00;
constexpr uint8_t Sn_MR_TCP    = 0x01;
constexpr uint8_t Sn_MR_UDP    = 0x02;
constexpr uint8_t Sn_MR_MACRAW = 0x04;
constexpr uint8_t Sn_MR_MULTI  = 0x80;   // UDP multicast (IGMP join on OPEN)

// -- Socket Command values --
constexpr uint8_t Sn_CR_OPEN    = 0x01;
constexpr uint8_t Sn_CR_LISTEN  = 0x02;
constexpr uint8_t Sn_CR_CONNECT = 0x04;
constexpr uint8_t Sn_CR_DISCON  = 0x08;
constexpr uint8_t Sn_CR_CLOSE   = 0x10;
constexpr uint8_t Sn_CR_SEND    = 0x20;
constexpr uint8_t Sn_CR_RECV    = 0x40;

// -- Socket Status values (W5500 datasheet Table 5.2) --
constexpr uint8_t SOCK_CLOSED      = 0x00;
constexpr uint8_t SOCK_INIT        = 0x13;   // TCP/UDP opened, ready to connect/listen
constexpr uint8_t SOCK_LISTEN      = 0x14;   // TCP server listening
constexpr uint8_t SOCK_SYNSENT     = 0x15;   // TCP client: SYN sent, awaiting SYN-ACK
constexpr uint8_t SOCK_SYNRECV     = 0x16;   // TCP server: SYN received, sent SYN-ACK
constexpr uint8_t SOCK_ESTABLISHED = 0x17;   // TCP connection established
constexpr uint8_t SOCK_CLOSE_WAIT  = 0x1C;   // Remote sent FIN
constexpr uint8_t SOCK_UDP         = 0x22;
constexpr uint8_t SOCK_MACRAW      = 0x42;

// -- Interrupt bits --
constexpr uint8_t Sn_IR_SENDOK     = 0x10;
constexpr uint8_t Sn_IR_TIMEOUT    = 0x08;
constexpr uint8_t Sn_IR_RECV       = 0x04;
constexpr uint8_t Sn_IR_DISCON     = 0x02;
constexpr uint8_t Sn_IR_CON        = 0x01;

// ==========================================================================
// W5500 Driver Configuration
// ==========================================================================

struct W5500Config {
    int spi_host;       // e.g. SPI2_HOST (HSPI) or SPI3_HOST (VSPI)
    int cs_gpio;        // Chip select GPIO (required for VDM; -1 only if using FDM)
    int int_gpio;       // Interrupt GPIO (-1 for polled mode)
    int rst_gpio;       // Hardware reset GPIO (-1 if using rst_set_level or soft reset)
    int sclk_hz;        // SPI clock frequency (e.g. 20'000'000 for 20 MHz)
    int mosi_gpio;      // MOSI GPIO
    int miso_gpio;      // MISO GPIO
    int sclk_gpio;      // SCLK GPIO
    /// Optional external RST (e.g. MCP23S17). Called with level 0 then 1.
    void (*rst_set_level)(void* ctx, int level) = nullptr;
    void* rst_ctx = nullptr;
};

// ==========================================================================
// W5500 Class
// ==========================================================================

class W5500 {
    friend class W5500SpiHal;
public:
    W5500() = default;
    ~W5500();

    // Initialise SPI bus, hardware-reset the chip, verify chip version.
    // Returns true if VERSIONR == 0x04. One-shot; subsequent calls are no-ops.
    bool init(const W5500Config& cfg);

    // Runtime chip recovery: force-close sockets, GPIO/MR hard reset, re-apply
    // MAC/IP/buffers. SPI bus stays up. Caller must drop live TCP/UDP sockets
    // first (e.g. scanner disconnect). Returns true if VERSIONR == 0x04.
    bool recover();

    // Close MACRAW socket and de-init SPI.
    void deinit();

    // Open socket in MACRAW mode. Only socket 0 supports MACRAW.
    bool openMacRaw(uint8_t sock = 0);

    // Close a socket.
    void closeSocket(uint8_t sock = 0);

    // Send a raw Ethernet frame through the MACRAW socket.
    // Returns the number of bytes sent, or -1 on error.
    int sendFrame(const uint8_t* data, uint16_t len);

    // Receive a raw Ethernet frame. Returns frame length, 0 if none, -1 error.
    // The caller provides a buffer of at least bufSize bytes.
    int recvFrame(uint8_t* buf, uint16_t bufSize);

    // Read W5500 chip version (expect 0x04).
    uint8_t getVersion();

    // Read the factory-burned MAC address (6 bytes).
    void getMacAddress(uint8_t mac[6]);

    // Check if the PHY link is up.
    bool isLinkUp();

    // Hold the SPI2 device lock across a Class 1 exchange so each register
    // access skips per-transaction bus arbitration (polling transmit path).
    bool acquireBus();
    void releaseBus();

private:
    // SPI register access
    uint8_t  readReg(uint8_t blockSelect, uint16_t addr);
    void     writeReg(uint8_t blockSelect, uint16_t addr, uint8_t value);
    uint16_t readReg16(uint8_t blockSelect, uint16_t addr);
    void     writeReg16(uint8_t blockSelect, uint16_t addr, uint16_t value);
    void     readBuf(uint8_t blockSelect, uint16_t addr, uint8_t* buf, uint16_t len);
    void     writeBuf(uint8_t blockSelect, uint16_t addr, const uint8_t* buf, uint16_t len);

    // Socket register convenience wrappers
    uint8_t  readSocketReg(uint8_t sock, uint16_t addr);
    void     writeSocketReg(uint8_t sock, uint16_t addr, uint8_t value);
    uint16_t readSocketReg16(uint8_t sock, uint16_t addr);
    void     writeSocketReg16(uint8_t sock, uint16_t addr, uint16_t value);

    // Hardware reset sequence (RSTn pin)
    void hardReset();

    // Re-apply MAC/IP/socket buffers after hardReset (shared by init/recover).
    bool configureAfterReset();

    // Wait for a socket command to complete (Sn_CR clears to 0).
    bool waitForCommand(uint8_t sock, uint32_t timeoutMs = 100);

    // Raw SPI transfer
    void spiTransfer(const uint8_t* txData, uint8_t* rxData, size_t len);

    W5500Config cfg_;
    bool initialized_ = false;
    bool bus_acquired_ = false;

#ifdef ESP_PLATFORM
    void* spiHandle_ = nullptr;   // spi_device_handle_t
#else
    void* spiHandle_ = nullptr;
#endif
};

#ifdef __cplusplus
}
#endif

#endif  // W5500_H

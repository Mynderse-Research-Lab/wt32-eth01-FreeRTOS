// W5500 hardware TCP/UDP socket API.
//
// Generic, transport-agnostic socket layer on top of the W5500 register I/O.
// Owns all W5500-specific knowledge: register addresses, socket state
// machines, SPI framing, FIFO pointer math. No knowledge of EtherNet/IP,
// lwIP, or any application-layer protocol.
//
// All public functions acquire an internal mutex before SPI access.
// Blocking operations (connect, recv) use Strategy 1 busy-poll with
// vTaskDelay(1) yield to avoid starving higher-priority tasks.

#ifndef W5500_SOCKET_H
#define W5500_SOCKET_H

#include <cstdint>
#include <cstddef>

#include "W5500Hal.h"

namespace w5500 {

enum class SocketMode : uint8_t {
    kTcp = 0x01,   // Sn_MR_TCP
    kUdp = 0x02,   // Sn_MR_UDP
};

// --- Socket lifecycle ---

// Open a socket in TCP or UDP mode. Finds the first free socket
// (Sn_SR == SOCK_CLOSED), sets the local port and mode, then issues OPEN.
// TCP uses an ephemeral local port when localPort is 0. Returns socket number
// (0-7) on success, -1 if no free socket.
// For UDP with udpMulticastListenIp != 0: sets Sn_MR_MULTI and joins the
// multicast group (Sn_DIPR/Sn_DPORT) before OPEN per W5500 datasheet.
int socketOpen(W5500Hal& hal, SocketMode mode, uint16_t localPort = 0,
               uint32_t udpMulticastListenIp = 0);

// Ephemeral local port for outbound TCP/UDP sockets (49152+).
uint16_t nextEphemeralPort();

// Close a socket and return it to SOCK_CLOSED state.
void socketClose(W5500Hal& hal, uint8_t sock);

// TCP client connect. Sets destination IP/port, issues OPEN+CONNECT,
// polls Sn_SR until SOCK_ESTABLISHED or timeout. Returns true on connect.
bool socketConnect(W5500Hal& hal, uint8_t sock, uint32_t ip, uint16_t port,
                   uint32_t timeoutMs);

// TCP server: bind to port and listen.
bool socketListen(W5500Hal& hal, uint8_t sock, uint16_t port);

// --- Data transfer ---

// Send data on a connected TCP socket. Returns bytes sent, or -1 on error.
int socketSend(W5500Hal& hal, uint8_t sock, const uint8_t* data, size_t len);

// Receive data from a connected TCP socket.
// Returns bytes read, 0 if no data within timeout, -1 on error/disconnect.
int socketRecv(W5500Hal& hal, uint8_t sock, uint8_t* buf, size_t maxLen,
               uint32_t timeoutMs);

// Send UDP datagram to a specific destination.
int socketSendTo(W5500Hal& hal, uint8_t sock, const uint8_t* data, size_t len,
                 uint32_t destIp, uint16_t destPort);

// Receive UDP datagram. Returns payload bytes, 0 if no data within timeout,
// -1 on error. W5500 prepends an 8-byte header (dest IP 4B + dest port 2B +
// length 2B) which is stripped before returning.
int socketRecvFrom(W5500Hal& hal, uint8_t sock, uint8_t* buf, size_t maxLen,
                   uint32_t timeoutMs);

// Non-blocking UDP recv: return 0 immediately when RX_RSR has no full header.
int socketRecvFromNonBlocking(W5500Hal& hal, uint8_t sock, uint8_t* buf,
                              size_t maxLen);

// UDP socket bind.
bool socketBind(W5500Hal& hal, uint8_t sock, uint16_t port);

// --- Status ---

// True if socket is in SOCK_ESTABLISHED state.
bool socketIsConnected(W5500Hal& hal, uint8_t sock);

// True if socket is in SOCK_UDP mode and has been opened.
bool socketIsOpen(W5500Hal& hal, uint8_t sock);

// Number of bytes available in the socket's RX FIFO (reads Sn_RX_RSR).
// For UDP, this includes the 8-byte header. For TCP, this is raw payload.
int socketRxAvailable(W5500Hal& hal, uint8_t sock);

// Current socket status register value (Sn_SR).
uint8_t socketStatus(W5500Hal& hal, uint8_t sock);

}  // namespace w5500

#endif  // W5500_SOCKET_H

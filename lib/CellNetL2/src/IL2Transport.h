/**
 * @file IL2Transport.h
 * @brief Abstract interface for Layer-2 raw Ethernet frame transport.
 *
 * Decouples framing, validation, and node dispatch logic from the underlying
 * network hardware (ESP-IDF LAN8720 EMAC, Linux AF_PACKET, or Host Fakes).
 */

#ifndef CELL_NET_IL2_TRANSPORT_H
#define CELL_NET_IL2_TRANSPORT_H

#include <cstddef>
#include <cstdint>
#include <functional>

namespace CellNet {

/**
 * @brief Callback function type invoked upon raw Ethernet frame arrival.
 * @param buffer Pointer to the raw received Ethernet frame.
 * @param length Total length of the received frame in bytes.
 */
using RxFrameCallback = std::function<void(const uint8_t* buffer, size_t length)>;

/**
 * @brief Abstract interface for physical Layer-2 packet I/O.
 */
class IL2Transport {
 public:
  virtual ~IL2Transport() = default;

  /**
   * @brief Transmit a raw Layer-2 Ethernet frame.
   * @param data Pointer to frame data (starting with 14-byte Ethernet MAC header).
   * @param length Total size of the frame in bytes.
   * @return true if successfully queued/transmitted, false on failure.
   */
  virtual bool sendFrame(const uint8_t* data, size_t length) = 0;

  /**
   * @brief Retrieve the 6-byte hardware MAC address of the local network interface.
   * @param mac_out Output buffer of at least 6 bytes.
   * @return true if MAC is valid, false otherwise.
   */
  virtual bool getMacAddress(uint8_t mac_out[6]) const = 0;

  /**
   * @brief Check if the underlying Ethernet physical link is up.
   * @return true if link is established (e.g. 100BASE-TX full-duplex), false if unplugged.
   */
  virtual bool isLinkUp() const = 0;

  /**
   * @brief Register the callback handler for incoming raw Ethernet frames.
   * @param callback Callback function.
   */
  virtual void setRxCallback(RxFrameCallback callback) = 0;
};

}  // namespace CellNet

#endif  // CELL_NET_IL2_TRANSPORT_H

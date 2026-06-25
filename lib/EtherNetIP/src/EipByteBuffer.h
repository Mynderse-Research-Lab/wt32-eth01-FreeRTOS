// Little-endian byte (de)serialization helpers for the EtherNet/IP encoding
// layer. Header-only, freestanding C++17: no ESP-IDF, no exceptions, no
// dynamic surprises. EtherNet/IP / CIP scalars are little-endian on the wire.
//
// ByteWriter appends to a std::vector<uint8_t>; ByteReader walks a const byte
// range with explicit bounds checks (every read reports success/failure rather
// than throwing, so the same code runs unchanged on the MCU later).

#ifndef ETHERNET_IP_EIP_BYTE_BUFFER_H
#define ETHERNET_IP_EIP_BYTE_BUFFER_H

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

namespace eip {

using Bytes = std::vector<uint8_t>;

class ByteWriter {
public:
  explicit ByteWriter(Bytes& out) : out_(out) {}

  void u8(uint8_t v) { out_.push_back(v); }

  void u16(uint16_t v) {
    out_.push_back(static_cast<uint8_t>(v & 0xFF));
    out_.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
  }

  void u32(uint32_t v) {
    out_.push_back(static_cast<uint8_t>(v & 0xFF));
    out_.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
    out_.push_back(static_cast<uint8_t>((v >> 16) & 0xFF));
    out_.push_back(static_cast<uint8_t>((v >> 24) & 0xFF));
  }

  void i32(int32_t v) { u32(static_cast<uint32_t>(v)); }
  void i16(int16_t v) { u16(static_cast<uint16_t>(v)); }
  void i8(int8_t v) { u8(static_cast<uint8_t>(v)); }

  void bytes(const uint8_t* data, size_t len) {
    out_.insert(out_.end(), data, data + len);
  }

  void bytes(const Bytes& data) { bytes(data.data(), data.size()); }

  // Zero-fill `count` bytes (reserved/padding fields).
  void pad(size_t count) { out_.insert(out_.end(), count, 0u); }

  size_t size() const { return out_.size(); }

private:
  Bytes& out_;
};

class ByteReader {
public:
  ByteReader(const uint8_t* data, size_t len) : data_(data), len_(len) {}
  explicit ByteReader(const Bytes& b) : data_(b.data()), len_(b.size()) {}

  bool u8(uint8_t& v) {
    if (remaining() < 1) return false;
    v = data_[pos_++];
    return true;
  }

  bool u16(uint16_t& v) {
    if (remaining() < 2) return false;
    v = static_cast<uint16_t>(data_[pos_]) |
        (static_cast<uint16_t>(data_[pos_ + 1]) << 8);
    pos_ += 2;
    return true;
  }

  bool u32(uint32_t& v) {
    if (remaining() < 4) return false;
    v = static_cast<uint32_t>(data_[pos_]) |
        (static_cast<uint32_t>(data_[pos_ + 1]) << 8) |
        (static_cast<uint32_t>(data_[pos_ + 2]) << 16) |
        (static_cast<uint32_t>(data_[pos_ + 3]) << 24);
    pos_ += 4;
    return true;
  }

  bool i32(int32_t& v) {
    uint32_t raw = 0;
    if (!u32(raw)) return false;
    v = static_cast<int32_t>(raw);
    return true;
  }

  bool i16(int16_t& v) {
    uint16_t raw = 0;
    if (!u16(raw)) return false;
    v = static_cast<int16_t>(raw);
    return true;
  }

  bool i8(int8_t& v) {
    uint8_t raw = 0;
    if (!u8(raw)) return false;
    v = static_cast<int8_t>(raw);
    return true;
  }

  // Copy `len` raw bytes into dst.
  bool bytes(uint8_t* dst, size_t len) {
    if (remaining() < len) return false;
    std::memcpy(dst, data_ + pos_, len);
    pos_ += len;
    return true;
  }

  // Copy `len` raw bytes into a fresh vector.
  bool bytes(Bytes& dst, size_t len) {
    if (remaining() < len) return false;
    dst.assign(data_ + pos_, data_ + pos_ + len);
    pos_ += len;
    return true;
  }

  bool skip(size_t len) {
    if (remaining() < len) return false;
    pos_ += len;
    return true;
  }

  size_t remaining() const { return len_ - pos_; }
  size_t position() const { return pos_; }
  const uint8_t* cursor() const { return data_ + pos_; }

private:
  const uint8_t* data_;
  size_t len_;
  size_t pos_ = 0;
};

}  // namespace eip

#endif  // ETHERNET_IP_EIP_BYTE_BUFFER_H

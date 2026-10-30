#ifndef APP_CONFIG_SET_H
#define APP_CONFIG_SET_H

// One complete parameter set as it sits in storage: a header, then one record
// per parameter. A key is a 32-bit hash so a set stays near 1.7 kB; three of
// them have to fit the 16 kB NVS partition. test_app_config rejects a hash
// collision. A key the firmware no longer has is skipped on read, and a key the
// set lacks keeps its compile-time default.

#include "ConfigStorage.h"
#include <cstdint>
#include <cstring>
#include <vector>

namespace Config {

constexpr int kConfigSetSlots = 3;
constexpr uint32_t kConfigSetMagic = 0x47434647u;
constexpr uint8_t kConfigSetFormat = 1;
constexpr size_t kConfigSetMaxBytes = 2048;

enum class ConfigSetSource : uint8_t { Edited = 0, Restored = 1, Defaults = 2 };

struct ConfigSetHeader {
    uint32_t magic = kConfigSetMagic;
    uint32_t seq = 0;           // higher is newer
    uint32_t unix_s = 0;        // newest change in the set, 0 = clock was not set
    uint32_t payload_hash = 0;
    uint16_t payload_len = 0;
    uint16_t param_count = 0;
    uint8_t format = kConfigSetFormat;
    uint8_t source = 0;
    uint16_t reserved = 0;
};

inline uint32_t configHash(const void* data, size_t len) {
    const uint8_t* p = static_cast<const uint8_t*>(data);
    uint32_t h = 2166136261u;
    for (size_t i = 0; i < len; ++i) {
        h = (h ^ p[i]) * 16777619u;
    }
    return h;
}

inline uint32_t configKeyHash(const char* key) {
    return configHash(key, std::strlen(key));
}

// A valid blob: known format, whole payload, matching hash.
inline bool parseConfigSet(const uint8_t* blob, size_t len, ConfigSetHeader& out) {
    if (len < sizeof(ConfigSetHeader)) {
        return false;
    }
    std::memcpy(&out, blob, sizeof(ConfigSetHeader));
    if (out.magic != kConfigSetMagic || out.format != kConfigSetFormat ||
        sizeof(ConfigSetHeader) + out.payload_len != len) {
        return false;
    }
    return configHash(blob + sizeof(ConfigSetHeader), out.payload_len) == out.payload_hash;
}

class ConfigSetWriter : public IParamIo {
public:
    bool getInt32(const char*, int32_t&) override { return false; }
    bool getFloat(const char*, float&) override { return false; }
    bool getBool(const char*, bool&) override { return false; }
    bool getString(const char*, char*, size_t) override { return false; }

    bool setInt32(const char* key, int32_t val) override {
        append(key, kTagInt, &val, sizeof(val));
        return true;
    }
    bool setFloat(const char* key, float val) override {
        append(key, kTagFloat, &val, sizeof(val));
        return true;
    }
    bool setBool(const char* key, bool val) override {
        const uint8_t raw = val ? 1 : 0;
        append(key, kTagBool, &raw, sizeof(raw));
        return true;
    }
    bool setString(const char* key, const char* val) override {
        const char* text = val != nullptr ? val : "";
        size_t len = std::strlen(text);
        if (len > 255) {
            len = 255;
        }
        const uint8_t raw_len = static_cast<uint8_t>(len);
        append(key, kTagString, &raw_len, sizeof(raw_len));
        const uint8_t* bytes = reinterpret_cast<const uint8_t*>(text);
        payload_.insert(payload_.end(), bytes, bytes + len);
        return true;
    }

    const std::vector<uint8_t>& payload() const { return payload_; }
    uint16_t count() const { return count_; }

    static constexpr uint8_t kTagInt = 0;
    static constexpr uint8_t kTagFloat = 1;
    static constexpr uint8_t kTagBool = 2;
    static constexpr uint8_t kTagString = 3;

private:
    void append(const char* key, uint8_t tag, const void* body, size_t len) {
        const uint32_t hash = configKeyHash(key);
        const uint8_t* h = reinterpret_cast<const uint8_t*>(&hash);
        payload_.insert(payload_.end(), h, h + sizeof(hash));
        payload_.push_back(tag);
        const uint8_t* bytes = static_cast<const uint8_t*>(body);
        payload_.insert(payload_.end(), bytes, bytes + len);
        ++count_;
    }

    std::vector<uint8_t> payload_;
    uint16_t count_ = 0;
};

class ConfigSetReader : public IParamIo {
public:
    ConfigSetReader(const uint8_t* payload, size_t len) : payload_(payload), len_(len) {}

    bool getInt32(const char* key, int32_t& out_val) override {
        const uint8_t* body = find(key, ConfigSetWriter::kTagInt);
        if (body == nullptr) {
            return false;
        }
        std::memcpy(&out_val, body, sizeof(out_val));
        return true;
    }
    bool getFloat(const char* key, float& out_val) override {
        const uint8_t* body = find(key, ConfigSetWriter::kTagFloat);
        if (body == nullptr) {
            return false;
        }
        std::memcpy(&out_val, body, sizeof(out_val));
        return true;
    }
    bool getBool(const char* key, bool& out_val) override {
        const uint8_t* body = find(key, ConfigSetWriter::kTagBool);
        if (body == nullptr) {
            return false;
        }
        out_val = *body != 0;
        return true;
    }
    bool getString(const char* key, char* out_buf, size_t max_len) override {
        const uint8_t* body = find(key, ConfigSetWriter::kTagString);
        if (body == nullptr || max_len == 0) {
            return false;
        }
        size_t len = body[0];
        if (len > max_len - 1) {
            len = max_len - 1;
        }
        std::memcpy(out_buf, body + 1, len);
        out_buf[len] = '\0';
        return true;
    }

    bool setInt32(const char*, int32_t) override { return false; }
    bool setFloat(const char*, float) override { return false; }
    bool setBool(const char*, bool) override { return false; }
    bool setString(const char*, const char*) override { return false; }

private:
    static constexpr size_t kRecordHead = sizeof(uint32_t) + 1;  // hash, tag

    const uint8_t* find(const char* key, uint8_t tag) const {
        const uint32_t want = configKeyHash(key);
        size_t i = 0;
        while (i + kRecordHead <= len_) {
            uint32_t hash = 0;
            std::memcpy(&hash, payload_ + i, sizeof(hash));
            const uint8_t rec_tag = payload_[i + sizeof(hash)];
            const uint8_t* body = payload_ + i + kRecordHead;
            size_t body_len = 0;
            if (rec_tag == ConfigSetWriter::kTagInt || rec_tag == ConfigSetWriter::kTagFloat) {
                body_len = 4;
            } else if (rec_tag == ConfigSetWriter::kTagBool) {
                body_len = 1;
            } else if (rec_tag == ConfigSetWriter::kTagString && i + kRecordHead < len_) {
                body_len = 1u + *body;
            } else {
                return nullptr;
            }
            if (i + kRecordHead + body_len > len_) {
                return nullptr;
            }
            if (hash == want && rec_tag == tag) {
                return body;
            }
            i += kRecordHead + body_len;
        }
        return nullptr;
    }

    const uint8_t* payload_;
    size_t len_;
};

} // namespace Config

#endif // APP_CONFIG_SET_H

#ifndef APP_CONFIG_MEMORY_STORAGE_H
#define APP_CONFIG_MEMORY_STORAGE_H

#include "ConfigStorage.h"
#include <map>
#include <string>
#include <vector>
#include <cstring>

namespace Config {

class MemoryStorage : public IConfigStorage {
public:
    bool init() override { return true; }

    bool getInt32(const char* key, int32_t& out_val) override {
        auto it = ints_.find(key);
        if (it != ints_.end()) {
            out_val = it->second;
            return true;
        }
        return false;
    }

    bool setInt32(const char* key, int32_t val) override {
        ints_[key] = val;
        return true;
    }

    bool getFloat(const char* key, float& out_val) override {
        auto it = floats_.find(key);
        if (it != floats_.end()) {
            out_val = it->second;
            return true;
        }
        return false;
    }

    bool setFloat(const char* key, float val) override {
        floats_[key] = val;
        return true;
    }

    bool getBool(const char* key, bool& out_val) override {
        auto it = bools_.find(key);
        if (it != bools_.end()) {
            out_val = it->second;
            return true;
        }
        return false;
    }

    bool setBool(const char* key, bool val) override {
        bools_[key] = val;
        return true;
    }

    bool getString(const char* key, char* out_buf, size_t max_len) override {
        auto it = strings_.find(key);
        if (it != strings_.end()) {
            std::strncpy(out_buf, it->second.c_str(), max_len - 1);
            out_buf[max_len - 1] = '\0';
            return true;
        }
        return false;
    }

    bool setString(const char* key, const char* val) override {
        strings_[key] = val ? val : "";
        return true;
    }

    bool getBlob(const char* key, void* out_buf, size_t& in_out_len) override {
        auto it = blobs_.find(key);
        if (it == blobs_.end() || it->second.size() > in_out_len) {
            return false;
        }
        std::memcpy(out_buf, it->second.data(), it->second.size());
        in_out_len = it->second.size();
        return true;
    }

    BlobRead readBlob(const char* key, void* out_buf, size_t& in_out_len) override {
        auto it = blobs_.find(key);
        if (it == blobs_.end()) return BlobRead::kNotFound;
        return getBlob(key, out_buf, in_out_len) ? BlobRead::kOk : BlobRead::kError;
    }

    bool setBlob(const char* key, const void* data, size_t len) override {
        const uint8_t* bytes = static_cast<const uint8_t*>(data);
        blobs_[key].assign(bytes, bytes + len);
        return true;
    }

    bool eraseKey(const char* key) override {
        const size_t erased = ints_.erase(key) + floats_.erase(key) + bools_.erase(key) +
                              strings_.erase(key) + blobs_.erase(key);
        return erased > 0;
    }

    bool commit() override { return true; }

private:
    std::map<std::string, int32_t> ints_;
    std::map<std::string, float> floats_;
    std::map<std::string, bool> bools_;
    std::map<std::string, std::string> strings_;
    std::map<std::string, std::vector<uint8_t>> blobs_;
};

} // namespace Config

#endif // APP_CONFIG_MEMORY_STORAGE_H

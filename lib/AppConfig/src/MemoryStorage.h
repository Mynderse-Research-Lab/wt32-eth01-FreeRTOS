#ifndef APP_CONFIG_MEMORY_STORAGE_H
#define APP_CONFIG_MEMORY_STORAGE_H

#include "ConfigStorage.h"
#include <map>
#include <string>
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

    bool eraseAll() override {
        ints_.clear();
        floats_.clear();
        bools_.clear();
        strings_.clear();
        return true;
    }

    bool commit() override { return true; }

private:
    std::map<std::string, int32_t> ints_;
    std::map<std::string, float> floats_;
    std::map<std::string, bool> bools_;
    std::map<std::string, std::string> strings_;
};

} // namespace Config

#endif // APP_CONFIG_MEMORY_STORAGE_H

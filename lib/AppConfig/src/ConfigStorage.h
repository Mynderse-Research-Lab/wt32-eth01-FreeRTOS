#ifndef APP_CONFIG_STORAGE_H
#define APP_CONFIG_STORAGE_H

#include <cstdint>
#include <cstddef>

namespace Config {

class IConfigStorage {
public:
    virtual ~IConfigStorage() = default;
    virtual bool init() = 0;
    virtual bool getInt32(const char* key, int32_t& out_val) = 0;
    virtual bool setInt32(const char* key, int32_t val) = 0;
    virtual bool getFloat(const char* key, float& out_val) = 0;
    virtual bool setFloat(const char* key, float val) = 0;
    virtual bool getBool(const char* key, bool& out_val) = 0;
    virtual bool setBool(const char* key, bool val) = 0;
    virtual bool getString(const char* key, char* out_buf, size_t max_len) = 0;
    virtual bool setString(const char* key, const char* val) = 0;
    virtual bool eraseAll() = 0;
    virtual bool commit() = 0;
};

} // namespace Config

#endif // APP_CONFIG_STORAGE_H

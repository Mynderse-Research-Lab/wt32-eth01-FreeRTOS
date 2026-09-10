#ifndef APP_CONFIG_NVS_STORAGE_H
#define APP_CONFIG_NVS_STORAGE_H

#include "ConfigStorage.h"

namespace Config {

class NvsStorage : public IConfigStorage {
public:
    explicit NvsStorage(const char* nvs_namespace = "gantry_cfg");
    ~NvsStorage() override;

    bool init() override;
    bool getInt32(const char* key, int32_t& out_val) override;
    bool setInt32(const char* key, int32_t val) override;
    bool getFloat(const char* key, float& out_val) override;
    bool setFloat(const char* key, float val) override;
    bool getBool(const char* key, bool& out_val) override;
    bool setBool(const char* key, bool val) override;
    bool getString(const char* key, char* out_buf, size_t max_len) override;
    virtual bool setString(const char* key, const char* val) override;
    bool eraseAll() override;
    bool commit() override;

private:
    const char* ns_;
    uint32_t handle_{0};
    bool initialized_{false};
};

} // namespace Config

#endif // APP_CONFIG_NVS_STORAGE_H

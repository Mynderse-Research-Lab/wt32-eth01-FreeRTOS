#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "AppConfigData.h"
#include "ConfigSchema.h"
#include "ConfigStorage.h"
#include <memory>
#include <string>

namespace Config {

class AppConfig {
public:
    static AppConfig& instance();

    /**
     * @brief Initialize storage and load settings.
     * If storage is nullptr, creates the default storage (NvsStorage on target, MemoryStorage on host).
     */
    bool init(std::unique_ptr<IConfigStorage> storage = nullptr);

    /**
     * @brief Reset in-memory values to compile-time Kconfig defaults.
     */
    void loadDefaults();

    /**
     * @brief Load all settings from storage. Falls back to defaults for missing keys.
     */
    bool load();

    /**
     * @brief Save all current settings to storage.
     */
    bool save();

    /**
     * @brief Erase storage and reset in-memory values to defaults.
     */
    bool factoryReset();

    /**
     * @brief Read-only access to current active configuration data.
     */
    const AppConfigData& data() const { return data_; }

    /**
     * @brief Mutable access (for direct internal configuration updates).
     */
    AppConfigData& mutableData() { return data_; }

    // ========================================================================
    // Developer Mode Security
    // ========================================================================
    bool isDeveloperModeUnlocked() const { return dev_unlocked_; }
    bool unlockDeveloperMode(const char* password);
    void lockDeveloperMode() { dev_unlocked_ = false; }
    bool changeDeveloperPassword(const char* old_pw, const char* new_pw);

    // ========================================================================
    // Generic Key-Value Access (used by UI menu system)
    // ========================================================================
    bool getParamInt(const char* key, int32_t& out_val) const;
    bool setParamInt(const char* key, int32_t val, bool& out_reboot_required);

    bool getParamFloat(const char* key, float& out_val) const;
    bool setParamFloat(const char* key, float val, bool& out_reboot_required);

    bool getParamBool(const char* key, bool& out_val) const;
    bool setParamBool(const char* key, bool val, bool& out_reboot_required);

    bool getParamString(const char* key, char* out_buf, size_t max_len) const;
    bool setParamString(const char* key, const char* val, bool& out_reboot_required);

private:
    AppConfig();
    ~AppConfig() = default;
    AppConfig(const AppConfig&) = delete;
    AppConfig& operator=(const AppConfig&) = delete;

    AppConfigData data_;
    std::unique_ptr<IConfigStorage> storage_;
    bool dev_unlocked_{false};
};

} // namespace Config

#endif // APP_CONFIG_H

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "AppConfigData.h"
#include "ConfigSchema.h"
#include "ConfigSet.h"
#include "ConfigStorage.h"
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace Config {

// The stored parameter set, for the boot log and 'cfg export'.
struct ConfigSetInfo {
    bool valid = false;
    bool damaged = false;       // a set is stored but failed its hash
    uint32_t seq = 0;
    uint32_t unix_s = 0;        // newest change in the set, 0 = clock was not set
    uint16_t param_count = 0;
};

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
     * @brief Load the stored set and overlay any per-key values older firmware
     * left in NVS, storing the result if it differs. Compile-time defaults only
     * fill keys the set does not hold. A damaged set is left in place and the
     * firmware runs on defaults until an operator saves over it.
     */
    bool load();

    /**
     * @brief Drop the keys merged at boot. Call once the boot is confirmed, so
     * a rollback to an image that reads those keys still finds them.
     */
    void commitPending();

    /**
     * @brief Write the running values to NVS as the stored set.
     *
     * Parameter changes stay in RAM until this runs: a flash write can erase a
     * sector, and that stall outlasts the 32 ms Class 1 timeout. Callers close
     * Class 1 first; requestSave() goes through the handler that does.
     */
    bool saveSet();

    /**
     * @brief Run the registered save, which makes the write safe for Class 1.
     */
    bool requestSave();
    void setSaveHandler(std::function<bool()> handler) { save_handler_ = std::move(handler); }

    /**
     * @brief Revert the running values to compile-time defaults. The stored set
     * is untouched until saveSet().
     */
    bool factoryReset();

    /**
     * @brief True while settings have persistent storage behind them.
     */
    bool hasStorage() const { return storage_ != nullptr; }

    /**
     * @brief The stored set the running values came from.
     */
    ConfigSetInfo activeSet() const;

    /**
     * @brief Changes not yet in the stored set. A reboot discards them.
     */
    int pendingChanges() const;

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

    /**
     * @brief True when the key exists and the current developer lock allows it.
     */
    bool isParamVisible(const char* key) const;

    /**
     * @brief Write the current value into out_buf (no units). Hidden keys fail.
     */
    bool formatParamValue(const char* key, char* out_buf, size_t max_len) const;

    /**
     * @brief Parse a console/UI text value, apply bounds, and persist.
     * Developer keys fail while developer mode is locked.
     */
    bool applyParamText(const char* key, const char* text, bool& out_reboot_required);

private:
    AppConfig();
    ~AppConfig() = default;
    AppConfig(const AppConfig&) = delete;
    AppConfig& operator=(const AppConfig&) = delete;

    void readParams(IParamIo& io);
    void writeParams(IParamIo& io) const;
    bool writeSet(const std::vector<uint8_t>& payload, uint16_t param_count, uint32_t unix_s);
    void recordChange(const char* key);

    AppConfigData data_;
    std::unique_ptr<IConfigStorage> storage_;
    bool dev_unlocked_{false};
    ConfigSetHeader set_{};
    bool set_valid_{false};
    bool set_damaged_{false};
    bool set_saved_{false};              // the boot found or wrote a good set
    bool defaults_unsaved_{false};       // factoryReset() has not been saved yet
    mutable std::mutex change_mutex_;    // the change lists below
    std::vector<std::string> merged_;    // per-key values merged into the set at boot
    std::vector<std::string> changed_;   // keys changed since the last save
    std::function<bool()> save_handler_;
};

} // namespace Config

#endif // APP_CONFIG_H

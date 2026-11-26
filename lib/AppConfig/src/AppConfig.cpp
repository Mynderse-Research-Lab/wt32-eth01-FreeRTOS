/**
 * @file AppConfig.cpp
 * @brief AppConfig instance, init, load and save of the NVS set, developer mode.
 *
 * One of the AppConfig*.cpp files that implement Config::AppConfig;
 * AppConfigInternal.h holds what they share.
 */

#include "AppConfigInternal.h"

namespace Config {

namespace {

// The set lives in NVS under this key, and only saveSet() and boot write it.
constexpr char kSetKey[] = "cfg_set";
// Left by firmware that wrote each change to NVS; cleaned up at boot.
constexpr char kOldRevertKey[] = "cfg_revert";
constexpr char kChangedAtKey[] = "cfg_chg_unix";
// Anything earlier means nobody has set the clock since power-up.
constexpr std::time_t kClockValidAfter = 1700000000;

uint32_t clockUnixSeconds() {
    const std::time_t now = std::time(nullptr);
    return now > kClockValidAfter ? static_cast<uint32_t>(now) : 0;
}

// Reads through to the store and remembers which keys it held.
class FoundKeys : public IParamIo {
public:
    explicit FoundKeys(IParamIo& inner) : inner_(inner) {}

    bool getInt32(const char* key, int32_t& out_val) override {
        return note(key, inner_.getInt32(key, out_val));
    }
    bool getFloat(const char* key, float& out_val) override {
        return note(key, inner_.getFloat(key, out_val));
    }
    bool getBool(const char* key, bool& out_val) override {
        return note(key, inner_.getBool(key, out_val));
    }
    bool getString(const char* key, char* out_buf, size_t max_len) override {
        return note(key, inner_.getString(key, out_buf, max_len));
    }

    bool setInt32(const char*, int32_t) override { return false; }
    bool setFloat(const char*, float) override { return false; }
    bool setBool(const char*, bool) override { return false; }
    bool setString(const char*, const char*) override { return false; }

    std::vector<std::string> take() { return std::move(keys_); }

private:
    bool note(const char* key, bool found) {
        if (found) {
            keys_.emplace_back(key);
        }
        return found;
    }

    IParamIo& inner_;
    std::vector<std::string> keys_;
};

}  // namespace

AppConfig& AppConfig::instance() {
    static AppConfig s_instance;
    return s_instance;
}

AppConfig::AppConfig() {
    loadDefaults();
}

void AppConfig::loadDefaults() {
    loadMotionDefaults();
    loadCellDefaults();
}

bool AppConfig::init(std::unique_ptr<IConfigStorage> storage) {
    if (storage) {
        storage_ = std::move(storage);
    } else {
#if defined(ESP_PLATFORM)
        storage_ = std::make_unique<NvsStorage>("gantry_cfg");
#else
        storage_ = std::make_unique<MemoryStorage>();
#endif
    }
    if (!storage_->init()) {
        storage_.reset();
        return false;
    }
    return load();
}

bool AppConfig::load() {
    if (!storage_) return false;
    loadDefaults();

    std::vector<uint8_t> blob(kConfigSetMaxBytes);
    size_t len = blob.size();
    const bool stored = storage_->getBlob(kSetKey, blob.data(), len);
    set_valid_ = stored && parseConfigSet(blob.data(), len, set_);
    set_damaged_ = stored && !set_valid_;
    if (set_valid_) {
        ConfigSetReader reader(blob.data() + sizeof(ConfigSetHeader), set_.payload_len);
        readParams(reader);
    }

    // Only firmware from before saveSet() left per-key values behind.
    FoundKeys found(*storage_);
    readParams(found);
    int32_t changed_at = 0;
    storage_->getInt32(kChangedAtKey, changed_at);
    {
        std::lock_guard<std::mutex> lock(change_mutex_);
        merged_ = found.take();
        changed_.clear();
        defaults_unsaved_ = false;
    }
    if (set_damaged_) {
        set_saved_ = false;
        return true;
    }

    ConfigSetWriter writer;
    writeParams(writer);
    const std::vector<uint8_t>& payload = writer.payload();
    const bool unchanged =
        set_valid_ && set_.payload_len == payload.size() &&
        std::memcmp(blob.data() + sizeof(ConfigSetHeader), payload.data(), payload.size()) == 0;
    const uint32_t stamp = changed_at != 0 ? static_cast<uint32_t>(changed_at) : clockUnixSeconds();
    set_saved_ = unchanged || writeSet(payload, writer.count(), stamp);
    return true;
}

bool AppConfig::writeSet(const std::vector<uint8_t>& payload, uint16_t param_count,
                         uint32_t unix_s) {
    if (sizeof(ConfigSetHeader) + payload.size() > kConfigSetMaxBytes) {
        return false;
    }
    ConfigSetHeader header;
    header.seq = set_valid_ ? set_.seq + 1 : 1;
    header.unix_s = unix_s;
    header.payload_len = static_cast<uint16_t>(payload.size());
    header.payload_hash = configHash(payload.data(), payload.size());
    header.param_count = param_count;

    std::vector<uint8_t> blob(sizeof(header) + payload.size());
    std::memcpy(blob.data(), &header, sizeof(header));
    std::memcpy(blob.data() + sizeof(header), payload.data(), payload.size());
    if (!storage_->setBlob(kSetKey, blob.data(), blob.size()) || !storage_->commit()) {
        return false;
    }
    set_ = header;
    set_valid_ = true;
    set_damaged_ = false;
    return true;
}

void AppConfig::commitPending() {
    if (!storage_ || !set_saved_) return;
    std::lock_guard<std::mutex> lock(change_mutex_);
    for (const std::string& key : merged_) {
        storage_->eraseKey(key.c_str());
    }
    merged_.clear();
    storage_->eraseKey(kChangedAtKey);
    storage_->eraseKey(kOldRevertKey);
    storage_->commit();
}

void AppConfig::recordChange(const char* key) {
    std::lock_guard<std::mutex> lock(change_mutex_);
    if (std::find(changed_.begin(), changed_.end(), key) == changed_.end()) {
        changed_.emplace_back(key);
    }
}

bool AppConfig::saveSet() {
    if (!storage_) return false;
    ConfigSetWriter writer;
    writeParams(writer);
    const std::vector<uint8_t>& payload = writer.payload();
    const bool unchanged = set_valid_ && set_.payload_len == payload.size() &&
                           set_.payload_hash == configHash(payload.data(), payload.size());
    if (!unchanged && !writeSet(payload, writer.count(), clockUnixSeconds())) {
        return false;
    }
    std::lock_guard<std::mutex> lock(change_mutex_);
    changed_.clear();
    defaults_unsaved_ = false;
    set_saved_ = true;
    return true;
}

bool AppConfig::requestSave() {
    return save_handler_ ? save_handler_() : false;
}

bool AppConfig::factoryReset() {
    loadDefaults();
    std::lock_guard<std::mutex> lock(change_mutex_);
    defaults_unsaved_ = true;
    return true;
}

ConfigSetInfo AppConfig::activeSet() const {
    ConfigSetInfo info;
    info.valid = set_valid_;
    info.damaged = set_damaged_;
    if (set_valid_) {
        info.seq = set_.seq;
        info.unix_s = set_.unix_s;
        info.param_count = set_.param_count;
    }
    return info;
}

int AppConfig::pendingChanges() const {
    std::lock_guard<std::mutex> lock(change_mutex_);
    return static_cast<int>(changed_.size()) + (defaults_unsaved_ ? 1 : 0);
}

bool AppConfig::unlockDeveloperMode(const char* password) {
    if (password == nullptr) return false;
    if (std::strcmp(password, data_.dev_mode_password) == 0) {
        dev_unlocked_ = true;
        return true;
    }
    return false;
}

bool AppConfig::changeDeveloperPassword(const char* old_pw, const char* new_pw) {
    if (!old_pw || !new_pw || std::strlen(new_pw) == 0) return false;
    if (std::strcmp(old_pw, data_.dev_mode_password) != 0) return false;

    std::strncpy(data_.dev_mode_password, new_pw, sizeof(data_.dev_mode_password) - 1);
    data_.dev_mode_password[sizeof(data_.dev_mode_password) - 1] = '\0';
    recordChange("dev_pw");
    return true;
}

} // namespace Config

#include "NvsStorage.h"
#include <cstring>

#if defined(ESP_PLATFORM)
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"

static const char* TAG = "NvsStorage";

namespace Config {

NvsStorage::NvsStorage(const char* nvs_namespace)
    : ns_(nvs_namespace ? nvs_namespace : "gantry_cfg") {}

NvsStorage::~NvsStorage() {
    if (initialized_ && handle_ != 0) {
        nvs_close(static_cast<nvs_handle_t>(handle_));
        handle_ = 0;
        initialized_ = false;
    }
}

bool NvsStorage::init() {
    if (initialized_) return true;

    nvs_handle_t h;
    esp_err_t err = nvs_open(ns_, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS namespace '%s': %s", ns_, esp_err_to_name(err));
        return false;
    }
    handle_ = static_cast<uint32_t>(h);
    initialized_ = true;
    return true;
}

bool NvsStorage::getInt32(const char* key, int32_t& out_val) {
    if (!initialized_ || !key) return false;
    esp_err_t err = nvs_get_i32(static_cast<nvs_handle_t>(handle_), key, &out_val);
    return (err == ESP_OK);
}

bool NvsStorage::setInt32(const char* key, int32_t val) {
    if (!initialized_ || !key) return false;
    esp_err_t err = nvs_set_i32(static_cast<nvs_handle_t>(handle_), key, val);
    return (err == ESP_OK);
}

bool NvsStorage::getFloat(const char* key, float& out_val) {
    if (!initialized_ || !key) return false;
    size_t size = sizeof(float);
    esp_err_t err = nvs_get_blob(static_cast<nvs_handle_t>(handle_), key, &out_val, &size);
    return (err == ESP_OK && size == sizeof(float));
}

bool NvsStorage::setFloat(const char* key, float val) {
    if (!initialized_ || !key) return false;
    esp_err_t err = nvs_set_blob(static_cast<nvs_handle_t>(handle_), key, &val, sizeof(float));
    return (err == ESP_OK);
}

bool NvsStorage::getBool(const char* key, bool& out_val) {
    if (!initialized_ || !key) return false;
    uint8_t raw = 0;
    esp_err_t err = nvs_get_u8(static_cast<nvs_handle_t>(handle_), key, &raw);
    if (err == ESP_OK) {
        out_val = (raw != 0);
        return true;
    }
    return false;
}

bool NvsStorage::setBool(const char* key, bool val) {
    if (!initialized_ || !key) return false;
    uint8_t raw = val ? 1 : 0;
    esp_err_t err = nvs_set_u8(static_cast<nvs_handle_t>(handle_), key, raw);
    return (err == ESP_OK);
}

bool NvsStorage::getString(const char* key, char* out_buf, size_t max_len) {
    if (!initialized_ || !key || !out_buf || max_len == 0) return false;
    size_t required = 0;
    esp_err_t err = nvs_get_str(static_cast<nvs_handle_t>(handle_), key, nullptr, &required);
    if (err != ESP_OK || required > max_len) return false;
    err = nvs_get_str(static_cast<nvs_handle_t>(handle_), key, out_buf, &required);
    return (err == ESP_OK);
}

bool NvsStorage::setString(const char* key, const char* val) {
    if (!initialized_ || !key || !val) return false;
    esp_err_t err = nvs_set_str(static_cast<nvs_handle_t>(handle_), key, val);
    return (err == ESP_OK);
}

bool NvsStorage::eraseAll() {
    if (!initialized_) return false;
    esp_err_t err = nvs_erase_all(static_cast<nvs_handle_t>(handle_));
    if (err == ESP_OK) {
        nvs_commit(static_cast<nvs_handle_t>(handle_));
        return true;
    }
    return false;
}

bool NvsStorage::commit() {
    if (!initialized_) return false;
    return (nvs_commit(static_cast<nvs_handle_t>(handle_)) == ESP_OK);
}

} // namespace Config

#else // Non-ESP target fallback

namespace Config {

NvsStorage::NvsStorage(const char* nvs_namespace) : ns_(nvs_namespace) {}
NvsStorage::~NvsStorage() {}
bool NvsStorage::init() { return true; }
bool NvsStorage::getInt32(const char*, int32_t&) { return false; }
bool NvsStorage::setInt32(const char*, int32_t) { return true; }
bool NvsStorage::getFloat(const char*, float&) { return false; }
bool NvsStorage::setFloat(const char*, float) { return true; }
bool NvsStorage::getBool(const char*, bool&) { return false; }
bool NvsStorage::setBool(const char*, bool) { return true; }
bool NvsStorage::getString(const char*, char*, size_t) { return false; }
bool NvsStorage::setString(const char*, const char*) { return true; }
bool NvsStorage::eraseAll() { return true; }
bool NvsStorage::commit() { return true; }

} // namespace Config

#endif

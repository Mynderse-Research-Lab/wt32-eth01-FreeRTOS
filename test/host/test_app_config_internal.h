/**
 * @file test_app_config_internal.h
 * @brief Shared storage and boot helpers for the AppConfig host tests.
 */

#pragma once

#include "unity.h"
#include "AppConfig.h"
#include "MemoryStorage.h"
#include "ConfigSchema.h"
#include "gantry_runtime_params.h"

#include <cstdlib>
#include <cstring>
#include <set>
#include <string>
#include <vector>

using namespace Config;

namespace app_config_test {

// Survives AppConfig::init(), the way NVS survives a reboot.
class SharedStorage : public IConfigStorage {
public:
    explicit SharedStorage(MemoryStorage& mem) : mem_(mem) {}
    bool init() override { return true; }
    bool getInt32(const char* k, int32_t& v) override { return mem_.getInt32(k, v); }
    bool setInt32(const char* k, int32_t v) override {
        ++flash_writes;
        return mem_.setInt32(k, v);
    }
    bool getFloat(const char* k, float& v) override { return mem_.getFloat(k, v); }
    bool setFloat(const char* k, float v) override { return mem_.setFloat(k, v); }
    bool getBool(const char* k, bool& v) override { return mem_.getBool(k, v); }
    bool setBool(const char* k, bool v) override { return mem_.setBool(k, v); }
    bool getString(const char* k, char* b, size_t n) override { return mem_.getString(k, b, n); }
    bool setString(const char* k, const char* v) override { return mem_.setString(k, v); }
    bool getBlob(const char* k, void* b, size_t& n) override { return mem_.getBlob(k, b, n); }
    BlobRead readBlob(const char* k, void* b, size_t& n) override {
        if (fail_blob_reads) {
            return BlobRead::kError;
        }
        return mem_.readBlob(k, b, n);
    }
    bool setBlob(const char* k, const void* d, size_t n) override {
        if (fail_blobs) {
            return false;
        }
        ++flash_writes;
        return mem_.setBlob(k, d, n);
    }
    bool eraseKey(const char* k) override { return mem_.eraseKey(k); }
    bool commit() override { return true; }

    static bool fail_blobs;
    static bool fail_blob_reads;
    static uint32_t flash_writes;

private:
    MemoryStorage& mem_;
};

inline
bool SharedStorage::fail_blobs = false;
inline
bool SharedStorage::fail_blob_reads = false;
inline
uint32_t SharedStorage::flash_writes = 0;
inline
MemoryStorage* s_nvs = nullptr;

inline
void powerOn() {
    delete s_nvs;
    s_nvs = new MemoryStorage();
    SharedStorage::fail_blobs = false;
    SharedStorage::fail_blob_reads = false;
}

// Boot without the confirmation that drops merged keys.
inline
void bootOnly() {
    AppConfig::instance().init(std::make_unique<SharedStorage>(*s_nvs));
}

inline
void boot() {
    bootOnly();
    AppConfig::instance().commitPending();
}

inline
void set(const char* key, int32_t val) {
    bool reboot = false;
    TEST_ASSERT_TRUE(AppConfig::instance().setParamInt(key, val, reboot));
}

inline
int speed() {
    return AppConfig::instance().data().gantry_default_speed_mm_per_s;
}

inline
uint32_t activeSeq() {
    return AppConfig::instance().activeSet().seq;
}

}  // namespace

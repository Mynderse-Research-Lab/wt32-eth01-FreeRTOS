/**
 * @file AppConfigText.cpp
 * @brief Parameter values as text: visibility, formatting and parsing for cfg.
 *
 * One of the AppConfig*.cpp files that implement Config::AppConfig;
 * AppConfigInternal.h holds what they share.
 */

#include "AppConfigInternal.h"

namespace Config {

namespace {

bool equalsIgnoreCase(const char* a, const char* b) {
    if (a == nullptr || b == nullptr) {
        return false;
    }
    while (*a != '\0' && *b != '\0') {
        if (std::tolower(static_cast<unsigned char>(*a)) !=
            std::tolower(static_cast<unsigned char>(*b))) {
            return false;
        }
        ++a;
        ++b;
    }
    return *a == '\0' && *b == '\0';
}

const char* skipSpace(const char* s) {
    while (s != nullptr && *s != '\0' &&
           std::isspace(static_cast<unsigned char>(*s))) {
        ++s;
    }
    return s;
}

bool parseBoolText(const char* text, bool& out) {
    if (equalsIgnoreCase(text, "1") || equalsIgnoreCase(text, "on") ||
        equalsIgnoreCase(text, "true") || equalsIgnoreCase(text, "yes")) {
        out = true;
        return true;
    }
    if (equalsIgnoreCase(text, "0") || equalsIgnoreCase(text, "off") ||
        equalsIgnoreCase(text, "false") || equalsIgnoreCase(text, "no")) {
        out = false;
        return true;
    }
    return false;
}

bool consumedNumber(const char* end) {
    end = skipSpace(end);
    return end != nullptr && *end == '\0';
}

} // namespace

bool AppConfig::isParamVisible(const char* key) const {
    const ParamSchema* schema = findParamSchema(key);
    if (schema == nullptr) {
        return false;
    }
    if (schema->access == AccessLevel::DEVELOPER && !dev_unlocked_) {
        return false;
    }
    return true;
}

bool AppConfig::formatParamValue(const char* key, char* out_buf, size_t max_len) const {
    if (out_buf == nullptr || max_len == 0 || !isParamVisible(key)) {
        return false;
    }
    const ParamSchema* schema = findParamSchema(key);
    if (schema == nullptr) {
        return false;
    }

    if (schema->type == ParamType::INT) {
        int32_t value = 0;
        if (!getParamInt(key, value)) {
            return false;
        }
        std::snprintf(out_buf, max_len, "%ld", static_cast<long>(value));
        return true;
    }
    if (schema->type == ParamType::FLOAT) {
        float value = 0.0f;
        if (!getParamFloat(key, value)) {
            return false;
        }
        std::snprintf(out_buf, max_len, "%.2f", static_cast<double>(value));
        return true;
    }
    if (schema->type == ParamType::BOOL) {
        bool value = false;
        if (!getParamBool(key, value)) {
            return false;
        }
        std::snprintf(out_buf, max_len, "%s", value ? "ON" : "OFF");
        return true;
    }
    if (schema->type == ParamType::STRING) {
        return getParamString(key, out_buf, max_len);
    }
    if (schema->type == ParamType::CHOICE) {
        int32_t value = 0;
        if (!getParamInt(key, value)) {
            return false;
        }
        if (value >= 0 && value < static_cast<int32_t>(schema->choices.size())) {
            std::snprintf(out_buf, max_len, "%s", schema->choices[static_cast<size_t>(value)]);
        } else {
            std::snprintf(out_buf, max_len, "%ld", static_cast<long>(value));
        }
        return true;
    }
    return false;
}

bool AppConfig::applyParamText(const char* key, const char* text, bool& out_reboot_required) {
    if (!isParamVisible(key) || text == nullptr) {
        return false;
    }
    const ParamSchema* schema = findParamSchema(key);
    if (schema == nullptr) {
        return false;
    }
    const char* body = skipSpace(text);
    if (body == nullptr || *body == '\0') {
        return false;
    }

    if (schema->type == ParamType::BOOL) {
        bool value = false;
        if (!parseBoolText(body, value)) {
            return false;
        }
        return setParamBool(key, value, out_reboot_required);
    }
    if (schema->type == ParamType::INT) {
        char* end = nullptr;
        long parsed = std::strtol(body, &end, 10);
        if (!consumedNumber(end)) {
            return false;
        }
        return setParamInt(key, static_cast<int32_t>(parsed), out_reboot_required);
    }
    if (schema->type == ParamType::CHOICE) {
        char* end = nullptr;
        long parsed = std::strtol(body, &end, 10);
        if (consumedNumber(end) && end != body) {
            return setParamInt(key, static_cast<int32_t>(parsed), out_reboot_required);
        }
        for (size_t i = 0; i < schema->choices.size(); ++i) {
            if (equalsIgnoreCase(body, schema->choices[i])) {
                return setParamInt(key, static_cast<int32_t>(i), out_reboot_required);
            }
        }
        return false;
    }
    if (schema->type == ParamType::FLOAT) {
        char* end = nullptr;
        float parsed = std::strtof(body, &end);
        if (!consumedNumber(end)) {
            return false;
        }
        return setParamFloat(key, parsed, out_reboot_required);
    }
    if (schema->type == ParamType::STRING) {
        return setParamString(key, body, out_reboot_required);
    }
    return false;
}

} // namespace Config

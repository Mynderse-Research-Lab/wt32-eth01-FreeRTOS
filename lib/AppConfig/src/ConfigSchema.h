#ifndef APP_CONFIG_SCHEMA_H
#define APP_CONFIG_SCHEMA_H

#include <cstdint>
#include <vector>

namespace Config {

enum class ParamType {
    INT,
    FLOAT,
    BOOL,
    STRING,
    CHOICE
};

enum class AccessLevel {
    END_USER,
    DEVELOPER
};

struct ParamSchema {
    const char* key;             // NVS key (<= 15 characters)
    const char* label;           // Human-readable title for UI
    const char* section;         // Menu category / section
    ParamType type;              // Value data type
    AccessLevel access;          // END_USER vs DEVELOPER
    bool reboot_required;        // Triggers "Reboot now?" prompt if changed
    float min_val;               // Min bound (for numeric)
    float max_val;               // Max bound (for numeric)
    float step;                  // Step increment for rotary editor
    const char* units;           // Display units ("mm", "mm/s", "deg", etc.)
    std::vector<const char*> choices; // Options for CHOICE type
    const char* description = nullptr;  // What the value does; shown by 'cfg <key>'
};

const std::vector<ParamSchema>& getParamSchemas();
const ParamSchema* findParamSchema(const char* key);

/// Keys renamed on 2026-11-16: camera, vision and jaw keys left the cv_ conveyor prefix.
struct RenamedParamKey {
    const char* old_key;
    const char* new_key;
};
const std::vector<RenamedParamKey>& getRenamedParamKeys();
/// New name for a renamed key, or nullptr.
const char* renamedParamKey(const char* old_key);
/// Name a key had before it was renamed, or nullptr.
const char* legacyParamKey(const char* new_key);

} // namespace Config

#endif // APP_CONFIG_SCHEMA_H

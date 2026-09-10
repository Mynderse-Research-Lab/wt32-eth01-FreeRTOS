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
};

const std::vector<ParamSchema>& getParamSchemas();
const ParamSchema* findParamSchema(const char* key);

} // namespace Config

#endif // APP_CONFIG_SCHEMA_H

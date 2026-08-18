/**
 * @file gantry_console_parse.h
 * @brief Host-testable UART console argument parsing (no ESP-IDF).
 */
#pragma once

#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace GantryConsole {

enum class LinearUnitMode { MM = 0, INCH = 1 };
enum class AxisToken { X, Z, THETA };

inline constexpr float kMmPerInch = 25.4f;
inline constexpr uint32_t kMinSpeedMmPerS = 1;
inline constexpr uint32_t kMinAccelMmPerS2 = 100;
inline constexpr uint32_t kMinAccelDegPerS2 = 1;

inline const char* linearUnitLabel(LinearUnitMode mode) {
    return (mode == LinearUnitMode::INCH) ? "in" : "mm";
}

inline float convertMmToSelected(float value_mm, LinearUnitMode mode) {
    if (mode == LinearUnitMode::INCH) {
        return value_mm / kMmPerInch;
    }
    return value_mm;
}

inline float convertSelectedToMm(float value_selected, LinearUnitMode mode) {
    if (mode == LinearUnitMode::INCH) {
        return value_selected * kMmPerInch;
    }
    return value_selected;
}

inline uint32_t applyRangeLimitU32(uint32_t value, uint32_t min_value,
                                   uint32_t max_value, bool enabled) {
    if (!enabled) {
        return value;
    }
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

inline bool parseAxisToken(const char* tok, AxisToken& out) {
    if (tok == nullptr) {
        return false;
    }
    if (std::strcmp(tok, "x") == 0) {
        out = AxisToken::X;
        return true;
    }
    if (std::strcmp(tok, "z") == 0) {
        out = AxisToken::Z;
        return true;
    }
    if (std::strcmp(tok, "t") == 0 || std::strcmp(tok, "theta") == 0) {
        out = AxisToken::THETA;
        return true;
    }
    return false;
}

inline bool parseLinearUnit(const char* unit, LinearUnitMode& out) {
    if (unit == nullptr) {
        return false;
    }
    if (std::strcmp(unit, "mm") == 0) {
        out = LinearUnitMode::MM;
        return true;
    }
    if (std::strcmp(unit, "in") == 0 || std::strcmp(unit, "inch") == 0 ||
        std::strcmp(unit, "inches") == 0) {
        out = LinearUnitMode::INCH;
        return true;
    }
    return false;
}

struct SpeedParse {
    bool ok = false;
    bool has_deg = false;
    int speed_mm = 0;
    int speed_deg = 0;
};

inline SpeedParse parseSpeedCommand(const char* cmd) {
    SpeedParse p;
    int parsed = std::sscanf(cmd, "speed %d %d", &p.speed_mm, &p.speed_deg);
    if (parsed < 1 || p.speed_mm <= 0) {
        return p;
    }
    p.ok = true;
    p.has_deg = (parsed >= 2 && p.speed_deg > 0);
    return p;
}

struct AccelParse {
    bool ok = false;
    int n = 0;
    int accel = 0;
    int decel = 0;
    int accel_deg = 0;
    int decel_deg = 0;
};

inline AccelParse parseAccelCommand(const char* cmd) {
    AccelParse p;
    p.n = std::sscanf(cmd, "accel %d %d %d %d", &p.accel, &p.decel,
                      &p.accel_deg, &p.decel_deg);
    if (p.n < 1 || p.accel <= 0) {
        return p;
    }
    if (p.n >= 2 && p.decel <= 0) {
        return p;
    }
    if (p.n >= 3 && p.accel_deg <= 0) {
        return p;
    }
    if (p.n >= 4 && p.decel_deg <= 0) {
        return p;
    }
    p.ok = true;
    return p;
}

struct MoveParse {
    bool ok = false;
    float x = 0.0f;
    float z = 0.0f;
    float theta = 0.0f;
};

inline MoveParse parseMoveCommand(const char* cmd) {
    MoveParse p;
    const int n = std::sscanf(cmd, "move %f %f %f", &p.x, &p.z, &p.theta);
    p.ok = (n >= 3);
    return p;
}

struct ThetaLimParse {
    bool ok = false;
    float min_deg = 0.0f;
    float max_deg = 0.0f;
};

inline ThetaLimParse parseThetaLimCommand(const char* cmd) {
    ThetaLimParse p;
    if (std::sscanf(cmd, "thetalim %f %f", &p.min_deg, &p.max_deg) != 2) {
        return p;
    }
    p.ok = (p.min_deg < p.max_deg);
    return p;
}

struct PuuCalParse {
    bool ok = false;
    char axis = '\0';
    float commanded = 0.0f;
    float measured = 0.0f;
};

inline PuuCalParse parsePuuCalCommand(const char* cmd) {
    PuuCalParse p;
    if (std::sscanf(cmd, "puucal %c %f %f", &p.axis, &p.commanded, &p.measured) != 3) {
        return p;
    }
    p.axis = static_cast<char>(std::tolower(static_cast<unsigned char>(p.axis)));
    p.ok = (p.axis == 'x' || p.axis == 'z' || p.axis == 't') &&
           p.commanded > 0.0f && p.measured > 0.0f;
    return p;
}

inline bool suggestPuuScale(double current, float commanded, float measured,
                            double& suggested) {
    if (current <= 0.0 || commanded <= 0.0f || measured <= 0.0f) {
        return false;
    }
    suggested = current * (static_cast<double>(commanded) /
                           static_cast<double>(measured));
    return true;
}

}  // namespace GantryConsole

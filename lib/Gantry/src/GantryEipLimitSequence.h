/**
 * @file GantryEipLimitSequence.h
 * @brief Host-testable EIP drive-managed home/cal phase decisions.
 *
 * Joint-space seek signs (both axes):
 *   Home seeks joint min (negative delta).
 *   Cal seeks joint max (positive delta).
 * Clear-edge creeps reverse until the limit warning deasserts. Joint 0 is
 * that sample — the moment the switch is disabled — with no extra offset.
 *
 * Warning map (drive TBIO after assign):
 *   X: A014/PL = joint min, A015/NL = joint max.
 *   Z: A015/NL = joint min (retract / −Z), A014/PL = joint max (+Z down).
 * SAFE_Z band is from Z− / A015 (retract); that is geometry, not warning polarity.
 * Z Absolute joint sense is inverted so homeSeek (−joint) moves toward A015.
 */
#pragma once

#include <cstdint>

namespace Gantry {
namespace EipLimit {

enum class Phase : uint8_t {
    kIdle = 0,
    kHomeSeekMin,
    kHomeCreepClear,
    kHomeSettle,
    kCalSeekMax,
    kCalCreepClear,
    kCalReturnZero,
};

enum class AxisRole : uint8_t {
    kNone = 0,
    kX,
    kZ,
};

/// Absolute seek distance beyond any soft stroke (drive trips on TBIO).
inline constexpr float kSeekTravelMm = 600.0f;

inline constexpr float homeSeekDeltaMm() { return -kSeekTravelMm; }
inline constexpr float homeCreepDeltaMm() { return +kSeekTravelMm; }
inline constexpr float calSeekDeltaMm() { return +kSeekTravelMm; }
inline constexpr float calCreepDeltaMm() { return -kSeekTravelMm; }

inline const char* axisLetter(AxisRole role) {
    return role == AxisRole::kZ ? "Z" : "X";
}

/// Joint-min / joint-max warning labels (drive PL/NL after TBIO assign).
inline const char* minWarningLabel(AxisRole role) {
    return role == AxisRole::kZ ? "A015/NL" : "A014/PL";
}
inline const char* maxWarningLabel(AxisRole role) {
    return role == AxisRole::kZ ? "A014/PL" : "A015/NL";
}
/// True when this axis maps A015 to joint min (Z only).
inline bool jointMinIsA015(AxisRole role) {
    return role == AxisRole::kZ;
}

/// Outcomes while seeking the min (home) endstop.
enum class SeekOutcome : uint8_t {
    kWait,          // still moving or waiting to arm move
    kStartMove,     // caller should start Absolute delta
    kTripped,       // min/max warning active -> enter creep phase
    kFailNoTrip,    // busy ended without trip
};

inline SeekOutcome evaluateSeek(bool endWarning, bool busy, bool sawBusy) {
    if (endWarning) {
        return SeekOutcome::kTripped;
    }
    if (busy) {
        return SeekOutcome::kWait;  // caller sets sawBusy
    }
    if (!sawBusy) {
        return SeekOutcome::kStartMove;
    }
    return SeekOutcome::kFailNoTrip;
}

/// Outcomes while creeping clear of an endstop warning.
enum class CreepOutcome : uint8_t {
    kWait,
    kStartMove,
    kCleared,  // warning gone = switch disabled → latch joint 0 here
};

inline CreepOutcome evaluateCreep(bool endWarning, bool busy) {
    if (!endWarning) {
        return CreepOutcome::kCleared;
    }
    if (busy) {
        return CreepOutcome::kWait;
    }
    return CreepOutcome::kStartMove;
}

}  // namespace EipLimit
}  // namespace Gantry

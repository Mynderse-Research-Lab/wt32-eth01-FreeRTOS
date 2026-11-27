/**
 * @file GantryDriveRef.h
 * @brief Live drive position-reference (homed / in-reference) for the
 *        workspace-calibrated latch.
 */

#ifndef GANTRY_DRIVE_REF_H
#define GANTRY_DRIVE_REF_H

#include <cstdint>

namespace Gantry {

/// Kinetix `homed_status` / HCS01 `in_reference` as seen on Class 1 actuals.
/// kUnknown: no live T→O — do not treat as a power-loss / unhome.
enum class DrivePositionRef : uint8_t {
    kUnknown = 0,
    kHomed,
    kLost,
};

}  // namespace Gantry

#endif  // GANTRY_DRIVE_REF_H

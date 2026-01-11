# Gantry Library

Multi-axis gantry control system for ESP32 with servo driver support.

## Features

- **X-axis Motion Control**: Real hardware servo driver (SDF08NK8X) with encoder feedback
- **Y/Theta Axes**: Realistic simulation with trapezoidal velocity profiles
- **Workspace Coordinate System**: User-friendly mm-based API with origin offset
- **Homing & Calibration**: Automatic homing and axis length calibration
- **Backlash Compensation**: Optional on-demand mechanical play compensation
- **Safety Features**: Maximum speed/acceleration limits, soft limits, timeout protection
- **FreeRTOS Compatible**: Non-blocking operations, proper task delays
- **Error Handling**: Comprehensive error codes and status reporting

## Quick Start

```cpp
#include "Gantry.h"

// Configure X-axis servo driver
BergerdaServo::DriverConfig xConfig;
xConfig.encoder_ppr = 2500;
// ... configure pins ...

// Create and initialize gantry
Gantry gantry(xConfig, GRIPPER_PIN);
gantry.setLimitPins(MIN_LIMIT_PIN, MAX_LIMIT_PIN);
gantry.begin();
gantry.enable();

// Home the system
if (gantry.home() == GantryError::OK) {
    // Move to position (workspace coordinates, mm)
    gantry.moveToMm(100, 50, 0);  // X=100mm, Y=50mm, Theta=0°
    
    // Wait for completion
    gantry.waitForMotionComplete();
}
```

## Documentation

See the following files for detailed information:
- **FINAL_STATUS.md**: Complete status and feature overview
- **Gantry.h**: Comprehensive API documentation (Doxygen)
- **BUGS_AND_GAPS_REVIEW.md**: Latest bug review
- **MINOR_FIXES_APPLIED.md**: Recent improvements

## Thread Safety

⚠️ **NOT thread-safe by default**. All methods must be called from:
- Single FreeRTOS task (recommended), OR
- Multiple tasks with mutex protection

See `Gantry.h` for detailed thread safety documentation and examples.

## Coordinate Systems

- **Workspace Coordinates**: User-facing API (mm from workspace origin)
  - Used by: `moveToMm()`, `getCurrentX()`, `getTargetX()`, `moveRelative()`
- **Home Coordinates**: Internal storage (pulses from home position)
  - Used by: `moveTo()` (pulse-based API)

Automatic transformation between coordinate systems.

## Status

✅ **Production Ready** - All critical bugs fixed, comprehensive features implemented.

See FINAL_STATUS.md for complete status and development history.

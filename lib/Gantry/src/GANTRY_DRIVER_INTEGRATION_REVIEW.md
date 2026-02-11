# Gantry Library - Driver Integration Review

**Date:** Feb 10th 2026  
**Reviewer:** AI Assistant  
**Purpose:** Integrate SDF08NK8X driver library improvements into Gantry library

---

## Executive Summary

The Gantry library currently uses the SDF08NK8X driver library but does not leverage several new features that were added in the driver branch. This review identifies **8 major integration opportunities** to improve reliability, safety, and functionality.

---

## Current State Analysis

### ✅ What's Working
- Basic X-axis servo driver integration (`axisX_` member)
- Limit pin configuration (`setLimitPins()`)
- Enable/disable functionality
- Motion status checking (`isBusy()`)

### ❌ Missing Features (Stubs/TODOs)
1. **Homing** (`home()`) - Currently a stub (line 193-204)
2. **Calibration** (`calibrate()`) - Currently a stub (line 206-218)
3. **Limit switch debouncing** - Not implemented
4. **Alarm monitoring** - Not implemented
5. **Travel distance measurement** - Not integrated
6. **Limit switch safety** - Not enforced
7. **GPIO interrupt support** - Not configured

---

## Available Driver Library Features

### 1. **Homing System** ⭐ HIGH PRIORITY
```cpp
// Driver API:
bool startHoming(uint32_t speed = 10000);
bool isHoming() const;
```

**Current Gantry Implementation:**
```cpp
void Gantry::home() {
    // TODO: Implement homing sequence
    // This is a stub implementation
}
```

**Recommendation:** Replace stub with driver API call:
```cpp
void Gantry::home() {
    if (!initialized_ || !enabled_) {
        return;
    }
    if (xMinPin_ < 0) {
        return; // No limit pin configured
    }
    
    // Use driver's built-in homing
    axisX_.startHoming(axisX_.getConfig().homing_speed_pps);
    
    // Wait for completion (non-blocking check in update())
    // Or make this async and check isHoming() in update()
}
```

---

### 2. **Limit Switch Debouncing** ⭐ HIGH PRIORITY
```cpp
// Driver API:
void updateLimitDebounce(bool force = false);
bool getLimitMinDebounced() const;
bool getLimitMaxDebounced() const;
```

**Current Gantry Implementation:**
- No debouncing - direct `digitalRead()` calls (if any)
- Limit switches configured but not debounced

**Recommendation:** 
- Call `updateLimitDebounce()` in `update()` method
- Use debounced values for all limit checks
- Configure limit pins in driver config during `begin()`

```cpp
void Gantry::begin() {
    // ... existing code ...
    
    // Configure limit switches in driver config
    BergerdaServo::DriverConfig& xConfig = const_cast<BergerdaServo::DriverConfig&>(axisX_.getConfig());
    xConfig.limit_min_pin = xMinPin_;
    xConfig.limit_max_pin = xMaxPin_;
    
    // Re-initialize driver with limit pins
    if (!axisX_.initialize()) {
        return false;
    }
}

void Gantry::update() {
    if (!initialized_) {
        return;
    }
    
    // Update limit switch debouncing
    axisX_.updateLimitDebounce();
    
    // ... rest of update logic ...
}
```

---

### 3. **Calibration (Travel Distance Measurement)** ⭐ HIGH PRIORITY
```cpp
// Driver API:
bool measureTravelDistance(uint32_t speed = 10000,
                          uint32_t accel = 5000,
                          uint32_t decel = 5000,
                          uint32_t timeout_ms = 60000);
bool hasTravelDistance() const;
uint32_t getTravelDistance() const;
```

**Current Gantry Implementation:**
```cpp
int Gantry::calibrate() {
    // TODO: Implement calibration sequence
    // This is a stub implementation
    return 0;
}
```

**Recommendation:** Use driver's travel measurement:
```cpp
int Gantry::calibrate() {
    if (!initialized_ || !enabled_) {
        return 0;
    }
    if (xMinPin_ < 0 || xMaxPin_ < 0) {
        return 0; // Both limit pins required
    }
    
    // Use driver's built-in travel measurement
    uint32_t speed = axisX_.getConfig().homing_speed_pps;
    if (axisX_.measureTravelDistance(speed, speed, speed, 90000)) {
        // Wait for completion (check in update())
        // Once complete, get travel distance
        if (axisX_.hasTravelDistance()) {
            uint32_t travel_pulses = axisX_.getTravelDistance();
            axisLength_ = pulsesToMm(travel_pulses);
            return axisLength_;
        }
    }
    
    return 0; // Failed
}
```

---

### 4. **Limit Switch Safety** ⭐ HIGH PRIORITY
**Current State:** No safety checks before movement

**Driver Feature:** Driver automatically blocks movement into active limits

**Recommendation:** 
- Ensure limit pins are configured in driver config
- Driver will automatically prevent unsafe moves
- Add explicit checks in `moveTo()` for better error reporting:

```cpp
GantryError Gantry::moveTo(const JointConfig& joint, ...) {
    // ... existing validation ...
    
    // Check limit switches before movement
    axisX_.updateLimitDebounce(true);
    if (joint.x > pulsesToMm(axisX_.getPosition()) && axisX_.getLimitMaxDebounced()) {
        return GantryError::LIMIT_SWITCH_FAILED;
    }
    if (joint.x < pulsesToMm(axisX_.getPosition()) && axisX_.getLimitMinDebounced()) {
        return GantryError::LIMIT_SWITCH_FAILED;
    }
    
    // ... rest of moveTo logic ...
}
```

---

### 5. **Alarm Monitoring** ⭐ MEDIUM PRIORITY
```cpp
// Driver API:
bool isAlarmActive() const;
```

**Current State:** No alarm monitoring

**Recommendation:** Add alarm checks:
```cpp
bool Gantry::isAlarmActive() const {
    if (!initialized_) {
        return false;
    }
    return axisX_.isAlarmActive();
}

// Add to GantryStatus:
struct GantryStatus {
    // ... existing fields ...
    bool alarmActive;  // Add this
};

// Check in update():
void Gantry::update() {
    // ... existing code ...
    
    // Check for alarm condition
    if (axisX_.isAlarmActive()) {
        // Stop motion, disable, or handle alarm
        if (isBusy()) {
            axisX_.stopMotion(0);
        }
        enabled_ = false;
    }
}
```

---

### 6. **GPIO Interrupt Support** ⭐ MEDIUM PRIORITY
```cpp
// Driver API:
void notifyLimitIrq();
```

**Current State:** No interrupt handling

**Recommendation:** Configure GPIO interrupts for immediate limit detection:
```cpp
void Gantry::begin() {
    // ... existing code ...
    
    // Configure limit switch interrupts (if pins are set)
    if (xMinPin_ >= 0) {
        attachInterrupt(digitalPinToInterrupt(xMinPin_), []() {
            // ISR - notify driver
            // Note: Need access to axisX_ instance - may need static/global or different approach
        }, CHANGE);
    }
    if (xMaxPin_ >= 0) {
        attachInterrupt(digitalPinToInterrupt(xMaxPin_), []() {
            // ISR - notify driver
        }, CHANGE);
    }
}
```

**Note:** This requires careful design due to ISR context limitations. Consider a static callback or instance pointer.

---

### 7. **Position Reporting** ⭐ LOW PRIORITY
**Driver Feature:** Automatic position logging when motion stops

**Current State:** No position reporting

**Recommendation:** Add callback support:
```cpp
// In Gantry constructor or begin():
axisX_.setPositionReachedCallback([](uint32_t position) {
    // Log position or update status
    // Note: ISR context - keep it short
});
```

---

### 8. **Configurable Homing Speed** ⭐ LOW PRIORITY
**Driver Feature:** `homing_speed_pps` in config

**Current State:** Not used

**Recommendation:** Expose homing speed configuration:
```cpp
void Gantry::setHomingSpeed(uint32_t speed_pps) {
    if (initialized_) {
        BergerdaServo::DriverConfig& config = const_cast<BergerdaServo::DriverConfig&>(axisX_.getConfig());
        config.homing_speed_pps = speed_pps;
    }
}
```

---

## Implementation Priority

### Phase 1: Critical Safety & Functionality (Immediate)
1. ✅ **Limit switch debouncing** - Prevents false triggers
2. ✅ **Homing implementation** - Replace stub with driver API
3. ✅ **Calibration implementation** - Replace stub with travel measurement
4. ✅ **Limit switch safety** - Prevent movement into limits

### Phase 2: Enhanced Reliability (Short-term)
5. ✅ **Alarm monitoring** - Detect driver faults
6. ✅ **GPIO interrupt support** - Immediate limit detection

### Phase 3: Quality of Life (Long-term)
7. ✅ **Position reporting** - Better diagnostics
8. ✅ **Configurable homing speed** - User control

---

## Code Changes Required

### File: `Gantry.h`
**Additions:**
- `bool isAlarmActive() const;` - Public method
- `void setHomingSpeed(uint32_t speed_pps);` - Configuration
- `GantryStatus::alarmActive` - Status field

### File: `Gantry.cpp`
**Modifications:**
1. **`begin()`** - Configure limit pins in driver config
2. **`home()`** - Implement using `axisX_.startHoming()`
3. **`calibrate()`** - Implement using `measureTravelDistance()`
4. **`update()`** - Add `updateLimitDebounce()` call
5. **`moveTo()`** - Add limit safety checks
6. **`isBusy()`** - Check `isHoming()` status
7. **New:** `isAlarmActive()` - Wrapper for driver method

---

## Testing Checklist

- [ ] Homing sequence completes successfully
- [ ] Calibration measures travel distance correctly
- [ ] Limit switches debounce properly (no false triggers)
- [ ] Movement blocked when limit switch active
- [ ] Alarm condition detected and handled
- [ ] GPIO interrupts trigger correctly
- [ ] Position reporting works on motion completion
- [ ] Homing speed configuration persists

---

## Migration Notes

### Breaking Changes
- **None** - All changes are additive or replace stubs

### Backward Compatibility
- ✅ Existing API remains unchanged
- ✅ Stub methods replaced with real implementations
- ✅ New optional features don't affect existing code

### Configuration Updates
- Limit pins should be set via `setLimitPins()` **before** `begin()`
- Driver config's `limit_min_pin` and `limit_max_pin` will be set automatically

---

## Example Integration Code

See `GANTRY_INTEGRATION_EXAMPLE.cpp` for complete implementation examples.

---

## Conclusion

The Gantry library can significantly benefit from the driver library improvements. **Priority 1-4 items should be implemented immediately** to replace stubs and add critical safety features. Priority 5-8 items enhance reliability and user experience but can be implemented incrementally.

**Estimated Implementation Time:**
- Phase 1: 2-4 hours
- Phase 2: 1-2 hours  
- Phase 3: 1 hour

**Total: 4-7 hours** for complete integration.


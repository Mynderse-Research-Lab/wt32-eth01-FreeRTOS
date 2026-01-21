# Conveyor Pick-and-Place Application Review

## System Specifications
- **X-axis**: Ball screw with 1 rotation = 40mm pitch
- **Application**: Pick items from a moving conveyor, place in bin
- **Input**: Conveyor speed and item X location via MQTT

---

## CRITICAL FLAWS

### 1. ❌ **Wrong Unit Conversion (CRITICAL)**

**Location**: `Gantry.cpp` lines 477-493

```cpp
const float defaultEncoderPPR = 2500.0f; // WRONG - hardcoded, doesn't match servo
const float pitch = kinematicParams_.x_axis_ball_screw_pitch_mm; // 40mm
```

**Problem**: 
- Encoder PPR is hardcoded as 2500, but servo driver uses 6000 steps/rev
- This causes ALL position calculations to be off by 2.4x!

**Correct Calculation**:
- Ball screw pitch: 40mm/revolution
- Servo steps: 6000 pulses/revolution
- **1mm = 6000 / 40 = 150 pulses**
- Current code calculates: 1mm = 2500 / 40 = 62.5 pulses (WRONG!)

**Fix**: Use actual encoder PPR from driver config, not hardcoded value.

---

### 2. ❌ **No Conveyor Speed Compensation (CRITICAL)**

**Location**: `main.cpp` - `GantryCommand` struct

```cpp
struct GantryCommand {
  int32_t x;      // Static position only!
  int32_t y;
  int32_t theta;
  uint32_t speed;
  bool home;
  bool calibrate;
};
```

**Problem**: No field for conveyor speed! The gantry targets a static position while the item moves away.

**Missing**:
- `conveyor_speed_mm_per_s` - Item velocity
- `detection_timestamp_ms` - When item was detected (for latency compensation)
- `item_width_mm` - For grip timing

---

### 3. ❌ **No Interception Algorithm**

**Problem**: Gantry moves to where the item **WAS**, not where it **WILL BE**.

**Required Algorithm**:
```
item_current_x = received_x + conveyor_speed * network_latency
time_to_reach = distance / gantry_speed
intercept_x = item_current_x + conveyor_speed * time_to_reach
```

Iterative refinement may be needed since gantry travel time depends on distance.

---

### 4. ❌ **Speed Conversion is Wrong**

**Location**: `Gantry.cpp` line 187

```cpp
uint32_t speed_pps = (uint32_t)(speed_mm_per_s * 100.0f); // MAGIC NUMBER!
```

**Problem**: The `* 100.0f` factor is arbitrary and incorrect.

**Correct**:
```cpp
// speed_pps = speed_mm_per_s * (pulses_per_rev / pitch_mm)
uint32_t speed_pps = (uint32_t)(speed_mm_per_s * (6000.0f / 40.0f));
// = speed_mm_per_s * 150.0f
```

---

### 5. ❌ **Blocking Wait Prevents Real-Time Tracking**

**Location**: `main.cpp` lines 285-288

```cpp
while (gantry.isBusy()) {
    vTaskDelay(busyCheckDelay);  // Can't update target during motion!
}
```

**Problem**: Once motion starts, it cannot be updated or aborted. For a moving conveyor:
- If the conveyor speeds up/slows down, we can't adjust
- If item falls off, we can't abort
- No real-time tracking capability

---

### 6. ❌ **500ms Grip Delay is Way Too Slow**

**Location**: `main.cpp` line 248

```cpp
const TickType_t gripDelay = pdMS_TO_TICKS(500);  // 500ms!
```

**Problem**: At 100mm/s conveyor speed, item moves 50mm during grip delay!

**Typical Requirements**:
- Pneumatic gripper: 50-100ms
- Vacuum gripper: 30-50ms
- Should be configurable via MQTT

---

### 7. ❌ **Y-Axis is Simulated (Not Implemented)**

**Location**: `Gantry.cpp` lines 113-114

```cpp
currentY_ = y;  // Simulated - instant update
currentTheta_ = theta;  // Simulated - instant update
```

**Problem**: The Y-axis (vertical descent for picking) doesn't actually move! Without Y-axis:
- Cannot descend to pick
- Cannot ascend after picking
- The entire pick sequence is broken

---

### 8. ❌ **No Pick Zone / Window Validation**

**Problem**: No check if item is within reachable pick zone.

**Required**:
```cpp
const float PICK_ZONE_MIN_X = 50.0f;   // Start of pick window
const float PICK_ZONE_MAX_X = 400.0f;  // End of pick window

if (intercept_x < PICK_ZONE_MIN_X || intercept_x > PICK_ZONE_MAX_X) {
    // Item will leave pick zone before we can reach it - SKIP
    return;
}
```

---

### 9. ❌ **No S-Curve Motion Profile**

**Location**: `SDF08NK8X.cpp` motion profile

Current profile: **Trapezoidal** (constant acceleration)

**Problem**: Trapezoidal profiles cause:
- Mechanical jerk at acceleration/deceleration transitions
- Vibration in the gantry structure
- Not "graceful" as required

**Required**: S-curve (jerk-limited) profile for smooth motion.

---

### 10. ❌ **Command Queue Drops New Commands**

**Location**: `main.cpp` lines 152-158

```cpp
if (xQueueSend(commandQueue, &cmd, 0) != pdTRUE) {
    LOG_ERROR(TAG_MQTT, "Queue Full! Dropped command.");
}
```

**Problem**: For real-time tracking, NEW commands should have priority over OLD queued commands. Current design drops the newest (most relevant) data.

**Fix**: Use `xQueueOverwrite()` or clear queue before adding new item position.

---

## MODERATE FLAWS

### 11. ⚠️ **No Network Latency Compensation**

MQTT messages have latency (typically 10-50ms). At 100mm/s conveyor speed, 50ms = 5mm position error.

**Fix**: Include timestamp in MQTT message, compensate on receive.

---

### 12. ⚠️ **No Maximum Speed Limit Based on Mechanics**

The gantry may have mechanical speed limits not enforced in software.

**Required**:
```cpp
const uint32_t MAX_X_SPEED_MM_PER_S = 500;  // Based on ball screw limits
const uint32_t MAX_ACCEL_MM_PER_S2 = 2000;  // Based on motor torque
```

---

### 13. ⚠️ **Acceleration Set to speed/2 (Arbitrary)**

**Location**: `Gantry.cpp` lines 132-133

```cpp
uint32_t accel = speed / 2;
uint32_t decel = speed / 2;
```

**Problem**: Acceleration should be based on motor capability and load, not speed.

---

### 14. ⚠️ **No Bin Position Handling**

After picking, where does the gantry place the item? No "bin position" is defined or handled.

---

### 15. ⚠️ **MQTT Status Only Every 5 Seconds**

**Location**: `main.cpp` line 348

```cpp
const TickType_t publishInterval = pdMS_TO_TICKS(5000);
```

**Problem**: Too slow for monitoring real-time pick-and-place operations.

---

## RECOMMENDED ARCHITECTURE

### Enhanced Command Structure

```cpp
struct PickCommand {
    // Item detection
    float item_x_mm;                  // X position when detected
    float conveyor_speed_mm_per_s;    // Conveyor velocity
    uint32_t detection_timestamp_ms;  // When item was detected
    
    // Item properties
    float item_width_mm;              // For grip timing
    float item_height_mm;             // For Y descent
    
    // Motion parameters
    uint32_t max_speed_mm_per_s;      // Speed limit
    uint32_t acceleration_mm_per_s2;  // Acceleration
    uint32_t grip_delay_ms;           // Gripper actuation time
    
    // Bin position
    float bin_x_mm;                   // Where to place
    float bin_y_mm;
};
```

### Interception Algorithm

```cpp
float calculateInterceptPosition(const PickCommand& cmd) {
    // Compensate for network latency
    uint32_t latency_ms = millis() - cmd.detection_timestamp_ms;
    float item_current_x = cmd.item_x_mm + 
        cmd.conveyor_speed_mm_per_s * (latency_ms / 1000.0f);
    
    // Iterative interception calculation
    float gantry_x = getCurrentX_mm();
    float intercept_x = item_current_x;
    
    for (int i = 0; i < 3; i++) {  // 3 iterations usually converge
        float distance = fabs(intercept_x - gantry_x);
        float travel_time = estimateTravelTime(distance, cmd.max_speed_mm_per_s);
        intercept_x = item_current_x + 
            cmd.conveyor_speed_mm_per_s * travel_time;
    }
    
    return intercept_x;
}
```

### Pick-and-Place State Machine

```cpp
enum class PickState {
    IDLE,               // Waiting for command
    MOVING_TO_PICK,     // Moving X to intercept
    DESCENDING,         // Moving Y down
    GRIPPING,           // Actuating gripper
    ASCENDING,          // Moving Y up with item
    MOVING_TO_BIN,      // Moving X to bin
    RELEASING,          // Opening gripper
    RETURNING,          // Returning to idle position
};
```

---

## PRIORITY FIX LIST

| Priority | Issue | Impact | Effort |
|----------|-------|--------|--------|
| P0 | Fix unit conversion (PPR) | All positions wrong | Low |
| P0 | Add conveyor speed to command | Can't track items | Medium |
| P0 | Implement interception algorithm | Can't catch items | Medium |
| P1 | Implement Y-axis control | Can't pick items | High |
| P1 | Reduce grip delay | Items move during grip | Low |
| P1 | Add pick zone validation | Wasted motion | Low |
| P2 | S-curve motion profile | Smoother motion | High |
| P2 | Real-time target update | Better tracking | Medium |
| P2 | Latency compensation | Position accuracy | Low |

---

## QUICK WINS

### 1. Fix Encoder PPR (5 minutes)
```cpp
// In Gantry.cpp, replace hardcoded 2500.0f:
float getEncoderPPR() const {
    return 6000.0f;  // Or read from xConfig.encoder_ppr
}
```

### 2. Add Conveyor Speed to MQTT (10 minutes)
```cpp
// In GantryCommand:
float conveyor_speed_mm_per_s;

// In onMqttMessage:
char *vPtr = strstr(buf, "v:");  // v for velocity
if (vPtr) cmd.conveyor_speed_mm_per_s = atof(vPtr + 2);
```

### 3. Reduce Grip Delay (1 minute)
```cpp
const TickType_t gripDelay = pdMS_TO_TICKS(100);  // 100ms instead of 500ms
```

### 4. Use Queue Overwrite (5 minutes)
```cpp
// Replace xQueueSend with xQueueOverwrite for single-item queue
// Or use xQueueReset() before xQueueSend()
xQueueReset(commandQueue);
xQueueSend(commandQueue, &cmd, 0);
```

---

## CONCLUSION

The current implementation is suitable for **static position targeting** but has fundamental issues for **moving conveyor** pick-and-place:

1. **Unit conversions are wrong** - positions are off by 2.4x
2. **No conveyor motion tracking** - targets static positions
3. **Y-axis not implemented** - can't actually pick items
4. **Motion too slow/jerky** - not "graceful"

A significant refactor is needed to support real-time conveyor tracking with interception algorithms.

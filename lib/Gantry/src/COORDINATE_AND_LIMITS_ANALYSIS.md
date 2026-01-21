# Coordinate Systems, Limits, and Backlash Analysis

## 1. End-Effector vs X-Axis Coordinate Transformation

### Current Implementation

**X-Axis Position Tracking:**
- `getCurrentX()` and `getTargetX()` return the **X-axis position** (base horizontal axis)
- Internal storage (`currentX_`, `targetX_`) tracks X-axis position in pulses from home
- No transformation is applied to account for end-effector offset

**Gripper/End-Effector Offsets (from header):**
- `GRIPPER_Y_OFFSET_MM = 385mm` - Gripper Y offset from X origin
- `GRIPPER_Z_OFFSET_MM = 80mm` - Gripper Z offset from X origin  
- `THETA_X_OFFSET_MM = -55mm` - Theta X offset from Y vertical

**User Requirement:**
> "moveToMm(0) should move the end-effector center to home, not the horizontal axis"

### Gap Identified

**Problem**: The current implementation does NOT account for the end-effector offset from the X-axis when reporting positions.

**Expected Behavior:**
- `moveToMm(0)` should position the **end-effector center** at the workspace origin (home position)
- `getCurrentX()` and `getTargetX()` should return the **end-effector center position**, not the X-axis position
- The X-axis must be moved to account for the offset so that the end-effector center is at the target

**Missing Transformation:**
- When user commands `moveToMm(x_workspace)`, the system should:
  1. Calculate desired end-effector center position: `x_workspace`
  2. Account for end-effector offset (need to determine: is there an X offset? THETA_X_OFFSET_MM = -55mm suggests there might be)
  3. Calculate required X-axis position: `x_axis = x_end_effector - end_effector_x_offset`
  4. Move X-axis to that position

**Questions to Resolve:**
1. What is the end-effector X offset from the X-axis origin?
   - `THETA_X_OFFSET_MM = -55mm` is the Theta offset from Y vertical
   - Is there a direct X offset between X-axis origin and end-effector center?
   - If Theta = 0°, what is the end-effector X position relative to X-axis?

2. Should `workspace_origin_offset_mm_` account for this, or should it be a separate transformation?

### Recommendation

Add end-effector offset compensation to the coordinate transformation:
- Add a method to get/set end-effector X offset (or determine it from mechanical layout)
- Transform user coordinates (end-effector position) to X-axis coordinates before commanding motion
- Transform X-axis position to end-effector position when reporting

---

## 2. Hard Limits vs Soft Limits

### Hard Limits (Physical Constraints)

**Definition**: Physical mechanical limits of the axis - cannot be exceeded without physical damage.

**Implementation:**
- Stored in: `axisLength_` (in pulses)
- Set by: `calibrate()` - measures physical axis length from home to max limit switch
- Range: `0` (home) to `axisLength_` (max limit switch position)
- Units: Pulses (home coordinates)

**Checking:**
- Code location: `moveTo()` lines 286-290
  ```cpp
  if (axisLength_ > 0) {
    if (x_pulses < 0 || (uint32_t)x_pulses > (uint32_t)axisLength_) {
      return GantryError::INVALID_POSITION;
    }
  }
  ```
- Applied to: X-axis position in home coordinates (pulses)

**Backlash Compensation:**
- Code location: `moveTo()` lines 340-345
  ```cpp
  if (axisLength_ > 0 && compensated_target > (int32_t)axisLength_) {
    compensated_target = axisLength_;  // Clamp to max
  }
  if (compensated_target < 0) {
    compensated_target = 0;  // Clamp to min
  }
  ```
- **Backlash compensation RESPECTS hard limits** - clamped to 0 and axisLength_

### Soft Limits (User-Defined Workspace Boundaries)

**Definition**: User-configurable safety/work area limits in workspace coordinates - can be set tighter than hard limits for safety.

**Implementation:**
- Stored in: `soft_limit_min_mm_` and `soft_limit_max_mm_` (in mm)
- Enabled by: `setSoftLimits(min_mm, max_mm)` - also enables soft limits
- Range: User-defined in workspace coordinates (mm)
- Units: Millimeters (workspace coordinates)

**Checking:**
- Code location 1: `moveToMm()` lines 418-422 (workspace coordinates - direct check)
  ```cpp
  if (soft_limits_enabled_) {
    if (x_mm < soft_limit_min_mm_ || x_mm > soft_limit_max_mm_) {
      return GantryError::INVALID_POSITION;
    }
  }
  ```
- Code location 2: `moveTo()` lines 274-280 (home coordinates - converts to workspace for check)
  ```cpp
  if (soft_limits_enabled_) {
    int32_t x_mm_home = pulsesToMm(x_pulses);
    int32_t x_mm_workspace = x_mm_home - workspace_origin_offset_mm_;  // BUG: should be +?
    if (x_mm_workspace < soft_limit_min_mm_ || x_mm_workspace > soft_limit_max_mm_) {
      return GantryError::INVALID_POSITION;
    }
  }
  ```
- Applied to: User-commanded position in workspace coordinates (mm)

**Backlash Compensation:**
- Code location: `moveTo()` lines 323-352
- **Backlash compensation DOES NOT check soft limits** - only clamps to hard limits
- Gap: After adding compensation pulses, the code should validate against soft limits but currently doesn't

### Comparison

| Aspect | Hard Limits | Soft Limits |
|--------|-------------|-------------|
| **Type** | Physical mechanical constraint | User-configurable safety boundary |
| **Set By** | Automatic (calibration) | User (setSoftLimits) |
| **Storage** | `axisLength_` (pulses) | `soft_limit_min_mm_`, `soft_limit_max_mm_` (mm) |
| **Coordinates** | Home coordinates (pulses from home) | Workspace coordinates (mm from workspace origin) |
| **Range** | 0 to axisLength_ | User-defined |
| **Backlash Checked** | ✅ Yes (clamped to 0 and axisLength_) | ❌ No (GAP) |
| **Purpose** | Prevent physical damage | Define safe work area |

---

## 3. Backlash Compensation and Limits

### How Backlash Compensation Works

**Purpose**: Compensate for mechanical play/backlash in the drive system by adding extra movement when direction changes.

**Current Implementation:**
1. Detects direction change (lines 331)
2. Adds compensation pulses in new direction (lines 334, 336)
   - Positive direction: `compensated_target += backlash_compensation_pulses_`
   - Negative direction: `compensated_target -= backlash_compensation_pulses_`
3. Clamps to hard limits only (lines 340-345)
4. Does NOT check soft limits

### Relationship with Limits

**Hard Limits:**
- ✅ **Respected**: Backlash compensation clamps the compensated target to hard limits
  - If compensated target > axisLength_: clamped to axisLength_
  - If compensated target < 0: clamped to 0
- **Reasoning**: Physical limits must never be exceeded

**Soft Limits:**
- ❌ **NOT Respected**: Backlash compensation does NOT check soft limits
- **Problem**: Compensated target can exceed soft limits even if original target was within soft limits
- **Example Scenario**:
  - Soft limits: 0 to 500mm
  - Target: 480mm (within soft limits)
  - Backlash compensation: +60 pulses (e.g., ~24mm)
  - Compensated target: 504mm (exceeds soft limit)
  - **Result**: Move proceeds even though it violates soft limits

### Recommendation

**Add soft limits check after backlash compensation:**

1. Calculate compensated target (current code)
2. Clamp to hard limits (current code)
3. **NEW**: Convert to workspace coordinates and check soft limits
4. **NEW**: If soft limits violated, either:
   - Reject the move (return error), OR
   - Reduce compensation to fit within soft limits, OR
   - Clamp compensated target to soft limits (with user option)

**Code Location**: After line 345 in `moveTo()`, before line 354

---

## 4. Abort Flag Clearing

### Current Behavior

**Abort Flag**: `abort_requested_` (boolean flag)

**Set In:**
- `abortMotion()` line 194: Sets `abort_requested_ = true`

**Reset In:**
- `home()` line 671: Resets at start of homing
- `calibrate()` line 820: Resets at start of calibration
- **NOT reset after regular moves**

**Checked In:**
- `waitForAxisMotionComplete()` line 93
- `waitForMotionComplete()` line 528
- `home()` loops lines 688, 714, 780
- `calibrate()` loops lines 824, 848

### Problem

If `abortMotion()` is called during a regular move (not homing/calibration):
- Flag is set to `true`
- Move is stopped
- Flag remains `true`
- Subsequent calls to `waitForMotionComplete()` return immediately (line 528-529)
- Flag only clears if `home()` or `calibrate()` is called

### Recommendation

**Clear abort flag after aborting regular moves:**

Option 1: Clear in `abortMotion()` after stopping motion
- Pros: Simple, flag cleared immediately
- Cons: If abort is detected in wait loops, flag might already be cleared

Option 2: Clear in `waitForMotionComplete()` when flag is detected
- Pros: Clears when actually used
- Cons: Flag persists if waitForMotionComplete() is not called

Option 3: Clear in `update()` when motion completes after abort
- Pros: Clears when motion actually completes
- Cons: More complex, requires tracking abort state

**Recommended**: Option 2 - Clear in `waitForMotionComplete()` when flag is detected (line 528-529)

---

## Summary of Gaps

1. **End-Effector Offset**: Missing transformation between X-axis position and end-effector center position
2. **Soft Limits in Backlash**: Backlash compensation doesn't check soft limits
3. **Abort Flag**: Not cleared after aborting regular moves
4. **Coordinate Transformation Bug**: `moveTo()` uses subtraction while `getCurrentX()` uses addition (from previous analysis)

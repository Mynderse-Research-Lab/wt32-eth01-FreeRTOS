# Gantry Library - Bugs and Functional Gaps Analysis

## Critical Bugs

### 1. **Coordinate Transformation Inconsistency** ⚠️ CRITICAL

**Location**: `moveTo()` line 276 vs `getCurrentX()`/`getTargetX()` line 938/951

**Issue**: Inconsistent coordinate transformation between workspace and home coordinates.

- In `moveTo()` (soft limits check): Uses **SUBTRACTION** (line 276)
  ```cpp
  int32_t x_mm_workspace = x_mm_home - workspace_origin_offset_mm_;
  ```
  
- In `getCurrentX()`/`getTargetX()`: Uses **ADDITION** (lines 938, 951)
  ```cpp
  return mm_from_home + workspace_origin_offset_mm_;
  ```

- In `moveToMm()`: Uses **ADDITION** (line 429)
  ```cpp
  int32_t x_mm_home = x_mm + workspace_origin_offset_mm_;
  ```

**Impact**: The soft limits check in `moveTo()` uses the wrong transformation. If `moveToMm(0)` and `moveTo(pulses_for_0mm)` are called with the same target, they may be validated differently against soft limits, potentially allowing moves that should be rejected or rejecting moves that should be allowed.

**Expected Behavior**: The transformations should be inverse operations:
- If `moveToMm`: `home_mm = workspace_mm + offset` (line 429)
- Then `getCurrentX`: `workspace_mm = home_mm - offset` (should be subtraction)
- And `moveTo` soft limits: `workspace_mm = home_mm - offset` (line 276 - correct)

**Current Behavior**: 
- `moveToMm()`: Uses **ADDITION** - `home_mm = workspace_mm + offset` (line 429)
- `getCurrentX()`/`getTargetX()`: Uses **ADDITION** - `workspace_mm = home_mm + offset` (lines 938, 951) ❌
- `moveTo()` soft limits: Uses **SUBTRACTION** - `workspace_mm = home_mm - offset` (line 276) ✓

**Analysis**: 
- `moveTo()` uses subtraction (correct inverse of `moveToMm()`)
- `getCurrentX()`/`getTargetX()` use addition (incorrect - not inverse of `moveToMm()`)

**Mathematical Verification**:
- `moveToMm(0)` with offset=50: `home_mm = 0 + 50 = 50mm` (moves to 50mm from home)
- At home (home_mm=0): `getCurrentX()` should return `0 - 50 = -50mm` but returns `0 + 50 = 50mm`

However, the documentation states: "At home (0mm from home): getCurrentX() returns 50mm (workspace origin)" which suggests addition is intended.

**Note**: There's also a documentation contradiction:
- Line 241: "x_mm=0 means 50mm from home" (consistent with implementation)
- Line 387: "moveToMm(0) moves to home" (contradicts implementation and line 241)

**Recommendation**: 
1. **Verify the intended behavior** - Does "workspace origin" mean workspace position 0, or the point where workspace coordinates are 0?
2. **Fix the inconsistency** - Either:
   - Change `getCurrentX()`/`getTargetX()` to use subtraction (if `moveTo()` is correct), OR
   - Change `moveTo()` line 276 to use addition (if `getCurrentX()` is correct)
3. **Fix documentation** - Resolve contradiction between lines 241 and 387

---

### 2. **Abort Flag Not Reset After Regular Moves** ⚠️ MEDIUM

**Location**: `abortMotion()` line 194, `waitForMotionComplete()` line 528

**Issue**: The `abort_requested_` flag is set in `abortMotion()` but only reset in `home()` and `calibrate()`. If `abortMotion()` is called during a regular move (not homing/calibration), the flag remains set and will cause `waitForMotionComplete()` to return immediately on subsequent calls.

**Impact**: After aborting a regular move, subsequent calls to `waitForMotionComplete()` will immediately return `OK` instead of actually waiting, which could mask motion completion issues.

**Current Behavior**:
- `abortMotion()` sets `abort_requested_ = true` (line 194)
- `waitForMotionComplete()` checks the flag and returns early (line 528)
- Flag is only reset in `home()` (line 671) and `calibrate()` (line 820)
- No reset after regular move completion

**Recommendation**: Reset `abort_requested_` flag in one of:
1. `abortMotion()` after notifying (if abort is considered "handled")
2. `waitForMotionComplete()` when it detects the flag
3. `update()` when motion completes normally
4. Add a public `clearAbortFlag()` method

---

## Functional Gaps and Issues

### 3. **Backlash Compensation Doesn't Respect Soft Limits** ⚠️ MEDIUM

**Location**: `moveTo()` lines 323-352

**Issue**: Backlash compensation can push the target beyond soft limits. The compensation clamps to `axisLength_` and 0 (hard limits) but doesn't check soft limits.

**Impact**: With backlash compensation enabled and soft limits set, the compensated target might exceed soft limits, potentially causing unexpected behavior.

**Current Behavior**: 
- Backlash compensation adds/subtracts pulses (lines 334, 336)
- Clamps to `axisLength_` and 0 (lines 340-345)
- No soft limits check on compensated target

**Recommendation**: After calculating compensated target, validate against soft limits (in workspace coordinates) before proceeding.

---

### 4. **Motion Completion Callback Timing Issues** ⚠️ LOW-MEDIUM

**Location**: `update()` lines 596-602, 639-641

**Issue**: Motion completion callback logic has potential issues:

1. **X-axis completion** (lines 596-602): Only notifies if no stub movements are active. If X completes while Y/Theta are still moving, the callback doesn't fire until all axes complete.
2. **Stub completion** (lines 639-641): Only notifies if `!isBusy()`, which checks X motion too. This is correct, but the logic means the callback fires at different times depending on which axis completes last.

**Impact**: The callback timing may not match user expectations. If a user expects a callback when X-axis completes, but Y/Theta are still moving, they won't get it.

**Current Behavior**: Callback fires only when ALL motion completes.

**Recommendation**: Consider whether the API should support:
- Single callback for all motion completion (current behavior)
- Separate callbacks per axis
- Or document the current behavior more clearly

---

### 5. **Missing Validation: encoder_ppr Zero Check in moveToMm()** ⚠️ LOW

**Location**: `moveToMm()` lines 436-447

**Issue**: The speed conversion checks for `encoder_ppr > 0`, but acceleration/deceleration conversion (lines 452-464) doesn't validate `encoder_ppr` before use. If `encoder_ppr == 0`, the division will result in 0, which might not be caught.

**Current Behavior**: 
- Speed conversion validates `encoder_ppr > 0` (line 436)
- Acceleration/deceleration conversions check `config.encoder_ppr > 0` (lines 455, 461)
- Actually, it does check! This is not a bug.

**Status**: **FALSE ALARM** - The code does check `encoder_ppr > 0` for acceleration/deceleration.

---

### 6. **No Validation of Workspace Offset Range** ⚠️ LOW

**Location**: `setWorkspaceOriginOffset()` - header inline function

**Issue**: `setWorkspaceOriginOffset()` accepts any `int32_t` value without validation. An extremely large offset could cause overflow in coordinate transformations.

**Impact**: Very large offsets could cause integer overflow in `moveToMm()`, `getCurrentX()`, or `getTargetX()`.

**Current Behavior**: No validation or bounds checking.

**Recommendation**: Add validation or document reasonable limits for the offset value.

---

### 7. **Calibration Calls home() Twice When returnToHome=true** ⚠️ LOW

**Location**: `calibrate()` lines 823, 912

**Issue**: When `calibrate(true)` is called, it calls `home()` twice - once at the start (line 823) and once at the end (line 912). This is actually correct behavior (calibration needs to home first, then return home), but the abort flag is reset twice, which is redundant but not harmful.

**Status**: **NOT A BUG** - This is intentional and correct behavior.

---

### 8. **Missing Error Handling: moveTo() Failure After Setting Target** ⚠️ LOW

**Location**: `moveTo()` lines 360-367

**Issue**: If `axisX_.moveToPosition()` fails after setting stub targets and calculating backlash compensation, the internal state (`targetX_`, `targetY_`, `targetTheta_`, `y_moving_`, `theta_moving_`) is partially updated, which could cause inconsistencies.

**Current Behavior**: 
- Stub targets are set (lines 378-406)
- Backlash compensation calculated (lines 323-352)
- `targetX_` set (line 367)
- If `axisX_.moveToPosition()` fails, stub state is already set

**Impact**: Stub targets are set even if X-axis move fails, potentially causing confusion.

**Recommendation**: Set stub state after X-axis move succeeds, or rollback on failure.

---

### 9. **getAxisLength() Returns 0 Before Calibration** ⚠️ LOW

**Location**: `getAxisLength()` line 968

**Issue**: `getAxisLength()` returns 0 before calibration is performed. This is correct (axisLength_ is 0), but could be confusing to users who expect a meaningful value.

**Status**: **NOT A BUG** - This is expected behavior, but could be better documented.

---

### 10. **Soft Limits Checked in Two Places with Different Logic** ⚠️ LOW

**Location**: `moveTo()` line 274-281, `moveToMm()` line 418-423

**Issue**: Soft limits are validated in both `moveTo()` and `moveToMm()`, but `moveToMm()` validates directly in workspace coordinates (simpler), while `moveTo()` converts from pulses to mm to workspace coordinates (more complex).

**Impact**: The coordinate conversion bug (#1) affects the `moveTo()` check but not `moveToMm()`.

**Recommendation**: Fix bug #1 to ensure consistency.

---

## Code Quality and Documentation Issues

### 11. **Documentation Inconsistency: Coordinate Transformation**

**Location**: Header file line 163

**Issue**: The header documentation states:
- "Writing (moveToMm): workspace_mm → convert to home_pulses → subtract offset → store"

But the implementation adds the offset, not subtracts.

**Status**: Documentation error - should say "add offset" to match implementation.

---

### 12. **Missing Documentation: Abort Flag Behavior**

**Issue**: The `abort_requested_` flag behavior (when it's reset, when it persists) is not documented in the public API documentation.

**Recommendation**: Document the abort behavior in `abortMotion()` method documentation.

---

## Summary

### Critical (Fix Immediately)
1. **Coordinate transformation inconsistency** (#1) - Could cause incorrect soft limit validation

### Medium Priority (Fix Soon)
2. **Abort flag not reset** (#2) - Causes unexpected behavior after abort
3. **Backlash compensation ignores soft limits** (#3) - Could violate safety limits
4. **Motion completion callback timing** (#4) - May not match user expectations

### Low Priority (Consider for Future)
5. **Workspace offset validation** (#6)
6. **Error handling in moveTo()** (#8)
7. **Documentation updates** (#11, #12)

---

## Testing Recommendations

1. **Test coordinate transformation consistency**:
   - Call `moveToMm(0)` and `moveTo(pulses_for_0mm)` with soft limits enabled
   - Verify both accept/reject consistently
   - Test with various offset values

2. **Test abort flag behavior**:
   - Abort a regular move, then call `waitForMotionComplete()`
   - Verify expected behavior

3. **Test backlash compensation with soft limits**:
   - Enable backlash compensation and soft limits
   - Move in alternating directions near soft limit boundaries
   - Verify compensation doesn't violate soft limits

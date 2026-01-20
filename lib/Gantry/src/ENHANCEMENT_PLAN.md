# Gantry Library Enhancement Plan

## Overview

This plan outlines the enhancements to be applied to the Gantry library based on the recommendations in `GANTRY_REPRESENTATION_GUIDE.md`. The enhancements are organized into phases, prioritized by usefulness and implementation complexity.

## Goals

1. **Formal Kinematic Representation**: Add structured types for joint configurations and end-effector poses
2. **Enhanced API**: Provide cleaner, more intuitive interfaces for kinematics operations
3. **Trajectory Planning**: Add waypoint-based trajectory planning capabilities
4. **Backwards Compatibility**: Maintain all existing APIs to ensure no breaking changes
5. **Memory Efficiency**: Keep memory footprint minimal for WT32-ETH01 (4MB RAM, 320KB RAM)

## Phase 1: Core Structures and Enhanced Kinematics API (HIGH PRIORITY)

### 1.1 Add Joint Configuration Structure

**File**: `lib/Gantry/src/Gantry.h`

**Changes**:
- Add `JointConfig` struct before the `Gantry` class definition
- Add `EndEffectorPose` struct before the `Gantry` class definition
- Make these structures public so users can use them

**Implementation**:
```cpp
/**
 * @struct JointConfig
 * @brief Joint space configuration for 3-axis gantry
 * 
 * Represents the gantry configuration in joint space (joint positions).
 */
struct JointConfig {
    float q_x;      // X-axis position (mm) - horizontal (right-to-left)
    float q_y;      // Y-axis position (mm) - vertical (down-to-up)
    float q_theta;  // Theta angle (degrees) - rotation around Y-axis
    
    /**
     * @brief Check if joint configuration is within limits
     * @return true if all joints are within valid range
     */
    bool isValid() const;
    
    /**
     * @brief Get joint limits
     */
    static void getLimits(float& x_min, float& x_max,
                         float& y_min, float& y_max,
                         float& theta_min, float& theta_max);
};

/**
 * @struct EndEffectorPose
 * @brief End-effector pose in workspace/cartesian space
 * 
 * Represents the end-effector position and orientation in workspace coordinates.
 */
struct EndEffectorPose {
    float x;      // X position (mm) - workspace coordinates
    float y;      // Y position (mm) - workspace coordinates
    float z;      // Z position (mm) - typically constant for gantry
    float theta;  // Orientation (degrees)
    
    // Default constructor
    EndEffectorPose() : x(0), y(0), z(0), theta(0) {}
    
    // Constructor with values
    EndEffectorPose(float x_val, float y_val, float z_val, float theta_val)
        : x(x_val), y(y_val), z(z_val), theta(theta_val) {}
};
```

**Estimated Memory**: ~28 bytes per instance (negligible)

**Files to Modify**:
- `lib/Gantry/src/Gantry.h` - Add structs (public, before class)
- `lib/Gantry/src/Gantry.cpp` - Implement `JointConfig::isValid()` and `getLimits()`

**Testing**:
- Create joint config, validate limits
- Create end-effector pose, verify defaults

### 1.2 Add Enhanced Kinematics Methods

**File**: `lib/Gantry/src/Gantry.h` and `Gantry.cpp`

**Changes**:
- Add new public methods that work with `JointConfig` and `EndEffectorPose`
- Keep existing `forwardKinematicsX()` and `inverseKinematicsX()` methods (backwards compatible)

**New Methods**:
```cpp
// In Gantry class (public section):

/**
 * @brief Forward kinematics: Joint space -> Workspace
 * @param joint Joint configuration in joint space
 * @return End-effector pose in workspace coordinates
 */
EndEffectorPose forwardKinematics(const JointConfig& joint) const;

/**
 * @brief Inverse kinematics: Workspace -> Joint space
 * @param pose Desired end-effector pose in workspace coordinates
 * @return Required joint configuration
 */
JointConfig inverseKinematics(const EndEffectorPose& pose) const;

/**
 * @brief Get current joint configuration
 * @return Current joint positions in joint space
 */
JointConfig getCurrentJointConfig() const;

/**
 * @brief Get target joint configuration
 * @return Target joint positions in joint space
 */
JointConfig getTargetJointConfig() const;

/**
 * @brief Get current end-effector pose
 * @return Current end-effector pose in workspace coordinates
 */
EndEffectorPose getCurrentEndEffectorPose() const;

/**
 * @brief Get target end-effector pose
 * @return Target end-effector pose in workspace coordinates
 */
EndEffectorPose getTargetEndEffectorPose() const;
```

**Implementation Notes**:
- `forwardKinematics()` will use existing `forwardKinematicsX()` logic
- `inverseKinematics()` will use existing `inverseKinematicsX()` logic
- `getCurrentJointConfig()` will convert from internal `currentX_`, `currentY_`, `currentTheta_`
- `getCurrentEndEffectorPose()` will call `forwardKinematics(getCurrentJointConfig())`

**Estimated Memory**: ~100 bytes (stack allocations only)

**Files to Modify**:
- `lib/Gantry/src/Gantry.h` - Add method declarations
- `lib/Gantry/src/Gantry.cpp` - Implement methods

**Testing**:
- Test forward kinematics with known joint configs
- Test inverse kinematics with known poses
- Verify round-trip: forward(inverse(pose)) == pose
- Compare with existing methods to ensure consistency

### 1.3 Add Move Methods Using New Structures

**File**: `lib/Gantry/src/Gantry.h` and `Gantry.cpp`

**Changes**:
- Add overloaded `moveTo()` methods that accept `JointConfig` or `EndEffectorPose`
- Keep existing `moveTo()` and `moveToMm()` methods (backwards compatible)

**New Methods**:
```cpp
/**
 * @brief Move to joint configuration
 * @param joint Target joint configuration
 * @param speed_mm_per_s Motion speed in mm/s (for X and Y axes)
 * @param speed_deg_per_s Motion speed in deg/s (for Theta axis)
 * @param acceleration_mm_per_s2 Acceleration in mm/s² (0 = use default)
 * @param deceleration_mm_per_s2 Deceleration in mm/s² (0 = use default)
 * @return GantryError code
 */
GantryError moveTo(const JointConfig& joint,
                   uint32_t speed_mm_per_s = 50,
                   uint32_t speed_deg_per_s = 30,
                   uint32_t acceleration_mm_per_s2 = 0,
                   uint32_t deceleration_mm_per_s2 = 0);

/**
 * @brief Move to end-effector pose
 * @param pose Target end-effector pose in workspace coordinates
 * @param speed_mm_per_s Motion speed in mm/s (for X and Y axes)
 * @param speed_deg_per_s Motion speed in deg/s (for Theta axis)
 * @param acceleration_mm_per_s2 Acceleration in mm/s² (0 = use default)
 * @param deceleration_mm_per_s2 Deceleration in mm/s² (0 = use default)
 * @return GantryError code
 */
GantryError moveTo(const EndEffectorPose& pose,
                   uint32_t speed_mm_per_s = 50,
                   uint32_t speed_deg_per_s = 30,
                   uint32_t acceleration_mm_per_s2 = 0,
                   uint32_t deceleration_mm_per_s2 = 0);
```

**Implementation Notes**:
- `moveTo(JointConfig)` will convert to workspace coordinates and call `moveToMm()`
- `moveTo(EndEffectorPose)` will use inverse kinematics to get joint config, then call existing methods
- Both will validate limits using `JointConfig::isValid()`

**Estimated Memory**: Minimal (stack allocations only)

**Files to Modify**:
- `lib/Gantry/src/Gantry.h` - Add method declarations
- `lib/Gantry/src/Gantry.cpp` - Implement methods

**Testing**:
- Test movement to joint config
- Test movement to end-effector pose
- Verify limits are checked
- Compare with existing move methods

---

## Phase 2: Trajectory Planning with Waypoints (MEDIUM PRIORITY)

### 2.1 Add Waypoint Structure

**File**: `lib/Gantry/src/Gantry.h`

**Changes**:
- Add `Waypoint` struct for trajectory planning
- Add template `WaypointQueue` class (optional: can be separate file)

**Implementation**:
```cpp
/**
 * @struct Waypoint
 * @brief Waypoint for trajectory planning
 * 
 * Represents a point in a trajectory with associated motion parameters.
 */
struct Waypoint {
    EndEffectorPose pose;      // Target pose in workspace coordinates
    uint32_t speed_mm_per_s;   // Speed for this segment (mm/s)
    uint32_t speed_deg_per_s;  // Speed for theta (deg/s)
    uint32_t acceleration_mm_per_s2;  // Acceleration (mm/s², 0 = use default)
    uint32_t deceleration_mm_per_s2;  // Deceleration (mm/s², 0 = use default)
    
    // Default constructor
    Waypoint() : speed_mm_per_s(50), speed_deg_per_s(30),
                 acceleration_mm_per_s2(0), deceleration_mm_per_s2(0) {}
    
    // Constructor with pose
    Waypoint(const EndEffectorPose& p) : pose(p), speed_mm_per_s(50),
                                         speed_deg_per_s(30),
                                         acceleration_mm_per_s2(0),
                                         deceleration_mm_per_s2(0) {}
};

/**
 * @class WaypointQueue
 * @brief Circular buffer queue for waypoints
 * @tparam MAX_WAYPOINTS Maximum number of waypoints (default: 16)
 * 
 * Thread-safe queue for managing trajectory waypoints.
 */
template<size_t MAX_WAYPOINTS = 16>
class WaypointQueue {
private:
    Waypoint waypoints_[MAX_WAYPOINTS];
    size_t head_ = 0;
    size_t tail_ = 0;
    size_t count_ = 0;
    
public:
    /**
     * @brief Add waypoint to queue
     * @param wp Waypoint to add
     * @return true if added, false if queue is full
     */
    bool push(const Waypoint& wp);
    
    /**
     * @brief Remove waypoint from queue
     * @param wp Reference to store popped waypoint
     * @return true if waypoint was popped, false if queue is empty
     */
    bool pop(Waypoint& wp);
    
    /**
     * @brief Get number of waypoints in queue
     * @return Number of waypoints
     */
    size_t size() const { return count_; }
    
    /**
     * @brief Check if queue is empty
     * @return true if empty
     */
    bool empty() const { return count_ == 0; }
    
    /**
     * @brief Check if queue is full
     * @return true if full
     */
    bool full() const { return count_ >= MAX_WAYPOINTS; }
    
    /**
     * @brief Clear all waypoints from queue
     */
    void clear();
};
```

**Estimated Memory**: ~500 bytes for 16 waypoints (configurable via template parameter)

**Files to Modify**:
- `lib/Gantry/src/Gantry.h` - Add structs (or create `GantryWaypoint.h`)

**Decision**: Keep in `Gantry.h` for simplicity, or create separate header?

### 2.2 Add Trajectory Execution Methods

**File**: `lib/Gantry/src/Gantry.h` and `Gantry.cpp`

**Changes**:
- Add methods to manage waypoint queue
- Add method to execute next waypoint from queue

**New Methods**:
```cpp
// In Gantry class (public section):

/**
 * @brief Set waypoint queue for trajectory planning
 * @tparam MAX_WAYPOINTS Maximum number of waypoints
 * @param queue Reference to waypoint queue
 * @note Only one queue can be active at a time
 */
template<size_t MAX_WAYPOINTS>
void setWaypointQueue(WaypointQueue<MAX_WAYPOINTS>* queue);

/**
 * @brief Execute next waypoint from queue
 * @return GantryError code
 * @note Returns OK if no waypoint available (queue empty)
 *       Returns ALREADY_MOVING if motion already in progress
 */
GantryError executeNextWaypoint();

/**
 * @brief Check if waypoint queue is active
 * @return true if waypoint queue is set and has waypoints
 */
bool hasWaypoints() const;
```

**Implementation Notes**:
- Store pointer to waypoint queue (template requires pointer or separate handling)
- `executeNextWaypoint()` will call `moveTo()` with next waypoint
- Queue execution can be automatic (in `update()`) or manual (user calls `executeNextWaypoint()`)

**Decision**: 
- Option A: Automatic execution (queue processed automatically in `update()`)
- Option B: Manual execution (user must call `executeNextWaypoint()`)

**Recommendation**: Option B (manual) for more control, but add helper method for automatic mode.

**Estimated Memory**: ~8 bytes (pointer to queue)

**Files to Modify**:
- `lib/Gantry/src/Gantry.h` - Add method declarations
- `lib/Gantry/src/Gantry.cpp` - Implement methods

**Testing**:
- Add waypoints to queue
- Execute waypoints sequentially
- Verify queue ordering
- Test with full queue
- Test with empty queue

---

## Phase 3: Advanced Features (LOW PRIORITY - OPTIONAL)

### 3.1 Enhanced Joint Limits Management

**File**: `lib/Gantry/src/Gantry.h` and `Gantry.cpp`

**Changes**:
- Add methods to set/get joint limits dynamically
- Currently limits are compile-time constants

**New Methods**:
```cpp
/**
 * @brief Set joint limits
 * @param x_min, x_max X-axis limits (mm)
 * @param y_min, y_max Y-axis limits (mm)
 * @param theta_min, theta_max Theta limits (degrees)
 */
void setJointLimits(float x_min, float x_max,
                   float y_min, float y_max,
                   float theta_min, float theta_max);

/**
 * @brief Get joint limits
 */
void getJointLimits(float& x_min, float& x_max,
                   float& y_min, float& y_max,
                   float& theta_min, float& theta_max) const;
```

**Estimated Memory**: ~24 bytes (3 pairs of floats)

**Files to Modify**:
- `lib/Gantry/src/Gantry.h` - Add member variables and methods
- `lib/Gantry/src/Gantry.cpp` - Implement methods

**Testing**:
- Set custom limits
- Verify validation works with custom limits
- Test with invalid limits (min > max)

### 3.2 Spline Interpolation (Optional)

**File**: New file `lib/Gantry/src/GantrySpline.h` (optional)

**Changes**:
- Add cubic spline interpolation for smooth trajectories
- Keep as optional feature (can be disabled via define)

**Decision**: Defer to future if needed. Current trapezoidal profiles are sufficient.

### 3.3 Matrix-Based Kinematics (Optional)

**File**: New file `lib/Gantry/src/GantryMatrix.h` (optional)

**Changes**:
- Add ArduinoEigen-based kinematics (if needed)
- Keep as optional feature (requires ArduinoEigen library)

**Decision**: Defer unless matrix operations are specifically needed. Current implementation is sufficient.

---

## Implementation Order

### Recommended Sequence:

1. **Phase 1.1**: Add JointConfig and EndEffectorPose structures
2. **Phase 1.2**: Implement enhanced kinematics methods
3. **Phase 1.3**: Add move methods using new structures
4. **Phase 2.1**: Add Waypoint and WaypointQueue structures
5. **Phase 2.2**: Add trajectory execution methods
6. **Phase 3**: Advanced features (if needed)

## Backwards Compatibility

All existing APIs will remain unchanged:
- ✅ `moveTo(int32_t x_pulses, int32_t y_stub, int32_t theta_stub, ...)` - unchanged
- ✅ `moveToMm(int32_t x_mm, int32_t y_mm, int32_t theta_deg, ...)` - unchanged
- ✅ `forwardKinematicsX()` - unchanged
- ✅ `inverseKinematicsX()` - unchanged
- ✅ All existing getter methods - unchanged
- ✅ All existing configuration methods - unchanged

New methods are additions only - no breaking changes.

## Testing Strategy

For each phase:

1. **Unit Tests** (if test framework available):
   - Test new structures and methods in isolation
   - Test kinematics round-trips
   - Test limit validation

2. **Integration Tests**:
   - Test new methods with real hardware (or simulation)
   - Compare results with existing methods
   - Test backwards compatibility (existing code still works)

3. **Memory Profiling**:
   - Measure memory usage before and after each phase
   - Ensure memory footprint remains acceptable (< 10KB total)

4. **Performance Testing**:
   - Ensure new methods don't add significant overhead
   - Compare execution time with existing methods

## Documentation Updates

For each phase, update:
- `lib/Gantry/README.md` - Add new features to feature list
- `lib/Gantry/src/Gantry.h` - Add Doxygen documentation for new structures/methods
- `lib/Gantry/examples/` - Add example code showing new features

## Timeline Estimate

- **Phase 1**: 2-3 days (core structures and methods)
- **Phase 2**: 2-3 days (waypoint trajectory planning)
- **Phase 3**: 1-2 days (advanced features, if needed)

**Total**: ~1-2 weeks for Phases 1-2 (recommended), Phase 3 optional.

## Memory Budget

Current usage: ~1KB
After Phase 1: ~1.1KB (+100 bytes)
After Phase 2: ~1.6KB (+500 bytes for waypoint queue)
After Phase 3: ~1.8KB (+200 bytes)

**Total**: Well within 320KB RAM limit (0.5% usage)

## Success Criteria

1. ✅ All existing code continues to work without modification
2. ✅ New structures provide cleaner API for kinematics
3. ✅ Waypoint trajectory planning works reliably
4. ✅ Memory usage remains < 2KB for all enhancements
5. ✅ Documentation is complete and clear
6. ✅ Code passes all tests

## Next Steps

1. Review and approve this plan
2. Create feature branch: `feature/gantry-enhancements`
3. Implement Phase 1.1 (structures)
4. Test Phase 1.1
5. Continue with remaining phases
6. Update documentation
7. Merge to main branch

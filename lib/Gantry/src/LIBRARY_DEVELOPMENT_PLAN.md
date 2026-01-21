# Gantry Library Development Plan for WT32-ETH01

## Overview

This document outlines a comprehensive plan for developing a production-ready gantry control library optimized for WT32-ETH01 (ESP32 with 4MB flash, ~320KB RAM).

**Target Configuration:**
- **X-axis**: Horizontal prismatic joint (base, right-to-left)
- **Y-axis**: Vertical prismatic joint (down-to-up)
- **Theta**: Rotational joint at end of Y-axis

## Library Goals

1. **Modular Design**: Separate concerns into distinct modules
2. **Memory Efficient**: Optimized for embedded systems (<10KB RAM usage)
3. **PlatformIO Ready**: Proper library structure with configuration
4. **Well Documented**: Doxygen comments, examples, README
5. **Extensible**: Easy to add new features (planning, kinematics, etc.)
6. **Production Ready**: Thread-safe considerations, error handling, testing

---

## Phase 1: Library Structure & Organization

### 1.1 Directory Structure

```
lib/Gantry/
├── library.json              # PlatformIO library configuration
├── README.md                 # User documentation
├── LICENSE                   # License file
├── CHANGELOG.md              # Version history
├── src/                      # Source files
│   ├── Gantry.h              # Main class header
│   ├── Gantry.cpp            # Main class implementation
│   ├── GantryConfig.h        # Configuration structures
│   ├── GantryConfig.cpp      # Configuration implementation
│   ├── GantryKinematics.h    # Kinematics functions
│   ├── GantryKinematics.cpp  # Kinematics implementation
│   ├── GantryTrajectory.h    # Trajectory planning
│   └── GantryTrajectory.cpp  # Trajectory implementation
├── examples/                 # Example sketches
│   ├── BasicMotion/
│   │   └── BasicMotion.ino
│   ├── HomingExample/
│   │   └── HomingExample.ino
│   ├── MultiWaypoint/
│   │   └── MultiWaypoint.ino
│   └── FreeRTOSExample/
│       └── FreeRTOSExample.ino
├── docs/                     # Additional documentation
│   ├── API.md                # API reference
│   ├── ARCHITECTURE.md       # Architecture overview
│   ├── COORDINATE_SYSTEMS.md # Coordinate system documentation
│   └── KINEMATICS.md         # Kinematics documentation
└── test/                     # Unit tests (optional, for development)
    └── test_gantry.cpp
```

### 1.2 PlatformIO Library Configuration (`library.json`)

```json
{
  "name": "Gantry",
  "version": "1.0.0",
  "description": "Multi-axis gantry control library for ESP32 (WT32-ETH01)",
  "keywords": [
    "gantry",
    "motion-control",
    "esp32",
    "servo",
    "kinematics",
    "robotics"
  ],
  "authors": {
    "name": "Your Name",
    "email": "your.email@example.com"
  },
  "repository": {
    "type": "git",
    "url": "https://github.com/yourusername/gantry-library"
  },
  "license": "MIT",
  "homepage": "https://github.com/yourusername/gantry-library",
  "frameworks": "arduino",
  "platforms": "espressif32",
  "dependencies": {
    "SDF08NK8X": "*"
  },
  "examples": [
    "examples/BasicMotion/BasicMotion.ino",
    "examples/HomingExample/HomingExample.ino",
    "examples/MultiWaypoint/MultiWaypoint.ino",
    "examples/FreeRTOSExample/FreeRTOSExample.ino"
  ]
}
```

---

## Phase 2: Core Module Design

### 2.1 Configuration Module (`GantryConfig.h`)

**Purpose**: Define data structures for gantry configuration, joint states, and end-effector poses.

```cpp
// GantryConfig.h - Configuration structures

#ifndef GANTRY_CONFIG_H
#define GANTRY_CONFIG_H

#include <stdint.h>
#include <Arduino.h>

namespace Gantry {

/**
 * @struct JointConfig
 * @brief Joint space configuration (internal representation)
 */
struct JointConfig {
    float x;      // X-axis position (mm) - horizontal (right-to-left)
    float y;      // Y-axis position (mm) - vertical (down-to-up)
    float theta;  // Theta angle (degrees) - rotation around Y-axis
    
    // Constructors
    JointConfig() : x(0.0f), y(0.0f), theta(0.0f) {}
    JointConfig(float x_val, float y_val, float theta_val) 
        : x(x_val), y(y_val), theta(theta_val) {}
    
    // Operators
    JointConfig operator+(const JointConfig& other) const {
        return JointConfig(x + other.x, y + other.y, theta + other.theta);
    }
    
    JointConfig operator-(const JointConfig& other) const {
        return JointConfig(x - other.x, y - other.y, theta - other.theta);
    }
};

/**
 * @struct JointLimits
 * @brief Joint limits for validation
 */
struct JointLimits {
    float x_min, x_max;
    float y_min, y_max;
    float theta_min, theta_max;
    
    JointLimits() 
        : x_min(0.0f), x_max(0.0f),
          y_min(0.0f), y_max(0.0f),
          theta_min(-90.0f), theta_max(90.0f) {}
    
    bool isValid(const JointConfig& config) const {
        return (config.x >= x_min && config.x <= x_max) &&
               (config.y >= y_min && config.y <= y_max) &&
               (config.theta >= theta_min && config.theta <= theta_max);
    }
};

/**
 * @struct EndEffectorPose
 * @brief End-effector pose in workspace/cartesian coordinates
 */
struct EndEffectorPose {
    float x, y, z;    // Position (mm)
    float theta;      // Orientation (degrees)
    
    EndEffectorPose() : x(0.0f), y(0.0f), z(0.0f), theta(0.0f) {}
    EndEffectorPose(float x_val, float y_val, float z_val, float theta_val)
        : x(x_val), y(y_val), z(z_val), theta(theta_val) {}
};

/**
 * @struct KinematicParameters
 * @brief Mechanical parameters for kinematics
 */
struct KinematicParameters {
    float y_axis_z_offset_mm;      // Y-axis Z offset from X (default: 80mm)
    float theta_x_offset_mm;       // Theta X offset from Y vertical (default: -55mm)
    float gripper_y_offset_mm;     // Gripper Y offset (default: 385mm)
    float gripper_z_offset_mm;     // Gripper Z offset (default: 80mm)
    float x_axis_ball_screw_pitch_mm; // Ball-screw pitch (default: 40mm)
    
    KinematicParameters()
        : y_axis_z_offset_mm(80.0f),
          theta_x_offset_mm(-55.0f),
          gripper_y_offset_mm(385.0f),
          gripper_z_offset_mm(80.0f),
          x_axis_ball_screw_pitch_mm(40.0f) {}
};

/**
 * @struct GantryConfig
 * @brief Complete gantry configuration
 */
struct GantryConfig {
    JointLimits limits;
    KinematicParameters kinematic_params;
    float workspace_origin_offset_mm;
    
    GantryConfig() : workspace_origin_offset_mm(0.0f) {}
};

} // namespace Gantry

#endif // GANTRY_CONFIG_H
```

### 2.2 Kinematics Module (`GantryKinematics.h`)

**Purpose**: Forward and inverse kinematics functions.

```cpp
// GantryKinematics.h - Kinematics functions

#ifndef GANTRY_KINEMATICS_H
#define GANTRY_KINEMATICS_H

#include "GantryConfig.h"

namespace Gantry {

/**
 * @class Kinematics
 * @brief Forward and inverse kinematics for 3-axis gantry
 */
class Kinematics {
public:
    /**
     * @brief Forward kinematics: Joint space -> Cartesian space
     * @param joints Joint configuration
     * @param params Kinematic parameters
     * @return End-effector pose
     */
    static EndEffectorPose forward(const JointConfig& joints, 
                                    const KinematicParameters& params);
    
    /**
     * @brief Inverse kinematics: Cartesian space -> Joint space
     * @param pose End-effector pose
     * @param params Kinematic parameters
     * @return Joint configuration
     */
    static JointConfig inverse(const EndEffectorPose& pose,
                                const KinematicParameters& params);
    
    /**
     * @brief Validate joint configuration against limits
     * @param joints Joint configuration
     * @param limits Joint limits
     * @return true if valid
     */
    static bool validate(const JointConfig& joints, const JointLimits& limits);
};

} // namespace Gantry

#endif // GANTRY_KINEMATICS_H
```

### 2.3 Trajectory Planning Module (`GantryTrajectory.h`)

**Purpose**: Trajectory planning and waypoint management.

```cpp
// GantryTrajectory.h - Trajectory planning

#ifndef GANTRY_TRAJECTORY_H
#define GANTRY_TRAJECTORY_H

#include "GantryConfig.h"
#include <stdint.h>

namespace Gantry {

/**
 * @struct Waypoint
 * @brief Single waypoint in a trajectory
 */
struct Waypoint {
    JointConfig position;
    float speed_mm_per_s;
    float acceleration_mm_per_s2;
    
    Waypoint() : speed_mm_per_s(200.0f), acceleration_mm_per_s2(1000.0f) {}
};

/**
 * @struct TrapezoidalProfile
 * @brief Trapezoidal velocity profile parameters
 */
struct TrapezoidalProfile {
    float t_accel;      // Acceleration time (s)
    float t_cruise;     // Cruise time (s)
    float t_decel;      // Deceleration time (s)
    float total_time;   // Total time (s)
    float max_speed;    // Maximum speed
    bool valid;         // Profile validity flag
    
    TrapezoidalProfile() : t_accel(0), t_cruise(0), t_decel(0), 
                          total_time(0), max_speed(0), valid(false) {}
};

/**
 * @class TrajectoryPlanner
 * @brief Simple trajectory planner for point-to-point motion
 */
class TrajectoryPlanner {
public:
    /**
     * @brief Calculate trapezoidal velocity profile
     * @param start Start position
     * @param target Target position
     * @param max_speed Maximum speed
     * @param acceleration Acceleration rate
     * @param deceleration Deceleration rate
     * @return Trapezoidal profile
     */
    static TrapezoidalProfile calculateProfile(float start, float target,
                                               float max_speed,
                                               float acceleration,
                                               float deceleration);
    
    /**
     * @brief Interpolate position from trapezoidal profile
     * @param profile Trapezoidal profile
     * @param start Start position
     * @param target Target position
     * @param elapsed_s Elapsed time (seconds)
     * @return Interpolated position
     */
    static float interpolate(const TrapezoidalProfile& profile,
                            float start, float target, float elapsed_s);
};

} // namespace Gantry

#endif // GANTRY_TRAJECTORY_H
```

### 2.4 Main Gantry Class Structure

**Purpose**: Keep existing `Gantry` class but integrate new modules.

The main `Gantry` class will:
- Use `GantryConfig` for configuration
- Use `GantryKinematics` for kinematics
- Use `GantryTrajectory` for trajectory planning
- Maintain backward compatibility with existing API

---

## Phase 3: Implementation Phases

### Phase 3.1: Core Refactoring (Week 1-2)

**Tasks:**
1. ✅ Create `GantryConfig.h/cpp` with configuration structures
2. ✅ Extract kinematic functions to `GantryKinematics.h/cpp`
3. ✅ Extract trajectory functions to `GantryTrajectory.h/cpp`
4. ✅ Update `Gantry.h/cpp` to use new modules
5. ✅ Ensure backward compatibility
6. ✅ Update includes and namespaces

**Deliverables:**
- Modular source code
- All tests passing
- Backward compatibility maintained

### Phase 3.2: Library Configuration (Week 2)

**Tasks:**
1. ✅ Create `library.json` with proper configuration
2. ✅ Create `LICENSE` file
3. ✅ Update `README.md` with library documentation
4. ✅ Create `CHANGELOG.md`
5. ✅ Test PlatformIO library discovery

**Deliverables:**
- Properly configured library.json
- Documentation files
- Library installs correctly via PlatformIO

### Phase 3.3: Documentation (Week 3)

**Tasks:**
1. ✅ Add Doxygen comments to all public APIs
2. ✅ Create `docs/API.md` with API reference
3. ✅ Create `docs/ARCHITECTURE.md`
4. ✅ Create `docs/COORDINATE_SYSTEMS.md`
5. ✅ Create `docs/KINEMATICS.md`
6. ✅ Generate Doxygen documentation

**Deliverables:**
- Complete API documentation
- Architecture documentation
- User guides

### Phase 3.4: Examples (Week 3-4)

**Tasks:**
1. ✅ Create `examples/BasicMotion/BasicMotion.ino`
2. ✅ Create `examples/HomingExample/HomingExample.ino`
3. ✅ Create `examples/MultiWaypoint/MultiWaypoint.ino`
4. ✅ Create `examples/FreeRTOSExample/FreeRTOSExample.ino`
5. ✅ Test all examples on hardware

**Deliverables:**
- Working example sketches
- Example documentation

### Phase 3.5: Testing & Validation (Week 4)

**Tasks:**
1. ✅ Test on WT32-ETH01 hardware
2. ✅ Memory profiling (ensure <10KB RAM usage)
3. ✅ Performance profiling
4. ✅ Edge case testing
5. ✅ Thread safety validation (if applicable)

**Deliverables:**
- Test report
- Performance benchmarks
- Memory usage report

---

## Phase 4: Advanced Features (Future)

### 4.1 Enhanced Trajectory Planning

- Multi-waypoint trajectories
- Cubic spline interpolation
- S-curve profiles
- Collision avoidance (if needed)

### 4.2 Enhanced Kinematics

- Full 6-DOF support (if extending)
- Jacobian computation
- Singularity detection
- Workspace analysis

### 4.3 Configuration Management

- Persistent configuration (EEPROM/Preferences)
- Configuration validation
- Factory reset
- Calibration data storage

---

## Memory Budget (WT32-ETH01)

Target: <10KB RAM usage

| Component | Estimated RAM |
|-----------|---------------|
| Gantry class instance | 1-2 KB |
| Configuration structures | 0.5 KB |
| Trajectory buffers | 0.5-1 KB |
| Waypoint queue (32 waypoints) | 0.5 KB |
| Temporary calculations | <1 KB |
| **Total** | **~4-5 KB** |

**Status**: ✅ Well within 320KB RAM limit

---

## Thread Safety Considerations

**Current Status**: Not thread-safe (documented)

**Options for Future:**
1. Add mutex wrapper class
2. Document thread-safe usage patterns
3. Provide FreeRTOS-specific implementations

**Recommendation**: Keep non-thread-safe, document usage patterns (current approach)

---

## Versioning Strategy

Follow Semantic Versioning (SemVer):
- **MAJOR**: Breaking API changes
- **MINOR**: New features (backward compatible)
- **PATCH**: Bug fixes

**Initial Version**: 1.0.0

---

## Testing Strategy

### Unit Testing (Optional)
- Use ArduinoUnit or similar
- Test kinematics functions
- Test trajectory planning
- Test configuration validation

### Integration Testing
- Test on hardware
- Test with real servo driver
- Test homing/calibration
- Test motion profiles

### Memory Testing
- Monitor heap usage
- Test with maximum waypoints
- Test edge cases

---

## Migration Plan (From Current Code)

### Step 1: Create New Modules
- Create `GantryConfig.h/cpp`
- Create `GantryKinematics.h/cpp`
- Create `GantryTrajectory.h/cpp`

### Step 2: Move Existing Code
- Extract configuration structures
- Extract kinematic functions
- Extract trajectory functions

### Step 3: Update Main Class
- Include new modules
- Update implementation to use modules
- Maintain existing API

### Step 4: Test
- Ensure all functionality works
- Test backward compatibility
- Update examples

### Step 5: Move to Library
- Move files to `lib/Gantry/src/`
- Create `library.json`
- Update includes in main project

---

## Success Criteria

✅ **Library Structure:**
- Proper PlatformIO library structure
- Modular design with separate concerns
- All files in correct locations

✅ **Functionality:**
- All existing features work
- Backward compatibility maintained
- New modular design functional

✅ **Documentation:**
- Complete API documentation
- Examples for common use cases
- Architecture documentation

✅ **Quality:**
- Memory usage <10KB
- Code passes compilation
- Examples work on hardware
- Thread safety documented

---

## Timeline Summary

| Phase | Duration | Deliverables |
|-------|----------|--------------|
| Phase 1: Structure | 1 day | Directory structure, library.json |
| Phase 2: Design | 2 days | Module headers, design docs |
| Phase 3.1: Core Refactoring | 1-2 weeks | Modular source code |
| Phase 3.2: Library Config | 2-3 days | library.json, LICENSE, CHANGELOG |
| Phase 3.3: Documentation | 1 week | Complete documentation |
| Phase 3.4: Examples | 1 week | Working examples |
| Phase 3.5: Testing | 1 week | Test reports, validation |
| **Total** | **4-6 weeks** | **Production-ready library** |

---

## Next Steps

1. **Review this plan** and adjust as needed
2. **Start with Phase 1**: Create directory structure
3. **Begin Phase 2**: Design module interfaces
4. **Execute Phase 3**: Implement in phases
5. **Iterate**: Test and refine

---

## Notes

- This plan maintains backward compatibility with existing code
- The modular design allows for future extensions
- Memory efficiency is prioritized for embedded systems
- Documentation is emphasized for library distribution
- Examples help users understand usage patterns

---

**Last Updated**: [Current Date]
**Version**: 1.0.0
**Status**: Draft - Ready for Review

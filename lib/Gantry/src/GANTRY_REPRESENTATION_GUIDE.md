# 3-Axis Gantry Representation in C++ - Guide

## Current Representation Analysis

Your existing `Gantry` class already provides a solid foundation for representing a 3-axis gantry system:

- **X-axis**: Horizontal prismatic joint (base, right-to-left)
- **Y-axis**: Vertical prismatic joint (down-to-up)
- **Theta**: Rotational joint at the end of the Y-axis

### Current Structure

Your implementation uses:
- Joint space representation: `currentX_`, `currentY_`, `currentTheta_`
- Forward/inverse kinematics functions
- Trapezoidal velocity profiles
- Workspace coordinate transformations

## Recommended Approaches for WT32-ETH01 (4MB RAM)

Given the hardware constraints (ESP32 with ~320KB RAM, 4MB flash), here are suitable options:

### 1. **Custom Lightweight Implementation (RECOMMENDED)**

Your current approach is actually ideal for embedded systems. Enhance it with a formal kinematic model structure:

```cpp
// Enhanced joint configuration representation
struct GantryConfiguration {
    // Joint positions (joint space)
    float q_x;      // X-axis position (mm) - horizontal
    float q_y;      // Y-axis position (mm) - vertical
    float q_theta;  // Theta angle (degrees) - rotation
    
    // Joint limits
    float q_x_min, q_x_max;
    float q_y_min, q_y_max;
    float q_theta_min, q_theta_max;
    
    // Kinematic parameters (DH-like parameters for your gantry)
    float link_lengths[3];  // Link lengths if needed
    float offsets[3];       // Joint offsets
};

// End-effector pose (cartesian space)
struct EndEffectorPose {
    float x;      // X position (mm)
    float y;      // Y position (mm)
    float z;      // Z position (mm) - may be constant
    float theta;  // Orientation (degrees)
};
```

**Pros:**
- Minimal memory footprint
- Full control over representation
- Already implemented and working
- Easy to optimize for your specific needs

**Cons:**
- Need to implement all kinematic functions yourself

### 2. **ArduinoEigen (Lightweight Matrix Library)**

If you need matrix operations for kinematics, ArduinoEigen is a lightweight port:

```cpp
#include <Eigen.h>  // ArduinoEigen

// Use fixed-size matrices for efficiency
using Matrix3f = Eigen::Matrix<float, 3, 3>;
using Vector3f = Eigen::Matrix<float, 3, 1>;

// Transformation matrix representation
Matrix3f forwardKinematics(const Vector3f& joint_angles) {
    // Build transformation matrices
    // Fixed-size matrices are stack-allocated (efficient)
}
```

**Pros:**
- Lightweight (only what you include)
- Fixed-size matrices are stack-allocated
- No dynamic memory allocation
- Suitable for 4MB flash systems

**Cons:**
- Still adds some overhead
- May be overkill for simple gantry

**Installation:** PlatformIO library ID: `5776` (ArduinoEigen)

### 3. **Simple Math Library (Alternative)**

For basic trigonometry and vector math without matrix overhead:

```cpp
// Simple kinematic functions using basic math
float forwardKinematicsX(float x_joint, float theta) {
    return x_joint + THETA_X_OFFSET_MM * cos(radians(theta));
}

float forwardKinematicsY(float x_joint, float y_joint, float theta) {
    return y_joint;  // Y is independent in your case
}
```

## Motion Planning for WT32-ETH01

### Recommended: Simple Trajectory Planning (What You Have)

Your current trapezoidal velocity profile implementation is perfect for embedded systems:

```cpp
// Already implemented in your StubProfile structure
// - Trapezoidal velocity profiles
// - Linear interpolation
// - Simple point-to-point motion
```

**Capabilities:**
- ✅ Point-to-point motion
- ✅ Velocity/acceleration limits
- ✅ Multi-axis coordination
- ✅ Low memory footprint
- ✅ Real-time execution

### Advanced Options (If Needed)

#### A. Simple Spline Interpolation

For smooth curves, use cubic splines (lightweight):

```cpp
// Cubic spline for smooth trajectories
struct SplinePoint {
    float t;        // Time
    float position; // Position
    float velocity; // Velocity
};

// Simple cubic spline interpolation
float cubicSpline(float t, const SplinePoint& p0, const SplinePoint& p1) {
    // Hermite cubic interpolation
    // Memory: O(n) where n = number of waypoints
}
```

**Memory**: ~100-200 bytes per waypoint

#### B. Multi-Segment Trajectories

Extend your current implementation to support waypoint sequences:

```cpp
struct Waypoint {
    float x, y, theta;
    float speed;
};

// Queue of waypoints (circular buffer)
static constexpr size_t MAX_WAYPOINTS = 32;
Waypoint waypoint_queue[MAX_WAYPOINTS];
size_t queue_head = 0;
size_t queue_tail = 0;
```

**Memory**: ~500 bytes for 32 waypoints

### Libraries to AVOID (Too Heavy)

These are too resource-intensive for WT32-ETH01:

- ❌ **KDL (Orocos Kinematics and Dynamics Library)**: Requires several MB RAM
- ❌ **Pinocchio**: Complex robot dynamics, requires significant RAM
- ❌ **OMPL (Open Motion Planning Library)**: Requires GB of RAM for planning
- ❌ **MoveIt**: ROS-based, requires Linux and significant resources
- ❌ **Robowflex**: ROS-based, not suitable for embedded

## Recommended Library Stack for Your Gantry

### Minimal Approach (Current + Enhancements)

1. **Custom kinematics** (what you have) ✅
2. **ArduinoEigen** (optional, if matrix operations needed)
3. **Simple spline library** (if smooth curves needed)

### Example Enhanced Structure

```cpp
class EnhancedGantry {
private:
    // Joint space configuration
    struct JointConfig {
        float x;      // Horizontal position (mm)
        float y;      // Vertical position (mm)
        float theta;  // Rotation angle (degrees)
    };
    
    // End-effector pose (workspace coordinates)
    struct EndEffectorPose {
        float x, y, z;
        float theta;
    };
    
    // Forward kinematics
    EndEffectorPose forwardKinematics(const JointConfig& joint) const {
        EndEffectorPose pose;
        pose.x = joint.x + THETA_X_OFFSET_MM;
        pose.y = joint.y;
        pose.z = Y_AXIS_Z_OFFSET_MM;  // Constant
        pose.theta = joint.theta;
        return pose;
    }
    
    // Inverse kinematics
    JointConfig inverseKinematics(const EndEffectorPose& pose) const {
        JointConfig joint;
        joint.x = pose.x - THETA_X_OFFSET_MM;
        joint.y = pose.y;
        joint.theta = pose.theta;
        return joint;
    }
};
```

## Coordinate System Clarification

Based on your note:
- **X-axis**: Horizontal (right-to-left) - this is your base prismatic joint
- **Y-axis**: Vertical (down-to-up) - this is your vertical prismatic joint  
- **Theta**: Rotation around Y-axis (at end of vertical axis)

Your current representation is correct! The key insight is that for a gantry:
- X and Y are independent (orthogonal prismatic joints)
- Theta rotates around Y, so it affects orientation but not X/Y position in your current kinematics

## Memory Budget Estimate

For WT32-ETH01 with 320KB RAM:

- Current Gantry class: ~500-1000 bytes
- Trapezoidal profiles: ~100 bytes per axis
- Waypoint queue (32 waypoints): ~500 bytes
- ArduinoEigen (if used): ~2-5KB (compile-time, mostly flash)
- **Total**: ~2-6KB RAM usage (well within limits)

## Conclusion

**Your current implementation is excellent for embedded systems!**

**Recommendations:**
1. ✅ Keep your custom implementation (it's ideal)
2. ✅ Consider ArduinoEigen only if you need matrix operations
3. ✅ Add simple spline interpolation if you need smooth curves
4. ✅ Extend with waypoint queues for multi-point trajectories
5. ❌ Avoid heavy robotics libraries (KDL, Pinocchio, OMPL, MoveIt)

Your approach of simple trapezoidal profiles + custom kinematics is the standard approach for embedded motion control systems.

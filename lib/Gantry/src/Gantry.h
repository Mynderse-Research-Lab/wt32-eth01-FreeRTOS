#ifndef GANTRY_H
#define GANTRY_H

#include "SDF08NK8X.h"
#include <Arduino.h>

#if defined(ARDUINO_ARCH_ESP32)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#endif

/**
 * @enum GantryError
 * @brief Error codes for Gantry operations
 */
enum class GantryError {
  OK,                      // Operation successful
  NOT_INITIALIZED,         // Gantry not initialized
  MOTOR_NOT_ENABLED,       // Motor not enabled
  ALREADY_MOVING,          // Motion already in progress
  INVALID_POSITION,        // Position out of valid range
  INVALID_PARAMETER,       // Invalid parameter (speed, etc.)
  TIMEOUT,                 // Operation timed out
  LIMIT_SWITCH_FAILED,     // Limit switch not triggered
  CALIBRATION_FAILED,      // Calibration failed
  CONVERSION_ERROR         // Unit conversion error
};

/**
 * @enum HomingStatus
 * @brief Status of homing operation
 */
enum class HomingStatus {
  IDLE,        // Not homing
  IN_PROGRESS, // Homing in progress
  COMPLETE,    // Homing completed successfully
  FAILED       // Homing failed
};

/**
 * @struct GantryStatus
 * @brief Complete status snapshot of gantry system
 */
struct GantryStatus {
  // Position (current)
  int32_t currentX_mm;        // Current X position in mm
  int32_t currentY_mm;        // Current Y position in mm
  int32_t currentTheta_deg;   // Current Theta angle in degrees
  
  // Target positions
  int32_t targetX_mm;         // Target X position in mm
  int32_t targetY_mm;         // Target Y position in mm
  int32_t targetTheta_deg;    // Target Theta angle in degrees
  
  // Motion state
  bool isBusy;                // True if any axis is moving
  bool xMoving;               // True if X axis is moving
  bool yMoving;               // True if Y axis is moving (simulated)
  bool thetaMoving;           // True if Theta axis is moving (simulated)
  
  // System state
  bool initialized;           // True if initialized
  bool enabled;               // True if motor enabled
  bool gripperActive;         // True if gripper closed
  
  // Configuration
  int32_t axisLength_mm;      // X-axis length in mm (after calibration)
  int32_t workspaceOriginOffset_mm; // Workspace origin offset in mm
  
  // Timestamp
  uint32_t lastUpdate_ms;     // Timestamp of status update
};

/**
 * @struct JointConfig
 * @brief Joint space configuration for 3-axis gantry
 * 
 * Represents the gantry configuration in joint space (joint positions).
 * Joint space coordinates use mm for X/Y positions and degrees for Theta rotation.
 * 
 * Coordinate System:
 * - q_x: X-axis position (mm) - horizontal (right-to-left), in home coordinates
 * - q_y: Y-axis position (mm) - vertical (down-to-up)
 * - q_theta: Theta angle (degrees) - rotation around Y-axis
 */
struct JointConfig {
    float q_x;      // X-axis position (mm) - horizontal (right-to-left)
    float q_y;      // Y-axis position (mm) - vertical (down-to-up)
    float q_theta;  // Theta angle (degrees) - rotation around Y-axis
    
    // Default constructor
    JointConfig() : q_x(0.0f), q_y(0.0f), q_theta(0.0f) {}
    
    // Constructor with values
    JointConfig(float x, float y, float theta) : q_x(x), q_y(y), q_theta(theta) {}
    
    /**
     * @brief Check if joint configuration is within default limits
     * @return true if all joints are within valid range
     * @note X limits are approximate (actual X limit depends on axisLength_ after calibration)
     */
    bool isValid() const;
    
    /**
     * @brief Get default joint limits
     * @param x_min, x_max X-axis limits (mm) - x_max is default maximum
     * @param y_min, y_max Y-axis limits (mm)
     * @param theta_min, theta_max Theta limits (degrees)
     * @note Actual X limits depend on axisLength_ after calibration (check via Gantry::getAxisLength())
     */
    static void getLimits(float& x_min, float& x_max,
                         float& y_min, float& y_max,
                         float& theta_min, float& theta_max);
    
    // Default joint limits (static constants)
    static constexpr float X_MIN = 0.0f;
    static constexpr float X_MAX_DEFAULT = 10000.0f;  // Default maximum (actual limit depends on calibration)
    static constexpr float Y_MIN = 0.0f;
    static constexpr float Y_MAX = 150.0f;  // Y_AXIS_STROKE_LENGTH_MM
    static constexpr float THETA_MIN = -90.0f;  // THETA_MIN_DEGREES
    static constexpr float THETA_MAX = 90.0f;   // THETA_MAX_DEGREES
};

/**
 * @struct EndEffectorPose
 * @brief End-effector pose in workspace/cartesian space
 * 
 * Represents the end-effector position and orientation in workspace coordinates.
 * Workspace coordinates use mm for positions and degrees for orientation.
 * 
 * Coordinate System:
 * - x: X position (mm) - workspace coordinates
 * - y: Y position (mm) - workspace coordinates
 * - z: Z position (mm) - typically constant for gantry
 * - theta: Orientation (degrees) - rotation around Y-axis
 */
struct EndEffectorPose {
    float x;      // X position (mm) - workspace coordinates
    float y;      // Y position (mm) - workspace coordinates
    float z;      // Z position (mm) - typically constant for gantry
    float theta;  // Orientation (degrees)
    
    // Default constructor
    EndEffectorPose() : x(0.0f), y(0.0f), z(0.0f), theta(0.0f) {}
    
    // Constructor with values
    EndEffectorPose(float x_val, float y_val, float z_val, float theta_val)
        : x(x_val), y(y_val), z(z_val), theta(theta_val) {}
};

/**
 * @class Gantry
 * @brief XY Gantry control system for multi-axis motion control
 * 
 * Mechanical Layout:
 * - X-axis: Horizontal axis (implemented with SDF08NK8X servo driver, ball-screw with X_AXIS_BALL_SCREW_PITCH_MM pitch)
 * - Y-axis: Vertical axis mounted on X with Y_AXIS_Z_OFFSET_MM Z offset (simulated/stub in this version)
 * - Theta: Rotary axis along Y direction, offset THETA_X_OFFSET_MM in X from Y vertical (simulated/stub)
 * - Theta range: THETA_MIN_DEGREES to THETA_MAX_DEGREES (0° parallel to -Z axis)
 * - Y-axis stroke length: Y_AXIS_STROKE_LENGTH_MM
 * - Gripper: Mounted colinear to Y theta axis
 * - Gripper center: GRIPPER_Y_OFFSET_MM offset in Y, GRIPPER_Z_OFFSET_MM in Z from X origin
 *
 * @note Y and Theta axes are SIMULATED (stub implementation) in this initial version.
 *       They will be replaced with actual hardware drivers in future versions.
 * 
 * THREAD SAFETY:
 * This class is NOT thread-safe. All methods modify or read shared internal state
 * (position tracking, motion state, configuration) without synchronization.
 * 
 * Safe Usage Patterns:
 * - Single Task: Call all methods from one FreeRTOS task (recommended)
 * - Multiple Tasks: Protect all method calls with a mutex (see example below)
 * - Update Loop: update() MUST be called frequently (every loop iteration) from
 *   the same task that calls other methods, or from a dedicated update task with
 *   mutex protection
 * 
 * What Needs Protection:
 * - All public methods (state modification: moveTo, home, calibrate, enable, etc.)
 * - All state queries (getCurrentX, getStatus, isBusy, etc.)
 * - Configuration changes (setWorkspaceOriginOffset, setBacklashCompensation, etc.)
 * - Callback registration (setMotionCompleteCallback)
 * 
 * Unsafe Operations (will cause race conditions):
 * - Calling moveTo() from one task while update() runs in another (without mutex)
 * - Reading getStatus() while home() modifies state (without mutex)
 * - Multiple tasks calling any methods simultaneously (without mutex)
 * 
 * Example (Single Task - Recommended):
 * @code
 * void gantryTask(void *param) {
 *   Gantry *gantry = (Gantry *)param;
 *   while (1) {
 *     gantry->update();  // Must be called frequently
 *     // ... other operations ...
 *     gantry->moveToMm(100, 50, 0);
 *     vTaskDelay(pdMS_TO_TICKS(10));
 *   }
 * }
 * @endcode
 * 
 * Example (Multiple Tasks with Mutex):
 * @code
 * SemaphoreHandle_t gantry_mutex = xSemaphoreCreateMutex();
 * 
 * void controlTask(void *param) {
 *   Gantry *gantry = (Gantry *)param;
 *   while (1) {
 *     if (xSemaphoreTake(gantry_mutex, pdMS_TO_TICKS(100))) {
 *       gantry->moveToMm(100, 50, 0);
 *       xSemaphoreGive(gantry_mutex);
 *     }
 *     vTaskDelay(pdMS_TO_TICKS(100));
 *   }
 * }
 * 
 * void updateTask(void *param) {
 *   Gantry *gantry = (Gantry *)param;
 *   while (1) {
 *     if (xSemaphoreTake(gantry_mutex, portMAX_DELAY)) {
 *       gantry->update();  // Must be called frequently
 *       xSemaphoreGive(gantry_mutex);
 *     }
 *     vTaskDelay(pdMS_TO_TICKS(1));
 *   }
 * }
 * @endcode
 * 
 * COORDINATE SYSTEMS:
 * The library uses two coordinate systems that are automatically transformed:
 * - Workspace Coordinates (user-facing): mm relative to workspace origin
 *   - Used by: moveToMm(), getCurrentX(), getTargetX(), moveRelative()
 *   - Workspace origin = home position + workspace_origin_offset_mm_
 *   - Example: If offset=50mm, workspace origin is 50mm from home
 * - Home Coordinates (internal): pulses relative to home position (0 pulses = home)
 *   - Used by: moveTo() (pulse-based API)
 *   - Internal storage (currentX_, targetX_) uses home coordinates
 * 
 * Coordinate Transformation:
 * - Writing (moveToMm): workspace_mm → convert to home_pulses → subtract offset → store
 * - Reading (getCurrentX/TargetX): stored_pulses → convert to home_mm → add offset → workspace_mm
 * 
 * Example with workspace_origin_offset_mm_ = 50mm:
 * - moveToMm(100mm): Moves to 50mm from home, getTargetX() returns 100mm (workspace-relative)
 * - moveTo(pulses_for_150mm): Moves to 100mm from home, getTargetX() returns 150mm (workspace-relative)
 * - At home (0mm from home): getCurrentX() returns 50mm (workspace origin)
 */
class Gantry {
public:
  /**
   * @brief Motion completion callback type
   */
  using MotionCompleteCallback = void (*)(bool success);
  
  /**
   * @brief Construct a new Gantry object
   *
   * @param xConfig Configuration for the X-axis Servo Driver
   * @param gripperPin GPIO pin for the gripper
   */
  Gantry(const BergerdaServo::DriverConfig &xConfig, int gripperPin);

  /**
   * @brief Initialize the Gantry system
   * @return GantryError code
   */
  GantryError begin();

  /**
   * @brief Enable all axes
   * @return GantryError code
   */
  GantryError enable();

  /**
   * @brief Disable all axes
   * @return GantryError code
   */
  GantryError disable();

  /**
   * @brief Abort all motion without disabling motor
   * @return GantryError code
   * @note Also aborts homing and calibration operations if in progress
   *       Uses lightweight boolean flag checked in operation loops
   */
  GantryError abortMotion();

  /**
   * @brief Move Gantry to target coordinates (X in pulses)
   *
   * @param x_pulses Target X position in pulses relative to HOME position (0 pulses = home)
   * @param y_stub Target Y position in mm (vertical axis, 0-150mm range, simulated)
   * @param theta_stub Target Theta angle in degrees (rotary axis, -90 to +90°, 0° = -Z axis, simulated)
   * @param speed Motion speed in pps (pulses per second, for X axis only)
   * @param acceleration Acceleration rate in pps² (0 = use default)
   * @param deceleration Deceleration rate in pps² (0 = use default)
   * @return GantryError code
   * @note Y and Theta axes are simulated with trapezoidal velocity profiles (similar to X-axis)
   * @note Coordinates: x_pulses is in HOME coordinate space (not workspace coordinates)
   *       Use moveToMm() for workspace coordinate space (mm relative to workspace origin)
   */
  GantryError moveTo(int32_t x_pulses, int32_t y_stub, int32_t theta_stub,
                     uint32_t speed = 5000, uint32_t acceleration = 0, uint32_t deceleration = 0);

  /**
   * @brief Move Gantry to target coordinates (X in mm)
   *
   * @param x_mm Target X position in mm relative to WORKSPACE ORIGIN (not home position)
   * @param y_mm Target Y position in mm (vertical axis, 0-150mm range, simulated)
   * @param theta_deg Target Theta angle in degrees (rotary axis, -90 to +90°, 0° = -Z axis, simulated)
   * @param speed_mm_per_s Motion speed in mm/s (converted to pps internally)
   * @param acceleration_mm_per_s2 Acceleration in mm/s² (0 = use default)
   * @param deceleration_mm_per_s2 Deceleration in mm/s² (0 = use default)
   * @return GantryError code
   * @note Coordinates: x_mm is in WORKSPACE coordinate space (mm relative to workspace origin)
   *       Workspace origin = home position + workspace_origin_offset_mm_
   *       Example: If offset=50mm, x_mm=0 means 50mm from home, x_mm=100 means 150mm from home
   */
  GantryError moveToMm(int32_t x_mm, int32_t y_mm, int32_t theta_deg,
                       uint32_t speed_mm_per_s = 200, uint32_t acceleration_mm_per_s2 = 0, uint32_t deceleration_mm_per_s2 = 0);

  /**
   * @brief Move Gantry by relative distance
   *
   * @param delta_x_mm Relative X movement in mm
   * @param delta_y_mm Relative Y movement in mm
   * @param delta_theta_deg Relative Theta rotation in degrees
   * @param speed_mm_per_s Motion speed in mm/s
   * @param acceleration_mm_per_s2 Acceleration in mm/s² (0 = use default)
   * @param deceleration_mm_per_s2 Deceleration in mm/s² (0 = use default)
   * @return GantryError code
   */
  GantryError moveRelative(int32_t delta_x_mm, int32_t delta_y_mm, int32_t delta_theta_deg,
                           uint32_t speed_mm_per_s = 200, uint32_t acceleration_mm_per_s2 = 0, uint32_t deceleration_mm_per_s2 = 0);

  /**
   * @brief Control the gripper
   *
   * @param active true to Close/Active, false to Open/Release
   * @return GantryError code (OK on success, NOT_INITIALIZED if not initialized)
   */
  GantryError grip(bool active);

  /**
   * @brief Check if Gantry is currently busy (moving, homing, or calibrating)
   * @return true if busy (any axis moving, or homing/calibration in progress)
   * @note This method is not thread-safe if called from multiple tasks
   * @note Returns true if any axis is moving, or if homing/calibration operations are in progress
   */
  bool isBusy() const;

  /**
   * @brief Wait for motion to complete
   * @param timeout_ms Timeout in milliseconds (0 = no timeout)
   * @return GantryError code (OK if completed, TIMEOUT if timeout)
   */
  GantryError waitForMotionComplete(uint32_t timeout_ms = 0);

  /**
   * @brief Main update loop - call frequently
   */
  void update();

  // Homing & Calibration
  void setLimitPins(int xMinPin, int xMaxPin);
  GantryError home();
  GantryError calibrate(bool returnToHome = false);
  
  /**
   * @brief Get homing status
   * @return HomingStatus
   */
  HomingStatus getHomingStatus() const { return homing_status_; }
  
  /**
   * @brief Check if homing operation is currently in progress
   * @return true if homing is in progress, false otherwise
   */
  bool isHoming() const { return homing_status_ == HomingStatus::IN_PROGRESS; }
  
  /**
   * @brief Check if calibration operation is currently in progress
   * @return true if calibration is in progress, false otherwise
   */
  bool isCalibrating() const { return calibrating_; }

  // Accessors - Current Positions
  /**
   * @brief Get current X position in workspace coordinates
   * @return Current X position in mm relative to workspace origin (not home position)
   * @note Workspace coordinates: workspace origin = home + workspace_origin_offset_mm_
   */
  int32_t getCurrentX() const;
  int32_t getCurrentY() const { return currentY_; }  // Y position in mm (0-150mm, simulated)
  int32_t getCurrentTheta() const { return currentTheta_; }  // Theta angle in degrees (-90 to +90°, simulated)
  
  // Accessors - Target Positions
  /**
   * @brief Get target X position in workspace coordinates
   * @return Target X position in mm relative to workspace origin (not home position)
   * @note Workspace coordinates: workspace origin = home + workspace_origin_offset_mm_
   */
  int32_t getTargetX() const;
  int32_t getTargetY() const { return targetY_; }  // Target Y position in mm
  int32_t getTargetTheta() const { return targetTheta_; }  // Target Theta angle in degrees
  
  // Accessors - Remaining Distance
  int32_t getRemainingDistanceX() const;  // Remaining X distance to target in mm
  int32_t getRemainingDistanceY() const { 
    int32_t diff = targetY_ - currentY_; 
    return (diff < 0) ? -diff : diff; 
  }  // Remaining Y distance in mm
  int32_t getRemainingDistanceTheta() const { 
    int32_t diff = targetTheta_ - currentTheta_; 
    return (diff < 0) ? -diff : diff; 
  }  // Remaining Theta in degrees
  
  // Accessors - Position Error (for control loops)
  /**
   * @brief Get position error in pulses (signed: target - current)
   * @return Position error in pulses (positive = behind target, negative = ahead of target)
   * @note Fast method optimized for high-frequency control loops (5kHz+)
   *       Returns error in home coordinates (pulses from home)
   */
  int32_t getPositionErrorX() const { 
    if (!initialized_) return 0;
    return targetX_ - currentX_; 
  }
  
  /**
   * @brief Get position error in mm (signed: target - current)
   * @return Position error in mm in workspace coordinates
   * @note Slower than getPositionErrorX() due to coordinate conversion
   */
  int32_t getPositionErrorXmm() const;
  
  // Accessors - System State
  int32_t getXEncoder() const { return axisX_.getEncoderPosition(); }  // X encoder feedback (pulses)
  int32_t getAxisLength() const;  // X-axis length in mm (after calibration)
  int32_t getAxisLengthPulses() const { return axisLength_; }  // X-axis length in pulses
  bool isGripperActive() const { return gripperActive_; }  // Gripper state (true = closed, false = open)
  bool isEnabled() const { return axisX_.isEnabled(); }  // Motor enabled state

  // Status
  GantryStatus getStatus() const;

  // Coordinate Conversion Helpers (Static)
  static int32_t mmToPulses(int32_t mm, uint32_t encoder_ppr);
  static int32_t pulsesToMm(int32_t pulses, uint32_t encoder_ppr);
  
  // Coordinate Conversion Helpers (Instance)
  int32_t mmToPulses(int32_t mm) const;
  int32_t pulsesToMm(int32_t pulses) const;
  
  // Kinematics (Forward and Inverse)
  /**
   * @brief Forward kinematics: Calculate end-effector X position from joint positions
   * @param x_axis_mm X-axis position in mm (from home)
   * @param theta_deg Theta angle in degrees (rotation around Y axis)
   * @return End-effector X position in mm (from home)
   * @note Theta rotation around Y axis doesn't change X position
   */
  int32_t forwardKinematicsX(int32_t x_axis_mm, int32_t theta_deg) const;
  
  /**
   * @brief Inverse kinematics: Calculate required X-axis position from end-effector position
   * @param end_effector_x_mm Desired end-effector X position in mm (from home)
   * @param theta_deg Theta angle in degrees (rotation around Y axis)
   * @return Required X-axis position in mm (from home)
   * @note Theta rotation around Y axis doesn't affect X position calculation
   */
  int32_t inverseKinematicsX(int32_t end_effector_x_mm, int32_t theta_deg) const;

  // Configuration
  void setHomingTimeout(uint32_t timeout_ms) { homing_timeout_ms_ = timeout_ms; }
  /**
   * @brief Set workspace origin offset
   * @param offset_mm Offset in mm from home position to workspace origin
   * @note Workspace origin = home position + offset_mm
   *       This allows the coordinate system to have its origin at a different location than home.
   *       All mm-based APIs (moveToMm(), getCurrentX(), getTargetX()) use workspace coordinates.
   *       Example: If offset=50mm, workspace origin is 50mm from home. moveToMm(0) moves to home.
   */
  void setWorkspaceOriginOffset(int32_t offset_mm) { workspace_origin_offset_mm_ = offset_mm; }
  int32_t getWorkspaceOriginOffset() const { return workspace_origin_offset_mm_; }
  void setSoftLimits(int32_t min_mm, int32_t max_mm);
  void getSoftLimits(int32_t &min_mm, int32_t &max_mm) const;
  void setDefaultAcceleration(uint32_t acceleration_pps2) { default_acceleration_ = acceleration_pps2; }
  void setDefaultDeceleration(uint32_t deceleration_pps2) { default_deceleration_ = deceleration_pps2; }
  uint32_t getDefaultAcceleration() const { return default_acceleration_; }
  uint32_t getDefaultDeceleration() const { return default_deceleration_; }
  
  /**
   * @brief Set backlash compensation parameters
   * @param enabled Enable/disable backlash compensation
   * @param pulses Backlash compensation in pulses (default: 60)
   * @note Backlash compensation adds extra movement when direction changes to account for mechanical play
   */
  void setBacklashCompensation(bool enabled, uint32_t pulses = 60) {
    backlash_compensation_enabled_ = enabled;
    backlash_compensation_pulses_ = pulses;
  }
  
  /**
   * @brief Get backlash compensation enabled state
   * @return true if enabled
   */
  bool isBacklashCompensationEnabled() const { return backlash_compensation_enabled_; }
  
  /**
   * @brief Get backlash compensation value
   * @return Backlash compensation in pulses
   */
  uint32_t getBacklashCompensationPulses() const { return backlash_compensation_pulses_; }
  
  /**
   * @brief Reset backlash direction tracking
   * @note This resets the last move direction used for backlash compensation detection.
   *       Useful when backlash tracking state needs to be reset without homing.
   *       Backlash tracking is also automatically reset after homing.
   */
  void resetBacklashTracking() { last_move_direction_ = 0; }
  
  // Callbacks
  void setMotionCompleteCallback(MotionCompleteCallback callback) { motion_complete_callback_ = callback; }

private:
  // Constants
  static constexpr int32_t HOMING_BACKOFF_PULSES = 2000;
  static constexpr int32_t HOMING_SMALL_BACKOFF_PULSES = 1000;
  static constexpr int32_t HOMING_LONG_MOVE_PULSES = 1000000;
  static constexpr uint32_t HOMING_BACKOFF_SPEED = 1000;
  static constexpr uint32_t HOMING_SPEED = 2000;
  static constexpr uint32_t HOMING_DEFAULT_TIMEOUT_MS = 60000; // 60 seconds
  static constexpr uint32_t DEFAULT_ACCELERATION = 3000;
  static constexpr uint32_t DEFAULT_DECELERATION = 3000;
  // Maximum safe limits (pps and pps²) - conservative values for safety
  static constexpr uint32_t MAX_SPEED_PPS = 50000;           // 50k pps max speed
  static constexpr uint32_t MAX_ACCELERATION_PPS2 = 20000;   // 20k pps² max acceleration
  static constexpr uint32_t MAX_DECELERATION_PPS2 = 20000;   // 20k pps² max deceleration
  
  // Default stub motion parameters (realistic simulation)
  static constexpr float DEFAULT_Y_SPEED_MM_PER_S = 50.0f;      // 50 mm/s
  static constexpr float DEFAULT_Y_ACCEL_MM_PER_S2 = 100.0f;    // 100 mm/s²
  static constexpr float DEFAULT_Y_DECEL_MM_PER_S2 = 100.0f;    // 100 mm/s²
  static constexpr float DEFAULT_THETA_SPEED_DEG_PER_S = 30.0f;  // 30 deg/s
  static constexpr float DEFAULT_THETA_ACCEL_DEG_PER_S2 = 60.0f; // 60 deg/s²
  static constexpr float DEFAULT_THETA_DECEL_DEG_PER_S2 = 60.0f; // 60 deg/s²
  
  // Mechanical limits (for validation/documentation)
  static constexpr int32_t Y_AXIS_STROKE_LENGTH_MM = 150;
  static constexpr int32_t THETA_MIN_DEGREES = -90;
  static constexpr int32_t THETA_MAX_DEGREES = 90;
  
  // Mechanical offsets and dimensions
  static constexpr int32_t Y_AXIS_Z_OFFSET_MM = 80;           // Y axis Z offset from X
  static constexpr int32_t THETA_X_OFFSET_MM = -55;           // Theta X offset from Y vertical
  static constexpr int32_t GRIPPER_Y_OFFSET_MM = 385;         // Gripper Y offset from X origin
  static constexpr int32_t GRIPPER_Z_OFFSET_MM = 80;          // Gripper Z offset from X origin
  static constexpr int32_t X_AXIS_BALL_SCREW_PITCH_MM = 40;   // X-axis ball-screw pitch (mm per revolution)

  BergerdaServo::ServoDriver axisX_;
  int gripperPin_;
  int xMinPin_;
  int xMaxPin_;
  int32_t axisLength_;
  bool initialized_;
  bool gripperActive_;
  HomingStatus homing_status_;
  bool calibrating_;  // Track calibration in progress state

  // Homing configuration
  uint32_t homing_timeout_ms_;
  
  // Motion configuration
  uint32_t default_acceleration_;
  uint32_t default_deceleration_;
  
  // Workspace configuration
  /**
   * @brief Workspace origin offset in mm from home position
   * 
   * Coordinate System Transformation:
   * - Workspace origin = home position + workspace_origin_offset_mm_
   * - User-facing APIs (moveToMm, getCurrentX, getTargetX) use workspace coordinates
   * - Internal storage (currentX_, targetX_) uses home coordinates (pulses from home)
   * - Transformation: workspace_mm ↔ (home_mm + offset_mm) ↔ home_pulses
   * 
   * Example: If offset=50mm:
   * - At home (0mm from home): getCurrentX() returns 50mm (workspace origin)
   * - moveToMm(100mm): Moves to 50mm from home, getTargetX() returns 100mm (workspace-relative)
   */
  int32_t workspace_origin_offset_mm_;
  int32_t soft_limit_min_mm_;
  int32_t soft_limit_max_mm_;
  bool soft_limits_enabled_;
  
  // Backlash compensation (on-demand feature)
  bool backlash_compensation_enabled_;
  uint32_t backlash_compensation_pulses_;
  int8_t last_move_direction_;  // -1 = negative, 0 = unknown, +1 = positive

  // Position tracking
  int32_t currentX_;  // Current X position in pulses
  
  // Stubs
  int32_t currentY_;
  int32_t targetY_;
  int32_t currentTheta_;
  int32_t targetTheta_;
  int32_t targetX_;  // Target X position in pulses

  bool y_moving_;
  bool theta_moving_;
  uint32_t stub_move_start_ms_;
  
  // Stub motion profiles (trapezoidal velocity, similar to X-axis)
  struct StubProfile {
    int32_t start_pos;      // Start position (mm for Y, degrees for Theta)
    int32_t target_pos;     // Target position
    float max_speed;        // Max speed (mm/s for Y, deg/s for Theta)
    float accel_rate;       // Acceleration (mm/s² for Y, deg/s² for Theta)
    float decel_rate;       // Deceleration
    float t_accel;          // Time to accelerate (seconds)
    float t_cruise;         // Time at constant speed (seconds)
    float t_decel;          // Time to decelerate (seconds)
    float total_time;       // Total movement time (seconds)
  };
  StubProfile y_profile_;
  StubProfile theta_profile_;
  bool x_was_moving_prev_;  // Previous X motion state for completion detection
  bool abort_requested_;    // Lightweight abort flag for homing/calibration
  
  // Callbacks
  MotionCompleteCallback motion_complete_callback_;

  // Helper methods
  void log(const char *msg);
  GantryError waitForAxisMotionComplete(uint32_t timeout_ms);
  void notifyMotionComplete(bool success);
  
  // Stub profile helpers (trapezoidal velocity, similar to X-axis)
  void calculateStubProfile(StubProfile &profile, int32_t start_pos, int32_t target_pos,
                            float max_speed, float accel_rate, float decel_rate);
  int32_t interpolateStubPosition(const StubProfile &profile, float elapsed_s);
};

#endif // GANTRY_H

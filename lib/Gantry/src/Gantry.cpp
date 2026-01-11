#include "Gantry.h"
#include <cmath>
#include <cstdlib>  // For abs()

// ============================================================================
// STATIC COORDINATE CONVERSION HELPERS
// ============================================================================

int32_t Gantry::mmToPulses(int32_t mm, uint32_t encoder_ppr) {
  if (encoder_ppr == 0) {
    return 0;
  }
  // Formula: pulses = (mm * encoder_ppr) / pitch_mm
  int64_t mm_64 = (int64_t)mm;
  int64_t encoder_ppr_64 = (int64_t)encoder_ppr;
  int64_t pitch_64 = (int64_t)X_AXIS_BALL_SCREW_PITCH_MM;
  // Round to nearest: (mm * encoder_ppr * 2 + pitch) / (pitch * 2)
  int64_t pulses_scaled = (mm_64 * encoder_ppr_64 * 2 + pitch_64) / (pitch_64 * 2);
  
  // Check for overflow before casting to int32_t
  if (pulses_scaled > INT32_MAX) {
    return INT32_MAX;
  } else if (pulses_scaled < INT32_MIN) {
    return INT32_MIN;
  }
  return (int32_t)pulses_scaled;
}

int32_t Gantry::pulsesToMm(int32_t pulses, uint32_t encoder_ppr) {
  if (encoder_ppr == 0) {
    return 0;
  }
  // Formula: mm = (pulses * pitch_mm) / encoder_ppr
  int64_t pulses_64 = (int64_t)pulses;
  int64_t pitch_64 = (int64_t)X_AXIS_BALL_SCREW_PITCH_MM;
  int64_t encoder_ppr_64 = (int64_t)encoder_ppr;
  // Round to nearest: (pulses * pitch * 2 + encoder_ppr) / (encoder_ppr * 2)
  int64_t mm_scaled = (pulses_64 * pitch_64 * 2 + encoder_ppr_64) / (encoder_ppr_64 * 2);
  
  // Check for overflow before casting to int32_t
  if (mm_scaled > INT32_MAX) {
    return INT32_MAX;
  } else if (mm_scaled < INT32_MIN) {
    return INT32_MIN;
  }
  return (int32_t)mm_scaled;
}

// ============================================================================
// JOINT CONFIG IMPLEMENTATION
// ============================================================================

bool JointConfig::isValid() const {
  return (q_x >= X_MIN && q_x <= X_MAX_DEFAULT) &&
         (q_y >= Y_MIN && q_y <= Y_MAX) &&
         (q_theta >= THETA_MIN && q_theta <= THETA_MAX);
}

void JointConfig::getLimits(float& x_min, float& x_max,
                            float& y_min, float& y_max,
                            float& theta_min, float& theta_max) {
  x_min = X_MIN;
  x_max = X_MAX_DEFAULT;
  y_min = Y_MIN;
  y_max = Y_MAX;
  theta_min = THETA_MIN;
  theta_max = THETA_MAX;
}

// ============================================================================
// CONSTRUCTOR
// ============================================================================

Gantry::Gantry(const BergerdaServo::DriverConfig &xConfig, int gripperPin)
    : axisX_(xConfig), gripperPin_(gripperPin), xMinPin_(-1), xMaxPin_(-1),
      axisLength_(0), initialized_(false), gripperActive_(false),
      homing_status_(HomingStatus::IDLE), calibrating_(false),
      homing_timeout_ms_(HOMING_DEFAULT_TIMEOUT_MS),
      default_acceleration_(DEFAULT_ACCELERATION),
      default_deceleration_(DEFAULT_DECELERATION),
      workspace_origin_offset_mm_(0),
      soft_limit_min_mm_(0),
      soft_limit_max_mm_(0),
      soft_limits_enabled_(false),
      backlash_compensation_enabled_(false),
      backlash_compensation_pulses_(60),
      last_move_direction_(0),
      currentX_(0), currentY_(0), targetY_(0), currentTheta_(0),
      targetTheta_(0), targetX_(0), y_moving_(false), theta_moving_(false),
      stub_move_start_ms_(0), x_was_moving_prev_(false), abort_requested_(false),
      motion_complete_callback_(nullptr) {
  // Initialize stub profiles
  y_profile_ = {};
  theta_profile_ = {};
}

// ============================================================================
// HELPER METHODS
// ============================================================================

void Gantry::log(const char *msg) {
  // Use Serial if available, otherwise do nothing
  // This allows for future logging abstraction
  if (Serial) {
    Serial.println(msg);
  }
}

GantryError Gantry::waitForAxisMotionComplete(uint32_t timeout_ms) {
  uint32_t start_time = millis();
  
  while (axisX_.isMotionActive()) {
    // Check for abort request (lightweight check)
    if (abort_requested_) {
      return GantryError::OK;  // Return OK since abort was requested
    }
    
    if (timeout_ms > 0) {
      // Unsigned subtraction correctly handles millis() overflow (wraparound after ~49 days)
      // If millis() wraps: (new_small - old_large) = large_positive, which is correct
      uint32_t elapsed = millis() - start_time;
      if (elapsed > timeout_ms) {
        return GantryError::TIMEOUT;
      }
    }
    
#if defined(ARDUINO_ARCH_ESP32)
    vTaskDelay(pdMS_TO_TICKS(10));
#else
    delay(10);
#endif
  }
  
  return GantryError::OK;
}

void Gantry::notifyMotionComplete(bool success) {
  if (motion_complete_callback_) {
    motion_complete_callback_(success);
  }
}

// ============================================================================
// INITIALIZATION
// ============================================================================

GantryError Gantry::begin() {
  if (initialized_) {
    log("[Gantry] Already initialized");
    return GantryError::OK;  // Already initialized is not an error
  }

  // Validate gripper pin
  if (gripperPin_ < 0) {
    log("[Gantry] ERROR: Invalid gripper pin");
    return GantryError::INVALID_PARAMETER;
  }

  pinMode(gripperPin_, OUTPUT);
  digitalWrite(gripperPin_, LOW);
  gripperActive_ = false;

  // Configure limit switch pins if set
  if (xMinPin_ >= 0)
    pinMode(xMinPin_, INPUT_PULLUP);
  if (xMaxPin_ >= 0)
    pinMode(xMaxPin_, INPUT_PULLUP);

  if (!axisX_.initialize()) {
    log("[Gantry] ERROR: X-axis initialization failed");
    return GantryError::NOT_INITIALIZED;
  }

  initialized_ = true;
  log("[Gantry] Initialized successfully");
  return GantryError::OK;
}

// ============================================================================
// ENABLE/DISABLE/ABORT
// ============================================================================

GantryError Gantry::enable() {
  if (!initialized_) {
    log("[Gantry] ERROR: Not initialized. Call begin() first.");
    return GantryError::NOT_INITIALIZED;
  }
  if (!axisX_.enable()) {
    return GantryError::MOTOR_NOT_ENABLED;
  }
  // Note: Y/Theta stubs are simulated, no actual enable needed
  return GantryError::OK;
}

GantryError Gantry::disable() {
  if (!initialized_) {
    return GantryError::NOT_INITIALIZED;
  }
  // Stop any ongoing motion first
  if (isBusy()) {
    axisX_.stopMotion(0);
    y_moving_ = false;
    theta_moving_ = false;
  }
  axisX_.disable();
  return GantryError::OK;
}

GantryError Gantry::abortMotion() {
  if (!initialized_) {
    return GantryError::NOT_INITIALIZED;
  }
  
  // Set abort flag (checked by homing/calibration loops and waitForAxisMotionComplete)
  abort_requested_ = true;
  
  // Stop X-axis motion
  axisX_.stopMotion(0);
  
  // Stop stub movements
  y_moving_ = false;
  theta_moving_ = false;
  
  // Clear homing/calibration state if operations are in progress
  if (homing_status_ == HomingStatus::IN_PROGRESS) {
    homing_status_ = HomingStatus::FAILED;
  }
  if (calibrating_) {
    calibrating_ = false;
  }
  
  // Note: abort_requested_ flag is NOT reset here
  // It will be reset by homing/calibration operations when they detect the abort
  // This ensures the flag remains set for synchronous checking in operation loops
  
  notifyMotionComplete(false);  // Notify that motion was aborted
  
  return GantryError::OK;
}

// ============================================================================
// COORDINATE CONVERSION (INSTANCE METHODS)
// ============================================================================

int32_t Gantry::mmToPulses(int32_t mm) const {
  if (!initialized_) {
    return 0;
  }
  const BergerdaServo::DriverConfig &config = axisX_.getConfig();
  // Validate encoder_ppr before conversion (defensive programming)
  if (config.encoder_ppr == 0) {
    // Static method also checks for 0, but validate here for early detection
    return 0;
  }
  return mmToPulses(mm, config.encoder_ppr);
}

int32_t Gantry::pulsesToMm(int32_t pulses) const {
  if (!initialized_) {
    return 0;
  }
  const BergerdaServo::DriverConfig &config = axisX_.getConfig();
  // Validate encoder_ppr before conversion (defensive programming)
  if (config.encoder_ppr == 0) {
    // Static method also checks for 0, but validate here for early detection
    return 0;
  }
  return pulsesToMm(pulses, config.encoder_ppr);
}

// ============================================================================
// MOTION CONTROL
// ============================================================================

GantryError Gantry::moveTo(int32_t x_pulses, int32_t y_stub, int32_t theta_stub,
                           uint32_t speed, uint32_t acceleration, uint32_t deceleration) {
  if (!initialized_) {
    log("[Gantry] ERROR: Not initialized. Call begin() first.");
    return GantryError::NOT_INITIALIZED;
  }

  if (!axisX_.isEnabled()) {
    log("[Gantry] ERROR: Motor not enabled. Call enable() first.");
    return GantryError::MOTOR_NOT_ENABLED;
  }

  // Validate x_pulses is non-negative (moveToPosition expects uint32_t)
  if (x_pulses < 0) {
    log("[Gantry] ERROR: Negative X position not supported");
    return GantryError::INVALID_POSITION;
  }

  // Check soft limits if enabled (check in workspace coordinate space - mm relative to workspace origin)
  // x_pulses is in home coordinates, convert to workspace coordinates for limit checking
  if (soft_limits_enabled_) {
    int32_t x_mm_home = pulsesToMm(x_pulses);  // Convert to mm from home
    int32_t x_mm_workspace = x_mm_home - workspace_origin_offset_mm_;  // Convert to workspace coordinates
    if (x_mm_workspace < soft_limit_min_mm_ || x_mm_workspace > soft_limit_max_mm_) {
      log("[Gantry] ERROR: X position exceeds soft limits");
      return GantryError::INVALID_POSITION;
    }
  }
  
  // x_pulses is already in home coordinates (pulses from home position)
  // Internal storage (targetX_) stores home coordinates
  // Check bounds if axisLength_ is set (after calibration)
  if (axisLength_ > 0) {
    if (x_pulses < 0 || (uint32_t)x_pulses > (uint32_t)axisLength_) {
      log("[Gantry] ERROR: X position exceeds axis length");
      return GantryError::INVALID_POSITION;
    }
  }

  // Validate speed parameter
  if (speed == 0) {
    log("[Gantry] ERROR: Speed must be greater than 0");
    return GantryError::INVALID_PARAMETER;
  }
  if (speed > MAX_SPEED_PPS) {
    log("[Gantry] ERROR: Speed exceeds maximum limit");
    return GantryError::INVALID_PARAMETER;
  }

  // Check if already moving
  if (isBusy()) {
    log("[Gantry] ERROR: Already moving. Wait for completion or stop motion first.");
    return GantryError::ALREADY_MOVING;
  }

  // Use defaults if acceleration/deceleration not specified
  uint32_t accel = (acceleration == 0) ? default_acceleration_ : acceleration;
  uint32_t decel = (deceleration == 0) ? default_deceleration_ : deceleration;
  
  // Validate acceleration/deceleration limits
  if (accel > MAX_ACCELERATION_PPS2) {
    log("[Gantry] ERROR: Acceleration exceeds maximum limit");
    return GantryError::INVALID_PARAMETER;
  }
  if (decel > MAX_DECELERATION_PPS2) {
    log("[Gantry] ERROR: Deceleration exceeds maximum limit");
    return GantryError::INVALID_PARAMETER;
  }

  // Apply backlash compensation if enabled (on-demand feature)
  int32_t compensated_target = x_pulses;  // x_pulses is in home coordinates
  if (backlash_compensation_enabled_ && backlash_compensation_pulses_ > 0) {
    // Determine current movement direction
    int32_t position_diff = x_pulses - currentX_;  // Compare in home coordinates
    int8_t current_direction = (position_diff > 0) ? 1 : ((position_diff < 0) ? -1 : 0);
    
    // Apply compensation if direction changed (and not first move)
    if (current_direction != 0 && last_move_direction_ != 0 && current_direction != last_move_direction_) {
      // Direction changed - add compensation pulses in new direction (overshoot to take up backlash)
      if (current_direction > 0) {
        compensated_target += (int32_t)backlash_compensation_pulses_;
      } else {
        compensated_target -= (int32_t)backlash_compensation_pulses_;
      }
      
      // Ensure compensated target doesn't exceed bounds
      if (axisLength_ > 0 && compensated_target > (int32_t)axisLength_) {
        compensated_target = axisLength_;  // Clamp to max
      }
      if (compensated_target < 0) {
        compensated_target = 0;  // Clamp to min
      }
    }
    
    // Update last direction for next move
    if (current_direction != 0) {
      last_move_direction_ = current_direction;
    }
  }

  // Validate compensated target is non-negative before cast
  if (compensated_target < 0) {
    log("[Gantry] ERROR: Compensated target position is negative");
    return GantryError::INVALID_POSITION;
  }

  // 1. Move X Axis
  if (!axisX_.moveToPosition((uint32_t)compensated_target, speed, accel, decel)) {
    log("[Gantry] ERROR: X-axis move command failed");
    return GantryError::INVALID_PARAMETER;
  }

  // Update target positions (store compensated target when compensation is applied)
  targetX_ = compensated_target;

  // 2. Setup Y Stub (vertical axis, simulated)
  // Validate Y range: 0 to Y_AXIS_STROKE_LENGTH_MM (150mm)
  if (y_stub < 0 || y_stub > Y_AXIS_STROKE_LENGTH_MM) {
    log("[Gantry] ERROR: Y position out of range (0-150mm)");
    return GantryError::INVALID_POSITION;
  }
  
  // 2. Setup Y Stub with trapezoidal velocity profile (similar to X-axis)
  if (y_stub != currentY_) {
    targetY_ = y_stub;
    y_moving_ = true;
    // Calculate trapezoidal profile (similar to X-axis servo driver)
    calculateStubProfile(y_profile_, currentY_, targetY_,
                        DEFAULT_Y_SPEED_MM_PER_S,
                        DEFAULT_Y_ACCEL_MM_PER_S2,
                        DEFAULT_Y_DECEL_MM_PER_S2);
  } else {
    y_moving_ = false;
  }

  // 3. Setup Theta Stub with trapezoidal velocity profile (similar to X-axis)
  // Validate Theta range: -90° to +90° (0° = parallel to -Z axis)
  if (theta_stub < THETA_MIN_DEGREES || theta_stub > THETA_MAX_DEGREES) {
    log("[Gantry] ERROR: Theta angle out of range (-90 to +90 degrees)");
    return GantryError::INVALID_POSITION;
  }
  
  if (theta_stub != currentTheta_) {
    targetTheta_ = theta_stub;
    theta_moving_ = true;
    // Calculate trapezoidal profile (similar to X-axis servo driver)
    calculateStubProfile(theta_profile_, currentTheta_, targetTheta_,
                        DEFAULT_THETA_SPEED_DEG_PER_S,
                        DEFAULT_THETA_ACCEL_DEG_PER_S2,
                        DEFAULT_THETA_DECEL_DEG_PER_S2);
  } else {
    theta_moving_ = false;
  }

  if (y_moving_ || theta_moving_) {
    stub_move_start_ms_ = millis();
  }

  return GantryError::OK;
}

GantryError Gantry::moveToMm(int32_t x_mm, int32_t y_mm, int32_t theta_deg,
                             uint32_t speed_mm_per_s, uint32_t acceleration_mm_per_s2, uint32_t deceleration_mm_per_s2) {
  // Validate soft limits directly in workspace coordinates (simpler and clearer)
  if (soft_limits_enabled_) {
    if (x_mm < soft_limit_min_mm_ || x_mm > soft_limit_max_mm_) {
      log("[Gantry] ERROR: X position exceeds soft limits");
      return GantryError::INVALID_POSITION;
    }
  }
  
  // Convert workspace coordinates (mm from workspace origin) to home coordinates
  // x_mm is desired END-EFFECTOR position in workspace coordinate space
  // Workspace origin = home position + workspace_origin_offset_mm_
  // Therefore: end_effector_home_mm = x_mm + workspace_origin_offset_mm_
  int32_t end_effector_home_mm = x_mm + workspace_origin_offset_mm_;  // Convert workspace to home coordinates
  
  // Apply inverse kinematics: Calculate required X-axis position from end-effector position
  int32_t x_axis_home_mm = inverseKinematicsX(end_effector_home_mm, theta_deg);
  
  // Convert to pulses
  int32_t x_pulses = mmToPulses(x_axis_home_mm);
  
  // Convert mm/s to pps for X axis
  // Formula: speed_pps = (speed_mm_per_s * encoder_ppr) / pitch_mm
  const BergerdaServo::DriverConfig &config = axisX_.getConfig();
  uint32_t speed_pps = 0;
  if (config.encoder_ppr > 0 && X_AXIS_BALL_SCREW_PITCH_MM > 0) {
    speed_pps = (uint32_t)((uint64_t)speed_mm_per_s * config.encoder_ppr / X_AXIS_BALL_SCREW_PITCH_MM);
    // Validate conversion result - if speed_mm_per_s > 0 but conversion results in 0,
    // the speed is too low and would result in incorrect motion
    if (speed_pps == 0 && speed_mm_per_s > 0) {
      log("[Gantry] ERROR: Speed too low - conversion to pps resulted in zero");
      return GantryError::INVALID_PARAMETER;
    }
  } else if (speed_mm_per_s > 0) {
    log("[Gantry] ERROR: Invalid encoder configuration for speed conversion");
    return GantryError::INVALID_PARAMETER;
  }
  
  // Convert acceleration/deceleration
  uint32_t accel_pps2 = 0;
  uint32_t decel_pps2 = 0;
  if (acceleration_mm_per_s2 > 0) {
    // Approximate conversion: pps² ≈ (mm/s² * encoder_ppr / pitch)
    const BergerdaServo::DriverConfig &config = axisX_.getConfig();
    if (config.encoder_ppr > 0) {
      accel_pps2 = (uint32_t)((uint64_t)acceleration_mm_per_s2 * config.encoder_ppr / X_AXIS_BALL_SCREW_PITCH_MM);
    }
  }
  if (deceleration_mm_per_s2 > 0) {
    const BergerdaServo::DriverConfig &config = axisX_.getConfig();
    if (config.encoder_ppr > 0) {
      decel_pps2 = (uint32_t)((uint64_t)deceleration_mm_per_s2 * config.encoder_ppr / X_AXIS_BALL_SCREW_PITCH_MM);
    }
  }
  
  return moveTo(x_pulses, y_mm, theta_deg, speed_pps, accel_pps2, decel_pps2);
}

GantryError Gantry::moveRelative(int32_t delta_x_mm, int32_t delta_y_mm, int32_t delta_theta_deg,
                                 uint32_t speed_mm_per_s, uint32_t acceleration_mm_per_s2, uint32_t deceleration_mm_per_s2) {
  // Calculate target positions
  int32_t target_x_mm = getCurrentX() + delta_x_mm;
  int32_t target_y_mm = getCurrentY() + delta_y_mm;
  int32_t target_theta_deg = getCurrentTheta() + delta_theta_deg;
  
  // Validate Y range: 0 to Y_AXIS_STROKE_LENGTH_MM (150mm)
  if (target_y_mm < 0 || target_y_mm > Y_AXIS_STROKE_LENGTH_MM) {
    log("[Gantry] ERROR: Target Y position out of range (0-150mm)");
    return GantryError::INVALID_POSITION;
  }
  
  // Validate Theta range: -90° to +90°
  if (target_theta_deg < THETA_MIN_DEGREES || target_theta_deg > THETA_MAX_DEGREES) {
    log("[Gantry] ERROR: Target Theta angle out of range (-90 to +90 degrees)");
    return GantryError::INVALID_POSITION;
  }
  
  // Note: X position validation will be done in moveToMm()
  
  return moveToMm(target_x_mm, target_y_mm, target_theta_deg, speed_mm_per_s, acceleration_mm_per_s2, deceleration_mm_per_s2);
}

// ============================================================================
// GRIPPER CONTROL
// ============================================================================

GantryError Gantry::grip(bool active) {
  if (!initialized_) {
    return GantryError::NOT_INITIALIZED;
  }
  digitalWrite(gripperPin_, active ? HIGH : LOW);
  gripperActive_ = active;
  return GantryError::OK;
}

// ============================================================================
// STATUS AND MONITORING
// ============================================================================

bool Gantry::isBusy() const {
  if (!initialized_) {
    return false;
  }
  // Busy if X is moving OR Stubs are "moving" OR homing/calibration in progress
  return axisX_.isMotionActive() || y_moving_ || theta_moving_ || 
         (homing_status_ == HomingStatus::IN_PROGRESS) || calibrating_;
}

GantryError Gantry::waitForMotionComplete(uint32_t timeout_ms) {
  if (!initialized_) {
    return GantryError::NOT_INITIALIZED;
  }
  
  uint32_t start_time = millis();
  
  while (isBusy()) {
    // Check for abort request (lightweight check)
    if (abort_requested_) {
      return GantryError::OK;  // Return OK since abort was requested
    }
    
    if (timeout_ms > 0) {
      // Unsigned subtraction correctly handles millis() overflow (wraparound after ~49 days)
      uint32_t elapsed = millis() - start_time;
      if (elapsed > timeout_ms) {
        return GantryError::TIMEOUT;
      }
    }
    
#if defined(ARDUINO_ARCH_ESP32)
    vTaskDelay(pdMS_TO_TICKS(10));
#else
    delay(10);
#endif
  }
  
  // Note: Motion complete callback is handled by update(), not here
  // This prevents duplicate callback invocations
  return GantryError::OK;
}

GantryStatus Gantry::getStatus() const {
  GantryStatus status = {};
  
  status.currentX_mm = getCurrentX();
  status.currentY_mm = currentY_;
  status.currentTheta_deg = currentTheta_;
  
  status.targetX_mm = getTargetX();  // Use getTargetX() for consistency (applies workspace offset)
  status.targetY_mm = targetY_;
  status.targetTheta_deg = targetTheta_;
  
  status.isBusy = isBusy();
  status.xMoving = axisX_.isMotionActive();
  status.yMoving = y_moving_;
  status.thetaMoving = theta_moving_;
  
  status.initialized = initialized_;
  status.enabled = axisX_.isEnabled();
  status.gripperActive = gripperActive_;
  
  status.axisLength_mm = getAxisLength();
  status.workspaceOriginOffset_mm = workspace_origin_offset_mm_;
  
  status.lastUpdate_ms = millis();
  
  return status;
}

// ============================================================================
// UPDATE LOOP
// ============================================================================

void Gantry::update() {
  if (!initialized_) {
    return;
  }

  // Update current X position from encoder (use getEncoderPosition for signed support)
  currentX_ = axisX_.getEncoderPosition();

  // Check if X motion just completed (notify callback)
  // Note: This is a simple check - motion completion is detected when
  // isMotionActive() transitions from true to false
  bool x_is_moving_now = axisX_.isMotionActive();
  if (x_was_moving_prev_ && !x_is_moving_now) {
    // X motion just completed
    // Only notify if no stub movements are active
    if (!y_moving_ && !theta_moving_) {
      notifyMotionComplete(true);
    }
  }
  x_was_moving_prev_ = x_is_moving_now;

  // Simulate Y/Theta movement with trapezoidal velocity profile (similar to X-axis)
  if (y_moving_ || theta_moving_) {
    uint32_t current_ms = millis();
    // Unsigned subtraction handles millis() overflow correctly
    uint32_t elapsed_ms = current_ms - stub_move_start_ms_;
    float elapsed_s = elapsed_ms / 1000.0f;
    
    bool stub_was_moving = y_moving_ || theta_moving_;
    
    // Update Y position using trapezoidal profile
    if (y_moving_) {
      if (elapsed_s >= y_profile_.total_time) {
        // Move complete
        currentY_ = targetY_;
        y_moving_ = false;
      } else {
        // Interpolate position based on profile phase
        currentY_ = interpolateStubPosition(y_profile_, elapsed_s);
      }
    }
    
    // Update Theta position using trapezoidal profile
    if (theta_moving_) {
      if (elapsed_s >= theta_profile_.total_time) {
        // Move complete
        currentTheta_ = targetTheta_;
        theta_moving_ = false;
      } else {
        // Interpolate position based on profile phase
        currentTheta_ = interpolateStubPosition(theta_profile_, elapsed_s);
      }
    }
    
    // Notify if stub movement just completed
    if (stub_was_moving && !y_moving_ && !theta_moving_ && !isBusy()) {
      notifyMotionComplete(true);
    }
  }
}

// ============================================================================
// HOMING AND CALIBRATION
// ============================================================================

void Gantry::setLimitPins(int xMinPin, int xMaxPin) {
  xMinPin_ = xMinPin;
  xMaxPin_ = xMaxPin;
}

GantryError Gantry::home() {
  if (!initialized_) {
    log("[Gantry] ERROR: Not initialized. Call begin() first.");
    return GantryError::NOT_INITIALIZED;
  }

  if (xMinPin_ < 0) {
    log("[Gantry] ERROR: Min limit pin not configured");
    return GantryError::INVALID_PARAMETER;
  }

  if (!axisX_.isEnabled()) {
    log("[Gantry] ERROR: Motor not enabled. Call enable() first.");
    return GantryError::MOTOR_NOT_ENABLED;
  }

  homing_status_ = HomingStatus::IN_PROGRESS;
  abort_requested_ = false;  // Reset abort flag at start of homing

  // 1. Move towards Min (Negative)
  // Assuming Triggered = LOW (Switch closes to GND)
  log("[Gantry] Homing X...");

  // Check if already triggered
  if (digitalRead(xMinPin_) == LOW) {
    log("[Gantry] Already at Home. Backing off...");
    if (!axisX_.moveRelative(HOMING_BACKOFF_PULSES, HOMING_BACKOFF_SPEED, 
                              HOMING_BACKOFF_SPEED, HOMING_BACKOFF_SPEED)) {
      log("[Gantry] ERROR: Failed to back off from home");
      homing_status_ = HomingStatus::FAILED;
      return GantryError::LIMIT_SWITCH_FAILED;
    }
    
    GantryError err = waitForAxisMotionComplete(homing_timeout_ms_);
    if (abort_requested_) {
      log("[Gantry] Homing aborted during backoff");
      homing_status_ = HomingStatus::FAILED;
      return GantryError::OK;
    }
    if (err != GantryError::OK) {
      log("[Gantry] ERROR: Timeout waiting for backoff motion");
      axisX_.stopMotion(0);
      homing_status_ = HomingStatus::FAILED;
      return err;
    }
  }

  // Move Negative until trigger
  if (!axisX_.moveRelative(-HOMING_LONG_MOVE_PULSES, HOMING_SPEED, 
                            HOMING_SPEED, HOMING_SPEED)) {
    log("[Gantry] ERROR: Failed to start homing move");
    homing_status_ = HomingStatus::FAILED;
    return GantryError::LIMIT_SWITCH_FAILED;
  }

  // Wait for limit switch with timeout
  uint32_t start_time = millis();
  bool switch_triggered = false;
  while (axisX_.isMotionActive()) {
    // Check for abort request (lightweight check)
    if (abort_requested_) {
      log("[Gantry] Homing aborted by user");
      axisX_.stopMotion(0);
      homing_status_ = HomingStatus::FAILED;
      return GantryError::OK;  // Return OK since abort was requested (not an error)
    }
    
    if (digitalRead(xMinPin_) == LOW) {
      // Triggered!
      axisX_.stopMotion(0); // Hard stop
      switch_triggered = true;
      break;
    }
    
    // Check timeout
    // Unsigned subtraction correctly handles millis() overflow (wraparound after ~49 days)
    uint32_t elapsed = millis() - start_time;
    if (elapsed > homing_timeout_ms_) {
      log("[Gantry] ERROR: Homing timeout - limit switch not triggered");
      axisX_.stopMotion(0);
      homing_status_ = HomingStatus::FAILED;
      return GantryError::TIMEOUT;
    }

#if defined(ARDUINO_ARCH_ESP32)
    vTaskDelay(pdMS_TO_TICKS(1));
#else
    delay(1);
#endif
  }

  if (!switch_triggered) {
    log("[Gantry] ERROR: Motion stopped but switch not triggered");
    homing_status_ = HomingStatus::FAILED;
    return GantryError::LIMIT_SWITCH_FAILED;
  }

  // Small delay to ensure motion has stopped
#if defined(ARDUINO_ARCH_ESP32)
  vTaskDelay(pdMS_TO_TICKS(50));
#else
  delay(50);
#endif

  // 2. Zero Encoder
  axisX_.resetEncoderPosition();
  currentX_ = 0;
  targetX_ = 0;
  last_move_direction_ = 0;  // Reset backlash direction tracking after homing

  // Verify encoder was reset
  int32_t encoder_pos = axisX_.getEncoderPosition();
  if (encoder_pos != 0) {
    log("[Gantry] WARNING: Encoder reset may have failed");
    // Still continue, but log warning
  }

  // 3. Back off slightly
  if (!axisX_.moveRelative(HOMING_SMALL_BACKOFF_PULSES, HOMING_BACKOFF_SPEED,
                            HOMING_BACKOFF_SPEED, HOMING_BACKOFF_SPEED)) {
    log("[Gantry] ERROR: Failed to back off after home");
    homing_status_ = HomingStatus::FAILED;
    return GantryError::LIMIT_SWITCH_FAILED;
  }

  GantryError err = waitForAxisMotionComplete(homing_timeout_ms_);
  if (abort_requested_) {
    log("[Gantry] Homing aborted during final backoff");
    homing_status_ = HomingStatus::FAILED;
    return GantryError::OK;
  }
  if (err != GantryError::OK) {
    log("[Gantry] ERROR: Timeout waiting for final backoff");
    axisX_.stopMotion(0);
    homing_status_ = HomingStatus::FAILED;
    return err;
  }

  // Update current position after backoff
  currentX_ = axisX_.getEncoderPosition();
  targetX_ = currentX_;

  homing_status_ = HomingStatus::COMPLETE;
  log("[Gantry] Homing Complete.");
  return GantryError::OK;
}

GantryError Gantry::calibrate(bool returnToHome) {
  if (!initialized_) {
    log("[Gantry] ERROR: Not initialized. Call begin() first.");
    return GantryError::NOT_INITIALIZED;
  }

  if (xMaxPin_ < 0) {
    log("[Gantry] ERROR: Max limit pin not configured");
    return GantryError::INVALID_PARAMETER;
  }

  if (!axisX_.isEnabled()) {
    log("[Gantry] ERROR: Motor not enabled. Call enable() first.");
    return GantryError::MOTOR_NOT_ENABLED;
  }

  log("[Gantry] Calibrating X...");
  
  calibrating_ = true;  // Mark calibration as in progress
  abort_requested_ = false;  // Reset abort flag at start of calibration

  // 1. Home first
  GantryError err = home();
  if (abort_requested_) {
    log("[Gantry] Calibration aborted during homing");
    calibrating_ = false;  // Clear calibration flag
    return GantryError::OK;  // Return OK since abort was requested
  }
  if (err != GantryError::OK) {
    log("[Gantry] ERROR: Homing failed during calibration");
    calibrating_ = false;  // Clear calibration flag
    return GantryError::CALIBRATION_FAILED;
  }

  // 2. Move Positive towards Max
  if (!axisX_.moveRelative(HOMING_LONG_MOVE_PULSES, HOMING_SPEED,
                            HOMING_SPEED, HOMING_SPEED)) {
    log("[Gantry] ERROR: Failed to start calibration move");
    calibrating_ = false;  // Clear calibration flag
    return GantryError::CALIBRATION_FAILED;
  }

  // Wait for limit switch with timeout
  uint32_t start_time = millis();
  bool switch_triggered = false;
  while (axisX_.isMotionActive()) {
    // Check for abort request (lightweight check)
    if (abort_requested_) {
      log("[Gantry] Calibration aborted by user");
      axisX_.stopMotion(0);
      calibrating_ = false;  // Clear calibration flag
      return GantryError::OK;  // Return OK since abort was requested (not an error)
    }
    
    if (digitalRead(xMaxPin_) == LOW) {
      axisX_.stopMotion(0);
      switch_triggered = true;
      break;
    }
    
    // Check timeout
    // Unsigned subtraction correctly handles millis() overflow (wraparound after ~49 days)
    uint32_t elapsed = millis() - start_time;
    if (elapsed > homing_timeout_ms_) {
      log("[Gantry] ERROR: Calibration timeout - max limit switch not triggered");
      axisX_.stopMotion(0);
      calibrating_ = false;  // Clear calibration flag
      return GantryError::TIMEOUT;
    }

#if defined(ARDUINO_ARCH_ESP32)
    vTaskDelay(pdMS_TO_TICKS(1));
#else
    delay(1);
#endif
  }

  if (!switch_triggered) {
    log("[Gantry] ERROR: Motion stopped but max switch not triggered");
    calibrating_ = false;  // Clear calibration flag
    return GantryError::LIMIT_SWITCH_FAILED;
  }

  // Small delay to ensure motion has stopped
#if defined(ARDUINO_ARCH_ESP32)
  vTaskDelay(pdMS_TO_TICKS(50));
#else
  delay(50);
#endif

  // 3. Record Length (ensure positive)
  int32_t encoder_pos = axisX_.getEncoderPosition();
  axisLength_ = (encoder_pos < 0) ? -encoder_pos : encoder_pos;  // Use absolute value
  currentX_ = encoder_pos;
  targetX_ = encoder_pos;

  // Validate length is reasonable (at least some minimum)
  if (axisLength_ <= 0) {
    log("[Gantry] ERROR: Invalid axis length measured");
    calibrating_ = false;  // Clear calibration flag
    return GantryError::CALIBRATION_FAILED;
  }

  // Log calibration result
  char msg[128];
  snprintf(msg, sizeof(msg), "[Gantry] Calibration Complete. Length: %ld pulses (%ld mm)", 
           axisLength_, getAxisLength());
  log(msg);

  // 4. Return to home if requested
  if (returnToHome) {
    err = home();
    if (err != GantryError::OK) {
      log("[Gantry] WARNING: Failed to return to home after calibration");
      calibrating_ = false;  // Clear calibration flag
      return err;
    }
  }

  calibrating_ = false;  // Clear calibration flag on success
  return GantryError::OK;
}

// ============================================================================
// ACCESSORS
// ============================================================================

int32_t Gantry::getCurrentX() const {
  if (!initialized_) {
    return 0;
  }
  
  // Get X-axis position in mm (home coordinates)
  int32_t x_axis_home_mm = pulsesToMm(currentX_);
  
  // Apply forward kinematics: Calculate end-effector position from X-axis position
  int32_t end_effector_home_mm = forwardKinematicsX(x_axis_home_mm, currentTheta_);
  
  // Convert from home coordinates to workspace coordinates
  // workspace_mm = home_mm - workspace_origin_offset_mm_
  return end_effector_home_mm - workspace_origin_offset_mm_;
}

int32_t Gantry::getTargetX() const {
  if (!initialized_) {
    return 0;
  }
  
  // Get X-axis target position in mm (home coordinates)
  int32_t x_axis_home_mm = pulsesToMm(targetX_);
  
  // Apply forward kinematics: Calculate end-effector target position from X-axis position
  int32_t end_effector_home_mm = forwardKinematicsX(x_axis_home_mm, targetTheta_);
  
  // Convert from home coordinates to workspace coordinates
  // workspace_mm = home_mm - workspace_origin_offset_mm_
  return end_effector_home_mm - workspace_origin_offset_mm_;
}

int32_t Gantry::getRemainingDistanceX() const {
  int32_t diff = getTargetX() - getCurrentX();
  return (diff < 0) ? -diff : diff;  // Manual abs to avoid include dependency
}

int32_t Gantry::getPositionErrorXmm() const {
  if (!initialized_) {
    return 0;
  }
  // Get error in workspace coordinates (mm)
  return getTargetX() - getCurrentX();
}

int32_t Gantry::getAxisLength() const {
  return pulsesToMm(axisLength_);
}

// ============================================================================
// KINEMATICS (FORWARD AND INVERSE)
// ============================================================================

int32_t Gantry::forwardKinematicsX(int32_t x_axis_mm, int32_t theta_deg) const {
  // Forward kinematics: Calculate end-effector X position from joint positions
  // Theta rotates around Y axis, so it doesn't change the X position of the gripper center
  // End-effector X = X_axis + THETA_X_OFFSET_MM (constant, independent of Theta)
  return x_axis_mm + THETA_X_OFFSET_MM;
}

int32_t Gantry::inverseKinematicsX(int32_t end_effector_x_mm, int32_t theta_deg) const {
  // Inverse kinematics: Calculate required X-axis position from end-effector position
  // Theta rotates around Y axis, so it doesn't affect the X position calculation
  // X_axis = end_effector_x - THETA_X_OFFSET_MM
  return end_effector_x_mm - THETA_X_OFFSET_MM;
}

// ============================================================================
// STUB PROFILE HELPERS (Trapezoidal Velocity, Similar to X-axis)
// ============================================================================

void Gantry::calculateStubProfile(StubProfile &profile, int32_t start_pos, int32_t target_pos,
                                   float max_speed, float accel_rate, float decel_rate) {
  profile.start_pos = start_pos;
  profile.target_pos = target_pos;
  profile.max_speed = max_speed;
  profile.accel_rate = accel_rate;
  profile.decel_rate = decel_rate;
  
  int32_t distance = target_pos - start_pos;
  if (distance == 0) {
    profile.t_accel = 0;
    profile.t_cruise = 0;
    profile.t_decel = 0;
    profile.total_time = 0;
    return;
  }
  
  // Calculate time to accelerate to max speed: t = v / a
  profile.t_accel = max_speed / accel_rate;
  profile.t_decel = max_speed / decel_rate;
  
  // Calculate distance during accel/decel: d = 0.5 * a * t²
  float dist_accel = 0.5f * accel_rate * profile.t_accel * profile.t_accel;
  float dist_decel = 0.5f * decel_rate * profile.t_decel * profile.t_decel;
  float total_dist = (distance < 0) ? -distance : distance;
  
  // Handle short moves (triangular profile - won't reach max speed)
  if (dist_accel + dist_decel > total_dist) {
    // Scale down - won't reach max speed
    float scale = sqrtf(total_dist / (dist_accel + dist_decel));
    profile.t_accel *= scale;
    profile.t_decel *= scale;
    dist_accel = 0.5f * accel_rate * profile.t_accel * profile.t_accel;
    dist_decel = total_dist - dist_accel;
    profile.t_cruise = 0;
  } else {
    // Trapezoidal profile - has cruise phase
    float dist_cruise = total_dist - dist_accel - dist_decel;
    profile.t_cruise = dist_cruise / max_speed;
  }
  
  profile.total_time = profile.t_accel + profile.t_cruise + profile.t_decel;
}

int32_t Gantry::interpolateStubPosition(const StubProfile &profile, float elapsed_s) {
  if (elapsed_s <= 0) {
    return profile.start_pos;
  }
  if (elapsed_s >= profile.total_time) {
    return profile.target_pos;
  }
  
  int32_t distance = profile.target_pos - profile.start_pos;
  float current_pos = profile.start_pos;
  
  if (elapsed_s < profile.t_accel) {
    // Acceleration phase: s = 0.5 * a * t²
    float t = elapsed_s;
    float dist = 0.5f * profile.accel_rate * t * t;
    current_pos += (distance < 0) ? -dist : dist;
  } else if (elapsed_s < profile.t_accel + profile.t_cruise) {
    // Cruise phase: s = s_accel + v * (t - t_accel)
    float dist_accel = 0.5f * profile.accel_rate * profile.t_accel * profile.t_accel;
    float t_cruise = elapsed_s - profile.t_accel;
    float dist_cruise = profile.max_speed * t_cruise;
    float total_dist = dist_accel + dist_cruise;
    current_pos += (distance < 0) ? -total_dist : total_dist;
  } else {
    // Deceleration phase: s = s_start + s_accel + s_cruise + (v*t_decel - 0.5*a*t²)
    float dist_accel = 0.5f * profile.accel_rate * profile.t_accel * profile.t_accel;
    float dist_cruise = profile.max_speed * profile.t_cruise;
    float t_decel = elapsed_s - profile.t_accel - profile.t_cruise;
    float dist_decel = profile.max_speed * t_decel - 0.5f * profile.decel_rate * t_decel * t_decel;
    float total_dist = dist_accel + dist_cruise + dist_decel;
    current_pos += (distance < 0) ? -total_dist : total_dist;
  }
  
  return (int32_t)current_pos;
}

// ============================================================================
// CONFIGURATION
// ============================================================================

void Gantry::setSoftLimits(int32_t min_mm, int32_t max_mm) {
  if (min_mm >= max_mm) {
    // Disable soft limits if invalid
    soft_limits_enabled_ = false;
    return;
  }
  soft_limit_min_mm_ = min_mm;
  soft_limit_max_mm_ = max_mm;
  soft_limits_enabled_ = true;
}

void Gantry::getSoftLimits(int32_t &min_mm, int32_t &max_mm) const {
  min_mm = soft_limit_min_mm_;
  max_mm = soft_limit_max_mm_;
}

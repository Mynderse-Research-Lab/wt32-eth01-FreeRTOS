/**
 * @file Gantry.cpp
 * @brief Implementation of Gantry class.
 * @version 2.0.0
 */

#include "Gantry.h"
#include "GantryPulseMotorLinearAxis.h"
#include "GantryPulseMotorRotaryAxis.h"
#include "gpio_expander.h"
#include <cmath>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {
// Arduino-compat helpers, pure ESP-IDF implementation.
static inline unsigned long gantry_millis() {
    return (unsigned long)(esp_timer_get_time() / 1000LL);
}
static inline void gantry_delay(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}
}  // namespace

using namespace Gantry::Constants;

namespace Gantry {
static const char* TAG = "Gantry";

#ifndef GANTRY_DIAG_SKIP_AXIS_X_INIT
#define GANTRY_DIAG_SKIP_AXIS_X_INIT 0
#endif
#ifndef GANTRY_DIAG_SKIP_AXIS_Y_INIT
#define GANTRY_DIAG_SKIP_AXIS_Y_INIT 0
#endif
#ifndef GANTRY_DIAG_SKIP_THETA_INIT
#define GANTRY_DIAG_SKIP_THETA_INIT 0
#endif

namespace {

std::unique_ptr<GantryLinearAxis> makeLinearAxis(
    const char* axisLabel,
    const PulseMotor::DriverConfig& drv,
    const PulseMotor::DrivetrainConfig& dt) {
    switch (dt.type) {
        case PulseMotor::DRIVETRAIN_BALLSCREW:
        case PulseMotor::DRIVETRAIN_BELT:
        case PulseMotor::DRIVETRAIN_RACKPINION:
            return std::unique_ptr<GantryLinearAxis>(
                new GantryPulseMotorLinearAxis(drv, dt));
        default:
            ESP_LOGE(TAG,
                     "%s: DrivetrainType=%d is not linear; axis will not be created",
                     axisLabel, (int)dt.type);
            return nullptr;
    }
}

std::unique_ptr<GantryRotaryAxis> makeRotaryAxis(
    const char* axisLabel,
    const PulseMotor::DriverConfig& drv,
    const PulseMotor::DrivetrainConfig& dt) {
    if (dt.type == PulseMotor::DRIVETRAIN_ROTARY_DIRECT) {
        return std::unique_ptr<GantryRotaryAxis>(
            new GantryPulseMotorRotaryAxis(drv, dt));
    }
    ESP_LOGE(TAG,
             "%s: DrivetrainType=%d is not rotary-direct; axis will not be created",
             axisLabel, (int)dt.type);
    return nullptr;
}

// ---------------------------------------------------------------------------
// preparePinsForBoot helpers
//
// A pin integer passed through DriverConfig can mean:
//   - -1                         -> not wired; skip.
//   - GPIO_EXPANDER_DIRECT_FLAG  -> encoded direct ESP32 GPIO (e.g. pulse);
//                                   owned by LEDC / driver/gpio, skip here.
//   - >= GPIO_DIRECT_PIN_BASE    -> plain direct ESP32 GPIO; skip here.
//   - 0..15                      -> MCP23S17 logical pin; seed via
//                                   gpio_expander_*.
// The helper below ONLY touches MCP-routed pins so the Gantry library does
// not need driver/gpio at boot-seed time.
// ---------------------------------------------------------------------------
inline bool isMcpRoutedPin(int pin) {
    if (pin < 0) return false;
    if ((pin & GPIO_EXPANDER_DIRECT_FLAG) != 0) return false;
    if (pin >= GPIO_DIRECT_PIN_BASE) return false;
    return true;
}

void seedMcpOutputLow(const char* label, int pin) {
    if (!isMcpRoutedPin(pin)) return;
    esp_err_t dir = gpio_expander_set_direction(pin, /*is_output=*/true);
    esp_err_t wr  = gpio_expander_write(pin, /*level=*/0);
    if (dir != ESP_OK || wr != ESP_OK) {
        ESP_LOGW(TAG,
                 "preparePinsForBoot: MCP pin %d (%s) seed failed (dir=%d wr=%d)",
                 pin, label, (int)dir, (int)wr);
    }
}

void seedMcpInputPullup(const char* label, int pin) {
    if (!isMcpRoutedPin(pin)) return;
    esp_err_t dir = gpio_expander_set_direction(pin, /*is_output=*/false);
    esp_err_t pu  = gpio_expander_set_pullup   (pin, /*enable=*/true);
    if (dir != ESP_OK || pu != ESP_OK) {
        ESP_LOGW(TAG,
                 "preparePinsForBoot: MCP pin %d (%s) seed failed (dir=%d pu=%d)",
                 pin, label, (int)dir, (int)pu);
    }
}

void seedOneDriver(const char* axis, const PulseMotor::DriverConfig& drv) {
    // Outputs: DIR, ENABLE/SON, ALARM_RESET (if routed to MCP).
    seedMcpOutputLow(axis, drv.dir_pin);
    seedMcpOutputLow(axis, drv.enable_pin);
    seedMcpOutputLow(axis, drv.alarm_reset_pin);
    // Inputs: ALARM_STATUS needs pull-up (active-low open-drain typical).
    seedMcpInputPullup(axis, drv.alarm_pin);
}

} // namespace

// ============================================================================
// preparePinsForBoot (static)
// ============================================================================

void Gantry::preparePinsForBoot(const PulseMotor::DriverConfig& xDrv,
                                const PulseMotor::DriverConfig& yDrv,
                                const PulseMotor::DriverConfig& tDrv,
                                int gripperPin) {
    ESP_LOGI(TAG, "preparePinsForBoot: seeding MCP-routed pins to safe state");
    seedOneDriver("X",     xDrv);
    seedOneDriver("Y",     yDrv);
    seedOneDriver("Theta", tDrv);
    // Gripper: only seed when routed to the MCP; direct-GPIO gripper pins
    // are configured later by GantryEndEffector::begin().
    seedMcpOutputLow("GRIPPER", gripperPin);
}

// ============================================================================
// CONSTRUCTOR
// ============================================================================

Gantry::Gantry(const PulseMotor::DriverConfig&     xDrv,
               const PulseMotor::DrivetrainConfig& xDt,
               const PulseMotor::DriverConfig&     yDrv,
               const PulseMotor::DrivetrainConfig& yDt,
               const PulseMotor::DriverConfig&     tDrv,
               const PulseMotor::DrivetrainConfig& tDt,
               int gripperPin)
  : axisX_(makeLinearAxis("X", xDrv, xDt)),
    axisY_(makeLinearAxis("Y", yDrv, yDt)),
    axisTheta_(makeRotaryAxis("Theta", tDrv, tDt)),
    gripperPin_(gripperPin),
    xMinPin_(-1),
    xMaxPin_(-1),
    initialized_(false),
    enabled_(false),
    abortRequested_(false),
    homingInProgress_(false),
    calibrationInProgress_(false),
    gripperActive_(false),
    currentX_mm_(0.0f),
    currentY_(0),
    currentTheta_(0),
    targetY_(0),
    targetTheta_(0),
    axisLength_(0),
    config_(),
    kinematicParams_(),
    stepsPerRev_(DEFAULT_STEPS_PER_REV),
    motionState_(MotionState::IDLE),
    targetX_mm_(0.0f),
    targetY_mm_(0.0f),
    targetTheta_deg_(0.0f),
    safeYHeight_mm_(DEFAULT_SAFE_Y_HEIGHT_MM),
    speed_mm_per_s_(DEFAULT_SPEED_MM_PER_S),
    speed_deg_per_s_(DEFAULT_SPEED_DEG_PER_S),
    acceleration_mm_per_s2_(0),
    deceleration_mm_per_s2_(0),
    gripperTargetState_(false),
    gripperActuateStart_ms_(0),
    gripperActuateDurationMs_(GRIPPER_ACTUATE_TIME_MS),
    lastXPositionCounts_(0),
    xPulsesPerMmOverride_(0.0f) {
}

// ============================================================================
// INITIALIZATION
// ============================================================================

bool Gantry::begin() {
    if (initialized_) {
        return true;
    }
    ESP_LOGI(TAG, "[BEGIN] Enter Gantry::begin()");

    if (gripperPin_ >= 0) {
        ESP_LOGI(TAG, "[BEGIN] Configure end-effector pin=%d", gripperPin_);
        endEffector_.configurePin(gripperPin_, true);
        endEffector_.begin();
        gripperActive_ = false;
        ESP_LOGI(TAG, "[BEGIN] End-effector initialized");
    }

    xMinSwitch_.begin();
    xMaxSwitch_.begin();
    ESP_LOGI(TAG, "[BEGIN] Limit switch objects initialized");

    if (GANTRY_DIAG_SKIP_AXIS_X_INIT) {
        ESP_LOGW(TAG, "[BEGIN] Skipping X-axis initialize (diagnostic toggle)");
    } else if (axisX_) {
        ESP_LOGI(TAG, "[BEGIN] Initializing X-axis driver");
        if (!axisX_->begin()) {
            ESP_LOGE(TAG, "[BEGIN] X-axis initialize failed");
            return false;
        }
        ESP_LOGI(TAG, "[BEGIN] X-axis initialize OK");
    } else {
        ESP_LOGE(TAG, "[BEGIN] X-axis is null (misconfigured DrivetrainType)");
        return false;
    }

    if (GANTRY_DIAG_SKIP_AXIS_Y_INIT) {
        ESP_LOGW(TAG, "[BEGIN] Skipping Y-axis initialize (diagnostic toggle)");
    } else if (axisY_) {
        ESP_LOGI(TAG, "[BEGIN] Initializing Y-axis driver");
        if (!axisY_->begin()) {
            ESP_LOGE(TAG, "[BEGIN] Y-axis initialize failed");
            return false;
        }
        ESP_LOGI(TAG, "[BEGIN] Y-axis initialize OK");
    } else {
        ESP_LOGW(TAG, "[BEGIN] Y-axis is null (misconfigured DrivetrainType)");
    }

    if (GANTRY_DIAG_SKIP_THETA_INIT) {
        ESP_LOGW(TAG, "[BEGIN] Skipping theta initialize (diagnostic toggle)");
    } else if (axisTheta_) {
        ESP_LOGI(TAG, "[BEGIN] Initializing theta driver");
        if (!axisTheta_->begin()) {
            ESP_LOGW(TAG, "[BEGIN] Theta initialize failed (proceeding without rotary axis)");
        } else {
            ESP_LOGI(TAG, "[BEGIN] Theta initialize OK");
        }
    } else {
        ESP_LOGW(TAG, "[BEGIN] Theta-axis is null (misconfigured DrivetrainType)");
    }

    // Wire per-axis MOVE log tags so the axis classes can emit START/END/MOVE
    // lines tagged with the human-readable axis name. Default periodic rate
    // stays at 0 (off) until the console (or other consumer) calls
    // setAxisLogRateHz(). String literals have static storage duration so the
    // pointers remain valid for the program lifetime.
    if (axisX_)     axisX_->setLogTag("X");
    if (axisY_)     axisY_->setLogTag("Y");
    if (axisTheta_) axisTheta_->setLogTag("Theta");

    initialized_ = true;
    ESP_LOGI(TAG, "[BEGIN] Gantry::begin() complete");
    return true;
}

// ============================================================================
// ENABLE/DISABLE
// ============================================================================

void Gantry::enable() {
    GANTRY_CHECK_INITIALIZED();
    bool xEnabled = false;
    if (axisX_) {
        xEnabled = axisX_->isEnabled() || axisX_->enable();
    }
    if (axisY_) {
        if (xEnabled) {
            axisY_->enable();
        } else {
            axisY_->disable();
        }
    }
    if (axisTheta_) {
        if (xEnabled) {
            axisTheta_->enable();
        } else {
            axisTheta_->disable();
        }
    }
    enabled_ = xEnabled;
}

void Gantry::disable() {
    GANTRY_CHECK_INITIALIZED();
    if (isBusy()) {
        stopAllMotion();
    }
    if (axisX_)     axisX_->disable();
    if (axisY_)     axisY_->disable();
    if (axisTheta_) axisTheta_->disable();
    enabled_ = false;
}

// ============================================================================
// CONFIGURATION
// ============================================================================

void Gantry::setLimitPins(int xMinPin, int xMaxPin) {
    xMinPin_ = xMinPin;
    xMaxPin_ = xMaxPin;
    xMinSwitch_.configure(xMinPin_, true, true, 6);
    xMaxSwitch_.configure(xMaxPin_, true, true, 6);
}

void Gantry::setYAxisLimits(float minMm, float maxMm) {
    config_.limits.y_min = minMm;
    config_.limits.y_max = maxMm;
}

void Gantry::setThetaLimits(float minDeg, float maxDeg) {
    config_.limits.theta_min = minDeg;
    config_.limits.theta_max = maxDeg;
    if (axisTheta_) {
        axisTheta_->setAngleRange(minDeg, maxDeg);
    }
}

void Gantry::setJointLimits(float xMin, float xMax,
                            float yMin, float yMax,
                            float thetaMin, float thetaMax) {
    config_.limits.x_min     = xMin;
    config_.limits.x_max     = xMax;
    config_.limits.y_min     = yMin;
    config_.limits.y_max     = yMax;
    config_.limits.theta_min = thetaMin;
    config_.limits.theta_max = thetaMax;
    if (axisTheta_) {
        axisTheta_->setAngleRange(thetaMin, thetaMax);
    }
}

void Gantry::setEndEffectorPin(int pin, bool activeHigh) {
    gripperPin_ = pin;
    endEffector_.configurePin(pin, activeHigh);
    if (initialized_) {
        endEffector_.begin();
    }
}

void Gantry::setSafeYHeight(float safeHeight_mm) {
    if (safeHeight_mm > 0.0f) {
        safeYHeight_mm_ = safeHeight_mm;
    }
}

// ============================================================================
// MOTION CONTROL
// ============================================================================

void Gantry::moveTo(int32_t x, int32_t y, int32_t theta, uint32_t speed) {
    if (!initialized_ || !enabled_) {
        return;
    }
    if (speed == 0) {
        speed = 5000;
    }

    targetY_ = y;
    targetTheta_ = theta;

    targetX_mm_       = (float)x;
    targetY_mm_       = (float)y;
    targetTheta_deg_  = (float)theta;
    // Convert X pulses/s to mm/s using X axis scaling.
    const float xPpm = getPulsesPerMm();
    speed_mm_per_s_        = (uint32_t)((float)speed / xPpm);
    acceleration_mm_per_s2_ = 0;
    deceleration_mm_per_s2_ = 0;

    startSequentialMotion();
}

GantryError Gantry::moveTo(const JointConfig& joint,
                           uint32_t speed_mm_per_s,
                           uint32_t speed_deg_per_s,
                           uint32_t acceleration_mm_per_s2,
                           uint32_t deceleration_mm_per_s2) {
    GANTRY_CHECK_INITIALIZED_RET(GantryError::NOT_INITIALIZED);
    GANTRY_CHECK_ENABLED_RET(GantryError::MOTOR_NOT_ENABLED);
    GANTRY_CHECK_BUSY_RET(GantryError::ALREADY_MOVING);

    if (axisX_ && axisX_->isAlarmActive()) {
        return GantryError::TIMEOUT;
    }

    if (!Kinematics::validate(joint, config_.limits)) {
        return GantryError::INVALID_POSITION;
    }

    targetX_mm_       = joint.x;
    targetY_mm_       = joint.y;
    targetTheta_deg_  = joint.theta;
    speed_mm_per_s_   = speed_mm_per_s;
    speed_deg_per_s_  = speed_deg_per_s;
    acceleration_mm_per_s2_ = acceleration_mm_per_s2;
    deceleration_mm_per_s2_ = deceleration_mm_per_s2;

    startSequentialMotion();
    return GantryError::OK;
}

GantryError Gantry::moveTo(const EndEffectorPose& pose,
                           uint32_t speed_mm_per_s,
                           uint32_t speed_deg_per_s,
                           uint32_t acceleration_mm_per_s2,
                           uint32_t deceleration_mm_per_s2) {
    GANTRY_CHECK_INITIALIZED_RET(GantryError::NOT_INITIALIZED);
    GANTRY_CHECK_ENABLED_RET(GantryError::MOTOR_NOT_ENABLED);
    GANTRY_CHECK_BUSY_RET(GantryError::ALREADY_MOVING);

    JointConfig joint = inverseKinematics(pose);
    if (!Kinematics::validate(joint, config_.limits)) {
        return GantryError::INVALID_POSITION;
    }

    return moveTo(joint, speed_mm_per_s, speed_deg_per_s,
                  acceleration_mm_per_s2, deceleration_mm_per_s2);
}

bool Gantry::isBusy() const {
    if (!initialized_) {
        return false;
    }
    const bool xBusy = axisX_ && (axisX_->isMotionActive());
    const bool yBusy = axisY_ && axisY_->isBusy();
    const bool tBusy = axisTheta_ && axisTheta_->isMotionActive();
    return motionState_ != MotionState::IDLE || xBusy || yBusy || tBusy ||
           homingInProgress_ || calibrationInProgress_;
}

bool Gantry::isEnabled() const { return initialized_ && enabled_; }

void Gantry::requestAbort() {
    abortRequested_ = true;
    if (initialized_) {
        stopAllMotion();
    }
}

bool Gantry::isAbortRequested() const { return abortRequested_; }

void Gantry::update() {
    GANTRY_CHECK_INITIALIZED();

    if (motionState_ != MotionState::IDLE) {
        processSequentialMotion();
    }

    updateAxisPositions();
}

// ============================================================================
// HOMING AND CALIBRATION
// ============================================================================

void Gantry::home() {
    // Legacy entry point: only X is currently real, so behavior is unchanged.
    // Y and Theta stubs are NOT invoked here so existing callers that depend
    // on `home()` blocking only on the X sweep keep working.
    homeX();
}

void Gantry::homeX() {
    GANTRY_CHECK_INITIALIZED();
    GANTRY_CHECK_ENABLED();
    abortRequested_ = false;
    homingInProgress_ = false;

    if (!axisX_) return;
    if (axisX_->isAlarmActive()) return;

    if (!xMinSwitch_.isConfigured() || !xMaxSwitch_.isConfigured()) {
        return;
    }

    xMinSwitch_.update(true);
    xMaxSwitch_.update(true);

    if (xMinSwitch_.isActive()) {
        axisX_->stopMotion();
        axisX_->setCurrentPulses(0);
        return;
    }

    constexpr uint32_t kHomingStartPosition = 100000000;
    const uint32_t     speed                = getHomingSpeed();
    axisX_->setCurrentPulses(kHomingStartPosition);
    // Seed the position tracker to match the synthetic bump. Otherwise the
    // very next updateAxisPositions() tick sees currentXCounts (~100M) >
    // lastXPositionCounts_ (~0), mis-classifies it as "moving toward MAX",
    // and if MAX is active it instantly stop-motions the home before any
    // real pulses are emitted. (Confirmed by runtime evidence: setDirection
    // fires with logical=0, then "Homing did not start" appears 20ms later
    // with gate state max=1.)
    lastXPositionCounts_ = kHomingStartPosition;
    ESP_LOGI(TAG,
             "[HOME] seeded position tracker to %lu before negative-direction sweep",
             (unsigned long)kHomingStartPosition);
    if (!axisX_->moveToPulses(0, speed, speed, speed)) {
        stopAllMotion();
        return;
    }
    homingInProgress_ = true;
}

void Gantry::homeY() {
    // Y limit switches are defined in the pinout (PIN_Y_LIMIT_MIN/MAX) but the
    // Gantry class does not yet wire them up. Surface this as a warning so
    // operators know the command was received but did not act on hardware.
    ESP_LOGW(TAG, "[HOME] Y axis home not yet wired (Y limit switches not integrated)");
}

void Gantry::homeTheta() {
    // Theta currently has no limit switches; pins that used to host theta
    // limits were reassigned to DIR/ENABLE. Log a placeholder until the
    // switches are added to the hardware.
    ESP_LOGW(TAG, "[HOME] Theta axis home not yet wired (no theta limit switches)");
}

int Gantry::calibrate() {
    return calibrateX();
}

int Gantry::calibrateY() {
    ESP_LOGW(TAG, "[CAL] Y axis calibrate not yet wired (Y limit switches not integrated)");
    return 0;
}

int Gantry::calibrateTheta() {
    ESP_LOGW(TAG, "[CAL] Theta axis calibrate not yet wired (no theta limit switches)");
    return 0;
}

int Gantry::calibrateX() {
    GANTRY_CHECK_INITIALIZED_RET(0);
    GANTRY_CHECK_ENABLED_RET(0);
    abortRequested_ = false;

    if (!axisX_) return 0;
    if (axisX_->isAlarmActive()) return 0;

    calibrationInProgress_ = true;

    if (!xMinSwitch_.isConfigured() || !xMaxSwitch_.isConfigured()) {
        stopAllMotion();
        calibrationInProgress_ = false;
        return 0;
    }

    homeX();

    unsigned long start_ms = gantry_millis();
    while (homingInProgress_ && (gantry_millis() - start_ms) < CALIBRATION_TIMEOUT_MS) {
        if (abortRequested_) {
            stopAllMotion();
            calibrationInProgress_ = false;
            return 0;
        }
        xMinSwitch_.update();
        xMaxSwitch_.update();
        gantry_delay(10);
    }

    if (homingInProgress_ || !xMinSwitch_.isActive()) {
        stopAllMotion();
        calibrationInProgress_ = false;
        return 0;
    }

    const uint32_t speed = getHomingSpeed();
    constexpr uint32_t kCalibrationTarget = 1000000000UL;
    constexpr uint32_t kMinReleaseTimeoutMs = 3000;
    if (!axisX_->moveToPulses(kCalibrationTarget, speed, speed, speed)) {
        stopAllMotion();
        calibrationInProgress_ = false;
        return 0;
    }

    start_ms = gantry_millis();
    bool minReleased = false;
    while (axisX_->isMotionActive() &&
           (gantry_millis() - start_ms) < TRAVEL_MEASUREMENT_TIMEOUT_MS) {
        if (abortRequested_) {
            stopAllMotion();
            calibrationInProgress_ = false;
            return 0;
        }
        xMinSwitch_.update();
        xMaxSwitch_.update();

        if (!minReleased) {
            if (!xMinSwitch_.isActive()) {
                minReleased = true;
            } else if ((gantry_millis() - start_ms) > kMinReleaseTimeoutMs) {
                ESP_LOGW(TAG,
                         "Calibration abort: MIN limit did not release within %lu ms",
                         (unsigned long)kMinReleaseTimeoutMs);
                stopAllMotion();
                calibrationInProgress_ = false;
                return 0;
            }
        }

        gantry_delay(10);
        if (axisX_->isAlarmActive()) {
            stopAllMotion();
            calibrationInProgress_ = false;
            return 0;
        }
    }

    // The inner loop exited because axisX_->isMotionActive() became false.
    // The only place that stops motion mid-sweep is updateAxisPositions()'s
    // movingTowardMax + xMaxSwitch_.isActive() guard, which is also what
    // sets calibrationInProgress_=false. So a non-active stableState here
    // genuinely means we exited for a different reason (timeout, abort,
    // alarm) — those branches have already returned. Reaching this point
    // with stableState=true is the success signal.
    //
    // DO NOT call xMaxSwitch_.update(true) here. Force-overwriting the
    // debounced stableState with the raw level at this instant races
    // against the carriage's residual deceleration overshooting the
    // inductive sensor's detection zone. The debounce that already
    // committed during the inner loop is the correct truth — the carriage
    // really did pass over the MAX sensor for >=60ms of stable sampling.
    // (Confirmed by runtime evidence: every "Calibration failed" event is
    // preceded by max_limit transitioning 0->1 ~25ms earlier.)
    if (!xMaxSwitch_.isActive()) {
        ESP_LOGW(TAG,
                 "[CAL] inner loop exited but xMaxSwitch_ not active "
                 "(unexpected; aborting)");
        stopAllMotion();
        calibrationInProgress_ = false;
        return 0;
    }

    // Use the commanded driver position (LEDC pulse count), NOT
    // axisX_->getCurrentMm(). The latter routes through the encoder
    // accumulator when enable_encoder_feedback=true, and the X-axis
    // encoder is currently not returning pulses (see
    // PROGRAMMING_REFERENCE.md §11.1 — wiring TODO). We measured the
    // physical travel by commanding pulses from MIN to MAX, so the
    // pulse count between those two events is the authoritative travel
    // distance regardless of encoder availability.
    const double commandedPulses = (double)axisX_->getCurrentPulses();
    const double basePpm         = axisX_->pulsesPerMm();
    if (basePpm <= 0.0) {
        ESP_LOGE(TAG, "[CAL] pulses-per-mm not configured; cannot compute axis length");
        calibrationInProgress_ = false;
        return 0;
    }

    // Re-anchor X scaling to the known physical hard-travel envelope.
    // This keeps dead-reckoning travel accurate even if commanded pulses/rev
    // in the drive do not match the compile-time drivetrain constants yet.
    const double hardSpanMm = (double)AXIS_X_HARD_LIMIT_MAX_MM - (double)AXIS_X_HARD_LIMIT_MIN_MM;
    const double derivedLengthMm = commandedPulses / basePpm;
    if (hardSpanMm > 1.0 && commandedPulses > 0.0) {
        xPulsesPerMmOverride_ = (float)(commandedPulses / hardSpanMm);
        axisLength_ = (int32_t)(hardSpanMm + 0.5);
        ESP_LOGI(TAG,
                 "[CAL] travel measured %.1f mm from base ppm=%.4f; applying corrected ppm=%.4f from hard span=%.1f mm",
                 derivedLengthMm, basePpm, (double)xPulsesPerMmOverride_, hardSpanMm);
    } else {
        xPulsesPerMmOverride_ = 0.0f;
        axisLength_ = (int32_t)(derivedLengthMm + 0.5);
        ESP_LOGW(TAG,
                 "[CAL] hard span unavailable; using base ppm only (axisLength=%ld mm, ppm=%.4f)",
                 (long)axisLength_, basePpm);
    }
    calibrationInProgress_ = false;
    return axisLength_;
}

// ============================================================================
// GRIPPER
// ============================================================================

void Gantry::grip(bool active) {
    GANTRY_CHECK_INITIALIZED();
    if (endEffector_.isConfigured()) {
        endEffector_.setActive(active);
        gripperActive_ = active;
    }
}

// ============================================================================
// ACCESSORS
// ============================================================================

int Gantry::getXEncoder() const {
    if (!initialized_ || !axisX_) return 0;
    if (axisX_->isEncoderFeedbackEnabled()) {
        return axisX_->getEncoderPulses();
    }
    return (int32_t)axisX_->getCurrentPulses();
}

int Gantry::getXEncoderRaw() const {
    if (!initialized_ || !axisX_) return 0;
    return axisX_->getEncoderPulses();
}

int32_t Gantry::getXCommandedPulses() const {
    if (!initialized_ || !axisX_) return 0;
    if (homingInProgress_ || calibrationInProgress_) {
        return 0;
    }
    return (int32_t)axisX_->getCurrentPulses();
}

float Gantry::getXCommandedMm() const {
    const float ppm = getPulsesPerMm();
    if (ppm <= 0.0f) return 0.0f;
    return (float)((double)getXCommandedPulses() / (double)ppm);
}

float Gantry::getXEncoderMm() const {
    const float ppm = getPulsesPerMm();
    if (ppm <= 0.0f) return 0.0f;
    return (float)((double)getXEncoderRaw() / (double)ppm);
}

int Gantry::getCurrentY() const {
    return (int)(axisY_ ? axisY_->getCurrentMm() : (float)currentY_);
}

int Gantry::getCurrentTheta() const {
    return axisTheta_ ? (int)axisTheta_->getCurrentDeg() : currentTheta_;
}

// ============================================================================
// ALARM MONITORING
// ============================================================================

bool Gantry::isAlarmActive() const {
    if (!initialized_) return false;
    const bool xAlarm = axisX_     && axisX_->isAlarmActive();
    const bool yAlarm = axisY_     && axisY_->isAlarmActive();
    const bool tAlarm = axisTheta_ && axisTheta_->isAlarmActive();
    return xAlarm || yAlarm || tAlarm;
}

bool Gantry::clearAlarm() {
    if (!initialized_) return false;
    bool ok = false;
    if (axisX_)     ok = axisX_->clearAlarm()     || ok;
    if (axisY_)     ok = axisY_->clearAlarm()     || ok;
    if (axisTheta_) ok = axisTheta_->clearAlarm() || ok;
    return ok;
}

void Gantry::setHomingSpeed(uint32_t speed_pps) {
    // Homing speed lives in the driver config; route via the X driver
    // (and Y when its own home command is added) by mutating the config.
    if (!initialized_ || !axisX_) return;
    auto* xImpl = static_cast<GantryPulseMotorLinearAxis*>(axisX_.get());
    PulseMotor::DriverConfig& cfg =
        const_cast<PulseMotor::DriverConfig&>(xImpl->driver().getConfig());
    cfg.homing_speed_pps = speed_pps;
}

void Gantry::setAxisLogRateHz(uint32_t hz) {
    if (axisX_)     axisX_->setLogRateHz(hz);
    if (axisY_)     axisY_->setLogRateHz(hz);
    if (axisTheta_) axisTheta_->setLogRateHz(hz);
}

// ============================================================================
// KINEMATICS
// ============================================================================

EndEffectorPose Gantry::forwardKinematics(const JointConfig& joint) const {
    return Kinematics::forward(joint, kinematicParams_);
}

JointConfig Gantry::inverseKinematics(const EndEffectorPose& pose) const {
    return Kinematics::inverse(pose, kinematicParams_);
}

JointConfig Gantry::getCurrentJointConfig() const {
    JointConfig joint;
    joint.x     = currentX_mm_;
    joint.y     = axisY_     ? axisY_->getCurrentMm()     : (float)currentY_;
    joint.theta = axisTheta_ ? axisTheta_->getCurrentDeg() : (float)currentTheta_;
    return joint;
}

JointConfig Gantry::getTargetJointConfig() const {
    JointConfig joint;
    joint.x     = targetX_mm_;
    joint.y     = axisY_ ? axisY_->getTargetMm() : targetY_mm_;
    joint.theta = axisTheta_ ? axisTheta_->getTargetDeg() : (float)targetTheta_;
    return joint;
}

EndEffectorPose Gantry::getCurrentEndEffectorPose() const {
    return forwardKinematics(getCurrentJointConfig());
}

EndEffectorPose Gantry::getTargetEndEffectorPose() const {
    return forwardKinematics(getTargetJointConfig());
}

// ============================================================================
// HELPERS
// ============================================================================

void Gantry::setStepsPerRevolution(float steps_per_rev) {
    if (steps_per_rev > 0) {
        stepsPerRev_ = steps_per_rev;
    }
}

float Gantry::getPulsesPerMm() const {
    if (xPulsesPerMmOverride_ > 0.0f) {
        return xPulsesPerMmOverride_;
    }
    if (axisX_ && axisX_->pulsesPerMm() > 0.0) {
        return (float)axisX_->pulsesPerMm();
    }
    return DEFAULT_PULSES_PER_MM;
}

float Gantry::pulsesToMm(int32_t pulses) const {
    const float ppm = getPulsesPerMm();
    if (ppm <= 0.0f) return 0.0f;
    return (float)pulses / ppm;
}

int32_t Gantry::mmToPulses(float mm) const {
    return (int32_t)(mm * getPulsesPerMm());
}

uint32_t Gantry::getHomingSpeed() const {
    if (axisX_) {
        const uint32_t s = axisX_->homingSpeedPps();
        if (s > 0) return s;
    }
    return DEFAULT_HOMING_SPEED_PPS;
}

bool Gantry::moveYAxisTo(float targetY, float speed, float accel, float decel) {
    if (axisY_) {
        return axisY_->moveToMm(targetY, speed, accel, decel);
    }
    currentY_ = (int32_t)targetY;
    return true;
}

void Gantry::updateAxisPositions() {
    xMinSwitch_.update();
    xMaxSwitch_.update();

    if (!axisX_) {
        return;
    }

    uint32_t currentXCounts = axisX_->getCurrentPulses();
    if (axisX_->isMotionActive()) {
        bool movingTowardMax = currentXCounts > lastXPositionCounts_;
        bool movingTowardMin = currentXCounts < lastXPositionCounts_;

        if (movingTowardMax && xMaxSwitch_.isConfigured() && xMaxSwitch_.isActive()) {
            axisX_->stopMotion();
            homingInProgress_      = false;
            calibrationInProgress_ = false;
        } else if (movingTowardMin && xMinSwitch_.isConfigured() && xMinSwitch_.isActive()) {
            axisX_->stopMotion();
            axisX_->setCurrentPulses(0);
            homingInProgress_ = false;
        }
    } else if (homingInProgress_) {
        if (xMinSwitch_.isActive()) {
            axisX_->setCurrentPulses(0);
        }
        homingInProgress_ = false;
    }
    lastXPositionCounts_ = currentXCounts;

    currentX_mm_ = pulsesToMm((int32_t)currentXCounts);

    // Tick the X axis so it can run its MOVE-log state machine. The driver
    // itself runs on its own esp_timer, so update() is a logging-only hook
    // here (mirrors what's done for Y and Theta below).
    axisX_->update();

    if (axisY_) {
        axisY_->update();
        currentY_ = (int32_t)axisY_->getCurrentMm();
    }
    if (axisTheta_) {
        axisTheta_->update();
        currentTheta_ = (int32_t)axisTheta_->getCurrentDeg();
    }
}

void Gantry::stopAllMotion() {
    if (axisX_)     axisX_->stopMotion();
    if (axisY_)     axisY_->stopMotion();
    if (axisTheta_) axisTheta_->stopMotion();
    homingInProgress_      = false;
    calibrationInProgress_ = false;
    motionState_           = MotionState::IDLE;
}

// ============================================================================
// SEQUENTIAL MOTION
// ============================================================================

void Gantry::startSequentialMotion() {
    if (!enabled_ || motionState_ != MotionState::IDLE) {
        return;
    }

    const float currentY = axisY_ ? axisY_->getCurrentMm() : (float)currentY_;

    // Descending = picking (grip close). Ascending = placing (grip open).
    gripperTargetState_ = (targetY_mm_ < currentY);
    gripperActuateDurationMs_ = gripperTargetState_
        ? GRIPPER_ACTUATE_TIME_MS   // close; see docs (Constants)
        : GRIPPER_ACTUATE_TIME_MS;  // open;  see docs (Constants)

    if (currentY < safeYHeight_mm_) {
        motionState_ = MotionState::Y_RETRACTING;
        if (!moveYAxisTo(safeYHeight_mm_, (float)speed_mm_per_s_,
                         (float)acceleration_mm_per_s2_,
                         (float)deceleration_mm_per_s2_)) {
            stopAllMotion();
            return;
        }
        if (!axisY_) {
            motionState_ = MotionState::X_MOVING;
            startXAxisMotion();
        }
    } else if (targetY_mm_ < currentY) {
        motionState_ = MotionState::Y_DESCENDING;
        if (!moveYAxisTo(targetY_mm_, (float)speed_mm_per_s_,
                         (float)acceleration_mm_per_s2_,
                         (float)deceleration_mm_per_s2_)) {
            stopAllMotion();
            return;
        }
        if (!axisY_) {
            motionState_ = MotionState::GRIPPER_ACTUATING;
            grip(gripperTargetState_);
            gripperActuateStart_ms_ = gantry_millis();
        }
    } else {
        motionState_ = MotionState::X_MOVING;
        startXAxisMotion();
    }

    if (axisTheta_) {
        if (!axisTheta_->moveToDeg(targetTheta_deg_,
                                   (float)speed_deg_per_s_,
                                   0.0f, 0.0f)) {
            const float currentTheta = axisTheta_->getCurrentDeg();
            if (std::fabs(currentTheta - targetTheta_deg_) > 0.5f) {
                ESP_LOGE(TAG,
                         "[THETA] moveToDeg failed (current=%.2f target=%.2f) - aborting sequence",
                         currentTheta, targetTheta_deg_);
                stopAllMotion();
                return;
            }
            ESP_LOGI(TAG,
                     "[THETA] moveToDeg no-op treated as success (current=%.2f target=%.2f)",
                     currentTheta, targetTheta_deg_);
        }
        currentTheta_ = (int32_t)axisTheta_->getCurrentDeg();
    } else {
        currentTheta_ = (int32_t)targetTheta_deg_;
    }
}

void Gantry::processSequentialMotion() {
    if (!enabled_) {
        stopAllMotion();
        return;
    }

    switch (motionState_) {
        case MotionState::Y_DESCENDING:
            if (!axisY_ || !axisY_->isBusy()) {
                motionState_ = MotionState::GRIPPER_ACTUATING;
                grip(gripperTargetState_);
                gripperActuateStart_ms_ = gantry_millis();
            }
            break;

        case MotionState::GRIPPER_ACTUATING:
            if (gantry_millis() - gripperActuateStart_ms_ >= gripperActuateDurationMs_) {
                motionState_ = MotionState::Y_RETRACTING;
                if (!moveYAxisTo(safeYHeight_mm_, (float)speed_mm_per_s_,
                                 (float)acceleration_mm_per_s2_,
                                 (float)deceleration_mm_per_s2_)) {
                    stopAllMotion();
                    return;
                }
                if (!axisY_) {
                    motionState_ = MotionState::X_MOVING;
                    startXAxisMotion();
                }
            }
            break;

        case MotionState::Y_RETRACTING:
            if (!axisY_ || !axisY_->isBusy()) {
                motionState_ = MotionState::X_MOVING;
                startXAxisMotion();
            }
            break;

        case MotionState::X_MOVING:
            if (!axisX_ || !axisX_->isMotionActive()) {
                motionState_ = MotionState::IDLE;
            }
            break;

        case MotionState::THETA_MOVING:
            if (!axisTheta_) {
                motionState_ = MotionState::IDLE;
            }
            break;

        case MotionState::IDLE:
        default:
            break;
    }
}

void Gantry::startXAxisMotion() {
    if (!axisX_) {
        motionState_ = MotionState::IDLE;
        return;
    }

    const float xPpm = getPulsesPerMm();

    const int32_t driverReportedPulses = (int32_t)axisX_->getCurrentPulses();
    const int32_t trackedPulses        = (int32_t)(currentX_mm_ * xPpm);
    const int32_t encoderPulses        = axisX_->getEncoderPulses();
    const int32_t targetPulses         = (int32_t)(targetX_mm_ * xPpm);

    if (axisX_->isEncoderFeedbackEnabled()) {
        const int32_t encDrvMismatch = encoderPulses - driverReportedPulses;
        if (encDrvMismatch > 10 || encDrvMismatch < -10) {
            ESP_LOGW(TAG,
                     "[X_MOVE] encoder/driver mismatch: encoder=%ld driver=%ld mismatch=%ld (planning uses driver)",
                     (long)encoderPulses, (long)driverReportedPulses, (long)encDrvMismatch);
        }
    }

    const int32_t trackedMismatch = trackedPulses - driverReportedPulses;
    if (trackedMismatch > 10 || trackedMismatch < -10) {
        ESP_LOGW(TAG,
                 "[X_MOVE] tracked/driver mismatch: tracked=%ld driver=%ld mismatch=%ld",
                 (long)trackedPulses, (long)driverReportedPulses, (long)trackedMismatch);
    }

    axisX_->setCurrentPulses((uint32_t)driverReportedPulses);
    const int32_t deltaX = targetPulses - driverReportedPulses;

    ESP_LOGI(TAG,
             "[X_MOVE] current=%ld tracked=%ld driver=%ld encoder=%ld target=%ld delta=%ld speed=%lu accel=%lu decel=%lu",
             (long)driverReportedPulses, (long)trackedPulses, (long)driverReportedPulses,
             (long)encoderPulses, (long)targetPulses, (long)deltaX,
             (unsigned long)speed_mm_per_s_, (unsigned long)acceleration_mm_per_s2_,
             (unsigned long)deceleration_mm_per_s2_);

    if (deltaX == 0) {
        motionState_ = MotionState::IDLE;
        return;
    }

    const uint32_t speed_pps = (uint32_t)(speed_mm_per_s_ * xPpm);
    uint32_t accel_pps = (acceleration_mm_per_s2_ > 0)
        ? (uint32_t)(acceleration_mm_per_s2_ * xPpm) : 0;
    uint32_t decel_pps = (deceleration_mm_per_s2_ > 0)
        ? (uint32_t)(deceleration_mm_per_s2_ * xPpm) : 0;
    if (accel_pps == 0) accel_pps = speed_pps / 2;
    if (decel_pps == 0) decel_pps = speed_pps / 2;

    const uint32_t target_counts = (uint32_t)((int32_t)driverReportedPulses + deltaX);
    if (!axisX_->moveToPulses(target_counts, speed_pps, accel_pps, decel_pps)) {
        ESP_LOGE(TAG,
                 "[X_MOVE] moveToPulses rejected (delta=%ld, speed_pps=%lu, accel_pps=%lu, decel_pps=%lu)",
                 (long)deltaX, (unsigned long)speed_pps,
                 (unsigned long)accel_pps, (unsigned long)decel_pps);
        stopAllMotion();
    }
}

} // namespace Gantry

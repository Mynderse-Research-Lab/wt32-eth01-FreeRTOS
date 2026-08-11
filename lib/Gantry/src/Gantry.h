/**
 * @file Gantry.h
 * @brief Multi-axis gantry control system for ESP32.
 * @version 2.1.0
 *
 * Production motion is EtherNet/IP only (Kinetix 5100 X/Z; HCS01 theta deferred).
 * Application code drives axes exclusively through this Gantry facade.
 *
 * Coordinate convention (firmware-wide, as of 2026-05):
 *   - X: horizontal traverse along the gantry beam (across the conveyor belt).
 *   - Y: along-belt direction; NO gantry actuator. Conveyor downstream is
 *        the -Y direction. Battery positions on the belt carry y values;
 *        the motion layer does not consume them.
 *   - Z: vertical (gantry descent). +Z = up; joint Z=0 is homing datum; physical
 *        coordinated X+Z only within GANTRY_SAFE_Z_HEIGHT_MM of Z−/A015 (band
 *        ceiling = z_min + margin). Z limits: A015=min (−Z), A014=max (+Z);
 *        X: A014=min, A015=max. Z Absolute joint sense inverted so − seeks A015.
 *        The vertical actuator was called "Y" in pre-2026-05 firmware - it is
 *        the same hardware, just renamed.
 *   - Theta: rotation about Z.
 *
 * Verified hardware layout:
 *   - X: Allen-Bradley Kinetix 5100 + SCHUNK Beta 100-ZRS belt actuator.
 *   - Z: Allen-Bradley Kinetix 5100 + SCHUNK Beta 80-SRS ballscrew actuator.
 *   - Theta: Custom pulse-train driver + SCHUNK ERD 04-40-D-H-N rotary module.
 *   - End effector: SCHUNK KGG 100-80 pneumatic parallel gripper (digital).
 */

#ifndef GANTRY_H
#define GANTRY_H

#include "GantryConfig.h"
#include "GantryKinematics.h"
#include "GantryTrajectory.h"
#include "GantryLinearAxis.h"
#include "GantryRotaryAxis.h"
#include "GantryEndEffector.h"
#include "GantryLimitSwitch.h"
#include "GantryEipLimitSequence.h"
#include "GantryPathProfile.h"
#include "GantryUtils.h"
#include <memory>
#include <cstdint>
#include <cstddef>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace Gantry {

struct EndEffectorPose;

enum class GantryError {
    OK,
    NOT_INITIALIZED,
    MOTOR_NOT_ENABLED,
    ALREADY_MOVING,
    INVALID_POSITION,
    INVALID_PARAMETER,
    TIMEOUT,
    LIMIT_SWITCH_FAILED,
    CALIBRATION_FAILED,
    CONVERSION_ERROR
};

enum class HomingStatus {
    IDLE,
    IN_PROGRESS,
    COMPLETE,
    FAILED
};

struct GantryStatus {
    int32_t currentX_mm;
    int32_t currentZ_mm;
    int32_t currentTheta_deg;
    int32_t targetX_mm;
    int32_t targetZ_mm;
    int32_t targetTheta_deg;
    bool isBusy;
    bool xMoving;
    bool zMoving;
    bool thetaMoving;
    bool initialized;
    bool enabled;
    bool gripperActive;
    bool alarmActive;
    int32_t axisLength_mm;
    int32_t workspaceOriginOffset_mm;
    uint32_t lastUpdate_ms;
};

/**
 * @class Gantry
 * @brief Multi-axis gantry control.
 *
 * Construction takes pre-built EIP axis objects via dependency injection.
 * All drive control is EtherNet/IP only.
 */
class Gantry {
public:
    /**
     * @brief Dependency-injection constructor - accepts pre-built axis objects.
     *
     * Callers build GantryEipLinearAxis/GantryEipRotaryAxis externally.
     * The Gantry takes ownership of all unique_ptrs.
     */
    Gantry(std::unique_ptr<GantryLinearAxis> xAxis,
           std::unique_ptr<GantryLinearAxis> zAxis,
           std::unique_ptr<GantryRotaryAxis> thetaAxis,
           int gripperPin);

    // ---------- Boot-time helpers (static) ----------
    /**
     * @brief Seed MCP23S17 pin directions and safe-low levels before begin().
     *
     * REMOVED - MCP23S17 removed per 2026-07 refactor.
     */

    // ---------- Lifecycle ----------
    bool begin();
    void enable();
    void disable();

    // ---------- Configuration ----------
    void setLimitPins(int xMinPin, int xMaxPin);
    /// Drive-managed endstops: X/Z min/max switches take state from EIP assembly
    /// feedback (A014/A015 + Fault+Stopped heuristic), not GPIO.
    void configureDriveManagedLimits();
    /// True after configureDriveManagedLimits() (external drive-managed switches).
    bool driveManagedLimitsEnabled() const;
    /// Drive-managed Z min switch (A015/NL), if configured.
    const GantryLimitSwitch& zMinLimitSwitch() const { return zMinSwitch_; }
    const GantryLimitSwitch& zMaxLimitSwitch() const { return zMaxSwitch_; }
    /// Measured X soft max after successful EIP calibrateX (mm); 0 if unset.
    int32_t xAxisLengthMm() const { return axisLength_; }
    /// Measured Z soft max after successful EIP calibrateZ (mm); 0 if unset.
    int32_t zAxisLengthMm() const { return zAxisLength_; }
    void setZAxisLimits(float minMm, float maxMm);
    void setThetaLimits(float minDeg, float maxDeg);
    void setJointLimits(float xMin, float xMax,
                        float zMin, float zMax,
                        float thetaMin, float thetaMax);
    void setEndEffectorPin(int pin, bool activeHigh = true);
    void setSafeZHeight(float safeHeight_mm);
    /// Absolute joint-Z band ceiling for X traverse / coordinated X+Z
    /// (= z_min + Z− margin).
    float traverseClearanceZMm() const;
    /// True when current joint Z is in the SAFE_Z bottom band (X Absolute allowed).
    bool zInTraverseBand() const;

    // ---------- Motion ----------
    /// @brief Home X (Z/Theta: call homeZ / homeTheta separately; console `all` sequences).
    void home();
    /// @brief Calibrate X (Z/Theta: call calibrateZ / calibrateTheta separately).
    int  calibrate();

    /// @brief Full EIP bring-up: Z- (A015) → X home/cal → X=park →
    ///        Z+ stroke (A014) → return to SAFE_Z ceiling (z_min + margin).
    ///        Preferred over separate home/calibrate when both axes are present.
    bool startEipBringUp();
    bool eipBringUpInProgress() const;

    // Per-axis homing / calibration. X and Z use EIP drive-managed clear-edge
    // when configureDriveManagedLimits() is active. Theta remains a stub.
    void homeX();
    void homeZ();
    void homeTheta();
    int  calibrateX();
    int  calibrateZ();
    int  calibrateTheta();

    /// @brief EIP soft-home: set joint zero to the drive's current absolute PUU
    ///        on X and Z (no limit-switch sweep). After this, move(100) means
    ///        +100 mm from here - not absolute drive PUU for 100 mm.
    void softHomeJointDatum();

    /// @brief Set the periodic-while-busy MOVE log rate (Hz) for all axes.
    ///        0 disables periodic output; START/END events always fire.
    void setAxisLogRateHz(uint32_t hz);

    void requestAbort();
    bool isAbortRequested() const;

    void moveTo(int32_t x, int32_t z, int32_t theta, uint32_t speed);

    GantryError moveTo(const JointConfig& joint,
                       uint32_t speed_mm_per_s        = 50,
                       uint32_t speed_deg_per_s       = 30,
                       uint32_t acceleration_mm_per_s2 = 0,
                       uint32_t deceleration_mm_per_s2 = 0);

    GantryError moveTo(const EndEffectorPose& pose,
                       uint32_t speed_mm_per_s        = 50,
                       uint32_t speed_deg_per_s       = 30,
                       uint32_t acceleration_mm_per_s2 = 0,
                       uint32_t deceleration_mm_per_s2 = 0);

    bool isBusy() const;
    bool isEnabled() const;
    void update();

    // ---------- Gripper ----------
    void grip(bool active);

    // ---------- Position accessors ----------
    int     getXEncoder() const;
    int     getXEncoderRaw() const;
    int32_t getXCommandedPulses() const;
    float   getXCommandedMm() const;
    float   getXEncoderMm() const;
    int     getCurrentZ() const;
    float   getZCommandedMm() const;
    float   getZEncoderMm() const;
    int32_t getZEncoderPulses() const;
    float   getZPulsesPerMm() const;
    int     getCurrentTheta() const;

    // ---------- Diagnostics ----------
    bool isAlarmActive() const;
    bool clearAlarm();
    /// @brief Print X/Z EIP fault/warning summaries (no-op if no codes).
    void logDriveAlarmSummaries() const;
    bool getXDriveAlarmSummary(char* buf, size_t n) const;
    bool getZDriveAlarmSummary(char* buf, size_t n) const;
    void setHomingSpeed(uint32_t speed_pps);

    // ---------- Kinematics ----------
    EndEffectorPose forwardKinematics(const JointConfig& joint) const;
    JointConfig     inverseKinematics(const EndEffectorPose& pose) const;
    JointConfig     getCurrentJointConfig() const;
    JointConfig     getTargetJointConfig() const;
    EndEffectorPose getCurrentEndEffectorPose() const;
    EndEffectorPose getTargetEndEffectorPose() const;

    // ---------- Legacy helpers (kept for console/tests) ----------
    void  setStepsPerRevolution(float steps_per_rev);
    float getStepsPerRevolution() const { return stepsPerRev_; }

    /// @brief X-axis pulses-per-mm (used by console diagnostics).
    float getPulsesPerMm() const;

private:
    std::unique_ptr<GantryLinearAxis> axisX_;
    std::unique_ptr<GantryLinearAxis> axisZ_;
    std::unique_ptr<GantryRotaryAxis> axisTheta_;
    GantryEndEffector endEffector_;

    int  gripperPin_;
    int  xMinPin_;
    int  xMaxPin_;
    GantryLimitSwitch xMinSwitch_;
    GantryLimitSwitch xMaxSwitch_;
    GantryLimitSwitch zMinSwitch_;
    GantryLimitSwitch zMaxSwitch_;

    bool initialized_;
    bool enabled_;
    bool abortRequested_;
    bool homingInProgress_;
    bool calibrationInProgress_;
    bool gripperActive_;

    // Position tracking (mirrored from axis wrappers for cheap access)
    float   currentX_mm_;
    int32_t currentZ_;
    int32_t currentTheta_;
    int32_t targetZ_;
    int32_t targetTheta_;
    int32_t axisLength_;
    int32_t zAxisLength_;

    GantryConfig         config_;
    KinematicParameters  kinematicParams_;
    float                stepsPerRev_;

    // Sequential / path motion state machine. Naming follows +Z = up:
    //   Z_DESCENDING  - Z-alone lowering toward the belt.
    //   Z_RETRACTING  - Z-alone rising toward clearance / higher Z.
    //   X_MOVING      - X-alone traverse (Z held at/above clearance).
    //   XZ_MOVING     - coordinated X+Z Absolute (both endpoints >= clearance).
    enum class MotionState {
        IDLE,
        Z_DESCENDING,
        GRIPPER_ACTUATING,
        Z_RETRACTING,
        X_MOVING,
        XZ_MOVING,
        THETA_MOVING
    };
    MotionState motionState_;
    float       targetX_mm_;
    float       targetZ_mm_;
    float       targetTheta_deg_;
    // Margin above Z−/A015 (mm) for coordinated X+Z / X traverse bottom band.
    // Band ceiling = traverseClearanceZMm() = z_min + this (from Z−/A015).
    float       safeZMarginFromMaxMm_;
    uint32_t    speed_mm_per_s_;
    uint32_t    speed_deg_per_s_;
    uint32_t    acceleration_mm_per_s2_;
    uint32_t    deceleration_mm_per_s2_;
    bool        gripperTargetState_;
    uint32_t    gripperActuateStart_ms_;
    uint32_t    gripperActuateDurationMs_;
    uint32_t    lastXPositionCounts_;
    float       xPulsesPerMmOverride_;
    // Console/joint moves: path to joint target (no gripper). Pose/pick keeps gripper.
    bool        jointDirectMove_;

    // Planned 2-D path Absolute segments (max 3).
    static constexpr size_t kMaxPathSegments = 3;
    Path::PathSegment pathSegments_[kMaxPathSegments]{};
    size_t pathSegmentCount_ = 0;
    size_t pathSegmentIndex_ = 0;
    float pathSegStartX_mm_ = 0.0f;
    float pathSegStartZ_mm_ = 0.0f;
    // PnP: after path to pick completes, run gripper then retract to clearance.
    bool pathDoGripperAfter_ = false;

    // EIP drive-managed precision home/calibrate (clear-edge).
    // X: A014=min / A015=max. Z: A015=min (−Z) / A014=max (+Z).
    using EipLimitPhase = EipLimit::Phase;
    using EipLimitAxisRole = EipLimit::AxisRole;
    EipLimitPhase eipLimitPhase_ = EipLimitPhase::kIdle;
    EipLimitAxisRole eipLimitAxis_ = EipLimitAxisRole::kNone;
    unsigned long eipLimitPhaseStartMs_ = 0;
    // True once Absolute seek/creep has been observed busy (guards preempt race).
    bool eipLimitSawBusy_ = false;

    // Full bring-up SM (Z- datum → enter Z+ band → X home/cal → X park →
    // Z+ stroke → return to band floor). When active, single-axis eipLimitPhase_
    // is not used.
    enum class BringUpPhase : uint8_t {
        kIdle = 0,
        kZMinusSeek,
        kZMinusCreep,
        kZMinusSettle,
        kZEnterBand,
        kXHomeSeek,
        kXHomeCreep,
        kXHomeSettle,
        kXCalSeek,
        kXCalCreep,
        kXCalSettle,
        kXPark,
        kZPlusSeek,
        kZPlusCreep,
        kZPlusSettle,
        kZReturnBand,
    };
    BringUpPhase bringUpPhase_ = BringUpPhase::kIdle;
    float calXParkMm_ = GANTRY_CAL_X_PARK_MM;

    // Helpers
    float   pulsesToMm(int32_t pulses) const;
    int32_t mmToPulses(float mm) const;

    void startSequentialMotion();
    void processSequentialMotion();
    /// Arm X Absolute with explicit path-component profile (mm units).
    bool startXAxisMotion(float target_mm, float speed_mm_s, float accel_mm_s2,
                          float decel_mm_s2);

    bool planPathTo(float x0, float z0, float x1, float z1);
    bool armCurrentPathSegment();
    bool pathSegmentAxesIdle() const;
    /// Current joints within tolerance of the armed segment endpoint.
    bool pathSegmentAtTarget() const;
    void finishPathOrAdvance();
    void failPath(const char* why);
    void startThetaOrIdle();

    uint32_t getHomingSpeed() const;
    bool     moveZAxisTo(float targetZ, float speed, float accel, float decel);
    void     updateAxisPositions();
    void     stopAllMotion();

    GantryLinearAxis* eipLimitActiveAxis();
    const GantryLinearAxis* eipLimitActiveAxis() const;
    bool eipMinWarningActive() const;  // joint min (X:A014, Z:A015)
    bool eipMaxWarningActive() const;  // joint max (X:A015, Z:A014)
    float eipSeekSpeedMmS() const;
    float eipCreepSpeedMmS() const;
    float eipCalSeekSpeedMmS() const;
    bool eipStartMoveDelta(float delta_mm, float speed_mm_s);
    /// Refuse X Absolute unless Z is in the SAFE_Z bottom band.
    bool requireXTraverseInterlock(const char* what);
    void startEipPrecisionHome(EipLimitAxisRole role);
    void startEipPrecisionCalibrate(EipLimitAxisRole role);
    void advanceEipLimitSequence();
    void finishEipHomeOk();
    void finishEipCalOk();
    void failEipLimitSequence(const char* why);

    void advanceEipBringUp();
    void failEipBringUp(const char* why);
    void finishEipBringUpOk();
};

// ============================================================================
// WAYPOINT AND TRAJECTORY PLANNING (unchanged)
// ============================================================================

struct Waypoint {
    EndEffectorPose pose;
    uint32_t speed_mm_per_s;
    uint32_t speed_deg_per_s;
    uint32_t acceleration_mm_per_s2;
    uint32_t deceleration_mm_per_s2;

    Waypoint()
      : speed_mm_per_s(50), speed_deg_per_s(30),
        acceleration_mm_per_s2(0), deceleration_mm_per_s2(0) {}

    Waypoint(const EndEffectorPose& p)
      : pose(p), speed_mm_per_s(50), speed_deg_per_s(30),
        acceleration_mm_per_s2(0), deceleration_mm_per_s2(0) {}
};

template<size_t MAX_WAYPOINTS = 16>
class WaypointQueue {
private:
    Waypoint waypoints_[MAX_WAYPOINTS];
    size_t head_  = 0;
    size_t tail_  = 0;
    size_t count_ = 0;

public:
    bool push(const Waypoint& wp) {
        if (count_ >= MAX_WAYPOINTS) return false;
        waypoints_[tail_] = wp;
        tail_ = (tail_ + 1) % MAX_WAYPOINTS;
        count_++;
        return true;
    }
    bool pop(Waypoint& wp) {
        if (count_ == 0) return false;
        wp = waypoints_[head_];
        head_ = (head_ + 1) % MAX_WAYPOINTS;
        count_--;
        return true;
    }
    size_t size() const { return count_; }
    bool   empty() const { return count_ == 0; }
    bool   full() const { return count_ >= MAX_WAYPOINTS; }
    void   clear() { head_ = 0; tail_ = 0; count_ = 0; }
};

} // namespace Gantry

#endif // GANTRY_H

/**
 * @file Gantry.h
 * @brief Multi-axis gantry control system for ESP32.
 * @version 2.1.0
 *
 * Production motion is EtherNet/IP only (Kinetix 5100 X/Z; HCS01 theta gated
 * by CONFIG_EIP_AXIS_THETA, default on).
 * Application code drives axes exclusively through this Gantry facade.
 *
 * Coordinate convention (firmware-wide, as of 2026-08):
 *   - X: horizontal traverse along the gantry beam (across the conveyor belt).
 *   - Y: along-belt direction; NO gantry actuator. Conveyor downstream is
 *        +Y. Battery positions on the belt carry y values;
 *        the motion layer does not consume them.
 *   - Z: vertical (gantry descent). +Z = down (toward the belt). Joint Z=0
 *        is the sample where A015 deasserts (switch disabled) during creep —
 *        no extra offset. That end is retracted. X Absolute only while Z is
 *        in the SAFE_Z retract/traverse band (interlock, not a path via);
 *        in-band X and Z may run together. Theta may turn only in that same
 *        band. Above the band, Z moves alone.
 *        Z limits: A015=min (retract), A014=max (+Z / belt); X: A014=min,
 *        A015=max. Z Absolute joint sense inverted so − seeks A015.
 *        The vertical actuator was called "Y" in pre-2026-05 firmware - it is
 *        the same hardware, just renamed.
 *   - Theta: rotation about Z (right-handed about +Z).
 *
 * Verified hardware layout:
 *   - X: Allen-Bradley Kinetix 5100 + SCHUNK Beta 100-ZRS belt actuator.
 *   - Z: Allen-Bradley Kinetix 5100 + SCHUNK Beta 80-SRS ballscrew actuator.
 *   - Theta: Bosch Rexroth HCS01 + SCHUNK ERD 04-40-D-H-N (EIP assemblies 101/102).
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
    JointLimits getJointLimits() const { return config_.limits; }
    void setEndEffectorPin(int pin, bool activeHigh = true);
    void setSafeZHeight(float safeHeight_mm);
    /// Absolute joint-Z band ceiling for X traverse / coordinated X+Z
    /// (= z_min + Z− margin).
    float traverseClearanceZMm() const;
    /// True when current joint Z is in the SAFE_Z retract/traverse band
    /// (X Absolute and theta allowed).
    bool zInTraverseBand() const;

    // ---------- Motion ----------
    /// @brief Home X (Z/Theta: call homeZ / homeTheta separately; console `all` sequences).
    void home();
    /// @brief Calibrate X (Z/Theta: call calibrateZ / calibrateTheta separately).
    int  calibrate();

    /// @brief Full EIP bring-up: Z- (A015 clear = Z=0) → X home/cal →
    ///        X=park → Z+ stroke (A014) → return to SAFE_Z ceiling
    ///        (z_min + margin). Preferred over separate home/calibrate when
    ///        both axes are present.
    bool startEipBringUp();
    bool eipBringUpInProgress() const;

    // Per-axis homing / calibration. X and Z: seek min → creep until switch
    // disables → latch that sample as joint 0. Theta: HIPERFACE origin —
    // aligned when |S-0-0051| is small (C0300 at mechanical home); otherwise
    // firmware offsets and shrinks thetalim to remaining drive travel.
    void homeX();
    void homeZ();
    bool homeTheta();
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
                       uint32_t deceleration_mm_per_s2 = 0,
                       uint32_t acceleration_deg_per_s2 = 0,
                       uint32_t deceleration_deg_per_s2 = 0);

    GantryError moveTo(const EndEffectorPose& pose,
                       uint32_t speed_mm_per_s        = 50,
                       uint32_t speed_deg_per_s       = 30,
                       uint32_t acceleration_mm_per_s2 = 0,
                       uint32_t deceleration_mm_per_s2 = 0,
                       uint32_t acceleration_deg_per_s2 = 0,
                       uint32_t deceleration_deg_per_s2 = 0);

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
    float   getCurrentThetaDeg() const;
    float   getThetaDriveAbsDeg() const;
    bool    isThetaDriveOriginAligned() const;
    float   getThetaPulsesPerDeg() const;
    bool    setThetaPuuPerDeg(double puu_per_deg);
    bool    hasThetaAxis() const;
    bool    hasThetaLiveFeedback() const;
    bool    getThetaCipStatus(char* buf, size_t n) const;

    // ---------- Diagnostics ----------
    bool isAlarmActive() const;
    bool clearAlarm();
    /// @brief Print X/Z/theta EIP fault/warning summaries (no-op if no codes).
    void logDriveAlarmSummaries() const;
    bool getXDriveAlarmSummary(char* buf, size_t n) const;
    bool getZDriveAlarmSummary(char* buf, size_t n) const;
    bool getThetaDriveAlarmSummary(char* buf, size_t n) const;
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

    // Sequential / path motion state machine. +Z = down (toward belt):
    //   Z_DESCENDING  - Z-alone increasing joint Z (toward belt / A014).
    //   Z_RETRACTING  - Z-alone decreasing joint Z (toward A015 / retract).
    //   X_MOVING      - X-alone (Z in SAFE_Z band).
    //   XZ_MOVING     - in-band X+Z, or retract with X unlocked once in-band.
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
    // Margin from Z−/A015 (mm) for X traverse / SAFE_Z retract band.
    // Band ceiling = traverseClearanceZMm() = z_min + this.
    float       safeZMarginFromMaxMm_;
    uint32_t    speed_mm_per_s_;
    uint32_t    speed_deg_per_s_;
    uint32_t    acceleration_mm_per_s2_;
    uint32_t    deceleration_mm_per_s2_;
    uint32_t    acceleration_deg_per_s2_;
    uint32_t    deceleration_deg_per_s2_;
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
    bool pathSegXArmed_ = false;
    bool pathSegZArmed_ = false;
    // Theta: wait for 25% of the in-band X+Z segment, then run at window speed.
    bool thetaPending_ = false;
    float thetaPendingSpeedDegS_ = 0.0f;
    size_t thetaWindowSegIndex_ = 0;
    bool thetaHoldsDescent_ = false;
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

    // Full bring-up SM (Z- A015-clear = 0 → X home/cal → X park →
    // Z+ stroke → return to band ceiling). When active, single-axis
    // eipLimitPhase_ is not used.
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
    /// Start deferred path X once Z is in the SAFE_Z band (retract overlap).
    void tryArmDeferredPathX();
    bool pathSegmentAxesIdle() const;
    /// Current joints within tolerance of the armed segment endpoint.
    bool pathSegmentAtTarget() const;
    void finishPathOrAdvance();
    void failPath(const char* why);
    void startThetaOrIdle();
    float thetaAccelDegPerS2() const;
    float thetaDecelDegPerS2() const;
    /// Refuse theta unless Z is in the SAFE_Z retract/traverse band.
    bool requireThetaTraverseInterlock(const char* what);
    void scheduleThetaForPath();
    void tryStartPendingTheta();
    float pathSegProgressFrac() const;
    bool commandThetaMove(float speed_deg_s);

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
    float eipHomeCalAccelMmS2() const;
    bool eipStartMoveDelta(float delta_mm, float speed_mm_s);
    /// Refuse X Absolute unless Z is in the SAFE_Z retract/traverse band.
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

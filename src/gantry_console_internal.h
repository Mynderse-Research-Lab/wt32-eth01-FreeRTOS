/**
 * @file gantry_console_internal.h
 * @brief Declarations shared by the gantry_console_*.cpp files that make up
 *        the test console. Private to src/; include it only from those files.
 */
#pragma once

#include "gantry_test_console.h"

#if CONFIG_GANTRY_SELFTEST
#include "basic_tests.h"
#endif

// Console driver auto-tune (Kinetix Mode 1 / HCS01 C1800). Off for this version —
// re-enable with -DGANTRY_CONSOLE_AUTOTUNE=1 when bring-up needs embedded tuning.
#ifndef GANTRY_CONSOLE_AUTOTUNE
#define GANTRY_CONSOLE_AUTOTUNE 0
#endif

#include "EipScannerTask.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "CipEthernetLink.h"
#include "CipMessageRouter.h"
#include "EipSession.h"
#include "EipSocketW5500.h"
#include "Hcs01ComwsClient.h"
#include "Kinetix5100ParamAccess.h"
#if GANTRY_CONSOLE_AUTOTUNE
#include "Kinetix5100TuningClient.h"
#endif
#include "freertos/task.h"
#include "ethernet_app_config.h"
#include "EthernetLink.h"
#include "gantry_app_constants.h"
#include "axis_drivetrain_params.h"
#include "gantry_console_parse.h"
#include "GantryPathProfile.h"
#include "gpio_expander.h"
#include "gantry_net_console.h"
#include "gantry_ota.h"
#include "pick_scheduler.h"
#include "theta_trace.h"
#include "boot_log_ring.h"
#include "AppConfig.h"
#include "gantry_runtime_params.h"
#include "ConfigSchema.h"
#include "MCP23S17.h"
#include "W5500Diag.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#ifndef MCP_DEBUG_CMDS
#define MCP_DEBUG_CMDS 1
#endif

#include <atomic>
#include <ctype.h>
#include <cstdarg>
#include <cmath>
#include <cstdio>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

extern "C" int getchar(void);

namespace GantryConsoleImpl {

[[maybe_unused]] static const char *TAG = "GantryConsole";

using GantryConsole::AxisToken;
using GantryConsole::LinearUnitMode;
using GantryConsole::applyRangeLimitU32;
using GantryConsole::kMinAccelDegPerS2;
using GantryConsole::kMinAccelMmPerS2;
using GantryConsole::kMinSpeedMmPerS;
using GantryConsole::parseAxisToken;

// Shared constants.
constexpr uint16_t kHcs01CipParamClass = 0x64;

// Shared state (each defined once, in the file named).
// gantry_console_session.cpp
extern std::atomic<bool> g_homeCompletedThisSession;
extern std::atomic<bool> g_calibratedThisSession;
extern std::atomic<bool> g_homeZCompletedThisSession;
extern std::atomic<bool> g_calibratedZThisSession;
extern std::atomic<bool> g_homeThetaThisSession;
extern std::atomic<bool> g_calibratedThetaThisSession;
extern std::atomic<bool> g_thetaHomePending;
extern std::atomic<bool> g_calibrationInProgress;
extern std::atomic<bool> g_testCycleInProgress;
extern uint32_t g_moveSpeedMmPerS;
extern uint32_t g_moveSpeedDegPerS;
extern uint32_t g_moveAccelMmPerS2;
extern uint32_t g_moveDecelMmPerS2;
extern uint32_t g_moveAccelDegPerS2;
extern uint32_t g_moveDecelDegPerS2;
extern bool g_motionProfileRangeLimitEnabled;
extern LinearUnitMode g_linearUnitMode;
extern uint32_t g_livePosFrequencyHz;
extern uint32_t g_lastLiveMotionLogMs;

// Shared functions (defined in the file named).
// gantry_console_session.cpp
const Config::AppConfigData &stored();
bool limitsWired(const GantryTestConsoleConfig *cfg);
bool physicalLimitsAvailable(const GantryTestConsoleConfig *cfg);
uint32_t maxLinearSpeedMmPerS();
uint32_t maxLinearAccelMmPerS2();
uint32_t maxThetaSpeedDegPerS();
uint32_t maxThetaAccelDegPerS2();
uint32_t maxThetaDecelDegPerS2();
const char *getLinearUnitLabel();
float convertMmToSelected(float valueMm);
float convertSelectedToMm(float valueSelected);
void logLiveMotionState(const GantryTestConsoleConfig *cfg);
void resolveThetaHome(const GantryTestConsoleConfig *cfg);
void maybeLatchWorkspaceCalibrated(Gantry::Gantry *gantry);
void startWorkspaceUncalibrated(Gantry::Gantry *gantry);
void printWorkspaceCalibrated(Gantry::Gantry *gantry);
void monitorControlVariableFlips(const GantryTestConsoleConfig *cfg);
// gantry_console_status.cpp
void printStatus(Gantry::Gantry *gantry);
void printPuuInfo(Gantry::Gantry *gantry);
void runPuuCalCommand(Gantry::Gantry *gantry, const char *cmd);
void runPuuSetCommand(Gantry::Gantry *gantry, const char *cmd);
void runThetaLimCommand(Gantry::Gantry *gantry, const char *cmd);
void printLimits(const GantryTestConsoleConfig *cfg);
void printActivePins(const GantryTestConsoleConfig *cfg);
// gantry_console_drive_diag.cpp
void runEthLinkCommand(const GantryTestConsoleConfig *cfg, const char *cmd);
void runAutotuneCommand(const GantryTestConsoleConfig *cfg, const char *cmd);
// gantry_console_cip.cpp
bool parseHcsIdnToInstance(const char *idn, uint16_t &out_inst);
bool readCipAttrRaw(eip::EipSession &session, uint16_t class_id, uint16_t inst_id,
                    uint16_t attr_id, eip::Bytes &out_data, uint8_t *out_status = nullptr);
bool writeCipAttrRaw(eip::EipSession &session, uint16_t class_id, uint16_t inst_id,
                     uint16_t attr_id, const eip::Bytes &data, uint8_t *out_status = nullptr);
bool readHcsIdnCip(eip::EipSession &session, const char *idn, char *out_str, size_t max_len,
                   uint32_t *out_attr3 = nullptr, size_t *out_len = nullptr);
bool readK5100Attr(eip::EipSession &session, uint16_t class_id, uint16_t inst_id,
                   uint16_t attr_id, uint8_t size_bytes, uint32_t &out_val);
// gantry_console_drv.cpp
void runDrvCommand(const GantryTestConsoleConfig *cfg, const char *cmd);
// gantry_console_io.cpp
void runMcpPinModeCommand(const char *cmd);
void runMcpDumpCommand(const char *cmd);
void runMcpRegCommand(const char *cmd);
void runFieldDoutCommand(const char *cmd);
void runFieldDinCommand(const char *cmd);
void runGpioDriveCommand(const char *cmd);
// gantry_console_homing.cpp
void waitGantryIdle(const GantryTestConsoleConfig *cfg, uint32_t timeout_ms);
void runHomeForAxis(const GantryTestConsoleConfig *cfg, AxisToken axis);
void runCalibrateForAxis(const GantryTestConsoleConfig *cfg, AxisToken axis);
// gantry_console_bringup.cpp
bool runEipBringUpSequence(const GantryTestConsoleConfig *cfg);
void dispatchPerAxisCommand(const GantryTestConsoleConfig *cfg, const char *cmd,
                            const char *leading,
                            void (*runFn)(const GantryTestConsoleConfig *,
                                          AxisToken));
// gantry_console_testcycle.cpp
const char *gantryErrorName(Gantry::GantryError err);
bool testCycleMoveLeg(const GantryTestConsoleConfig *cfg, const char *name,
                      float from_x_mm, float from_z_mm, float x_mm, float z_mm,
                      float theta_deg, int expected_segs, uint32_t speed_deg_per_s,
                      uint32_t accel_deg_per_s2, uint32_t decel_deg_per_s2,
                      uint32_t speed_mm_per_s = 0, uint32_t accel_mm_per_s2 = 0,
                      uint32_t decel_mm_per_s2 = 0);
void testCycleTask(void *param);
// gantry_console_thetapath.cpp
void testThetaPathTask(void *param);
// gantry_console_cfg.cpp
bool cmdTokenIs(const char *cmd, const char *word);
bool saveParameters(const GantryTestConsoleConfig *cfg);
void runCfgCommand(const GantryTestConsoleConfig *cfg, const char *cmd);
// gantry_console_commands.cpp
void processCommand(const GantryTestConsoleConfig *cfg, const char *cmd);

}  // namespace GantryConsoleImpl

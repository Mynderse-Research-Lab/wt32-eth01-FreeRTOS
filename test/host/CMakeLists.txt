# Standalone host (native) unit-test project for the WT32-ETH01 gantry firmware.
#
# This project does NOT use ESP-IDF. It compiles only the hardware-independent
# motion logic (kinematics, trajectory) and exercises it with the Unity test
# framework so the core math can be regression-tested on a plain CI runner.
#
# Build + run:
#   cmake -S test/host -B build/host
#   cmake --build build/host
#   ctest --test-dir build/host --output-on-failure
#
# The full firmware still builds with ESP-IDF (idf.py -C idf build); that is a
# separate target and is exercised in CI, not here.

cmake_minimum_required(VERSION 3.16)
project(gantry_host_tests LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)

enable_testing()

# Repo root, two levels up from test/host/.
get_filename_component(REPO_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)

# --- Unity test framework -----------------------------------------------------
# Fetch a pinned release of ThrowTheSwitch/Unity. Cached in the build tree, so
# repeated configures (e.g. from the commit hook) do not re-download.
include(FetchContent)
FetchContent_Declare(
    unity
    GIT_REPOSITORY https://github.com/ThrowTheSwitch/Unity.git
    GIT_TAG        v2.6.0
)
FetchContent_MakeAvailable(unity)

option(GANTRY_HOST_COVERAGE
    "Instrument host tests with gcov and add a gcovr coverage target (no threshold)"
    OFF)
if(GANTRY_HOST_COVERAGE)
    add_compile_options(--coverage -O0 -g)
    add_link_options(--coverage)
endif()

# --- Hardware-independent logic under test ------------------------------------
# Only sources that compile without ESP-IDF / FreeRTOS belong here.
add_library(gantry_logic STATIC
    "${REPO_ROOT}/lib/Gantry/src/kinematics/GantryKinematics.cpp"
    "${REPO_ROOT}/lib/Gantry/src/motion/GantryTrajectory.cpp"
    "${REPO_ROOT}/lib/Gantry/src/GantryConfig.cpp"
)
target_include_directories(gantry_logic PUBLIC
    "${REPO_ROOT}/lib/Gantry/src"
    "${REPO_ROOT}/lib/Gantry/src/motion"
    "${REPO_ROOT}/lib/Gantry/src/homing"
    "${REPO_ROOT}/lib/Gantry/src/kinematics"
    "${REPO_ROOT}/lib/Gantry/src/linear_axis"
    "${REPO_ROOT}/lib/Gantry/src/rotary_axis"
    "${REPO_ROOT}/lib/Gantry/src/io"
    "${REPO_ROOT}/include"
    "${REPO_ROOT}/src/pick_scheduler"
    "${REPO_ROOT}/src/pick_scheduler/intercept"
    "${REPO_ROOT}/src/pick_scheduler/strike"
    "${REPO_ROOT}/src/pick_scheduler/grasp"
    "${REPO_ROOT}/src/pick_scheduler/diag"
)

# --- EtherNet/IP encoding layer under test ------------------------------------
# Pure, freestanding C++17 (no ESP-IDF). Same sources the firmware will link in
# a later phase; here they are regression-tested byte-for-byte on the host.
add_library(ethernet_ip STATIC
    "${REPO_ROOT}/lib/EtherNetIP/src/protocol/EipEncapsulation.cpp"
    "${REPO_ROOT}/lib/EtherNetIP/src/protocol/EipCpf.cpp"
    "${REPO_ROOT}/lib/EtherNetIP/src/cip/CipMessageRouter.cpp"
    "${REPO_ROOT}/lib/EtherNetIP/src/cip/CipEthernetLink.cpp"
    "${REPO_ROOT}/lib/EtherNetIP/src/cip/CipTcpIpInterface.cpp"
    "${REPO_ROOT}/lib/EtherNetIP/src/timing/EipRunTimeTrace.cpp"
    "${REPO_ROOT}/lib/EtherNetIP/src/protocol/EipConnectionManager.cpp"
    "${REPO_ROOT}/lib/EtherNetIP/src/kinetix/Kinetix5100Assembly.cpp"
    "${REPO_ROOT}/lib/EtherNetIP/src/kinetix/Kinetix5100ParamAccess.cpp"
    "${REPO_ROOT}/lib/EtherNetIP/src/hcs01/Hcs01ControlStatus.cpp"
    "${REPO_ROOT}/lib/EtherNetIP/src/hcs01/Hcs01Assembly.cpp"
    "${REPO_ROOT}/lib/EtherNetIP/src/hcs01/Hcs01ComwsClient.cpp"
    "${REPO_ROOT}/lib/EtherNetIP/src/session/EipSession.cpp"
    "${REPO_ROOT}/lib/EtherNetIP/src/scanner/EipIoConnection.cpp"
    "${REPO_ROOT}/lib/EtherNetIP/src/scanner/EipScanner.cpp"
    "${REPO_ROOT}/lib/EtherNetIP/src/task/EipMultiScanner.cpp"
    "${REPO_ROOT}/lib/EtherNetIP/src/task/EipMultiScannerConnect.cpp"
    "${REPO_ROOT}/lib/EtherNetIP/src/scanner/EipProcessImage.cpp"
    "${REPO_ROOT}/lib/EtherNetIP/src/timing/EipClass1TimingStats.cpp"
    "${REPO_ROOT}/lib/EtherNetIP/src/timing/EipReliabilityStats.cpp"
)
target_include_directories(ethernet_ip PUBLIC
    "${REPO_ROOT}/lib/EtherNetIP/src"
    "${REPO_ROOT}/lib/EtherNetIP/src/cip"
    "${REPO_ROOT}/lib/EtherNetIP/src/protocol"
    "${REPO_ROOT}/lib/EtherNetIP/src/session"
    "${REPO_ROOT}/lib/EtherNetIP/src/scanner"
    "${REPO_ROOT}/lib/EtherNetIP/src/task"
    "${REPO_ROOT}/lib/EtherNetIP/src/timing"
    "${REPO_ROOT}/lib/EtherNetIP/src/kinetix"
    "${REPO_ROOT}/lib/EtherNetIP/src/hcs01"
)
# Reliability + timing helpers are header/cpp already in ethernet_ip SRCS via EipReliabilityStats

# --- AppConfig layer under test ------------------------------------------------
add_library(app_config STATIC
    "${REPO_ROOT}/lib/AppConfig/src/AppConfig.cpp"
    "${REPO_ROOT}/lib/AppConfig/src/AppConfigAccess.cpp"
    "${REPO_ROOT}/lib/AppConfig/src/schema/AppConfigDefaultsCell.cpp"
    "${REPO_ROOT}/lib/AppConfig/src/schema/AppConfigDefaultsMotion.cpp"
    "${REPO_ROOT}/lib/AppConfig/src/AppConfigParamIo.cpp"
    "${REPO_ROOT}/lib/AppConfig/src/AppConfigText.cpp"
    "${REPO_ROOT}/lib/AppConfig/src/schema/ConfigSchema.cpp"
)
target_include_directories(app_config PUBLIC
    "${REPO_ROOT}/lib/AppConfig/src"
    "${REPO_ROOT}/lib/AppConfig/src/schema"
    "${REPO_ROOT}/lib/AppConfig/src/storage"
    "${REPO_ROOT}/lib/Gantry/src"
    "${REPO_ROOT}/lib/Gantry/src/motion"
    "${REPO_ROOT}/lib/Gantry/src/homing"
    "${REPO_ROOT}/lib/Gantry/src/kinematics"
    "${REPO_ROOT}/lib/Gantry/src/linear_axis"
    "${REPO_ROOT}/lib/Gantry/src/rotary_axis"
    "${REPO_ROOT}/lib/Gantry/src/io"
    "${REPO_ROOT}/include"
    "${REPO_ROOT}/src/pick_scheduler"
    "${REPO_ROOT}/src/pick_scheduler/intercept"
    "${REPO_ROOT}/src/pick_scheduler/strike"
    "${REPO_ROOT}/src/pick_scheduler/grasp"
    "${REPO_ROOT}/src/pick_scheduler/diag"
)

# --- Test executables ---------------------------------------------------------
# Each test file is its own Unity runner registered with CTest.
function(add_host_test name)
    add_executable(${name} "${name}.cpp")
    target_link_libraries(${name} PRIVATE gantry_logic unity)
    add_test(NAME ${name} COMMAND ${name})
endfunction()

add_host_test(test_kinematics)
add_host_test(test_trajectory)
# thtrace summary of one HCS01 move (header-only, reuses TrajectoryPlanner)
add_host_test(test_theta_trace_analysis)
target_include_directories(test_theta_trace_analysis PRIVATE
    "${REPO_ROOT}/lib/ThetaTrace/src")

# EtherNet/IP encoding tests link the ethernet_ip lib instead of gantry_logic.
add_executable(test_eip_encoding "test_eip_encoding.cpp")
target_link_libraries(test_eip_encoding PRIVATE ethernet_ip unity)
add_test(NAME test_eip_encoding COMMAND test_eip_encoding)

add_executable(test_cip_ethernet_link "test_cip_ethernet_link.cpp")
target_link_libraries(test_cip_ethernet_link PRIVATE ethernet_ip unity)
add_test(NAME test_cip_ethernet_link COMMAND test_cip_ethernet_link)

add_executable(test_eip_run_time_trace "test_eip_run_time_trace.cpp")
target_link_libraries(test_eip_run_time_trace PRIVATE ethernet_ip unity)
add_test(NAME test_eip_run_time_trace COMMAND test_eip_run_time_trace)

add_executable(test_k5100_param_access "test_k5100_param_access.cpp")
target_link_libraries(test_k5100_param_access PRIVATE ethernet_ip unity)
add_test(NAME test_k5100_param_access COMMAND test_k5100_param_access)

add_executable(test_hcs01_assembly "test_hcs01_assembly.cpp")
target_link_libraries(test_hcs01_assembly PRIVATE ethernet_ip unity)
add_test(NAME test_hcs01_assembly COMMAND test_hcs01_assembly)

add_executable(test_hcs01_comws_client "test_hcs01_comws_client.cpp")
target_link_libraries(test_hcs01_comws_client PRIVATE ethernet_ip unity)
add_test(NAME test_hcs01_comws_client COMMAND test_hcs01_comws_client)

add_executable(test_eip_transport
    "test_eip_transport.cpp"
    "test_eip_transport_session.cpp"
    "test_eip_transport_close.cpp"
    "test_eip_transport_drain.cpp"
    "test_eip_transport_reopen.cpp"
    "test_eip_transport_nop.cpp"
)
target_link_libraries(test_eip_transport PRIVATE ethernet_ip unity)
add_test(NAME test_eip_transport COMMAND test_eip_transport)

add_executable(test_app_config
    "test_app_config.cpp"
    "test_app_config_values.cpp"
    "test_app_config_store.cpp"
    "test_app_config_schema.cpp"
)
target_link_libraries(test_app_config PRIVATE app_config unity)
add_test(NAME test_app_config COMMAND test_app_config)

add_executable(test_eip_axis
    "test_eip_axis.cpp"
    "test_eip_axis_linear.cpp"
    "test_eip_axis_alarm.cpp"
    "test_eip_axis_limits.cpp"
    "test_eip_axis_rotary.cpp"
    "test_eip_axis_units.cpp"
    "${REPO_ROOT}/lib/Gantry/src/linear_axis/GantryEipLinearAxis.cpp"
    "${REPO_ROOT}/lib/Gantry/src/linear_axis/GantryEipLinearAxisPhases.cpp"
    "${REPO_ROOT}/lib/Gantry/src/linear_axis/GantryEipLinearAxisAlarm.cpp"
    "${REPO_ROOT}/lib/Gantry/src/rotary_axis/GantryEipRotaryAxis.cpp"
    "${REPO_ROOT}/lib/Gantry/src/rotary_axis/GantryEipRotaryAxisUpdate.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/stubs/host_test_clock.cpp"
)
target_include_directories(test_eip_axis PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/stubs"
    "${REPO_ROOT}/lib/Gantry/src"
    "${REPO_ROOT}/lib/Gantry/src/motion"
    "${REPO_ROOT}/lib/Gantry/src/homing"
    "${REPO_ROOT}/lib/Gantry/src/kinematics"
    "${REPO_ROOT}/lib/Gantry/src/linear_axis"
    "${REPO_ROOT}/lib/Gantry/src/rotary_axis"
    "${REPO_ROOT}/lib/Gantry/src/io"
    "${REPO_ROOT}/lib/EtherNetIP/src"
    "${REPO_ROOT}/lib/EtherNetIP/src/cip"
    "${REPO_ROOT}/lib/EtherNetIP/src/protocol"
    "${REPO_ROOT}/lib/EtherNetIP/src/session"
    "${REPO_ROOT}/lib/EtherNetIP/src/scanner"
    "${REPO_ROOT}/lib/EtherNetIP/src/task"
    "${REPO_ROOT}/lib/EtherNetIP/src/timing"
    "${REPO_ROOT}/lib/EtherNetIP/src/kinetix"
    "${REPO_ROOT}/lib/EtherNetIP/src/hcs01"
    "${REPO_ROOT}/include"
    "${REPO_ROOT}/src/pick_scheduler"
    "${REPO_ROOT}/src/pick_scheduler/intercept"
    "${REPO_ROOT}/src/pick_scheduler/strike"
    "${REPO_ROOT}/src/pick_scheduler/grasp"
    "${REPO_ROOT}/src/pick_scheduler/diag"
)
target_link_libraries(test_eip_axis PRIVATE ethernet_ip unity)
add_test(NAME test_eip_axis COMMAND test_eip_axis)

# EIP home/cal phase helpers (header-only policy shared with the Gantry class)
add_executable(test_eip_limit_sequence "test_eip_limit_sequence.cpp")
target_include_directories(test_eip_limit_sequence PRIVATE
    "${REPO_ROOT}/lib/Gantry/src"
    "${REPO_ROOT}/lib/Gantry/src/motion"
    "${REPO_ROOT}/lib/Gantry/src/homing"
    "${REPO_ROOT}/lib/Gantry/src/kinematics"
    "${REPO_ROOT}/lib/Gantry/src/linear_axis"
    "${REPO_ROOT}/lib/Gantry/src/rotary_axis"
    "${REPO_ROOT}/lib/Gantry/src/io"
)
target_link_libraries(test_eip_limit_sequence PRIVATE unity)
add_test(NAME test_eip_limit_sequence COMMAND test_eip_limit_sequence)

# 2-D X-Z path profile (header-only decompose + segment planner)
add_executable(test_path_profile "test_path_profile.cpp")
target_include_directories(test_path_profile PRIVATE
    "${REPO_ROOT}/lib/Gantry/src"
    "${REPO_ROOT}/lib/Gantry/src/motion"
    "${REPO_ROOT}/lib/Gantry/src/homing"
    "${REPO_ROOT}/lib/Gantry/src/kinematics"
    "${REPO_ROOT}/lib/Gantry/src/linear_axis"
    "${REPO_ROOT}/lib/Gantry/src/rotary_axis"
    "${REPO_ROOT}/lib/Gantry/src/io"
)
target_link_libraries(test_path_profile PRIVATE unity)
add_test(NAME test_path_profile COMMAND test_path_profile)

# W5500 deferred log for real-time paths (G-B13)
add_executable(test_w5500_deferred_log
    "test_w5500_deferred_log.cpp"
    "${REPO_ROOT}/lib/W5500/src/diag/W5500DeferredLog.cpp"
)
target_include_directories(test_w5500_deferred_log PRIVATE
    "${REPO_ROOT}/lib/W5500/src"
    "${REPO_ROOT}/lib/W5500/src/hal"
    "${REPO_ROOT}/lib/W5500/src/socket"
    "${REPO_ROOT}/lib/W5500/src/diag")
target_link_libraries(test_w5500_deferred_log PRIVATE unity)
add_test(NAME test_w5500_deferred_log COMMAND test_w5500_deferred_log)

# W5500 SPI frame encoding test (pure C++ constexpr logic, no ESP-IDF)
add_executable(test_w5500_spi "test_w5500_spi.cpp")
target_include_directories(test_w5500_spi PRIVATE
    "${REPO_ROOT}/lib/W5500/src"
    "${REPO_ROOT}/lib/W5500/src/hal"
    "${REPO_ROOT}/lib/W5500/src/socket"
    "${REPO_ROOT}/lib/W5500/src/diag"
)
target_link_libraries(test_w5500_spi PRIVATE unity)
add_test(NAME test_w5500_spi COMMAND test_w5500_spi)

# W5500 Class 1 UDP send fast-path (dest cache / transaction diet)
add_executable(test_w5500_socket_fastpath
    "test_w5500_socket_fastpath.cpp"
    "test_w5500_socket_fastpath_send.cpp"
    "test_w5500_socket_fastpath_recv.cpp"
    "test_w5500_socket_fastpath_tcp.cpp"
    "${REPO_ROOT}/lib/W5500/src/socket/W5500Socket.cpp"
    "${REPO_ROOT}/lib/W5500/src/socket/W5500SocketTcp.cpp"
    "${REPO_ROOT}/lib/W5500/src/diag/W5500Diag.cpp"
    "${REPO_ROOT}/lib/W5500/src/hal/W5500BusMutex.cpp"
)
target_include_directories(test_w5500_socket_fastpath PRIVATE
    "${REPO_ROOT}/lib/W5500/src"
    "${REPO_ROOT}/lib/W5500/src/hal"
    "${REPO_ROOT}/lib/W5500/src/socket"
    "${REPO_ROOT}/lib/W5500/src/diag"
)
target_link_libraries(test_w5500_socket_fastpath PRIVATE unity)
add_test(NAME test_w5500_socket_fastpath COMMAND test_w5500_socket_fastpath)

# W5500 dropout diagnostics (failure snapshot + SPI read-back check)
add_executable(test_w5500_diag
    "test_w5500_diag.cpp"
    "${REPO_ROOT}/lib/W5500/src/diag/W5500Diag.cpp"
    "${REPO_ROOT}/lib/W5500/src/hal/W5500BusMutex.cpp"
)
target_include_directories(test_w5500_diag PRIVATE
    "${REPO_ROOT}/lib/W5500/src"
    "${REPO_ROOT}/lib/W5500/src/hal"
    "${REPO_ROOT}/lib/W5500/src/socket"
    "${REPO_ROOT}/lib/W5500/src/diag"
)
target_link_libraries(test_w5500_diag PRIVATE unity)
add_test(NAME test_w5500_diag COMMAND test_w5500_diag)

# W5500 poll helper + soft-miss gate + reliability counters
add_executable(test_w5500_poll "test_w5500_poll.cpp")
target_include_directories(test_w5500_poll PRIVATE
    "${REPO_ROOT}/lib/W5500/src"
    "${REPO_ROOT}/lib/W5500/src/hal"
    "${REPO_ROOT}/lib/W5500/src/socket"
    "${REPO_ROOT}/lib/W5500/src/diag"
    "${REPO_ROOT}/lib/EtherNetIP/src"
    "${REPO_ROOT}/lib/EtherNetIP/src/cip"
    "${REPO_ROOT}/lib/EtherNetIP/src/protocol"
    "${REPO_ROOT}/lib/EtherNetIP/src/session"
    "${REPO_ROOT}/lib/EtherNetIP/src/scanner"
    "${REPO_ROOT}/lib/EtherNetIP/src/task"
    "${REPO_ROOT}/lib/EtherNetIP/src/timing"
    "${REPO_ROOT}/lib/EtherNetIP/src/kinetix"
    "${REPO_ROOT}/lib/EtherNetIP/src/hcs01"
)
target_link_libraries(test_w5500_poll PRIVATE ethernet_ip unity)
add_test(NAME test_w5500_poll COMMAND test_w5500_poll)

# --- Gantry orchestration (Gantry.cpp + fakes, ESP-IDF stubs) ---------------
set(GANTRY_ORCH_SRCS
    "${REPO_ROOT}/lib/Gantry/src/Gantry.cpp"
    "${REPO_ROOT}/lib/Gantry/src/GantryAxisState.cpp"
    "${REPO_ROOT}/lib/Gantry/src/homing/GantryEipBringUp.cpp"
    "${REPO_ROOT}/lib/Gantry/src/homing/GantryEipHomeCal.cpp"
    "${REPO_ROOT}/lib/Gantry/src/homing/GantryHoming.cpp"
    "${REPO_ROOT}/lib/Gantry/src/motion/GantryMotion.cpp"
    "${REPO_ROOT}/lib/Gantry/src/motion/GantryPath.cpp"
    "${REPO_ROOT}/lib/Gantry/src/motion/GantryPathTheta.cpp"
    "${REPO_ROOT}/lib/Gantry/src/motion/GantrySequential.cpp"
    "${REPO_ROOT}/lib/Gantry/src/io/GantryLimitSwitch.cpp"
    "${REPO_ROOT}/lib/Gantry/src/io/GantryEndEffector.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/stubs/host_test_clock.cpp"
)
set(GANTRY_ORCH_INCLUDES
    "${CMAKE_CURRENT_LIST_DIR}/stubs"
    "${CMAKE_CURRENT_LIST_DIR}/fakes"
    "${REPO_ROOT}/lib/Gantry/src"
    "${REPO_ROOT}/lib/Gantry/src/motion"
    "${REPO_ROOT}/lib/Gantry/src/homing"
    "${REPO_ROOT}/lib/Gantry/src/kinematics"
    "${REPO_ROOT}/lib/Gantry/src/linear_axis"
    "${REPO_ROOT}/lib/Gantry/src/rotary_axis"
    "${REPO_ROOT}/lib/Gantry/src/io"
    "${REPO_ROOT}/include"
    "${REPO_ROOT}/src/pick_scheduler"
    "${REPO_ROOT}/src/pick_scheduler/intercept"
    "${REPO_ROOT}/src/pick_scheduler/strike"
    "${REPO_ROOT}/src/pick_scheduler/grasp"
    "${REPO_ROOT}/src/pick_scheduler/diag"
)

function(add_gantry_orch_lib name sequential)
    add_library(${name} STATIC ${GANTRY_ORCH_SRCS})
    target_include_directories(${name} PUBLIC ${GANTRY_ORCH_INCLUDES})
    target_compile_definitions(${name} PUBLIC
        CONFIG_GANTRY_THETA_SEQUENTIAL=${sequential}
    )
    target_link_libraries(${name} PUBLIC gantry_logic)
endfunction()

add_gantry_orch_lib(gantry_orch 0)
add_gantry_orch_lib(gantry_orch_seq 1)

add_executable(test_gantry_orchestration
    "test_gantry_orchestration.cpp"
    "test_gantry_orchestration_move.cpp"
    "test_gantry_orchestration_bringup.cpp"
    "test_gantry_orchestration_fault.cpp"
)
target_link_libraries(test_gantry_orchestration PRIVATE gantry_orch unity)
add_test(NAME test_gantry_orchestration COMMAND test_gantry_orchestration)

add_executable(test_gantry_orchestration_seq
    "test_gantry_orchestration.cpp"
    "test_gantry_orchestration_move.cpp"
    "test_gantry_orchestration_bringup.cpp"
    "test_gantry_orchestration_fault.cpp"
)
target_link_libraries(test_gantry_orchestration_seq PRIVATE gantry_orch_seq unity)
add_test(NAME test_gantry_orchestration_seq COMMAND test_gantry_orchestration_seq)

# Console parse (header-only, lives in src/)
add_executable(test_console_parse "test_console_parse.cpp")
target_include_directories(test_console_parse PRIVATE
    "${REPO_ROOT}/src/main"
    "${REPO_ROOT}/src/pick_scheduler"
    "${REPO_ROOT}/src/pick_scheduler/intercept"
    "${REPO_ROOT}/src/pick_scheduler/strike"
    "${REPO_ROOT}/src/pick_scheduler/grasp"
    "${REPO_ROOT}/src/pick_scheduler/diag"
    "${REPO_ROOT}/src/console"
    "${REPO_ROOT}/src/console/net"
    "${REPO_ROOT}/src/console/motion"
    "${REPO_ROOT}/src/console/drives"
    "${REPO_ROOT}/src/console/system")
target_link_libraries(test_console_parse PRIVATE unity)
add_test(NAME test_console_parse COMMAND test_console_parse)

add_executable(test_pick_intercept
    "test_pick_intercept.cpp"
    "test_pick_intercept_belt.cpp"
    "test_pick_intercept_geometry.cpp"
    "test_pick_intercept_budget.cpp"
    "test_pick_intercept_tracker.cpp"
)
target_include_directories(test_pick_intercept PRIVATE
    "${REPO_ROOT}/include"
    "${REPO_ROOT}/src/pick_scheduler"
    "${REPO_ROOT}/src/pick_scheduler/intercept"
    "${REPO_ROOT}/src/pick_scheduler/strike"
    "${REPO_ROOT}/src/pick_scheduler/grasp"
    "${REPO_ROOT}/src/pick_scheduler/diag"
    "${REPO_ROOT}/lib/Gantry/src"
    "${REPO_ROOT}/lib/Gantry/src/motion"
    "${REPO_ROOT}/lib/Gantry/src/homing"
    "${REPO_ROOT}/lib/Gantry/src/kinematics"
    "${REPO_ROOT}/lib/Gantry/src/linear_axis"
    "${REPO_ROOT}/lib/Gantry/src/rotary_axis"
    "${REPO_ROOT}/lib/Gantry/src/io")
target_link_libraries(test_pick_intercept PRIVATE unity)
add_test(NAME test_pick_intercept COMMAND test_pick_intercept)

add_executable(test_coordinated_grasp "test_coordinated_grasp.cpp")
target_include_directories(test_coordinated_grasp PRIVATE
    "${REPO_ROOT}/include"
    "${REPO_ROOT}/src/pick_scheduler"
    "${REPO_ROOT}/src/pick_scheduler/intercept"
    "${REPO_ROOT}/src/pick_scheduler/strike"
    "${REPO_ROOT}/src/pick_scheduler/grasp"
    "${REPO_ROOT}/src/pick_scheduler/diag"
    "${REPO_ROOT}/lib/Gantry/src"
    "${REPO_ROOT}/lib/Gantry/src/motion"
    "${REPO_ROOT}/lib/Gantry/src/homing"
    "${REPO_ROOT}/lib/Gantry/src/kinematics"
    "${REPO_ROOT}/lib/Gantry/src/linear_axis"
    "${REPO_ROOT}/lib/Gantry/src/rotary_axis"
    "${REPO_ROOT}/lib/Gantry/src/io")
target_compile_definitions(test_coordinated_grasp PRIVATE
    COORDINATED_JSON="${REPO_ROOT}/test/host/data/coordinated_grasp_reference.json")
target_link_libraries(test_coordinated_grasp PRIVATE unity)
add_test(NAME test_coordinated_grasp COMMAND test_coordinated_grasp)

# Limit-switch debounce / pin encoding + end-effector polarity
add_executable(test_limit_switch_policy "test_limit_switch_policy.cpp")
target_include_directories(test_limit_switch_policy PRIVATE
    "${REPO_ROOT}/lib/Gantry/src"
    "${REPO_ROOT}/lib/Gantry/src/motion"
    "${REPO_ROOT}/lib/Gantry/src/homing"
    "${REPO_ROOT}/lib/Gantry/src/kinematics"
    "${REPO_ROOT}/lib/Gantry/src/linear_axis"
    "${REPO_ROOT}/lib/Gantry/src/rotary_axis"
    "${REPO_ROOT}/lib/Gantry/src/io"
    "${REPO_ROOT}/include"
    "${REPO_ROOT}/src/pick_scheduler"
    "${REPO_ROOT}/src/pick_scheduler/intercept"
    "${REPO_ROOT}/src/pick_scheduler/strike"
    "${REPO_ROOT}/src/pick_scheduler/grasp"
    "${REPO_ROOT}/src/pick_scheduler/diag"
)
target_link_libraries(test_limit_switch_policy PRIVATE unity)
add_test(NAME test_limit_switch_policy COMMAND test_limit_switch_policy)

# ScannerConfig builders + W5500 recover backoff
add_executable(test_eip_scanner_policy "test_eip_scanner_policy.cpp")
target_include_directories(test_eip_scanner_policy PRIVATE
    "${REPO_ROOT}/lib/EtherNetIP/src"
    "${REPO_ROOT}/lib/EtherNetIP/src/cip"
    "${REPO_ROOT}/lib/EtherNetIP/src/protocol"
    "${REPO_ROOT}/lib/EtherNetIP/src/session"
    "${REPO_ROOT}/lib/EtherNetIP/src/scanner"
    "${REPO_ROOT}/lib/EtherNetIP/src/task"
    "${REPO_ROOT}/lib/EtherNetIP/src/timing"
    "${REPO_ROOT}/lib/EtherNetIP/src/kinetix"
    "${REPO_ROOT}/lib/EtherNetIP/src/hcs01"
)
target_link_libraries(test_eip_scanner_policy PRIVATE ethernet_ip unity)
add_test(NAME test_eip_scanner_policy COMMAND test_eip_scanner_policy)

# Spi3Bus Class 1 critical counter (header-only)
add_executable(test_spi3_critical "test_spi3_critical.cpp")
target_include_directories(test_spi3_critical PRIVATE
    "${REPO_ROOT}/lib/Spi3Bus/src"
)
target_link_libraries(test_spi3_critical PRIVATE unity)
add_test(NAME test_spi3_critical COMMAND test_spi3_critical)

# EipSocketW5500 over the same mock HAL as the W5500 socket fast-path tests
add_executable(test_eip_socket_w5500
    "test_eip_socket_w5500.cpp"
    "${REPO_ROOT}/lib/EtherNetIP/src/session/EipSocketW5500.cpp"
    "${REPO_ROOT}/lib/W5500/src/socket/W5500Socket.cpp"
    "${REPO_ROOT}/lib/W5500/src/socket/W5500SocketTcp.cpp"
    "${REPO_ROOT}/lib/W5500/src/diag/W5500Diag.cpp"
    "${REPO_ROOT}/lib/W5500/src/hal/W5500BusMutex.cpp"
)
target_include_directories(test_eip_socket_w5500 PRIVATE
    "${REPO_ROOT}/lib/EtherNetIP/src"
    "${REPO_ROOT}/lib/EtherNetIP/src/cip"
    "${REPO_ROOT}/lib/EtherNetIP/src/protocol"
    "${REPO_ROOT}/lib/EtherNetIP/src/session"
    "${REPO_ROOT}/lib/EtherNetIP/src/scanner"
    "${REPO_ROOT}/lib/EtherNetIP/src/task"
    "${REPO_ROOT}/lib/EtherNetIP/src/timing"
    "${REPO_ROOT}/lib/EtherNetIP/src/kinetix"
    "${REPO_ROOT}/lib/EtherNetIP/src/hcs01"
    "${REPO_ROOT}/lib/W5500/src"
    "${REPO_ROOT}/lib/W5500/src/hal"
    "${REPO_ROOT}/lib/W5500/src/socket"
    "${REPO_ROOT}/lib/W5500/src/diag"
)
target_link_libraries(test_eip_socket_w5500 PRIVATE ethernet_ip unity)
add_test(NAME test_eip_socket_w5500 COMMAND test_eip_socket_w5500)

# Derived drivetrain macros (pulses/mm, Z critical-RPM cap, default limits)
add_executable(test_drivetrain_params "test_drivetrain_params.cpp")
target_include_directories(test_drivetrain_params PRIVATE
    "${REPO_ROOT}/include"
    "${REPO_ROOT}/src/pick_scheduler"
    "${REPO_ROOT}/src/pick_scheduler/intercept"
    "${REPO_ROOT}/src/pick_scheduler/strike"
    "${REPO_ROOT}/src/pick_scheduler/grasp"
    "${REPO_ROOT}/src/pick_scheduler/diag"
)
target_link_libraries(test_drivetrain_params PRIVATE unity)
add_test(NAME test_drivetrain_params COMMAND test_drivetrain_params)

# CellNet Layer-2 framing and node dispatch tests
add_executable(test_cell_net_l2
    "test_cell_net_l2.cpp"
    "test_cell_net_l2_framing.cpp"
    "test_cell_net_l2_node.cpp"
    "test_cell_net_l2_ota.cpp"
    "${REPO_ROOT}/lib/CellNetL2/src/CellNetL2Framing.cpp"
    "${REPO_ROOT}/lib/CellNetL2/src/CellNetL2.cpp"
)
target_include_directories(test_cell_net_l2 PRIVATE
    "${REPO_ROOT}/lib/CellNetL2/src"
    "${REPO_ROOT}/include"
    "${REPO_ROOT}/src/pick_scheduler"
    "${REPO_ROOT}/src/pick_scheduler/intercept"
    "${REPO_ROOT}/src/pick_scheduler/strike"
    "${REPO_ROOT}/src/pick_scheduler/grasp"
    "${REPO_ROOT}/src/pick_scheduler/diag"
)
target_link_libraries(test_cell_net_l2 PRIVATE unity)
add_test(NAME test_cell_net_l2 COMMAND test_cell_net_l2)

add_executable(test_cellnet_ota_receiver
    "test_cellnet_ota_receiver.cpp"
    "${REPO_ROOT}/lib/CellNetL2/src/CellNetOtaReceiver.cpp"
)
target_include_directories(test_cellnet_ota_receiver PRIVATE "${REPO_ROOT}/lib/CellNetL2/src")
target_link_libraries(test_cellnet_ota_receiver PRIVATE unity)
add_test(NAME test_cellnet_ota_receiver COMMAND test_cellnet_ota_receiver)


# Optional gcovr report: cmake -S test/host -B build/host -DGANTRY_HOST_COVERAGE=ON
# then cmake --build build/host --target coverage
# Invoke ctest as its own COMMAND so paths with spaces (Program Files) are not
# stuffed into a -DVAR="..." argument that mingw32-make then cannot exec.
if(GANTRY_HOST_COVERAGE)
    get_filename_component(_cxx_bin_dir "${CMAKE_CXX_COMPILER}" DIRECTORY)
    find_program(GCOV_EXECUTABLE gcov HINTS "${_cxx_bin_dir}" REQUIRED)
    add_custom_target(coverage
        COMMAND ${CMAKE_COMMAND} --build "${CMAKE_BINARY_DIR}"
        COMMAND ${CMAKE_CTEST_COMMAND} --test-dir "${CMAKE_BINARY_DIR}" --output-on-failure
        COMMAND ${CMAKE_COMMAND}
            -DREPO_ROOT=${REPO_ROOT}
            -DBUILD_DIR=${CMAKE_BINARY_DIR}
            -DGCOV_EXECUTABLE=${GCOV_EXECUTABLE}
            -P ${CMAKE_CURRENT_LIST_DIR}/run_coverage.cmake
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        COMMENT "Run host tests and emit gcovr HTML under coverage/index.html"
        VERBATIM
    )
endif()



# Simulation strike planner port (strike_plan.h) against gantry_twin.py values
add_executable(test_strike_plan "test_strike_plan.cpp")
target_include_directories(test_strike_plan PRIVATE
    "${REPO_ROOT}/include"
    "${REPO_ROOT}/src/pick_scheduler"
    "${REPO_ROOT}/src/pick_scheduler/intercept"
    "${REPO_ROOT}/src/pick_scheduler/strike"
    "${REPO_ROOT}/src/pick_scheduler/grasp"
    "${REPO_ROOT}/src/pick_scheduler/diag")
target_link_libraries(test_strike_plan PRIVATE unity)
add_test(NAME test_strike_plan COMMAND test_strike_plan)

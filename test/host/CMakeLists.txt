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

# --- Hardware-independent logic under test ------------------------------------
# Only sources that compile without ESP-IDF / FreeRTOS belong here.
add_library(gantry_logic STATIC
    "${REPO_ROOT}/lib/Gantry/src/GantryKinematics.cpp"
    "${REPO_ROOT}/lib/Gantry/src/GantryTrajectory.cpp"
    "${REPO_ROOT}/lib/Gantry/src/GantryConfig.cpp"
)
target_include_directories(gantry_logic PUBLIC
    "${REPO_ROOT}/lib/Gantry/src"
    "${REPO_ROOT}/include"
)

# --- EtherNet/IP encoding layer under test ------------------------------------
# Pure, freestanding C++17 (no ESP-IDF). Same sources the firmware will link in
# a later phase; here they are regression-tested byte-for-byte on the host.
add_library(ethernet_ip STATIC
    "${REPO_ROOT}/lib/EtherNetIP/src/EipEncapsulation.cpp"
    "${REPO_ROOT}/lib/EtherNetIP/src/EipCpf.cpp"
    "${REPO_ROOT}/lib/EtherNetIP/src/CipMessageRouter.cpp"
    "${REPO_ROOT}/lib/EtherNetIP/src/EipConnectionManager.cpp"
    "${REPO_ROOT}/lib/EtherNetIP/src/Kinetix5100Assembly.cpp"
    "${REPO_ROOT}/lib/EtherNetIP/src/Hcs01ControlStatus.cpp"
    "${REPO_ROOT}/lib/EtherNetIP/src/Hcs01Assembly.cpp"
    "${REPO_ROOT}/lib/EtherNetIP/src/EipSession.cpp"
    "${REPO_ROOT}/lib/EtherNetIP/src/EipIoConnection.cpp"
    "${REPO_ROOT}/lib/EtherNetIP/src/EipScanner.cpp"
    "${REPO_ROOT}/lib/EtherNetIP/src/EipMultiScanner.cpp"
    "${REPO_ROOT}/lib/EtherNetIP/src/EipProcessImage.cpp"
)
target_include_directories(ethernet_ip PUBLIC
    "${REPO_ROOT}/lib/EtherNetIP/src"
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

# EtherNet/IP encoding tests link the ethernet_ip lib instead of gantry_logic.
add_executable(test_eip_encoding "test_eip_encoding.cpp")
target_link_libraries(test_eip_encoding PRIVATE ethernet_ip unity)
add_test(NAME test_eip_encoding COMMAND test_eip_encoding)

add_executable(test_hcs01_assembly "test_hcs01_assembly.cpp")
target_link_libraries(test_hcs01_assembly PRIVATE ethernet_ip unity)
add_test(NAME test_hcs01_assembly COMMAND test_hcs01_assembly)

add_executable(test_eip_transport "test_eip_transport.cpp")
target_link_libraries(test_eip_transport PRIVATE ethernet_ip unity)
add_test(NAME test_eip_transport COMMAND test_eip_transport)

add_executable(test_eip_axis "test_eip_axis.cpp"
    "${REPO_ROOT}/lib/Gantry/src/GantryEipLinearAxis.cpp"
    "${REPO_ROOT}/lib/Gantry/src/GantryEipRotaryAxis.cpp"
)
target_include_directories(test_eip_axis PRIVATE
    "${REPO_ROOT}/lib/Gantry/src"
    "${REPO_ROOT}/lib/EtherNetIP/src"
    "${REPO_ROOT}/include"
)
target_link_libraries(test_eip_axis PRIVATE ethernet_ip unity)
add_test(NAME test_eip_axis COMMAND test_eip_axis)

# W5500 SPI frame encoding test (pure C++ constexpr logic, no ESP-IDF)
add_executable(test_w5500_spi "test_w5500_spi.cpp")
target_include_directories(test_w5500_spi PRIVATE
    "${REPO_ROOT}/lib/W5500/src"
)
target_link_libraries(test_w5500_spi PRIVATE unity)
add_test(NAME test_w5500_spi COMMAND test_w5500_spi)



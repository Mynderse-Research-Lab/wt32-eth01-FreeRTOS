/**
 * @file main_internal.h
 * @brief Declarations shared by main.cpp and the main_*.cpp files that
 *        start the application. Private to src/.
 */
#pragma once

#include "CellNetL2.h"
#include "EspEthL2Transport.h"
#include "EthernetLink.h"
#include "Gantry.h"
#include "gantry_test_console.h"

[[maybe_unused]] static const char* TAG = "GantryApp";

// ---------------------------------------------------------------------------
// Task parameter structures
// ---------------------------------------------------------------------------
struct UpdateTaskConfig {
    Gantry::Gantry* gantry;
};

struct UiTaskConfig {
    Gantry::Gantry* gantry;
    Network::EthernetLink* eth;
};

// main_tasks.cpp
void gantryUpdateTask(void* param);
void tftUiTask(void* param);

// What the motion stage hands to the later stages.
struct BootMotion {
    Gantry::Gantry* gantry = nullptr;      // nullptr: Gantry::begin() failed, stop the boot
    w5500::W5500Hal* w5500_hal = nullptr;  // nullptr: no W5500 (offline or not built)
};

// Boot stages, called in this order by app_main (main.cpp).

// main_init_io.cpp: NVS and the stored parameters, then the log UART driver.
void bootInitConfig();
// main_init_io.cpp: LAN8720 crystal, SPI3 line probe, SPI3 bus, MCP23S17, TFT.
void bootInitIo();
// main_init_motion.cpp: W5500, process images, axes, Gantry, EIP scanner task.
BootMotion bootInitMotion();
// main_tasks.cpp: pick scheduler, Gantry update, TFT UI and UART console tasks.
// nullptr when the pick scheduler task could not start (motors disabled).
GantryTestConsoleConfig* bootStartTasks(Gantry::Gantry& gantry, w5500::W5500Hal* w5500_hal,
                                        Network::EthernetLink& ethernetLink,
                                        CellNet::CellNetL2Node& l2Node);
// main_init_net.cpp: TCP console, OTA server, CellNet L2 and NTP once the LAN
// is up. True when the link came up in time.
bool bootStartNetwork(Gantry::Gantry& gantry, GantryTestConsoleConfig& consoleCfg,
                      Network::EthernetLink& ethernetLink,
                      CellNet::EspEthL2Transport& l2Transport,
                      CellNet::CellNetL2Node& l2Node);

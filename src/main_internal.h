/**
 * @file main_internal.h
 * @brief Declarations shared by main.cpp and the main_*.cpp files that
 *        start the application. Private to src/.
 */
#pragma once

#include "EthernetLink.h"
#include "Gantry.h"

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

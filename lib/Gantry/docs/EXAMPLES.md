# Gantry Library Examples

**Version:** 1.0.0  
**Last Updated:** Feb 10th 2026

Complete code examples for using the Gantry library.

---

## Table of Contents

- [Basic Setup](#basic-setup)
- [Simple Motion](#simple-motion)
- [Homing & Calibration](#homing--calibration)
- [Sequential Motion](#sequential-motion)
- [Pick-and-Place](#pick-and-place)
- [FreeRTOS Integration](#freertos-integration)
- [Kinematics Usage](#kinematics-usage)
- [Error Handling](#error-handling)

---

## Basic Setup

### Complete Initialization Example

```cpp
#include "Gantry.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "Gantry";

// Pin definitions
#define X_STEP_PIN 32
#define X_DIR_PIN 33
#define X_ENABLE_PIN 25
#define X_MIN_LIMIT 35
#define X_MAX_LIMIT 36
#define Y_STEP_PIN 26
#define Y_DIR_PIN 27
#define Y_ENABLE_PIN 14
#define THETA_PWM_PIN 13
#define GRIPPER_PIN 12

static Gantry::Gantry* g_gantry = nullptr;

// Gantry update task
void gantryUpdateTask(void *pvParameters) {
    Gantry::Gantry* gantry = (Gantry::Gantry*)pvParameters;
    
    ESP_LOGI(TAG, "Initializing Gantry...");
    
    // Initialize
    if (!gantry->begin()) {
        ESP_LOGE(TAG, "Gantry initialization failed!");
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "Gantry initialized successfully");
    
    // Enable motors
    gantry->enable();
    ESP_LOGI(TAG, "Motors enabled");
    
    // Home X-axis
    ESP_LOGI(TAG, "Homing X-axis...");
    gantry->home();
    while (gantry->isBusy()) {
        gantry->update();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGI(TAG, "Homing complete");
    
    // Main update loop
    while (1) {
        gantry->update();
        vTaskDelay(pdMS_TO_TICKS(10));  // 100 Hz update rate
    }
}

void app_main(void) {
    // Configure X-axis servo driver
    BergerdaServo::DriverConfig xConfig;
    xConfig.step_pin = X_STEP_PIN;
    xConfig.dir_pin = X_DIR_PIN;
    xConfig.enable_pin = X_ENABLE_PIN;
    xConfig.encoder_ppr = 6000;
    xConfig.homing_speed_pps = 6000;
    xConfig.max_speed_pps = 10000;
    xConfig.max_accel_pps2 = 5000;
    
    // Create gantry instance
    static Gantry::Gantry gantry(xConfig, GRIPPER_PIN);
    g_gantry = &gantry;
    
    // Configure limit switches
    gantry.setLimitPins(X_MIN_LIMIT, X_MAX_LIMIT);
    
    // Configure Y-axis stepper
    gantry.setYAxisPins(Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN);
    gantry.setYAxisStepsPerMm(200.0f);  // 200 steps/mm
    gantry.setYAxisLimits(0.0f, 200.0f);
    gantry.setYAxisMotionLimits(100.0f, 500.0f, 500.0f);
    
    // Configure theta servo
    gantry.setThetaServo(THETA_PWM_PIN, 0);
    gantry.setThetaLimits(-90.0f, 90.0f);
    gantry.setThetaPulseRange(1000, 2000);
    
    // Set safe height
    gantry.setSafeYHeight(150.0f);
    
    // Create gantry update task
    xTaskCreate(gantryUpdateTask, "GantryUpdate", 4096, &gantry, 5, NULL);
    
    // Other initialization...
}
```

---

## Simple Motion

### Move to Joint Configuration

```cpp
void moveToPosition(Gantry::Gantry* gantry) {
    Gantry::JointConfig target;
    target.x = 200.0f;    // 200mm from home
    target.y = 50.0f;     // 50mm extended
    target.theta = 45.0f; // 45 degrees
    
    GantryError result = gantry->moveTo(target, 50, 30);
    if (result != GantryError::OK) {
        ESP_LOGE(TAG, "Move failed: %d", (int)result);
        return;
    }
    
    // Wait for completion
    while (gantry->isBusy()) {
        gantry->update();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    ESP_LOGI(TAG, "Move complete");
}
```

### Move to End-Effector Pose

```cpp
void moveToPose(Gantry::Gantry* gantry) {
    Gantry::EndEffectorPose target;
    target.x = 200.0f;
    target.y = 100.0f;
    target.z = 80.0f;
    target.theta = 90.0f;
    
    gantry->moveTo(target, 50, 30);
    
    while (gantry->isBusy()) {
        gantry->update();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
```

---

## Homing & Calibration

### Homing Sequence

```cpp
void homeGantry(Gantry::Gantry* gantry) {
    ESP_LOGI(TAG, "Starting homing sequence...");
    
    gantry->enable();
    gantry->home();
    
    TickType_t startTime = xTaskGetTickCount();
    while (gantry->isBusy() && (xTaskGetTickCount() - startTime) < pdMS_TO_TICKS(30000)) {
        gantry->update();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    if (gantry->isBusy()) {
        ESP_LOGE(TAG, "Homing timeout");
    } else {
        ESP_LOGI(TAG, "Homing complete");
    }
}
```

### Calibration Sequence

```cpp
void calibrateGantry(Gantry::Gantry* gantry) {
    ESP_LOGI(TAG, "Starting calibration...");
    
    gantry->enable();
    
    int axisLength = gantry->calibrate();
    if (axisLength > 0) {
        ESP_LOGI(TAG, "Calibration successful: Axis length = %d mm", axisLength);
    } else {
        ESP_LOGE(TAG, "Calibration failed");
    }
}
```

---

## Sequential Motion

### Understanding Sequential Motion

The library automatically sequences motion:

```cpp
void demonstrateSequentialMotion() {
    // Target position with low Y (picking position)
    Gantry::JointConfig pickPos;
    pickPos.x = 300.0f;
    pickPos.y = 30.0f;   // Low position
    pickPos.theta = 0.0f;
    
    // Motion sequence:
    // 1. Y descends to 30mm
    // 2. Gripper closes (picking)
    // 3. Y retracts to safe height (150mm)
    // 4. X moves to 300mm
    // 5. Theta moves to 0° (independent)
    
    gantry.moveTo(pickPos, 50, 30);
    
    while (gantry.isBusy()) {
        gantry.update();
        delay(10);
        
        // Can monitor motion state here if needed
    }
}
```

---

## Pick-and-Place

### Complete Pick-and-Place Sequence

```cpp
void pickAndPlace(float pickX, float pickY, float placeX, float placeY) {
    // Move to pick position
    Gantry::JointConfig pickPos;
    pickPos.x = pickX;
    pickPos.y = pickY;  // Low position for picking
    pickPos.theta = 0.0f;
    
    Serial.println("Moving to pick position...");
    gantry.moveTo(pickPos, 50, 30);
    while (gantry.isBusy()) {
        gantry.update();
        delay(10);
    }
    Serial.println("At pick position");
    
    // Gripper closes automatically during sequential motion
    delay(200);  // Ensure gripper has time to close
    
    // Move to place position
    Gantry::JointConfig placePos;
    placePos.x = placeX;
    placePos.y = placeY;  // Low position for placing
    placePos.theta = 90.0f;
    
    Serial.println("Moving to place position...");
    gantry.moveTo(placePos, 50, 30);
    while (gantry.isBusy()) {
        gantry.update();
        delay(10);
    }
    Serial.println("At place position");
    
    // Gripper opens automatically during sequential motion
    delay(200);  // Ensure gripper has time to open
    
    Serial.println("Pick-and-place complete");
}

void loop() {
    // Pick from position (200, 30) and place at (400, 30)
    pickAndPlace(200.0f, 30.0f, 400.0f, 30.0f);
    
    delay(5000);  // Wait 5 seconds before next cycle
}
```

### Multiple Pick-and-Place Operations

```cpp
void multiplePickAndPlace() {
    struct Position {
        float x, y;
    };
    
    Position pickPositions[] = {
        {100.0f, 30.0f},
        {200.0f, 30.0f},
        {300.0f, 30.0f}
    };
    
    Position placePosition = {400.0f, 30.0f};
    
    for (int i = 0; i < 3; i++) {
        // Pick
        Gantry::JointConfig pick;
        pick.x = pickPositions[i].x;
        pick.y = pickPositions[i].y;
        pick.theta = 0.0f;
        
        gantry.moveTo(pick, 50, 30);
        while (gantry.isBusy()) {
            gantry.update();
            delay(10);
        }
        delay(200);  // Gripper close time
        
        // Place
        Gantry::JointConfig place;
        place.x = placePosition.x;
        place.y = placePosition.y;
        place.theta = 90.0f;
        
        gantry.moveTo(place, 50, 30);
        while (gantry.isBusy()) {
            gantry.update();
            delay(10);
        }
        delay(200);  // Gripper open time
    }
}
```

---

## FreeRTOS Integration

### Single Task Approach (Recommended)

```cpp
#include "Gantry.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "Gantry";

static Gantry::Gantry* g_gantry = nullptr;

void gantryTask(void *pvParameters) {
    Gantry::Gantry* gantry = (Gantry::Gantry*)pvParameters;
    
    // Initialize gantry in task
    gantry->begin();
    gantry->enable();
    gantry->home();
    
    while (gantry->isBusy()) {
        gantry->update();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    // Main loop
    while (1) {
        gantry->update();
        vTaskDelay(pdMS_TO_TICKS(10));  // 100 Hz update rate
    }
}

void app_main(void) {
    // Configure gantry (same as basic setup)
    BergerdaServo::DriverConfig xConfig;
    // ... configure ...
    
    static Gantry::Gantry gantry(xConfig, GRIPPER_PIN);
    g_gantry = &gantry;
    
    // Configure axes
    // ... configuration ...
    
    // Create gantry task
    xTaskCreate(
        gantryTask,      // Task function
        "Gantry",        // Task name
        4096,            // Stack size
        &gantry,         // Parameters
        5,               // Priority
        NULL             // Task handle
    );
    
    // Other initialization...
}
```

### Multi-Task Approach (With Mutex)

```cpp
#include "Gantry.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"

static const char *TAG = "Gantry";

static SemaphoreHandle_t gantryMutex = nullptr;

void gantryUpdateTask(void *pvParameters) {
    Gantry::Gantry* gantry = (Gantry::Gantry*)pvParameters;
    
    while (1) {
        if (xSemaphoreTake(gantryMutex, portMAX_DELAY)) {
            gantry->update();
            xSemaphoreGive(gantryMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void motionControlTask(void *pvParameters) {
    Gantry::Gantry* gantry = (Gantry::Gantry*)pvParameters;
    
    while (1) {
        // Plan motion
        Gantry::JointConfig target;
        target.x = 200.0f;
        target.y = 50.0f;
        target.theta = 45.0f;
        
        // Execute motion (with mutex)
        if (xSemaphoreTake(gantryMutex, portMAX_DELAY)) {
            gantry->moveTo(target, 50, 30);
            xSemaphoreGive(gantryMutex);
        }
        
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void app_main(void) {
    // Create mutex
    gantryMutex = xSemaphoreCreateMutex();
    
    // Configure and create gantry instance
    BergerdaServo::DriverConfig xConfig;
    // ... configure ...
    static Gantry::Gantry gantry(xConfig, GRIPPER_PIN);
    // ... configure axes ...
    
    // Create tasks
    xTaskCreate(gantryUpdateTask, "GantryUpdate", 2048, &gantry, 5, NULL);
    xTaskCreate(motionControlTask, "MotionCtrl", 2048, &gantry, 4, NULL);
    
    // Other initialization...
}
```

---

## Kinematics Usage

### Forward Kinematics

```cpp
void demonstrateForwardKinematics() {
    // Joint space position
    Gantry::JointConfig joint;
    joint.x = 200.0f;
    joint.y = 50.0f;
    joint.theta = 45.0f;
    
    // Calculate end-effector pose
    Gantry::EndEffectorPose pose = gantry.forwardKinematics(joint);
    
    Serial.printf("Joint: x=%.1f y=%.1f theta=%.1f\n",
                  joint.x, joint.y, joint.theta);
    Serial.printf("End-effector: x=%.1f y=%.1f z=%.1f theta=%.1f\n",
                  pose.x, pose.y, pose.z, pose.theta);
}
```

### Inverse Kinematics

```cpp
void demonstrateInverseKinematics() {
    // Desired end-effector pose
    Gantry::EndEffectorPose desired;
    desired.x = 200.0f;
    desired.y = 100.0f;
    desired.z = 80.0f;
    desired.theta = 90.0f;
    
    // Calculate required joint positions
    Gantry::JointConfig joint = gantry.inverseKinematics(desired);
    
    Serial.printf("Desired pose: x=%.1f y=%.1f z=%.1f theta=%.1f\n",
                  desired.x, desired.y, desired.z, desired.theta);
    Serial.printf("Required joint: x=%.1f y=%.1f theta=%.1f\n",
                  joint.x, joint.y, joint.theta);
    
    // Move to calculated joint position
    gantry.moveTo(joint, 50, 30);
}
```

### Round-Trip Verification

```cpp
void verifyKinematics() {
    // Start with joint configuration
    Gantry::JointConfig original;
    original.x = 150.0f;
    original.y = 75.0f;
    original.theta = 30.0f;
    
    // Forward kinematics
    Gantry::EndEffectorPose pose = gantry.forwardKinematics(original);
    
    // Inverse kinematics (should recover original)
    Gantry::JointConfig recovered = gantry.inverseKinematics(pose);
    
    // Verify
    float error = abs(original.x - recovered.x) +
                  abs(original.y - recovered.y) +
                  abs(original.theta - recovered.theta);
    
    if (error < 0.1f) {
        Serial.println("Kinematics verified: Round-trip successful");
    } else {
        Serial.println("WARNING: Kinematics round-trip error detected");
    }
}
```

---

## Error Handling

### Comprehensive Error Handling

```cpp
void moveWithErrorHandling(Gantry::JointConfig target) {
    // Check initialization
    if (!gantry.isBusy()) {  // Simple check
        Serial.println("Gantry not ready");
        return;
    }
    
    // Attempt move
    GantryError result = gantry.moveTo(target, 50, 30);
    
    // Handle errors
    switch (result) {
        case GantryError::OK:
            Serial.println("Move started successfully");
            break;
            
        case GantryError::NOT_INITIALIZED:
            Serial.println("ERROR: Gantry not initialized");
            Serial.println("Call begin() first");
            return;
            
        case GantryError::MOTOR_NOT_ENABLED:
            Serial.println("ERROR: Motors not enabled");
            Serial.println("Call enable() first");
            return;
            
        case GantryError::ALREADY_MOVING:
            Serial.println("WARNING: Motion already in progress");
            Serial.println("Wait for current motion to complete");
            return;
            
        case GantryError::INVALID_POSITION:
            Serial.println("ERROR: Position out of valid range");
            Serial.printf("Target: x=%.1f y=%.1f theta=%.1f\n",
                         target.x, target.y, target.theta);
            return;
            
        case GantryError::INVALID_PARAMETER:
            Serial.println("ERROR: Invalid motion parameter");
            Serial.println("Check speed/acceleration values");
            return;
            
        case GantryError::TIMEOUT:
            Serial.println("ERROR: Operation timeout");
            Serial.println("Check for alarms");
            break;
            
        default:
            Serial.printf("ERROR: Unknown error code %d\n", (int)result);
            return;
    }
    
    // Wait for completion with timeout
    unsigned long startTime = millis();
    while (gantry.isBusy() && (millis() - startTime) < 30000) {
        gantry.update();
        
        // Check for alarms
        if (gantry.isAlarmActive()) {
            Serial.println("ALARM: Motion stopped due to alarm");
            return;
        }
        
        delay(10);
    }
    
    if (gantry.isBusy()) {
        Serial.println("WARNING: Motion did not complete within timeout");
    } else {
        Serial.println("Motion completed successfully");
    }
}
```

### Alarm Monitoring

```cpp
void monitorAlarms() {
    if (gantry.isAlarmActive()) {
        Serial.println("ALARM ACTIVE!");
        
        // Stop all motion
        gantry.disable();
        
        // Wait and retry
        delay(1000);
        gantry.enable();
        
        // Re-home if needed
        gantry.home();
    }
}
```

---

## Advanced Examples

### Waypoint Trajectory (Future)

```cpp
void waypointTrajectory() {
    // Create waypoint queue
    Gantry::WaypointQueue<16> waypoints;
    
    // Add waypoints
    Gantry::Waypoint wp1;
    wp1.pose = Gantry::EndEffectorPose(100.0f, 50.0f, 80.0f, 0.0f);
    wp1.speed_mm_per_s = 50;
    waypoints.push(wp1);
    
    Gantry::Waypoint wp2;
    wp2.pose = Gantry::EndEffectorPose(200.0f, 75.0f, 80.0f, 45.0f);
    wp2.speed_mm_per_s = 70;
    waypoints.push(wp2);
    
    // Execute waypoints (implementation pending)
    // gantry.executeWaypoints(waypoints);
}
```

### Status Monitoring

```cpp
void printStatus() {
    Serial.println("=== Gantry Status ===");
    Serial.printf("X Position: %d mm\n", gantry.getXEncoder());
    Serial.printf("Y Position: %d mm\n", gantry.getCurrentY());
    Serial.printf("Theta: %d deg\n", gantry.getCurrentTheta());
    Serial.printf("Busy: %s\n", gantry.isBusy() ? "Yes" : "No");
    Serial.printf("Alarm: %s\n", gantry.isAlarmActive() ? "Yes" : "No");
    
    // Get current joint configuration
    Gantry::JointConfig current = gantry.getCurrentJointConfig();
    Serial.printf("Joint: x=%.1f y=%.1f theta=%.1f\n",
                  current.x, current.y, current.theta);
    
    // Get current end-effector pose
    Gantry::EndEffectorPose pose = gantry.getCurrentEndEffectorPose();
    Serial.printf("End-effector: x=%.1f y=%.1f z=%.1f theta=%.1f\n",
                  pose.x, pose.y, pose.z, pose.theta);
}
```

---

**Last Updated:** Feb 10th 2026  
**Version:** 1.0.0


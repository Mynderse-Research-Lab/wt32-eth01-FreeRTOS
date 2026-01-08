# SDF08NK8X Servo Driver Library

Library for controlling Bergerda SDF-08-N-K-8X servo drivers via Pulse/Direction interface.

## Features

- **Deterministic Motion Control**: Uses high-precision `esp_timer` and hardware `LEDC` to generate strictly timed trapezoidal velocity profiles (Acceleration -> Cruise -> Deceleration).
- **Hardware Encoder Feedback**: Reads quadrature encoder via PCNT with **multi-turn software accumulation** (64-bit tracking).
- **Real-Time Feedback**: Reports both commanded position and actual encoder feedback in real-time.
- **Smooth Stopping**: Supports gradual deceleration when stopping mid-move.
- **Status Monitoring**: Monitors critical driver signals:
  - `Position Reached` (IN-POS)
  - `Brake Released` (BRK)
  - `Alarm Active` (ALM)
- **Thread Safety**: Built-in FreeRTOS mutex support for safe operation in multi-tasking environments.
- **Framework Version Agnostic**: Supports both Arduino ESP32 Core v2.x and v3.x.

## Wiring (Recommended)

| Signal | Driver CN1 Pin | ESP32 GPIO | Description |
|--------|----------------|------------|-------------|
| PULSE  | Pin 18         | Any (Def: -) | Pulse Input (High Speed) |
| SIGN   | Pin 19         | Any (Def: -) | Direction Input |
| ENABLE | Pin 21 (IN0)   | Any (Def: -) | Servo On (SON) |
| ALARM  | Pin 15 (OUT3)  | Any (Def: -) | Alarm Signal (Active Low) |
| RST    | Pin 9 (IN1)    | Any (Def: -) | Alarm Reset |
| A+     | Pin 23         | Any (Def: -) | Encoder A+ |
| B+     | Pin 24         | Any (Def: -) | Encoder B+ |
| POS    | Pin 1 (OUT1)   | Any (Def: -) | Position Reached |
| BRK    | Pin 14 (OUT2)  | Any (Def: -) | Brake Release |

*Note: You must define your specific ESP32 GPIO pins in the software configuration.*

## Installation

1. Copy the `SDF08NK8X` folder into your project's `lib/` directory.
2. Include the header in your main file:
   ```cpp
   #include "SDF08NK8X.h"
   ```

## Usage Example (FreeRTOS)

```cpp
#include <Arduino.h>
#include "SDF08NK8X.h"

// Define your ESP32 Pin Assignment
#define PIN_PULSE    18  // Connect to Driver Pin 18
#define PIN_DIR      19  // Connect to Driver Pin 19
#define PIN_ENABLE   21  // Connect to Driver Pin 21
#define PIN_ENC_A    23  // Connect to Driver Pin 23
#define PIN_ENC_B    24  // Connect to Driver Pin 24

BergerdaServo::DriverConfig config;
// Config handled in setup()

BergerdaServo::ServoDriver driver(config);

void motionTask(void *parameter) {
  // Initialize driver
  if (!driver.initialize()) {
    Serial.println("Driver Init Failed!");
    vTaskDelete(NULL);
  }

  driver.enable();

  while (1) {
    // Determine target position (e.g., toggle between 0 and 10000)
    static bool toggle = false;
    uint32_t target = toggle ? 10000 : 0;
    toggle = !toggle;

    // Execute Move
    driver.moveToPosition(target, 10000, 5000, 5000);

    // Wait for completion while monitoring status
    while (driver.isMotionActive()) {
       BergerdaServo::DriveStatus status = driver.getStatus();
       Serial.printf("Cmd: %u | Enc: %d\n", status.current_position, status.encoder_position);
       vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    vTaskDelay(pdMS_TO_TICKS(1000)); // Pause for 1 second
  }
}

void setup() {
  Serial.begin(115200);
  
  // Configure Pins (Map ESP32 GPIOs to Driver Functions)
  config.output_pin_nos[6] = PIN_PULSE;
  config.output_pin_nos[7] = PIN_DIR;
  config.output_pin_nos[0] = PIN_ENABLE;
  
  config.input_pin_nos[3] = PIN_ENC_A;
  config.input_pin_nos[4] = PIN_ENC_B;
  
  config.enable_encoder_feedback = true; 
  config.pcnt_unit = PCNT_UNIT_0;

  // Create Task
  xTaskCreate(
    motionTask,    // Task function
    "MotionTask",  // Name
    4096,          // Stack size
    NULL,          // Parameter
    1,             // Priority
    NULL           // Handle
  );
}

void loop() {
  // Empty - work is done in FreeRTOS task
  vTaskDelay(pdMS_TO_TICKS(1000)); 
}
```

## Known Limitations

1. **Supported Modes**: This library exclusively supports `Pulse/Direction` control mode (`PN04=0`, `PN08=0`).

## License

MIT License.

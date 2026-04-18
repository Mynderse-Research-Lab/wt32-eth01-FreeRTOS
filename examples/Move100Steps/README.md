# Move 100 Steps Example

This example demonstrates how to move a servo motor exactly 100 steps (pulses) using the PulseMotor driver library with **internal step counting only** (no encoder feedback).

## Overview

The example performs the following sequence:
1. Initializes the servo driver with proper pin configuration
2. Enables the motor
3. Moves the motor 100 steps in the positive direction
4. Waits for motion to complete
5. Moves the motor 100 steps back (negative direction)
6. Repeats the cycle

## Hardware Setup

### Required Components
- WT32-ETH01 or ESP32 development board
- Pulse-train motor driver
- Servo motor (encoder not required)
- Power supply for servo driver and motor
- USB cable for programming

### Pin Connections

| ESP32 GPIO | Signal | Direction | Driver CN1 Pin | Description |
|------------|--------|-----------|----------------|-------------|
| GPIO 2     | PULSE  | OUTPUT    | Pin 18         | Pulse signal |
| GPIO 4     | DIR    | OUTPUT    | Pin 19         | Direction signal |
| GPIO 12    | ENABLE | OUTPUT    | Pin 21 (IN0)   | Servo enable (SON) |
| GPIO 14    | LIMIT_MIN | INPUT (PU) | External switch | Home limit switch (active LOW) |
| GPIO 32    | LIMIT_MAX | INPUT (PU) | External switch | End limit switch (active LOW) |

**Note**: 
- This example does NOT use encoder feedback. Position is tracked internally by counting pulses sent to the driver.
- Limit switches use internal pullup resistors. When switch is open: GPIO reads HIGH. When switch is closed: GPIO reads LOW.

### Wiring Diagram

```
ESP32 (WT32-ETH01)          Pulse-Train Driver
─────────────────          ─────────────────
GPIO 2  ────────────────── CN1 Pin 18 (PULSE)
GPIO 4  ────────────────── CN1 Pin 19 (DIR)
GPIO 12 ────────────────── CN1 Pin 21 (ENABLE/SON)
GND     ────────────────── CN1 GND (Common ground)

Limit Switches (NO - Normally Open):
────────────────────────────────────
Home Limit Switch (MIN):
  COM ──────────────────── GND
  NO  ──────────────────── GPIO 14 (with internal pullup)

End Limit Switch (MAX):
  COM ──────────────────── GND
  NO  ──────────────────── GPIO 32 (with internal pullup)

Note: 
- Encoder connections are NOT required for this example.
- Position is tracked internally by counting pulses.
- Limit switches stop the motor when triggered (active LOW).
```

## Configuration

### Motion Parameters

You can adjust the motion parameters in the code:

```cpp
const uint32_t STEPS_TO_MOVE = 100;        // Number of steps to move
const uint32_t MAX_SPEED = 5000;          // Maximum speed (pulses per second)
const uint32_t ACCELERATION = 2000;       // Acceleration rate (pps²)
const uint32_t DECELERATION = 2000;       // Deceleration rate (pps²)
```

- **STEPS_TO_MOVE**: Number of steps (pulses) to move. Change this to move a different distance.
- **MAX_SPEED**: Maximum speed during movement in pulses per second (pps)
- **ACCELERATION**: How quickly the motor accelerates to max speed (pps²)
- **DECELERATION**: How quickly the motor decelerates to stop (pps²)

## Usage

1. **Connect Hardware**: Wire the ESP32 to the servo driver according to the pin connections above.

2. **Upload Code**: 
   - Open this example in PlatformIO
   - Connect your ESP32 via USB
   - Build and upload the firmware

3. **Monitor Output**: 
   - Open serial monitor at 115200 baud
   - Watch the motor move 100 steps forward, then 100 steps back
   - The serial output shows position, encoder feedback, and speed

## Expected Serial Output

```
========================================
Move 100 Steps Example
========================================

Initializing servo driver...
✓ Driver initialized successfully
Enabling motor...
✓ Motor enabled

Initial Position: 0 steps (internal counter)
Limit Switches: MIN=open MAX=open

Ready to move 100 steps...

----------------------------------------
Current Position: 0 steps
Limit Switches: MIN=open MAX=open

Moving +100 steps...
Speed: 5000 pps, Accel: 2000 pps², Decel: 2000 pps²
✓ Move command accepted
Waiting for motion to complete...
  Position: 100 steps | Speed: 0 pps
✓ Motion complete!
Final Position: 100 steps
Distance moved: 100 steps

Waiting 2 seconds...

----------------------------------------
Current Position: 100 steps
Limit Switches: MIN=open MAX=open

Moving -100 steps (back to start)...
✓ Move command accepted
Waiting for motion to complete...
  Position: 0 steps | Speed: 0 pps
✓ Motion complete!
Final Position: 0 steps
Distance moved: -100 steps

Waiting 3 seconds before repeating...
```

**Example with Limit Switch Triggered:**
```
----------------------------------------
Current Position: 0 steps
Limit Switches: MIN=open MAX=ACTIVE

⚠ WARNING: End limit switch (MAX) is ACTIVE!
   Cannot move forward. Skipping this move.
```

## Key Concepts

### Steps vs. Pulses
- **Steps** and **pulses** are the same thing in this context
- Each pulse sent to the driver moves the motor by one step
- The number of steps depends on your motor's step angle and any gearing

### Relative Movement
- `moveRelative(100, ...)` moves 100 steps forward from current position
- `moveRelative(-100, ...)` moves 100 steps backward from current position
- The sign determines direction (positive = forward, negative = backward)

### Motion Profile
- The motor uses a trapezoidal velocity profile:
  - **Acceleration phase**: Motor speeds up from 0 to max speed
  - **Cruise phase**: Motor runs at constant max speed
  - **Deceleration phase**: Motor slows down to stop
- This provides smooth motion without sudden starts/stops

### Internal Step Counting
- Position is tracked internally by counting pulses sent to the driver
- The `getPosition()` method returns the commanded position (number of pulses sent)
- This is an open-loop system - no feedback from actual motor position
- If the motor misses steps, the internal counter will not reflect the actual position

### Limit Switch Protection
- **Home Limit (MIN)**: Stops motor when moving backward (negative direction)
- **End Limit (MAX)**: Stops motor when moving forward (positive direction)
- Limit switches are checked:
  - Before starting a move (prevents movement if limit is already active)
  - During motion (stops motor immediately if limit is triggered)
- Motor stops with deceleration when a limit switch is triggered
- Limit switches use internal pullup resistors (active LOW when switch closes)

## Troubleshooting

### Motor Doesn't Move
1. Check power supply is connected and adequate
2. Verify motor is enabled (`driver->isEnabled()` returns true)
3. Check pin connections are correct
4. Verify servo driver is not in alarm state
5. Check that limit switches are not triggered (if connected)

### Motion is Erratic
1. Reduce max speed
2. Increase acceleration/deceleration values
3. Check for mechanical binding
4. Verify encoder connections

### Position Doesn't Match
1. **Note**: This example uses open-loop control (no encoder feedback)
2. If position doesn't match, the motor may have missed steps
3. Check for mechanical binding or excessive load
4. Reduce speed or increase acceleration/deceleration values
5. For closed-loop control, enable encoder feedback in the configuration

### Limit Switch Issues
1. **Motor stops unexpectedly**: Check if limit switch is triggered
   - Serial output shows limit switch status
   - Verify limit switch wiring (COM to GND, NO to GPIO)
2. **Cannot move in one direction**: Limit switch may be stuck or miswired
   - Check limit switch continuity with multimeter
   - Verify pullup resistor is working (GPIO should read HIGH when switch is open)
3. **Limit switch not stopping motor**: 
   - Verify wiring is correct
   - Check that switch closes properly (GPIO should read LOW when switch is closed)
   - Ensure limit switch is checked during motion (code does this automatically)

## Modifying the Example

### Change Number of Steps
```cpp
const uint32_t STEPS_TO_MOVE = 200;  // Move 200 steps instead
```

### Change Speed
```cpp
const uint32_t MAX_SPEED = 10000;  // Faster movement (10000 pps)
```

### Move Only Once
Remove the `loop()` function and put the movement code in `setup()`:

```cpp
void setup() {
  // ... initialization code ...
  
  // Move 100 steps once
  driver->moveRelative(100, MAX_SPEED, ACCELERATION, DECELERATION);
  while (driver->isMotionActive()) {
    driver->update();
    delay(10);
  }
}

void loop() {
  // Empty - motion happens once in setup()
}
```

## Related Examples

- See other examples in the `examples/` directory for more advanced usage
- Check `lib/PulseMotor/README.md` for detailed API documentation

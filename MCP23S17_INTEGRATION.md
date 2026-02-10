# MCP23S17 GPIO Expander Integration

## Overview

All digital GPIO operations have been moved to MCP23S17 SPI GPIO expander. PWM and encoder pins remain on direct ESP32 GPIO.

## Pin Assignment

### MCP23S17 GPIO Expander (SPI)

**SPI Pins (Direct ESP32 GPIO):**
- CS: GPIO 5
- MISO: GPIO 19
- MOSI: GPIO 23
- SCLK: GPIO 18

**MCP23S17 Port A (Pins 0-7):**
- Pin 0: X-axis PULSE (output)
- Pin 1: X-axis DIR (output)
- Pin 2: X-axis ENABLE (output)
- Pin 3: X-axis LIMIT_MIN (input, pullup)
- Pin 4: X-axis LIMIT_MAX (input, pullup)
- Pin 5: Y-axis STEP (output)
- Pin 6: Y-axis DIR (output)
- Pin 7: Y-axis ENABLE (output)

**MCP23S17 Port B (Pins 8-15):**
- Pin 8: GRIPPER (output)
- Pin 9: LED (output)
- Pins 10-15: Available for future use

### Direct ESP32 GPIO (PWM/Encoder)

- GPIO 13: Theta-axis PWM (LEDC output)
- GPIO 35: Encoder A+ (PCNT input, input-only pin)
- GPIO 36: Encoder B+ (PCNT input, input-only pin)

## Library Modifications Required

The following libraries need modifications to support MCP23S17:

### 1. SDF08NK8X Library

**File:** `lib/SDF08NK8X/src/SDF08NK8X.cpp`

**Changes needed:**
- Replace `gpio_set_level()` calls with `gpio_expander_write()`
- Replace `gpio_get_level()` calls with `gpio_expander_read()`
- Replace `gpio_config()` calls with `gpio_expander_set_direction()`
- Update limit switch reading to use `gpio_expander_read()`

**Example:**
```cpp
// Old:
gpio_set_level((gpio_num_t)pin, level);

// New:
gpio_expander_write(pin, level);
```

### 2. GantryAxisStepper Library

**File:** `lib/Gantry/src/GantryAxisStepper.cpp`

**Changes needed:**
- Replace `digitalWrite()` / `gpio_set_level()` with `gpio_expander_write()`
- Replace `digitalRead()` / `gpio_get_level()` with `gpio_expander_read()`
- Replace `pinMode()` / `gpio_config()` with `gpio_expander_set_direction()`

**Example:**
```cpp
// Old:
digitalWrite(stepPin_, HIGH);
digitalWrite(stepPin_, LOW);

// New:
gpio_expander_write(stepPin_, 1);
gpio_expander_write(stepPin_, 0);
```

### 3. GantryEndEffector Library

**File:** `lib/Gantry/src/GantryEndEffector.cpp`

**Changes needed:**
- Replace `digitalWrite()` with `gpio_expander_write()`
- Replace `pinMode()` with `gpio_expander_set_direction()`

### 4. Gantry Library (Limit Switches)

**File:** `lib/Gantry/src/Gantry.cpp`

**Changes needed:**
- Update limit switch reading to use `gpio_expander_read()`
- Note: Limit switches are handled by SDF08NK8X library, so changes there will propagate

## GPIO Abstraction Layer

The GPIO abstraction layer (`include/gpio_expander.h` / `src/gpio_expander.c`) provides:

- `gpio_expander_init()` - Initialize MCP23S17
- `gpio_expander_set_direction()` - Configure pin direction
- `gpio_expander_set_pullup()` - Configure pull-up resistor
- `gpio_expander_write()` - Write pin level
- `gpio_expander_read()` - Read pin level

**Pin Numbering:**
- MCP23S17 pins: 0-15 (Port A: 0-7, Port B: 8-15)
- Direct GPIO pins: 16+ (ESP32 GPIO numbers)

## Current Status

✅ **Completed:**
- MCP23S17 library implementation
- GPIO abstraction layer
- Main application initialization
- Pin definitions updated

⚠️ **Pending:**
- SDF08NK8X library modifications
- GantryAxisStepper library modifications
- GantryEndEffector library modifications
- Testing and verification

## Testing

After library modifications:

1. Verify MCP23S17 initialization
2. Test digital outputs (PULSE, DIR, ENABLE, STEP pins)
3. Test digital inputs (limit switches)
4. Verify PWM still works (Theta servo)
5. Verify encoder reading still works (PCNT)

## Notes

- Encoder pins (GPIO 35, 36) must remain on direct ESP32 GPIO for PCNT functionality
- PWM pins must remain on direct ESP32 GPIO for LEDC functionality
- All other digital GPIO can be moved to MCP23S17
- MCP23S17 provides 16 GPIO pins, sufficient for current needs

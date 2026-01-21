# Basic Driver Test

A minimal test program to verify the SDF08NK8X servo driver library is working correctly.

## Purpose

This test program performs basic verification of the driver library:
1. **Driver Initialization** - Tests if the driver can be initialized
2. **Motor Enable** - Tests if the motor can be enabled
3. **Status Check** - Verifies status reading works
4. **Enable State** - Confirms enable state reporting
5. **Limit Switch Reading** - Tests limit switch input reading
6. **Limit Switch Protection** - Verifies limit switch logic
7. **Simple Move** - Optional test move with limit switch monitoring

## Hardware Setup

### Required
- WT32-ETH01 or ESP32 development board
- SDF08NK8X servo driver
- USB cable

### Optional (for move test)
- Servo motor connected to driver
- Power supply for servo driver

### Pin Connections

| ESP32 GPIO | Signal | Driver CN1 Pin | Description |
|------------|--------|----------------|-------------|
| GPIO 2     | PULSE  | Pin 18         | Pulse signal |
| GPIO 4     | DIR    | Pin 19         | Direction signal |
| GPIO 12    | ENABLE | Pin 21 (IN0)   | Servo enable (SON) |
| GPIO 14    | LIMIT_MIN | External switch | Home limit switch (active LOW) |
| GPIO 32    | LIMIT_MAX | External switch | End limit switch (active LOW) |

### Limit Switch Wiring

```
Limit Switch (Home)          ESP32
─────────────────           ──────
   COM ──────────────────── GND
   NO  ──────────────────── GPIO 14 (with internal pullup)

Limit Switch (End)
─────────────────
   COM ──────────────────── GND
   NO  ──────────────────── GPIO 32 (with internal pullup)
```

**Note**: Limit switches use internal pullup resistors. When switch is open: GPIO reads HIGH. When switch is closed: GPIO reads LOW (ACTIVE).

## Usage

1. **Upload the program** to your ESP32
2. **Open serial monitor** at 115200 baud
3. **Observe test results** - All tests should pass if library is working

## Expected Output

```
========================================
Basic Driver Library Test
========================================

Test 1: Driver Initialization
-------------------------------
✓ PASS: Driver initialized successfully

Test 2: Motor Enable
-------------------------------
✓ PASS: Motor enabled successfully

Test 3: Status Check
-------------------------------
  Servo Enabled: YES
  Motion Active: NO
  Current Position: 0 steps
  Current Speed: 0 pps
✓ PASS: Status read successfully

Test 4: Enable State Check
-------------------------------
✓ PASS: Driver reports enabled state correctly

Test 5: Limit Switch Reading
-------------------------------
  Limit MIN (Home): open (HIGH)
  Limit MAX (End):  open (HIGH)
✓ PASS: Limit switches read consistently

Note: Limit switches should read HIGH (open) when not connected.
      If switches are connected and closed, they will read LOW (ACTIVE).

Test 6: Limit Switch Protection
-------------------------------
✓ End limit (MAX) is open - forward movement allowed
✓ Home limit (MIN) is open - backward movement allowed

Test 7: Simple Move Test (10 steps)
-------------------------------
Attempting to move 10 steps forward...
✓ Move command accepted
Waiting for motion to complete...
✓ Motion completed
  Final Position: 10 steps

========================================
Test Summary
========================================
Basic driver library tests completed.
If all tests passed, the library is working correctly.
```

## Troubleshooting

### Test 1 Fails (Initialization)
- Check pin connections
- Verify GPIO pins are correct
- Check for compilation errors

### Test 2 Fails (Motor Enable)
- Verify driver is powered
- Check ENABLE pin connection (GPIO 12)
- Check if driver has alarm condition

### Test 5 Fails (Move Test)
- **This is normal if motor is not connected**
- The move command will be accepted but motion won't occur
- Connect motor and power supply to test actual movement

## Notes

- This test does NOT require encoder feedback
- Limit switches are optional - tests will pass even if switches are not connected
- Move test is optional - library works even if motor isn't connected
- All tests should pass to confirm library is functioning correctly
- Limit switches use internal pullup resistors (active LOW when switch closes)

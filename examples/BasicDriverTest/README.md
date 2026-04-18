# BasicDriverTest

Minimal test program for the `PulseMotor` library. Verifies:

1. Driver initialization
2. Motor enable / disable
3. Status read
4. Limit-switch reads
5. A short forward move (3000 pulses)

## Hardware Requirements

- WT32-ETH01 or any ESP32 development board
- A pulse+direction motor driver (verified targets: Bergerda SDF08NK8X, Allen-Bradley Kinetix 5100, custom pulse-train driver for SCHUNK ERD 04-40-D-H-N)
- Servo motor (optional for the basic tests; required for the move test)

## Pin Connections

| Signal   | ESP32 GPIO | Notes                                             |
|----------|------------|---------------------------------------------------|
| PULSE    | 2          | LEDC output (must be direct ESP32 GPIO)           |
| DIR      | 4          | Direction input                                   |
| ENABLE   | 12         | SON / servo-on                                    |
| LIMIT_MIN| 14         | Active-low, internal pullup                       |
| LIMIT_MAX| 32         | Active-low, internal pullup                       |

## Notes

- If the move test reports `ledc setup failed` in the ESP32 logs, inspect the
  LEDC channel assignment and the `ledc_resolution` value in `DriverConfig`.
- Limit switches default to active-low with internal pull-ups.

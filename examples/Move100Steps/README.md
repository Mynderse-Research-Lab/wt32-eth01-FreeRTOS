# Move100Steps

Minimal example of commanding a +/-100-pulse move using the `PulseMotor`
library with no encoder feedback (position is tracked by the internal pulse
counter).

Works against any pulse+direction driver. Verified target drivers:

- Bergerda SDF08NK8X
- Allen-Bradley Kinetix 5100 (PTI pulse+direction step mode)
- Custom pulse-train driver for SCHUNK ERD 04-40-D-H-N

## Hardware Requirements

- WT32-ETH01 or ESP32 development board
- One pulse+direction motor driver + motor (encoder not required)

## Pin Connections

| Signal    | ESP32 GPIO |
|-----------|------------|
| PULSE     | 2          |
| DIR       | 4          |
| ENABLE    | 12         |
| LIMIT_MIN | 14         |
| LIMIT_MAX | 32         |

## Flow

1. Driver + motor initialisation
2. Move +100 pulses
3. Wait for motion-complete
4. Move -100 pulses back to the starting position

## API cheat sheet

```cpp
PulseMotor::DriverConfig cfg;
cfg.pulse_pin  = 2;
cfg.dir_pin    = 4;
cfg.enable_pin = 12;
cfg.pulse_mode = PulseMotor::PulseMode::PULSE_DIRECTION;

PulseMotor::PulseMotorDriver drv(cfg);
drv.initialize();
drv.enable();
drv.moveRelative(100, /*speed_pps*/ 5000,
                 /*accel_pps2*/ 2000, /*decel_pps2*/ 2000);
```

See `lib/PulseMotor/README.md` for the full API reference.

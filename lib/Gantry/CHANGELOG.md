# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.0.0] - 2024-XX-XX

### Added
- Initial library skeleton with modular design
- `GantryConfig` module for configuration structures:
  - `JointConfig` - Joint space representation (x, y, theta)
  - `EndEffectorPose` - Cartesian space representation
  - `JointLimits` - Joint limits validation
  - `KinematicParameters` - Mechanical parameters
  - `GantryConfig` - Complete configuration
- `GantryKinematics` module for forward/inverse kinematics:
  - Forward kinematics (joint space -> cartesian space)
  - Inverse kinematics (cartesian space -> joint space)
  - Joint validation functions
- `GantryTrajectory` module for trajectory planning:
  - Trapezoidal velocity profiles
  - Position interpolation
  - Waypoint structures (for future use)
- PlatformIO library configuration (`library.json`)
- License file (MIT)
- Documentation structure

### Notes
- Library skeleton created for modular gantry control system
- Designed for WT32-ETH01 (ESP32 with 4MB flash, ~320KB RAM)
- Target memory usage: <10KB RAM
- Optimized for embedded systems

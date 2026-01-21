// motion_config.h  (WT32-ETH02 baseline – draft)
#ifndef MOTION_CONFIG_H
#define MOTION_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

#define NUM_AXES 3 // X, Y, THETA
#define AXIS_X 0
#define AXIS_Y 1
#define AXIS_THETA 2

#define NUM_SEGMENTS 64

typedef struct {
  double target_pos[NUM_AXES]; // mm or deg
  double start_pos[NUM_AXES];  // mm or deg
  double v_start;              // steps/s
  double v_peak;               // steps/s
  uint16_t accel_segments;     // segments used for accel/decel
  bool valid;                  // command ready flag
} MotionCommand;

typedef struct {
  uint32_t segment_steps[NUM_AXES][NUM_SEGMENTS];
  uint32_t segment_delay[NUM_AXES][NUM_SEGMENTS];
  uint16_t active_segments[NUM_AXES];
  volatile bool ready;
  volatile bool executing;
  volatile bool completed;
} MotionProfile;

typedef struct {
  volatile uint32_t current_step[NUM_AXES];
  volatile uint16_t current_segment[NUM_AXES];
} SystemStatus;

extern MotionCommand g_motion_command;
extern MotionProfile g_motion_profile;
extern SystemStatus g_system_status;

#endif // MOTION_CONFIG_H

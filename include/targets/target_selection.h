#pragma once

#include "drone/drone_types.h"
#include "types.h"

struct SelectedTarget {
  Position firepoint;
  int target_index;
  double effective_time;
};

struct TargetSelectionConfig {
  const Drone &drone;
  const TargetTrack &current_target_positions;
  const TargetTrack &next_target_positions;
  const MotionParams &motion_params;

  double velocity_time_step;
  double attack_speed;
  double acceleration_path;
  double angular_speed;
};
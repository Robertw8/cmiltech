#pragma once

#include "drone/drone_types.h"
#include "types.h"

struct TargetEvaluation {
  Position predicted_position;
  Position firepoint;
  double ammo_flight_time;
  double drone_arrival_time;
};

struct TargetEvaluationConfig {
  const Drone &drone;

  const Position &current_target_position;
  const Position &next_target_position;
  const MotionParams &motion_params;

  double velocity_time_step;
  double attack_speed;
  double acceleration_path;
};

std::optional<TargetEvaluation>
EvaluateTarget(const TargetEvaluationConfig &config);
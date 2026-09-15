#pragma once

#include "drone_types.h"
#include "types.h"
#include <optional>

struct TurnResult {
  double new_direction;
  bool turn_finished;
};

struct DroneMotionResult {
  double new_speed;
  DroneState new_state;
};

struct DroneMotionConfig {
  double current_speed;
  double attack_speed;
  double acceleration_path;
  double time_step;
  DroneState drone_state;
};

struct DronePositionConfig {
  const Position &current_position;
  double speed;
  double direction;
  double time_step;
};

struct TurnAngleConfig {
  const Position &current_position;
  const Position &target_position;
  double current_direction;
};

struct TurnConfig {
  double current_direction;
  double delta_angle;
  double angular_speed;
  double time_step;
};

struct DroneArrivalTimeConfig {
  const Position &current_position;
  const Position &firepoint;
  double attack_speed;
};

struct SwitchPenaltyConfig {
  DroneState drone_state;
  double current_speed;
  double acceleration;
  double remaining_turn_angle;
  double angular_speed;
};

std::optional<DroneMotionResult>
CalculateNextDroneMotion(const DroneMotionConfig &config);

Position CalculateNextDronePosition(const DronePositionConfig &config);

double CalculateTurnAngle(const TurnAngleConfig &config);

TurnResult CalculateTurn(const TurnConfig &config);

std::optional<double>
EstimateDroneArrivalTime(const DroneArrivalTimeConfig &config);

std::optional<double> CalculateSwitchPenalty(const SwitchPenaltyConfig &config);
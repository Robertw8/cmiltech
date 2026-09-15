#include "drone/drone_math.h"
#include "drone/drone_types.h"
#include "simulation/simulation_math.h"
#include <algorithm>
#include <cmath>
#include <numbers>
#include <optional>

double NormalizeTurn(double turn) {
  return std::remainder(turn, 2.0 * std::numbers::pi);
}

std::optional<DroneMotionResult>
CalculateNextDroneMotion(const DroneMotionConfig &config) {
  if (config.acceleration_path <= 0 || config.time_step <= 0 ||
      config.attack_speed <= 0 || config.current_speed < 0) {
    return std::nullopt;
  }

  const double acceleration = config.attack_speed * config.attack_speed /
                              (2.0 * config.acceleration_path);

  const double delta_speed = acceleration * config.time_step;

  double speed = config.current_speed;
  DroneState state = config.drone_state;

  switch (state) {
  case DroneState::ACCELERATING:
    speed += delta_speed;

    if (speed >= config.attack_speed) {
      speed = config.attack_speed;
      state = DroneState::MOVING;
    }

    break;

  case DroneState::DECELERATING:
    speed -= delta_speed;

    if (speed <= 0) {
      speed = 0;
      state = DroneState::STOPPED;
    }

    break;

  case DroneState::STOPPED:
  case DroneState::TURNING:
    speed = 0;
    break;

  case DroneState::MOVING:
    speed = config.attack_speed;
    break;
  }

  return DroneMotionResult{
      .new_speed = speed,
      .new_state = state,
  };
}

Position CalculateNextDronePosition(const DronePositionConfig &config) {
  const double distance = config.speed * config.time_step;

  return Position{
      .x = config.current_position.x + distance * std::cos(config.direction),

      .y = config.current_position.y + distance * std::sin(config.direction),
  };
}

double CalculateTurnAngle(const TurnAngleConfig &config) {
  const double desired_x = config.target_position.x - config.current_position.x;

  const double desired_y = config.target_position.y - config.current_position.y;

  const double desired_direction = std::atan2(desired_y, desired_x);

  return NormalizeTurn(desired_direction - config.current_direction);
}

TurnResult CalculateTurn(const TurnConfig &config) {
  const double max_turn_step = config.angular_speed * config.time_step;

  const double applied_turn =
      std::clamp(config.delta_angle, -max_turn_step, max_turn_step);

  const double new_direction =
      NormalizeTurn(config.current_direction + applied_turn);

  const bool turn_finished = std::abs(config.delta_angle) <= max_turn_step;

  return TurnResult{
      .new_direction = new_direction,
      .turn_finished = turn_finished,
  };
}

std::optional<double>
EstimateDroneArrivalTime(const DroneArrivalTimeConfig &config) {

  if (config.attack_speed <= 0) {
    return std::nullopt;
  }

  const double distance =
      MeasureDistance(config.current_position, config.firepoint);

  return distance / config.attack_speed;
}

std::optional<double>
CalculateSwitchPenalty(const SwitchPenaltyConfig &config) {
  switch (config.drone_state) {
  case DroneState::STOPPED:
    return 0;
  case DroneState::MOVING:
  case DroneState::ACCELERATING:
  case DroneState::DECELERATING:
    if (config.acceleration <= 0) {
      return std::nullopt;
    }

    return config.current_speed / config.acceleration;
  case DroneState::TURNING:
    if (config.angular_speed <= 0) {
      return std::nullopt;
    }

    return std::abs(config.remaining_turn_angle) / config.angular_speed;
  }

  return std::nullopt;
}
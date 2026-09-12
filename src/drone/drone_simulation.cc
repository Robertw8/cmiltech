#include "drone/drone_simulation.h"
#include "drone/drone_math.h"
#include <cmath>

namespace {

bool ApplyMotionStep(Drone &drone, const DroneSimulationConfig &config) {
  auto result = CalculateNextDroneMotion({
      .current_speed = drone.speed,
      .attack_speed = config.attack_speed,
      .acceleration_path = config.acceleration_path,
      .time_step = config.time_step,
      .drone_state = drone.state,
  });

  if (!result) {
    return false;
  }

  drone.speed = result->new_speed;
  drone.state = result->new_state;

  return true;
}

void ApplyPositionStep(Drone &drone, const DroneSimulationConfig &config) {
  drone.position = CalculateNextDronePosition({
      .current_position = drone.position,
      .speed = drone.speed,
      .direction = drone.direction,
      .time_step = config.time_step,
  });
}

double GetTurnAngle(const Drone &drone, const Position &target_position) {
  return CalculateTurnAngle({
      .current_position = drone.position,
      .target_position = target_position,
      .current_direction = drone.direction,
  });
}

TurnResult GetTurnResult(const Drone &drone, double turn_angle,
                         const DroneSimulationConfig &config) {
  return CalculateTurn({
      .current_direction = drone.direction,
      .delta_angle = turn_angle,
      .angular_speed = config.angular_speed,
      .time_step = config.time_step,
  });
}

} // namespace

std::optional<Drone> UpdateDroneStep(const Drone &drone,
                                     const Position &target_position,
                                     const DroneSimulationConfig &config) {
  if (config.time_step <= 0 || config.attack_speed <= 0 ||
      config.acceleration_path <= 0 || config.angular_speed <= 0 ||
      config.turn_threshold < 0) {
    return std::nullopt;
  }

  Drone next = drone;

  double turn_angle = GetTurnAngle(next, target_position);

  switch (next.state) {
  case DroneState::MOVING: {
    if (std::abs(turn_angle) > config.turn_threshold) {
      next.state = DroneState::DECELERATING;

      if (!ApplyMotionStep(next, config)) {
        return std::nullopt;
      }
    } else {
      const TurnResult turn = GetTurnResult(next, turn_angle, config);

      next.direction = turn.new_direction;
    }

    ApplyPositionStep(next, config);
    break;
  }

  case DroneState::DECELERATING: {
    if (!ApplyMotionStep(next, config)) {
      return std::nullopt;
    }

    if (next.speed > 0) {
      ApplyPositionStep(next, config);
      break;
    }

    turn_angle = GetTurnAngle(next, target_position);

    if (std::abs(turn_angle) > config.turn_threshold) {
      next.state = DroneState::TURNING;
    } else {
      const TurnResult turn = GetTurnResult(next, turn_angle, config);

      next.direction = turn.new_direction;
      next.state = DroneState::ACCELERATING;
    }

    break;
  }

  case DroneState::TURNING: {
    next.speed = 0;

    const TurnResult turn = GetTurnResult(next, turn_angle, config);

    next.direction = turn.new_direction;

    if (turn.turn_finished) {
      next.state = DroneState::ACCELERATING;
    }

    break;
  }

  case DroneState::ACCELERATING: {
    if (!ApplyMotionStep(next, config)) {
      return std::nullopt;
    }

    ApplyPositionStep(next, config);
    break;
  }

  case DroneState::STOPPED: {
    next.speed = 0;

    if (std::abs(turn_angle) > config.turn_threshold) {
      next.state = DroneState::TURNING;
    } else {
      const TurnResult turn = GetTurnResult(next, turn_angle, config);

      next.direction = turn.new_direction;
      next.state = DroneState::ACCELERATING;
    }

    break;
  }
  }

  return next;
}
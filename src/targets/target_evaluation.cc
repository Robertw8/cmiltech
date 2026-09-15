#include "targets/target_evaluation.h"
#include "drone/drone_math.h"
#include "simulation/simulation.h"
#include "simulation/simulation_math.h"
#include "targets/target_tracking.h"

std::optional<TargetEvaluation>
EvaluateTarget(const TargetEvaluationConfig &config) {
  auto flight_time_result = GetFlightTime(config.motion_params);

  if (!flight_time_result) {
    return std::nullopt;
  }

  const double ammo_flight_time = *flight_time_result;

  auto horizontal_distance_result =
      CalculateHorizontalDistance(config.motion_params, ammo_flight_time);

  if (!horizontal_distance_result) {
    return std::nullopt;
  }

  const double horizontal_distance = *horizontal_distance_result;
  const double euclidean_distance =
      MeasureDistance(config.drone.position, config.current_target_position);

  auto simulation_result =
      CalculateSimulation({.drone_position = config.drone.position,
                           .target_position = config.current_target_position,
                           .acceleration_path = config.acceleration_path,
                           .horizontal_distance = horizontal_distance,
                           .euclidean_distance = euclidean_distance});

  if (!simulation_result) {
    return std::nullopt;
  }

  const Simulation simulation = *simulation_result;

  auto drone_arrival_time_result =
      EstimateDroneArrivalTime({.current_position = config.drone.position,
                                .firepoint = simulation.firepoint,
                                .attack_speed = config.attack_speed});

  if (!drone_arrival_time_result) {
    return std::nullopt;
  }

  const double drone_arrival_time = *drone_arrival_time_result;

  auto target_velocity_result = CalculateTargetVelocity(
      {.current_position = config.current_target_position,
       .next_position = config.next_target_position,
       .time_step = config.velocity_time_step});

  if (!target_velocity_result) {
    return std::nullopt;
  }

  const TargetVelocity target_velocity = *target_velocity_result;

  const double future_time = drone_arrival_time + ammo_flight_time;

  const Position predicted_target_position =
      PredictTargetPosition({.current_position = config.current_target_position,
                             .velocity = target_velocity,
                             .future_time = future_time});

  const double predicted_distance =
      MeasureDistance(config.drone.position, predicted_target_position);

  auto predicted_simulation_result =
      CalculateSimulation({.drone_position = config.drone.position,
                           .target_position = predicted_target_position,
                           .acceleration_path = config.acceleration_path,
                           .horizontal_distance = horizontal_distance,
                           .euclidean_distance = predicted_distance});

  if (!predicted_simulation_result) {
    return std::nullopt;
  }

  const Simulation predicted_simulation = *predicted_simulation_result;

  auto final_arrival_time_result =
      EstimateDroneArrivalTime({.current_position = config.drone.position,
                                .firepoint = predicted_simulation.firepoint,
                                .attack_speed = config.attack_speed});

  if (!final_arrival_time_result) {
    return std::nullopt;
  }

  const double final_arrival_time = *final_arrival_time_result;

  return TargetEvaluation{
      .predicted_position = predicted_target_position,
      .firepoint = predicted_simulation.firepoint,
      .ammo_flight_time = ammo_flight_time,
      .drone_arrival_time = final_arrival_time,
  };
}
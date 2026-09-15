#include "simulation/simulation.h"
#include <cmath>

namespace {

Position CalculateIntermediatePoint(const SimulationConfig &config) {
  const double distance = config.horizontal_distance + config.acceleration_path;

  const double ratio = distance / config.euclidean_distance;

  return Position{
      .x = config.target_position.x -
           (config.target_position.x - config.drone_position.x) * ratio,

      .y = config.target_position.y -
           (config.target_position.y - config.drone_position.y) * ratio,
  };
}

std::optional<Position> CalculateFirePoint(const SimulationConfig &config) {
  if (config.euclidean_distance <= 0 ||
      !std::isfinite(config.euclidean_distance)) {
    return std::nullopt;
  }

  const double ratio =
      (config.euclidean_distance - config.horizontal_distance) /
      config.euclidean_distance;

  return Position{
      .x = config.drone_position.x +
           (config.target_position.x - config.drone_position.x) * ratio,

      .y = config.drone_position.y +
           (config.target_position.y - config.drone_position.y) * ratio,
  };
}

} // namespace

std::optional<Simulation> CalculateSimulation(const SimulationConfig &config) {
  auto firepoint_result = CalculateFirePoint(config);

  if (!firepoint_result) {
    return std::nullopt;
  }

  Simulation result{
      .firepoint = *firepoint_result,
      .intermediate_point = std::nullopt,
  };

  if (config.horizontal_distance + config.acceleration_path >
      config.euclidean_distance) {

    result.intermediate_point = CalculateIntermediatePoint(config);
  }

  return result;
}
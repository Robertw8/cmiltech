#pragma once

#include "types.h"
#include <optional>

struct SimulationConfig {
  Position drone_position;
  Position target_position;

  double acceleration_path;
  double horizontal_distance;
  double euclidean_distance;
};

struct Simulation {
  Position firepoint;
  std::optional<Position> intermediate_point;
};

std::optional<Simulation> CalculateSimulation(const SimulationConfig &config);
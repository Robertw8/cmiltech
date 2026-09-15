#pragma once

#include "drone_types.h"
#include <optional>

struct DroneSimulationConfig {
  double time_step;
  double attack_speed;
  double acceleration_path;
  double angular_speed;
  double turn_threshold;
};

std::optional<Drone> UpdateDroneStep(const Drone &drone,
                                     const Position &target_position,
                                     const DroneSimulationConfig &config);
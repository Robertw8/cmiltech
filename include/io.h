#pragma once
#include "simulation/simulation.h"
#include "types.h"

#include <string>

struct InputData {
  double xd;
  double yd;
  double zd;

  double initial_direction;
  double attack_speed;
  double acceleration_path;

  std::string ammo_name;

  double array_time_step;
  double hit_radius;
  double angular_speed;
  double turn_threshold;
};

InputData ReadInput(const std::string &filename);

TargetTracks ReadTargetTracks(const std::string &filename);

void CreateOutput(const SimulationResult &simulation_result);
#pragma once
#include "simulation.h"
#include <string>
using namespace std;

struct InputData {
  double xd;
  double yd;
  double zd;

  double target_x;
  double target_y;

  double attack_speed;
  double acceleration_path;

  string ammo_name;
};

InputData ReadInput(const string &filename);

void CreateOutput(const SimulationResult &simulation_result);
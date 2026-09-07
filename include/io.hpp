#pragma once
#include "simulation.hpp"
#include <string>

struct InputData {
  double x;
  double y;
  double z;

  double targetX;
  double targetY;

  double attackSpeed;
  double accelerationPath;

  std::string ammoName;
};

InputData readInput(const std::string &filename);

void createOutput(const SimulationResult &simulationResult);
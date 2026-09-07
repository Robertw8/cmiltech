#include "io.hpp"
#include "simulation.hpp"
#include <fstream>
#include <stdexcept>
#include <string>

InputData readInput(const std::string &filename) {
  std::ifstream input(filename);

  if (!input) {
    throw std::runtime_error("File not found");
  }

  InputData inputData{};

  if (!(input >> inputData.x >> inputData.y >> inputData.z >>
        inputData.targetX >> inputData.targetY >> inputData.attackSpeed >>
        inputData.accelerationPath >> inputData.ammoName)) {
    throw std::runtime_error("Invalid input format");
  }

  return inputData;
}

void createOutput(const SimulationResult &simulationResult) {
  std::ofstream output("output.txt");

  if (!output) {
    throw std::runtime_error("Cannot create output file");
  }

  output << simulationResult.destinationCoords.x << ' '
         << simulationResult.destinationCoords.y << "\n";

  if (simulationResult.intermediatePoint) {
    output << simulationResult.intermediatePoint->x << ' '
           << simulationResult.intermediatePoint->y << "\n";
  }
}
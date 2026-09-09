#include "io.h"
#include "simulation.h"
#include <fstream>
#include <stdexcept>
#include <string>

InputData ReadInput(const string &filename) {
  ifstream input(filename);

  if (!input) {
    throw runtime_error("File not found");
  }

  InputData input_data{};

  if (!(input >> input_data.xd >> input_data.yd >> input_data.zd >>
        input_data.target_x >> input_data.target_y >> input_data.attack_speed >>
        input_data.acceleration_path >> input_data.ammo_name)) {
    throw runtime_error("Invalid input format");
  }

  return input_data;
}

void CreateOutput(const SimulationResult &simulation_result) {
  ofstream output("output.txt");

  if (!output) {
    throw runtime_error("Cannot create output file");
  }

  output << simulation_result.firepoint.x << ' '
         << simulation_result.firepoint.y << "\n";

  if (simulation_result.intermediate_point) {
    output << simulation_result.intermediate_point->x << ' '
           << simulation_result.intermediate_point->y << "\n";
  }
}
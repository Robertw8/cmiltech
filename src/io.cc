#include "io.h"
#include <fstream>
#include <stdexcept>

InputData ReadInput(const std::string &filename) {
  std::ifstream input(filename);

  if (!input) {
    throw std::runtime_error("File not found");
  }

  InputData input_data{};

  if (!(input >> input_data.xd >> input_data.yd >> input_data.zd >>
        input_data.initial_direction >> input_data.attack_speed >>
        input_data.acceleration_path >> input_data.ammo_name >>
        input_data.array_time_step >> input_data.hit_radius >>
        input_data.angular_speed >> input_data.turn_threshold)) {

    throw std::runtime_error("Invalid input format");
  }

  return input_data;
}

TargetTracks ReadTargetTracks(const std::string &filename) {
  std::ifstream input(filename);

  if (!input) {
    throw std::runtime_error("File not found");
  }

  TargetTracks target_tracks{};

  for (auto &track : target_tracks) {
    for (auto &position : track) {
      if (!(input >> position.x >> position.y)) {
        throw std::runtime_error("Invalid target tracks");
      }
    }
  }

  return target_tracks;
}

void CreateOutput(const Simulation &simulation) {
  std::ofstream output("output.txt");

  if (!output) {
    throw std::runtime_error("Cannot create output file");
  }

  output << simulation.firepoint.x << ' ' << simulation.firepoint.y << '\n';

  if (simulation.intermediate_point) {
    output << simulation.intermediate_point->x << ' '
           << simulation.intermediate_point->y << '\n';
  }
}
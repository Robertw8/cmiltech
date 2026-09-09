#include "ammo.h"
#include "io.h"
#include "simulation.h"
#include "types.h"
#include <iostream>

int main() {
  try {
    InputData input = ReadInput("input.txt");

    const Ammo *ammo = FindAmmo(input.ammo_name);

    if (ammo == nullptr) {
      cerr << "Ammo not found\n";
      return 1;
    }

    const MotionParams motion_params = {
        .mass = ammo->mass,
        .drag = ammo->drag,
        .lift = ammo->lift,
        .speed = input.attack_speed,
        .height = input.zd,
    };
    const Coords target_coords = {.x = input.target_x, .y = input.target_y};
    const Coords drone_coords = {.x = input.xd, .y = input.yd};

    auto time_result = GetFlightTime(motion_params);

    if (!time_result) {
      cerr << "Invalid flight time\n";
      return 1;
    }

    const double time = *time_result;
    auto horizontal_distance_result =
        CalculateHorizontalDistance(motion_params, time);

    if (!horizontal_distance_result) {
      cerr << "Invalid horizontal distance\n";
      return 1;
    }

    const double horizontal_distance = *horizontal_distance_result;
    const double euclidean_distance =
        MeasureDistance(drone_coords, target_coords);
    const double acceleration_path = input.acceleration_path;

    auto simulation_result =
        CalculateSimulation(drone_coords, target_coords, acceleration_path,
                            horizontal_distance, euclidean_distance);

    if (!simulation_result) {
      cerr << "Invalid destination coordinates\n";
      return 1;
    }

    const SimulationResult simulation = *simulation_result;

    CreateOutput(simulation);

    return 0;
  } catch (const exception &error) {
    cerr << error.what() << '\n';
    return 1;
  }
}

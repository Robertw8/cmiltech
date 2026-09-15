#include "ammo.h"
#include "io.h"
#include "simulation/simulation.h"
#include "simulation/simulation_math.h"
#include "targets/target_tracking.h"
#include <cstddef>
#include <iostream>

int main() {
  try {
    const InputData input = ReadInput("input.txt");

    const TargetTracks target_tracks = ReadTargetTracks("targets.txt");

    const Ammo *ammo = FindAmmo(input.ammo_name);

    if (ammo == nullptr) {
      std::cerr << "Ammo not found\n";
      return 1;
    }

    const MotionParams motion_params{
        .mass = ammo->mass,
        .drag = ammo->drag,
        .lift = ammo->lift,
        .speed = input.attack_speed,
        .height = input.zd,
    };

    auto target_positions_result = GetTargetPositionsAtTime({
        .target_tracks = target_tracks,
        .time = 0.0,
        .time_step = input.array_time_step,
    });

    if (!target_positions_result) {
      std::cerr << "Invalid target tracks\n";
      return 1;
    }

    constexpr std::size_t kInitialTargetIndex = 0;

    const Position target_position =
        (*target_positions_result)[kInitialTargetIndex];

    const Position drone_position{
        .x = input.xd,
        .y = input.yd,
    };

    auto time_result = GetFlightTime(motion_params);

    if (!time_result) {
      std::cerr << "Invalid flight time\n";
      return 1;
    }

    const double ammo_flight_time = *time_result;

    auto horizontal_distance_result =
        CalculateHorizontalDistance(motion_params, ammo_flight_time);

    if (!horizontal_distance_result) {
      std::cerr << "Invalid horizontal distance\n";
      return 1;
    }

    const double horizontal_distance = *horizontal_distance_result;

    const double euclidean_distance =
        MeasureDistance(drone_position, target_position);

    const SimulationConfig simulation_config{
        .drone_position = drone_position,
        .target_position = target_position,
        .acceleration_path = input.acceleration_path,
        .horizontal_distance = horizontal_distance,
        .euclidean_distance = euclidean_distance,
    };

    auto simulation_result = CalculateSimulation(simulation_config);

    if (!simulation_result) {
      std::cerr << "Invalid destination coordinates\n";
      return 1;
    }

    CreateOutput(*simulation_result);

    return 0;

  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
#pragma once
#include "types.h"
#include <optional>
using namespace std;

double MeasureDistance(const Coords &point_a, const Coords &point_b);

optional<double> GetFlightTime(const MotionParams &motion_params);

optional<double> CalculateHorizontalDistance(const MotionParams &motion_params,
                                             double time);

struct SimulationResult {
  Coords firepoint;
  optional<Coords> intermediate_point;
};

optional<SimulationResult> CalculateSimulation(const Coords &drone_coords,
                                               const Coords &target_coords,
                                               double acceleration_path,
                                               double horizontal_distance,
                                               double euclidean_distance);
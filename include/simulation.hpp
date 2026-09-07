#pragma once
#include "types.hpp"
#include <optional>

double measureDistance(const Coords &pointA, const Coords &pointB);

std::optional<double> getFlightTime(const MotionParams &motionParams);

std::optional<double>
calculateHorizontalDistance(const MotionParams &motionParams, double time);

struct SimulationResult {
  Coords destinationCoords;
  std::optional<Coords> intermediatePoint;
};

std::optional<SimulationResult> calculateSimulation(const Coords &droneCoords,
                                                    const Coords &targetCoords,
                                                    double accelerationPath,
                                                    double horizontalDistance,
                                                    double euclideanDistance);
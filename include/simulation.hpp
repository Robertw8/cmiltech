#pragma once
#include "types.hpp"
#include <optional>

double measureDistance(const Coords &pointA, const Coords &pointB);

std::optional<double> getFlightTime(const MotionParams &motionParams);

std::optional<double>
calculateHorizontalDistance(const MotionParams &motionParams, double time);

Coords calculateDestinationCoords(const Coords &droneCoords,
                                  const Coords &targetCoords,
                                  double horizontalDistance,
                                  double euclideanDistance,
                                  double accelerationPath);
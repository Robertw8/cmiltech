#pragma once
#include "types.h"
#include <optional>

double MeasureDistance(const Position &point_a, const Position &point_b);

std::optional<double> GetFlightTime(const MotionParams &motion_params);

std::optional<double>
CalculateHorizontalDistance(const MotionParams &motion_params, double time);
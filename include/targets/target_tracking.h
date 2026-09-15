#pragma once

#include "types.h"
#include <optional>

struct TargetTrackingConfig {
  const TargetTracks &target_tracks;
  double time;
  double time_step;
};

struct TargetVelocity {
  double vx;
  double vy;
};

struct TargetVelocityConfig {
  const Position &current_position;
  const Position &next_position;
  double time_step;
};

struct TargetPredictionConfig {
  const Position &current_position;
  const TargetVelocity &velocity;
  double future_time;
};

std::optional<TargetVelocity>
CalculateTargetVelocity(const TargetVelocityConfig &config);

std::optional<TargetPositions>
GetTargetPositionsAtTime(const TargetTrackingConfig &config);

Position PredictTargetPosition(const TargetPredictionConfig &config);
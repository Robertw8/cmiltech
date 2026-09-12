#pragma once
#include "types.h"
#include <optional>

struct TargetTrackingConfig {
  const TargetTracks &target_tracks;
  double time;
  double time_step;
};

std::optional<TargetPositions>
GetTargetPositionsAtTime(const TargetTrackingConfig &config);
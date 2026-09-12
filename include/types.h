#pragma once

#include <array>
#include <cstddef>

struct Position {
  double x;
  double y;
};

struct MotionParams {
  double mass;
  double drag;
  double lift;
  double speed;
  double height;
};

inline constexpr std::size_t kTargetCount = 5;
inline constexpr std::size_t kTrackPointCount = 60;

using TargetTrack = std::array<Position, kTrackPointCount>;
using TargetTracks = std::array<TargetTrack, kTargetCount>;
using TargetPositions = std::array<Position, kTargetCount>;
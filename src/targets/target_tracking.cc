#include "targets/target_tracking.h"

namespace {

std::optional<Position> InterpolateTargetPosition(const TargetTrack &track,
                                                  double time,
                                                  double time_step) {
  if (time_step <= 0 || time < 0 || track.empty()) {
    return std::nullopt;
  }

  const double step = time / time_step;

  const auto whole_step = static_cast<std::size_t>(step);

  const auto index = whole_step % track.size();

  const auto next_index = (index + 1) % track.size();

  const double fraction = step - static_cast<double>(whole_step);

  const double x =
      track[index].x + (track[next_index].x - track[index].x) * fraction;

  const double y =
      track[index].y + (track[next_index].y - track[index].y) * fraction;

  return Position{x, y};
}

} // namespace

std::optional<TargetPositions>
GetTargetPositionsAtTime(const TargetTrackingConfig &config) {
  if (config.time_step <= 0 || config.time < 0) {
    return std::nullopt;
  }

  TargetPositions positions{};

  for (std::size_t i = 0; i < config.target_tracks.size(); ++i) {
    auto result = InterpolateTargetPosition(config.target_tracks[i],
                                            config.time, config.time_step);

    if (!result) {
      return std::nullopt;
    }

    positions[i] = *result;
  }

  return positions;
}
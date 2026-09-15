#include "targets/target_selection.h"
#include "drone/drone_math.h"
#include "targets/target_evaluation.h"
#include "types.h"
#include <optional>

std::optional<SelectedTarget>
SelectBestTarget(const TargetSelectionConfig &config) {
  std::optional<SelectedTarget> best_target = std::nullopt;
  std::array<std::optional<TargetEvaluation>, kTargetCount> evaluations{};

  for (std::size_t i = 0; i < config.current_target_positions.size(); ++i) {
    const Position current_position = config.current_target_positions[i];
    const Position next_position = config.next_target_positions[i];

    evaluations[i] = EvaluateTarget({
        .drone = config.drone,
        .current_target_position = current_position,
        .next_target_position = next_position,
        .motion_params = config.motion_params,
        .velocity_time_step = config.velocity_time_step,
        .attack_speed = config.attack_speed,
        .acceleration_path = config.acceleration_path,
    });
  }

  double switch_penalty = 0;

  if (config.drone.target_index >= 0) {
    const std::size_t current_target_index =
        static_cast<std::size_t>(config.drone.target_index);

    if (current_target_index >= evaluations.size()) {
      return std::nullopt;
    }

    double remaining_turn_angle = 0;

    if (config.drone.state == DroneState::TURNING) {
      if (!evaluations[current_target_index]) {
        return std::nullopt;
      }

      remaining_turn_angle = CalculateTurnAngle({
          .current_position = config.drone.position,
          .target_position = evaluations[current_target_index]->firepoint,
          .current_direction = config.drone.direction,
      });
    }

    if (config.acceleration_path <= 0 || config.attack_speed <= 0) {
      return std::nullopt;
    }

    const double acceleration = (config.attack_speed * config.attack_speed) /
                                (2.0 * config.acceleration_path);

    auto switch_penalty_result = CalculateSwitchPenalty({
        .drone_state = config.drone.state,
        .current_speed = config.drone.speed,
        .acceleration = acceleration,
        .remaining_turn_angle = remaining_turn_angle,
        .angular_speed = config.angular_speed,
    });

    if (!switch_penalty_result) {
      return std::nullopt;
    }

    switch_penalty = *switch_penalty_result;
  }

  for (std::size_t i = 0; i < evaluations.size(); ++i) {
    if (!evaluations[i]) {
      continue;
    }

    const TargetEvaluation &evaluation = *evaluations[i];

    double candidate_time = evaluation.drone_arrival_time;

    if (config.drone.target_index >= 0 &&
        static_cast<int>(i) != config.drone.target_index) {
      candidate_time += switch_penalty;
    }

    if (!best_target || candidate_time < best_target->effective_time) {
      best_target = SelectedTarget{
          .firepoint = evaluation.firepoint,
          .target_index = static_cast<int>(i),
          .effective_time = candidate_time,
      };
    }
  }

  return best_target;
}
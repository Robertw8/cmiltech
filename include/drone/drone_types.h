#pragma once

#include "types.h"

enum class DroneState { STOPPED, ACCELERATING, DECELERATING, TURNING, MOVING };

struct Drone {
  Position position{};
  double speed = 0.0;
  double direction = 0.0;
  DroneState state = DroneState::STOPPED;
  int target_index = -1;
};
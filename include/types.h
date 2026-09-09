#pragma once

struct Coords {
  double x;
  double y;
};

struct CubicCoefficients {
  double a;
  double b;
  double c;
};

struct MotionParams {
  double mass;
  double drag;
  double lift;
  double speed;
  double height;
};

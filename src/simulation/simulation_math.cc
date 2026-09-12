#include "simulation/simulation_math.h"
#include <cmath>
#include <numbers>

namespace {

constexpr double kGravity = 9.81;

struct CubicCoefficients {
  double a;
  double b;
  double c;
};

std::optional<double> CalculateP(const CubicCoefficients &coefficients) {
  if (coefficients.a == 0) {
    return std::nullopt;
  }

  return -(coefficients.b * coefficients.b) /
         (3.0 * coefficients.a * coefficients.a);
}

std::optional<double> CalculatePhi(double p, double q) {
  if (p >= 0) {
    return std::nullopt;
  }

  const double acos_argument = ((3.0 * q) / (2.0 * p)) * std::sqrt(-3.0 / p);

  if (acos_argument < -1.0 || acos_argument > 1.0) {
    return std::nullopt;
  }

  return std::acos(acos_argument);
}

CubicCoefficients CalculateCoefficients(const MotionParams &params) {
  const double mass = params.mass;
  const double drag = params.drag;
  const double lift = params.lift;
  const double speed = params.speed;
  const double height = params.height;

  const double a = drag * kGravity * mass - 2.0 * drag * drag * lift * speed;

  const double b =
      -3.0 * kGravity * mass * mass + 3.0 * drag * lift * mass * speed;

  const double c = 6.0 * mass * mass * height;

  return {a, b, c};
}

std::optional<double> CalculateRoot(const CubicCoefficients &coefficients) {
  const double a = coefficients.a;
  const double b = coefficients.b;
  const double c = coefficients.c;

  auto p_result = CalculateP(coefficients);

  if (!p_result) {
    return std::nullopt;
  }

  const double p = *p_result;

  const double q = (2.0 * b * b * b) / (27.0 * a * a * a) + c / a;

  auto phi_result = CalculatePhi(p, q);

  if (!phi_result) {
    return std::nullopt;
  }

  const double phi = *phi_result;

  const double root = 2.0 * std::sqrt(-p / 3.0) *
                          std::cos((phi + 4.0 * std::numbers::pi) / 3.0) -
                      b / (3.0 * a);

  return root;
}

} // namespace

double MeasureDistance(const Position &point_a, const Position &point_b) {
  return std::hypot(point_b.x - point_a.x, point_b.y - point_a.y);
}

std::optional<double> GetFlightTime(const MotionParams &motion_params) {
  const CubicCoefficients coefficients = CalculateCoefficients(motion_params);

  auto root_result = CalculateRoot(coefficients);

  if (!root_result) {
    return std::nullopt;
  }

  const double root = *root_result;

  if (!std::isfinite(root) || root <= 0) {
    return std::nullopt;
  }

  return root;
}

std::optional<double>
CalculateHorizontalDistance(const MotionParams &motion_params, double time) {
  const double mass = motion_params.mass;
  const double drag = motion_params.drag;
  const double lift = motion_params.lift;
  const double speed = motion_params.speed;

  if (mass <= 0 || speed <= 0 || time <= 0) {
    return std::nullopt;
  }

  const double time_2 = time * time;
  const double time_3 = time_2 * time;
  const double time_4 = time_3 * time;
  const double time_5 = time_4 * time;

  const double mass_2 = mass * mass;
  const double mass_3 = mass_2 * mass;
  const double mass_4 = mass_3 * mass;

  const double drag_2 = drag * drag;
  const double drag_3 = drag_2 * drag;
  const double drag_4 = drag_3 * drag;

  const double lift_2 = lift * lift;
  const double lift_3 = lift_2 * lift;
  const double lift_4 = lift_3 * lift;

  const double lift_factor = 1.0 + lift_2;

  const double term_1 = time_2 * drag * speed / (2.0 * mass);

  const double term_2 = time_3 *
                        (6.0 * drag * kGravity * lift * mass -
                         6.0 * drag_2 * (lift_2 - 1.0) * speed) /
                        (36.0 * mass_2);

  const double term_3 =
      time_4 *
      (-6.0 * drag_2 * kGravity * lift * (1.0 + lift_2 + lift_4) * mass +

       3.0 * drag_3 * lift_2 * (1.0 + lift_2) * speed +

       6.0 * drag_3 * lift_4 * (1.0 + lift_2) * speed) /
      (36.0 * lift_factor * lift_factor * mass_3);

  const double term_4 = time_5 *
                        (3.0 * drag_3 * kGravity * lift_3 * mass -
                         3.0 * drag_4 * lift_2 * lift_factor * speed) /
                        (36.0 * lift_factor * mass_4);

  const double distance = speed * time - term_1 + term_2 + term_3 + term_4;

  if (!std::isfinite(distance) || distance <= 0) {
    return std::nullopt;
  }

  return distance;
}
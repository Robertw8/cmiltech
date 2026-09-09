#include "simulation.h"
#include "types.h"
#include <cmath>
#include <numbers>

constexpr double kGravity = 9.81;

double MeasureDistance(const Coords &point_a, const Coords &point_b) {
  double diff_x = point_b.x - point_a.x;
  double diff_y = point_b.y - point_a.y;

  return sqrt(diff_x * diff_x + diff_y * diff_y);
}

optional<double> CalculateP(const CubicCoefficients &coefficients) {
  double a = coefficients.a;
  double b = coefficients.b;

  if (a == 0) {
    return nullopt;
  }

  return -(b * b) / (3.0 * a * a);
}

optional<double> CalculatePhi(double p, double q) {
  if (p >= 0) {
    return nullopt;
  }

  double acos_arg = ((3.0 * q) / (2.0 * p)) * sqrt(-3.0 / p);

  if (acos_arg < -1.0 || acos_arg > 1.0) {
    return nullopt;
  }

  return acos(acos_arg);
}

CubicCoefficients CalculateCoefficients(const MotionParams &motion_params) {
  double mass = motion_params.mass;
  double drag = motion_params.drag;
  double lift = motion_params.lift;
  double speed = motion_params.speed;
  double height = motion_params.height;

  double a = drag * kGravity * mass - 2.0 * (drag * drag) * lift * speed;
  double b = -3.0 * kGravity * (mass * mass) + 3.0 * drag * lift * mass * speed;
  double c = 6.0 * (mass * mass) * height;

  return {a, b, c};
}

optional<double> CalculateRoot(const CubicCoefficients &coefficients) {
  double a = coefficients.a;
  double b = coefficients.b;
  double c = coefficients.c;

  auto p_result = CalculateP(coefficients);

  if (!p_result) {
    return nullopt;
  }

  double p = *p_result;

  double q = (2.0 * b * b * b) / (27.0 * a * a * a) + c / a;

  auto phi_result = CalculatePhi(p, q);

  if (!phi_result) {
    return nullopt;
  }

  double phi = *phi_result;

  double root = 2.0 * sqrt(-p / 3.0) * cos((phi + 4.0 * numbers::pi) / 3.0) -
                b / (3.0 * a);

  return root;
}

optional<double> GetFlightTime(const MotionParams &motion_params) {
  const CubicCoefficients coefficients = CalculateCoefficients(motion_params);
  auto root_result = CalculateRoot(coefficients);

  if (!root_result) {
    return nullopt;
  }

  double root = *root_result;

  if (root <= 0) {
    return nullopt;
  }

  return root;
}

optional<double> CalculateHorizontalDistance(const MotionParams &motion_params,
                                             double time) {
  const double mass = motion_params.mass;
  const double drag = motion_params.drag;
  const double lift = motion_params.lift;
  const double speed = motion_params.speed;

  if (mass <= 0 || speed <= 0 || time <= 0) {
    return nullopt;
  }

  const double time_2 = time * time;
  const double time_3 = time_2 * time;
  const double time_4 = time_3 * time;
  const double time_5 = time_4 * time;

  const double mass_2 = mass * mass;
  const double mass_3 = mass_2 * mass;
  const double mass_4 = mass_3 * mass;

  const double drag_2 = drag * drag;
  const double drag_3 = drag_3 * drag;
  const double drag_4 = drag_3 * drag;

  const double lift_2 = lift * lift;
  const double lift_3 = lift_2 * lift;
  const double lift_4 = lift_3 * lift;

  const double term_1 = (time_2 * drag * speed / (2.0 * mass));

  const double term_2 = (time_3 * (6.0 * drag * kGravity * lift * mass -
                                   6.0 * drag_2 * (lift_2 - 1.0) * speed)) /
                        (36.0 * mass_2);

  const double term_3 =
      time_4 *
      (-6.0 * drag_2 * kGravity * lift * (1.0 + lift_2 + lift_4) * mass +
       3.0 * drag_3 * lift_2 * (1.0 + lift_2) * speed +
       6.0 * drag_3 * lift_4 * (1.0 + lift_2) * speed) /
      (36.0 * pow(1.0 + lift_2, 2.0) * mass_3);

  const double term_4 = time_5 *
                        (3.0 * drag_3 * kGravity * lift_3 * mass -
                         3.0 * drag_4 * lift_2 * (1.0 + lift_2) * speed) /
                        (36.0 * (1.0 + lift_2) * mass_4);

  const double distance = speed * time - term_1 + term_2 + term_3 + term_4;

  if (!isfinite(distance) || distance <= 0) {
    return nullopt;
  }

  return distance;
}

Coords CalculateIntermediatePoint(const Coords &drone_coords,
                                  const Coords &target_coords,
                                  double acceleration_path,
                                  double horizontal_distance,
                                  double euclidean_distance) {
  const double x =
      target_coords.x - (target_coords.x - drone_coords.x) *
                            (horizontal_distance + acceleration_path) /
                            euclidean_distance;

  const double y =
      target_coords.y - (target_coords.y - drone_coords.y) *
                            (horizontal_distance + acceleration_path) /
                            euclidean_distance;

  return {x, y};
}

optional<Coords> CalculateFirePoint(const Coords &drone_coords,
                                    const Coords &target_coords,
                                    double horizontal_distance,
                                    double euclidean_distance) {

  if (euclidean_distance <= 0 || !isfinite(euclidean_distance)) {
    return nullopt;
  }

  const double ratio =
      (euclidean_distance - horizontal_distance) / euclidean_distance;

  const double firepoint_x =
      drone_coords.x + (target_coords.x - drone_coords.x) * ratio;
  const double firepoint_y =
      drone_coords.y + (target_coords.y - drone_coords.y) * ratio;

  return Coords{firepoint_x, firepoint_y};
}

// Orchestration
optional<SimulationResult> calculateSimulation(const Coords &drone_coords,
                                               const Coords &target_coords,
                                               double acceleration_path,
                                               double horizontal_distance,
                                               double euclidean_distance) {
  auto firepoint_result = CalculateFirePoint(
      drone_coords, target_coords, horizontal_distance, euclidean_distance);

  if (!firepoint_result) {
    return nullopt;
  }

  const Coords firepoint = *firepoint_result;

  if (horizontal_distance + acceleration_path > euclidean_distance) {
    const Coords intermediate_point = CalculateIntermediatePoint(
        drone_coords, target_coords, acceleration_path, horizontal_distance,
        euclidean_distance);

    return SimulationResult{firepoint, intermediate_point};
  }

  return SimulationResult{firepoint};
}
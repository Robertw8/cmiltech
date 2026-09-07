#include "simulation.hpp"
#include "types.hpp"
#include <cmath>
#include <numbers>

constexpr double gravity = 9.81;

double measureDistance(const Coords &pointA, const Coords &pointB) {
  double diffX = pointB.x - pointA.x;
  double diffY = pointB.y - pointA.y;

  return std::sqrt(diffX * diffX + diffY * diffY);
}

std::optional<double> calculateP(const CubicCoefficients &coefficients) {
  double a = coefficients.a;
  double b = coefficients.b;

  if (a == 0) {
    return std::nullopt;
  }

  return -(b * b) / (3.0 * a * a);
}

std::optional<double> calculatePhi(double p, double q) {
  if (p >= 0) {
    return std::nullopt;
  }

  double acosArg = ((3.0 * q) / (2.0 * p)) * std::sqrt(-3.0 / p);

  if (acosArg < -1.0 || acosArg > 1.0) {
    return std::nullopt;
  }

  return std::acos(acosArg);
}

CubicCoefficients calculateCoefficients(const MotionParams &motionParams) {
  double mass = motionParams.mass;
  double drag = motionParams.drag;
  double lift = motionParams.lift;
  double speed = motionParams.speed;
  double height = motionParams.height;

  double a = drag * gravity * mass - 2.0 * (drag * drag) * lift * speed;
  double b = -3.0 * gravity * (mass * mass) + 3.0 * drag * lift * mass * speed;
  double c = 6.0 * (mass * mass) * height;

  return {a, b, c};
}

std::optional<double> calculateRoot(const CubicCoefficients &coefficients) {
  double a = coefficients.a;
  double b = coefficients.b;
  double c = coefficients.c;

  auto pResult = calculateP(coefficients);

  if (!pResult) {
    return std::nullopt;
  }

  double p = *pResult;

  double q = (2.0 * b * b * b) / (27.0 * a * a * a) + c / a;

  auto phiResult = calculatePhi(p, q);

  if (!phiResult) {
    return std::nullopt;
  }

  double phi = *phiResult;

  double root = 2.0 * std::sqrt(-p / 3.0) *
                    std::cos((phi + 4.0 * std::numbers::pi) / 3.0) -
                b / (3.0 * a);

  return root;
}

std::optional<double> getFlightTime(const MotionParams &motionParams) {
  const CubicCoefficients coefficients = calculateCoefficients(motionParams);
  auto rootResult = calculateRoot(coefficients);

  if (!rootResult) {
    return std::nullopt;
  }

  double root = *rootResult;

  if (root <= 0) {
    return std::nullopt;
  }

  return root;
}

std::optional<double>
calculateHorizontalDistance(const MotionParams &motionParams, double time) {
  const double mass = motionParams.mass;
  const double drag = motionParams.drag;
  const double lift = motionParams.lift;
  const double speed = motionParams.speed;

  if (mass <= 0 || speed <= 0 || time <= 0) {
    return std::nullopt;
  }

  const double time2 = time * time;
  const double time3 = time2 * time;
  const double time4 = time3 * time;
  const double time5 = time4 * time;

  const double mass2 = mass * mass;
  const double mass3 = mass2 * mass;
  const double mass4 = mass3 * mass;

  const double drag2 = drag * drag;
  const double drag3 = drag2 * drag;
  const double drag4 = drag3 * drag;

  const double lift2 = lift * lift;
  const double lift3 = lift2 * lift;
  const double lift4 = lift3 * lift;

  const double term1 = (time2 * drag * speed / (2.0 * mass));

  const double term2 = (time3 * (6.0 * drag * gravity * lift * mass -
                                 6.0 * drag2 * (lift2 - 1.0) * speed)) /
                       (36.0 * mass2);

  const double term3 =
      time4 *
      (-6.0 * drag2 * gravity * lift * (1.0 + lift2 + lift4) * mass +
       3.0 * drag3 * lift2 * (1.0 + lift2) * speed +
       6.0 * drag3 * lift4 * (1.0 + lift2) * speed) /
      (36.0 * std::pow(1.0 + lift2, 2.0) * mass3);

  const double term4 = time5 *
                       (3.0 * drag3 * gravity * lift3 * mass -
                        3.0 * drag4 * lift2 * (1.0 + lift2) * speed) /
                       (36.0 * (1.0 + lift2) * mass4);

  const double distance = speed * time - term1 + term2 + term3 + term4;

  if (!std::isfinite(distance) || distance <= 0) {
    return std::nullopt;
  }

  return distance;
}

Coords calculateIntermediatePoint(const Coords &droneCoords,
                                  const Coords &targetCoords,
                                  double accelerationPath,
                                  double horizontalDistance,
                                  double euclideanDistance) {
  const double x =
      targetCoords.x - (targetCoords.x - droneCoords.x) *
                           (horizontalDistance + accelerationPath) /
                           euclideanDistance;

  const double y =
      targetCoords.y - (targetCoords.y - droneCoords.y) *
                           (horizontalDistance + accelerationPath) /
                           euclideanDistance;

  return {x, y};
}

std::optional<Coords> calculateDestinationCoords(const Coords &droneCoords,
                                                 const Coords &targetCoords,
                                                 double horizontalDistance,
                                                 double euclideanDistance) {

  if (euclideanDistance <= 0 || !std::isfinite(euclideanDistance)) {
    return std::nullopt;
  }

  const double ratio =
      (euclideanDistance - horizontalDistance) / euclideanDistance;

  const double destinationX =
      droneCoords.x + (targetCoords.x - droneCoords.x) * ratio;
  const double destinationY =
      droneCoords.y + (targetCoords.y - droneCoords.y) * ratio;

  return Coords{destinationX, destinationY};
}

std::optional<SimulationResult> calculateSimulation(const Coords &droneCoords,
                                                    const Coords &targetCoords,
                                                    double accelerationPath,
                                                    double horizontalDistance,
                                                    double euclideanDistance) {
  auto destinationCoordsResult = calculateDestinationCoords(
      droneCoords, targetCoords, horizontalDistance, euclideanDistance);

  if (!destinationCoordsResult) {
    return std::nullopt;
  }

  const Coords destinationCoords = *destinationCoordsResult;

  if (horizontalDistance + accelerationPath > euclideanDistance) {
    const Coords intermediatePoint =
        calculateIntermediatePoint(droneCoords, targetCoords, accelerationPath,
                                   horizontalDistance, euclideanDistance);

    return SimulationResult{destinationCoords, intermediatePoint};
  }

  return SimulationResult{destinationCoords};
}
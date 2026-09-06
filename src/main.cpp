#include "ammo.hpp"
#include "io.hpp"
#include "simulation.hpp"
#include "types.hpp"
#include <iostream>

int main() {
  try {
    InputData input = readInput("input.txt");

    const Ammo *ammo = findAmmo(input.ammoName);

    if (ammo == nullptr) {
      std::cerr << "Ammo not found\n";
      return 1;
    }

    const MotionParams motionParams = {
        .mass = ammo->mass,
        .drag = ammo->drag,
        .lift = ammo->lift,
        .speed = input.attackSpeed,
        .height = input.z,
    };
    const Coords targetCoords = {.x = input.targetX, .y = input.targetY};
    const Coords droneCoords = {.x = input.x, .y = input.y};

    auto timeResult = getFlightTime(motionParams);

    if (!timeResult) {
      std::cerr << "Invalid flight time\n";
      return 1;
    }

    const double time = *timeResult;
    auto horizontalDistanceResult =
        calculateHorizontalDistance(motionParams, time);

    if (!horizontalDistanceResult) {
      std::cerr << "Invalid horizontal distance\n";
      return 1;
    }

    const double horizontalDistance = *horizontalDistanceResult;
    const double euclideanDistance = measureDistance(droneCoords, targetCoords);
    const double accelerationPath = input.accelerationPath;
    const Coords destinationCoords = calculateDestinationCoords(
        droneCoords, targetCoords, horizontalDistance, euclideanDistance,
        accelerationPath);

    std::cout << destinationCoords.x << ", " << destinationCoords.y
              << std::endl;

    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}

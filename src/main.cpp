
#include "ammo.hpp"
#include "io.hpp"
#include <iostream>

int main() {
  InputData input = readInput("input.txt");

  const Ammo *ammo = findAmmo(input.ammoName);

  if (ammo == nullptr) {
    std::cerr << "Ammo not found\n";
    return 1;
  }

  std::cout << ammo->drag << std::endl;

  return 0;
}

#include "ammo.hpp"
#include <string_view>

const Ammo *findAmmo(const std::string_view &ammoName) {
  for (const Ammo &ammo : ammoList) {
    if (ammo.name == ammoName) {
      return &ammo;
    }
  }

  return nullptr;
}
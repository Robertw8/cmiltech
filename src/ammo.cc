#include "ammo.h"

const Ammo *FindAmmo(std::string_view ammo_name) {
  for (const Ammo &ammo : ammo_list) {
    if (ammo.name == ammo_name) {
      return &ammo;
    }
  }

  return nullptr;
}
#include "ammo.h"
#include <string_view>

const Ammo *FindAmmo(const string_view &ammo_name) {
  for (const Ammo &ammo : ammo_list) {
    if (ammo.name == ammo_name) {
      return &ammo;
    }
  }

  return nullptr;
}
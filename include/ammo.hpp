#pragma once
#include <array>
#include <string_view>

enum class AmmoType { VOG, M67, RKG, GlidingVOG, GlidingRKG };

struct Ammo {
    std::string_view name;
    double mass;
    double drag;
    double lift;
};

inline constexpr std::array<Ammo, 5> ammoList{
    {
        {"VOG-17", 0.35, 0.07, 0.0},
        {"M67", 0.60, 0.10, 0.0},
        {"RKG-3", 1.20, 0.10, 0.0},
        {"GLIDING-VOG", 0.45, 0.10, 1.0},
        {"GLIDING-RKG", 1.40, 0.10, 1.0}
    }
};

const Ammo *findAmmo(const std::string_view &ammoName);

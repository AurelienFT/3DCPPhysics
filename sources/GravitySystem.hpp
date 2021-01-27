#pragma once

#include "Gravity.hpp"
#include "Library.hpp"

class GravitySystem {
public:
    DLLATTRIB explicit GravitySystem() {};
    DLLATTRIB void update(float dt, std::uint64_t);
};
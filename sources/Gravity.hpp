#pragma once

#include "Maths/Math.hpp"

class Gravity final {
public:
    ml::vec3 direction{0.0f, -1.0f, 0.0f};
    bool hasGravity = true; 
};
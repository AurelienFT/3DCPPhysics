#pragma once

#include "../ICollisionShape.hpp"
#include "Maths/Vectors.hpp"

class Ray {
public:
  DLLATTRIB Ray(ml::vec3 position, ml::vec3 direction) : position(position), direction(direction) {}
  DLLATTRIB ~Ray(void) = default;

  DLLATTRIB ml::vec3 GetPosition() const {
    return position;
  }
  DLLATTRIB ml::vec3 GetDirection() const {
    return direction;
  }

protected:
  ml::vec3 position;   // World space position
  ml::vec3 direction;  // Normalised world space direction
};

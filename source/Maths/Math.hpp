#pragma once

#include "Matrix.hpp"
#include "Vectors.hpp"

namespace ml {
  using mat4 = Matrix4<float>;
  using vec3 = Vector3<float>;
  using vec2 = Vector2<float>;

  inline float radians(float f) {
    return f * 0.0174533f;
  }
}

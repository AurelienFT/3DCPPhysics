#pragma once

#include "Maths/Math.hpp"

class Transform final {
public:
  ml::mat4 matrix{
  {
  {1.0f, 0.0f, 0.0f, 0.0f},
  {0.0f, 1.0f, 0.0f, 0.0f},
  {0.0f, 0.0f, 1.0f, 0.0f},
  {0.0f, 0.0f, 0.0f, 1.0f},
  },
  };
};

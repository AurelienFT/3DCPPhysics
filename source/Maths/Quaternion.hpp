#pragma once

#include <cmath>
#include "Matrix.hpp"

class Quaternion {
public:
  float x;
  float y;
  float z;
  float w;

public:
  explicit Quaternion(float x, float y, float z, float w);
  [[nodiscard]] Quaternion operator*(const Quaternion& b) const;
  [[nodiscard]] Quaternion operator+(const Quaternion& b) const;
  [[nodiscard]] Quaternion conjugate() const;
  void                              normalize();
  [[nodiscard]] Matrix<float, 4, 4> toRotationMatrix() const;
  [[nodiscard]] Matrix<float, 3, 3> toMatrix3() const;
  [[nodiscard]] static Quaternion   fromMatrix(Matrix<float, 4, 4> mat);
  [[nodiscard]] static Quaternion   fromMatrix(Matrix<float, 3, 3> mat);
  [[nodiscard]] static Quaternion   nlerp(Quaternion a, Quaternion b, float blend);
  [[nodiscard]] static Quaternion   slerp(Quaternion a, Quaternion b, float blend);
};

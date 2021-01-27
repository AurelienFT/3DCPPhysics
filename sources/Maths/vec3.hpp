#pragma once

#include <cmath>
#include <algorithm>

class vec3 final {
private:
  // bool cmpf(float a, float b, float epsilon = 0.005f) {
  //   return fabs(a - b) < epsilon;
  // }

  static constexpr float epsilon{1.401298e-45f};

public:
  union {
    struct {
      float x, y, z;
    };
    struct {
      float r, g, b;
    };
  };

public:
  [[nodiscard]] inline vec3 operator+(const vec3 &vec) const {
    return {x + vec.x, y + vec.y, z + vec.z};
  }

  [[nodiscard]] inline vec3 operator-(const vec3 &vec) const {
    return {x - vec.x, y - vec.y, z - vec.z};
  }

  [[nodiscard]] inline vec3 operator/(const vec3 &vec) const {
    return {x / vec.x, y / vec.y, z / vec.z};
  }

  [[nodiscard]] inline vec3 operator*(const vec3 &vec) const {
    return {x * vec.x, y * vec.y, z * vec.z};
  }

  [[nodiscard]] inline vec3 operator+(float f) const {
    return {x + f, y + f, z + f};
  }

  [[nodiscard]] inline vec3 operator-(float f) const {
    return {x - f, y - f, z - f};
  }

  [[nodiscard]] inline vec3 operator/(float f) const {
    return {x / f, y / f, z / f};
  }

  [[nodiscard]] inline vec3 operator*(float f) const {
    return {x * f, y * f, z * f};
  }

  [[nodiscard]] inline vec3 &operator+=(const vec3 &vec) {
    x += vec.x;
    y += vec.y;
    z += vec.z;
    return *this;
  }

  [[nodiscard]] inline vec3 &operator-=(const vec3 &vec) {
    x -= vec.x;
    y -= vec.y;
    z -= vec.z;
    return *this;
  }

  [[nodiscard]] inline vec3 &operator/=(const vec3 &vec) {
    x /= vec.x;
    y /= vec.y;
    z /= vec.z;
    return *this;
  }

  [[nodiscard]] inline vec3 &operator*=(const vec3 &vec) {
    x *= vec.x;
    y *= vec.y;
    z *= vec.z;
    return *this;
  }

  [[nodiscard]] inline vec3 &operator+=(float f) {
    x += f;
    y += f;
    z += f;
    return *this;
  }

  [[nodiscard]] inline vec3 &operator-=(float f) {
    x -= f;
    y -= f;
    z -= f;
    return *this;
  }

  [[nodiscard]] inline vec3 &operator/=(float f) {
    x /= f;
    y /= f;
    z /= f;
    return *this;
  }

  [[nodiscard]] inline vec3 &operator*=(float f) {
    x *= f;
    y *= f;
    z *= f;
    return *this;
  }

  [[nodiscard]] inline vec3 &operator=(const vec3 &vec) {
    x = vec.x;
    y = vec.y;
    z = vec.z;
    return *this;
  }

  [[nodiscard]] inline vec3 &operator=(float f) {
    x = f;
    y = f;
    z = f;
    return *this;
  }

  [[nodiscard]] inline bool operator==(const vec3 &vec) const {
    return x == vec.x && y == vec.y && z == vec.z;
  }

  [[nodiscard]] inline bool operator==(float f) const {
    return x == f && y == f && z == f;
  }

  [[nodiscard]] inline float length() const {
    return sqrt((x * x) + (y * y) + (z * z));
  }

  [[nodiscard]] inline float dot(const vec3 &vec) const {
    return x * vec.x + y * vec.y + z * vec.z;
  }

  [[nodiscard]] inline vec3 lerp(const vec3 &vec, float p) const {
    return *this + (vec - *this) * p;
  }

  [[nodiscard]] inline vec3 clamp(const vec3 &min, const vec3 &max) const {
    return {std::clamp<float>(x, min.x, max.x), std::clamp<float>(y, min.y, max.y), std::clamp<float>(z, min.z, max.z)};
  }

  inline void normalize() {
    float l{length()};
    float m{1.0f / (l + epsilon)};
    x *= m;
    y *= m;
    z *= m;
  }

  [[nodiscard]] inline std::size_t hash() const {
    std::size_t seed{0};
    seed ^= std::hash<float>()(x) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= std::hash<float>()(y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= std::hash<float>()(z) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
  }

  [[nodiscard]] inline float getMaxElement() const {
    return std::max(x, std::max(y, z));
  }

  [[nodiscard]] inline vec3 cross(const vec3 &vec) const {
    return {y * vec.z - z * vec.y, z * vec.x - x * vec.z, x * vec.y - y * vec.x};
  }
};

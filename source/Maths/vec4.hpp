#pragma once

#include <cmath>
#include <algorithm>

class vec4 final {
private:
  // bool cmpf(float a, float b, float epsilon = 0.005f) {
  //   return fabs(a - b) < epsilon;
  // }

  static constexpr float epsilon{1.401298e-45f};

public:
  union {
    struct {
      float x, y, z, w;
    };
    struct {
      float r, g, b, a;
    };
  };

public:
  [[nodiscard]] inline vec4 operator+(const vec4 &vec) const {
    return {x + vec.x, y + vec.y, z + vec.z, w + vec.w};
  }

  [[nodiscard]] inline vec4 operator-(const vec4 &vec) const {
    return {x - vec.x, y - vec.y, z - vec.z, w - vec.w};
  }

  [[nodiscard]] inline vec4 operator/(const vec4 &vec) const {
    return {x / vec.x, y / vec.y, z / vec.z, w / vec.w};
  }

  [[nodiscard]] inline vec4 operator*(const vec4 &vec) const {
    return {x * vec.x, y * vec.y, z * vec.z, w * vec.w};
  }

  [[nodiscard]] inline vec4 operator+(float f) const {
    return {x + f, y + f, z + f, w + f};
  }

  [[nodiscard]] inline vec4 operator-(float f) const {
    return {x - f, y - f, z - f, w - f};
  }

  [[nodiscard]] inline vec4 operator/(float f) const {
    return {x / f, y / f, z / f, w / f};
  }

  [[nodiscard]] inline vec4 operator*(float f) const {
    return {x * f, y * f, z * f, w * f};
  }

  [[nodiscard]] inline vec4 &operator+=(const vec4 &vec) {
    x += vec.x;
    y += vec.y;
    z += vec.z;
    w += vec.w;
    return *this;
  }

  [[nodiscard]] inline vec4 &operator-=(const vec4 &vec) {
    x -= vec.x;
    y -= vec.y;
    z -= vec.z;
    w -= vec.w;
    return *this;
  }

  [[nodiscard]] inline vec4 &operator/=(const vec4 &vec) {
    x /= vec.x;
    y /= vec.y;
    z /= vec.z;
    w /= vec.w;
    return *this;
  }

  [[nodiscard]] inline vec4 &operator*=(const vec4 &vec) {
    x *= vec.x;
    y *= vec.y;
    z *= vec.z;
    w *= vec.w;
    return *this;
  }

  [[nodiscard]] inline vec4 &operator+=(float f) {
    x += f;
    y += f;
    z += f;
    w += f;
    return *this;
  }

  [[nodiscard]] inline vec4 &operator-=(float f) {
    x -= f;
    y -= f;
    z -= f;
    w -= f;
    return *this;
  }

  [[nodiscard]] inline vec4 &operator/=(float f) {
    x /= f;
    y /= f;
    z /= f;
    w /= f;
    return *this;
  }

  [[nodiscard]] inline vec4 &operator*=(float f) {
    x *= f;
    y *= f;
    z *= f;
    w *= f;
    return *this;
  }

  [[nodiscard]] inline vec4 &operator=(const vec4 &vec) {
    x = vec.x;
    y = vec.y;
    z = vec.z;
    w = vec.w;
    return *this;
  }

  [[nodiscard]] inline vec4 &operator=(float f) {
    x = f;
    y = f;
    z = f;
    w = f;
    return *this;
  }

  [[nodiscard]] inline bool operator==(const vec4 &vec) const {
    return x == vec.x && y == vec.y && z == vec.z && w == vec.w;
  }

  [[nodiscard]] inline bool operator==(float f) const {
    return x == f && y == f && z == f && w == f;
  }

  [[nodiscard]] inline float length() const {
    return sqrt((x * x) + (y * y) + (z * z) + (w * w));
  }

  [[nodiscard]] inline float dot(const vec4 &vec) const {
    return x * vec.x + y * vec.y + z * vec.z + w * vec.w;
  }

  [[nodiscard]] inline vec4 lerp(const vec4 &vec, float p) const {
    return *this + (vec - *this) * p;
  }

  [[nodiscard]] inline vec4 clamp(const vec4 &min, const vec4 &max) const {
    return {std::clamp<float>(x, min.x, max.x), std::clamp<float>(y, min.y, max.y), std::clamp<float>(z, min.z, max.z), std::clamp<float>(w, min.w, max.w)};
  }

  inline void normalize() {
    float l{length()};
    float m{1.0f / (l + epsilon)};
    x *= m;
    y *= m;
    z *= m;
    w *= m;
  }

  [[nodiscard]] inline std::size_t hash() const {
    std::size_t seed{0};
    seed ^= std::hash<float>()(x) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= std::hash<float>()(y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= std::hash<float>()(z) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= std::hash<float>()(w) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
  }

  [[nodiscard]] inline float getMaxElement() const {
    return std::max(x, std::max(y, std::max(z, w)));
  }
};

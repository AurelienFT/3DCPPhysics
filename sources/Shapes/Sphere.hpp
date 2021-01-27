
#pragma once

#include "Library.hpp"
#include "Maths/Vectors.hpp"
#include "ICollisionShape.hpp"
#include "Transform.hpp"

class Sphere final : public ICollisionShape {
public:
  DLLATTRIB explicit Sphere(ml::vec3 center, float radius) noexcept;
  DLLATTRIB explicit Sphere(const Sphere &second) noexcept;

  DLLATTRIB void                setCenter(const ml::vec3 &center) noexcept;
  [[nodiscard]] DLLATTRIB auto  getCenter() const noexcept -> ml::vec3;
  DLLATTRIB void                setRadius(const float &radius) noexcept;
  [[nodiscard]] DLLATTRIB float getRadius() const noexcept;
  [[nodiscard]] DLLATTRIB auto  getPoints(const ml::mat4 &transform) const noexcept -> ml::vec3;  // Called by collide(...)

  [[nodiscard]] DLLATTRIB bool operator==(const Sphere &second) const noexcept;

  DLLATTRIB ml::vec3 getLocalPosition() const override;

private:
  ml::vec3 m_center{0.0f, 0.0f, 0.0f};
  float    m_radius{0.0f};
  ml::mat4 m_oldTransform{};
};

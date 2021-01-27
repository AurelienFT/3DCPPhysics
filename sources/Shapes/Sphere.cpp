#include "Sphere.hpp"

Sphere::Sphere(ml::vec3 center, float radius) noexcept : ICollisionShape(ShapeType::SPHERE), m_center{center}, m_radius{radius} {}

Sphere::Sphere(const Sphere &second) noexcept : ICollisionShape(ShapeType::SPHERE), m_center{second.m_center}, m_radius{second.m_radius} {}

void Sphere::setCenter(const ml::vec3 &center) noexcept {
  m_center = center;
}
[[nodiscard]] auto Sphere::getCenter() const noexcept -> ml::vec3 {
  return m_center;
}

auto Sphere::getPoints(const ml::mat4 &transform) const noexcept -> ml::vec3 {
  return (transform * m_center);
}

void Sphere::setRadius(const float &radius) noexcept {
  m_radius = radius;
}

[[nodiscard]] float Sphere::getRadius() const noexcept {
  return m_radius;
}

bool Sphere::operator==(const Sphere &second) const noexcept {
  return (m_center == second.m_center && m_radius == second.m_radius);
}

ml::vec3 Sphere::getLocalPosition() const {
  return m_center;
}

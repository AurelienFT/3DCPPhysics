#include "Capsule.hpp"

Capsule::Capsule(const ml::vec3 &start, const ml::vec3 &end, const float &radius) noexcept : ICollisionShape(ShapeType::CAPSULE), m_start{start}, m_end{end}, m_radius{radius} {}

Capsule::Capsule(const Capsule &second) noexcept : ICollisionShape(ShapeType::CAPSULE), m_start{second.m_start}, m_end{second.m_end}, m_radius{second.m_radius} {}

auto Capsule::getPoints(const ml::mat4 &transform, bool forceInvalidate) -> std::vector<ml::vec3> {
  if (!forceInvalidate && transform == m_oldTransform && m_pointsCache.size() > 0)
    return m_pointsCache;

  std::vector<ml::vec3> points{m_start, m_end};

  for (auto &point : points) {
    point = transform * point;
  }

  m_pointsCache  = points;
  m_oldTransform = transform;
  return points;
}

void Capsule::setStart(const ml::vec3 &start) noexcept {
  m_start = start;
}

[[nodiscard]] auto Capsule::getStart() const noexcept -> ml::vec3 {
  return m_start;
}

void Capsule::setEnd(const ml::vec3 &end) noexcept {
  m_end = end;
}

[[nodiscard]] auto Capsule::getEnd() const noexcept -> ml::vec3 {
  return m_end;
}

void Capsule::setRadius(const float &radius) noexcept {
  m_radius = radius;
}

[[nodiscard]] auto Capsule::getRadius() const noexcept -> float {
  return m_radius;
}

[[nodiscard]] bool Capsule::operator==(const Capsule &second) const noexcept {
  return (second.m_start == m_start && second.m_end == m_end && second.m_radius == m_radius
          && second.m_oldTransform == m_oldTransform
          && second.m_pointsCache == second.m_pointsCache);
}

ml::vec3 Capsule::getLocalPosition() const {
  return (m_start + m_end) * 0.5f;
}
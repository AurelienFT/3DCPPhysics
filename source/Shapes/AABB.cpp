#include <algorithm>
#include <iostream>

#include "AABB.hpp"

AABB::AABB(const ml::vec3 &min, const ml::vec3 &max) noexcept : ICollisionShape(ShapeType::AABB), m_min{min}, m_max{max} {
  this->m_shapeType = ShapeType::AABB;
}

AABB::AABB(const AABB &second) noexcept : ICollisionShape(ShapeType::AABB), m_min{second.m_min}, m_max{second.m_max} {}

auto AABB::getPoints(const ml::mat4 &transform, bool forceInvalidate) -> std::vector<ml::vec3> {
  if (!forceInvalidate && transform == m_oldTransform && m_pointsCache.size() > 0) {
    return m_pointsCache;
  }
  std::vector<ml::vec3> points;
  points.push_back(m_min);
  points.emplace_back(m_max.x, m_min.y, m_min.z);
  points.emplace_back(m_max.x, m_max.y, m_min.z);
  points.emplace_back(m_min.x, m_max.y, m_min.z);
  points.emplace_back(m_min.x, m_max.y, m_max.z);
  points.emplace_back(m_min.x, m_min.y, m_max.z);
  points.emplace_back(m_max.x, m_min.y, m_max.z);
  points.push_back(m_max);

  for (auto &point : points) {
    point = transform * point;
  }

  float max_x = (*std::max_element(points.begin(),
                                   points.end(),
                                   [](const ml::vec3 &a, const ml::vec3 &b) {
                                     return a.x < b.x;
                                   }))
                .x;
  float max_y = (*std::max_element(points.begin(),
                                   points.end(),
                                   [](const ml::vec3 &a, const ml::vec3 &b) {
                                     return a.y < b.y;
                                   }))
                .y;
  float max_z = (*std::max_element(points.begin(),
                                   points.end(),
                                   [](const ml::vec3 &a, const ml::vec3 &b) {
                                     return a.z < b.z;
                                   }))
                .z;
  float min_x = (*std::min_element(points.begin(),
                                   points.end(),
                                   [](const ml::vec3 &a, const ml::vec3 &b) {
                                     return a.x < b.x;
                                   }))
                .x;
  float min_y = (*std::min_element(points.begin(),
                                   points.end(),
                                   [](const ml::vec3 &a, const ml::vec3 &b) {
                                     return a.y < b.y;
                                   }))
                .y;
  float min_z = (*std::min_element(points.begin(),
                                   points.end(),
                                   [](const ml::vec3 &a, const ml::vec3 &b) {
                                     return a.z < b.z;
                                   }))
                .z;
  points.clear();
  points.emplace_back(min_x, min_y, min_z);
  points.emplace_back(max_x, min_y, min_z);
  points.emplace_back(max_x, max_y, min_z);
  points.emplace_back(min_x, max_y, min_z);
  points.emplace_back(min_x, max_y, max_z);
  points.emplace_back(min_x, min_y, max_z);
  points.emplace_back(max_x, min_y, max_z);
  points.emplace_back(max_x, max_y, max_z);
  m_pointsCache  = points;
  m_oldTransform = transform;
  return points;
}

void AABB::setMin(const ml::vec3 &min) noexcept {
  m_min         = min;
  m_pointsCache = getPoints(m_oldTransform, true);
}

void AABB::setMax(const ml::vec3 &max) noexcept {
  m_max         = max;
  m_pointsCache = getPoints(m_oldTransform, true);
}

auto AABB::getMin() const noexcept -> ml::vec3 {
  return m_min;
}

auto AABB::getMax() const noexcept -> ml::vec3 {
  return m_max;
}

bool AABB::operator==(const AABB &second) const noexcept {
  return (second.m_min == m_min && second.m_max == m_max && second.m_oldTransform == m_oldTransform && second.m_pointsCache == second.m_pointsCache);
}

ml::vec3 AABB::getLocalPosition() const {
  return (m_max + m_min) * 0.5f;
}

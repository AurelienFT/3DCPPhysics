#include "OBB.hpp"

OBB::OBB(const ml::vec3 &min, const ml::vec3 &max) noexcept : ICollisionShape{ShapeType::OBB}, m_min{min}, m_max{max} {}

OBB::OBB(const OBB &second) noexcept : ICollisionShape{ShapeType::OBB}, m_min{second.m_min}, m_max{second.m_max}, m_oldTransform{second.m_oldTransform}, m_pointsCache{second.m_pointsCache} {}

auto OBB::getPoints(const ml::mat4 &transform, bool forceInvalidate) -> std::vector<ml::vec3> {
  if (!forceInvalidate && transform == m_oldTransform && m_pointsCache.size() > 0)
    return m_pointsCache;

  std::vector<ml::vec3> points{
  m_min,
  ml::vec3{m_max.x, m_min.y, m_min.z},
  ml::vec3{m_max.x, m_max.y, m_min.z},
  ml::vec3{m_min.x, m_max.y, m_min.z},
  ml::vec3{m_min.x, m_max.y, m_max.z},
  ml::vec3{m_min.x, m_min.y, m_max.z},
  ml::vec3{m_max.x, m_min.y, m_max.z},
  m_max,
  };

  for (auto &point : points) {
    point = transform * point;
  }

  m_pointsCache  = points;
  m_oldTransform = transform;
  
  m_edges        = {
  std::tuple<std::array<ml::vec3, 2>, std::array<int, 2>>({m_pointsCache[2], m_pointsCache[3]}, {0, 3}),
  std::tuple<std::array<ml::vec3, 2>, std::array<int, 2>>({m_pointsCache[3], m_pointsCache[4]}, {0, 4}),
  std::tuple<std::array<ml::vec3, 2>, std::array<int, 2>>({m_pointsCache[4], m_pointsCache[7]}, {0, 2}),
  std::tuple<std::array<ml::vec3, 2>, std::array<int, 2>>({m_pointsCache[2], m_pointsCache[7]}, {0, 5}),
  std::tuple<std::array<ml::vec3, 2>, std::array<int, 2>>({m_pointsCache[6], m_pointsCache[5]}, {1, 2}),
  std::tuple<std::array<ml::vec3, 2>, std::array<int, 2>>({m_pointsCache[5], m_pointsCache[0]}, {1, 4}),
  std::tuple<std::array<ml::vec3, 2>, std::array<int, 2>>({m_pointsCache[0], m_pointsCache[1]}, {1, 3}),
  std::tuple<std::array<ml::vec3, 2>, std::array<int, 2>>({m_pointsCache[1], m_pointsCache[6]}, {1, 5}),
  std::tuple<std::array<ml::vec3, 2>, std::array<int, 2>>({m_pointsCache[4], m_pointsCache[5]}, {2, 4}),
  std::tuple<std::array<ml::vec3, 2>, std::array<int, 2>>({m_pointsCache[6], m_pointsCache[7]}, {2, 5}),
  std::tuple<std::array<ml::vec3, 2>, std::array<int, 2>>({m_pointsCache[0], m_pointsCache[3]}, {3, 4}),
  std::tuple<std::array<ml::vec3, 2>, std::array<int, 2>>({m_pointsCache[2], m_pointsCache[1]}, {3, 5}),
  };


  m_faces = {
  std::tuple<std::array<int, 4>, std::array<int, 4>, ml::vec3>({0, 1, 2, 3}, {2, 3, 4, 7}, ml::vec3(0.0f, 0.0f, 0.0f)),
  std::tuple<std::array<int, 4>, std::array<int, 4>, ml::vec3>({4, 5, 6, 7}, {6, 5, 0, 1}, ml::vec3(0.0f, 0.0f, 0.0f)),
  std::tuple<std::array<int, 4>, std::array<int, 4>, ml::vec3>({2, 8, 4, 9}, {7, 4, 5, 6}, ml::vec3(0.0f, 0.0f, 0.0f)),
  std::tuple<std::array<int, 4>, std::array<int, 4>, ml::vec3>({6, 10, 0, 11}, {1, 0, 3, 2}, ml::vec3(0.0f, 0.0f, 0.0f)),
  std::tuple<std::array<int, 4>, std::array<int, 4>, ml::vec3>({1, 10, 5, 8}, {4, 3, 0, 5}, ml::vec3(0.0f, 0.0f, 0.0f)),
  std::tuple<std::array<int, 4>, std::array<int, 4>, ml::vec3>({3, 9, 7, 11}, {2, 7, 6, 1}, ml::vec3(0.0f, 0.0f, 0.0f)),
  };

  for (std::tuple<std::array<int, 4>, std::array<int, 4>, ml::vec3> &tup : m_faces) {
    std::array<int, 4> vertices = std::get<OBB::VERTICES>(tup);
    Vector3<float>     a{points[vertices[1]] - points[vertices[0]]};
    Vector3<float>     normal = a.cross(points[vertices[2]] - points[vertices[0]]);
    normal.normalize();
    std::get<OBB::NORMAL>(tup) = normal;
  }

  return points;
}  // Called by collide(...)

auto OBB::getSupport(const ml::vec3 &axis) const noexcept -> ml::vec3 {
  float    distance = -FLT_MAX;
  Vector3f furthest = ml::vec3(0.0f, 0.0f, 0.0f);
  for (int i = 0; i < m_pointsCache.size(); i++) {
    float projection = m_pointsCache[i].dot(axis);
    if (projection > distance) {
      distance = projection;
      furthest = m_pointsCache[i];
    }
  }
  return furthest;
}

void OBB::setMin(const ml::vec3 &min) noexcept {
  m_min         = min;
  m_pointsCache = getPoints(m_oldTransform, true);
}

auto OBB::getMin() const noexcept -> ml::vec3 {
  return m_min;
}

void OBB::setMax(const ml::vec3 &max) noexcept {
  m_max         = max;
  m_pointsCache = getPoints(m_oldTransform, true);
}

auto OBB::getMax() const noexcept -> ml::vec3 {
  return m_max;
}

bool OBB::operator==(const OBB &second) const noexcept {
  return (second.m_min == m_min && second.m_max == m_max && second.m_oldTransform == m_oldTransform && second.m_pointsCache == second.m_pointsCache);
}

ml::vec3 OBB::getLocalPosition() const {
  return (m_max + m_min) * 0.5f;
}

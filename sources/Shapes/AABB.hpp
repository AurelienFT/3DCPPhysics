#pragma once

#include "Library.hpp"
#include "Maths/Vectors.hpp"
#include "ICollisionShape.hpp"
#include "Transform.hpp"

// Deduire les deux autres points puis la hitbox
class AABB final : public ICollisionShape {
public:
  DLLATTRIB explicit AABB(const ml::vec3 &min, const ml::vec3 &max) noexcept;
  DLLATTRIB explicit AABB(const AABB &second) noexcept;

  [[nodiscard]] DLLATTRIB auto getPoints(const ml::mat4 &transform, bool forceInvalidate = false) -> std::vector<ml::vec3>;  // Called by collide(...)

  DLLATTRIB void               setMin(const ml::vec3 &min) noexcept;
  [[nodiscard]] DLLATTRIB auto getMin() const noexcept -> ml::vec3;
  DLLATTRIB void               setMax(const ml::vec3 &max) noexcept;
  [[nodiscard]] DLLATTRIB auto getMax() const noexcept -> ml::vec3;

  [[nodiscard]] DLLATTRIB bool operator==(const AABB &second) const noexcept;

  DLLATTRIB ml::vec3 getLocalPosition() const override;

private:
  ml::vec3              m_min{0.0f, 0.0f, 0.0f};
  ml::vec3              m_max{0.0f, 0.0f, 0.0f};
  ml::mat4              m_oldTransform{};
  std::vector<ml::vec3> m_pointsCache{};
};

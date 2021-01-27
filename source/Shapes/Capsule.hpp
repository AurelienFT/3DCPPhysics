#pragma once

#include "Library.hpp"
#include "Maths/Vectors.hpp"
#include "ICollisionShape.hpp"
#include "Transform.hpp"

class Capsule final : public ICollisionShape {
public:
  DLLATTRIB explicit Capsule(const ml::vec3 &top, const ml::vec3 &bottom, const float &radius) noexcept;
  DLLATTRIB explicit Capsule(const Capsule &second) noexcept;

  [[nodiscard]] DLLATTRIB auto getPoints(const ml::mat4 &transform, bool forceInvalidate = false) -> std::vector<ml::vec3>;  // Called by collide(...)

  DLLATTRIB void               setStart(const ml::vec3 &start) noexcept;
  [[nodiscard]] DLLATTRIB auto getStart() const noexcept -> ml::vec3;
  DLLATTRIB void               setEnd(const ml::vec3 &end) noexcept;
  [[nodiscard]] DLLATTRIB auto getEnd() const noexcept -> ml::vec3;
  DLLATTRIB void               setRadius(const float &radius) noexcept;
  [[nodiscard]] DLLATTRIB auto getRadius() const noexcept -> float;

  [[nodiscard]] DLLATTRIB bool operator==(const Capsule &second) const noexcept;

  DLLATTRIB ml::vec3 getLocalPosition() const override;

private:
  ml::vec3              m_start{0.0f, 0.0f, 0.0f};
  ml::vec3              m_end{0.0f, 0.0f, 0.0f};
  float                 m_radius{0.0f};
  ml::mat4              m_oldTransform{};
  std::vector<ml::vec3> m_pointsCache{};
};

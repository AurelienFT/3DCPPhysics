#pragma once

#include <cfloat>

#include "Library.hpp"
#include "Maths/Vectors.hpp"
#include "ICollisionShape.hpp"
#include "Transform.hpp"

class OBB final : public ICollisionShape {
public:
  enum TupleFaces {
    EDGESINDEX,
    VERTICES,
    NORMAL
  };

  enum TupleEdges {
    EDGES,
    FACES,
  };

  DLLATTRIB explicit OBB(const ml::vec3 &min, const ml::vec3 &max) noexcept;
  DLLATTRIB explicit OBB(const OBB &second) noexcept;

  [[nodiscard]] DLLATTRIB auto getPoints(const ml::mat4 &transform, bool forceInvalidate = false) -> std::vector<ml::vec3>;  // Called by collide(...)

  DLLATTRIB void               setMin(const ml::vec3 &min) noexcept;
  [[nodiscard]] DLLATTRIB auto getMin() const noexcept -> ml::vec3;
  DLLATTRIB void               setMax(const ml::vec3 &max) noexcept;
  [[nodiscard]] DLLATTRIB auto getMax() const noexcept -> ml::vec3;

  [[nodiscard]] auto getSupport(const ml::vec3 &axis) const noexcept -> ml::vec3;
  [[nodiscard]] bool operator==(const OBB &second) const noexcept;

  std::vector<std::tuple<std::array<ml::vec3, 2>, std::array<int, 2>>> m_edges{};
  std::vector<std::tuple<std::array<int, 4>, std::array<int, 4>, ml::vec3>> m_faces{};
  std::vector<ml::vec3> m_pointsCache{};

  DLLATTRIB ml::vec3 getLocalPosition() const override;


private:
  ml::vec3              m_min{0.0f, 0.0f, 0.0f};
  ml::vec3              m_max{0.0f, 0.0f, 0.0f};

  ml::mat4              m_oldTransform{};
};

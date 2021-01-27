#pragma once

#include <memory>
#include "Maths/Math.hpp"
#include "Maths/Vectors.hpp"
#include "ShapeType.hpp"

#include "Library.hpp"

class ICollisionShape {
public:
  DLLATTRIB explicit ICollisionShape() noexcept : m_shapeType(ShapeType::UNKNOWN) {}
  DLLATTRIB explicit ICollisionShape(ShapeType t) noexcept : m_shapeType(t) {}
  DLLATTRIB virtual ~ICollisionShape() = default;

  DLLATTRIB virtual ml::vec3 getLocalPosition() const = 0;

  ShapeType m_shapeType;
};

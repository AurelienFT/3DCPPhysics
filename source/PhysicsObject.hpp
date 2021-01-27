#pragma once

#include <memory>
#include <array>

#include "ICollisionShape.hpp"
#include "Maths/Math.hpp"
#include "Maths/Quaternion.hpp"
#include "Library.hpp"

class PhysicsObject final {
public:
  std::unique_ptr<ICollisionShape> m_shape{nullptr};

private:
  const float UNIT_MULTIPLIER = 100.0f;
  // const float UNIT_RECIPROCAL = 1.0f / UNIT_MULTIPLIER;

  float m_inverseMass = 1.0f;
  float m_elasticity  = 0.8f;
  float m_friction    = 0.8f;

  bool m_isRigid = false;

  // linear stuff
  ml::vec3 m_linearVelocity{0.0f, 0.0f, 0.0f};
  ml::vec3 m_force{0.0f, 0.0f, 0.0f};

  // angular stuff
  ml::vec3            m_angularVelocity{0.0f, 0.0f, 0.0f};
  ml::vec3            m_torque{0.0f, 0.0f, 0.0f};
  ml::vec3            inverseInertia{0.0f, 0.0f, 0.0f};
  Matrix<float, 3, 3> m_inverseInteriaTensor{std::array<std::array<float, 3>, 3>{
  std::array<float, 3>{1.0f, 0.0f, 0.0f},
  std::array<float, 3>{0.0f, 1.0f, 0.0f},
  std::array<float, 3>{0.0f, 0.0f, 1.0f},
  }};

public:
  DLLATTRIB explicit PhysicsObject(std::unique_ptr<ICollisionShape> shape);

  DLLATTRIB void clearForces() noexcept;

  DLLATTRIB ml::vec3 getLinearVelocity() const;
  DLLATTRIB ml::vec3 getAngularVelocity() const;

  DLLATTRIB ml::vec3 getTorque() const;
  DLLATTRIB ml::vec3 getForce() const;

  DLLATTRIB void  setInverseMass(float invMass);
  DLLATTRIB float getInverseMass() const;

  DLLATTRIB void setIsRigid(bool isRigid);
  DLLATTRIB bool getIsRigid() const;

  DLLATTRIB void applyAngularImpulse(const ml::vec3 &force);
  DLLATTRIB void applyLinearImpulse(const ml::vec3 &force);

  DLLATTRIB void addForce(const ml::vec3 &force);

  DLLATTRIB void addForceAtPosition(const ml::vec3 &force, const ml::vec3 &position);

  DLLATTRIB void addTorque(const ml::vec3 &torque);

  DLLATTRIB void setLinearVelocity(const ml::vec3 &v);
  DLLATTRIB void setAngularVelocity(const ml::vec3 &v);

  DLLATTRIB void initCubeInertia();
  DLLATTRIB void initSphereInertia();

  DLLATTRIB Matrix<float, 3, 3> getInertiaTensor();
};

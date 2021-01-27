#include "PhysicsObject.hpp"

ml::vec3 PhysicsObject::getLinearVelocity() const {
  return m_linearVelocity;
}

ml::vec3 PhysicsObject::getAngularVelocity() const {
  return m_angularVelocity;
}

ml::vec3 PhysicsObject::getTorque() const {
  return m_torque;
}

ml::vec3 PhysicsObject::getForce() const {
  return m_force;
}

void PhysicsObject::setInverseMass(float invMass) {
  m_inverseMass = invMass;
}

float PhysicsObject::getInverseMass() const {
  return m_inverseMass;
}

void PhysicsObject::setIsRigid(bool isRigid) {
  m_isRigid = isRigid;
}

bool PhysicsObject::getIsRigid() const {
  return m_isRigid;
}

void PhysicsObject::setLinearVelocity(const ml::vec3 &v) {
  m_linearVelocity = v;
}

void PhysicsObject::setAngularVelocity(const ml::vec3 &v) {
  m_angularVelocity = v;
}

Matrix<float, 3, 3> PhysicsObject::getInertiaTensor() {
  return m_inverseInteriaTensor;
}

PhysicsObject::PhysicsObject(std::unique_ptr<ICollisionShape> shape) : m_shape{std::move(shape)} {
  m_inverseMass = 1.0f;
  m_elasticity  = 0.8f;
  m_friction    = 0.8f;
  m_isRigid     = false;
}

void PhysicsObject::applyAngularImpulse(const ml::vec3 &force) {
  m_angularVelocity += m_inverseInteriaTensor * force;
}

void PhysicsObject::applyLinearImpulse(const ml::vec3 &force) {
  // m_linearVelocity /= ml::vec3(2.0f, 2.0f, 2.0f);
  m_linearVelocity += force * m_inverseMass;
}

void PhysicsObject::addForce(const ml::vec3 &addedForce) {
  m_force += addedForce;
}

void PhysicsObject::addForceAtPosition(const ml::vec3 &addedForce, const ml::vec3 &position) {
  // need more function in math
  ml::vec3 localPos = position;  // TODO change it if not working

  m_force += addedForce * PhysicsObject::UNIT_MULTIPLIER;
  m_torque += addedForce.cross(localPos);
}

void PhysicsObject::addTorque(const ml::vec3 &addedTorque) {
  m_torque += addedTorque;
}

void PhysicsObject::clearForces() noexcept {
  m_force  = ml::vec3(0.0f, 0.0f, 0.0f);
  m_torque = ml::vec3(0.0f, 0.0f, 0.0f);
}

void PhysicsObject::initCubeInertia() {
  // define get local scale
  ml::vec3 dimensions{1, 1, 1};  //= m_modelMatrix->getLocalScale(); // TODO change here to adjust
  ml::vec3 dimsSqr = dimensions * dimensions;

  inverseInertia.x = (12.0f * m_inverseMass) / (dimsSqr.y + dimsSqr.z);
  inverseInertia.y = (12.0f * m_inverseMass) / (dimsSqr.x + dimsSqr.z);
  inverseInertia.z = (12.0f * m_inverseMass) / (dimsSqr.x + dimsSqr.y);
}

void PhysicsObject::initSphereInertia() {
  // get local scale
  float radius = 1;  //= m_modelMatrix->getLocalScale().GetMaxElement(); // TODO change here to adjust
  float i      = 2.5f * m_inverseMass / (radius * radius);

  inverseInertia = ml::vec3(i, i, i);
}

#pragma once

#include "Transform.hpp"
#include "PhysicsObject.hpp"

#include "Shapes/AABB.hpp"
#include "Shapes/Sphere.hpp"
#include "Shapes/OBB.hpp"
#include "Shapes/Capsule.hpp"
#include "Shapes/Raycasting.hpp"

#include "Maths/Math.hpp"
#include "Log.hpp"

#include "Library.hpp"

namespace Systems {
  class Physics;
}

class ContactPoint final {
public:
  ml::vec3 localA{0.0f, 0.0f, 0.0f};  // where did the collision occur ...
  ml::vec3 localB{0.0f, 0.0f, 0.0f};  // in the frame of each object !
  ml::vec3 normal{0.0f, 0.0f, 0.0f};  // In world space too
  float    penetration{-FLT_MAX};
};

class CollisionInfo final {
public:
  ContactPoint            point{};
  int firstCollider{};
  int secondCollider{};
  int                     framesLeft{2};

public:
  DLLATTRIB void addContactPoint(const ml::vec3 &localA, const ml::vec3 &localB, const ml::vec3 &normal, float p);
};

struct RayCollision {
  int node;                          // Node that was hit
  ml::vec3                collidedAt{0.0f, 0.0f, 0.0f};  // WORLD SPACE pos of the collision !
  float                   rayDistance = 0;
};

// https://research.ncl.ac.uk/game/mastersdegree/gametechnologies/physicstutorials/4collisiondetection/Physics%20-%20Collision%20Detection.pdf
// https://research.ncl.ac.uk/game/mastersdegree/gametechnologies/physicstutorials/5collisionresponse/Physics%20-%20Collision%20Response.pdf
class PhysicsSystem {
private:
  std::vector<CollisionInfo> m_collisions;

  Log m_logger{"PhysicsSystem"};
  std::function<void(int, int)> m_callbackCollision{};
private:
  DLLATTRIB void                      collisionDections();
  DLLATTRIB void                      collisionResolution();
  DLLATTRIB void                      impulseResolveCollision(CollisionInfo &p) const;
  DLLATTRIB void                      integrateVelocity(float dt);
  [[nodiscard]] DLLATTRIB bool        checkCollisionExists(CollisionInfo existedOne, CollisionInfo toCompare);
  [[nodiscard]] DLLATTRIB static auto closestPointOnLineSegment(ml::vec3 A, ml::vec3 B, ml::vec3 Point) -> ml::vec3;
  [[nodiscard]] DLLATTRIB static auto getEntityWorldPositionAABB(const ICollisionShape &shape, const ml::mat4 &matrix) -> ml::vec3;
  [[nodiscard]] DLLATTRIB static auto getEntityWorldPosition(const ICollisionShape &shape, const ml::mat4 &matrix) -> ml::vec3;

  [[nodiscard]] DLLATTRIB static bool collide(AABB &firstCollider, const ml::mat4 &modelMatrixFirstCollider, AABB &secondCollider, const ml::mat4 &modelMatrixSecondCollider, CollisionInfo &collisionInfo) noexcept;
  [[nodiscard]] DLLATTRIB static bool collide(const Sphere &firstCollider, const ml::mat4 &modelMatrixFirstCollider, const Sphere &secondCollider, const ml::mat4 &modelMatrixSecondcollider, CollisionInfo &collisionInfo) noexcept;
  [[nodiscard]] DLLATTRIB static bool collide(AABB &firstCollider, const ml::mat4 &modelMatrixFirstCollider, const Sphere &secondCollider, const ml::mat4 &modelMatrixSecondCollider, CollisionInfo &collisionInfo) noexcept;
  [[nodiscard]] DLLATTRIB static bool collide(OBB &firstCollider, const ml::mat4 &modelMatrixFirstCollider, OBB &secondCollider, const ml::mat4 &modelMatrixSecondCollider, CollisionInfo &collisionInfo) noexcept;          // https://gist.github.com/eliasdaler/502b54fcf1b515bcc50360ce874e81bc
  [[nodiscard]] DLLATTRIB static bool collide(Capsule &firstCollider, const ml::mat4 &modelMatrixFirstCollider, Capsule &secondCollider, const ml::mat4 &modelMatrixSecondCollider, CollisionInfo &collisionInfo) noexcept;  // https://wickedengine.net/2020/04/26/capsule-collision-detection/
  [[nodiscard]] DLLATTRIB static bool collide(Capsule &firstCollider, const ml::mat4 &modelMatrixFirstCollider, const Sphere &secondCollider, const ml::mat4 &modelMatrixSecondCollider, CollisionInfo &collisionInfo) noexcept;
  [[nodiscard]] DLLATTRIB static bool collide(AABB &firstCollider, const ml::mat4 &modelMatrixFirstCollider, Capsule &secondCollider, const ml::mat4 &modelMatrixSecondCollider, CollisionInfo &collisionInfo) noexcept;

  [[nodiscard]] DLLATTRIB bool RaySphereIntersection(const Ray &r, const ml::mat4 &worldTransform, const Sphere &volume, RayCollision &collision);

  [[nodiscard]] DLLATTRIB bool RayBoxIntersection(const Ray &r, const ml::vec3 &boxPos, const ml::vec3 &boxSize, RayCollision &collision);

  [[nodiscard]] DLLATTRIB bool RayAABBIntersection(const Ray &r, const ml::mat4 &worldTransform, AABB &volume, RayCollision &collision);

  [[nodiscard]] DLLATTRIB bool RayOBBIntersection(const Ray &r, const ml::mat4 &transform, OBB &volume, RayCollision &collision);

  [[nodiscard]] DLLATTRIB bool RayCapsuleIntersection(const Ray &r, const ml::mat4 &worldTransform, Capsule &volume, RayCollision &collision);

public:
  DLLATTRIB explicit PhysicsSystem() {};
  [[nodiscard]] DLLATTRIB bool RayIntersection(const Ray &r, RayCollision &collision);
  [[nodiscard]] DLLATTRIB void setCallbackCollision(std::function<void(int, int)> callbackCollision);

  DLLATTRIB void update2(float dt, std::uint64_t);
  DLLATTRIB void update(float dt, std::uint64_t);

};

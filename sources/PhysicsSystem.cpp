#include "PhysicsSystem.hpp"

void CollisionInfo::addContactPoint(const ml::vec3 &localA, const ml::vec3 &localB, const ml::vec3 &normal, float p) {
  point.localA      = localA;
  point.localB      = localB;
  point.normal      = normal;
  point.penetration = p;
}

bool PhysicsSystem::collide(AABB &firstCollider, const ml::mat4 &modelMatrixFirstCollider, AABB &secondCollider, const ml::mat4 &modelMatrixSecondCollider, CollisionInfo &collisionInfo) noexcept {
  Log logger{"PhysicsSystem"};
  // logger.Debug("Check collisions AABB/AABB");
  auto     firstPoints       = firstCollider.getPoints(modelMatrixFirstCollider, true);
  auto     secondPoints      = secondCollider.getPoints(modelMatrixSecondCollider, true);
  ml::vec3 minFirstCollider  = firstPoints.front();
  ml::vec3 maxFirstCollider  = firstPoints.back();
  ml::vec3 minSecondCollider = secondPoints.front();
  ml::vec3 maxSecondCollider = secondPoints.back();

  if (maxFirstCollider.x > minSecondCollider.x && minFirstCollider.x < maxSecondCollider.x && maxFirstCollider.y > minSecondCollider.y && minFirstCollider.y < maxSecondCollider.y && maxFirstCollider.z > minSecondCollider.z && minFirstCollider.z < maxSecondCollider.z) {
    static const ml::vec3 faces[6] = {
    ml::vec3(-1, 0, 0),
    ml::vec3(1, 0, 0),
    ml::vec3(0, -1, 0),
    ml::vec3(0, 1, 0),
    ml::vec3(0, 0, -1),
    ml::vec3(0, 0, 1),
    };

    float distances[6] = {
    (maxSecondCollider.x - minFirstCollider.x),  // distance of box ’b’ to ’left ’ of ’a ’.
    (maxFirstCollider.x - minSecondCollider.x),  // distance of box ’b’ to ’right ’ of ’a ’.
    (maxSecondCollider.y - minFirstCollider.y),  // distance of box ’b’ to ’bottom ’ of ’a ’.
    (maxFirstCollider.y - minSecondCollider.y),  // distance of box ’b’ to ’top ’ of ’a ’.
    (maxSecondCollider.z - minFirstCollider.z),  // distance of box ’b’ to ’far ’ of ’a ’.
    (maxFirstCollider.z - minSecondCollider.z)   // distance of box ’b’ to ’near ’ of ’a ’.
    };
    float    penetration = FLT_MAX;
    ml::vec3 bestAxis(0.0f, 0.0f, 0.0f);

    for (int i = 0; i < 6; i++) {
      if (distances[i] < penetration) {
        penetration = distances[i];
        bestAxis    = faces[i];
      }
    }
    // std::cout << "Collide AABB/AABB with penetration = " << penetration << std::endl;
    collisionInfo.addContactPoint(ml::vec3(0.0f, 0.0f, 0.0f), ml::vec3(0.0f, 0.0f, 0.0f), bestAxis, penetration);
    // logger.Debug("AABB/AABB collided with a normal vector : {{0}, {1}, {2}} and a penetration of {3}", bestAxis.x, bestAxis.y, bestAxis.z, penetration);
    return true;
  }
  // logger.Debug("AABB/AABB didn't collide");
  return false;
}

bool PhysicsSystem::collide(const Sphere &firstCollider, const ml::mat4 &modelMatrixFirstCollider, const Sphere &secondCollider, const ml::mat4 &modelMatrixSecondCollider, CollisionInfo &collisionInfo) noexcept {
  Log logger{"PhysicsSystem"};
  // logger.Debug("Check collisions Sphere/Sphere");
  auto     firstCenter  = firstCollider.getPoints(modelMatrixFirstCollider);
  auto     secondCenter = secondCollider.getPoints(modelMatrixSecondCollider);
  float    radii        = firstCollider.getRadius() + secondCollider.getRadius();
  ml::vec3 delta        = secondCenter - firstCenter;

  float deltaLength = delta.length();
  if (deltaLength < radii) {
    float penetration = (radii - deltaLength);
    delta.normalize();
    ml::vec3 normal = delta;
    ml::vec3 localA = normal * firstCollider.getRadius();
    ml::vec3 localB = (normal * -1) * secondCollider.getRadius();
    collisionInfo.addContactPoint(localA, localB, normal, penetration);
    // logger.Debug("Sphere/Sphere collided with a normal vector : {{0}, {1}, {2}} and a penetration of {3}", normal.x, normal.y, normal.z, penetration);
    return true;
  }
  // logger.Debug("Sphere/Sphere didn't collide");
  return false;
}

bool PhysicsSystem::collide(AABB &firstCollider, const ml::mat4 &modelMatrixFirstCollider, const Sphere &secondCollider, const ml::mat4 &modelMatrixSecondCollider, CollisionInfo &collisionInfo) noexcept {
  Log logger{"PhysicsSystem"};
  // logger.Debug("Check collisions AABB/Sphere");
  auto     firstPoints       = firstCollider.getPoints(modelMatrixFirstCollider);
  auto     secondCenter      = secondCollider.getPoints(modelMatrixSecondCollider);
  ml::vec3 minFirstCollider  = firstPoints.front();
  ml::vec3 maxFirstCollider  = firstPoints.back();
  ml::vec3 boxHalfSize       = (maxFirstCollider - minFirstCollider) * 0.5f;
  ml::vec3 delta             = secondCenter - ((maxFirstCollider + minFirstCollider) * 0.5f);
  ml::vec3 closestPointOnBox = delta.clamp((boxHalfSize * -1), boxHalfSize);
  ml::vec3 localPoint        = delta - closestPointOnBox;
  float    distance          = localPoint.length();
  if (distance < secondCollider.getRadius()) {
    localPoint.normalize();
    ml::vec3 collisionNormal = localPoint;
    float    penetration     = (secondCollider.getRadius() - distance);
    ml::vec3 localA          = ml::vec3(0.0f, 0.0f, 0.0f);
    ml::vec3 localB          = (collisionNormal * -1) * secondCollider.getRadius();
    collisionInfo.addContactPoint(localA, localB, collisionNormal, penetration);
    // logger.Debug("AABB/Sphere collided with a normal vector : {{0}, {1}, {2}} and a penetration of {3}", collisionNormal.x, collisionNormal.y, collisionNormal.z, penetration);
    return true;
  }
  // logger.Debug("AABB/Sphere didn't collide");
  return false;
}

// Returns right hand perpendicular vector
ml::vec3 getNormal(const ml::vec3 &v) {
  return ml::vec3(-v.y, v.x, v.z);
}

bool isMinkowskiFace(ml::vec3 edgeAFaceANormal, ml::vec3 edgeAFaceBNormal, ml::vec3 edgeADirection, ml::vec3 edgeBFaceANormal, ml::vec3 edgeBFaceBNormal, ml::vec3 edgeBDirection) {
  ml::vec3 edgeACross = edgeAFaceBNormal.cross(edgeAFaceANormal);
  ml::vec3 edgeBCross = edgeBFaceBNormal.cross(edgeBFaceANormal);

  float edgeBFaceADirection = edgeBFaceANormal.dot(edgeACross);
  float edgeBFaceBDirection = edgeBFaceBNormal.dot(edgeACross);
  float edgeAFaceADirection = edgeAFaceANormal.dot(edgeBCross);
  float edgeAFaceBDirection = edgeAFaceBNormal.dot(edgeBCross);

  return ((edgeBFaceADirection * edgeBFaceBDirection < 0) && (edgeAFaceADirection * edgeAFaceBDirection < 0) && (edgeBFaceADirection * edgeAFaceBDirection > 0));
}

bool queryEdgeCollisions(OBB &reference, OBB &incident, CollisionInfo &results) {
  auto edgesA = reference.m_edges;
  auto edgesB = reference.m_edges;
  for (int i = 0; i < edgesA.size(); i++) {
    auto     referenceFaceA   = reference.m_faces[std::get<OBB::FACES>(edgesA[i])[0]];
    auto     referenceFaceB   = reference.m_faces[std::get<OBB::FACES>(edgesA[i])[1]];
    ml::vec3 edgeAFaceANormal = std::get<OBB::NORMAL>(referenceFaceA);
    ml::vec3 edgeAFaceBNormal = std::get<OBB::NORMAL>(referenceFaceB);
    for (int j = 0; j < edgesB.size(); j++) {
      auto     incidentFaceA    = incident.m_faces[std::get<OBB::FACES>(edgesB[j])[0]];
      ml::vec3 edgeBFaceANormal = std::get<OBB::NORMAL>(incidentFaceA);
      edgeBFaceANormal *= -1;
      auto     incidentFaceB    = incident.m_faces[std::get<OBB::FACES>(edgesB[j])[1]];
      ml::vec3 edgeBFaceBNormal = std::get<OBB::NORMAL>(incidentFaceB);
      edgeBFaceBNormal *= -1;
      // COULD BE WRONG : (B - A) * transform == (B * transform - A * transform)
      ml::vec3 edgeADirection = std::get<OBB::EDGES>(edgesA[i])[1] - std::get<OBB::EDGES>(edgesA[i])[0];  // edgesA.get(i).getTransformedDirection(reference.getWorldTransform()).normalize();
      edgeADirection.normalize();
      ml::vec3 edgeBDirection = std::get<OBB::EDGES>(edgesA[j])[1] - std::get<OBB::EDGES>(edgesA[j])[0];  // edgesB.get(j).getTransformedDirection(incident.getWorldTransform()).normalize();
      edgeBDirection.normalize();

      if (isMinkowskiFace(edgeAFaceANormal, edgeAFaceBNormal, edgeADirection, edgeBFaceANormal, edgeBFaceBNormal, edgeBDirection)) {
        ml::vec3 axis = edgeADirection.cross(edgeBDirection);
        if (axis.length() == 0) {
          continue;
        }
        axis.normalize();
        ml::vec3 transformedPointA = std::get<OBB::EDGES>(edgesA[i])[0];
        ml::vec3 transformedPointB = std::get<OBB::EDGES>(edgesB[i])[0];

        if (axis.dot(transformedPointA - ((reference.m_pointsCache[reference.m_pointsCache.size() - 1] + reference.m_pointsCache[0]) * 0.5f)) < 0) {
          axis *= -1;
        }

        float distance = axis.dot(transformedPointB - transformedPointA);
        if (distance > 0) {
          return false;
        }
        if (distance > results.point.penetration) {
          results.addContactPoint(ml::vec3(0.0f, 0.0f, 0.0f), ml::vec3(0.0f, 0.0f, 0.0f), axis, distance);
        }
      }
    }
  }
  return true;
}

bool queryFaceCollisions(OBB &reference, OBB &incident, CollisionInfo &results) {
  for (int i = 0; i < reference.m_faces.size(); i++) {
    ml::vec3 axis       = std::get<OBB::NORMAL>(reference.m_faces[i]);
    ml::vec3 planePoint = reference.getSupport(axis);
    float    distance   = axis.dot(planePoint);

    ml::vec3 negatedNormal = axis * -1.0f;
    ml::vec3 support       = incident.getSupport(negatedNormal);

    distance = axis.dot(planePoint) - distance;
    if (distance > 0) {
      return false;
    }
    if (distance > results.point.penetration) {
      results.addContactPoint(ml::vec3(0.0f, 0.0f, 0.0f), ml::vec3(0.0f, 0.0f, 0.0f), axis, distance);
    }
  }
  return true;
}

bool PhysicsSystem::collide(OBB &firstCollider, const ml::mat4 &modelMatrixFirstCollider, OBB &secondCollider, const ml::mat4 &modelMatrixSecondCollider, CollisionInfo &collisionInfo) noexcept {
  auto firstPoints  = firstCollider.getPoints(modelMatrixFirstCollider, true);
  auto secondPoints = secondCollider.getPoints(modelMatrixSecondCollider, true);
  if (!queryFaceCollisions(firstCollider, secondCollider, collisionInfo)) {
    return true;
  }
  if (!queryFaceCollisions(secondCollider, firstCollider, collisionInfo)) {
    return true;
  }
  if (!queryEdgeCollisions(secondCollider, firstCollider, collisionInfo)) {
    return true;
  }
  return false;
}

auto PhysicsSystem::getEntityWorldPositionAABB(const ICollisionShape &shape, const ml::mat4 &matrix) -> ml::vec3 {
  return matrix.getTranslation() * shape.getLocalPosition();
}

auto PhysicsSystem::getEntityWorldPosition(const ICollisionShape &shape, const ml::mat4 &matrix) -> ml::vec3 {
  return matrix * shape.getLocalPosition();
}

bool PhysicsSystem::checkCollisionExists(CollisionInfo existedOne, CollisionInfo toCompare) {
  if ((existedOne.firstCollider == toCompare.firstCollider && existedOne.secondCollider == toCompare.secondCollider) || (existedOne.firstCollider == toCompare.secondCollider && existedOne.secondCollider == toCompare.firstCollider))
    return true;
  return false;
}

auto PhysicsSystem::closestPointOnLineSegment(ml::vec3 A, ml::vec3 B, ml::vec3 Point) -> ml::vec3 {
  ml::vec3 AB = B - A;
  float    t  = (Point - A).dot(AB) / AB.dot(AB);
  // std::cout << "test nan = " << AB.dot(AB) << std::endl;
  // maybe parenthesis mistake
  return A + (AB * std::min(std::max(t, 0.0f), 1.0f));
}

bool PhysicsSystem::collide(Capsule &firstCollider, const ml::mat4 &modelMatrixFirstCollider, Capsule &secondCollider, const ml::mat4 &modelMatrixSecondCollider, CollisionInfo &collisionInfo) noexcept {
  Log logger{"PhysicsSystem"};
  // logger.Debug("Check collisions Capsule/Capsule");
  std::vector<ml::vec3> pointsFirstCollider{firstCollider.getPoints(modelMatrixFirstCollider)};
  std::vector<ml::vec3> pointsSecondCollider{secondCollider.getPoints(modelMatrixSecondCollider)};
  ml::vec3              a_Normal = pointsFirstCollider.front() - pointsFirstCollider.back();
  a_Normal.normalize();
  ml::vec3 a_LineEndOffset = a_Normal * firstCollider.getRadius();
  ml::vec3 a_A             = pointsFirstCollider.back() + a_LineEndOffset;
  ml::vec3 a_B             = pointsFirstCollider.front() - a_LineEndOffset;

  // capsule B:
  ml::vec3 b_Normal = pointsSecondCollider.front() - pointsSecondCollider.back();
  b_Normal.normalize();
  ml::vec3 b_LineEndOffset = b_Normal * secondCollider.getRadius();
  ml::vec3 b_A             = pointsSecondCollider.back() + b_LineEndOffset;
  ml::vec3 b_B             = pointsSecondCollider.front() - b_LineEndOffset;

  // vectors between line endpoints:
  ml::vec3 v0 = b_A - a_A;
  ml::vec3 v1 = b_B - a_A;
  ml::vec3 v2 = b_A - a_B;
  ml::vec3 v3 = b_B - a_B;

  // squared distances:
  float d0 = v0.dot(v0);
  float d1 = v1.dot(v1);
  float d2 = v2.dot(v2);
  float d3 = v3.dot(v3);

  // select best potential endpoint on capsule A:
  ml::vec3 bestA{0.0f, 0.0f, 0.0f};
  if (d2 < d0 || d2 < d1 || d3 < d0 || d3 < d1) {
    bestA = a_B;
  } else {
    bestA = a_A;
  }

  ml::vec3 bestB = PhysicsSystem::closestPointOnLineSegment(b_A, b_B, bestA);

  bestA                       = PhysicsSystem::closestPointOnLineSegment(a_A, a_B, bestB);
  ml::vec3 penetration_normal = bestA - bestB;
  float    len                = penetration_normal.length();
  penetration_normal.normalize();
  float penetration_depth = firstCollider.getRadius() + secondCollider.getRadius() - len;
  if (penetration_depth > 0) {
    ml::vec3 collisionNormal = penetration_normal * -1.0f;
    float    penetration     = penetration_depth;
    ml::vec3 localA          = ml::vec3(0.0f, 0.0f, 0.0f);
    ml::vec3 localB          = ml::vec3(0.0f, 0.0f, 0.0f);
    collisionInfo.addContactPoint(localA, localB, collisionNormal, penetration);
    // logger.Debug("Capsule/Capsule collided with a normal vector : {{0}, {1}, {2}} and a penetration of {3}", collisionNormal.x, collisionNormal.y, collisionNormal.z, penetration);
    return true;
  }
  // logger.Debug("Capsule/Capsule didn't collide");
  return false;
}

bool PhysicsSystem::collide(Capsule &firstCollider, const ml::mat4 &modelMatrixFirstCollider, const Sphere &secondCollider, const ml::mat4 &modelMatrixSecondCollider, CollisionInfo &collisionInfo) noexcept {
  Log logger{"PhysicsSystem"};
  // logger.Debug("Check collisions Capsule/Sphere");
  std::vector<ml::vec3> pointsFirstCollider{firstCollider.getPoints(modelMatrixFirstCollider)};
  auto                  secondCenter{secondCollider.getPoints(modelMatrixSecondCollider)};

  ml::vec3 a_Normal = pointsFirstCollider.front() - pointsFirstCollider.back();
  a_Normal.normalize();
  ml::vec3 a_LineEndOffset = a_Normal * firstCollider.getRadius();
  ml::vec3 a_A             = pointsFirstCollider.back() + a_LineEndOffset;
  ml::vec3 a_B             = pointsFirstCollider.front() - a_LineEndOffset;
  ml::vec3 bestA           = PhysicsSystem::closestPointOnLineSegment(a_A, a_B, secondCenter);

  const ml::mat4 matrix{
  {
  {1.0f, 0.0f, 0.0f, 0.0f},
  {0.0f, 1.0f, 0.0f, 0.0f},
  {0.0f, 0.0f, 1.0f, 0.0f},
  {0.0f, 0.0f, 0.0f, 1.0f},
  },
  };
  // logger.Debug("Send collision to Sphere/Sphere");
  return (collide(Sphere(bestA, firstCollider.getRadius()), matrix, Sphere(secondCenter, secondCollider.getRadius()), matrix, collisionInfo));
}

bool PhysicsSystem::collide(AABB &secondCollider, const ml::mat4 &modelMatrixSecondCollider, Capsule &firstCollider, const ml::mat4 &modelMatrixFirstCollider, CollisionInfo &collisionInfo) noexcept {
  Log logger{"PhysicsSystem"};
  // logger.Debug("Check collisions AABB/Capsule");
  std::vector<ml::vec3> pointsFirstCollider{firstCollider.getPoints(modelMatrixFirstCollider)};
  auto                  secondPoints{secondCollider.getPoints(modelMatrixSecondCollider)};
  auto                  secondCenter = PhysicsSystem::getEntityWorldPosition(secondCollider, modelMatrixSecondCollider);
  ml::vec3              a_Normal     = pointsFirstCollider.front() - pointsFirstCollider.back();
  a_Normal.normalize();
  ml::vec3       a_LineEndOffset = a_Normal * firstCollider.getRadius();
  ml::vec3       a_A             = pointsFirstCollider.back() + a_LineEndOffset;
  ml::vec3       a_B             = pointsFirstCollider.front() - a_LineEndOffset;
  ml::vec3       bestA           = PhysicsSystem::closestPointOnLineSegment(a_A, a_B, secondCenter);
  const ml::mat4 matrix{
  {
  {1.0f, 0.0f, 0.0f, 0.0f},
  {0.0f, 1.0f, 0.0f, 0.0f},
  {0.0f, 0.0f, 1.0f, 0.0f},
  {0.0f, 0.0f, 0.0f, 1.0f},
  },
  };
  AABB aabb{AABB(secondPoints.front(), secondPoints.back())};
  // logger.Debug("Send collision to AABB/Sphere");
  return (collide(aabb, matrix, Sphere(bestA, firstCollider.getRadius()), matrix, collisionInfo));
}

void PhysicsSystem::collisionDections() {
  /* FIX : auto &entities{getItems()};
  for (auto i = entities.begin(); i != entities.end(); i++) {
    for (auto j = i + 1; j != entities.end(); j++) {
      auto &&[entityI, physicsI, transformI]{*i};
      auto &&[entityJ, physicsJ, transformJ]{*j};
      // m_logger.Debug("Testing collision with {0}, and {1}", entityI, entityJ);
      if (entityI == entityJ) {
        // m_logger.Debug("Skip collisions because objects are equals.");
        continue;
      }

      CollisionInfo info{
      .firstCollider  = entityI,
      .secondCollider = entityJ,
      };

      auto it = std::find_if(m_collisions.begin(), m_collisions.end(), [this, info](CollisionInfo toCompare) {
        return checkCollisionExists(info, toCompare);
      });

      if (it != m_collisions.end()) {
        // m_logger.Debug("Skip collisions because a resolution is already active with this two colliders.");
        continue;
      }

      if (physicsI.m_shape->m_shapeType == ShapeType::AABB && physicsJ.m_shape->m_shapeType == ShapeType::AABB) {
        if (collide(reinterpret_cast<AABB &>(*physicsI.m_shape), transformI.matrix, reinterpret_cast<AABB &>(*physicsJ.m_shape), transformJ.matrix, info) == true)
          m_collisions.push_back(info);
      } else if (physicsI.m_shape->m_shapeType == ShapeType::SPHERE && physicsJ.m_shape->m_shapeType == ShapeType::SPHERE) {
        if (collide(reinterpret_cast<Sphere &>(*physicsI.m_shape), transformI.matrix, reinterpret_cast<Sphere &>(*physicsJ.m_shape), transformJ.matrix, info) == true)
          m_collisions.push_back(info);
      } else if (physicsI.m_shape->m_shapeType == ShapeType::AABB && physicsJ.m_shape->m_shapeType == ShapeType::SPHERE) {
        if (collide(reinterpret_cast<AABB &>(*physicsI.m_shape), transformI.matrix, reinterpret_cast<Sphere &>(*physicsJ.m_shape), transformJ.matrix, info) == true)
          m_collisions.push_back(info);
      } else if (physicsI.m_shape->m_shapeType == ShapeType::SPHERE && physicsJ.m_shape->m_shapeType == ShapeType::AABB) {
        if (collide(reinterpret_cast<AABB &>(*physicsJ.m_shape), transformJ.matrix, reinterpret_cast<Sphere &>(*physicsI.m_shape), transformI.matrix, info) == true) {
          // std::cout << "in here" << std::endl;
          info.firstCollider  = entityJ;
          info.secondCollider = entityI;
          m_collisions.push_back(info);
        }
      } else if (physicsI.m_shape->m_shapeType == ShapeType::CAPSULE && physicsJ.m_shape->m_shapeType == ShapeType::CAPSULE) {
        if (collide(reinterpret_cast<Capsule &>(*physicsI.m_shape), transformI.matrix, reinterpret_cast<Capsule &>(*physicsJ.m_shape), transformJ.matrix, info) == true)
          m_collisions.push_back(info);
      } else if (physicsI.m_shape->m_shapeType == ShapeType::CAPSULE && physicsJ.m_shape->m_shapeType == ShapeType::SPHERE) {
        if (collide(reinterpret_cast<Capsule &>(*physicsI.m_shape), transformI.matrix, reinterpret_cast<Sphere &>(*physicsJ.m_shape), transformJ.matrix, info) == true)
          m_collisions.push_back(info);
      } else if (physicsI.m_shape->m_shapeType == ShapeType::SPHERE && physicsJ.m_shape->m_shapeType == ShapeType::CAPSULE) {
        if (collide(reinterpret_cast<Capsule &>(*physicsJ.m_shape), transformJ.matrix, reinterpret_cast<Sphere &>(*physicsI.m_shape), transformI.matrix, info) == true) {
          info.firstCollider  = entityJ;
          info.secondCollider = entityI;
          m_collisions.push_back(info);
        }
      } else if (physicsI.m_shape->m_shapeType == ShapeType::CAPSULE && physicsJ.m_shape->m_shapeType == ShapeType::AABB) {
        if (collide(reinterpret_cast<AABB &>(*physicsJ.m_shape), transformJ.matrix, reinterpret_cast<Capsule &>(*physicsI.m_shape), transformI.matrix, info) == true) {
          info.firstCollider  = entityJ;
          info.secondCollider = entityI;
          m_collisions.push_back(info);
        }
      } else if (physicsI.m_shape->m_shapeType == ShapeType::AABB && physicsJ.m_shape->m_shapeType == ShapeType::CAPSULE) {
        if (collide(reinterpret_cast<AABB &>(*physicsI.m_shape), transformI.matrix, reinterpret_cast<Capsule &>(*physicsJ.m_shape), transformJ.matrix, info) == true)
          m_collisions.push_back(info);
      } else if (physicsI.m_shape->m_shapeType == ShapeType::OBB && physicsJ.m_shape->m_shapeType == ShapeType::OBB) {
        if (collide(reinterpret_cast<OBB &>(*physicsJ.m_shape), transformJ.matrix, reinterpret_cast<OBB &>(*physicsI.m_shape), transformI.matrix, info) == true)
          m_collisions.push_back(info);
      }
    }
  }*/
}

void PhysicsSystem::collisionResolution() {
  for (auto i{m_collisions.begin()}; i != m_collisions.end();) {
    if (i->framesLeft == 2) {
      if (m_callbackCollision) {
        m_callbackCollision(i->firstCollider, i->secondCollider);
      }
      impulseResolveCollision(*i);
    }
    i->framesLeft = i->framesLeft - 1;
    if (i->framesLeft < 0) {
      i = m_collisions.erase(i);
    } else
      ++i;
  }
}

void PhysicsSystem::impulseResolveCollision(CollisionInfo &p) const {
  // m_logger.Debug("Resolve collisions between {0} and {1}", p.firstCollider, p.secondCollider);
  /* FIX : auto &&[physA, transformA]{m_admin.getComponents<Components::Physics, Components::Transform>(p.firstCollider)};
  auto &&[physB, transformB]{m_admin.getComponents<Components::Physics, Components::Transform>(p.secondCollider)};

  float totalMass = physA.getInverseMass() + physB.getInverseMass();

  // Separate them out using projection
  if (!physA.getIsRigid()) {
    transformA.matrix.setTranslation(transformA.matrix.getTranslation() - (p.point.normal * p.point.penetration * (physA.getInverseMass() / totalMass)));
  }
  if (!physB.getIsRigid()) {
    transformB.matrix.setTranslation(transformB.matrix.getTranslation() + (p.point.normal * p.point.penetration * (physB.getInverseMass() / totalMass)));
  }

  ml::vec3 relativeA{p.point.localA - getEntityWorldPosition(*physA.m_shape.get(), transformA.matrix)};
  ml::vec3 relativeB{p.point.localB - getEntityWorldPosition(*physB.m_shape.get(), transformB.matrix)};

  auto shapeA{(*physA.m_shape.get()).m_shapeType};
  auto shapeB{(*physB.m_shape.get()).m_shapeType};
  bool shouldDo{false};
  // AABB
  shouldDo = shouldDo || (shapeA == ShapeType::AABB && shapeB == ShapeType::AABB);

  // Sphere
  shouldDo = shouldDo || (shapeA == ShapeType::SPHERE && shapeB == ShapeType::SPHERE);

  // AABB / Sphere
  shouldDo = shouldDo || (shapeA == ShapeType::AABB && shapeB == ShapeType::SPHERE);
  shouldDo = shouldDo || (shapeA == ShapeType::SPHERE && shapeB == ShapeType::AABB);

  // AABB / Capsule
  shouldDo = shouldDo || (shapeA == ShapeType::AABB && shapeB == ShapeType::CAPSULE);
  shouldDo = shouldDo || (shapeA == ShapeType::CAPSULE && shapeB == ShapeType::AABB);

  if (shouldDo) {
    relativeA = p.point.localA - getEntityWorldPositionAABB(*physA.m_shape.get(), transformA.matrix);
    relativeB = p.point.localB - getEntityWorldPositionAABB(*physB.m_shape.get(), transformB.matrix);
  }

  ml::vec3 angVelocityA{physA.getAngularVelocity().cross(relativeA)};
  ml::vec3 angVelocityB{physB.getAngularVelocity().cross(relativeB)};

  ml::vec3 fullVelocityA{physA.getLinearVelocity() + angVelocityA};
  ml::vec3 fullVelocityB{physB.getLinearVelocity() + angVelocityB};
  ml::vec3 contactVelocity{fullVelocityB - fullVelocityA};

  float impulseForce = contactVelocity.dot(p.point.normal);

  // now to work out the effect of inertia ....
  ml::vec3 inertiaA      = static_cast<ml::vec3>(physA.getInertiaTensor() * relativeA.cross(p.point.normal)).cross(relativeA);
  ml::vec3 inertiaB      = static_cast<ml::vec3>(physB.getInertiaTensor() * relativeA.cross(p.point.normal)).cross(relativeB);
  float    angularEffect = (inertiaA + inertiaB).dot(p.point.normal);

  float cRestitution = 0.66f;  // disperse some kinectic energy

  float j = (-(1.0f + cRestitution) * impulseForce) / (totalMass + angularEffect);

  ml::vec3 fullImpulse = p.point.normal * j;
  if (!physA.getIsRigid()) {
    ml::vec3 reverseImpulse = fullImpulse * -1;
    // m_logger.Debug("Apply linear impulse {{0}, {1}, {2}} to {3}", reverseImpulse.x, reverseImpulse.y, reverseImpulse.z, p.firstCollider);
    physA.applyLinearImpulse(fullImpulse * -1);
    if ((*physA.m_shape.get()).m_shapeType != ShapeType::CAPSULE) {
      // m_logger.Debug("Apply angular impulse {{0}, {1}, {2}} to {3}", reverseImpulse.x, reverseImpulse.y, reverseImpulse.z, p.firstCollider);
      physA.applyAngularImpulse(relativeA.cross(fullImpulse * -1));
    }
  }
  if (!physB.getIsRigid()) {
    // m_logger.Debug("Apply linear impulse {{0}, {1}, {2}} to {3}", fullImpulse.x, fullImpulse.y, fullImpulse.z, p.secondCollider);
    physB.applyLinearImpulse(fullImpulse);
    if ((*physB.m_shape.get()).m_shapeType != ShapeType::CAPSULE) {
      // m_logger.Debug("Apply angular impulse {{0}, {1}, {2}} to {3}", fullImpulse.x, fullImpulse.y, fullImpulse.z, p.secondCollider);
      physB.applyAngularImpulse(relativeB.cross(fullImpulse));
    }
  }
  // m_logger.Debug("Collision between {0} and {1} resolved", p.firstCollider, p.secondCollider);
  */
}

void PhysicsSystem::integrateVelocity(float dt) {
  float dampingFactor = 1.0f - 0.95f;
  float frameDamping  = powf(dampingFactor, dt);

  /* FIX : auto &entities{getItems()};
  for (auto &&[entity, physics, transform] : entities) {
    // m_logger.Debug("Resolve velocity for {0}", entity);
    ml::vec3 position{transform.matrix.getTranslation()};
    ml::vec3 linearVel{physics.getLinearVelocity()};
    position += linearVel * dt;

    transform.matrix.setTranslation(position);
    // Linear Damping
    linearVel = linearVel * frameDamping;
    physics.setLinearVelocity(linearVel);
    // m_logger.Debug("Set linear velocity to {{0}, {1}, {2}}", linearVel.x, linearVel.y, linearVel.z);
    // first implem angular
    Quaternion orientation{Quaternion::fromMatrix(transform.matrix.getRotation())};
    ml::vec3   angVel{physics.getAngularVelocity()};

    ml::vec3 tempVec{angVel * dt * 0.5f};
    orientation = orientation + (Quaternion(tempVec.x, tempVec.y, tempVec.z, 0.0f) * orientation);

    orientation.normalize();
    transform.matrix.setRotation(orientation.toMatrix3());
    // Damp the angular velocity too
    angVel = angVel * frameDamping;
    physics.setAngularVelocity(angVel);
    // m_logger.Debug("Set linear velocity to {{0}, {1}, {2}}", angVel.x, angVel.y, angVel.z);
    // m_logger.Debug("Velocity resolved for {0}", entity);
  } */
}

void PhysicsSystem::update(float dt, std::uint64_t) {
  // collisionDections();
  // collisionResolution();
  // integrateVelocity(dt);
  for (std::size_t i{0}; i < 5; ++i) {
    update2(dt / 5.0f, 0);
  }
}

void PhysicsSystem::update2(float dt, std::uint64_t) {
  collisionDections();
  collisionResolution();
  integrateVelocity(dt);
}

bool PhysicsSystem::RayIntersection(const Ray &r, RayCollision &collision) {
  ml::vec3 position  = r.GetPosition();
  ml::vec3 direction = r.GetDirection();
  // m_logger.Debug("Raycast from {{0}, {1}, {2}} to direction {{3}, {4}, {5}}", position.x, position.y, position.z, direction.x, direction.y, direction.z);
  /* FIX : auto &entities{getItems()};
  for (auto &&[entity, physics, transform] : entities) {

    switch (physics.m_shape->m_shapeType) {
      case ShapeType::AABB:
        if (RayAABBIntersection(r, transform.matrix, reinterpret_cast<AABB &>(*physics.m_shape), collision)) {
          collision.node = entity;
        }
        break;
      case ShapeType::OBB:
        if (RayOBBIntersection(r, transform.matrix, reinterpret_cast<OBB &>(*physics.m_shape), collision)) {
          collision.node = entity;
        }
        break;
      case ShapeType::SPHERE:
        if (RaySphereIntersection(r, transform.matrix, reinterpret_cast<Sphere &>(*physics.m_shape), collision)) {
          collision.node = entity;
        }
        break;
      case ShapeType::CAPSULE:
        if (RayCapsuleIntersection(r, transform.matrix, reinterpret_cast<Capsule &>(*physics.m_shape), collision)) {
          collision.node = entity;
        }
        break;
    }
  }
  if (collision.rayDistance > 0.0f) {
    // m_logger.Debug("Raycast found object {0} at {{1}, {2}, {3}}", collision.node, collision.collidedAt.x, collision.collidedAt.y, collision.collidedAt.z);
    return true;
  }
  // m_logger.Debug("Raycast didn't found anything."); */
  return false;
}

bool PhysicsSystem::RaySphereIntersection(const Ray &r, const ml::mat4 &worldTransform, const Sphere &volume, RayCollision &collision) {
  ml::vec3 spherePos    = PhysicsSystem::getEntityWorldPosition(volume, worldTransform);
  float    sphereRadius = volume.getRadius();
  // Get the direction between the ray origin and the sphere origin
  ml::vec3 dir = (spherePos - r.GetPosition());
  // Then project the sphere ’s origin onto our ray direction vector
  float sphereProj = dir.dot(r.GetDirection());

  if (sphereProj < 0.0f) {
    return false;  // point is behind the ray !
  }
  // Get closest point on ray line to sphere
  ml::vec3 point      = r.GetPosition() + (r.GetDirection() * sphereProj);
  float    sphereDist = (point - spherePos).length();
  if (sphereDist > sphereRadius) {
    return false;
  }
  float offset = sqrt((sphereRadius * sphereRadius) - (sphereDist * sphereDist));

  if (sphereProj - (offset) > collision.rayDistance && collision.rayDistance > 0)
    return false;
  collision.rayDistance = sphereProj - (offset);
  collision.collidedAt  = r.GetPosition() + (r.GetDirection() * collision.rayDistance);
  return true;
}

bool PhysicsSystem::RayBoxIntersection(const Ray &r, const ml::vec3 &boxPos, const ml::vec3 &boxSize, RayCollision &collision) {
  ml::vec3 boxMin = boxPos - boxSize;
  ml::vec3 boxMax = boxPos + boxSize;
  ml::vec3 rayPos = r.GetPosition();
  ml::vec3 rayDir = r.GetDirection();
  ml::vec3 tVals(-1, -1, -1);
  for (int i = 0; i < 3; ++i) {  // get best 3 intersections
    if (rayDir[i] > 0) {
      tVals[i] = (boxMin[i] - rayPos[i]) / rayDir[i];
    } else if (rayDir[i] < 0) {
      tVals[i] = (boxMax[i] - rayPos[i]) / rayDir[i];
    }
  }
  float bestT = tVals.getMaxElement();
  if (bestT < 0.0f) {
    return false;  // no backwards rays !
  }
  ml::vec3    intersection = rayPos + (rayDir * bestT);
  const float epsilon      = 0.0001f;  // an amount of leeway in our calcs
  for (int i = 0; i < 3; ++i) {
    if (intersection[i] + epsilon < boxMin[i] || intersection[i] - epsilon > boxMax[i]) {
      return false;  // best intersection doesn ’t touch the box !
    }
  }
  if (bestT > collision.rayDistance && collision.rayDistance > 0)
    return false;
  collision.collidedAt  = intersection;
  collision.rayDistance = bestT;
  return true;
}

void PhysicsSystem::setCallbackCollision(std::function<void(int, int)> callbackCollision) {
  m_callbackCollision = callbackCollision;
}


bool PhysicsSystem::RayAABBIntersection(const Ray &r, const ml::mat4 &worldTransform, AABB &volume, RayCollision &collision) {
  ml::vec3 boxPos           = PhysicsSystem::getEntityWorldPosition(volume, worldTransform);
  auto     firstPoints      = volume.getPoints(worldTransform);
  auto     minFirstCollider = firstPoints.front();
  auto     maxFirstCollider = firstPoints.back();
  auto     boxSize          = (maxFirstCollider - minFirstCollider) * 0.5f;
  return RayBoxIntersection(r, boxPos, boxSize, collision);
}

bool PhysicsSystem::RayOBBIntersection(const Ray &r, const ml::mat4 &worldTransform, OBB &volume, RayCollision &collision) {
  Quaternion orientation  = Quaternion::fromMatrix(worldTransform.getRotation());
  ml::vec3   position     = PhysicsSystem::getEntityWorldPosition(volume, worldTransform);
  auto       transform    = orientation.toMatrix3();
  auto       invTransform = orientation.conjugate().toMatrix3();
  ml::vec3   localRayPos  = r.GetPosition() - position;
  Ray        tempRay(invTransform * localRayPos, invTransform * r.GetDirection());
  auto       firstPoints      = volume.getPoints(worldTransform);
  auto       minFirstCollider = firstPoints.front();
  auto       maxFirstCollider = firstPoints.back();
  auto       boxSize          = (maxFirstCollider - minFirstCollider) * 0.5f;

  bool collided = RayBoxIntersection(tempRay, ml::vec3(0, 0, 0), boxSize, collision);
  if (collided) {
    collision.collidedAt = transform * collision.collidedAt + position;
  }
  return collided;
}

bool PhysicsSystem::RayCapsuleIntersection(const Ray &r, const ml::mat4 &worldTransform, Capsule &volume, RayCollision &collision) {
  std::vector<ml::vec3> pointsFirstCollider{volume.getPoints(worldTransform)};
  ml::vec3              a_Normal = pointsFirstCollider.front() - pointsFirstCollider.back();
  a_Normal.normalize();
  ml::vec3 a_LineEndOffset = a_Normal * volume.getRadius();
  ml::vec3 a_A             = pointsFirstCollider.back() + a_LineEndOffset;
  ml::vec3 a_B             = pointsFirstCollider.front() - a_LineEndOffset;

  // RAY
  ml::vec3 b_A = r.GetPosition();
  ml::vec3 b_B = r.GetPosition() + (r.GetDirection() * 100000);  // TODO update this to handle infinity

  // vectors between line endpoints:
  ml::vec3 v0 = b_A - a_A;
  ml::vec3 v1 = b_B - a_A;
  ml::vec3 v2 = b_A - a_B;
  ml::vec3 v3 = b_B - a_B;

  // squared distances:
  float d0 = v0.dot(v0);
  float d1 = v1.dot(v1);
  float d2 = v2.dot(v2);
  float d3 = v3.dot(v3);

  // select best potential endpoint on capsule A:
  ml::vec3 bestA{0.0f, 0.0f, 0.0f};
  if (d2 < d0 || d2 < d1 || d3 < d0 || d3 < d1) {
    bestA = a_B;
  } else {
    bestA = a_A;
  }

  ml::vec3 bestB = PhysicsSystem::closestPointOnLineSegment(b_A, b_B, bestA);

  bestA                       = PhysicsSystem::closestPointOnLineSegment(a_A, a_B, bestB);
  ml::vec3 penetration_normal = bestA - bestB;
  float    len                = penetration_normal.length();
  penetration_normal.normalize();
  float penetration_depth = volume.getRadius() - len;
  if ((penetration_depth > 0 && (PhysicsSystem::getEntityWorldPosition(volume, worldTransform) - r.GetPosition()).length() < collision.rayDistance) || collision.rayDistance == 0) {
    collision.collidedAt  = PhysicsSystem::getEntityWorldPosition(volume, worldTransform);
    collision.rayDistance = (PhysicsSystem::getEntityWorldPosition(volume, worldTransform) - r.GetPosition()).length();
    return true;
  }
  return false;
}

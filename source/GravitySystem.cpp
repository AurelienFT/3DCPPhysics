#include "GravitySystem.hpp"

void GravitySystem::update(float dt, std::uint64_t) {
  //FIX
  //for (auto &&[entity, gravity, physics] : entities) {
    //if (!physics.getIsRigid() && gravity.hasGravity)
    //  physics.applyLinearImpulse(ml::vec3{0.0f, -9.81f * 5.0f * dt, 0.0f});

    // if () {
    //   auto     tmp{physics.getLinearVelocity()};
    //   ml::vec3 tmp2{0.0f, 0.0f, 0.0f};

    //   if (tmp.y > 0) {
    //     tmp2.y += -9.81f;
    //     tmp2.y *= .75f;
    //   } else if (tmp.y < 0) {
    //     tmp2.y += -9.81f;
    //     tmp2.y *= 1.5f;
    //   } else
    //     tmp2.y += -9.81f;

    //   physics.applyLinearImpulse(tmp2 * dt);
    // }
  //}
}

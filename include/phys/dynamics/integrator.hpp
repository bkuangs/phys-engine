#pragma once
#include "phys/dynamics/rigid_body.hpp"

namespace phys {

void integrateVelocity(RigidBody& body, const Vec3& gravity, float dt);

}

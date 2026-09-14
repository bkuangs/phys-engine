#include <phys/dynamics/rigid_body.hpp>

namespace phys {

void RigidBody::integratePosition(float dt)
{
    if (isStatic) return;
    position += linVelo * dt;
}

void RigidBody::integrateRotation(float dt)
{
    if (isStatic) return;

    Quaternion spin{angularVelo.x, angularVelo.y, angularVelo.z, 0.0f};
    Quaternion delta = spin * rotation;

    rotation.x += 0.5f * dt * delta.x;
    rotation.y += 0.5f * dt * delta.y;
    rotation.z += 0.5f * dt * delta.z;
    rotation.w += 0.5f * dt * delta.w;

    rotation = rotation.normalized();
}

}

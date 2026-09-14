#include <phys/dynamics/integrator.hpp>

namespace phys {

void integrateVelocity(RigidBody& body, const Vec3& gravity, float dt)
{
	if (body.isStatic) return;

	Vec3 acceleration = gravity + body.getForce() * body.getInverseMass();
	body.setLinearVelocity(body.getLinearVelocity() + acceleration * dt);
}

}

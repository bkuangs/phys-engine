#include <phys/solver/sequential_impulse_solver.hpp>
#include <phys/world/physics_world.hpp>
#include <algorithm>

namespace phys {

namespace {

float dot(const Vec3& left, const Vec3& right)
{
	return left.x * right.x + left.y * right.y + left.z * right.z;
}

}

void SequentialImpulseSolver::solve(std::vector<ContactManifold>& contacts,
	PhysicsWorld& world, float dt)
{
	constexpr float slop = 0.005f;
	constexpr int iterations = 4;

	for (int iteration = 0; iteration < iterations; ++iteration) {
		for (ContactManifold& manifold : contacts) {
			RigidBody* bodyA = world.getBody(manifold.bodyA);
			RigidBody* bodyB = world.getBody(manifold.bodyB);
			if (!bodyA || !bodyB) continue;

			float inverseMassSum = bodyA->getInverseMass() + bodyB->getInverseMass();
			if (inverseMassSum <= 0.0f) continue;

			for (uint32_t index = 0; index < manifold.pointCount; ++index) {
				ContactPoint& point = manifold.points[index];
				float velocityAlongNormal = dot(bodyB->getLinearVelocity()
					- bodyA->getLinearVelocity(), manifold.normal);
				float bias = std::max(point.penetration - slop, 0.0f)
					* (0.2f / std::max(dt, 1e-6f));
				float restitutionVelocity = velocityAlongNormal < -1.0f
					? manifold.restitution * velocityAlongNormal : 0.0f;
				float impulseDelta = -(velocityAlongNormal + restitutionVelocity + bias)
					/ inverseMassSum;
				float previousImpulse = point.normalImpulse;
				point.normalImpulse = std::max(previousImpulse + impulseDelta, 0.0f);
				float appliedImpulse = point.normalImpulse - previousImpulse;
				Vec3 impulse = manifold.normal * appliedImpulse;

				bodyA->applyLinearImpulse(-impulse);
				bodyB->applyLinearImpulse(impulse);
			}
		}
	}
}

}

// TODO: Implement iterative normal and friction impulse solving.

/*
"SEQUENTIAL IMPULSE SOLVER" = iterative physics *constraint* algorithm to resolve collisions and contact 
								between rigid bodies.

There are 5 primary building blocks:
1. The constraint Jacobian: 
	Geometric foundation that tells us how the bodies’ velocities change a particular constraint.
	For a contact, it converts motion into how fast the contact points are approaching or separating.
2. Effective mass: 
	How strongly a particular contact resists having its relative velocity changed by an impulse
		a. Function of inverse mass and world-space inverse inertia tensor
		b. Geometry remains fixed over the velocity solve loop, so this is pre-computed once per step and cached
3. Right-hand bias terms: 
	Constraints rarely aim for pure zero relative velocity, rather Jv + b = 0
		a. This enables "bouncing" when relative velocity exceeds a threshold
		b. Or position correction during collision
4. Warm starting + impulse clamping: 
	Warm starting reuses the previous timestep’s impulses as an initial guess. 
		a. Save the final contact impulse from the previous timestep, then use the saved 
			impulse as an initial guess at the next timestamp.

	Impulse clamping keeps the solver’s impulses within allowed limits.
		a. An ordinary contact can push objects apart, but it cannot pull them together. If
			we define a positive normal impulse as push, then during each solver iteration, we 
			can clamp the accumulated impulse and apply only the difference.
5. Immediate state updates (Gauss-Seidel):
	Sequential Impulses applies the impulse vector immediately to the participating bodies' velocities.
	Because body velocities update in place, when the solver moves to the next constraint connected to 
	Body B, it immediately sees the corrected velocities.

In our implementation, we start with Baumgarte stabilization + Projected Gauss-Seidel (PGS).

┌────────────────────────────────────────────────────────┐
│ 1. PRE-SOLVE PASS                                      │
│    - Compute J, r_A, r_B, and M_eff = 1 / (J M^-1 J^T) │
│    - Compute restitution and Baumgarte bias terms      │
│    - Warm start: apply cached impulses from last frame │
└───────────────────────────┬────────────────────────────┘
                            │
                            ▼
┌────────────────────────────────────────────────────────┐
│ 2. VELOCITY ITERATION LOOP (e.g., 8 to 20 iterations)  │
│    For each constraint (contacts, joints, friction):   │
│      a. Compute current error: -(Jv + b)               │
│      b. Calculate delta impulse: Δλ = M_eff * error    │
│      c. Clamp total accumulated impulse                │
│      d. Apply Δλ to body velocities v and ω            │
└───────────────────────────┬────────────────────────────┘
                            │
                            ▼
┌────────────────────────────────────────────────────────┐
│ 3. INTEGRATE POSITIONS                                 │
│    - x += v * dt                                       │
│    - q += 0.5 * (ω * q) * dt                           │
│    - Cache accumulated impulses for next frame         │
└────────────────────────────────────────────────────────┘

One "sweep" means visiting every constraint once: 100 contacts x 10 sweeps = 1,000 contact updates

*/


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
	constexpr float slop = 0.005f;		// how much overlap we can tolerate before solver triggers
	constexpr int iterations = 8;

	for (int iteration = 0; iteration < iterations; ++iteration) {
		for (ContactManifold& manifold : contacts) {

			// Get per-contact manifold
			RigidBody* bodyA = world.getBody(manifold.bodyA);
			RigidBody* bodyB = world.getBody(manifold.bodyB);
			if (!bodyA || !bodyB) continue;

			// Loop through each contact point
			for (uint32_t index = 0; index < manifold.pointCount; ++index) {

				ContactPoint& point = manifold.points[index];
				Vec3 offsetA = bodyA->getRotation().rotate(point.localAnchorA);
				Vec3 offsetB = bodyB->getRotation().rotate(point.localAnchorB);
				Vec3 velocityA = bodyA->getLinearVelocity()
					+ Math3d::cross(bodyA->getAngularVelocity(), offsetA);
				Vec3 velocityB = bodyB->getLinearVelocity()
					+ Math3d::cross(bodyB->getAngularVelocity(), offsetB);

				float velocityAlongNormal = dot(
					velocityB - velocityA, manifold.normal);

				Vec3 angularJacobianA = Math3d::cross(offsetA, manifold.normal);
				Vec3 angularJacobianB = Math3d::cross(offsetB, manifold.normal);
				float inverseEffectiveMass = bodyA->getInverseMass()
					+ bodyB->getInverseMass()
					+ dot(angularJacobianA,
						bodyA->getInverseInertiaWorld() * angularJacobianA)
					+ dot(angularJacobianB,
						bodyB->getInverseInertiaWorld() * angularJacobianB);
				if (inverseEffectiveMass <= 0.0f) continue;

				// Bias = how aggresive we want the corrective velocity to be
				float bias = std::max(point.penetration - slop, 0.0f)
					* (0.2f / std::max(dt, 1e-6f));


				// ------ Gauss-Seidel: Each constraint immediately updates the bodies ---------
				float restitutionVelocity = velocityAlongNormal < -1.0f
					? manifold.restitution * velocityAlongNormal : 0.0f;
				float impulseDelta = -(velocityAlongNormal + restitutionVelocity - bias)
					/ inverseEffectiveMass;

				// Clamp to feasible range ("projected").
				// TODO: Friction constraints and warm-start application between sim steps
				float previousImpulse = point.normalImpulse;
				point.normalImpulse = std::max(previousImpulse + impulseDelta, 0.0f);
				float appliedImpulse = point.normalImpulse - previousImpulse;
				Vec3 impulse = manifold.normal * appliedImpulse;

				bodyA->applyImpulse(-impulse, offsetA);
				bodyB->applyImpulse(impulse, offsetB);
			}
		}
	}
}

}

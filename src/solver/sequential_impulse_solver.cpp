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
#include <array>

namespace phys
{

	namespace
	{

		float dot(const Vec3 &left, const Vec3 &right)
		{
			return left.x * right.x + left.y * right.y + left.z * right.z;
		}

		struct PreparedContactPoint
		{
			Vec3 offsetA{};
			Vec3 offsetB{};
			float inverseEffectiveMass = 0.0f;
			float bias = 0.0f;
			float restitutionVelocity = 0.0f;
		};

		struct PreparedManifold
		{
			ContactManifold *manifold = nullptr;
			RigidBody *bodyA = nullptr;
			RigidBody *bodyB = nullptr;
			std::array<PreparedContactPoint, 4> points{};
		};

	}

	void SequentialImpulseSolver::solve(std::vector<ContactManifold> &contacts,
										PhysicsWorld &world, float dt)
	{
		constexpr float slop = 0.005f; // how much overlap we can tolerate before solver triggers
		constexpr int iterations = 8;
		const float inverseDt = 1.0f / std::max(dt, 1e-6f);

		// Fixed terms while the Gauss-Seidel iterations update velocities.
		// Incoming velocity is a "snapshot" of the restitution velocity we will need; this doesn't 
		// change until the solver is done.

		// What DOES change is the current velocity computed each iteration; eg. how much impulse is 
		// still needed before we reach our fixed target

		// For example:
			// initial velocity:     -10
			// after iteration 1:     -4
			// after iteration 2:      1
			// after iteration 3:      5

		// If we recomputed restitution every time from current velo, the target itself would be changing
		// as we are actively solving it

		// The split is:
		// prepare:
		//     capture incoming velocity
		//     calculate restitution target

		// each iteration:
		//     read current velocity
		//     calculate error relative to fixed target
		//     apply impulse immediately
		std::vector<PreparedManifold> preparedContacts;
		preparedContacts.reserve(contacts.size());
		for (ContactManifold &manifold : contacts)
		{
			RigidBody *bodyA = world.getBody(manifold.bodyA);
			RigidBody *bodyB = world.getBody(manifold.bodyB);
			if (!bodyA || !bodyB)
				continue;

			PreparedManifold prepared;
			prepared.manifold = &manifold;
			prepared.bodyA = bodyA;
			prepared.bodyB = bodyB;

			for (uint32_t index = 0; index < manifold.pointCount; ++index)
			{
				ContactPoint &point = manifold.points[index];
				PreparedContactPoint &preparedPoint = prepared.points[index];
				preparedPoint.offsetA = bodyA->getRotation().rotate(point.localAnchorA);
				preparedPoint.offsetB = bodyB->getRotation().rotate(point.localAnchorB);

				Vec3 velocityA = bodyA->getLinearVelocity() + Math3d::cross(bodyA->getAngularVelocity(), preparedPoint.offsetA);
				Vec3 velocityB = bodyB->getLinearVelocity() + Math3d::cross(bodyB->getAngularVelocity(), preparedPoint.offsetB);
				float incomingVelocityAlongNormal = dot(
					velocityB - velocityA, manifold.normal);

				Vec3 angularJacobianA = Math3d::cross(
					preparedPoint.offsetA, manifold.normal);
				Vec3 angularJacobianB = Math3d::cross(
					preparedPoint.offsetB, manifold.normal);
				preparedPoint.inverseEffectiveMass = bodyA->getInverseMass() + bodyB->getInverseMass() + dot(angularJacobianA, bodyA->getInverseInertiaWorld() * angularJacobianA) + dot(angularJacobianB, bodyB->getInverseInertiaWorld() * angularJacobianB);
				if (preparedPoint.inverseEffectiveMass <= 0.0f)
					continue;

				preparedPoint.bias = std::max(point.penetration - slop, 0.0f) * (0.2f * inverseDt);
				preparedPoint.restitutionVelocity =
					incomingVelocityAlongNormal < -1.0f
						? manifold.restitution * incomingVelocityAlongNormal
						: 0.0f;
			}

			preparedContacts.push_back(prepared);
		}

		for (int iteration = 0; iteration < iterations; ++iteration)
		{
			for (PreparedManifold &prepared : preparedContacts)
			{
				ContactManifold &manifold = *prepared.manifold;
				RigidBody *bodyA = prepared.bodyA;
				RigidBody *bodyB = prepared.bodyB;

				// Loop through each contact point
				for (uint32_t index = 0; index < manifold.pointCount; ++index)
				{

					ContactPoint &point = manifold.points[index];
					PreparedContactPoint &preparedPoint = prepared.points[index];
					if (preparedPoint.inverseEffectiveMass <= 0.0f)
						continue;
					Vec3 offsetA = preparedPoint.offsetA;
					Vec3 offsetB = preparedPoint.offsetB;
					Vec3 velocityA = bodyA->getLinearVelocity() + Math3d::cross(bodyA->getAngularVelocity(), offsetA);
					Vec3 velocityB = bodyB->getLinearVelocity() + Math3d::cross(bodyB->getAngularVelocity(), offsetB);

					// Given that these objects are in contact, how fast are their already-known
					// contact points moving relative to each other along the contact normal?
					float velocityAlongNormal = dot(
						velocityB - velocityA, manifold.normal);

					// ------ Gauss-Seidel: Each constraint immediately updates the bodies ---------
					float impulseDelta = -(velocityAlongNormal + preparedPoint.restitutionVelocity - preparedPoint.bias) / preparedPoint.inverseEffectiveMass;

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

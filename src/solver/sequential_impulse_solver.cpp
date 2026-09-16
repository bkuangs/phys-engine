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
#include <cmath>
#include <utility>

namespace phys
{

	namespace
	{

		float dot(const Vec3 &left, const Vec3 &right)
		{
			return left.x * right.x + left.y * right.y + left.z * right.z;
		}

		float distanceSquared(const Vec3 &left, const Vec3 &right)
		{
			Vec3 delta = left - right;
			return dot(delta, delta);
		}

		struct PreparedContactPoint
		{
			Vec3 offsetA{};
			Vec3 offsetB{};
			Vec3 tangent1{};
			Vec3 tangent2{};
			float inverseEffectiveMass = 0.0f;
			float inverseTangentMass1 = 0.0f;
			float inverseTangentMass2 = 0.0f;
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

				// Prepare tangents for friction as well
				Vec3 tangentReference = std::abs(manifold.normal.x) < 0.9f
											? Vec3{1.0f, 0.0f, 0.0f}
											: Vec3{0.0f, 1.0f, 0.0f};
				// We need two tangent directions relative to the surface
				preparedPoint.tangent1 = Math3d::cross(
					manifold.normal, tangentReference);
				float tangentLength = std::sqrt(dot(
					preparedPoint.tangent1, preparedPoint.tangent1));
				if (tangentLength <= 1e-6f)
					continue;
				preparedPoint.tangent1 = preparedPoint.tangent1 / tangentLength;
				preparedPoint.tangent2 = Math3d::cross(
					manifold.normal, preparedPoint.tangent1);

				// Initial incoming velocity
				Vec3 velocityA = bodyA->getLinearVelocity() + Math3d::cross(bodyA->getAngularVelocity(), preparedPoint.offsetA);
				Vec3 velocityB = bodyB->getLinearVelocity() + Math3d::cross(bodyB->getAngularVelocity(), preparedPoint.offsetB);
				float incomingVelocityAlongNormal = dot(
					velocityB - velocityA, manifold.normal);

				Vec3 angularJacobianA = Math3d::cross(
					preparedPoint.offsetA, manifold.normal);
				Vec3 angularJacobianB = Math3d::cross(
					preparedPoint.offsetB, manifold.normal);
				preparedPoint.inverseEffectiveMass = bodyA->getInverseMass() + bodyB->getInverseMass() + dot(angularJacobianA, bodyA->getInverseInertiaWorld() * angularJacobianA) + dot(angularJacobianB, bodyB->getInverseInertiaWorld() * angularJacobianB);

				// Tangent angular Jacobians: Rotation can make the contact point slide sideways even if the center of mass has zero sideways velocity.
				Vec3 tangentAngularJacobianA = Math3d::cross(
					preparedPoint.offsetA, preparedPoint.tangent1);
				Vec3 tangentAngularJacobianB = Math3d::cross(
					preparedPoint.offsetB, preparedPoint.tangent1);
				preparedPoint.inverseTangentMass1 = bodyA->getInverseMass() + bodyB->getInverseMass() + dot(tangentAngularJacobianA, bodyA->getInverseInertiaWorld() * tangentAngularJacobianA) + dot(tangentAngularJacobianB, bodyB->getInverseInertiaWorld() * tangentAngularJacobianB);

				tangentAngularJacobianA = Math3d::cross(
					preparedPoint.offsetA, preparedPoint.tangent2);
				tangentAngularJacobianB = Math3d::cross(
					preparedPoint.offsetB, preparedPoint.tangent2);
				preparedPoint.inverseTangentMass2 = bodyA->getInverseMass() + bodyB->getInverseMass() + dot(tangentAngularJacobianA, bodyA->getInverseInertiaWorld() * tangentAngularJacobianA) + dot(tangentAngularJacobianB, bodyB->getInverseInertiaWorld() * tangentAngularJacobianB);
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

		// Restore impulses from the previous step before the iterative solve.
		// Local anchors provide a stable contact identity while body handles
		// distinguish contacts belonging to different body pairs.
		constexpr float cacheMatchDistanceSquared = 0.05f * 0.05f;
		for (PreparedManifold &prepared : preparedContacts)
		{
			ContactManifold &manifold = *prepared.manifold;
			RigidBody *bodyA = prepared.bodyA;
			RigidBody *bodyB = prepared.bodyB;
			for (uint32_t index = 0; index < manifold.pointCount; ++index)
			{
				ContactPoint &point = manifold.points[index];
				PreparedContactPoint &preparedPoint = prepared.points[index];
				if (preparedPoint.inverseEffectiveMass <= 0.0f)
					continue;

				for (const PhysicsWorld::CachedContact &cached : world.cachedContacts)
				{
					if (!(cached.bodyA == manifold.bodyA && cached.bodyB == manifold.bodyB) || distanceSquared(cached.localAnchorA, point.localAnchorA) > cacheMatchDistanceSquared || distanceSquared(cached.localAnchorB, point.localAnchorB) > cacheMatchDistanceSquared)
						continue;

					point.normalImpulse = std::max(cached.normalImpulse, 0.0f);
					float tangentLimit = std::max(manifold.friction, 0.0f) * point.normalImpulse;
					point.tangentImpulse1 = Math3d::clamp(
						cached.tangentImpulse1, -tangentLimit, tangentLimit);
					point.tangentImpulse2 = Math3d::clamp(
						cached.tangentImpulse2, -tangentLimit, tangentLimit);
					Vec3 warmImpulse = manifold.normal * point.normalImpulse + preparedPoint.tangent1 * point.tangentImpulse1 + preparedPoint.tangent2 * point.tangentImpulse2;
					bodyA->applyImpulse(-warmImpulse, preparedPoint.offsetA);
					bodyB->applyImpulse(warmImpulse, preparedPoint.offsetB);
					break;
				}
			}
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
					// TODO: Friction warm-start application between sim steps
					float previousImpulse = point.normalImpulse;
					point.normalImpulse = std::max(previousImpulse + impulseDelta, 0.0f);
					float appliedImpulse = point.normalImpulse - previousImpulse;
					Vec3 impulse = manifold.normal * appliedImpulse;

					bodyA->applyImpulse(-impulse, offsetA);
					bodyB->applyImpulse(impulse, offsetB);

					if (preparedPoint.inverseTangentMass1 <= 0.0f || preparedPoint.inverseTangentMass2 <= 0.0f)
						continue;

					// Friction uses the current contact normal impulse as its limit.
					// Each tangent is solved independently against the Coulomb box.
					float tangentLimit = std::max(manifold.friction, 0.0f) * point.normalImpulse;
					velocityA = bodyA->getLinearVelocity() + Math3d::cross(bodyA->getAngularVelocity(), offsetA);
					velocityB = bodyB->getLinearVelocity() + Math3d::cross(bodyB->getAngularVelocity(), offsetB);
					Vec3 relativeVelocity = velocityB - velocityA;

					float tangentVelocity1 = dot(
						relativeVelocity, preparedPoint.tangent1);
					float tangentImpulseDelta1 = -tangentVelocity1 / preparedPoint.inverseTangentMass1;
					float previousTangentImpulse1 = point.tangentImpulse1;
					point.tangentImpulse1 = Math3d::clamp(
						previousTangentImpulse1 + tangentImpulseDelta1,
						-tangentLimit, tangentLimit);
					float appliedTangentImpulse1 = point.tangentImpulse1 - previousTangentImpulse1;
					Vec3 tangentImpulse = preparedPoint.tangent1 * appliedTangentImpulse1;
					bodyA->applyImpulse(-tangentImpulse, offsetA);
					bodyB->applyImpulse(tangentImpulse, offsetB);

					velocityA = bodyA->getLinearVelocity() + Math3d::cross(bodyA->getAngularVelocity(), offsetA);
					velocityB = bodyB->getLinearVelocity() + Math3d::cross(bodyB->getAngularVelocity(), offsetB);
					relativeVelocity = velocityB - velocityA;
					float tangentVelocity2 = dot(
						relativeVelocity, preparedPoint.tangent2);
					float tangentImpulseDelta2 = -tangentVelocity2 / preparedPoint.inverseTangentMass2;
					float previousTangentImpulse2 = point.tangentImpulse2;
					point.tangentImpulse2 = Math3d::clamp(
						previousTangentImpulse2 + tangentImpulseDelta2,
						-tangentLimit, tangentLimit);
					float appliedTangentImpulse2 = point.tangentImpulse2 - previousTangentImpulse2;
					tangentImpulse = preparedPoint.tangent2 * appliedTangentImpulse2;
					bodyA->applyImpulse(-tangentImpulse, offsetA);
					bodyB->applyImpulse(tangentImpulse, offsetB);
				}
			}
		}

		std::vector<PhysicsWorld::CachedContact> nextCache;
		nextCache.reserve(contacts.size() * 2);
		for (const PreparedManifold &prepared : preparedContacts)
		{
			const ContactManifold &manifold = *prepared.manifold;
			for (uint32_t index = 0; index < manifold.pointCount; ++index)
			{
				const ContactPoint &point = manifold.points[index];
				nextCache.push_back({manifold.bodyA,
									 manifold.bodyB,
									 point.localAnchorA,
									 point.localAnchorB,
									 point.normalImpulse,
									 point.tangentImpulse1,
									 point.tangentImpulse2});
			}
		}
		world.cachedContacts = std::move(nextCache);
	}

}

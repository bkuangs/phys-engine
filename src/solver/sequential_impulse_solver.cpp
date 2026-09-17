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
#include "../world/step_workspace.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <numeric>
#include <tuple>
#include <utility>

namespace phys
{

	namespace
	{

		using Clock = std::chrono::steady_clock;

		double elapsedMs(Clock::time_point start)
		{
			return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
		}

		float dot(const Vec3 &left, const Vec3 &right)
		{
			return left.x * right.x + left.y * right.y + left.z * right.z;
		}

		float distanceSquared(const Vec3 &left, const Vec3 &right)
		{
			Vec3 delta = left - right;
			return dot(delta, delta);
		}

		using detail::PreparedBody;
		using detail::PreparedContactPoint;
		using detail::PreparedManifold;

	}

	void SequentialImpulseSolver::solve(std::vector<ContactManifold> &contacts,
										PhysicsWorld &world, float dt)
	{
		detail::SolverWorkspace workspace;
		solve(contacts, world, dt, workspace);
	}

	void SequentialImpulseSolver::solve(std::vector<ContactManifold> &contacts,
										PhysicsWorld &world, float dt,
										detail::SolverWorkspace &workspace)
	{
		SolverStats &stats = world.stats.solverDetails;
		stats = {};
		auto prepareStart = Clock::now();
		constexpr float slop = 0.005f; // how much overlap we can tolerate before solver triggers
		constexpr int iterations = 8;
		const float inverseDt = 1.0f / std::max(dt, 1e-6f);
		std::size_t preparedPointCount = 0;
		std::size_t warmStartComparisons = 0;
		std::size_t warmStartMatches = 0;

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
		auto &preparedBodies = workspace.preparedBodies;
		auto &preparedContacts = workspace.preparedContacts;
		preparedContacts.clear();
		if (++workspace.generation == 0)
		{
			for (PreparedBody &prepared : preparedBodies)
				prepared.generation = 0;
			workspace.generation = 1;
		}
		auto prepareBody = [&](RigidBody *body, uint32_t index) -> PreparedBody * {
			if (preparedBodies.size() < world.slots.size())
				preparedBodies.resize(world.slots.size());
			PreparedBody &prepared = preparedBodies[index];
			if (prepared.generation != workspace.generation)
			{
				prepared.body = body;
				prepared.inverseMass = body->getInverseMass();
				prepared.inverseInertiaWorld = body->getInverseInertiaWorld();
				prepared.generation = workspace.generation;
			}
			return &prepared;
		};
		auto applyCachedImpulse = [](const PreparedBody &prepared, const Vec3 &impulse, const Vec3 &offset) {
			RigidBody *body = prepared.body;
			if (body->isStatic)
				return;
			body->linearVelocity += impulse * prepared.inverseMass;
			body->angularVelocity += prepared.inverseInertiaWorld * Math3d::cross(offset, impulse);
		};
		auto applyAxisImpulse = [](const PreparedBody &prepared, const Vec3 &impulse,
								   const Vec3 &angularResponse, float magnitude) {
			RigidBody *body = prepared.body;
			if (body->isStatic)
				return;
			body->linearVelocity += impulse * prepared.inverseMass;
			body->angularVelocity += angularResponse * magnitude;
		};

		for (ContactManifold &manifold : contacts)
		{
			RigidBody *bodyA = world.getBody(manifold.bodyA);
			RigidBody *bodyB = world.getBody(manifold.bodyB);
			if (!bodyA || !bodyB)
				continue;
			if (world.sleepingEnabled
				&& (bodyA->isStatic || bodyA->isSleeping())
				&& (bodyB->isStatic || bodyB->isSleeping()))
				continue;

			if (preparedContacts.empty() && preparedContacts.capacity() < contacts.size())
				preparedContacts.reserve(contacts.size());
			PreparedManifold prepared;
			prepared.manifold = &manifold;
			prepared.bodyA = prepareBody(bodyA, manifold.bodyA.index);
			prepared.bodyB = prepareBody(bodyB, manifold.bodyB.index);

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
				preparedPoint.normalResponseA = prepared.bodyA->inverseInertiaWorld * angularJacobianA;
				preparedPoint.normalResponseB = prepared.bodyB->inverseInertiaWorld * angularJacobianB;
				preparedPoint.inverseEffectiveMass = prepared.bodyA->inverseMass + prepared.bodyB->inverseMass + dot(angularJacobianA, preparedPoint.normalResponseA) + dot(angularJacobianB, preparedPoint.normalResponseB);

				// Tangent angular Jacobians: Rotation can make the contact point slide sideways even if the center of mass has zero sideways velocity.
				Vec3 tangentAngularJacobianA = Math3d::cross(
					preparedPoint.offsetA, preparedPoint.tangent1);
				Vec3 tangentAngularJacobianB = Math3d::cross(
					preparedPoint.offsetB, preparedPoint.tangent1);
				preparedPoint.tangentResponseA1 = prepared.bodyA->inverseInertiaWorld * tangentAngularJacobianA;
				preparedPoint.tangentResponseB1 = prepared.bodyB->inverseInertiaWorld * tangentAngularJacobianB;
				preparedPoint.inverseTangentMass1 = prepared.bodyA->inverseMass + prepared.bodyB->inverseMass + dot(tangentAngularJacobianA, preparedPoint.tangentResponseA1) + dot(tangentAngularJacobianB, preparedPoint.tangentResponseB1);

				tangentAngularJacobianA = Math3d::cross(
					preparedPoint.offsetA, preparedPoint.tangent2);
				tangentAngularJacobianB = Math3d::cross(
					preparedPoint.offsetB, preparedPoint.tangent2);
				preparedPoint.tangentResponseA2 = prepared.bodyA->inverseInertiaWorld * tangentAngularJacobianA;
				preparedPoint.tangentResponseB2 = prepared.bodyB->inverseInertiaWorld * tangentAngularJacobianB;
				preparedPoint.inverseTangentMass2 = prepared.bodyA->inverseMass + prepared.bodyB->inverseMass + dot(tangentAngularJacobianA, preparedPoint.tangentResponseA2) + dot(tangentAngularJacobianB, preparedPoint.tangentResponseB2);
				if (preparedPoint.inverseEffectiveMass <= 0.0f)
					continue;
				++preparedPointCount;

				preparedPoint.bias = std::max(point.penetration - slop, 0.0f) * (0.2f * inverseDt);
				preparedPoint.restitutionVelocity =
					incomingVelocityAlongNormal < -1.0f
						? manifold.restitution * incomingVelocityAlongNormal
						: 0.0f;
			}

			preparedContacts.push_back(prepared);
		}
		world.stats.solvedContactCount = preparedContacts.size();
		stats.preparedPoints = preparedPointCount;
		stats.prepareMs = elapsedMs(prepareStart);

		// Restore impulses from the previous step before the iterative solve.
		// Local anchors provide a stable contact identity while body handles
		// distinguish contacts belonging to different body pairs.
		auto cachePairLess = [](const auto &left, const auto &right) {
			return std::tie(left.bodyA.index, left.bodyA.generation, left.bodyB.index, left.bodyB.generation)
				< std::tie(right.bodyA.index, right.bodyA.generation, right.bodyB.index, right.bodyB.generation);
		};
		constexpr float cacheMatchDistanceSquared = 0.05f * 0.05f;
		auto warmStart = Clock::now();
		for (PreparedManifold &prepared : preparedContacts)
		{
			ContactManifold &manifold = *prepared.manifold;
			const auto cachedRange = std::equal_range(
				world.cachedContacts.begin(), world.cachedContacts.end(), manifold, cachePairLess);
			for (uint32_t index = 0; index < manifold.pointCount; ++index)
			{
				ContactPoint &point = manifold.points[index];
				PreparedContactPoint &preparedPoint = prepared.points[index];
				if (preparedPoint.inverseEffectiveMass <= 0.0f)
					continue;

				for (auto cachedPoint = cachedRange.first; cachedPoint != cachedRange.second; ++cachedPoint)
				{
					++warmStartComparisons;
					const PhysicsWorld::CachedContact &cached = *cachedPoint;
					if (distanceSquared(cached.localAnchorA, point.localAnchorA) > cacheMatchDistanceSquared || distanceSquared(cached.localAnchorB, point.localAnchorB) > cacheMatchDistanceSquared)
						continue;

					point.normalImpulse = std::max(cached.normalImpulse, 0.0f);
					float tangentLimit = std::max(manifold.friction, 0.0f) * point.normalImpulse;
					point.tangentImpulse1 = Math3d::clamp(
						cached.tangentImpulse1, -tangentLimit, tangentLimit);
					point.tangentImpulse2 = Math3d::clamp(
						cached.tangentImpulse2, -tangentLimit, tangentLimit);
					Vec3 warmImpulse = manifold.normal * point.normalImpulse + preparedPoint.tangent1 * point.tangentImpulse1 + preparedPoint.tangent2 * point.tangentImpulse2;
					applyCachedImpulse(*prepared.bodyA, -warmImpulse, preparedPoint.offsetA);
					applyCachedImpulse(*prepared.bodyB, warmImpulse, preparedPoint.offsetB);
					++warmStartMatches;
					break;
				}
			}
		}
		stats.warmStartComparisons = warmStartComparisons;
		stats.warmStartMatches = warmStartMatches;
		stats.warmStartMs = elapsedMs(warmStart);

		auto velocityIterationsStart = Clock::now();
		for (int iteration = 0; iteration < iterations; ++iteration)
		{
			for (PreparedManifold &prepared : preparedContacts)
			{
				ContactManifold &manifold = *prepared.manifold;
				RigidBody *bodyA = prepared.bodyA->body;
				RigidBody *bodyB = prepared.bodyB->body;

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

					applyAxisImpulse(*prepared.bodyA, -impulse, preparedPoint.normalResponseA, -appliedImpulse);
					applyAxisImpulse(*prepared.bodyB, impulse, preparedPoint.normalResponseB, appliedImpulse);

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
					applyAxisImpulse(*prepared.bodyA, -tangentImpulse, preparedPoint.tangentResponseA1, -appliedTangentImpulse1);
					applyAxisImpulse(*prepared.bodyB, tangentImpulse, preparedPoint.tangentResponseB1, appliedTangentImpulse1);

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
					applyAxisImpulse(*prepared.bodyA, -tangentImpulse, preparedPoint.tangentResponseA2, -appliedTangentImpulse2);
					applyAxisImpulse(*prepared.bodyB, tangentImpulse, preparedPoint.tangentResponseB2, appliedTangentImpulse2);
				}
			}
		}
		stats.velocityPointVisits = preparedPointCount * iterations;
		stats.velocityIterationsMs = elapsedMs(velocityIterationsStart);

		auto cacheUpdateStart = Clock::now();
		auto &nextCache = world.nextCachedContacts;
		nextCache.clear();
		std::size_t requiredCacheSize = world.sleepingEnabled
			? world.cachedContacts.size()
			: 0;
		for (const PreparedManifold &prepared : preparedContacts)
			requiredCacheSize += prepared.manifold->pointCount;
		if (nextCache.capacity() < requiredCacheSize)
			nextCache.reserve(requiredCacheSize);
		if (world.sleepingEnabled)
			for (const auto &cached : world.cachedContacts)
			{
				const RigidBody *bodyA = world.getBody(cached.bodyA);
				const RigidBody *bodyB = world.getBody(cached.bodyB);
				if (bodyA && bodyB
					&& (bodyA->isSleeping() || bodyB->isSleeping())
					&& (bodyA->isStatic || bodyA->isSleeping())
					&& (bodyB->isStatic || bodyB->isSleeping()))
					nextCache.push_back(cached);
			}
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
		// Keep the first-match order within each pair; never reorder the live solve.
		if (std::is_sorted(nextCache.begin(), nextCache.end(), cachePairLess))
			world.cachedContacts.swap(nextCache);
		else
		{
			auto &cacheOrder = workspace.cacheOrder;
			cacheOrder.resize(nextCache.size());
			std::iota(cacheOrder.begin(), cacheOrder.end(), 0);
			std::sort(cacheOrder.begin(), cacheOrder.end(), [&](std::size_t left, std::size_t right) {
				if (cachePairLess(nextCache[left], nextCache[right]))
					return true;
				if (cachePairLess(nextCache[right], nextCache[left]))
					return false;
				return left < right;
			});
			world.cachedContacts.clear();
			if (world.cachedContacts.capacity() < nextCache.size())
				world.cachedContacts.reserve(nextCache.size());
			for (std::size_t index : cacheOrder)
				world.cachedContacts.push_back(nextCache[index]);
			nextCache.clear();
		}

		preparedContacts.clear();
		stats.cacheUpdateMs = elapsedMs(cacheUpdateStart);
	}

}

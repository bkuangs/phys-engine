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

		std::size_t prepareContactPoints(
			PreparedManifold &prepared, float inverseDt)
		{
			constexpr float slop = 0.005f;
			ContactManifold &manifold = *prepared.manifold;
			RigidBody *bodyA = prepared.bodyA->body;
			RigidBody *bodyB = prepared.bodyB->body;
			std::size_t pointCount = 0;
			for (uint32_t index = 0; index < manifold.pointCount; ++index)
			{
				ContactPoint &point = manifold.points[index];
				PreparedContactPoint &preparedPoint = prepared.points[index];
				preparedPoint.offsetA = bodyA->getRotation().rotate(point.localAnchorA);
				preparedPoint.offsetB = bodyB->getRotation().rotate(point.localAnchorB);

				Vec3 tangentReference = std::abs(manifold.normal.x) < 0.9f
					? Vec3{1.0f, 0.0f, 0.0f}
					: Vec3{0.0f, 1.0f, 0.0f};
				preparedPoint.tangent1 = Math3d::cross(
					manifold.normal, tangentReference);
				float tangentLength = std::sqrt(dot(
					preparedPoint.tangent1, preparedPoint.tangent1));
				if (tangentLength <= 1e-6f)
					continue;
				preparedPoint.tangent1 = preparedPoint.tangent1 / tangentLength;
				preparedPoint.tangent2 = Math3d::cross(
					manifold.normal, preparedPoint.tangent1);

				Vec3 velocityA = bodyA->getLinearVelocity()
					+ Math3d::cross(bodyA->getAngularVelocity(), preparedPoint.offsetA);
				Vec3 velocityB = bodyB->getLinearVelocity()
					+ Math3d::cross(bodyB->getAngularVelocity(), preparedPoint.offsetB);
				float incomingVelocityAlongNormal = dot(
					velocityB - velocityA, manifold.normal);

				Vec3 angularJacobianA = Math3d::cross(
					preparedPoint.offsetA, manifold.normal);
				Vec3 angularJacobianB = Math3d::cross(
					preparedPoint.offsetB, manifold.normal);
				preparedPoint.normalResponseA =
					prepared.bodyA->inverseInertiaWorld * angularJacobianA;
				preparedPoint.normalResponseB =
					prepared.bodyB->inverseInertiaWorld * angularJacobianB;
				float inverseMassSum =
					prepared.bodyA->inverseMass + prepared.bodyB->inverseMass;
				preparedPoint.inverseEffectiveMass = inverseMassSum
					+ dot(angularJacobianA, preparedPoint.normalResponseA)
					+ dot(angularJacobianB, preparedPoint.normalResponseB);

				Vec3 tangentAngularJacobianA = Math3d::cross(
					preparedPoint.offsetA, preparedPoint.tangent1);
				Vec3 tangentAngularJacobianB = Math3d::cross(
					preparedPoint.offsetB, preparedPoint.tangent1);
				preparedPoint.tangentResponseA1 =
					prepared.bodyA->inverseInertiaWorld * tangentAngularJacobianA;
				preparedPoint.tangentResponseB1 =
					prepared.bodyB->inverseInertiaWorld * tangentAngularJacobianB;
				float tangentMass00 = inverseMassSum
					+ dot(tangentAngularJacobianA, preparedPoint.tangentResponseA1)
					+ dot(tangentAngularJacobianB, preparedPoint.tangentResponseB1);
				preparedPoint.normalTangentResponse1 =
					inverseMassSum * dot(manifold.normal, preparedPoint.tangent1)
					+ dot(tangentAngularJacobianA, preparedPoint.normalResponseA)
					+ dot(tangentAngularJacobianB, preparedPoint.normalResponseB);

				tangentAngularJacobianA = Math3d::cross(
					preparedPoint.offsetA, preparedPoint.tangent2);
				tangentAngularJacobianB = Math3d::cross(
					preparedPoint.offsetB, preparedPoint.tangent2);
				preparedPoint.tangentResponseA2 =
					prepared.bodyA->inverseInertiaWorld * tangentAngularJacobianA;
				preparedPoint.tangentResponseB2 =
					prepared.bodyB->inverseInertiaWorld * tangentAngularJacobianB;
				float tangentMass11 = inverseMassSum
					+ dot(tangentAngularJacobianA, preparedPoint.tangentResponseA2)
					+ dot(tangentAngularJacobianB, preparedPoint.tangentResponseB2);
				preparedPoint.normalTangentResponse2 =
					inverseMassSum * dot(manifold.normal, preparedPoint.tangent2)
					+ dot(tangentAngularJacobianA, preparedPoint.normalResponseA)
					+ dot(tangentAngularJacobianB, preparedPoint.normalResponseB);
				float tangentMass01 =
					dot(tangentAngularJacobianA, preparedPoint.tangentResponseA1)
					+ dot(tangentAngularJacobianB, preparedPoint.tangentResponseB1);
				float tangentDeterminant =
					tangentMass00 * tangentMass11 - tangentMass01 * tangentMass01;
				if (tangentDeterminant > 0.0f)
				{
					float inverseTangentDeterminant = 1.0f / tangentDeterminant;
					preparedPoint.inverseTangentMass00 =
						tangentMass11 * inverseTangentDeterminant;
					preparedPoint.inverseTangentMass01 =
						-tangentMass01 * inverseTangentDeterminant;
					preparedPoint.inverseTangentMass11 =
						tangentMass00 * inverseTangentDeterminant;
				}
				else
				{
					preparedPoint.inverseTangentMass00 = 0.0f;
					preparedPoint.inverseTangentMass01 = 0.0f;
					preparedPoint.inverseTangentMass11 = 0.0f;
				}
				if (preparedPoint.inverseEffectiveMass <= 0.0f)
					continue;
				++pointCount;

				preparedPoint.bias = std::max(
					point.penetration - slop, 0.0f) * (0.2f * inverseDt);
				preparedPoint.restitutionVelocity =
					incomingVelocityAlongNormal < -1.0f
						? manifold.restitution * incomingVelocityAlongNormal
						: 0.0f;
			}
			return pointCount;
		}

	}

	void SequentialImpulseSolver::solve(std::vector<ContactManifold> &contacts,
										PhysicsWorld &world, float dt)
	{
		detail::SolverWorkspace workspace;
		detail::ParallelFor workers;
		solve(contacts, world, dt, workspace, workers, world.solverWorkerCount);
	}

	void SequentialImpulseSolver::solve(std::vector<ContactManifold> &contacts,
										PhysicsWorld &world, float dt,
										detail::SolverWorkspace &workspace,
										detail::ParallelFor &workers,
										std::size_t workerCount)
	{
		SolverStats &stats = world.stats.solverDetails;
		stats = {};
		auto prepareStart = Clock::now();
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
		auto applyTangentImpulse = [](const PreparedBody &prepared, const Vec3 &impulse,
									  const Vec3 &angularResponse) {
			RigidBody *body = prepared.body;
			if (body->isStatic)
				return;
			body->linearVelocity += impulse * prepared.inverseMass;
			body->angularVelocity += angularResponse;
		};

		constexpr std::size_t minimumParallelContacts = 4096;
		const bool deferPreparation = workerCount > 1
			&& contacts.size() >= minimumParallelContacts;
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
			if (!deferPreparation)
				preparedPointCount += prepareContactPoints(prepared, inverseDt);
			preparedContacts.push_back(prepared);
		}

		if (deferPreparation)
		{
			auto &preparedPointCounts = workspace.preparedPointCounts;
			preparedPointCounts.resize(preparedContacts.size());
			auto preparePoints = [&](std::size_t index) {
				preparedPointCounts[index] =
					prepareContactPoints(preparedContacts[index], inverseDt);
			};
			if (preparedContacts.size() >= minimumParallelContacts)
				workers.run(
					workerCount, preparedContacts.size(), preparePoints);
			else
				for (std::size_t index = 0; index < preparedContacts.size(); ++index)
					preparePoints(index);
			preparedPointCount = std::accumulate(
				preparedPointCounts.begin(), preparedPointCounts.end(), std::size_t{0});
		}
		world.stats.solvedContactCount = preparedContacts.size();
		stats.preparedPoints = preparedPointCount;

		auto &islandParents = workspace.islandParents;
		auto &islandLookup = workspace.islandLookup;
		auto &contactIslands = workspace.contactIslands;
		auto &islandContactCounts = workspace.islandContactCounts;
		auto &islandPointCounts = workspace.islandPointCounts;
		auto &islandOffsets = workspace.islandOffsets;
		auto &islandWriteOffsets = workspace.islandWriteOffsets;
		auto &islandContactIndices = workspace.islandContactIndices;
		auto &islandOrder = workspace.islandOrder;
		bool parallelIslands = workerCount > 1
			&& preparedContacts.size() >= minimumParallelContacts;
		if (parallelIslands)
		{
			islandParents.resize(world.slots.size());
			std::iota(islandParents.begin(), islandParents.end(), 0);
			auto islandRoot = [&](std::size_t index) {
				while (islandParents[index] != index)
				{
					islandParents[index] = islandParents[islandParents[index]];
					index = islandParents[index];
				}
				return index;
			};
			for (const PreparedManifold &prepared : preparedContacts)
			{
				if (prepared.bodyA->body->isStatic || prepared.bodyB->body->isStatic)
					continue;
				std::size_t rootA = islandRoot(prepared.manifold->bodyA.index);
				std::size_t rootB = islandRoot(prepared.manifold->bodyB.index);
				if (rootA != rootB)
					islandParents[std::max(rootA, rootB)] = std::min(rootA, rootB);
			}

			const std::size_t invalidIsland = preparedContacts.size();
			islandLookup.assign(world.slots.size(), invalidIsland);
			contactIslands.resize(preparedContacts.size());
			islandContactCounts.clear();
			islandPointCounts.clear();
			for (std::size_t index = 0; index < preparedContacts.size(); ++index)
			{
				const PreparedManifold &prepared = preparedContacts[index];
				std::size_t dynamicBody = prepared.bodyA->body->isStatic
					? prepared.manifold->bodyB.index
					: prepared.manifold->bodyA.index;
				std::size_t root = islandRoot(dynamicBody);
				std::size_t &island = islandLookup[root];
				if (island == invalidIsland)
				{
					island = islandContactCounts.size();
					islandContactCounts.push_back(0);
					islandPointCounts.push_back(0);
				}
				contactIslands[index] = island;
				++islandContactCounts[island];
				islandPointCounts[island] += prepared.manifold->pointCount;
			}

			islandOffsets.resize(islandContactCounts.size() + 1);
			islandOffsets[0] = 0;
			std::partial_sum(
				islandContactCounts.begin(), islandContactCounts.end(),
				islandOffsets.begin() + 1);
			islandWriteOffsets.assign(islandOffsets.begin(), islandOffsets.end() - 1);
			islandContactIndices.resize(preparedContacts.size());
			for (std::size_t index = 0; index < preparedContacts.size(); ++index)
			{
				std::size_t island = contactIslands[index];
				islandContactIndices[islandWriteOffsets[island]++] = index;
			}

			stats.islandCount = islandContactCounts.size();
			for (std::size_t count : islandContactCounts)
				stats.largestIslandContacts = std::max(stats.largestIslandContacts, count);
			for (std::size_t count : islandPointCounts)
				stats.largestIslandPoints = std::max(stats.largestIslandPoints, count);
			parallelIslands = islandContactCounts.size() > 1;
			islandOrder.resize(islandContactCounts.size());
			std::iota(islandOrder.begin(), islandOrder.end(), 0);
			auto largestIsland = std::max_element(
				islandPointCounts.begin(), islandPointCounts.end());
			if (largestIsland != islandPointCounts.end())
				std::swap(
					islandOrder.front(),
					islandOrder[static_cast<std::size_t>(
						largestIsland - islandPointCounts.begin())]);
		}
		stats.prepareMs = elapsedMs(prepareStart);

		// Restore impulses from the previous step before the iterative solve.
		// Local anchors provide a stable contact identity while body handles
		// distinguish contacts belonging to different body pairs.
		auto cachePairLess = [](const auto &left, const auto &right) {
			return std::tie(left.bodyA.index, left.bodyA.generation, left.bodyB.index, left.bodyB.generation)
				< std::tie(right.bodyA.index, right.bodyA.generation, right.bodyB.index, right.bodyB.generation);
		};
		constexpr float cacheMatchDistanceSquared = 0.05f * 0.05f;
		auto warmStartPrepared = [&](PreparedManifold &prepared,
									 std::size_t &comparisons,
									 std::size_t &matches) {
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
					++comparisons;
					const PhysicsWorld::CachedContact &cached = *cachedPoint;
					if (distanceSquared(cached.localAnchorA, point.localAnchorA) > cacheMatchDistanceSquared || distanceSquared(cached.localAnchorB, point.localAnchorB) > cacheMatchDistanceSquared)
						continue;

					point.normalImpulse = std::max(cached.normalImpulse, 0.0f);
					float tangentLimit = std::max(manifold.friction, 0.0f) * point.normalImpulse;
					float tangentLengthSquared =
						cached.tangentImpulse1 * cached.tangentImpulse1
						+ cached.tangentImpulse2 * cached.tangentImpulse2;
					float tangentScale = tangentLengthSquared > tangentLimit * tangentLimit
						? tangentLimit / std::sqrt(tangentLengthSquared)
						: 1.0f;
					point.tangentImpulse1 = cached.tangentImpulse1 * tangentScale;
					point.tangentImpulse2 = cached.tangentImpulse2 * tangentScale;
					Vec3 warmImpulse = manifold.normal * point.normalImpulse + preparedPoint.tangent1 * point.tangentImpulse1 + preparedPoint.tangent2 * point.tangentImpulse2;
					applyCachedImpulse(*prepared.bodyA, -warmImpulse, preparedPoint.offsetA);
					applyCachedImpulse(*prepared.bodyB, warmImpulse, preparedPoint.offsetB);
					++matches;
					break;
				}
			}
		};
		auto warmStart = Clock::now();
		if (parallelIslands)
		{
			auto &islandComparisons = workspace.islandWarmStartComparisons;
			auto &islandMatches = workspace.islandWarmStartMatches;
			islandComparisons.assign(islandContactCounts.size(), 0);
			islandMatches.assign(islandContactCounts.size(), 0);
			auto warmStartIsland = [&](std::size_t job) {
				std::size_t island = islandOrder[job];
				for (std::size_t offset = islandOffsets[island];
					 offset < islandOffsets[island + 1]; ++offset)
					warmStartPrepared(
						preparedContacts[islandContactIndices[offset]],
						islandComparisons[island], islandMatches[island]);
			};
			workers.run(
				workerCount, islandContactCounts.size(), warmStartIsland, 1);
			warmStartComparisons = std::accumulate(
				islandComparisons.begin(), islandComparisons.end(), std::size_t{0});
			warmStartMatches = std::accumulate(
				islandMatches.begin(), islandMatches.end(), std::size_t{0});
		}
		else
			for (PreparedManifold &prepared : preparedContacts)
				warmStartPrepared(prepared, warmStartComparisons, warmStartMatches);
		stats.warmStartComparisons = warmStartComparisons;
		stats.warmStartMatches = warmStartMatches;
		stats.warmStartMs = elapsedMs(warmStart);

		auto solvePrepared = [&](PreparedManifold &prepared) {
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
					const Vec3 &offsetA = preparedPoint.offsetA;
					const Vec3 &offsetB = preparedPoint.offsetB;
					const Vec3 velocityA = bodyA->getLinearVelocity() + Math3d::cross(bodyA->getAngularVelocity(), offsetA);
					const Vec3 velocityB = bodyB->getLinearVelocity() + Math3d::cross(bodyB->getAngularVelocity(), offsetB);
					const Vec3 relativeVelocity = velocityB - velocityA;

					// Given that these objects are in contact, how fast are their already-known
					// contact points moving relative to each other along the contact normal?
					float velocityAlongNormal = dot(relativeVelocity, manifold.normal);

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

					if (preparedPoint.inverseTangentMass00 <= 0.0f
						|| preparedPoint.inverseTangentMass11 <= 0.0f)
						continue;

					// Derive post-normal tangent velocity from the pre-normal velocity
					// and the cached response, then solve against the Coulomb disk.
					float tangentLimit = std::max(manifold.friction, 0.0f) * point.normalImpulse;
					float tangentVelocity1 = dot(
						relativeVelocity, preparedPoint.tangent1)
						+ preparedPoint.normalTangentResponse1 * appliedImpulse;
					float tangentVelocity2 = dot(
						relativeVelocity, preparedPoint.tangent2)
						+ preparedPoint.normalTangentResponse2 * appliedImpulse;
					float tangentImpulseDelta1 = -(
						preparedPoint.inverseTangentMass00 * tangentVelocity1
						+ preparedPoint.inverseTangentMass01 * tangentVelocity2);
					float tangentImpulseDelta2 = -(
						preparedPoint.inverseTangentMass01 * tangentVelocity1
						+ preparedPoint.inverseTangentMass11 * tangentVelocity2);
					float previousTangentImpulse1 = point.tangentImpulse1;
					float previousTangentImpulse2 = point.tangentImpulse2;
					point.tangentImpulse1 += tangentImpulseDelta1;
					point.tangentImpulse2 += tangentImpulseDelta2;
					float tangentLengthSquared =
						point.tangentImpulse1 * point.tangentImpulse1
						+ point.tangentImpulse2 * point.tangentImpulse2;
					if (tangentLengthSquared > tangentLimit * tangentLimit)
					{
						float scale = tangentLimit / std::sqrt(tangentLengthSquared);
						point.tangentImpulse1 *= scale;
						point.tangentImpulse2 *= scale;
					}
					float appliedTangentImpulse1 =
						point.tangentImpulse1 - previousTangentImpulse1;
					float appliedTangentImpulse2 = point.tangentImpulse2 - previousTangentImpulse2;
					Vec3 tangentImpulse =
						preparedPoint.tangent1 * appliedTangentImpulse1
						+ preparedPoint.tangent2 * appliedTangentImpulse2;
					Vec3 angularResponseA =
						preparedPoint.tangentResponseA1 * appliedTangentImpulse1
						+ preparedPoint.tangentResponseA2 * appliedTangentImpulse2;
					Vec3 angularResponseB =
						preparedPoint.tangentResponseB1 * appliedTangentImpulse1
						+ preparedPoint.tangentResponseB2 * appliedTangentImpulse2;
					applyTangentImpulse(*prepared.bodyA, -tangentImpulse, -angularResponseA);
					applyTangentImpulse(*prepared.bodyB, tangentImpulse, angularResponseB);
			}
		};
		auto velocityIterationsStart = Clock::now();
		if (parallelIslands)
		{
			auto solveIsland = [&](std::size_t job) {
				std::size_t island = islandOrder[job];
				for (int iteration = 0; iteration < iterations; ++iteration)
					for (std::size_t offset = islandOffsets[island];
						 offset < islandOffsets[island + 1]; ++offset)
						solvePrepared(preparedContacts[islandContactIndices[offset]]);
			};
			workers.run(workerCount, islandContactCounts.size(), solveIsland, 1);
		}
		else
			for (int iteration = 0; iteration < iterations; ++iteration)
				for (PreparedManifold &prepared : preparedContacts)
					solvePrepared(prepared);
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

#include <phys/world/physics_world.hpp>
#include <phys/collision/narrowphase.hpp>
#include <phys/solver/sequential_impulse_solver.hpp>
#include <chrono>
#include <algorithm>
#include <cmath>
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

    }

    RigidBodyHandle PhysicsWorld::addBody(const RigidBody &body)
    {
        if (!freeList.empty())
        { // re-use deleted bodies
            uint32_t index = freeList.back();
            freeList.pop_back();

            Slot &slot = slots[index];
            slot.body = body;
            slot.alive = true;
            return {index, slot.generation};
        }

        slots.push_back(Slot{body, 0, true});
        return {static_cast<uint32_t>(slots.size() - 1), 0};
    }

    bool PhysicsWorld::createSphere(float radius, Vec3 position, float density,
                                    bool isStatic, float restitution, float friction, RigidBodyHandle &body,
                                    ColliderHandle &collider, std::string &errorMessage)
    {
        RigidBody createdBody;
        if (!RigidBody::createSphere(radius, position, density, isStatic,
                                     restitution, friction, createdBody, errorMessage))
        {
            return false;
        }

        body = addBody(createdBody);

        // Create global collision geometry and link Collider to RigidBody.
        // Tells the engine that a new sphere is physically occupying space.
        Collider createdCollider;
        createdCollider.body = body;
        createdCollider.shape = Sphere{radius};
        collider = addCollider(createdCollider);
        return true;
    }

    bool PhysicsWorld::createBox(float width, float height, float depth, Vec3 position,
                                 float density, bool isStatic, float restitution, float friction,
                                 RigidBodyHandle &body,
                                 ColliderHandle &collider, std::string &errorMessage)
    {
        RigidBody createdBody;
        if (!RigidBody::createBox(width, height, depth, position, density, isStatic,
                                  restitution, friction, createdBody, errorMessage))
        {
            return false;
        }

        body = addBody(createdBody);
        Collider createdCollider;
        createdCollider.body = body;
        createdCollider.shape = Box{{width * 0.5f, height * 0.5f, depth * 0.5f}};
        collider = addCollider(createdCollider);
        return true;
    }

    void PhysicsWorld::removeBody(RigidBodyHandle handle)
    {
        if (handle.index >= slots.size())
            return; // index out of bounds

        Slot &slot = slots[handle.index];

        // Check that the slot is being used and reject old handles
        if (!slot.alive || slot.generation != handle.generation)
            return;

        slot.alive = false;
        slot.generation++;
        freeList.push_back(handle.index);

        // Remove all colliders belonging to this body
        for (uint32_t index = 0; index < colliderSlots.size(); ++index)
        {
            ColliderSlot &colliderSlot = colliderSlots[index];
            if (colliderSlot.alive && colliderSlot.collider.body == handle)
            {
                colliderSlot.alive = false;
                colliderSlot.generation++;
                freeColliderList.push_back(index);
            }
        }
    }

    RigidBody *PhysicsWorld::getBody(RigidBodyHandle handle)
    {
        if (handle.index >= slots.size())
            return nullptr;

        Slot &slot = slots[handle.index];
        if (!slot.alive || slot.generation != handle.generation)
            return nullptr;

        return &slot.body;
    }

    const RigidBody *PhysicsWorld::getBody(RigidBodyHandle handle) const
    {
        if (handle.index >= slots.size())
            return nullptr;

        const Slot &slot = slots[handle.index];
        if (!slot.alive || slot.generation != handle.generation)
            return nullptr;

        return &slot.body;
    }

    ColliderHandle PhysicsWorld::addCollider(const Collider &collider)
    {
        if (!getBody(collider.body))
            return {};

        if (!freeColliderList.empty())
        {
            uint32_t index = freeColliderList.back();
            freeColliderList.pop_back();

            ColliderSlot &slot = colliderSlots[index];
            slot.collider = collider;
            slot.alive = true;
            return {index, slot.generation};
        }

        colliderSlots.push_back(ColliderSlot{collider, 0, true});
        return {static_cast<uint32_t>(colliderSlots.size() - 1), 0};
    }

    void PhysicsWorld::removeCollider(ColliderHandle handle)
    {
        if (handle.index >= colliderSlots.size())
            return;

        ColliderSlot &slot = colliderSlots[handle.index];
        if (!slot.alive || slot.generation != handle.generation)
            return;

        slot.alive = false;
        slot.generation++;
        freeColliderList.push_back(handle.index);
    }

    Collider *PhysicsWorld::getCollider(ColliderHandle handle)
    {
        if (handle.index >= colliderSlots.size())
            return nullptr;

        ColliderSlot &slot = colliderSlots[handle.index];
        if (!slot.alive || slot.generation != handle.generation)
            return nullptr;

        return &slot.collider;
    }

    const Collider *PhysicsWorld::getCollider(ColliderHandle handle) const
    {
        if (handle.index >= colliderSlots.size())
            return nullptr;

        const ColliderSlot &slot = colliderSlots[handle.index];
        if (!slot.alive || slot.generation != handle.generation)
            return nullptr;

        return &slot.collider;
    }

    void PhysicsWorld::step(float dt)
    {
        auto stepStart = Clock::now();

        auto velocityStart = Clock::now();

        // Apply gravity to active bodies
        for (Slot &slot : slots)
        {
            if (!slot.alive)
                continue;
            slot.body.integrateVelocity(gravity, dt);
        }
        stats.integrateVelocityMs = elapsedMs(velocityStart);

        // Get world-space collider AABBs from current body poses
        for (ColliderSlot &slot : colliderSlots)
        {
            if (!slot.alive)
                continue;

            const RigidBody *body = getBody(slot.collider.body);
            if (!body)
                continue;

            Transform bodyTransform{body->getPosition(), body->getRotation()};
            slot.collider.bounds = Aabb::fromCollider(slot.collider, bodyTransform);
        }

        // BROAD-PHASE: Collect AABB-overlapping collider pairs as candidates.
        auto broadPhaseStart = Clock::now();
        std::vector<std::pair<uint32_t, uint32_t>> candidatePairs;
        std::size_t possiblePairs = 0;

        for (uint32_t first = 0; first < colliderSlots.size(); ++first)
        {
            ColliderSlot &firstSlot = colliderSlots[first];
            if (!firstSlot.alive)
                continue;

            for (uint32_t second = first + 1; second < colliderSlots.size(); ++second)
            {
                ColliderSlot &secondSlot = colliderSlots[second];
                if (!secondSlot.alive)
                    continue;
                if (firstSlot.collider.body == secondSlot.collider.body)
                    continue;

                ++possiblePairs;
                if (firstSlot.collider.bounds.overlaps(secondSlot.collider.bounds))
                {
                    candidatePairs.emplace_back(first, second);
                }
            }
        }
        stats.broadPhaseMs = elapsedMs(broadPhaseStart);
        stats.possiblePairs = possiblePairs;
        stats.candidatePairs = candidatePairs.size();

        // NARROW-PHASE: Each candidate pair gets delegated to shape-specific query.
        auto narrowPhaseStart = Clock::now();
        currentContacts.clear();

        for (const auto &pair : candidatePairs)
        {
            ColliderSlot &firstSlot = colliderSlots[pair.first];
            ColliderSlot &secondSlot = colliderSlots[pair.second];

            const RigidBody *bodyA = getBody(firstSlot.collider.body);
            const RigidBody *bodyB = getBody(secondSlot.collider.body);
            if (!bodyA || !bodyB)
                continue;

            Transform bodyTransformA{bodyA->getPosition(), bodyA->getRotation()};
            Transform bodyTransformB{bodyB->getPosition(), bodyB->getRotation()};
            Transform transformA{
                bodyTransformA.position + bodyTransformA.orientation.rotate(
                                              firstSlot.collider.localTransform.position),
                bodyTransformA.orientation * firstSlot.collider.localTransform.orientation};
            Transform transformB{
                bodyTransformB.position + bodyTransformB.orientation.rotate(
                                              secondSlot.collider.localTransform.position),
                bodyTransformB.orientation * secondSlot.collider.localTransform.orientation};

            ContactManifold manifold;
            if (NarrowPhase::generateContact(
                    firstSlot.collider, transformA,
                    secondSlot.collider, transformB,
                    manifold))
            {
                for (uint32_t index = 0; index < manifold.pointCount; ++index)
                {
                    ContactPoint &point = manifold.points[index];
                    point.localAnchorA = firstSlot.collider.localTransform.position + firstSlot.collider.localTransform.orientation.rotate(
                                                                                          point.localAnchorA);
                    point.localAnchorB = secondSlot.collider.localTransform.position + secondSlot.collider.localTransform.orientation.rotate(
                                                                                           point.localAnchorB);
                }
                manifold.restitution = std::max(
                    bodyA->restitution, bodyB->restitution);
                manifold.friction = std::sqrt(
                    std::max(bodyA->friction, 0.0f) * std::max(bodyB->friction, 0.0f));
                currentContacts.push_back(manifold);
            }
        }
        stats.narrowPhaseMs = elapsedMs(narrowPhaseStart);
        stats.contactCount = currentContacts.size();

        auto solverStart = Clock::now();
        SequentialImpulseSolver::solve(currentContacts, *this, dt);
        stats.solverMs = elapsedMs(solverStart);

        // Pose-integration: advance each active body by its current velocities.
        auto poseStart = Clock::now();
        for (Slot &slot : slots)
        {
            if (!slot.alive)
                continue;

            slot.body.integratePosition(dt);
            slot.body.integrateRotation(dt);
            slot.body.clearForces();
        }
        stats.integratePoseMs = elapsedMs(poseStart);

        stats.totalMs = elapsedMs(stepStart);
    }

}

#include <phys/world/physics_world.hpp>
#include <phys/dynamics/integrator.hpp>

namespace phys {

RigidBodyHandle PhysicsWorld::addBody(const RigidBody& body)
{
    if (!freeList.empty()) {
        uint32_t index = freeList.back();
        freeList.pop_back();

        Slot& slot = slots[index];
        slot.body = body;
        slot.alive = true;
        return {index, slot.generation};
    }

    slots.push_back(Slot{body, 0, true});
    return {static_cast<uint32_t>(slots.size() - 1), 0};
}

bool PhysicsWorld::createSphere(float radius, Vec3 position, float density,
    bool isStatic, float restitution, RigidBodyHandle& body,
    ColliderHandle& collider, std::string& errorMessage)
{
    RigidBody createdBody;
    if (!RigidBody::createSphere(radius, position, density, isStatic,
        restitution, createdBody, errorMessage)) {
        return false;
    }

    body = addBody(createdBody);
    Collider createdCollider;
    createdCollider.body = body;
    createdCollider.type = ShapeType::Sphere;
    createdCollider.sphere.radius = radius;
    collider = addCollider(createdCollider);
    return true;
}

bool PhysicsWorld::createBox(float width, float height, float depth, Vec3 position,
    float density, bool isStatic, float restitution, RigidBodyHandle& body,
    ColliderHandle& collider, std::string& errorMessage)
{
    RigidBody createdBody;
    if (!RigidBody::createBox(width, height, depth, position, density, isStatic,
        restitution, createdBody, errorMessage)) {
        return false;
    }

    body = addBody(createdBody);
    Collider createdCollider;
    createdCollider.body = body;
    createdCollider.type = ShapeType::Box;
    createdCollider.box.halfExtents = {width * 0.5f, height * 0.5f, depth * 0.5f};
    collider = addCollider(createdCollider);
    return true;
}

void PhysicsWorld::removeBody(RigidBodyHandle handle)
{
    if (handle.index >= slots.size()) return;

    Slot& slot = slots[handle.index];
    if (!slot.alive || slot.generation != handle.generation) return;

    slot.alive = false;
    slot.generation++;
    freeList.push_back(handle.index);

    for (uint32_t index = 0; index < colliderSlots.size(); ++index) {
        ColliderSlot& colliderSlot = colliderSlots[index];
        if (colliderSlot.alive && colliderSlot.collider.body == handle) {
            colliderSlot.alive = false;
            colliderSlot.generation++;
            freeColliderList.push_back(index);
        }
    }
}

RigidBody* PhysicsWorld::getBody(RigidBodyHandle handle)
{
    if (handle.index >= slots.size()) return nullptr;

    Slot& slot = slots[handle.index];
    if (!slot.alive || slot.generation != handle.generation) return nullptr;

    return &slot.body;
}

const RigidBody* PhysicsWorld::getBody(RigidBodyHandle handle) const
{
    if (handle.index >= slots.size()) return nullptr;

    const Slot& slot = slots[handle.index];
    if (!slot.alive || slot.generation != handle.generation) return nullptr;

    return &slot.body;
}

ColliderHandle PhysicsWorld::addCollider(const Collider& collider)
{
    if (!getBody(collider.body)) return {};

    if (!freeColliderList.empty()) {
        uint32_t index = freeColliderList.back();
        freeColliderList.pop_back();

        ColliderSlot& slot = colliderSlots[index];
        slot.collider = collider;
        slot.alive = true;
        return {index, slot.generation};
    }

    colliderSlots.push_back(ColliderSlot{collider, 0, true});
    return {static_cast<uint32_t>(colliderSlots.size() - 1), 0};
}

void PhysicsWorld::removeCollider(ColliderHandle handle)
{
    if (handle.index >= colliderSlots.size()) return;

    ColliderSlot& slot = colliderSlots[handle.index];
    if (!slot.alive || slot.generation != handle.generation) return;

    slot.alive = false;
    slot.generation++;
    freeColliderList.push_back(handle.index);
}

Collider* PhysicsWorld::getCollider(ColliderHandle handle)
{
    if (handle.index >= colliderSlots.size()) return nullptr;

    ColliderSlot& slot = colliderSlots[handle.index];
    if (!slot.alive || slot.generation != handle.generation) return nullptr;

    return &slot.collider;
}

const Collider* PhysicsWorld::getCollider(ColliderHandle handle) const
{
    if (handle.index >= colliderSlots.size()) return nullptr;

    const ColliderSlot& slot = colliderSlots[handle.index];
    if (!slot.alive || slot.generation != handle.generation) return nullptr;

    return &slot.collider;
}

void PhysicsWorld::step(float dt)
{
    for (Slot& slot : slots) {
        if (!slot.alive) continue;
        integrateVelocity(slot.body, gravity, dt);
    }

    // TODO: update collider world transforms/bounds from current body poses.
    // TODO: compute broad-phase candidate pairs (Collision::BroadPhase).
    // TODO: generate narrow-phase contact manifolds (Collision::NarrowPhase).
    // TODO: prepare/solve contact constraints (Solver::SequentialImpulseSolver).

    // Pose-integration: advance each active body by its current velocities.
    for (Slot& slot : slots) {
        if (!slot.alive) continue;

        slot.body.integratePosition(dt);
        slot.body.integrateRotation(dt);
        slot.body.clearForces();
    }

    for (ColliderSlot& slot : colliderSlots) {
        if (!slot.alive) continue;

        const RigidBody* body = getBody(slot.collider.body);
        if (!body) continue;

        Transform bodyTransform{body->getPosition(), body->getRotation()};
        slot.collider.bounds = Aabb::fromCollider(slot.collider, bodyTransform);
    }
}

}

// TODO: Orchestrate the pipeline documented in docs/architecture.md.

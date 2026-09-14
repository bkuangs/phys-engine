#include <phys/world/physics_world.hpp>

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

void PhysicsWorld::removeBody(RigidBodyHandle handle)
{
    if (handle.index >= slots.size()) return;

    Slot& slot = slots[handle.index];
    if (!slot.alive || slot.generation != handle.generation) return;

    slot.alive = false;
    slot.generation++;
    freeList.push_back(handle.index);
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

void PhysicsWorld::step(float dt)
{
    // TODO: integrate forces/torques into velocities (Dynamics::Integrator).
    // TODO: update collider world transforms/bounds from current body poses.
    // TODO: compute broad-phase candidate pairs (Collision::BroadPhase).
    // TODO: generate narrow-phase contact manifolds (Collision::NarrowPhase).
    // TODO: prepare/solve contact constraints (Solver::SequentialImpulseSolver).
    // TODO: integrate corrected velocities into positions/orientations.
    (void)dt;
}

}

// TODO: Orchestrate the pipeline documented in docs/architecture.md.

#pragma once
#include <cstdint>
#include <vector>
#include "phys/core/constants.hpp"
#include "phys/math/vec3.hpp"
#include "phys/dynamics/rigid_body.hpp"

namespace phys {

// Opaque, stable reference to a body. Validated via generation on lookup,
// so it stays safe to hold across addBody/removeBody calls elsewhere.
struct RigidBodyHandle
{
    uint32_t index = 0;
    uint32_t generation = 0;
};

inline bool operator==(const RigidBodyHandle& a, const RigidBodyHandle& b) {
    return a.index == b.index && a.generation == b.generation;
}

class PhysicsWorld
{
public:
    // Use 'static' members to access variables without having to create an object
    static constexpr float minBodySize = BodyLimits::minSize;
    static constexpr float maxBodySize = BodyLimits::maxSize;
    static constexpr float minDensity = BodyLimits::minDensity;
    static constexpr float maxDensity = BodyLimits::maxDensity;

    Vec3 gravity{0.0f, -9.81f, 0.0f};

    RigidBodyHandle addBody(const RigidBody& body);
    void removeBody(RigidBodyHandle handle);

    // Returns nullptr if the handle is stale (body removed / never valid).
    // Do not hold the returned pointer across another addBody/removeBody call.
    RigidBody* getBody(RigidBodyHandle handle);
    const RigidBody* getBody(RigidBodyHandle handle) const;

    // Advances the simulation by a fixed timestep. See docs/architecture.md
    // "Step pipeline" for stage ordering; stages are stubbed until their
    // owning subsystems (Integrator, BroadPhase, NarrowPhase, Solver) exist.
    void step(float dt);

private:
    struct Slot
    {
        RigidBody body;
        uint32_t generation = 0;
        bool alive = false;
    };

    std::vector<Slot> slots;
    std::vector<uint32_t> freeList;
};

}


struct RigidBodyHandle;

}

#pragma once
#include <string>
#include <vector>
#include "phys/core/constants.hpp"
#include "phys/math/vec3.hpp"
#include "phys/dynamics/rigid_body.hpp"
#include "phys/collision/collider.hpp"
#include "phys/collision/manifold.hpp"
#include "phys/world/body_handle.hpp"
#include "phys/world/step_stats.hpp"

namespace phys {

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

    bool createSphere(float radius, Vec3 position, float density, bool isStatic,
        float restitution, float friction, RigidBodyHandle& body,
        ColliderHandle& collider,
        std::string& errorMessage);
    bool createBox(float width, float height, float depth, Vec3 position,
        float density, bool isStatic, float restitution, float friction,
        RigidBodyHandle& body,
        ColliderHandle& collider, std::string& errorMessage);

    // Returns nullptr if the handle is stale.
    // Use the returned pointer only for immediate work!
    RigidBody* getBody(RigidBodyHandle handle);
    const RigidBody* getBody(RigidBodyHandle handle) const;

    ColliderHandle addCollider(const Collider& collider);
    void removeCollider(ColliderHandle handle);
    Collider* getCollider(ColliderHandle handle);
    const Collider* getCollider(ColliderHandle handle) const;
    const std::vector<ContactManifold>& contacts() const { return currentContacts; }

    void step(float dt);

    const StepStats& lastStepStats() const { return stats; }

private:
    struct Slot
    {
        RigidBody body;
        uint32_t generation = 0;
        bool alive = false;
    };

    std::vector<Slot> slots;
    std::vector<uint32_t> freeList;

    struct ColliderSlot
    {
        Collider collider;
        uint32_t generation = 0;
        bool alive = false;
    };

    std::vector<ColliderSlot> colliderSlots;
    std::vector<uint32_t> freeColliderList;
    std::vector<ContactManifold> currentContacts;
    StepStats stats;
};

}

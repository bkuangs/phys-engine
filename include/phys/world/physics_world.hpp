#pragma once
#include <memory>
#include <string>
#include <vector>
#include "phys/core/constants.hpp"
#include "phys/math/vec3.hpp"
#include "phys/dynamics/rigid_body.hpp"
#include "phys/collision/broadphase.hpp"
#include "phys/collision/dynamic_aabb_tree.hpp"
#include "phys/collision/collider.hpp"
#include "phys/collision/manifold.hpp"
#include "phys/world/body_handle.hpp"
#include "phys/world/step_stats.hpp"

namespace phys
{

    class SequentialImpulseSolver;

    class PhysicsWorld
    {
    public:
        PhysicsWorld();
        ~PhysicsWorld();
        PhysicsWorld(const PhysicsWorld &other);
        PhysicsWorld &operator=(const PhysicsWorld &other);
        PhysicsWorld(PhysicsWorld &&other) noexcept;
        PhysicsWorld &operator=(PhysicsWorld &&other) noexcept;

        // Use 'static' members to access variables without having to create an object
        static constexpr float minBodySize = BodyLimits::minSize;
        static constexpr float maxBodySize = BodyLimits::maxSize;
        static constexpr float minDensity = BodyLimits::minDensity;
        static constexpr float maxDensity = BodyLimits::maxDensity;

        Vec3 gravity{0.0f, -9.81f, 0.0f};
        BroadPhaseAlgorithm broadPhaseAlgorithm = BroadPhaseAlgorithm::SweepAndPrune;

        RigidBodyHandle addBody(const RigidBody &body);
        void removeBody(RigidBodyHandle handle);

        bool createSphere(float radius, Vec3 position, float density, bool isStatic,
                          float restitution, float friction, RigidBodyHandle &body,
                          ColliderHandle &collider,
                          std::string &errorMessage);
        bool createBox(float width, float height, float depth, Vec3 position,
                       float density, bool isStatic, float restitution, float friction,
                       RigidBodyHandle &body,
                       ColliderHandle &collider, std::string &errorMessage);

        // Returns nullptr if the handle is stale.
        // Use the returned pointer only for immediate work!
        RigidBody *getBody(RigidBodyHandle handle);
        const RigidBody *getBody(RigidBodyHandle handle) const;

        ColliderHandle addCollider(const Collider &collider);
        void removeCollider(ColliderHandle handle);
        Collider *getCollider(ColliderHandle handle);
        const Collider *getCollider(ColliderHandle handle) const;
        const std::vector<ContactManifold> &contacts() const { return currentContacts; }

        void setSleepingEnabled(bool enabled);
        bool isSleepingEnabled() const { return sleepingEnabled; }

        void step(float dt);

        const StepStats &lastStepStats() const { return stats; }

    private:
        struct CachedContact
        {
            RigidBodyHandle bodyA{};
            RigidBodyHandle bodyB{};
            Vec3 localAnchorA{};
            Vec3 localAnchorB{};
            float normalImpulse = 0.0f;
            float tangentImpulse1 = 0.0f;
            float tangentImpulse2 = 0.0f;
        };

        friend class SequentialImpulseSolver;

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
            DynamicAabbTree::ProxyId treeProxy = DynamicAabbTree::noProxy;
            uint32_t treeGeneration = 0;
        };

        std::vector<ColliderSlot> colliderSlots;
        DynamicAabbTree dynamicTree;
        std::vector<uint32_t> freeColliderList;
        std::vector<ContactManifold> currentContacts;
        std::vector<CachedContact> cachedContacts; // Sorted by ordered body handles; stable within each pair.
        std::vector<CachedContact> nextCachedContacts;
        StepStats stats;

        struct StepWorkspace;
        std::unique_ptr<StepWorkspace> stepWorkspace;

        struct SleepState
        {
            Transform pose{};
            float mass = 0.0f;
            float restitution = 0.0f;
            float friction = 0.0f;
            bool isStatic = false;
            bool integratedVelocity = false;
            uint32_t parent = 0;
            bool hasAwake = false;
            bool hasSleeping = false;
            bool needsWake = false;
            float quietTime = 0.0f;
            float islandQuietTime = 0.0f;
        };

        bool sleepingEnabled = false;
        Vec3 sleepGravity{};
        std::vector<SleepState> sleepStates;
        std::vector<Collider> sleepColliders;

        void wakeContacts(RigidBodyHandle body);
        uint32_t sleepRoot(uint32_t index);
        void wakeSleepIslands();
        void prepareSleeping();
        void finishSleeping(float dt);
        StepWorkspace &workspace();
    };

}

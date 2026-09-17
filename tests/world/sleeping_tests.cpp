#include <phys/world/physics_world.hpp>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>

namespace
{
    constexpr float dt = 1.0f / 120.0f;

    bool check(bool condition, const char *message)
    {
        if (!condition)
            std::cerr << message << '\n';
        return condition;
    }

    bool near(phys::Vec3 a, phys::Vec3 b, float tolerance = 1e-6f)
    {
        return std::abs(a.x - b.x) <= tolerance && std::abs(a.y - b.y) <= tolerance
            && std::abs(a.z - b.z) <= tolerance;
    }

    bool sameRotation(phys::Quaternion a, phys::Quaternion b)
    {
        return a.x == b.x && a.y == b.y && a.z == b.z && a.w == b.w;
    }

    struct Object
    {
        phys::RigidBodyHandle body;
        phys::ColliderHandle collider;
    };

    Object box(phys::PhysicsWorld &world, phys::Vec3 position, bool isStatic = false,
               phys::Vec3 size = {1, 1, 1})
    {
        Object object;
        std::string error;
        if (!world.createBox(size.x, size.y, size.z, position, 1, isStatic, 0, 0.6f,
                             object.body, object.collider, error))
            throw std::runtime_error("Sleeping fixture setup failed: " + error);
        return object;
    }

    void advance(phys::PhysicsWorld &world, int steps)
    {
        for (int i = 0; i < steps; ++i)
            world.step(dt);
    }

    struct Scene
    {
        phys::PhysicsWorld world;
        Object floor, lower, upper, other;

        Scene(bool sleeping, phys::BroadPhaseAlgorithm algorithm)
        {
            world.broadPhaseAlgorithm = algorithm;
            world.setSleepingEnabled(sleeping);
            floor = box(world, {0, -0.5f, 0}, true, {10, 1, 10});
            lower = box(world, {0, 0.5f, 0});
            upper = box(world, {0, 1.52f, 0});
            other = box(world, {3, 0.5f, 0});
            advance(world, 360);
        }
    };

    bool testOptInAndStationaryContacts(phys::BroadPhaseAlgorithm algorithm)
    {
        Scene scene(false, algorithm);
        auto &world = scene.world;
        if (!check(!world.isSleepingEnabled() && world.lastStepStats().sleepingBodyCount == 0
                   && world.lastStepStats().awakeBodyCount == 3
                   && world.lastStepStats().solvedContactCount == 3,
                   "Default behavior must keep solving settled bodies"))
            return false;
        world.setSleepingEnabled(true);
        advance(world, 120);
        if (!check(world.lastStepStats().sleepingBodyCount == 3
                   && world.lastStepStats().awakeBodyCount == 0
                   && world.lastStepStats().solvedContactCount == 0
                   && world.contacts().size() == 3,
                   "Settled islands must sleep without disappearing from contacts"))
            return false;
        auto position = world.getBody(scene.upper.body)->getPosition();
        auto rotation = world.getBody(scene.upper.body)->getRotation();
        world.getBody(scene.upper.body)->applyForce({});
        world.getBody(scene.upper.body)->applyLinearImpulse({});
        world.getBody(scene.upper.body)->applyImpulse({}, {1, 0, 0});
        world.getCollider(scene.upper.collider);
        advance(world, 600);
        const auto *body = world.getBody(scene.upper.body);
        if (!check(body->isSleeping() && near(body->getPosition(), position, 0)
                   && sameRotation(body->getRotation(), rotation)
                   && near(body->getLinearVelocity(), {}, 0) && near(body->getAngularVelocity(), {}, 0)
                   && world.contacts().size() == 3 && world.lastStepStats().solvedContactCount == 0,
                   "Sleeping poses must stay fixed; reads and zero impulses must not wake them"))
            return false;
        for (const auto &contact : world.contacts())
            if (!check(contact.pointCount == 4, "Sleeping box contact points must remain visible"))
                return false;
        world.setSleepingEnabled(false);
        if (!check(!body->isSleeping(), "Disabling sleeping must wake bodies immediately"))
            return false;
        world.step(dt);
        return check(world.lastStepStats().sleepingBodyCount == 0
                     && world.lastStepStats().solvedContactCount == 3,
                     "Disabling sleeping must resume solving");
    }

    bool testQuietTimeAndMotion()
    {
        for (float timestep : {1.0f / 60.0f, dt})
        {
            phys::PhysicsWorld world;
            world.gravity = {};
            world.setSleepingEnabled(true);
            auto object = box(world, {});
            int halfSecondSteps = static_cast<int>(std::round(0.5f / timestep));
            for (int i = 0; i < halfSecondSteps; ++i)
            {
                world.step(timestep);
                if (!check(!world.getBody(object.body)->isSleeping(),
                           "Sleeping must wait for half a second of uninterrupted quiet time"))
                    return false;
            }
            world.step(timestep);
            world.step(timestep);
            if (!check(world.getBody(object.body)->isSleeping(), "Quiet time must use simulation seconds"))
                return false;
            world.getBody(object.body)->setLinearVelocity({0.1f, 0, 0});
            advance(world, 120);
            if (!check(!world.getBody(object.body)->isSleeping(), "Moving bodies must not sleep"))
                return false;
            world.getBody(object.body)->setLinearVelocity({});
            world.getBody(object.body)->setAngularVelocity({0, 0.1f, 0});
            advance(world, 120);
            if (!check(!world.getBody(object.body)->isSleeping(), "Spinning bodies must not sleep"))
                return false;
            world.getBody(object.body)->setAngularVelocity({});
            for (int i = 0; i < 120; ++i)
            {
                world.getBody(object.body)->applyForce({0.001f, 0, 0});
                world.step(dt);
            }
            if (!check(!world.getBody(object.body)->isSleeping(),
                       "Repeated external forces must prevent sleep even below the speed threshold"))
                return false;
            world.getBody(object.body)->setLinearVelocity({0.0501f, 0, 0});
            advance(world, 120);
            if (!check(!world.getBody(object.body)->isSleeping(), "Speeds just above the threshold must not sleep"))
                return false;
            world.getBody(object.body)->setLinearVelocity({0.05f, 0, 0});
            world.getBody(object.body)->setAngularVelocity({0, 0.05f, 0});
            advance(world, 120);
            if (!check(world.getBody(object.body)->isSleeping(), "The quiet-speed thresholds are inclusive"))
                return false;
        }
        return true;
    }

    bool testWakeMutatorsAndIslandIsolation(phys::BroadPhaseAlgorithm algorithm)
    {
        const Scene settled(true, algorithm);
        if (!check(settled.world.lastStepStats().sleepingBodyCount == 3, "Wake fixture did not sleep"))
            return false;
        for (int mutation = 0; mutation < 8; ++mutation)
        {
            auto scene = settled;
            auto &world = scene.world;
            auto *body = world.getBody(scene.upper.body);
            switch (mutation)
            {
            case 0: body->applyForce({0.1f, 0, 0}); break;
            case 1: body->applyLinearImpulse({0.001f, 0, 0}); break;
            case 2: body->applyImpulse({0.001f, 0, 0}, {0, 0.5f, 0}); break;
            case 3: body->setLinearVelocity({0.001f, 0, 0}); break;
            case 4: body->setAngularVelocity({0, 0.001f, 0}); break;
            case 5: body->wakeUp(); break;
            case 6: body->applyForce({1e-30f, 0, 0}); break;
            case 7: body->applyImpulse({1e-30f, 0, 0}, {}); break;
            }
            if (!check(!body->isSleeping(), "External mutation must immediately wake its body"))
                return false;
            world.step(dt);
            if (!check(!world.getBody(scene.lower.body)->isSleeping() && !body->isSleeping()
                       && world.getBody(scene.other.body)->isSleeping()
                       && world.lastStepStats().solvedContactCount == 2,
                       "Wake must propagate through dynamic contacts, but not through a static floor"))
                return false;
            advance(world, 20);
            if (!check(!body->isSleeping(), "Wake must reset the entire island quiet timer"))
                return false;
        }
        return true;
    }

    bool sameMotion(const phys::PhysicsWorld &a, const phys::PhysicsWorld &b, phys::RigidBodyHandle handle)
    {
        return near(a.getBody(handle)->getPosition(), b.getBody(handle)->getPosition())
            && near(a.getBody(handle)->getLinearVelocity(), b.getBody(handle)->getLinearVelocity())
            && near(a.getBody(handle)->getAngularVelocity(), b.getBody(handle)->getAngularVelocity());
    }

    bool testCollisionWakeAndWarmStart(phys::BroadPhaseAlgorithm algorithm)
    {
        Scene scene(true, algorithm);
        auto awake = scene.world;
        awake.setSleepingEnabled(false);
        auto projectile = box(scene.world, {0, 2.45f, 0});
        auto controlProjectile = box(awake, {0, 2.45f, 0});
        scene.world.getBody(projectile.body)->setLinearVelocity({0, -2, 0});
        awake.getBody(controlProjectile.body)->setLinearVelocity({0, -2, 0});
        scene.world.step(dt);
        awake.step(dt);
        if (!check(!scene.world.getBody(scene.lower.body)->isSleeping()
                   && !scene.world.getBody(scene.upper.body)->isSleeping()
                   && scene.world.getBody(scene.other.body)->isSleeping()
                   && sameMotion(scene.world, awake, scene.lower.body)
                   && sameMotion(scene.world, awake, scene.upper.body)
                   && sameMotion(scene.world, awake, projectile.body),
                   "Collision wake must preserve gravity and cached impulses on the impact step"))
            return false;

        Scene resting(true, algorithm);
        auto control = resting.world;
        control.setSleepingEnabled(false);
        advance(resting.world, 600);
        resting.world.getBody(resting.upper.body)->applyForce({1, 0, 0});
        control.getBody(resting.upper.body)->applyForce({1, 0, 0});
        resting.world.step(dt);
        control.step(dt);
        return check(sameMotion(resting.world, control, resting.lower.body)
                     && sameMotion(resting.world, control, resting.upper.body),
                     "Sleeping must retain warm-start impulses for subsequent waking");
    }

    bool testSupportChangesAndReuse(phys::BroadPhaseAlgorithm algorithm)
    {
        const Scene settled(true, algorithm);
        for (int change = 0; change < 6; ++change)
        {
            auto scene = settled;
            auto &world = scene.world;
            float oldHeight = world.getBody(scene.upper.body)->getPosition().y;
            switch (change)
            {
            case 0: world.removeBody(scene.floor.body); break;
            case 1: world.removeCollider(scene.floor.collider); break;
            case 2: world.getCollider(scene.floor.collider)->localTransform.position.y = -5; break;
            case 3: world.getCollider(scene.floor.collider)->shape = phys::Sphere{0.1f}; break;
            case 4:
            {
                auto replacement = box(world, {0, -5, 0}, true);
                world.getCollider(scene.floor.collider)->body = replacement.body;
                break;
            }
            case 5:
                world.removeBody(scene.lower.body);
                auto replacement = box(world, {20, 5, 0});
                if (!check(replacement.body.index == scene.lower.body.index
                           && replacement.body.generation != scene.lower.body.generation,
                           "Body slot must be reused with a new generation"))
                    return false;
                break;
            }
            world.step(dt);
            if (!check(!world.getBody(scene.upper.body)->isSleeping(),
                       "Removing or editing a support must wake the supported island"))
                return false;
            if (change == 5 && !check(world.getBody(scene.other.body)->isSleeping(),
                                     "Removing a dynamic body must not wake neighbors through the static floor"))
                return false;
            advance(world, 30);
            if (!check(world.getBody(scene.upper.body)->getPosition().y < oldHeight - 0.1f,
                       "Unsupported bodies must fall instead of remaining asleep"))
                return false;
        }
        auto scene = settled;
        scene.world.removeCollider(scene.upper.collider);
        phys::Collider collider;
        collider.body = scene.upper.body;
        collider.shape = phys::Sphere{0.5f};
        auto replacement = scene.world.addCollider(collider);
        if (!check(replacement.index == scene.upper.collider.index
                   && replacement.generation != scene.upper.collider.generation,
                   "Collider slot must be reused with a new generation"))
            return false;
        scene.world.removeCollider(scene.upper.collider);
        scene.world.step(dt);
        return check(scene.world.getCollider(replacement)
                     && !scene.world.getBody(scene.lower.body)->isSleeping(),
                     "Collider reuse must preserve wake propagation and reject stale removal");
    }

    bool testNewStaticContactAndAtomicSleep(phys::BroadPhaseAlgorithm algorithm)
    {
        Scene scene(true, algorithm);
        box(scene.world, {0, 2.45f, 0}, true);
        scene.world.step(dt);
        if (!check(!scene.world.getBody(scene.lower.body)->isSleeping()
                   && !scene.world.getBody(scene.upper.body)->isSleeping()
                   && scene.world.getBody(scene.other.body)->isSleeping(),
                   "A newly added static contact must wake only the affected dynamic island"))
            return false;

        Scene moving(false, algorithm);
        moving.world.setSleepingEnabled(true);
        for (int step = 0; step < 180; ++step)
        {
            moving.world.getBody(moving.upper.body)->applyForce({0.001f, 0, 0});
            moving.world.step(dt);
            if (!check(!moving.world.getBody(moving.lower.body)->isSleeping()
                       && !moving.world.getBody(moving.upper.body)->isSleeping(),
                       "An island must not partially sleep while a connected body is being driven"))
                return false;
        }
        return check(moving.world.getBody(moving.other.body)->isSleeping(),
                     "An independent island must still be allowed to sleep");
    }

    bool testPropertyChangesAndCopy(phys::BroadPhaseAlgorithm algorithm)
    {
        const Scene settled(true, algorithm);
        for (int change = 0; change < 5; ++change)
        {
            auto scene = settled;
            auto &world = scene.world;
            auto *body = world.getBody(scene.lower.body);
            switch (change)
            {
            case 0: body->mass *= 2; break;
            case 1: body->friction = 0; break;
            case 2: body->restitution = 1; break;
            case 3: body->isStatic = true; break;
            case 4: world.gravity = {}; break;
            }
            world.step(dt);
            if (!check(!world.getBody(scene.upper.body)->isSleeping(),
                       "Body properties and gravity changes must wake affected islands"))
                return false;
        }
        auto copied = settled.world;
        auto moved = std::move(copied);
        moved.broadPhaseAlgorithm = algorithm == phys::BroadPhaseAlgorithm::DynamicTree
            ? phys::BroadPhaseAlgorithm::SweepAndPrune : phys::BroadPhaseAlgorithm::DynamicTree;
        moved.step(dt);
        if (!check(moved.lastStepStats().sleepingBodyCount == 3
                   && moved.lastStepStats().solvedContactCount == 0,
                   "Backend switching must retain sleeping bodies and contacts"))
            return false;
        moved.getBody(settled.upper.body)->wakeUp();
        moved.step(dt);
        return check(!moved.getBody(settled.lower.body)->isSleeping()
                     && settled.world.getBody(settled.lower.body)->isSleeping(),
                     "Copied and moved worlds must own independent sleeping state");
    }
}

int main()
{
    if (!testQuietTimeAndMotion())
        return 1;
    for (auto algorithm : {phys::BroadPhaseAlgorithm::SweepAndPrune,
                           phys::BroadPhaseAlgorithm::UniformGrid,
                           phys::BroadPhaseAlgorithm::DynamicTree})
    {
        if (!testOptInAndStationaryContacts(algorithm)
            || !testWakeMutatorsAndIslandIsolation(algorithm)
            || !testCollisionWakeAndWarmStart(algorithm)
            || !testSupportChangesAndReuse(algorithm)
            || !testNewStaticContactAndAtomicSleep(algorithm)
            || !testPropertyChangesAndCopy(algorithm))
            return 1;
    }
    return 0;
}

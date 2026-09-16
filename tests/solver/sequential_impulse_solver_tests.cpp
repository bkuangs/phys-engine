#include <phys/solver/sequential_impulse_solver.hpp>
#include <phys/world/physics_world.hpp>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace
{

    constexpr float tolerance = 1e-5f;

    bool near(float actual, float expected)
    {
        return std::abs(actual - expected) <= tolerance;
    }

    bool testOffCenterContactProducesAngularVelocity()
    {
        phys::RigidBody staticBody;
        phys::RigidBody dynamicBody;
        std::string error;

        if (!phys::RigidBody::createBox(2.0f, 2.0f, 2.0f,
                                        {0.0f, 0.0f, 0.0f}, 1.0f, true, 0.0f, 0.0f, staticBody, error) ||
            !phys::RigidBody::createBox(2.0f, 2.0f, 2.0f,
                                        {0.0f, 0.0f, 0.0f}, 1.0f, false, 0.0f, 0.0f, dynamicBody, error))
        {
            std::cerr << "body creation failed: " << error << '\n';
            return false;
        }

        dynamicBody.setLinearVelocity({0.0f, -1.0f, 0.0f});

        phys::PhysicsWorld world;
        phys::RigidBodyHandle bodyA = world.addBody(staticBody);
        phys::RigidBodyHandle bodyB = world.addBody(dynamicBody);

        phys::ContactManifold manifold{};
        manifold.bodyA = bodyA;
        manifold.bodyB = bodyB;
        manifold.normal = {0.0f, 1.0f, 0.0f};
        manifold.pointCount = 1;
        manifold.points[0].localAnchorA = {1.0f, 0.0f, 0.0f};
        manifold.points[0].localAnchorB = {1.0f, 0.0f, 0.0f};

        std::vector<phys::ContactManifold> contacts{manifold};
        phys::SequentialImpulseSolver::solve(contacts, world, 1.0f / 60.0f);

        const phys::RigidBody *solvedBody = world.getBody(bodyB);
        if (!solvedBody)
        {
            std::cerr << "dynamic body was not found after solve\n";
            return false;
        }

        phys::Vec3 linearVelocity = solvedBody->getLinearVelocity();
        phys::Vec3 angularVelocity = solvedBody->getAngularVelocity();
        if (!near(linearVelocity.y, -0.6f) || !near(angularVelocity.z, 0.6f))
        {
            std::cerr << "expected linear y=-0.6 and angular z=0.6, got linear y="
                      << linearVelocity.y << " and angular z=" << angularVelocity.z << '\n';
            return false;
        }

        return true;
    }

    bool testRestitutionUsesIncomingVelocity()
    {
        phys::RigidBody staticBody;
        phys::RigidBody dynamicBody;
        std::string error;

        if (!phys::RigidBody::createBox(2.0f, 2.0f, 2.0f,
                                        {0.0f, 0.0f, 0.0f}, 1.0f, true, 0.0f, 0.0f, staticBody, error) ||
            !phys::RigidBody::createBox(2.0f, 2.0f, 2.0f,
                                        {0.0f, 0.0f, 0.0f}, 1.0f, false, 0.5f, 0.0f, dynamicBody, error))
        {
            std::cerr << "body creation failed: " << error << '\n';
            return false;
        }

        dynamicBody.setLinearVelocity({0.0f, -2.0f, 0.0f});

        phys::PhysicsWorld world;
        phys::RigidBodyHandle bodyA = world.addBody(staticBody);
        phys::RigidBodyHandle bodyB = world.addBody(dynamicBody);

        phys::ContactManifold manifold{};
        manifold.bodyA = bodyA;
        manifold.bodyB = bodyB;
        manifold.normal = {0.0f, 1.0f, 0.0f};
        manifold.pointCount = 1;
        manifold.restitution = 0.5f;

        std::vector<phys::ContactManifold> contacts{manifold};
        phys::SequentialImpulseSolver::solve(contacts, world, 1.0f / 60.0f);

        const phys::RigidBody *solvedBody = world.getBody(bodyB);
        if (!solvedBody || !near(solvedBody->getLinearVelocity().y, 1.0f))
        {
            std::cerr << "expected restitution velocity y=1, got y="
                      << (solvedBody ? solvedBody->getLinearVelocity().y : 0.0f) << '\n';
            return false;
        }

        return true;
    }

    bool testFrictionConstrainsTangentialVelocity()
    {
        phys::RigidBody staticBody;
        phys::RigidBody dynamicBody;
        std::string error;

        if (!phys::RigidBody::createBox(2.0f, 2.0f, 2.0f,
                                        {0.0f, 0.0f, 0.0f}, 1.0f, true, 0.0f, 0.0f, staticBody, error) ||
            !phys::RigidBody::createBox(2.0f, 2.0f, 2.0f,
                                        {0.0f, 0.0f, 0.0f}, 1.0f, false, 0.0f, 0.0f, dynamicBody, error))
        {
            std::cerr << "body creation failed: " << error << '\n';
            return false;
        }

        dynamicBody.setLinearVelocity({1.0f, -1.0f, 0.0f});

        phys::PhysicsWorld world;
        phys::RigidBodyHandle bodyA = world.addBody(staticBody);
        phys::RigidBodyHandle bodyB = world.addBody(dynamicBody);

        phys::ContactManifold manifold{};
        manifold.bodyA = bodyA;
        manifold.bodyB = bodyB;
        manifold.normal = {0.0f, 1.0f, 0.0f};
        manifold.pointCount = 1;
        manifold.friction = 0.5f;

        std::vector<phys::ContactManifold> contacts{manifold};
        phys::SequentialImpulseSolver::solve(contacts, world, 1.0f / 60.0f);

        const phys::RigidBody *solvedBody = world.getBody(bodyB);
        if (!solvedBody || !near(solvedBody->getLinearVelocity().y, 0.0f) || !near(solvedBody->getLinearVelocity().x, 0.5f))
        {
            std::cerr << "expected velocity x=0.5, y=0, got x="
                      << (solvedBody ? solvedBody->getLinearVelocity().x : 0.0f)
                      << " and y="
                      << (solvedBody ? solvedBody->getLinearVelocity().y : 0.0f)
                      << '\n';
            return false;
        }

        return true;
    }

    bool testWorldCombinesMaterialFriction()
    {
        phys::PhysicsWorld world;
        world.gravity = phys::Vec3::zero();
        phys::RigidBodyHandle bodyA;
        phys::RigidBodyHandle bodyB;
        phys::ColliderHandle colliderA;
        phys::ColliderHandle colliderB;
        std::string error;

        if (!world.createBox(2.0f, 2.0f, 2.0f, {}, 1.0f, true,
                             0.0f, 0.25f, bodyA, colliderA, error) ||
            !world.createBox(2.0f, 2.0f, 2.0f, {}, 1.0f, false,
                             0.0f, 1.0f, bodyB, colliderB, error))
        {
            std::cerr << "world body creation failed: " << error << '\n';
            return false;
        }

        world.step(1.0f / 60.0f);
        const auto &contacts = world.contacts();
        if (contacts.empty() || !near(contacts.front().friction, 0.5f))
        {
            std::cerr << "expected combined friction 0.5, got "
                      << (contacts.empty() ? 0.0f : contacts.front().friction)
                      << '\n';
            return false;
        }

        return true;
    }

    bool testBoxLandsOnStaticFloor()
    {
        phys::PhysicsWorld world;
        world.gravity = {0.0f, -9.81f, 0.0f};
        phys::RigidBodyHandle floorBody;
        phys::RigidBodyHandle boxBody;
        phys::ColliderHandle floorCollider;
        phys::ColliderHandle boxCollider;
        std::string error;

        if (!world.createBox(18.0f, 1.0f, 8.0f, {0.0f, -0.5f, 0.0f},
                             1.0f, true, 0.1f, 0.6f,
                             floorBody, floorCollider, error) ||
            !world.createBox(1.4f, 1.4f, 1.4f, {-1.5f, 3.0f, 0.0f},
                             1.0f, false, 0.2f, 0.5f,
                             boxBody, boxCollider, error))
        {
            std::cerr << "floor test body creation failed: " << error << '\n';
            return false;
        }

        for (int step = 0; step < 180; ++step)
            world.step(1.0f / 60.0f);

        const phys::RigidBody *body = world.getBody(boxBody);
        if (!body || body->getPosition().y < 0.19f || std::abs(body->getPosition().x + 1.5f) > 0.02f || std::abs(body->getLinearVelocity().x) > 0.02f || std::abs(body->getAngularVelocity().z) > 0.02f || world.contacts().empty() || world.contacts().front().pointCount < 2)
        {
            std::cerr << "box fell through floor: y="
                      << (body ? body->getPosition().y : 0.0f)
                      << ", contacts=" << world.contacts().size() << '\n';
            return false;
        }

        return true;
    }

}

int main()
{
    return testOffCenterContactProducesAngularVelocity() && testRestitutionUsesIncomingVelocity() && testFrictionConstrainsTangentialVelocity() && testWorldCombinesMaterialFriction() && testBoxLandsOnStaticFloor() ? 0 : 1;
}

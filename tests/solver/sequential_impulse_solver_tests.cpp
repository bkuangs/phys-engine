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
                                        {0.0f, 0.0f, 0.0f}, 1.0f, true, 0.0f, staticBody, error) ||
            !phys::RigidBody::createBox(2.0f, 2.0f, 2.0f,
                                        {0.0f, 0.0f, 0.0f}, 1.0f, false, 0.0f, dynamicBody, error))
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
                                        {0.0f, 0.0f, 0.0f}, 1.0f, true, 0.0f, staticBody, error) ||
            !phys::RigidBody::createBox(2.0f, 2.0f, 2.0f,
                                        {0.0f, 0.0f, 0.0f}, 1.0f, false, 0.5f, dynamicBody, error))
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

}

int main()
{
    return testOffCenterContactProducesAngularVelocity() && testRestitutionUsesIncomingVelocity() ? 0 : 1;
}

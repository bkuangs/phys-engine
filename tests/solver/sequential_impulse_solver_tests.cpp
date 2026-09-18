#include <phys/solver/sequential_impulse_solver.hpp>
#include <phys/world/physics_world.hpp>
#include <algorithm>
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

    bool near(phys::Vec3 actual, phys::Vec3 expected)
    {
        return near(actual.x, expected.x) && near(actual.y, expected.y) && near(actual.z, expected.z);
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

    bool testFrictionUsesCircularLimit()
    {
        phys::RigidBody staticBody;
        phys::RigidBody dynamicBody;
        std::string error;

        if (!phys::RigidBody::createBox(2.0f, 2.0f, 2.0f,
                                        {}, 1.0f, true, 0.0f, 0.0f, staticBody, error)
            || !phys::RigidBody::createBox(2.0f, 2.0f, 2.0f,
                                           {}, 1.0f, false, 0.0f, 0.0f, dynamicBody, error))
        {
            std::cerr << "body creation failed: " << error << '\n';
            return false;
        }

        dynamicBody.setLinearVelocity({1.0f, -1.0f, 1.0f});

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
        constexpr float expectedTangentVelocity = 0.6464466f;
        float tangentImpulseMagnitude = std::sqrt(
            contacts[0].points[0].tangentImpulse1 * contacts[0].points[0].tangentImpulse1
            + contacts[0].points[0].tangentImpulse2 * contacts[0].points[0].tangentImpulse2);
        float tangentLimit = manifold.friction * contacts[0].points[0].normalImpulse;
        if (!solvedBody || !near(solvedBody->getLinearVelocity().y, 0.0f)
            || !near(solvedBody->getLinearVelocity().x, expectedTangentVelocity)
            || !near(solvedBody->getLinearVelocity().z, expectedTangentVelocity)
            || !near(tangentImpulseMagnitude, tangentLimit))
        {
            phys::Vec3 velocity = solvedBody
                ? solvedBody->getLinearVelocity()
                : phys::Vec3{};
            std::cerr << "friction impulse did not use the circular Coulomb limit: velocity "
                      << velocity.x << ", " << velocity.y << ", " << velocity.z
                      << "; tangent impulse " << tangentImpulseMagnitude << '\n';
            return false;
        }
        return true;
    }

    bool testCoupledFrictionAtOffset()
    {
        phys::RigidBody staticBody;
        phys::RigidBody dynamicBody;
        std::string error;
        if (!phys::RigidBody::createBox(2.0f, 2.0f, 2.0f,
                                        {}, 1.0f, true, 0.0f, 0.0f, staticBody, error)
            || !phys::RigidBody::createBox(2.0f, 2.0f, 2.0f,
                                           {}, 1.0f, false, 0.0f, 0.0f, dynamicBody, error))
        {
            std::cerr << "body creation failed: " << error << '\n';
            return false;
        }
        dynamicBody.setLinearVelocity({1.0f, -1.0f, 1.0f});

        phys::PhysicsWorld world;
        phys::RigidBodyHandle bodyA = world.addBody(staticBody);
        phys::RigidBodyHandle bodyB = world.addBody(dynamicBody);
        phys::ContactManifold manifold{};
        manifold.bodyA = bodyA;
        manifold.bodyB = bodyB;
        manifold.normal = {0.0f, 1.0f, 0.0f};
        manifold.pointCount = 1;
        manifold.points[0].localAnchorA = {0.5f, 1.0f, 0.25f};
        manifold.points[0].localAnchorB = manifold.points[0].localAnchorA;
        manifold.friction = 10.0f;

        std::vector<phys::ContactManifold> contacts{manifold};
        phys::SequentialImpulseSolver::solve(contacts, world, 1.0f / 60.0f);

        const phys::RigidBody *solvedBody = world.getBody(bodyB);
        if (!solvedBody)
            return false;
        phys::Vec3 offset = solvedBody->getRotation().rotate(
            manifold.points[0].localAnchorB);
        phys::Vec3 contactVelocity = solvedBody->getLinearVelocity()
            + phys::Math3d::cross(solvedBody->getAngularVelocity(), offset);
        if (!near(contactVelocity.x, 0.0f) || !near(contactVelocity.z, 0.0f)
            || phys::Math3d::length(solvedBody->getAngularVelocity()) <= 0.1f)
        {
            std::cerr << "coupled off-center friction did not cancel tangential contact velocity\n";
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
        if (!body || body->getPosition().y < 0.68f || body->getPosition().y > 0.72f || std::abs(body->getPosition().x + 1.5f) > 0.02f || std::abs(body->getLinearVelocity().x) > 0.02f || std::abs(body->getAngularVelocity().z) > 0.02f || world.contacts().empty() || world.contacts().front().pointCount < 2)
        {
            std::cerr << "box fell through floor: y="
                      << (body ? body->getPosition().y : 0.0f)
                      << ", contacts=" << world.contacts().size() << '\n';
            return false;
        }

        return true;
    }

    bool testTiltedBoxSettlesFlat(float dt, float angle, phys::Vec3 axis)
    {
        phys::PhysicsWorld world;
        world.gravity = {0.0f, -9.81f, 0.0f};
        phys::RigidBodyHandle floorBody;
        phys::RigidBodyHandle boxBody;
        phys::ColliderHandle floorCollider;
        phys::ColliderHandle boxCollider;
        std::string error;
        if (!world.createBox(18.0f, 1.0f, 8.0f, {0.0f, -0.5f, 0.0f},
                             1.0f, true, 0.0f, 0.6f,
                             floorBody, floorCollider, error) ||
            !world.createBox(1.0f, 1.0f, 1.0f, {0.0f, 2.0f, 0.0f},
                             1.0f, false, 0.0f, 0.55f,
                             boxBody, boxCollider, error))
        {
            std::cerr << "tilted floor test setup failed: " << error << '\n';
            return false;
        }
        phys::RigidBody *body = world.getBody(boxBody);
        body->setAngularVelocity(axis * (2.0f * std::tan(angle * 0.5f)));
        body->integrateRotation(1.0f);
        body->setAngularVelocity({});

        for (int step = 0; step < 600; ++step)
            world.step(dt);

        float lowestCorner = body->getPosition().y;
        for (float x : {-0.5f, 0.5f})
            for (float y : {-0.5f, 0.5f})
                for (float z : {-0.5f, 0.5f})
                    lowestCorner = std::min(lowestCorner, body->getPosition().y
                        + body->getRotation().rotate({x, y, z}).y);
        float alignment = std::max({
            std::abs(body->getRotation().rotate({1.0f, 0.0f, 0.0f}).y),
            std::abs(body->getRotation().rotate({0.0f, 1.0f, 0.0f}).y),
            std::abs(body->getRotation().rotate({0.0f, 0.0f, 1.0f}).y)});
        if (!(alignment >= 0.995f && lowestCorner >= -0.02f
            && std::abs(body->getPosition().y - 0.5f) <= 0.03f
            && phys::Math3d::length(body->getLinearVelocity()) <= 0.08f
            && phys::Math3d::length(body->getAngularVelocity()) <= 0.1f))
        {
            std::cerr << "tilted box did not settle flat: dt=" << dt
                      << ", alignment=" << alignment
                      << ", lowest corner=" << lowestCorner
                      << ", center height=" << body->getPosition().y << '\n';
            return false;
        }
        return true;
    }

    bool testWarmStartPairIdentityAndStableMatching()
    {
        phys::RigidBody staticBody;
        phys::RigidBody dynamicBody;
        std::string error;
        if (!phys::RigidBody::createBox(1, 1, 1, {}, 1, true, 0, 0, staticBody, error)
            || !phys::RigidBody::createBox(1, 1, 1, {}, 1, false, 0, 0, dynamicBody, error))
        {
            std::cerr << "warm-start test setup failed: " << error << '\n';
            return false;
        }
        phys::PhysicsWorld world;
        auto bodyA = world.addBody(staticBody);
        auto bodyB = world.addBody(dynamicBody);
        auto bodyC = world.addBody(staticBody);
        world.getBody(bodyB)->isStatic = true;

        // Static participants retain the seeded impulses without applying them.
        // Interleaved/reversed pairs and near-duplicate anchors exercise cache matching.
        std::vector<phys::ContactManifold> seed;
        for (int index = 0; index < 64; ++index)
        {
            phys::ContactManifold contact{};
            contact.bodyA = index == 2 ? bodyB : bodyA;
            contact.bodyB = index == 2 ? bodyA : (index % 2 == 0 && index > 3 ? bodyC : bodyB);
            contact.normal = {0, 1, 0};
            contact.pointCount = 1;
            contact.points[0].normalImpulse = index == 3 ? 1.0f : 2.0f;
            if (index == 3)
            {
                contact.points[0].localAnchorA = {0.01f, 0, 0};
                contact.points[0].localAnchorB = {-0.01f, 0, 0};
            }
            if (index == 0)
                contact.points[0].localAnchorA = {1, 0, 0};
            if (index == 1)
                contact.points[0].localAnchorB = {1, 0, 0};
            if (index == 2)
                contact.points[0].normalImpulse = 9.0f;
            seed.push_back(contact);
        }
        phys::SequentialImpulseSolver::solve(seed, world, 1.0f / 60.0f);

        for (int phase = 0; phase < 3; ++phase)
        {
            if (phase != 0)
            {
                auto old = phase == 1 ? bodyB : bodyA;
                world.removeBody(old);
                auto replacement = world.addBody(phase == 1 ? dynamicBody : staticBody);
                if (replacement.index != old.index || replacement.generation == old.generation)
                {
                    std::cerr << "warm-start test did not reuse the requested body slot\n";
                    return false;
                }
                if (phase == 1)
                    bodyB = replacement;
                else
                    bodyA = replacement;
            }
            world.getBody(bodyB)->isStatic = false;
            world.getBody(bodyB)->setLinearVelocity({0, -4, 0});
            phys::ContactManifold contact{};
            contact.bodyA = bodyA;
            contact.bodyB = bodyB;
            contact.normal = {0, 1, 0};
            contact.pointCount = 2;
            std::vector<phys::ContactManifold> contacts{contact};
            phys::SequentialImpulseSolver::solve(contacts, world, 1.0f / 60.0f);
            if (!near(contacts[0].points[0].normalImpulse, phase == 0 ? 3.0f : 4.0f)
                || !near(contacts[0].points[1].normalImpulse, phase == 0 ? 1.0f : 0.0f)
                || !near(world.getBody(bodyB)->getLinearVelocity().y, 0.0f))
            {
                std::cerr << "warm-start matching changed order, anchors, or handle identity in phase "
                          << phase << '\n';
                return false;
            }
        }
        return true;
    }

    bool testPreparedBodyDataRefreshedBetweenSolves()
    {
        phys::RigidBody initialA, initialB;
        std::string error;
        if (!phys::RigidBody::createBox(2, 1, 3, {}, 1, false, 0, 0, initialA, error)
            || !phys::RigidBody::createBox(1, 3, 2, {}, 2, false, 0, 0, initialB, error))
        {
            std::cerr << "prepared-body test setup failed: " << error << '\n';
            return false;
        }
        phys::PhysicsWorld world;
        auto handleA = world.addBody(initialA);
        auto handleB = world.addBody(initialB);
        auto staleB = handleB;
        for (int phase = 0; phase < 5; ++phase)
        {
            std::vector<phys::ContactManifold> empty;
            phys::SequentialImpulseSolver::solve(empty, world, 1.0f / 120.0f);
            if (phase == 3)
            {
                world.removeBody(handleB);
                handleB = world.addBody(initialA);
                if (handleB.index != staleB.index || handleB.generation == staleB.generation)
                {
                    std::cerr << "prepared-body test did not reuse its body slot\n";
                    return false;
                }
            }
            auto *a = world.getBody(handleA);
            auto *b = world.getBody(handleB);
            a->isStatic = phase == 2;
            b->mass += 0.75f * phase;
            a->setAngularVelocity({0.3f, 0.7f, -0.2f});
            b->setAngularVelocity({-0.6f, 0.2f, 0.4f});
            a->integrateRotation(0.4f);
            b->integrateRotation(0.7f);
            a->setLinearVelocity({0.5f, 2, -0.25f});
            b->setLinearVelocity({-0.2f, -3, 0.4f});

            phys::ContactManifold contact{};
            contact.bodyA = handleA;
            contact.bodyB = handleB;
            contact.normal = phys::Math3d::normalize({0.2f, 1, -0.3f});
            contact.pointCount = 1;
            contact.points[0].localAnchorA = {0.4f, 0.2f, -0.3f};
            contact.points[0].localAnchorB = {-0.2f, -0.4f, 0.5f};

            auto expectedA = *a;
            auto expectedB = *b;
            auto offsetA = a->getRotation().rotate(contact.points[0].localAnchorA);
            auto offsetB = b->getRotation().rotate(contact.points[0].localAnchorB);
            auto jacobianA = phys::Math3d::cross(offsetA, contact.normal);
            auto jacobianB = phys::Math3d::cross(offsetB, contact.normal);
            float accumulatedImpulse = 0;
            // Single frictionless constraint, evaluated with the uncached public body math.
            for (int iteration = 0; iteration < 8; ++iteration)
            {
                float denominator = expectedA.getInverseMass() + expectedB.getInverseMass()
                    + phys::Math3d::dot(jacobianA, expectedA.getInverseInertiaWorld() * jacobianA)
                    + phys::Math3d::dot(jacobianB, expectedB.getInverseInertiaWorld() * jacobianB);
                auto velocityA = expectedA.getLinearVelocity()
                    + phys::Math3d::cross(expectedA.getAngularVelocity(), offsetA);
                auto velocityB = expectedB.getLinearVelocity()
                    + phys::Math3d::cross(expectedB.getAngularVelocity(), offsetB);
                float nextImpulse = std::max(0.0f, accumulatedImpulse
                    - phys::Math3d::dot(velocityB - velocityA, contact.normal) / denominator);
                auto impulse = contact.normal * (nextImpulse - accumulatedImpulse);
                expectedA.applyImpulse(-impulse, offsetA);
                expectedB.applyImpulse(impulse, offsetB);
                accumulatedImpulse = nextImpulse;
            }
            std::vector<phys::ContactManifold> contacts;
            if (phase >= 3)
            {
                auto staleContact = contact;
                staleContact.bodyB = staleB;
                contacts.push_back(staleContact);
            }
            contacts.push_back(contact);
            phys::SequentialImpulseSolver::solve(contacts, world, 1.0f / 120.0f);
            if (!near(a->getLinearVelocity(), expectedA.getLinearVelocity())
                || !near(a->getAngularVelocity(), expectedA.getAngularVelocity())
                || !near(b->getLinearVelocity(), expectedB.getLinearVelocity())
                || !near(b->getAngularVelocity(), expectedB.getAngularVelocity())
                || !near(contacts.back().points[0].normalImpulse, accumulatedImpulse)
                || world.lastStepStats().solvedContactCount != 1)
            {
                std::cerr << "prepared mass/inertia or angular response changed in phase " << phase << '\n';
                return false;
            }
        }
        return true;
    }

    bool testParallelIslandsMatchSerial()
    {
        constexpr std::size_t bodiesPerIsland = 65;
        phys::PhysicsWorld serial;
        std::vector<phys::RigidBodyHandle> handles;
        handles.reserve(bodiesPerIsland * 2);
        std::string error;
        for (std::size_t index = 0; index < bodiesPerIsland * 2; ++index)
        {
            phys::RigidBody body;
            if (!phys::RigidBody::createSphere(
                    1.0f, {}, 1.0f, false, 0.0f, 0.6f, body, error))
                return false;
            body.setLinearVelocity({
                static_cast<float>(index % 7) * 0.1f,
                -1.0f - static_cast<float>(index % 5) * 0.05f,
                static_cast<float>(index % 3) * -0.1f});
            handles.push_back(serial.addBody(body));
        }

        std::vector<phys::ContactManifold> serialContacts;
        for (std::size_t island = 0; island < 2; ++island)
            for (std::size_t first = 0; first < bodiesPerIsland; ++first)
                for (std::size_t second = first + 1;
                     second < bodiesPerIsland; ++second)
                {
                    phys::ContactManifold contact{};
                    contact.bodyA = handles[island * bodiesPerIsland + first];
                    contact.bodyB = handles[island * bodiesPerIsland + second];
                    contact.normal = {0.0f, 1.0f, 0.0f};
                    contact.pointCount = 1;
                    contact.points[0].localAnchorA = {0.1f, 0.0f, -0.2f};
                    contact.points[0].localAnchorB = {-0.1f, 0.0f, 0.2f};
                    contact.points[0].penetration = 0.01f;
                    contact.friction = 0.6f;
                    serialContacts.push_back(contact);
                }

        phys::PhysicsWorld parallel = serial;
        std::vector<phys::ContactManifold> parallelContacts = serialContacts;
        parallel.setSolverWorkerCount(4);
        for (int pass = 0; pass < 2; ++pass)
        {
            phys::SequentialImpulseSolver::solve(
                serialContacts, serial, 1.0f / 120.0f);
            phys::SequentialImpulseSolver::solve(
                parallelContacts, parallel, 1.0f / 120.0f);
            for (phys::RigidBodyHandle handle : handles)
            {
                const phys::RigidBody *serialBody = serial.getBody(handle);
                const phys::RigidBody *parallelBody = parallel.getBody(handle);
                if (!serialBody || !parallelBody
                    || serialBody->getLinearVelocity().x != parallelBody->getLinearVelocity().x
                    || serialBody->getLinearVelocity().y != parallelBody->getLinearVelocity().y
                    || serialBody->getLinearVelocity().z != parallelBody->getLinearVelocity().z
                    || serialBody->getAngularVelocity().x != parallelBody->getAngularVelocity().x
                    || serialBody->getAngularVelocity().y != parallelBody->getAngularVelocity().y
                    || serialBody->getAngularVelocity().z != parallelBody->getAngularVelocity().z)
                {
                    std::cerr << "parallel island solve changed body velocity\n";
                    return false;
                }
            }
            for (std::size_t index = 0; index < serialContacts.size(); ++index)
            {
                const auto &serialPoint = serialContacts[index].points[0];
                const auto &parallelPoint = parallelContacts[index].points[0];
                if (serialPoint.normalImpulse != parallelPoint.normalImpulse
                    || serialPoint.tangentImpulse1 != parallelPoint.tangentImpulse1
                    || serialPoint.tangentImpulse2 != parallelPoint.tangentImpulse2)
                {
                    std::cerr << "parallel island solve changed contact impulse\n";
                    return false;
                }
            }
        }
        if (parallel.lastStepStats().solverDetails.islandCount != 2
            || parallel.lastStepStats().solverDetails.largestIslandContacts
                != bodiesPerIsland * (bodiesPerIsland - 1) / 2)
            return false;
        try
        {
            parallel.setSolverWorkerCount(0);
            std::cerr << "zero solver workers were accepted\n";
            return false;
        }
        catch (const std::invalid_argument &) {}
        return parallel.getSolverWorkerCount() == 4;
    }

}

int main()
{
    return testOffCenterContactProducesAngularVelocity() && testRestitutionUsesIncomingVelocity()
        && testFrictionConstrainsTangentialVelocity() && testFrictionUsesCircularLimit()
        && testCoupledFrictionAtOffset()
        && testWorldCombinesMaterialFriction()
        && testBoxLandsOnStaticFloor()
        && testWarmStartPairIdentityAndStableMatching()
        && testPreparedBodyDataRefreshedBetweenSolves()
        && testParallelIslandsMatchSerial()
        && testTiltedBoxSettlesFlat(1.0f / 60.0f, 0.3f, {0.0f, 0.0f, 1.0f})
        && testTiltedBoxSettlesFlat(1.0f / 120.0f, -0.6f, {1.0f, 0.0f, 0.0f})
        && testTiltedBoxSettlesFlat(1.0f / 60.0f, 0.5f,
            phys::Math3d::normalize({1.0f, 1.0f, 1.0f}))
        ? 0 : 1;
}

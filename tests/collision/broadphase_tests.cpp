#include <phys/collision/broadphase.hpp>
#include <phys/collision/narrowphase.hpp>
#include <phys/world/physics_world.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>

namespace {

bool matchesBruteForce(const std::vector<phys::Aabb>& bounds)
{
    std::vector<phys::BroadPhasePair> expected;
    for (std::size_t first = 0; first < bounds.size(); ++first)
        for (std::size_t second = first + 1; second < bounds.size(); ++second)
            if (bounds[first].overlaps(bounds[second]))
                expected.push_back({first, second});

    const auto actual = phys::BroadPhase::findCandidatePairs(bounds);
    if (actual.size() != expected.size()) {
        std::cerr << "broadphase pair count: expected " << expected.size()
                  << ", got " << actual.size() << '\n';
        return false;
    }
    for (std::size_t index = 0; index < expected.size(); ++index) {
        if (actual[index].first != expected[index].first
            || actual[index].second != expected[index].second) {
            std::cerr << "broadphase pair or ordering differs at " << index << '\n';
            return false;
        }
    }
    return true;
}

bool testBoundaryCases()
{
    return matchesBruteForce({})
        && matchesBruteForce({{{0, 0, 0}, {0, 0, 0}}})
        && matchesBruteForce(std::vector<phys::Aabb>(16, {{-1, -1, -1}, {1, 1, 1}}))
        && matchesBruteForce({
            {{4, 0, 0}, {5, 1, 1}},
            {{-2, -2, -2}, {0, 0, 0}},
            {{0, 0, 0}, {1, 1, 1}},
            {{1, 0, 0}, {2, 1, 1}},
            {{1, 1, 0}, {2, 2, 1}},
            {{1, 1, 1}, {2, 2, 2}},
            {{0, 4, 0}, {1, 5, 1}},
            {{0, 0, 4}, {1, 1, 5}},
            {{1, 1, 1}, {1, 1, 1}},
            {{-100, -0.5f, -100}, {100, 0, 100}}});
}

bool testRandomAndMovingBounds()
{
    std::mt19937 rng(42);
    std::uniform_real_distribution<float> position(-20.0f, 20.0f);
    std::uniform_real_distribution<float> extent(0.01f, 3.0f);
    for (int scene = 0; scene < 30; ++scene) {
        std::vector<phys::Aabb> bounds;
        for (int index = 0; index < 192; ++index) {
            phys::Vec3 center{position(rng), position(rng), position(rng)};
            if (scene % 3 == 0)
                center = center * 0.1f;
            phys::Vec3 halfExtents{extent(rng), extent(rng), extent(rng)};
            phys::Aabb box{center - halfExtents, center + halfExtents};
            if (scene % 3 == 1) {
                box.min.x = -1.0f;
                box.max.x = 1.0f;
            }
            bounds.push_back(box);
        }
        for (int step = 0; step < 4; ++step) {
            if (!matchesBruteForce(bounds))
                return false;
            for (auto& box : bounds) {
                phys::Vec3 motion{position(rng) * 0.05f, position(rng) * 0.05f,
                                  position(rng) * 0.05f};
                box.min += motion;
                box.max += motion;
            }
        }
    }
    return true;
}

bool worldMatchesBruteForce(phys::PhysicsWorld& world,
    const std::vector<phys::ColliderHandle>& handles)
{
    std::vector<phys::ColliderHandle> active;
    for (auto handle : handles)
        if (world.getCollider(handle))
            active.push_back(handle);
    std::sort(active.begin(), active.end(), [](auto left, auto right) {
        return left.index < right.index;
    });

    std::vector<phys::Aabb> bounds;
    std::vector<phys::Transform> transforms;
    for (auto handle : active) {
        const auto& collider = *world.getCollider(handle);
        const auto& body = *world.getBody(collider.body);
        phys::Transform bodyTransform{body.getPosition(), body.getRotation()};
        bounds.push_back(phys::Aabb::fromCollider(collider, bodyTransform));
        transforms.push_back({
            phys::transform(collider.localTransform.position, bodyTransform),
            bodyTransform.orientation * collider.localTransform.orientation});
    }

    std::size_t possiblePairs = 0;
    std::size_t candidatePairs = 0;
    std::vector<phys::ContactManifold> expectedContacts;
    for (std::size_t first = 0; first < active.size(); ++first) {
        for (std::size_t second = first + 1; second < active.size(); ++second) {
            const auto& a = *world.getCollider(active[first]);
            const auto& b = *world.getCollider(active[second]);
            if (a.body == b.body)
                continue;
            ++possiblePairs;
            if (!bounds[first].overlaps(bounds[second]))
                continue;
            ++candidatePairs;
            phys::ContactManifold manifold;
            if (phys::NarrowPhase::generateContact(
                    a, transforms[first], b, transforms[second], manifold))
                expectedContacts.push_back(manifold);
        }
    }

    world.step(1.0f / 60.0f);
    const auto& stats = world.lastStepStats();
    const auto& contacts = world.contacts();
    if (stats.possiblePairs != possiblePairs || stats.candidatePairs != candidatePairs
        || stats.contactCount != expectedContacts.size()
        || contacts.size() != expectedContacts.size()) {
        std::cerr << "world broadphase counts differ from brute force\n";
        return false;
    }
    for (std::size_t index = 0; index < contacts.size(); ++index) {
        const auto& actual = contacts[index];
        const auto& expected = expectedContacts[index];
        if (!(actual.bodyA == expected.bodyA && actual.bodyB == expected.bodyB)
            || actual.pointCount != expected.pointCount
            || phys::Math3d::length(actual.normal - expected.normal) > 1e-5f) {
            std::cerr << "world contact geometry or ordering differs from brute force\n";
            return false;
        }
    }
    return true;
}

bool testWorldFilteringAndSlotReuse()
{
    phys::PhysicsWorld world;
    world.gravity = {};
    std::vector<phys::ColliderHandle> handles;
    std::string error;
    auto addSphere = [&](phys::Vec3 position) {
        phys::RigidBodyHandle body;
        phys::ColliderHandle collider;
        if (!world.createSphere(1.0f, position, 1.0f, true, 0.0f, 0.0f,
                                body, collider, error)) {
            std::cerr << "world test setup failed: " << error << '\n';
            return false;
        }
        handles.push_back(collider);
        return true;
    };

    if (!worldMatchesBruteForce(world, handles)
        || !addSphere({1.5f, 0, 0}) || !worldMatchesBruteForce(world, handles)
        || !addSphere({0, 0, 0}) || !addSphere({0.75f, 0, 0})
        || !worldMatchesBruteForce(world, handles))
        return false;

    world.addBody(phys::RigidBody{});
    phys::Collider extra = *world.getCollider(handles[0]);
    extra.shape = phys::Box{{0.5f, 0.5f, 0.5f}};
    extra.localTransform = {{-1.0f, 0.1f, 0.0f}, 0.4f};
    handles.push_back(world.addCollider(extra));
    if (!worldMatchesBruteForce(world, handles))
        return false;

    world.removeCollider(handles[1]);
    if (!worldMatchesBruteForce(world, handles) || !addSphere({-0.5f, 0, 0}))
        return false;
    if (handles.back().index != handles[1].index
        || handles.back().generation == handles[1].generation) {
        std::cerr << "test did not reuse the removed collider slot\n";
        return false;
    }
    if (!worldMatchesBruteForce(world, handles))
        return false;

    world.removeBody(extra.body);
    if (!worldMatchesBruteForce(world, handles) || !addSphere({1, 0, 0})
        || !worldMatchesBruteForce(world, handles))
        return false;
    if (world.getCollider(handles.back())->body.generation == extra.body.generation) {
        std::cerr << "test did not reuse the removed body slot\n";
        return false;
    }
    world.getCollider(handles.back())->localTransform.position = {30, 0, 0};
    if (!worldMatchesBruteForce(world, handles))
        return false;

    for (auto handle : handles)
        world.removeCollider(handle);
    return worldMatchesBruteForce(world, handles);
}

}

int main()
{
    return testBoundaryCases() && testRandomAndMovingBounds()
        && testWorldFilteringAndSlotReuse() ? 0 : 1;
}

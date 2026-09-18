#include <phys/collision/broadphase.hpp>
#include <phys/collision/dynamic_aabb_tree.hpp>
#include <phys/collision/narrowphase.hpp>
#include <phys/world/physics_world.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include <vector>

namespace {

bool matchesBruteForce(const std::vector<phys::Aabb>& bounds,
                       phys::BroadPhaseAlgorithm algorithm)
{
    std::vector<phys::BroadPhasePair> expected;
    std::size_t expectedXComparisons = 0;
    for (std::size_t first = 0; first < bounds.size(); ++first) {
        for (std::size_t second = first + 1; second < bounds.size(); ++second) {
            if (bounds[first].min.x <= bounds[second].max.x
                && bounds[first].max.x >= bounds[second].min.x)
                ++expectedXComparisons;
            if (bounds[first].overlaps(bounds[second]))
                expected.push_back({first, second});
        }
    }

    phys::BroadPhaseStats stats{-1.0, -1.0, -1.0, -1.0, 1, 1, 1, 1, 1, -1.0};
    bool dynamic = algorithm == phys::BroadPhaseAlgorithm::DynamicTree;
    phys::DynamicAabbTree tree;
    if (dynamic)
        for (std::size_t index = 0; index < bounds.size(); ++index)
            tree.createProxy(bounds[index], index);
    const auto actual = dynamic ? tree.findCandidatePairs(&stats)
        : phys::BroadPhase::findCandidatePairs(bounds, &stats, algorithm);
    const auto withoutStats = dynamic ? tree.findCandidatePairs()
        : phys::BroadPhase::findCandidatePairs(bounds, nullptr, algorithm);
    bool grid = algorithm == phys::BroadPhaseAlgorithm::UniformGrid;
    if (stats.xWindowComparisons != (grid || dynamic ? 0 : expectedXComparisons)
        || stats.aabbPairs != expected.size() || withoutStats.size() != actual.size()) {
        std::cerr << "broadphase work counters or optional stats behavior differ\n";
        return false;
    }
    if (dynamic && (stats.treeProxyCount != bounds.size() || stats.treeLeafChecks < expected.size())) {
        std::cerr << "invalid tree work counters\n";
        return false;
    }
    if (grid ? (stats.gridComparisons < expected.size()
                || stats.gridEntries > bounds.size() * 64
                || stats.gridOverflowAabbs > bounds.size()
                || !(stats.gridCellSize > 0.0 && std::isfinite(stats.gridCellSize)))
             : (stats.gridComparisons != 0 || stats.gridEntries != 0
                || stats.gridOverflowAabbs != 0 || stats.gridCellSize != 0.0)) {
        std::cerr << "invalid grid work counters\n";
        return false;
    }
    for (double time : {stats.recordBuildMs, stats.recordSortMs, stats.sweepMs, stats.pairSortMs}) {
        if (!std::isfinite(time) || time < 0.0) {
            std::cerr << "invalid broadphase phase timing\n";
            return false;
        }
    }
    if (actual.size() != expected.size()) {
        std::cerr << "broadphase pair count: expected " << expected.size()
                  << ", got " << actual.size() << '\n';
        return false;
    }
    for (std::size_t index = 0; index < expected.size(); ++index) {
        if (actual[index].first != expected[index].first
            || actual[index].second != expected[index].second
            || actual[index].first != withoutStats[index].first
            || actual[index].second != withoutStats[index].second) {
            std::cerr << "broadphase pair or ordering differs at " << index << '\n';
            return false;
        }
    }
    return true;
}

bool matchesAll(const std::vector<phys::Aabb>& bounds)
{
    return matchesBruteForce(bounds, phys::BroadPhaseAlgorithm::SweepAndPrune)
        && matchesBruteForce(bounds, phys::BroadPhaseAlgorithm::UniformGrid)
        && matchesBruteForce(bounds, phys::BroadPhaseAlgorithm::DynamicTree);
}

bool testBoundaryCases()
{
    return matchesAll({})
        && matchesAll({{{0, 0, 0}, {0, 0, 0}}})
        && matchesAll(std::vector<phys::Aabb>(16, {{-1, -1, -1}, {1, 1, 1}}))
        && matchesAll({
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
            if (!matchesAll(bounds))
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

bool testGridBoundariesAndOverflow()
{
    std::vector<phys::Aabb> identical(2, {{-0.5f, -0.5f, -0.5f}, {0.5f, 0.5f, 0.5f}});
    phys::BroadPhaseStats stats;
    auto pairs = phys::BroadPhase::findCandidatePairs(
        identical, &stats, phys::BroadPhaseAlgorithm::UniformGrid);
    if (pairs.size() != 1 || stats.gridEntries != 16 || stats.gridComparisons != 8
        || stats.gridOverflowAabbs != 0 || stats.gridCellSize != 1.0) {
        std::cerr << "grid did not deduplicate a pair shared by eight cells\n";
        return false;
    }

    std::vector<phys::Aabb> atLimit(3, {{-0.5f, -0.5f, -0.5f}, {0.5f, 0.5f, 0.5f}});
    atLimit.push_back({{0, 0, 0}, {3, 3, 3}});
    atLimit.push_back({{0, 0, 0}, {4, 3, 3}});
    phys::BroadPhase::findCandidatePairs(
        atLimit, &stats, phys::BroadPhaseAlgorithm::UniformGrid);
    if (stats.gridOverflowAabbs != 1 || stats.gridEntries != 88 || !matchesAll(atLimit)) {
        std::cerr << "grid did not enforce the exact 64-cell membership boundary\n";
        return false;
    }

    constexpr float largest = std::numeric_limits<float>::max();
    std::vector<phys::Aabb> bounds{
        {{-0.5f, -0.5f, -0.5f}, {0.5f, 0.5f, 0.5f}},
        {{0.5f, -0.5f, -0.5f}, {1.5f, 0.5f, 0.5f}},
        {{-1.5f, -0.5f, -0.5f}, {-0.5f, 0.5f, 0.5f}},
        {{-100000, -0.5f, -100000}, {100000, 0, 100000}},
        {{largest, 0, 0}, {largest, 0, 0}},
        {{largest, 0, 0}, {largest, 0, 0}},
        {{-2147483648.0f, 0, 0}, {-2147483648.0f, 0, 0}},
        {{-2147483648.0f, 0, 0}, {-2147483648.0f, 0, 0}},
    };
    pairs = phys::BroadPhase::findCandidatePairs(
        bounds, &stats, phys::BroadPhaseAlgorithm::UniformGrid);
    if (stats.gridOverflowAabbs != 3) {
        std::cerr << "large floor and extreme coordinates did not use the overflow path\n";
        return false;
    }
    return matchesAll(bounds)
        && matchesAll({{{-1, -1, -1}, {1, 1, 1}},
                       {{-largest, -largest, -largest}, {largest, largest, largest}}});
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
    std::size_t xComparisons = 0;
    std::size_t aabbPairs = 0;
    std::vector<phys::ContactManifold> expectedContacts;
    for (std::size_t first = 0; first < active.size(); ++first) {
        for (std::size_t second = first + 1; second < active.size(); ++second) {
            const auto& a = *world.getCollider(active[first]);
            const auto& b = *world.getCollider(active[second]);
            if (bounds[first].min.x <= bounds[second].max.x
                && bounds[first].max.x >= bounds[second].min.x)
                ++xComparisons;
            if (bounds[first].overlaps(bounds[second]))
                ++aabbPairs;
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
        || stats.broadPhaseDetails.xWindowComparisons !=
            (world.broadPhaseAlgorithm == phys::BroadPhaseAlgorithm::SweepAndPrune ? xComparisons : 0)
        || stats.broadPhaseDetails.aabbPairs != aabbPairs
        || stats.contactCount != expectedContacts.size()
        || contacts.size() != expectedContacts.size()) {
        std::cerr << "world broadphase counts differ from brute force\n";
        return false;
    }
    if (world.broadPhaseAlgorithm == phys::BroadPhaseAlgorithm::DynamicTree
        && stats.broadPhaseDetails.treeProxyCount != active.size()) {
        std::cerr << "tree retained stale collider proxies\n";
        return false;
    }
    double measuredPhases = 0.0;
    for (double time : {stats.broadPhaseCollectMs, stats.broadPhaseFilterMs,
             stats.broadPhaseDetails.recordBuildMs, stats.broadPhaseDetails.recordSortMs,
             stats.broadPhaseDetails.sweepMs, stats.broadPhaseDetails.pairSortMs}) {
        if (!std::isfinite(time) || time < 0.0) {
            std::cerr << "invalid world broadphase phase timing\n";
            return false;
        }
        measuredPhases += time;
    }
    if (!(measuredPhases <= stats.broadPhaseMs + 1e-6)) {
        std::cerr << "broadphase phase timings exceed total broadphase time\n";
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

bool testWorldFilteringAndSlotReuse(phys::BroadPhaseAlgorithm algorithm)
{
    phys::PhysicsWorld world;
    if (world.broadPhaseAlgorithm != phys::BroadPhaseAlgorithm::SweepAndPrune) {
        std::cerr << "the default broadphase changed\n";
        return false;
    }
    world.broadPhaseAlgorithm = algorithm;
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

    if (algorithm == phys::BroadPhaseAlgorithm::DynamicTree)
        world.broadPhaseAlgorithm = phys::BroadPhaseAlgorithm::SweepAndPrune;
    world.removeBody(extra.body);
    if (!worldMatchesBruteForce(world, handles) || !addSphere({1, 0, 0})
        || !worldMatchesBruteForce(world, handles))
        return false;
    if (world.getCollider(handles.back())->body.generation == extra.body.generation) {
        std::cerr << "test did not reuse the removed body slot\n";
        return false;
    }
    world.broadPhaseAlgorithm = algorithm;
    if (!worldMatchesBruteForce(world, handles))
        return false;
    if (algorithm == phys::BroadPhaseAlgorithm::DynamicTree
        && (world.lastStepStats().broadPhaseDetails.treeRemovals != 2
            || world.lastStepStats().broadPhaseDetails.treeInsertions != 1)) {
        std::cerr << "tree did not refresh removed/reused slots after a backend switch\n";
        return false;
    }
    world.getCollider(handles.back())->localTransform.position = {30, 0, 0};
    if (!worldMatchesBruteForce(world, handles))
        return false;

    for (auto handle : handles)
        world.removeCollider(handle);
    return worldMatchesBruteForce(world, handles);
}

bool testWorldEvolutionMatches()
{
    std::array<phys::PhysicsWorld, 3> worlds;
    worlds[1].broadPhaseAlgorithm = phys::BroadPhaseAlgorithm::UniformGrid;
    worlds[2].broadPhaseAlgorithm = phys::BroadPhaseAlgorithm::DynamicTree;
    std::array<std::vector<phys::RigidBodyHandle>, 3> handles;
    std::string error;
    for (std::size_t variant = 0; variant < worlds.size(); ++variant) {
        auto& world = worlds[variant];
        phys::RigidBodyHandle body;
        phys::ColliderHandle collider;
        if (!world.createBox(30, 1, 20, {0, -0.5f, 0}, 1, true, 0, 0.6f,
                             body, collider, error)) {
            std::cerr << "evolution floor setup failed: " << error << '\n';
            return false;
        }
        for (int index = 0; index < 12; ++index) {
            phys::Vec3 position{static_cast<float>(index % 4) * 1.5f - 2.0f,
                                2.0f + static_cast<float>(index / 4) * 1.5f, 0};
            bool created = index % 2 == 0
                ? world.createBox(0.8f, 1.2f, 0.6f, position, 1, false, 0.1f, 0.5f,
                                   body, collider, error)
                : world.createSphere(0.5f, position, 1, false, 0.1f, 0.5f,
                                      body, collider, error);
            if (!created) {
                std::cerr << "evolution body setup failed: " << error << '\n';
                return false;
            }
            world.getBody(body)->setAngularVelocity({0.2f, 0.1f, -0.3f});
            handles[variant].push_back(body);
        }
    }
    for (int step = 0; step < 240; ++step) {
        if (step == 60)
            worlds[2].broadPhaseAlgorithm = phys::BroadPhaseAlgorithm::SweepAndPrune;
        if (step == 80)
            worlds[2].broadPhaseAlgorithm = phys::BroadPhaseAlgorithm::DynamicTree;
        for (auto& world : worlds)
            world.step(1.0f / 60.0f);
        for (std::size_t variant = 1; variant < worlds.size(); ++variant) {
            if (worlds[0].lastStepStats().candidatePairs != worlds[variant].lastStepStats().candidatePairs
                || worlds[0].contacts().size() != worlds[variant].contacts().size()) {
                std::cerr << "backend changed evolving world contact counts\n";
                return false;
            }
            for (std::size_t index = 0; index < handles[0].size(); ++index) {
                const auto& a = *worlds[0].getBody(handles[0][index]);
                const auto& b = *worlds[variant].getBody(handles[variant][index]);
                auto qa = a.getRotation();
                auto qb = b.getRotation();
                if (!(phys::Math3d::length(a.getPosition() - b.getPosition()) <= 1e-5f
                    && phys::Math3d::length(a.getLinearVelocity() - b.getLinearVelocity()) <= 1e-5f
                    && phys::Math3d::length(a.getAngularVelocity() - b.getAngularVelocity()) <= 1e-5f
                    && std::abs(qa.x - qb.x) + std::abs(qa.y - qb.y)
                        + std::abs(qa.z - qb.z) + std::abs(qa.w - qb.w) <= 1e-5f)) {
                    std::cerr << "backend changed an evolving body's pose or velocity\n";
                    return false;
                }
            }
        }
    }
    return true;
}

bool sameContact(const phys::ContactManifold& left, const phys::ContactManifold& right)
{
    if (!(left.bodyA == right.bodyA && left.bodyB == right.bodyB)
        || left.pointCount != right.pointCount
        || left.friction != right.friction || left.restitution != right.restitution
        || left.normal.x != right.normal.x || left.normal.y != right.normal.y
        || left.normal.z != right.normal.z)
        return false;
    for (uint32_t index = 0; index < left.pointCount; ++index) {
        const auto& a = left.points[index];
        const auto& b = right.points[index];
        if (a.localAnchorA.x != b.localAnchorA.x
            || a.localAnchorA.y != b.localAnchorA.y
            || a.localAnchorA.z != b.localAnchorA.z
            || a.localAnchorB.x != b.localAnchorB.x
            || a.localAnchorB.y != b.localAnchorB.y
            || a.localAnchorB.z != b.localAnchorB.z
            || a.penetration != b.penetration)
            return false;
    }
    return true;
}

bool testParallelNarrowPhaseMatchesSerial()
{
    phys::PhysicsWorld serial;
    serial.gravity = {};
    std::string error;
    for (int index = 0; index < 92; ++index) {
        phys::RigidBodyHandle body;
        phys::ColliderHandle collider;
        if (!serial.createSphere(1.0f, {}, 1.0f, true, 0.1f, 0.5f,
                                 body, collider, error)) {
            std::cerr << "parallel narrowphase setup failed: " << error << '\n';
            return false;
        }
    }

    phys::PhysicsWorld parallel = serial;
    parallel.setNarrowPhaseWorkerCount(4);
    parallel.setSolverWorkerCount(4);
    serial.step(0.0f);
    parallel.step(0.0f);
    if (parallel.getNarrowPhaseWorkerCount() != 4
        || parallel.getSolverWorkerCount() != 4
        || parallel.contacts().size() != serial.contacts().size()) {
        std::cerr << "parallel narrowphase contact count differs\n";
        return false;
    }
    for (std::size_t index = 0; index < serial.contacts().size(); ++index)
        if (!sameContact(serial.contacts()[index], parallel.contacts()[index])) {
            std::cerr << "parallel narrowphase contact or ordering differs\n";
            return false;
        }

    phys::PhysicsWorld copied = parallel;
    phys::PhysicsWorld moved = std::move(parallel);
    copied.step(0.0f);
    moved.step(0.0f);
    if (copied.getNarrowPhaseWorkerCount() != 4
        || moved.getNarrowPhaseWorkerCount() != 4
        || copied.getSolverWorkerCount() != 4
        || moved.getSolverWorkerCount() != 4
        || copied.contacts().size() != serial.contacts().size()
        || moved.contacts().size() != serial.contacts().size())
        return false;

    moved.setNarrowPhaseWorkerCount(1);
    moved.step(0.0f);
    if (moved.getNarrowPhaseWorkerCount() != 1
        || moved.getSolverWorkerCount() != 4
        || moved.contacts().size() != serial.contacts().size())
        return false;
    moved.setSolverWorkerCount(1);
    moved.step(0.0f);
    if (moved.getSolverWorkerCount() != 1
        || moved.contacts().size() != serial.contacts().size())
        return false;

    try {
        copied.setNarrowPhaseWorkerCount(0);
        std::cerr << "zero narrowphase workers were accepted\n";
        return false;
    }
    catch (const std::invalid_argument&) {}
    try {
        copied.setSolverWorkerCount(0);
        std::cerr << "zero solver workers were accepted\n";
        return false;
    }
    catch (const std::invalid_argument&) {}
    return copied.getNarrowPhaseWorkerCount() == 4
        && copied.getSolverWorkerCount() == 4;
}

bool testParallelSapMatchesSerial()
{
    phys::PhysicsWorld serial;
    serial.gravity = {};
    std::string error;
    for (int index = 0; index < 4096; ++index) {
        phys::RigidBodyHandle body;
        phys::ColliderHandle collider;
        float x = static_cast<float>(index / 2) * 2.0f;
        if (!serial.createSphere(0.5f, {x, 0.0f, 0.0f}, 1.0f, true,
                                 0.0f, 0.5f, body, collider, error)) {
            std::cerr << "parallel SAP setup failed: " << error << '\n';
            return false;
        }
    }

    phys::PhysicsWorld parallel = serial;
    parallel.setBroadPhaseWorkerCount(4);
    serial.step(0.0f);
    parallel.step(0.0f);
    if (parallel.getBroadPhaseWorkerCount() != 4
        || parallel.contacts().size() != serial.contacts().size()
        || parallel.lastStepStats().candidatePairs
            != serial.lastStepStats().candidatePairs
        || parallel.lastStepStats().broadPhaseDetails.xWindowComparisons
            != serial.lastStepStats().broadPhaseDetails.xWindowComparisons)
        return false;
    for (std::size_t index = 0; index < serial.contacts().size(); ++index)
        if (!sameContact(serial.contacts()[index], parallel.contacts()[index])) {
            std::cerr << "parallel SAP candidate order differs\n";
            return false;
        }

    try {
        parallel.setBroadPhaseWorkerCount(0);
        std::cerr << "zero broadphase workers were accepted\n";
        return false;
    }
    catch (const std::invalid_argument&) {}
    return parallel.getBroadPhaseWorkerCount() == 4;
}

}

int main()
{
    return testBoundaryCases() && testRandomAndMovingBounds()
        && testGridBoundariesAndOverflow()
        && testWorldFilteringAndSlotReuse(phys::BroadPhaseAlgorithm::SweepAndPrune)
        && testWorldFilteringAndSlotReuse(phys::BroadPhaseAlgorithm::UniformGrid)
        && testWorldFilteringAndSlotReuse(phys::BroadPhaseAlgorithm::DynamicTree)
        && testWorldEvolutionMatches()
        && testParallelNarrowPhaseMatchesSerial()
        && testParallelSapMatchesSerial() ? 0 : 1;
}

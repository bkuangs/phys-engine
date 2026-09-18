#include "bench_common.hpp"
#include "alloc_counter.hpp"
#include <phys/collision/broadphase.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

struct AllocationSummary
{
    std::size_t total = 0;
    std::size_t maximum = 0;
};

AllocationSummary countStepAllocations(phys::PhysicsWorld& world, std::size_t steps)
{
    AllocationSummary result;
    for (std::size_t step = 0; step < steps; ++step) {
        phys::bench::ScopedAllocCounter allocations;
        world.step(1.0f / 120.0f);
        const std::size_t count = allocations.count();
        result.total += count;
        result.maximum = std::max(result.maximum, count);
    }
    return result;
}

phys::PhysicsWorld makeSteadyScene(phys::BroadPhaseAlgorithm algorithm)
{
    phys::PhysicsWorld world;
    world.gravity = {};
    world.broadPhaseAlgorithm = algorithm;
    for (int index = 0; index < 4; ++index) {
        phys::RigidBodyHandle body;
        phys::ColliderHandle collider;
        std::string error;
        const float offset = static_cast<float>(index) * 0.05f;
        if (!world.createBox(2.0f, 2.0f, 2.0f, {offset, 0.0f, 0.0f},
                1.0f, true, 0.0f, 0.5f, body, collider, error))
            throw std::runtime_error(error);
    }
    return world;
}

bool parse(std::vector<std::string> arguments, phys::bench::ScalingOptions& options)
{
    std::vector<char*> argv;
    for (auto& argument : arguments)
        argv.push_back(argument.data());
    return phys::bench::parseScalingOptions(static_cast<int>(argv.size()), argv.data(), options);
}

bool testOptions()
{
    phys::bench::ScalingOptions options;
    if (!parse({"benchmark"}, options)
        || options.measuredSteps != 1200 || options.warmupSteps != 240
        || options.scene != phys::bench::ScalingScene::MixedFloor
        || options.algorithm != phys::BroadPhaseAlgorithm::SweepAndPrune
        || options.sleepingEnabled || options.narrowPhaseWorkers != 1)
        return false;
    if (!parse({"benchmark", "--sleep"}, options) || !options.sleepingEnabled
        || options.measuredSteps != 1200 || options.warmupSteps != 240)
        return false;
    if (!parse({"benchmark", "9", "tree", "mixed", "4", "3", "--sleep"}, options)
        || !options.sleepingEnabled || options.measuredSteps != 9 || options.warmupSteps != 4
        || options.algorithm != phys::BroadPhaseAlgorithm::DynamicTree
        || options.narrowPhaseWorkers != 3)
        return false;
    if (!parse({"benchmark", "7", "tree", "spheres", "0"}, options)
        || options.measuredSteps != 7 || options.warmupSteps != 0
        || options.scene != phys::bench::ScalingScene::Spheres
        || options.algorithm != phys::BroadPhaseAlgorithm::DynamicTree || options.sleepingEnabled)
        return false;
    return !parse({"benchmark", "0"}, options)
        && !parse({"benchmark", "-1"}, options)
        && !parse({"benchmark", "1.5"}, options)
        && !parse({"benchmark", "9999999999999999999999999"}, options)
        && !parse({"benchmark", "5", "invalid"}, options)
        && !parse({"benchmark", "5", "tree", "invalid"}, options)
        && !parse({"benchmark", "5", "tree", "mixed", "-1"}, options)
        && !parse({"benchmark", "5", "tree", "mixed", "0", "0"}, options)
        && !parse({"benchmark", "--sleep", "--sleep"}, options)
        && !parse({"benchmark", "5", "tree", "mixed", "0", "1", "extra"}, options);
}

bool testMixedScene()
{
    for (int count : {1, 3, 100, 10000}) {
        auto scene = phys::bench::makeMixedField(count, 42);
        if (scene.objects.size() != static_cast<std::size_t>(count)
            || scene.spheres != static_cast<std::size_t>((count + 1) / 2)
            || scene.boxes != static_cast<std::size_t>(count / 2)
            || !(scene.floorSize.y > 0.0f)
            || scene.floorSize.x * scene.floorSize.y * scene.floorSize.z > phys::BodyLimits::maxSize)
            return false;
        std::vector<phys::Aabb> bounds;
        bounds.push_back({{-scene.floorSize.x * 0.5f, -scene.floorSize.y, -scene.floorSize.z * 0.5f},
                          {scene.floorSize.x * 0.5f, 0, scene.floorSize.z * 0.5f}});
        for (const auto& object : scene.objects) {
            const auto* body = scene.world.getBody(object.body);
            const auto* collider = scene.world.getCollider(object.collider);
            if (!body || !collider || body->isStatic)
                return false;
            bounds.push_back(phys::Aabb::fromCollider(*collider, {body->getPosition(), body->getRotation()}));
        }
        if (!phys::BroadPhase::findCandidatePairs(bounds).empty()) {
            std::cerr << "mixed benchmark starts with overlapping shapes\n";
            return false;
        }
        scene.world.broadPhaseAlgorithm = phys::BroadPhaseAlgorithm::DynamicTree;
        scene.world.step(0.0f);
        if (scene.world.lastStepStats().possiblePairs
            != static_cast<std::size_t>(count) * (count + 1) / 2)
            return false;
    }
    auto first = phys::bench::makeMixedField(20, 42);
    auto second = phys::bench::makeMixedField(20, 42);
    for (std::size_t index = 0; index < first.objects.size(); ++index) {
        auto a = first.world.getBody(first.objects[index].body)->getPosition();
        auto b = second.world.getBody(second.objects[index].body)->getPosition();
        if (a.x != b.x || a.y != b.y || a.z != b.z
            || first.world.getCollider(first.objects[index].collider)->shape.index()
                != second.world.getCollider(second.objects[index].collider)->shape.index())
            return false;
    }
    return true;
}

bool testWarmupAndReport()
{
    phys::bench::ScalingOptions options;
    options.algorithm = phys::BroadPhaseAlgorithm::DynamicTree;
    options.measuredSteps = 120;
    auto report = phys::bench::runBenchmark(32, 120.0, 42, options);
    if (report.sampleCount != 120 || report.warmupSteps != 240
        || report.staticBodies != 1 || report.sphereBodies != 16 || report.boxBodies != 16
        || report.narrowPhaseWorkers != 1
        || report.totalTreeInsertions != 0 || report.broadPhaseDetails.treeProxyCount != 33
        || report.possiblePairs != 32 * 33 / 2
        || !(report.warmupTotalMs >= report.coldFirstStepMs)
        || !std::isfinite(report.stepTime.mean) || !std::isfinite(report.finalFloorPenetration)
        || report.slowestStep.measuredStep == 0
        || report.slowestStep.measuredStep > report.sampleCount
        || report.slowestStep.stats.totalMs != report.stepTime.max
        || report.totalAllocations < report.maxAllocationsPerStep
        || report.allocationsPerStep != static_cast<double>(report.totalAllocations) / report.sampleCount
        || !std::isfinite(report.solverTime.p99)
        || !std::isfinite(report.unattributedTime.max)
        || report.meanContacts <= 0 || report.meanContactPoints < report.meanContacts
        || std::abs(report.sphereSphereCandidatesPerStep
                + report.sphereBoxCandidatesPerStep
                + report.boxBoxCandidatesPerStep
                - report.meanCandidatePairs) > 1e-6
        || std::abs(report.sphereSphereContactsPerStep
                + report.sphereBoxContactsPerStep
                + report.boxBoxContactsPerStep
                - report.meanContacts) > 1e-6
        || report.solverPrepareMeanMs < 0 || report.solverWarmStartMeanMs < 0
        || report.solverVelocityIterationsMeanMs < 0
        || report.solverCacheUpdateMeanMs < 0
        || report.solverVelocityPointVisitsPerStep
            != report.solverPreparedPointsPerStep * 8
        || report.sleepingEnabled || report.meanAwakeBodies != 32 || report.meanSleepingBodies != 0)
        return false;
    std::ostringstream output;
    report.print(output);
    if (output.str().find("Dynamic bodies:             32") == std::string::npos
        || output.str().find("Warmup steps:               240") == std::string::npos
        || output.str().find("Measured steps:             120") == std::string::npos
        || output.str().find("Narrowphase workers:        1") == std::string::npos
        || output.str().find("Stage timing distribution (ms):") == std::string::npos
        || output.str().find("Slowest measured step (") == std::string::npos
        || output.str().find("Narrowphase detail (mean per step):") == std::string::npos
        || output.str().find("Solver detail (mean per step):") == std::string::npos
        || output.str().find("max / step:") == std::string::npos
        || output.str().find("candidates/manifolds:") == std::string::npos
        || output.str().find("no escaped/below-floor bodies") == std::string::npos)
        return false;
    options.sleepingEnabled = true;
    options.measuredSteps = 1;
    report = phys::bench::runBenchmark(32, 120.0, 42, options);
    if (!report.sleepingEnabled || report.meanSleepingBodies <= 0
        || std::abs(report.meanAwakeBodies + report.meanSleepingBodies - 32) > 1e-6
        || report.meanSolvedContacts > report.meanContacts)
        return false;
    std::ostringstream sleepingOutput;
    report.print(sleepingOutput);
    if (sleepingOutput.str().find("Sleeping:                   enabled") == std::string::npos)
        return false;
    options.sleepingEnabled = false;
    options.measuredSteps = 1;
    options.warmupSteps = 0;
    report = phys::bench::runBenchmark(5000, 120.0, 42, options);
    if (report.sampleCount != 1 || report.warmupSteps != 0
        || report.totalTreeInsertions != 5001 || report.warmupTotalMs != 0
        || report.coldFirstStepMs != report.firstStepMs)
        return false;
    options.scene = phys::bench::ScalingScene::Spheres;
    report = phys::bench::runBenchmark(10, 120.0, 42, options);
    return report.staticBodies == 0 && report.sphereBodies == 10 && report.boxBodies == 0
        && report.sampleCount == 1 && report.broadPhaseDetails.treeProxyCount == 10;
}

bool testSteadyStateAllocations()
{
    for (phys::BroadPhaseAlgorithm algorithm : {
             phys::BroadPhaseAlgorithm::SweepAndPrune,
             phys::BroadPhaseAlgorithm::UniformGrid,
             phys::BroadPhaseAlgorithm::DynamicTree}) {
        auto world = makeSteadyScene(algorithm);
        for (int step = 0; step < 3; ++step)
            world.step(1.0f / 120.0f);
        const AllocationSummary steady = countStepAllocations(world, 32);
        if (steady.total != 0 || steady.maximum != 0) {
            std::cerr << "steady allocation regression for algorithm "
                      << static_cast<int>(algorithm) << ": "
                      << steady.total << " total, " << steady.maximum << " max\n";
            return false;
        }

        phys::PhysicsWorld copied = world;
        for (int step = 0; step < 3; ++step)
            copied.step(1.0f / 120.0f);
        const AllocationSummary copiedSteady = countStepAllocations(copied, 8);
        if (copiedSteady.total != 0 || copiedSteady.maximum != 0) {
            std::cerr << "copied-world allocation regression for algorithm "
                      << static_cast<int>(algorithm) << '\n';
            return false;
        }

        phys::PhysicsWorld moved = std::move(world);
        const AllocationSummary movedSteady = countStepAllocations(moved, 8);
        if (movedSteady.total != 0 || movedSteady.maximum != 0) {
            std::cerr << "moved-world allocation regression for algorithm "
                      << static_cast<int>(algorithm) << '\n';
            return false;
        }

        for (int index = 0; index < 8; ++index) {
            phys::RigidBodyHandle body;
            phys::ColliderHandle collider;
            std::string error;
            if (!moved.createBox(2.0f, 2.0f, 2.0f,
                    {0.25f + static_cast<float>(index) * 0.05f, 0.0f, 0.0f},
                    1.0f, true, 0.0f, 0.5f, body, collider, error))
                throw std::runtime_error(error);
        }
        for (int step = 0; step < 3; ++step)
            moved.step(1.0f / 120.0f);
        const AllocationSummary grownSteady = countStepAllocations(moved, 8);
        if (grownSteady.total != 0 || grownSteady.maximum != 0) {
            std::cerr << "grown-world allocation regression for algorithm "
                      << static_cast<int>(algorithm) << ": "
                      << grownSteady.total << " total, " << grownSteady.maximum << " max\n";
            return false;
        }
    }

    phys::PhysicsWorld sleepingWorld;
    sleepingWorld.gravity = {};
    sleepingWorld.broadPhaseAlgorithm = phys::BroadPhaseAlgorithm::DynamicTree;
    sleepingWorld.setSleepingEnabled(true);
    for (int index = 0; index < 4; ++index) {
        phys::RigidBodyHandle body;
        phys::ColliderHandle collider;
        std::string error;
        if (!sleepingWorld.createBox(1.0f, 1.0f, 1.0f,
                {static_cast<float>(index) * 5.0f, 0.0f, 0.0f},
                1.0f, false, 0.0f, 0.5f, body, collider, error))
            throw std::runtime_error(error);
    }
    for (int step = 0; step < 240; ++step)
        sleepingWorld.step(1.0f / 120.0f);
    const AllocationSummary sleepingSteady = countStepAllocations(sleepingWorld, 32);
    const auto& stats = sleepingWorld.lastStepStats();
    if (stats.awakeBodyCount != 0 || stats.sleepingBodyCount != 4
        || sleepingSteady.total != 0 || sleepingSteady.maximum != 0) {
        std::cerr << "sleeping-world allocation regression: "
                  << sleepingSteady.total << " total, " << sleepingSteady.maximum
                  << " max, " << stats.awakeBodyCount << " awake\n";
        return false;
    }
    return true;
}

}

int main()
{
    try {
        if (testOptions() && testMixedScene() && testWarmupAndReport()
            && testSteadyStateAllocations())
            return 0;
        std::cerr << "benchmark configuration, scene, or report regression\n";
    }
    catch (const std::runtime_error& error) {
        std::cerr << "benchmark scene failed: " << error.what() << '\n';
    }
    return 1;
}

#include "bench_common.hpp"
#include <phys/collision/broadphase.hpp>
#include <cmath>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

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
        || options.algorithm != phys::BroadPhaseAlgorithm::SweepAndPrune || options.sleepingEnabled)
        return false;
    if (!parse({"benchmark", "--sleep"}, options) || !options.sleepingEnabled
        || options.measuredSteps != 1200 || options.warmupSteps != 240)
        return false;
    if (!parse({"benchmark", "9", "tree", "mixed", "4", "--sleep"}, options)
        || !options.sleepingEnabled || options.measuredSteps != 9 || options.warmupSteps != 4
        || options.algorithm != phys::BroadPhaseAlgorithm::DynamicTree)
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
        && !parse({"benchmark", "--sleep", "--sleep"}, options)
        && !parse({"benchmark", "5", "tree", "mixed", "0", "extra"}, options);
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
        || report.totalTreeInsertions != 0 || report.broadPhaseDetails.treeProxyCount != 33
        || report.possiblePairs != 32 * 33 / 2
        || !(report.warmupTotalMs >= report.coldFirstStepMs)
        || !std::isfinite(report.stepTime.mean) || !std::isfinite(report.finalFloorPenetration)
        || report.meanContacts <= 0 || report.meanContactPoints < report.meanContacts
        || report.sleepingEnabled || report.meanAwakeBodies != 32 || report.meanSleepingBodies != 0)
        return false;
    std::ostringstream output;
    report.print(output);
    if (output.str().find("Dynamic bodies:             32") == std::string::npos
        || output.str().find("Warmup steps:               240") == std::string::npos
        || output.str().find("Measured steps:             120") == std::string::npos
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

}

int main()
{
    try {
        if (testOptions() && testMixedScene() && testWarmupAndReport())
            return 0;
        std::cerr << "benchmark configuration, scene, or report regression\n";
    }
    catch (const std::runtime_error& error) {
        std::cerr << "benchmark scene failed: " << error.what() << '\n';
    }
    return 1;
}

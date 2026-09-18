#include "bench_common.hpp"
#include <algorithm>
#include <charconv>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace {

using Clock = std::chrono::steady_clock;
constexpr float dt = 1.0f / 120.0f;

phys::PhysicsWorld makeBoxStacks(std::vector<phys::RigidBodyHandle>& handles)
{
    phys::PhysicsWorld world;
    std::string error;
    phys::RigidBodyHandle body;
    phys::ColliderHandle collider;
    if (!world.createBox(30, 1, 20, {0, -0.5f, 0}, 1, true, 0, 0.6f,
                         body, collider, error))
        throw std::runtime_error("Profile floor setup failed: " + error);
    handles.reserve(512);
    for (int row = 0; row < 16; ++row)
        for (int column = 0; column < 16; ++column)
            for (int level = 0; level < 2; ++level) {
                phys::Vec3 position{-12.0f + 1.6f * column, 0.5f + 1.02f * level,
                                    -9.0f + 1.2f * row};
                if (!world.createBox(1, 1, 1, position, 1, false, 0, 0.6f,
                                     body, collider, error))
                    throw std::runtime_error("Profile box setup failed: " + error);
                handles.push_back(body);
            }
    return world;
}

std::pair<float, float> maxSpeeds(const phys::PhysicsWorld& world,
                                 const std::vector<phys::RigidBodyHandle>& handles)
{
    float linear = 0;
    float angular = 0;
    for (auto handle : handles) {
        const auto* body = world.getBody(handle);
        if (!body)
            throw std::runtime_error("Profile body handle became invalid");
        float speed = phys::Math3d::length(body->getLinearVelocity());
        float spin = phys::Math3d::length(body->getAngularVelocity());
        if (!std::isfinite(speed) || !std::isfinite(spin))
            throw std::runtime_error("Profile body velocity became non-finite");
        linear = std::max(linear, speed);
        angular = std::max(angular, spin);
    }
    return {linear, angular};
}

struct Totals
{
    std::size_t steps = 0;
    std::size_t contacts = 0;
    std::size_t minContacts = std::numeric_limits<std::size_t>::max();
    std::size_t maxContacts = 0;
    std::size_t reinsertions = 0;
    std::size_t solvedContacts = 0;
    std::size_t awakeBodies = 0;
    std::size_t sleepingBodies = 0;
    std::size_t minSleepingBodies = std::numeric_limits<std::size_t>::max();
    double step = 0;
    double broadphase = 0;
    double maintenance = 0;
    double query = 0;
    double narrowphase = 0;
    double solver = 0;
    double solverPrepare = 0;
    double solverWarmStart = 0;
    double solverVelocityIterations = 0;
    double solverCacheUpdate = 0;
    double integrateVelocity = 0;
    double integratePose = 0;
    std::size_t solverPreparedPoints = 0;
    std::size_t solverWarmStartComparisons = 0;
    std::size_t solverWarmStartMatches = 0;
    std::size_t solverVelocityPointVisits = 0;
    std::size_t solverIslands = 0;
    std::size_t solverLargestIslandContacts = 0;
    std::size_t solverLargestIslandContactsTotal = 0;
    std::size_t solverLargestIslandPoints = 0;
    std::size_t solverLargestIslandPointsTotal = 0;

    void record(const phys::StepStats& stats)
    {
        if (!std::isfinite(stats.totalMs))
            throw std::runtime_error("Profile step timing became non-finite");
        ++steps;
        contacts += stats.contactCount;
        minContacts = std::min(minContacts, stats.contactCount);
        maxContacts = std::max(maxContacts, stats.contactCount);
        reinsertions += stats.broadPhaseDetails.treeReinsertions;
        solvedContacts += stats.solvedContactCount;
        awakeBodies += stats.awakeBodyCount;
        sleepingBodies += stats.sleepingBodyCount;
        minSleepingBodies = std::min(minSleepingBodies, stats.sleepingBodyCount);
        step += stats.totalMs;
        broadphase += stats.broadPhaseMs;
        maintenance += stats.broadPhaseDetails.recordBuildMs;
        query += stats.broadPhaseDetails.sweepMs;
        narrowphase += stats.narrowPhaseMs;
        solver += stats.solverMs;
        solverPrepare += stats.solverDetails.prepareMs;
        solverWarmStart += stats.solverDetails.warmStartMs;
        solverVelocityIterations += stats.solverDetails.velocityIterationsMs;
        solverCacheUpdate += stats.solverDetails.cacheUpdateMs;
        solverPreparedPoints += stats.solverDetails.preparedPoints;
        solverWarmStartComparisons += stats.solverDetails.warmStartComparisons;
        solverWarmStartMatches += stats.solverDetails.warmStartMatches;
        solverVelocityPointVisits += stats.solverDetails.velocityPointVisits;
        solverIslands += stats.solverDetails.islandCount;
        solverLargestIslandContactsTotal += stats.solverDetails.largestIslandContacts;
        solverLargestIslandContacts = std::max(
            solverLargestIslandContacts, stats.solverDetails.largestIslandContacts);
        solverLargestIslandPointsTotal += stats.solverDetails.largestIslandPoints;
        solverLargestIslandPoints = std::max(
            solverLargestIslandPoints, stats.solverDetails.largestIslandPoints);
        integrateVelocity += stats.integrateVelocityMs;
        integratePose += stats.integratePoseMs;
    }
};

}

int main(int argc, char** argv)
{
    if (argc < 2 || argc > 8) {
        std::cerr << "Usage: phys_cpu_profile <spheres|boxes|mixed5k|mixed10k> [seconds=20] [--wait] [--sleep] [--workers=N] [--solver-workers=N] [--broadphase-workers=N]\n";
        return 1;
    }
    std::string_view scene = argv[1];
    int seconds = 20;
    if (argc >= 3) {
        std::string_view duration = argv[2];
        auto [end, error] = std::from_chars(duration.data(), duration.data() + duration.size(), seconds);
        if (error != std::errc{} || end != duration.data() + duration.size() || seconds < 1 || seconds > 300) {
            std::cerr << "Profile duration must be an integer from 1 to 300 seconds\n";
            return 1;
        }
    }
    bool mixed = scene == "mixed5k" || scene == "mixed10k";
    if (scene != "spheres" && scene != "boxes" && !mixed) {
        std::cerr << "Expected scene spheres, boxes, mixed5k, or mixed10k\n";
        return 1;
    }
    bool wait = false;
    bool sleeping = false;
    std::size_t narrowPhaseWorkers = 1;
    bool workersSet = false;
    std::size_t solverWorkers = 1;
    bool solverWorkersSet = false;
    std::size_t broadPhaseWorkers = 1;
    bool broadPhaseWorkersSet = false;
    for (int index = 3; index < argc; ++index) {
        std::string_view option = argv[index];
        if (option == "--wait" && !wait)
            wait = true;
        else if (option == "--sleep" && !sleeping)
            sleeping = true;
        else if (option.starts_with("--workers=") && !workersSet) {
            std::string_view count = option.substr(std::string_view("--workers=").size());
            auto [end, error] = std::from_chars(
                count.data(), count.data() + count.size(), narrowPhaseWorkers);
            if (error != std::errc{} || end != count.data() + count.size()
                || narrowPhaseWorkers == 0) {
                std::cerr << "Narrowphase workers must be a positive integer\n";
                return 1;
            }
            workersSet = true;
        }
        else if (option.starts_with("--solver-workers=") && !solverWorkersSet) {
            std::string_view count = option.substr(
                std::string_view("--solver-workers=").size());
            auto [end, error] = std::from_chars(
                count.data(), count.data() + count.size(), solverWorkers);
            if (error != std::errc{} || end != count.data() + count.size()
                || solverWorkers == 0) {
                std::cerr << "Solver workers must be a positive integer\n";
                return 1;
            }
            solverWorkersSet = true;
        }
        else if (option.starts_with("--broadphase-workers=") && !broadPhaseWorkersSet) {
            std::string_view count = option.substr(
                std::string_view("--broadphase-workers=").size());
            auto [end, error] = std::from_chars(
                count.data(), count.data() + count.size(), broadPhaseWorkers);
            if (error != std::errc{} || end != count.data() + count.size()
                || broadPhaseWorkers == 0) {
                std::cerr << "Broadphase workers must be a positive integer\n";
                return 1;
            }
            broadPhaseWorkersSet = true;
        }
        else {
            std::cerr << "Unknown or duplicate profile option: " << option << '\n';
            return 1;
        }
    }

    std::vector<phys::RigidBodyHandle> handles;
    phys::PhysicsWorld world;
    std::size_t dynamicBodies = 0;
    if (scene == "spheres") {
        world = phys::bench::makeSphereField(10000, 42);
        dynamicBodies = 10000;
    } else if (scene == "boxes") {
        world = makeBoxStacks(handles);
        dynamicBodies = handles.size();
    } else {
        int bodyCount = scene == "mixed5k" ? 5000 : 10000;
        auto mixedScene = phys::bench::makeMixedField(bodyCount, 42);
        handles.reserve(mixedScene.objects.size());
        for (const auto& object : mixedScene.objects)
            handles.push_back(object.body);
        world = std::move(mixedScene.world);
        dynamicBodies = handles.size();
    }
    world.broadPhaseAlgorithm = mixed
        ? phys::BroadPhaseAlgorithm::SweepAndPrune
        : phys::BroadPhaseAlgorithm::DynamicTree;
    world.setSleepingEnabled(sleeping);
    world.setNarrowPhaseWorkerCount(narrowPhaseWorkers);
    world.setSolverWorkerCount(solverWorkers);
    world.setBroadPhaseWorkerCount(broadPhaseWorkers);
    int warmup = 0;
    float warmLinear = 0;
    float warmAngular = 0;
    if (scene == "boxes") {
        do {
            for (int step = 0; step < 120; ++step)
                world.step(dt);
            warmup += 120;
            auto speeds = maxSpeeds(world, handles);
            warmLinear = speeds.first;
            warmAngular = speeds.second;
        } while (warmup < 1200 && (warmup < 240 || warmLinear > 0.05f || warmAngular > 0.05f
                                 || (sleeping && world.lastStepStats().sleepingBodyCount != handles.size())));
        if (warmLinear > 0.05f || warmAngular > 0.05f
            || (sleeping && world.lastStepStats().sleepingBodyCount != handles.size())) {
            std::cerr << "Box profiling scene did not settle within the warmup budget\n";
            return 1;
        }
    }
    else if (mixed) {
        for (warmup = 0; warmup < 240; ++warmup)
            world.step(dt);
        auto speeds = maxSpeeds(world, handles);
        warmLinear = speeds.first;
        warmAngular = speeds.second;
    }
    std::size_t warmPoints = 0;
    for (const auto& contact : world.contacts())
        warmPoints += contact.pointCount;
    std::cout << "PROFILE_READY scene=" << scene
              << " dynamic_bodies=" << dynamicBodies
              << " static_bodies=" << (scene == "spheres" ? 0 : 1)
              << " warmup_steps=" << warmup
              << " max_linear_speed=" << warmLinear << " max_angular_speed=" << warmAngular
              << " contacts=" << world.contacts().size() << " contact_points=" << warmPoints
              << " sleeping_enabled=" << sleeping
              << " narrowphase_workers=" << narrowPhaseWorkers
              << " solver_workers=" << solverWorkers
              << " broadphase_workers=" << broadPhaseWorkers
              << " sleeping_bodies=" << world.lastStepStats().sleepingBodyCount << std::endl;
    if (wait) {
        std::string line;
        if (!std::getline(std::cin, line)) {
            std::cerr << "Profile start signal was not received\n";
            return 1;
        }
    }

    Totals totals;
    std::size_t batches = 1;
    int batchSteps = 0;
    auto start = Clock::now();
    auto deadline = start + std::chrono::seconds(seconds);
    do {
        world.step(dt);
        totals.record(world.lastStepStats());
        ++batchSteps;
        if (Clock::now() >= deadline)
            break;
        if (scene == "spheres" && batchSteps == 25) {
            world = phys::bench::makeSphereField(10000, 42);
            world.broadPhaseAlgorithm = phys::BroadPhaseAlgorithm::DynamicTree;
            world.setSleepingEnabled(sleeping);
            batchSteps = 0;
            ++batches;
        }
    } while (true);

    double count = static_cast<double>(totals.steps);
    std::cout << std::fixed << std::setprecision(4)
              << "scene: " << scene << "\nsteps: " << totals.steps << "\nbatches: " << batches
              << "\nwall_seconds: " << std::chrono::duration<double>(Clock::now() - start).count()
              << "\nmean_step_ms: " << totals.step / count
              << "\nmean_broadphase_ms: " << totals.broadphase / count
              << "\nmean_tree_maintenance_ms: " << totals.maintenance / count
              << "\nmean_tree_query_ms: " << totals.query / count
              << "\nmean_narrowphase_ms: " << totals.narrowphase / count
              << "\nmean_solver_ms: " << totals.solver / count
              << "\nmean_solver_prepare_ms: " << totals.solverPrepare / count
              << "\nmean_solver_warm_start_ms: " << totals.solverWarmStart / count
              << "\nmean_solver_velocity_iterations_ms: " << totals.solverVelocityIterations / count
              << "\nmean_solver_cache_update_ms: " << totals.solverCacheUpdate / count
              << "\nmean_solver_prepared_points: " << totals.solverPreparedPoints / count
              << "\nmean_solver_warm_start_comparisons: " << totals.solverWarmStartComparisons / count
              << "\nmean_solver_warm_start_matches: " << totals.solverWarmStartMatches / count
              << "\nmean_solver_velocity_point_visits: " << totals.solverVelocityPointVisits / count
              << "\nmean_solver_islands: " << totals.solverIslands / count
              << "\nmean_largest_solver_island_contacts: "
              << totals.solverLargestIslandContactsTotal / count
              << "\nmax_solver_island_contacts: " << totals.solverLargestIslandContacts
              << "\nmean_largest_solver_island_points: "
              << totals.solverLargestIslandPointsTotal / count
              << "\nmax_solver_island_points: " << totals.solverLargestIslandPoints
              << "\nmean_integrate_velocity_ms: " << totals.integrateVelocity / count
              << "\nmean_integrate_pose_ms: " << totals.integratePose / count
              << "\nmean_contacts: " << totals.contacts / count
              << "\nmin_contacts: " << totals.minContacts << "\nmax_contacts: " << totals.maxContacts
              << "\nmean_solved_contacts: " << totals.solvedContacts / count
              << "\nmean_awake_bodies: " << totals.awakeBodies / count
              << "\nmean_sleeping_bodies: " << totals.sleepingBodies / count
              << "\nmin_sleeping_bodies: " << totals.minSleepingBodies
              << "\nreinsertions: " << totals.reinsertions << '\n';
    if (scene == "boxes") {
        auto speeds = maxSpeeds(world, handles);
        std::cout << "final_max_linear_speed: " << speeds.first
                  << "\nfinal_max_angular_speed: " << speeds.second << '\n';
        if (totals.contacts / count < 256.0 || speeds.first > 0.05f || speeds.second > 0.05f
            || (sleeping && totals.minSleepingBodies != handles.size())) {
            std::cerr << "Box profiling scene did not remain settled and contact-heavy\n";
            return 1;
        }
    }
    return 0;
}

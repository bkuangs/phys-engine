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
    double step = 0;
    double broadphase = 0;
    double maintenance = 0;
    double query = 0;
    double narrowphase = 0;
    double solver = 0;
    double integrateVelocity = 0;
    double integratePose = 0;

    void record(const phys::StepStats& stats)
    {
        if (!std::isfinite(stats.totalMs))
            throw std::runtime_error("Profile step timing became non-finite");
        ++steps;
        contacts += stats.contactCount;
        minContacts = std::min(minContacts, stats.contactCount);
        maxContacts = std::max(maxContacts, stats.contactCount);
        reinsertions += stats.broadPhaseDetails.treeReinsertions;
        step += stats.totalMs;
        broadphase += stats.broadPhaseMs;
        maintenance += stats.broadPhaseDetails.recordBuildMs;
        query += stats.broadPhaseDetails.sweepMs;
        narrowphase += stats.narrowPhaseMs;
        solver += stats.solverMs;
        integrateVelocity += stats.integrateVelocityMs;
        integratePose += stats.integratePoseMs;
    }
};

}

int main(int argc, char** argv)
{
    if (argc < 2 || argc > 4) {
        std::cerr << "Usage: phys_cpu_profile <spheres|boxes> [seconds=20] [--wait]\n";
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
    if ((scene != "spheres" && scene != "boxes") || (argc == 4 && std::string_view(argv[3]) != "--wait")) {
        std::cerr << "Expected scene spheres/boxes and optional --wait\n";
        return 1;
    }

    std::vector<phys::RigidBodyHandle> handles;
    auto world = scene == "spheres" ? phys::bench::makeSphereField(10000, 42) : makeBoxStacks(handles);
    world.broadPhaseAlgorithm = phys::BroadPhaseAlgorithm::DynamicTree;
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
        } while (warmup < 1200 && (warmup < 240 || warmLinear > 0.05f || warmAngular > 0.05f));
        if (warmLinear > 0.05f || warmAngular > 0.05f) {
            std::cerr << "Box profiling scene did not settle within the warmup budget\n";
            return 1;
        }
    }
    std::size_t warmPoints = 0;
    for (const auto& contact : world.contacts())
        warmPoints += contact.pointCount;
    std::cout << "PROFILE_READY scene=" << scene
              << " dynamic_bodies=" << (scene == "spheres" ? 10000 : 512)
              << " static_bodies=" << (scene == "spheres" ? 0 : 1)
              << " warmup_steps=" << warmup
              << " max_linear_speed=" << warmLinear << " max_angular_speed=" << warmAngular
              << " contacts=" << world.contacts().size() << " contact_points=" << warmPoints << std::endl;
    if (argc == 4) {
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
              << "\nmean_integrate_velocity_ms: " << totals.integrateVelocity / count
              << "\nmean_integrate_pose_ms: " << totals.integratePose / count
              << "\nmean_contacts: " << totals.contacts / count
              << "\nmin_contacts: " << totals.minContacts << "\nmax_contacts: " << totals.maxContacts
              << "\nreinsertions: " << totals.reinsertions << '\n';
    if (scene == "boxes") {
        auto speeds = maxSpeeds(world, handles);
        std::cout << "final_max_linear_speed: " << speeds.first
                  << "\nfinal_max_angular_speed: " << speeds.second << '\n';
        if (totals.contacts / count < 256.0 || speeds.first > 0.05f || speeds.second > 0.05f) {
            std::cerr << "Box profiling scene did not remain settled and contact-heavy\n";
            return 1;
        }
    }
    return 0;
}

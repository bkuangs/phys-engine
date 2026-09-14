#include "bench_common.hpp"
#include "alloc_counter.hpp"
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <random>
#include <string>

namespace phys::bench {

PhysicsWorld makeSphereField(int bodyCount, unsigned seed)
{
    PhysicsWorld world;

    std::mt19937 rng(seed);
    constexpr float radius = 0.5f;
    float side = std::cbrt(static_cast<float>(bodyCount)) * (radius * 4.0f);
    std::uniform_real_distribution<float> position(-side * 0.5f, side * 0.5f);

    for (int i = 0; i < bodyCount; ++i) {
        RigidBodyHandle body;
        ColliderHandle collider;
        std::string error;

        world.createSphere(
            radius,
            Vec3{position(rng), position(rng), position(rng)},
            1.0f,
            false,
            0.5f,
            body,
            collider,
            error
        );
    }

    return world;
}

BenchmarkReport runBenchmark(int bodyCount, double simulationHz, std::size_t sampleCount, unsigned seed)
{
    PhysicsWorld world = makeSphereField(bodyCount, seed);
    float dt = static_cast<float>(1.0 / simulationHz);
    double deadlineMs = 1000.0 / simulationHz;

    DurationStats stepStats;
    DurationStats broadPhaseStats;
    DurationStats narrowPhaseStats;
    DurationStats solverStats;
    std::size_t deadlineMisses = 0;
    std::size_t totalAllocations = 0;

    for (std::size_t i = 0; i < sampleCount; ++i) {
        ScopedAllocCounter allocs;
        world.step(dt);
        const StepStats& stats = world.lastStepStats();

        stepStats.record(stats.totalMs);
        broadPhaseStats.record(stats.broadPhaseMs);
        narrowPhaseStats.record(stats.narrowPhaseMs);
        solverStats.record(stats.solverMs);
        if (stats.totalMs > deadlineMs) ++deadlineMisses;
        totalAllocations += allocs.count();
    }

    const StepStats& lastStats = world.lastStepStats();

    BenchmarkReport report;
    report.bodyCount = bodyCount;
    report.simulationHz = simulationHz;
    report.sampleCount = sampleCount;
    report.stepTime = stepStats.summarize();
    report.deadlineMisses = deadlineMisses;
    report.possiblePairs = lastStats.possiblePairs;
    report.candidatePairs = lastStats.candidatePairs;
    report.broadPhaseMeanMs = broadPhaseStats.summarize().mean;
    report.narrowPhaseMeanMs = narrowPhaseStats.summarize().mean;
    report.solverMeanMs = solverStats.summarize().mean;
    report.allocationsPerStep = sampleCount > 0 ? totalAllocations / sampleCount : 0;
    return report;
}

std::size_t sampleCountForBodies(int bodyCount, std::size_t requestedSamples)
{
    if (bodyCount <= 2500) return requestedSamples;
    if (bodyCount <= 5000) return std::max<std::size_t>(100, requestedSamples / 4);
    return std::max<std::size_t>(25, requestedSamples / 20);
}

int parseSampleCount(int argc, char** argv, std::size_t defaultSamples)
{
    if (argc < 2) return static_cast<int>(defaultSamples);

    char* end = nullptr;
    long parsed = std::strtol(argv[1], &end, 10);
    if (end == argv[1] || *end != '\0' || parsed <= 0) {
        return static_cast<int>(defaultSamples);
    }
    return static_cast<int>(parsed);
}

void BenchmarkReport::print(std::ostream& out) const
{
    out << std::fixed << std::setprecision(2);
    out << "Bodies:                     " << bodyCount << "\n";
    out << "Simulation frequency:       " << simulationHz << " Hz\n\n";

    out << "Step time:\n";
    out << "    mean:                    " << stepTime.mean << " ms\n";
    out << "    p50:                     " << stepTime.p50 << " ms\n";
    out << "    p95:                     " << stepTime.p95 << " ms\n";
    out << "    p99:                     " << stepTime.p99 << " ms\n";
    out << "    max:                     " << stepTime.max << " ms\n\n";

    out << "Deadline misses (>" << (1000.0 / simulationHz) << "ms):   "
        << deadlineMisses << " / " << sampleCount << "\n\n";

    out << "Broadphase:\n";
    out << "    possible pairs:          " << possiblePairs << "\n";
    out << "    candidate pairs:         " << candidatePairs << "\n";
    out << "    broadphase time:         " << broadPhaseMeanMs << " ms\n\n";

    out << "Narrowphase time:            " << narrowPhaseMeanMs << " ms\n";
    out << "Solver time:                 " << solverMeanMs << " ms\n\n";

    out << "Heap allocations / step:     " << allocationsPerStep << "\n\n";
}

}

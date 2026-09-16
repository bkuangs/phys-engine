#pragma once
#include <cstddef>
#include <ostream>
#include <vector>
#include <phys/world/physics_world.hpp>
#include "bench_stats.hpp"

namespace phys::bench {

// Scatters bodyCount dynamic spheres in a cube whose volume scales with
// bodyCount, so collision density stays roughly comparable across scales.
PhysicsWorld makeSphereField(int bodyCount, unsigned seed);

struct BenchmarkReport
{
    int bodyCount = 0;
    double simulationHz = 0.0;
    std::size_t sampleCount = 0;
    BroadPhaseAlgorithm algorithm = BroadPhaseAlgorithm::SweepAndPrune;

    DurationStats::Summary stepTime;
    std::size_t deadlineMisses = 0;

    std::size_t possiblePairs = 0;
    std::size_t candidatePairs = 0;
    double broadPhaseMeanMs = 0.0;
    double broadPhaseCollectMeanMs = 0.0;
    double broadPhaseFilterMeanMs = 0.0;
    BroadPhaseStats broadPhaseDetails{}; // Mean timings; other fields from the last step.

    double narrowPhaseMeanMs = 0.0;
    double solverMeanMs = 0.0;

    std::size_t allocationsPerStep = 0;

    void print(std::ostream& out) const;
};

// Runs `sampleCount` fixed steps at `simulationHz` against a fresh world of
// `bodyCount` bodies and returns the aggregated report.
BenchmarkReport runBenchmark(int bodyCount, double simulationHz, std::size_t sampleCount,
    unsigned seed, BroadPhaseAlgorithm algorithm = BroadPhaseAlgorithm::SweepAndPrune);

std::size_t sampleCountForBodies(int bodyCount, std::size_t requestedSamples);
int parseSampleCount(int argc, char** argv, std::size_t defaultSamples);
bool parseBroadPhaseAlgorithm(int argc, char** argv, BroadPhaseAlgorithm& algorithm);

}

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

enum class ScalingScene { MixedFloor, Spheres };

struct ScalingOptions
{
    std::size_t measuredSteps = 1200;
    std::size_t warmupSteps = 240;
    BroadPhaseAlgorithm algorithm = BroadPhaseAlgorithm::SweepAndPrune;
    ScalingScene scene = ScalingScene::MixedFloor;
    bool sleepingEnabled = false;
    std::size_t narrowPhaseWorkers = 1;
    std::size_t solverWorkers = 1;
};

struct BenchmarkObject
{
    RigidBodyHandle body;
    ColliderHandle collider;
};

struct BenchmarkScene
{
    PhysicsWorld world;
    std::vector<BenchmarkObject> objects;
    Vec3 floorSize{};
    std::size_t spheres = 0;
    std::size_t boxes = 0;
};

BenchmarkScene makeMixedField(int bodyCount, unsigned seed);

struct TailStepSample
{
    std::size_t measuredStep = 0; // One-based index within the measured interval.
    std::size_t allocations = 0;
    std::size_t contactPoints = 0;
    double unattributedMs = 0.0;
    StepStats stats{};
};

struct BenchmarkReport
{
    int bodyCount = 0;
    double simulationHz = 0.0;
    std::size_t sampleCount = 0;
    BroadPhaseAlgorithm algorithm = BroadPhaseAlgorithm::SweepAndPrune;
    ScalingScene scene = ScalingScene::MixedFloor;
    std::size_t warmupSteps = 0;
    std::size_t staticBodies = 0;
    std::size_t sphereBodies = 0;
    std::size_t boxBodies = 0;
    Vec3 floorSize{};
    double warmupTotalMs = 0.0;
    double coldFirstStepMs = 0.0;

    DurationStats::Summary stepTime;
    DurationStats::Summary integrateVelocityTime;
    DurationStats::Summary broadPhaseTime;
    DurationStats::Summary narrowPhaseTime;
    DurationStats::Summary solverTime;
    DurationStats::Summary integratePoseTime;
    DurationStats::Summary unattributedTime;
    TailStepSample slowestStep;
    double firstStepMs = 0.0;
    std::size_t deadlineMisses = 0;

    std::size_t possiblePairs = 0;
    std::size_t candidatePairs = 0;
    double broadPhaseMeanMs = 0.0;
    double broadPhaseCollectMeanMs = 0.0;
    double broadPhaseFilterMeanMs = 0.0;
    BroadPhaseStats broadPhaseDetails{}; // Mean timings; other fields from the last step.
    std::size_t totalTreeInsertions = 0;
    std::size_t totalTreeRemovals = 0;
    std::size_t totalTreeReinsertions = 0;

    double narrowPhaseMeanMs = 0.0;
    double solverMeanMs = 0.0;
    double sphereSphereCandidatesPerStep = 0.0;
    double sphereBoxCandidatesPerStep = 0.0;
    double boxBoxCandidatesPerStep = 0.0;
    double sphereSphereContactsPerStep = 0.0;
    double sphereBoxContactsPerStep = 0.0;
    double boxBoxContactsPerStep = 0.0;
    double solverPrepareMeanMs = 0.0;
    double solverWarmStartMeanMs = 0.0;
    double solverVelocityIterationsMeanMs = 0.0;
    double solverCacheUpdateMeanMs = 0.0;
    double solverPreparedPointsPerStep = 0.0;
    double solverWarmStartComparisonsPerStep = 0.0;
    double solverWarmStartMatchesPerStep = 0.0;
    double solverVelocityPointVisitsPerStep = 0.0;
    double solverIslandsPerStep = 0.0;
    double solverLargestIslandContactsPerStep = 0.0;
    double solverLargestIslandPointsPerStep = 0.0;
    std::size_t solverMaxIslandContacts = 0;
    std::size_t solverMaxIslandPoints = 0;

    double allocationsPerStep = 0.0;
    std::size_t totalAllocations = 0;
    std::size_t maxAllocationsPerStep = 0;
    double meanContacts = 0.0;
    double meanContactPoints = 0.0;
    double meanCandidatePairs = 0.0;
    std::size_t minContacts = 0;
    std::size_t maxContacts = 0;
    float maxContactPenetration = 0.0f;
    float finalFloorPenetration = 0.0f;
    float finalMaxLinearSpeed = 0.0f;
    float finalMaxAngularSpeed = 0.0f;
    bool sleepingEnabled = false;
    std::size_t narrowPhaseWorkers = 1;
    std::size_t solverWorkers = 1;
    double meanAwakeBodies = 0.0;
    double meanSleepingBodies = 0.0;
    double meanSolvedContacts = 0.0;

    void print(std::ostream& out) const;
};

// bodyCount counts dynamic bodies; the mixed scene adds one static floor.
BenchmarkReport runBenchmark(int bodyCount, double simulationHz, unsigned seed,
    const ScalingOptions& options);

bool parseScalingOptions(int argc, char** argv, ScalingOptions& options);
int runScalingBenchmark(int argc, char** argv);
int parseSampleCount(int argc, char** argv, std::size_t defaultSamples);
bool parseBroadPhaseAlgorithm(int argc, char** argv, BroadPhaseAlgorithm& algorithm);

}

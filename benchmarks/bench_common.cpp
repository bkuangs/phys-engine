#include "bench_common.hpp"
#include "alloc_counter.hpp"
#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>

namespace phys::bench
{

    namespace
    {
        struct SceneHealth
        {
            float floorPenetration = 0;
            float linearSpeed = 0;
            float angularSpeed = 0;
        };

        SceneHealth checkScene(const BenchmarkScene &scene, const char *phase)
        {
            SceneHealth health;
            auto finite = [](Vec3 value) {
                return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
            };
            for (const BenchmarkObject &object : scene.objects)
            {
                const auto *body = scene.world.getBody(object.body);
                const auto *collider = scene.world.getCollider(object.collider);
                if (!body || !collider)
                    throw std::runtime_error("Benchmark lost a dynamic body or collider");
                Vec3 position = body->getPosition();
                float linear = Math3d::length(body->getLinearVelocity());
                float angular = Math3d::length(body->getAngularVelocity());
                if (!finite(position) || !std::isfinite(linear) || !std::isfinite(angular))
                    throw std::runtime_error(std::string("Non-finite benchmark state after ") + phase);
                Aabb bounds = Aabb::fromCollider(*collider, {position, body->getRotation()});
                if (!finite(bounds.min) || !finite(bounds.max) || !bounds.overlaps(bounds))
                    throw std::runtime_error(std::string("Invalid benchmark bounds after ") + phase);
                bool outside = bounds.max.x < -scene.floorSize.x * 0.5f || bounds.min.x > scene.floorSize.x * 0.5f
                    || bounds.max.z < -scene.floorSize.z * 0.5f || bounds.min.z > scene.floorSize.z * 0.5f;
                if (outside || bounds.max.y < -scene.floorSize.y)
                    throw std::runtime_error(std::string("Benchmark ") + (outside ? "escaped" : "fell below")
                        + " the floor: body " + std::to_string(object.body.index)
                        + " at (" + std::to_string(position.x) + ", " + std::to_string(position.y)
                        + ", " + std::to_string(position.z) + ") after " + phase);
                health.floorPenetration = std::max(health.floorPenetration, -bounds.min.y);
                health.linearSpeed = std::max(health.linearSpeed, linear);
                health.angularSpeed = std::max(health.angularSpeed, angular);
            }
            return health;
        }

        bool parseSteps(const char *text, const char *label, bool allowZero, std::size_t &result)
        {
            std::string_view input = text;
            auto [end, error] = std::from_chars(input.data(), input.data() + input.size(), result);
            if (error != std::errc{} || end != input.data() + input.size() || (!allowZero && result == 0))
            {
                std::cerr << label << " must be " << (allowZero ? "a nonnegative" : "a positive") << " integer\n";
                return false;
            }
            return true;
        }
    }

    BenchmarkReport runBenchmark(int bodyCount, double simulationHz, unsigned seed,
                                 const ScalingOptions &options)
    {
        if (bodyCount <= 0 || !std::isfinite(simulationHz) || simulationHz <= 0 || options.measuredSteps == 0)
            throw std::invalid_argument("Benchmark requires positive body count, frequency, and measured steps");
        float dt = static_cast<float>(1.0 / simulationHz);
        if (!std::isfinite(dt) || dt <= 0)
            throw std::invalid_argument("Benchmark timestep is outside the supported float range");
        BenchmarkScene scene;
        if (options.scene == ScalingScene::MixedFloor)
            scene = makeMixedField(bodyCount, seed);
        else if (options.scene == ScalingScene::Spheres)
        {
            scene.world = makeSphereField(bodyCount, seed);
            scene.spheres = static_cast<std::size_t>(bodyCount);
        }
        else
            throw std::invalid_argument("Unknown benchmark scene");
        PhysicsWorld &world = scene.world;
        world.broadPhaseAlgorithm = options.algorithm;
        double deadlineMs = 1000.0 / simulationHz;
        double warmupTotalMs = 0;
        double coldFirstStepMs = 0;
        for (std::size_t step = 0; step < options.warmupSteps; ++step)
        {
            world.step(dt);
            warmupTotalMs += world.lastStepStats().totalMs;
            if (step == 0)
                coldFirstStepMs = world.lastStepStats().totalMs;
        }
        if (options.scene == ScalingScene::MixedFloor)
            checkScene(scene, "warmup");

        DurationStats stepStats;
        DurationStats broadPhaseStats;
        DurationStats narrowPhaseStats;
        DurationStats solverStats;
        double broadPhaseCollectTotalMs = 0.0;
        double broadPhaseFilterTotalMs = 0.0;
        BroadPhaseStats broadPhaseTotals{};
        double firstStepMs = 0.0;
        std::size_t deadlineMisses = 0;
        std::size_t totalAllocations = 0;
        std::size_t totalContacts = 0;
        std::size_t totalContactPoints = 0;
        std::size_t totalCandidates = 0;
        std::size_t totalAwakeBodies = 0;
        std::size_t totalSleepingBodies = 0;
        std::size_t totalSolvedContacts = 0;
        std::size_t minContacts = std::numeric_limits<std::size_t>::max();
        std::size_t maxContacts = 0;
        float maxPenetration = 0;

        for (std::size_t i = 0; i < options.measuredSteps; ++i)
        {
            {
                ScopedAllocCounter allocs;
                world.step(dt);
                totalAllocations += allocs.count();
            }
            const StepStats &stats = world.lastStepStats();
            if (i == 0)
            {
                firstStepMs = stats.totalMs;
                if (options.warmupSteps == 0)
                    coldFirstStepMs = stats.totalMs;
            }

            stepStats.record(stats.totalMs);
            broadPhaseStats.record(stats.broadPhaseMs);
            narrowPhaseStats.record(stats.narrowPhaseMs);
            solverStats.record(stats.solverMs);
            broadPhaseCollectTotalMs += stats.broadPhaseCollectMs;
            broadPhaseFilterTotalMs += stats.broadPhaseFilterMs;
            broadPhaseTotals.recordBuildMs += stats.broadPhaseDetails.recordBuildMs;
            broadPhaseTotals.recordSortMs += stats.broadPhaseDetails.recordSortMs;
            broadPhaseTotals.sweepMs += stats.broadPhaseDetails.sweepMs;
            broadPhaseTotals.pairSortMs += stats.broadPhaseDetails.pairSortMs;
            broadPhaseTotals.treeInsertions += stats.broadPhaseDetails.treeInsertions;
            broadPhaseTotals.treeRemovals += stats.broadPhaseDetails.treeRemovals;
            broadPhaseTotals.treeReinsertions += stats.broadPhaseDetails.treeReinsertions;
            if (stats.totalMs > deadlineMs)
                ++deadlineMisses;
            totalContacts += stats.contactCount;
            totalCandidates += stats.candidatePairs;
            totalAwakeBodies += stats.awakeBodyCount;
            totalSleepingBodies += stats.sleepingBodyCount;
            totalSolvedContacts += stats.solvedContactCount;
            minContacts = std::min(minContacts, stats.contactCount);
            maxContacts = std::max(maxContacts, stats.contactCount);
            for (const auto &contact : world.contacts())
            {
                totalContactPoints += contact.pointCount;
                for (uint32_t point = 0; point < contact.pointCount; ++point)
                    maxPenetration = std::max(maxPenetration, contact.points[point].penetration);
            }
        }
        SceneHealth health;
        if (options.scene == ScalingScene::MixedFloor)
            health = checkScene(scene, "measurement");

        const StepStats &lastStats = world.lastStepStats();

        BenchmarkReport report;
        report.bodyCount = bodyCount;
        report.simulationHz = simulationHz;
        report.sampleCount = options.measuredSteps;
        report.algorithm = options.algorithm;
        report.scene = options.scene;
        report.warmupSteps = options.warmupSteps;
        report.warmupTotalMs = warmupTotalMs;
        report.coldFirstStepMs = coldFirstStepMs;
        report.staticBodies = options.scene == ScalingScene::MixedFloor ? 1 : 0;
        report.sphereBodies = scene.spheres;
        report.boxBodies = scene.boxes;
        report.floorSize = scene.floorSize;
        report.sleepingEnabled = world.isSleepingEnabled();
        report.stepTime = stepStats.summarize();
        report.firstStepMs = firstStepMs;
        report.deadlineMisses = deadlineMisses;
        report.possiblePairs = lastStats.possiblePairs;
        report.candidatePairs = lastStats.candidatePairs;
        report.broadPhaseMeanMs = broadPhaseStats.summarize().mean;
        report.broadPhaseDetails = lastStats.broadPhaseDetails;
        report.totalTreeInsertions = broadPhaseTotals.treeInsertions;
        report.totalTreeRemovals = broadPhaseTotals.treeRemovals;
        report.totalTreeReinsertions = broadPhaseTotals.treeReinsertions;
        {
            double samples = static_cast<double>(options.measuredSteps);
            report.broadPhaseCollectMeanMs = broadPhaseCollectTotalMs / samples;
            report.broadPhaseFilterMeanMs = broadPhaseFilterTotalMs / samples;
            report.broadPhaseDetails.recordBuildMs = broadPhaseTotals.recordBuildMs / samples;
            report.broadPhaseDetails.recordSortMs = broadPhaseTotals.recordSortMs / samples;
            report.broadPhaseDetails.sweepMs = broadPhaseTotals.sweepMs / samples;
            report.broadPhaseDetails.pairSortMs = broadPhaseTotals.pairSortMs / samples;
            report.meanContacts = totalContacts / samples;
            report.meanContactPoints = totalContactPoints / samples;
            report.meanCandidatePairs = totalCandidates / samples;
            report.meanAwakeBodies = totalAwakeBodies / samples;
            report.meanSleepingBodies = totalSleepingBodies / samples;
            report.meanSolvedContacts = totalSolvedContacts / samples;
        }
        report.narrowPhaseMeanMs = narrowPhaseStats.summarize().mean;
        report.solverMeanMs = solverStats.summarize().mean;
        report.allocationsPerStep = totalAllocations / options.measuredSteps;
        report.minContacts = minContacts;
        report.maxContacts = maxContacts;
        report.maxContactPenetration = maxPenetration;
        report.finalFloorPenetration = health.floorPenetration;
        report.finalMaxLinearSpeed = health.linearSpeed;
        report.finalMaxAngularSpeed = health.angularSpeed;
        return report;
    }

    bool parseScalingOptions(int argc, char **argv, ScalingOptions &options)
    {
        options = {};
        if (argc > 5)
        {
            std::cerr << "Usage: benchmark [measured_steps=1200] [sap|grid|tree] [mixed|spheres] [warmup_steps=240]\n";
            return false;
        }
        if (argc > 1 && !parseSteps(argv[1], "Measured steps", false, options.measuredSteps))
            return false;
        if (!parseBroadPhaseAlgorithm(argc, argv, options.algorithm))
            return false;
        if (argc > 3)
        {
            std::string_view name = argv[3];
            if (name == "spheres")
                options.scene = ScalingScene::Spheres;
            else if (name != "mixed")
            {
                std::cerr << "Scene must be mixed or spheres\n";
                return false;
            }
        }
        return argc < 5 || parseSteps(argv[4], "Warmup steps", true, options.warmupSteps);
    }

    int runScalingBenchmark(int argc, char **argv)
    {
        if (argc == 2 && (std::string_view(argv[1]) == "--help" || std::string_view(argv[1]) == "-h"))
        {
            std::cout << "Usage: benchmark [measured_steps=1200] [sap|grid|tree] [mixed|spheres] [warmup_steps=240]\n"
                         "Step counts are exact for every size; body counts exclude the static floor.\n";
            return 0;
        }
        ScalingOptions options;
        if (!parseScalingOptions(argc, argv, options))
            return 1;
        try
        {
            for (int bodyCount : {100, 500, 1000, 2500, 5000, 10000})
            {
                std::cerr << "Benchmarking " << bodyCount << " dynamic bodies: "
                          << options.warmupSteps << " warmup + " << options.measuredSteps << " measured steps\n";
                runBenchmark(bodyCount, 120.0, 42, options).print(std::cout);
                std::cout.flush();
            }
        }
        catch (const std::runtime_error &error)
        {
            std::cerr << "Benchmark failed: " << error.what() << '\n';
            return 1;
        }
        catch (const std::invalid_argument &error)
        {
            std::cerr << "Benchmark failed: " << error.what() << '\n';
            return 1;
        }
        return 0;
    }

    int parseSampleCount(int argc, char **argv, std::size_t defaultSamples)
    {
        if (argc < 2)
            return static_cast<int>(defaultSamples);

        char *end = nullptr;
        long parsed = std::strtol(argv[1], &end, 10);
        if (end == argv[1] || *end != '\0' || parsed <= 0)
        {
            return static_cast<int>(defaultSamples);
        }
        return static_cast<int>(parsed);
    }

    void BenchmarkReport::print(std::ostream &out) const
    {
        bool grid = algorithm == BroadPhaseAlgorithm::UniformGrid;
        bool tree = algorithm == BroadPhaseAlgorithm::DynamicTree;
        out << std::fixed << std::setprecision(2);
        out << "Dynamic bodies:             " << bodyCount << "\n";
        out << "Static bodies:              " << staticBodies << "\n";
        out << "Dynamic shapes:             " << sphereBodies << " spheres, " << boxBodies << " boxes\n";
        out << "Scene:                      " << (scene == ScalingScene::MixedFloor ? "mixed-floor" : "sphere field") << "\n";
        out << "Sleeping:                   " << (sleepingEnabled ? "enabled" : "disabled") << "\n";
        out << "Simulation frequency:       " << simulationHz << " Hz\n";
        out << "Broadphase algorithm:       "
            << (tree ? "dynamic AABB tree" : grid ? "uniform grid" : "sweep-and-prune") << "\n\n";

        out << "Warmup steps:               " << warmupSteps << " (" << warmupSteps / simulationHz << " simulated seconds)\n";
        out << "Measured steps:             " << sampleCount << " (" << sampleCount / simulationHz << " simulated seconds)\n";
        out << "Cold first step:            " << coldFirstStepMs << " ms\n";
        out << "Warmup step-time total:     " << warmupTotalMs << " ms\n\n";
        if (staticBodies != 0)
            out << "Floor dimensions:           " << floorSize.x << " x " << floorSize.y << " x " << floorSize.z << "\n\n";

        out << "Step time (measured interval):\n";
        out << "    first:                   " << firstStepMs << " ms\n";
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
        out << "    mean candidate pairs:    " << meanCandidatePairs << "\n";
        out << "    broadphase time:         " << broadPhaseMeanMs << " ms\n\n";

        out << "Broadphase detail (mean timings):\n" << std::setprecision(4);
        out << "    collect active bounds:   " << broadPhaseCollectMeanMs << " ms\n";
        out << (tree ? "    maintain tree:           "
                     : grid ? "    build grid entries:      " : "    build sweep records:     ")
            << broadPhaseDetails.recordBuildMs << " ms\n";
        if (!tree)
            out << (grid ? "    sort grid entries:       " : "    sort sweep records:      ")
                << broadPhaseDetails.recordSortMs << " ms\n";
        out << (tree ? "    query tree:              "
                     : grid ? "    grid scan + overflow:    " : "    sweep + emit pairs:      ")
            << broadPhaseDetails.sweepMs << " ms\n";
        out << (grid ? "    sort/deduplicate pairs:  " : "    sort output pairs:       ")
            << broadPhaseDetails.pairSortMs << " ms\n";
        out << "    map/filter pairs:        " << broadPhaseFilterMeanMs << " ms\n\n";
        out << std::setprecision(2);
        out << "Broadphase work (last step):\n";
        if (tree)
        {
            out << "    tree proxies:            " << broadPhaseDetails.treeProxyCount << "\n";
            out << "    tree height:             " << broadPhaseDetails.treeHeight << "\n";
            out << "    tree node-pair visits:   " << broadPhaseDetails.treeNodePairVisits << "\n";
            out << "    tight leaf checks:       " << broadPhaseDetails.treeLeafChecks << "\n";
            out << "    proxy reinsertions:      " << broadPhaseDetails.treeReinsertions << "\n";
        }
        else if (grid)
        {
            out << "    grid pair comparisons:   " << broadPhaseDetails.gridComparisons << "\n";
            out << "    cell entries:            " << broadPhaseDetails.gridEntries << "\n";
            out << "    overflow AABBs:          " << broadPhaseDetails.gridOverflowAabbs << "\n";
            out << "    cell width:              " << std::setprecision(6)
                << broadPhaseDetails.gridCellSize << std::setprecision(2) << "\n";
        }
        else
            out << "    X-window comparisons:    " << broadPhaseDetails.xWindowComparisons << "\n";
        out << "    AABB pairs (pre-filter):  " << broadPhaseDetails.aabbPairs << "\n\n";
        if (tree)
        {
            out << "Tree maintenance work (measured interval):\n";
            out << "    insertions:              " << totalTreeInsertions << "\n";
            out << "    removals:                " << totalTreeRemovals << "\n";
            out << "    reinsertions:            " << totalTreeReinsertions << "\n\n";
        }

        out << "Narrowphase time:            " << narrowPhaseMeanMs << " ms\n";
        out << "Solver time:                 " << solverMeanMs << " ms\n\n";

        out << "Engine heap allocations / step: " << allocationsPerStep << "\n\n";
        out << "Contact workload (measured interval):\n";
        out << "    mean manifolds:          " << meanContacts << "\n";
        out << "    min/max manifolds:       " << minContacts << " / " << maxContacts << "\n";
        out << "    mean contact points:     " << meanContactPoints << "\n";
        out << "    mean solved manifolds:   " << meanSolvedContacts << "\n";
        out << "    mean awake bodies:       " << meanAwakeBodies << "\n";
        out << "    mean sleeping bodies:    " << meanSleepingBodies << "\n";
        out << "    max penetration:         " << std::setprecision(4) << maxContactPenetration << "\n\n";
        if (staticBodies != 0)
        {
            out << "Scene health (after warmup and measurement): valid, no escaped/below-floor bodies\n";
            out << "    final floor penetration: " << finalFloorPenetration << "\n";
            out << "    final max linear speed:  " << finalMaxLinearSpeed << "\n";
            out << "    final max angular speed: " << finalMaxAngularSpeed << "\n\n";
        }
        out << std::setprecision(2);
    }

    bool parseBroadPhaseAlgorithm(int argc, char **argv, BroadPhaseAlgorithm &algorithm)
    {
        algorithm = BroadPhaseAlgorithm::SweepAndPrune;
        if (argc < 3 || std::string(argv[2]) == "sap")
            return true;
        if (std::string(argv[2]) == "grid")
        {
            algorithm = BroadPhaseAlgorithm::UniformGrid;
            return true;
        }
        if (std::string(argv[2]) == "tree")
        {
            algorithm = BroadPhaseAlgorithm::DynamicTree;
            return true;
        }
        std::cerr << "Unknown broadphase '" << argv[2] << "': expected sap, grid, or tree\n";
        return false;
    }

}

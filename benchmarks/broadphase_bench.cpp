#include <iostream>
#include "bench_common.hpp"

int main(int argc, char** argv)
{
    const std::size_t requestedSamples = phys::bench::parseSampleCount(argc, argv, 1000);
    // Broad-phase and narrow-phase currently run as one fused sphere pipeline
    // (see PhysicsWorld::step()); this reports the broad-phase-facing stats
    // from that same run until the stages are split into independent APIs.
    for (int bodyCount : {100, 500, 1000, 2500, 5000, 10000}) {
        const std::size_t samples = phys::bench::sampleCountForBodies(bodyCount, requestedSamples);
        phys::bench::BenchmarkReport report =
            phys::bench::runBenchmark(bodyCount, 120.0, samples, 42);
        report.print(std::cout);
    }

    return 0;
}

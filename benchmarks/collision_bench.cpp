#include <iostream>
#include "bench_common.hpp"

int main(int argc, char** argv)
{
    const std::size_t requestedSamples = phys::bench::parseSampleCount(argc, argv, 1000);
    phys::BroadPhaseAlgorithm algorithm;
    if (!phys::bench::parseBroadPhaseAlgorithm(argc, argv, algorithm))
        return 1;
    for (int bodyCount : {100, 500, 1000, 2500, 5000, 10000}) {
        const std::size_t samples = phys::bench::sampleCountForBodies(bodyCount, requestedSamples);
        phys::bench::BenchmarkReport report =
            phys::bench::runBenchmark(bodyCount, 120.0, samples, 42, algorithm);
        report.print(std::cout);
    }

    return 0;
}

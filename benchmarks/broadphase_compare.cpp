#include "alloc_counter.hpp"
#include "bench_common.hpp"
#include <phys/collision/broadphase.hpp>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <random>

namespace {

enum class Layout { Uniform, Clustered, MixedFloor, WideSizes };

std::vector<phys::Aabb> makeBounds(Layout layout, std::size_t count)
{
    std::mt19937 rng(42);
    float side = std::cbrt(static_cast<float>(count)) * 2.0f;
    std::uniform_real_distribution<float> position(-side * 0.5f, side * 0.5f);
    std::uniform_real_distribution<float> size(0.3f, 2.0f);
    std::uniform_real_distribution<float> logSize(std::log(0.1f), std::log(6.0f));
    std::uniform_real_distribution<float> angle(-3.14159f, 3.14159f);
    std::vector<phys::Aabb> bounds;
    bounds.reserve(count);
    for (std::size_t index = 0; index < count; ++index) {
        phys::Vec3 center{position(rng), position(rng), position(rng)};
        if (layout == Layout::Clustered) {
            center = center * 0.2f;
            center += {((index & 1) ? 1.0f : -1.0f) * side * 0.25f,
                       ((index & 2) ? 1.0f : -1.0f) * side * 0.25f,
                       ((index & 4) ? 1.0f : -1.0f) * side * 0.25f};
        }
        if (layout == Layout::MixedFloor || layout == Layout::WideSizes) {
            phys::Vec3 halfExtents{size(rng) * 0.5f, size(rng) * 0.5f, size(rng) * 0.5f};
            if (layout == Layout::WideSizes) {
                float halfSize = std::exp(logSize(rng)) * 0.5f;
                halfExtents = {halfSize, halfSize, halfSize};
            }
            phys::Collider collider;
            collider.shape = phys::Box{halfExtents};
            phys::Transform transform{
                center, phys::Quaternion::fromAxisAngle({0, 1, 0}, angle(rng))};
            bounds.push_back(phys::Aabb::fromCollider(collider, transform));
        }
        else
            bounds.push_back({center - phys::Vec3{0.5f, 0.5f, 0.5f},
                              center + phys::Vec3{0.5f, 0.5f, 0.5f}});
    }
    if (layout == Layout::MixedFloor)
        bounds[0] = {{-side, -0.5f, -side}, {side, 0.0f, side}};
    return bounds;
}

bool compare(Layout layout, const char* name, std::size_t count, std::size_t samples)
{
    auto bounds = makeBounds(layout, count);
    auto expected = phys::BroadPhase::findCandidatePairs(bounds);
    std::array<phys::bench::DurationStats, 2> times;
    std::array<std::size_t, 2> allocations{};
    std::array<phys::BroadPhaseStats, 2> stats{};
    std::array algorithms{phys::BroadPhaseAlgorithm::SweepAndPrune,
                          phys::BroadPhaseAlgorithm::UniformGrid};
    for (std::size_t iteration = 0; iteration < samples; ++iteration) {
        for (std::size_t offset = 0; offset < 2; ++offset) {
            std::size_t algorithm = (iteration + offset) % 2;
            std::vector<phys::BroadPhasePair> actual;
            double elapsed;
            {
                phys::bench::ScopedAllocCounter counter;
                auto start = std::chrono::steady_clock::now();
                actual = phys::BroadPhase::findCandidatePairs(
                    bounds, &stats[algorithm], algorithms[algorithm]);
                elapsed = std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - start).count();
                allocations[algorithm] += counter.count();
            }
            times[algorithm].record(elapsed);
            if (actual.size() != expected.size()
                || !std::equal(actual.begin(), actual.end(), expected.begin(),
                    [](const auto& left, const auto& right) {
                        return left.first == right.first && left.second == right.second;
                    })) {
                std::cerr << "pair mismatch for " << name << ", " << count << " AABBs\n";
                return false;
            }
        }
    }
    for (std::size_t algorithm = 0; algorithm < 2; ++algorithm) {
        const auto summary = times[algorithm].summarize();
        const auto& work = stats[algorithm];
        std::cout << name << ',' << count << ',' << (algorithm ? "grid" : "sap")
                  << ',' << samples << ',' << summary.mean << ',' << summary.p95
                  << ',' << allocations[algorithm] / samples
                  << ',' << (algorithm ? work.gridComparisons : work.xWindowComparisons)
                  << ',' << work.aabbPairs << ',' << work.gridEntries
                  << ',' << work.gridOverflowAabbs << ',' << work.gridCellSize << '\n';
    }
    return true;
}

}

int main(int argc, char** argv)
{
    const auto samples = static_cast<std::size_t>(phys::bench::parseSampleCount(argc, argv, 10));
    std::cout << "Standalone broadphase queries; fixed seed 42; static bounds; alternating SAP/grid\n"
              << "Every result is compared with SAP outside the timed region.\n"
              << "layout,aabbs,algorithm,samples,mean_ms,p95_ms,allocations,pair_checks,emitted_pairs,cell_entries,overflow_aabbs,cell_width\n"
              << std::fixed << std::setprecision(4);
    for (std::size_t count : {1000, 10000}) {
        if (!compare(Layout::Uniform, "uniform", count, samples)
            || !compare(Layout::Clustered, "clustered", count, samples)
            || !compare(Layout::MixedFloor, "mixed-floor", count, samples)
            || !compare(Layout::WideSizes, "wide-sizes", count, samples))
            return 1;
    }
    return 0;
}

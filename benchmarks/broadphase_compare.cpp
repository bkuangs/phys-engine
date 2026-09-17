#include "alloc_counter.hpp"
#include "bench_common.hpp"
#include <phys/collision/broadphase.hpp>
#include <phys/collision/dynamic_aabb_tree.hpp>
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

bool compare(Layout layout, const char* name, std::size_t count, std::size_t samples, bool moving)
{
    auto bounds = makeBounds(layout, count);
    std::array<phys::bench::DurationStats, 3> times;
    std::array<std::size_t, 3> allocations{};
    std::array<double, 3> firstTimes{};
    std::array<phys::BroadPhaseStats, 3> stats{};
    std::array algorithms{phys::BroadPhaseAlgorithm::SweepAndPrune,
                          phys::BroadPhaseAlgorithm::UniformGrid,
                          phys::BroadPhaseAlgorithm::DynamicTree};
    std::array names{"sap", "grid", "tree"};
    phys::DynamicAabbTree tree;
    std::vector<phys::DynamicAabbTree::ProxyId> proxies(count, phys::DynamicAabbTree::noProxy);
    std::size_t totalInsertions = 0;
    std::size_t totalReinsertions = 0;
    for (std::size_t iteration = 0; iteration < samples; ++iteration) {
        if (moving && iteration > 0) {
            for (std::size_t index = 0; index < bounds.size(); ++index) {
                if (layout == Layout::MixedFloor && index == 0)
                    continue;
                phys::Vec3 motion{
                    static_cast<float>(static_cast<int>(index % 7) - 3) * 0.015f,
                    static_cast<float>(static_cast<int>(index % 5) - 2) * 0.01f,
                    static_cast<float>(static_cast<int>(index % 3) - 1) * 0.02f};
                bounds[index].min += motion;
                bounds[index].max += motion;
                if (iteration == samples / 2 && index % 25 == 0) {
                    phys::Vec3 center = (bounds[index].min + bounds[index].max) * 0.5f;
                    phys::Vec3 halfExtents = (bounds[index].max - bounds[index].min) * 0.6f;
                    bounds[index] = {center - halfExtents, center + halfExtents};
                }
            }
        }
        auto expected = phys::BroadPhase::findCandidatePairs(bounds);
        for (std::size_t offset = 0; offset < algorithms.size(); ++offset) {
            std::size_t algorithm = (iteration + offset) % algorithms.size();
            std::vector<phys::BroadPhasePair> actual;
            double elapsed;
            {
                phys::bench::ScopedAllocCounter counter;
                auto start = std::chrono::steady_clock::now();
                if (algorithms[algorithm] == phys::BroadPhaseAlgorithm::DynamicTree) {
                    std::size_t insertions = 0;
                    std::size_t reinsertions = 0;
                    for (std::size_t index = 0; index < count; ++index) {
                        if (proxies[index] == phys::DynamicAabbTree::noProxy) {
                            proxies[index] = tree.createProxy(bounds[index], index);
                            ++insertions;
                        }
                        else if (tree.updateProxy(proxies[index], bounds[index]))
                            ++reinsertions;
                    }
                    double maintenanceMs = std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - start).count();
                    actual = tree.findCandidatePairs(&stats[algorithm]);
                    stats[algorithm].recordBuildMs = maintenanceMs;
                    totalInsertions += insertions;
                    totalReinsertions += reinsertions;
                }
                else
                    actual = phys::BroadPhase::findCandidatePairs(
                        bounds, &stats[algorithm], algorithms[algorithm]);
                elapsed = std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - start).count();
                allocations[algorithm] += counter.count();
            }
            times[algorithm].record(elapsed);
            if (iteration == 0)
                firstTimes[algorithm] = elapsed;
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
    for (std::size_t algorithm = 0; algorithm < algorithms.size(); ++algorithm) {
        const auto summary = times[algorithm].summarize();
        const auto& work = stats[algorithm];
        bool dynamic = algorithms[algorithm] == phys::BroadPhaseAlgorithm::DynamicTree;
        std::cout << name << ',' << (moving ? "moving" : "static") << ',' << count << ',' << names[algorithm]
                  << ',' << samples << ',' << summary.mean << ',' << summary.p50 << ',' << summary.p95
                  << ',' << firstTimes[algorithm]
                  << ',' << allocations[algorithm] / samples
                  << ',' << (dynamic ? work.treeLeafChecks : algorithm ? work.gridComparisons : work.xWindowComparisons)
                  << ',' << work.aabbPairs << ',' << work.gridEntries
                  << ',' << work.gridOverflowAabbs << ',' << work.gridCellSize
                  << ',' << work.treeNodePairVisits << ',' << work.treeHeight
                  << ',' << (dynamic ? totalInsertions : 0)
                  << ',' << (dynamic ? totalReinsertions : 0) << '\n';
    }
    return true;
}

}

int main(int argc, char** argv)
{
    const auto samples = static_cast<std::size_t>(phys::bench::parseSampleCount(argc, argv, 10));
    std::cout << "Standalone maintenance + queries; fixed seed 42; interleaved SAP/grid/tree\n"
              << "Tree construction is included in the first sample; tree persists across samples.\n"
              << "Every result is compared with SAP outside the timed region.\n"
              << "layout,motion,aabbs,algorithm,samples,mean_ms,p50_ms,p95_ms,first_ms,allocations,pair_checks,emitted_pairs,cell_entries,overflow_aabbs,cell_width,node_pair_visits,tree_height,insertions,reinsertions\n"
              << std::fixed << std::setprecision(4);
    for (std::size_t count : {1000, 10000}) {
        for (bool moving : {false, true}) {
            if (!compare(Layout::Uniform, "uniform", count, samples, moving)
                || !compare(Layout::Clustered, "clustered", count, samples, moving)
                || !compare(Layout::MixedFloor, "mixed-floor", count, samples, moving)
                || !compare(Layout::WideSizes, "wide-sizes", count, samples, moving))
                return 1;
        }
    }
    return 0;
}

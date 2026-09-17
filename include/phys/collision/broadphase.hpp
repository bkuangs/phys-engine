#pragma once
#include <cstddef>
#include <vector>
#include "phys/collision/aabb.hpp"

namespace phys {

enum class BroadPhaseAlgorithm
{
    SweepAndPrune,
    UniformGrid,
    DynamicTree
};

struct BroadPhasePair
{
    std::size_t first;
    std::size_t second;
};

struct BroadPhaseStats
{
    // Grid uses these phases for entry construction, cell sorting, scanning, and deduplication.
    // Tree uses recordBuildMs for maintenance and sweepMs for paired traversal.
    double recordBuildMs = 0.0;
    double recordSortMs = 0.0;
    double sweepMs = 0.0;
    double pairSortMs = 0.0;
    std::size_t xWindowComparisons = 0; // X-overlapping pairs, excluding terminating probes.
    std::size_t aabbPairs = 0; // Full AABB overlaps, before world-level body filtering.
    std::size_t gridComparisons = 0; // Includes repeated cell pairs and overflow checks.
    std::size_t gridEntries = 0;
    std::size_t gridOverflowAabbs = 0; // Exceeded cell-count or cell-coordinate limits.
    double gridCellSize = 0.0;
    std::size_t treeInsertions = 0;
    std::size_t treeRemovals = 0;
    std::size_t treeReinsertions = 0;
    std::size_t treeNodePairVisits = 0; // Includes same-subtree decomposition visits.
    std::size_t treeLeafChecks = 0;
    std::size_t treeHeight = 0;
    std::size_t treeProxyCount = 0;
};

class BroadPhase
{
public:
    // Single-threaded broadphase, including touching AABBs; SAP is the default.
    // Bounds must be finite and ordered; zero-width bounds are supported.
    // Returns lexicographically ordered input-index pairs with first < second.
    // When supplied, stats is overwritten for this call.
    // DynamicTree requires a persistent DynamicAabbTree instance instead.
    static std::vector<BroadPhasePair> findCandidatePairs(
        const std::vector<Aabb>& bounds, BroadPhaseStats* stats = nullptr,
        BroadPhaseAlgorithm algorithm = BroadPhaseAlgorithm::SweepAndPrune);
};

}

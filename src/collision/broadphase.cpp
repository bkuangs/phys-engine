#include <phys/collision/broadphase.hpp>
#include "../world/step_workspace.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace phys {

namespace {

using Clock = std::chrono::steady_clock;
using Milliseconds = std::chrono::duration<double, std::milli>;

void sortPairs(std::vector<BroadPhasePair>& pairs)
{
    std::sort(pairs.begin(), pairs.end(), [](const auto& left, const auto& right) {
        if (left.first != right.first)
            return left.first < right.first;
        return left.second < right.second;
    });
}

bool gridRange(const Aabb& bounds, double cellSize,
               std::array<int32_t, 3>& firstCell, std::array<int, 3>& cellCounts)
{
    constexpr std::size_t maxCellsPerAabb = 64;
    double lower[] = {std::floor(bounds.min.x / cellSize),
                      std::floor(bounds.min.y / cellSize),
                      std::floor(bounds.min.z / cellSize)};
    double upper[] = {std::floor(bounds.max.x / cellSize),
                      std::floor(bounds.max.y / cellSize),
                      std::floor(bounds.max.z / cellSize)};
    std::size_t totalCells = 1;
    for (int axis = 0; axis < 3; ++axis) {
        if (!std::isfinite(lower[axis]) || !std::isfinite(upper[axis])
            || lower[axis] < std::numeric_limits<int32_t>::min()
            || upper[axis] > std::numeric_limits<int32_t>::max()
            || lower[axis] > upper[axis])
            return false;
        int64_t count = static_cast<int64_t>(upper[axis])
                      - static_cast<int64_t>(lower[axis]) + 1;
        if (count > static_cast<int64_t>(maxCellsPerAabb / totalCells))
            return false;
        firstCell[axis] = static_cast<int32_t>(lower[axis]);
        cellCounts[axis] = static_cast<int>(count);
        totalCells *= static_cast<std::size_t>(count);
    }
    return true;
}

void findGridPairs(const std::vector<Aabb>& bounds, std::vector<BroadPhasePair>& pairs,
                   BroadPhaseStats* stats, detail::BroadPhaseWorkspace& workspace)
{
    auto recordBuildStart = Clock::now();
    auto& widths = workspace.gridWidths;
    widths.clear();
    if (widths.capacity() < bounds.size())
        widths.reserve(bounds.size());
    for (const Aabb& box : bounds) {
        double width = std::max({static_cast<double>(box.max.x) - box.min.x,
                                static_cast<double>(box.max.y) - box.min.y,
                                static_cast<double>(box.max.z) - box.min.z});
        if (width > 0.0 && std::isfinite(width))
            widths.push_back(width);
    }
    double cellSize = 1.0;
    if (!widths.empty()) {
        auto median = widths.begin() + widths.size() / 2;
        std::nth_element(widths.begin(), median, widths.end());
        cellSize = *median;
    }

    auto& entries = workspace.gridEntries;
    entries.clear();
    if (entries.capacity() < bounds.size() * 8)
        entries.reserve(bounds.size() * 8);
    auto& inGrid = workspace.gridMembership;
    inGrid.assign(bounds.size(), 0);
    auto& overflow = workspace.gridOverflow;
    overflow.clear();
    if (overflow.capacity() < bounds.size())
        overflow.reserve(bounds.size());
    for (std::size_t index = 0; index < bounds.size(); ++index) {
        std::array<int32_t, 3> firstCell{};
        std::array<int, 3> cellCounts{};
        if (!gridRange(bounds[index], cellSize, firstCell, cellCounts)) {
            overflow.push_back(index);
            continue;
        }
        inGrid[index] = 1;
        for (int x = 0; x < cellCounts[0]; ++x)
            for (int y = 0; y < cellCounts[1]; ++y)
                for (int z = 0; z < cellCounts[2]; ++z)
                    entries.push_back({{firstCell[0] + x, firstCell[1] + y,
                                        firstCell[2] + z}, index});
    }

    auto recordSortStart = Clock::now();
    std::sort(entries.begin(), entries.end(), [](const auto& left, const auto& right) {
        if (left.cell != right.cell)
            return left.cell < right.cell;
        return left.index < right.index;
    });

    auto scanStart = Clock::now();
    std::size_t comparisons = 0;
    pairs.clear();
    auto checkPair = [&](std::size_t first, std::size_t second) {
        ++comparisons;
        if (bounds[first].overlaps(bounds[second]))
            pairs.push_back({std::min(first, second), std::max(first, second)});
    };
    for (std::size_t begin = 0; begin < entries.size();) {
        std::size_t end = begin + 1;
        while (end < entries.size() && entries[end].cell == entries[begin].cell)
            ++end;
        for (std::size_t first = begin; first < end; ++first)
            for (std::size_t second = first + 1; second < end; ++second)
                checkPair(entries[first].index, entries[second].index);
        begin = end;
    }

    // Large/out-of-range AABBs bypass cell expansion, not collision detection.
    for (std::size_t first : overflow)
        for (std::size_t second = 0; second < bounds.size(); ++second)
            if (first != second && (inGrid[second] || first < second))
                checkPair(first, second);

    auto pairSortStart = Clock::now();
    sortPairs(pairs);
    pairs.erase(std::unique(pairs.begin(), pairs.end(), [](const auto& left, const auto& right) {
        return left.first == right.first && left.second == right.second;
    }), pairs.end());
    if (stats) {
        auto pairSortEnd = Clock::now();
        *stats = {
            Milliseconds(recordSortStart - recordBuildStart).count(),
            Milliseconds(scanStart - recordSortStart).count(),
            Milliseconds(pairSortStart - scanStart).count(),
            Milliseconds(pairSortEnd - pairSortStart).count(),
            0, pairs.size(), comparisons, entries.size(), overflow.size(), cellSize};
    }
}

}

std::vector<BroadPhasePair> BroadPhase::findCandidatePairs(
    const std::vector<Aabb>& bounds, BroadPhaseStats* stats, BroadPhaseAlgorithm algorithm)
{
    std::vector<BroadPhasePair> pairs;
    detail::BroadPhaseWorkspace workspace;
    detail::findCandidatePairs(bounds, pairs, stats, algorithm, workspace);
    return pairs;
}

void detail::findCandidatePairs(const std::vector<Aabb>& bounds,
    std::vector<BroadPhasePair>& pairs, BroadPhaseStats* stats,
    BroadPhaseAlgorithm algorithm, BroadPhaseWorkspace& workspace)
{
    if (algorithm == BroadPhaseAlgorithm::DynamicTree)
        throw std::invalid_argument("DynamicTree requires a persistent DynamicAabbTree instance");
    if (algorithm == BroadPhaseAlgorithm::UniformGrid)
        return findGridPairs(bounds, pairs, stats, workspace);

    auto recordBuildStart = Clock::now();
    auto& entries = workspace.sweepEntries;
    entries.clear();
    if (entries.capacity() < bounds.size())
        entries.reserve(bounds.size());
    for (std::size_t index = 0; index < bounds.size(); ++index)
        entries.push_back({bounds[index], index});
    auto recordSortStart = Clock::now();
    std::sort(entries.begin(), entries.end(), [](const auto& left, const auto& right) {
        if (left.bounds.min.x != right.bounds.min.x)
            return left.bounds.min.x < right.bounds.min.x;
        return left.originalIndex < right.originalIndex;
    });

    auto sweepStart = Clock::now();
    pairs.clear();
    std::size_t xWindowComparisons = 0;
    for (std::size_t i = 0; i < entries.size(); ++i) {
        const SweepEntry& first = entries[i];
        std::size_t j = i + 1;
        for (; j < entries.size(); ++j) {
            const SweepEntry& second = entries[j];
            if (second.bounds.min.x > first.bounds.max.x)
                break;
            // Sorted minima and the break condition already guarantee X overlap.
            if (first.bounds.min.y <= second.bounds.max.y
                && first.bounds.max.y >= second.bounds.min.y
                && first.bounds.min.z <= second.bounds.max.z
                && first.bounds.max.z >= second.bounds.min.z)
                pairs.push_back({std::min(first.originalIndex, second.originalIndex),
                                 std::max(first.originalIndex, second.originalIndex)});
        }
        xWindowComparisons += j - i - 1;
    }

    auto pairSortStart = Clock::now();
    // Preserve the original all-pairs traversal order for the contact solver.
    sortPairs(pairs);
    if (stats) {
        auto pairSortEnd = Clock::now();
        *stats = {
            Milliseconds(recordSortStart - recordBuildStart).count(),
            Milliseconds(sweepStart - recordSortStart).count(),
            Milliseconds(pairSortStart - sweepStart).count(),
            Milliseconds(pairSortEnd - pairSortStart).count(),
            xWindowComparisons,
            pairs.size()};
    }
}

}

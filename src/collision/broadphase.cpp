#include <phys/collision/broadphase.hpp>
#include <algorithm>

namespace phys {

std::vector<BroadPhasePair> BroadPhase::findCandidatePairs(const std::vector<Aabb>& bounds)
{
    struct SweepEntry
    {
        Aabb bounds;
        std::size_t originalIndex;
    };

    std::vector<SweepEntry> entries;
    entries.reserve(bounds.size());
    for (std::size_t index = 0; index < bounds.size(); ++index)
        entries.push_back({bounds[index], index});
    std::sort(entries.begin(), entries.end(), [](const auto& left, const auto& right) {
        if (left.bounds.min.x != right.bounds.min.x)
            return left.bounds.min.x < right.bounds.min.x;
        return left.originalIndex < right.originalIndex;
    });

    std::vector<BroadPhasePair> pairs;
    for (std::size_t i = 0; i < entries.size(); ++i) {
        const SweepEntry& first = entries[i];
        for (std::size_t j = i + 1; j < entries.size(); ++j) {
            const SweepEntry& second = entries[j];
            if (second.bounds.min.x > first.bounds.max.x)
                break;
            if (first.bounds.overlaps(second.bounds))
                pairs.push_back({std::min(first.originalIndex, second.originalIndex),
                                 std::max(first.originalIndex, second.originalIndex)});
        }
    }

    // Preserve the original all-pairs traversal order for the contact solver.
    std::sort(pairs.begin(), pairs.end(), [](const auto& left, const auto& right) {
        if (left.first != right.first)
            return left.first < right.first;
        return left.second < right.second;
    });
    return pairs;
}

}

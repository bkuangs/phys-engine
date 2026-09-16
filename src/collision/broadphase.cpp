#include <phys/collision/broadphase.hpp>
#include <algorithm>
#include <numeric>

namespace phys {

std::vector<BroadPhasePair> BroadPhase::findCandidatePairs(const std::vector<Aabb>& bounds)
{
    std::vector<std::size_t> order(bounds.size());
    std::iota(order.begin(), order.end(), std::size_t{0});
    std::sort(order.begin(), order.end(), [&](std::size_t left, std::size_t right) {
        if (bounds[left].min.x != bounds[right].min.x)
            return bounds[left].min.x < bounds[right].min.x;
        return left < right;
    });

    std::vector<BroadPhasePair> pairs;
    for (std::size_t i = 0; i < order.size(); ++i) {
        std::size_t first = order[i];
        for (std::size_t j = i + 1; j < order.size(); ++j) {
            std::size_t second = order[j];
            if (bounds[second].min.x > bounds[first].max.x)
                break;
            if (bounds[first].overlaps(bounds[second]))
                pairs.push_back({std::min(first, second), std::max(first, second)});
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

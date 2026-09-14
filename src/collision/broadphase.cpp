#include <phys/collision/broadphase.hpp>

namespace phys {

std::vector<BroadPhasePair> BroadPhase::findCandidatePairs(const std::vector<Aabb>& bounds)
{
    std::vector<BroadPhasePair> pairs;

    for (std::size_t i = 0; i < bounds.size(); ++i) {
        for (std::size_t j = i + 1; j < bounds.size(); ++j) {
            if (bounds[i].overlaps(bounds[j])) {
                pairs.push_back({i, j});
            }
        }
    }

    return pairs;
}

}

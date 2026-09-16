#pragma once
#include <cstddef>
#include <vector>
#include "phys/collision/aabb.hpp"

namespace phys {

struct BroadPhasePair
{
    std::size_t first;
    std::size_t second;
};

class BroadPhase
{
public:
    // Single-threaded X-axis sweep-and-prune, including touching AABBs.
    // Returns lexicographically ordered input-index pairs with first < second.
    static std::vector<BroadPhasePair> findCandidatePairs(const std::vector<Aabb>& bounds);
};

}

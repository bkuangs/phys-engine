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
    // Naive O(n^2) all-pairs AABB overlap; kept as the benchmark baseline.
    // TODO: replace/augment with a BVH (or grid / sweep-and-prune) once
    // profiling shows all-pairs no longer scales for the target body counts.
    static std::vector<BroadPhasePair> findCandidatePairs(const std::vector<Aabb>& bounds);
};

}

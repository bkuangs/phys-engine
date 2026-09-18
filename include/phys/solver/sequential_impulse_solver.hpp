#pragma once
#include <cstddef>
#include <vector>
#include "phys/collision/manifold.hpp"

namespace phys {

class PhysicsWorld;
namespace detail {
class ParallelFor;
struct SolverWorkspace;
}

class SequentialImpulseSolver
{
public:
	static void solve(std::vector<ContactManifold>& contacts, PhysicsWorld& world, float dt);

private:
    static void solve(std::vector<ContactManifold>& contacts, PhysicsWorld& world,
        float dt, detail::SolverWorkspace& workspace, detail::ParallelFor& workers,
        std::size_t workerCount);

    friend class PhysicsWorld;
};

}

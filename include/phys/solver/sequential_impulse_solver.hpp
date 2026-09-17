#pragma once
#include <vector>
#include "phys/collision/manifold.hpp"

namespace phys {

class PhysicsWorld;
namespace detail { struct SolverWorkspace; }

class SequentialImpulseSolver
{
public:
	static void solve(std::vector<ContactManifold>& contacts, PhysicsWorld& world, float dt);

private:
    static void solve(std::vector<ContactManifold>& contacts, PhysicsWorld& world,
        float dt, detail::SolverWorkspace& workspace);

    friend class PhysicsWorld;
};

}

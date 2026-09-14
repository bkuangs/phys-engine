#pragma once
#include <vector>
#include "phys/collision/manifold.hpp"

namespace phys {

class PhysicsWorld;

class SequentialImpulseSolver
{
public:
	static void solve(std::vector<ContactManifold>& contacts, PhysicsWorld& world, float dt);
};

}

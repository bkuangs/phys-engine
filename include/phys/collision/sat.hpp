#pragma once
#include "phys/math/vec3.hpp"
#include "phys/math/transform.hpp"

namespace phys {

struct Box;

// Separating-axis test for two local-space boxes and their world transforms.
// On overlap, outAxis/outDepth
// hold the minimum-translation axis (pointing from a toward b) and penetration depth.
bool testOBBOBB(const Box& a, const Transform& transformA,
				const Box& b, const Transform& transformB,
				Vec3& outAxis, float& outDepth);

}

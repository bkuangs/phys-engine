#pragma once
#include "phys/math/vec3.hpp"

namespace phys {

struct Box;

// Separating-axis test for two oriented boxes. On overlap, outAxis/outDepth
// hold the minimum-translation axis (pointing from a toward b) and penetration depth.
bool testOBBOBB(const Box& a, const Box& b, Vec3& outAxis, float& outDepth);

}

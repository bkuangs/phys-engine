#pragma once
#include "phys/math/vec3.hpp"

namespace phys {

class NarrowPhase
{
public:
    static bool intersectSphereSphere(Vec3 cA, float rA, Vec3 cB, float rB, Vec3& normal, float& depth);
};

}

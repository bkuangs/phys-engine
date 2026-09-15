#pragma once
#include "phys/math/vec3.hpp"

namespace phys {

// We don't store center so that the same sphere shape can be reused at different positions
// Its world-space center will be calculated from the body and collider transforms
struct Sphere
{
    float radius{};
};

}

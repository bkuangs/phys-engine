#pragma once
#include "vec3.hpp"

namespace phys {

static class Math3d
{
public:
    static float length(Vec3 v);

    static float distance(Vec3 v);

    static Vec3 normalize(Vec3 v);

    static float dot(Vec3 a, Vec3 b);

    static float cross(Vec3 a, Vec3 b);
}

}

#pragma once
#include "vec3.hpp"

namespace phys {

class Math3d
{
public:
    static float length(Vec3 v);

    static float distance(Vec3 a, Vec3 b);

    static Vec3 normalize(Vec3 v);

    static float dot(Vec3 a, Vec3 b);

    static float cross(Vec3 a, Vec3 b);

    static float clamp(float value, float min, float max)
    {
        if (min == max) return min;
        if (min > max) throw new ArgumentOutOfRangeException();
        if (value < min) return min;
        if (value > max) return max;

        return value;
    }
};

}

#pragma once
#include <stdexcept>
#include "vec3.hpp"

namespace phys {

class Math3d
{
public:
    static float length(Vec3 v);

    static float distance(Vec3 a, Vec3 b);

    static Vec3 normalize(Vec3 v);

    static float dot(Vec3 a, Vec3 b);

    static Vec3 cross(Vec3 a, Vec3 b)
    {
        return {
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x
        };
    }

    static float sin(float radians);

    static float cos(float radians);

    static float clamp(float value, float min, float max)
    {
        if (min == max) return min;
        if (min > max) throw std::invalid_argument("clamp minimum exceeds maximum");
        if (value < min) return min;
        if (value > max) return max;

        return value;
    }
};

}

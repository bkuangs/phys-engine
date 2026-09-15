#include "phys/math/math_utils.hpp"

#include <cmath>

namespace phys {

float Math3d::length(Vec3 value)
{
    return std::sqrt(value.x * value.x
        + value.y * value.y
        + value.z * value.z);
}

float Math3d::distance(Vec3 a, Vec3 b)
{
    return length(a - b);
}

Vec3 Math3d::normalize(Vec3 value)
{
    float valueLength = length(value);
    if (valueLength <= 1e-8f)
        return Vec3::zero();

    return value * (1.0f / valueLength);
}

float Math3d::dot(Vec3 a, Vec3 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3 Math3d::cross(Vec3 a, Vec3 b)
{
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

float Math3d::sin(float radians)
{
    return std::sin(radians);
}

float Math3d::cos(float radians)
{
    return std::cos(radians);
}

}

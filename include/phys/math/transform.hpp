#pragma once
#include "phys/math/vec3.hpp"
#include "phys/math/quaternion.hpp"

namespace phys {

struct Transform
{
    Vec3 position{};
    Quaternion orientation = Quaternion::identity();

    static constexpr Transform zero() { return {}; }

    Transform(Vec3 pos, Quaternion rot) : position(pos), orientation(rot) {}

    // Convenience for planar (z-axis) rotations.
    Transform(Vec3 pos, float zAngleRadians)
    : position(pos), orientation(Quaternion::fromAxisAngle({0.0f, 0.0f, 1.0f}, zAngleRadians)) {}
};

inline Vec3 transform(Vec3 v, const Transform& tf)
{
    return tf.orientation.rotate(v) + tf.position;
}

}


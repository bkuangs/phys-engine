#pragma once
#include "phys/math/vec3.hpp"
#include "phys/math/quaternion.hpp"

namespace phys {

struct Box
{
    Vec3 center{};
    Vec3 halfExtents{};
    Quaternion orientation = Quaternion::identity();

    // World-space direction of the box's local x/y/z axis (index 0/1/2).
    Vec3 axis(int index) const {
        Mat3 m = orientation.toMat3();
        switch (index) {
            case 0: return {m.m00, m.m10, m.m20};
            case 1: return {m.m01, m.m11, m.m21};
            default: return {m.m02, m.m12, m.m22};
        }
    }
};

}

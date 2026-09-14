#pragma once
#include "phys/math/vec3.hpp"

namespace phys {

class RigidBody;
struct Collider;
struct Transform;

struct Aabb
{
    Vec3 min{};
    Vec3 max{};

    bool overlaps(const Aabb& other) const {
        return min.x <= other.max.x && max.x >= other.min.x
            && min.y <= other.max.y && max.y >= other.min.y
            && min.z <= other.max.z && max.z >= other.min.z;
    }

    static Aabb fromCollider(const Collider& collider, const Transform& bodyTransform);
};

}

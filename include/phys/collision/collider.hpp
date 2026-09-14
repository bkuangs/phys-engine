#pragma once
#include <cstdint>
#include "phys/collision/aabb.hpp"
#include "phys/collision/shapes/box.hpp"
#include "phys/collision/shapes/sphere.hpp"
#include "phys/math/transform.hpp"
#include "phys/world/body_handle.hpp"

namespace phys {

enum class ShapeType
{
    Sphere,
    Box,
    ConvexHull
};

struct ColliderHandle
{
    uint32_t index = 0;
    uint32_t generation = 0;
};

inline bool operator==(const ColliderHandle& left, const ColliderHandle& right)
{
    return left.index == right.index && left.generation == right.generation;
}

struct Collider
{
    RigidBodyHandle body{};
    Transform localTransform{};
    ShapeType type = ShapeType::Sphere;
    Sphere sphere{};
    Box box{};
    Aabb bounds{};
};

}

#include <phys/collision/aabb.hpp>
#include <phys/collision/collider.hpp>
#include <phys/math/mat3.hpp>
#include <cmath>

namespace phys {

Aabb Aabb::fromCollider(const Collider& collider, const Transform& bodyTransform)
{
    Transform worldTransform {
        bodyTransform.position + bodyTransform.orientation.rotate(collider.localTransform.position),
        bodyTransform.orientation * collider.localTransform.orientation
    };

    if (collider.type == ShapeType::Sphere) {
        Vec3 radius{collider.sphere.radius, collider.sphere.radius, collider.sphere.radius};
        return {worldTransform.position - radius, worldTransform.position + radius};
    }

    // ConvexHull uses an empty bound until hull vertex storage is introduced.
    Vec3 halfExtents = collider.box.halfExtents;
    Mat3 m = worldTransform.orientation.toMat3();

    Vec3 worldExtent{
        std::abs(m.m00) * halfExtents.x + std::abs(m.m01) * halfExtents.y + std::abs(m.m02) * halfExtents.z,
        std::abs(m.m10) * halfExtents.x + std::abs(m.m11) * halfExtents.y + std::abs(m.m12) * halfExtents.z,
        std::abs(m.m20) * halfExtents.x + std::abs(m.m21) * halfExtents.y + std::abs(m.m22) * halfExtents.z
    };

    return {worldTransform.position - worldExtent, worldTransform.position + worldExtent};
}

}

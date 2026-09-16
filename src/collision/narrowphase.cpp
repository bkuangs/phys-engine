#include <phys/collision/narrowphase.hpp>
#include <phys/collision/sat.hpp>
#include <phys/math/math_utils.hpp>
#include <array>
#include <type_traits>
#include <utility>

namespace phys
{

    using math = Math3d;

    namespace
    {

        Vec3 toLocalPoint(const Vec3 &worldPoint, const Transform &transform)
        {
            return transform.orientation.conjugate().rotate(
                worldPoint - transform.position);
        }

        Vec3 toWorldPoint(const Vec3 &localPoint, const Transform &transform)
        {
            return transform.position + transform.orientation.rotate(localPoint);
        }

        Vec3 boxSupportPoint(const Box &box, const Transform &transform, Vec3 direction)
        {
            Vec3 localDirection = transform.orientation.conjugate().rotate(direction);
            Vec3 localPoint{
                localDirection.x >= 0.0f ? box.halfExtents.x : -box.halfExtents.x,
                localDirection.y >= 0.0f ? box.halfExtents.y : -box.halfExtents.y,
                localDirection.z >= 0.0f ? box.halfExtents.z : -box.halfExtents.z};
            return toWorldPoint(localPoint, transform);
        }

        float boxProjectedRadius(const Box &box, const Transform &transform,
                                 const Vec3 &axis)
        {
            Mat3 rotation = transform.orientation.toMat3();
            Vec3 axes[] = {
                {rotation.m00, rotation.m10, rotation.m20},
                {rotation.m01, rotation.m11, rotation.m21},
                {rotation.m02, rotation.m12, rotation.m22}};
            return std::abs(math::dot(axes[0], axis)) * box.halfExtents.x + std::abs(math::dot(axes[1], axis)) * box.halfExtents.y + std::abs(math::dot(axes[2], axis)) * box.halfExtents.z;
        }

        float overlapMidpoint(const Box &boxA, const Transform &transformA,
                              const Box &boxB, const Transform &transformB, const Vec3 &axis)
        {
            float centerA = math::dot(transformA.position, axis);
            float centerB = math::dot(transformB.position, axis);
            float radiusA = boxProjectedRadius(boxA, transformA, axis);
            float radiusB = boxProjectedRadius(boxB, transformB, axis);
            float lower = std::max(centerA - radiusA, centerB - radiusB);
            float upper = std::min(centerA + radiusA, centerB + radiusB);
            return (lower + upper) * 0.5f;
        }

        std::array<Vec3, 4> supportFaceVertices(
            const Box &box, const Transform &transform, Vec3 direction)
        {
            Vec3 localDirection = transform.orientation.conjugate().rotate(direction);
            int faceAxis = 0;
            float largest = std::abs(localDirection.x);
            if (std::abs(localDirection.y) > largest) {
                faceAxis = 1;
                largest = std::abs(localDirection.y);
            }
            if (std::abs(localDirection.z) > largest)
                faceAxis = 2;

            float faceSign = faceAxis == 0
                ? (localDirection.x >= 0.0f ? 1.0f : -1.0f)
                : faceAxis == 1
                    ? (localDirection.y >= 0.0f ? 1.0f : -1.0f)
                    : (localDirection.z >= 0.0f ? 1.0f : -1.0f);
            float extents[] = {
                box.halfExtents.x, box.halfExtents.y, box.halfExtents.z};
            int firstAxis = (faceAxis + 1) % 3;
            int secondAxis = (faceAxis + 2) % 3;
            std::array<Vec3, 4> vertices{};
            for (int index = 0; index < 4; ++index) {
                float coordinates[] = {0.0f, 0.0f, 0.0f};
                coordinates[faceAxis] = faceSign * extents[faceAxis];
                coordinates[firstAxis] = (index & 1) != 0
                    ? extents[firstAxis] : -extents[firstAxis];
                coordinates[secondAxis] = (index & 2) != 0
                    ? extents[secondAxis] : -extents[secondAxis];
                vertices[index] = toWorldPoint(
                    {coordinates[0], coordinates[1], coordinates[2]}, transform);
            }
            return vertices;
        }

        bool pointInsideBox(const Vec3 &point, const Box &box,
                            const Transform &transform)
        {
            Vec3 localPoint = transform.orientation.conjugate().rotate(
                point - transform.position);
            constexpr float tolerance = 1e-4f;
            return std::abs(localPoint.x) <= box.halfExtents.x + tolerance
                && std::abs(localPoint.y) <= box.halfExtents.y + tolerance
                && std::abs(localPoint.z) <= box.halfExtents.z + tolerance;
        }

        bool duplicatePoint(const std::array<Vec3, 4> &points,
                            uint32_t count, const Vec3 &point)
        {
            constexpr float toleranceSquared = 1e-6f;
            for (uint32_t index = 0; index < count; ++index) {
                Vec3 delta = points[index] - point;
                if (math::dot(delta, delta) <= toleranceSquared)
                    return true;
            }
            return false;
        }

        void setAnchors(ContactPoint &point,
                        const Vec3 &worldPointA, const Transform &transformA,
                        const Vec3 &worldPointB, const Transform &transformB)
        {
            point.localAnchorA = toLocalPoint(worldPointA, transformA);
            point.localAnchorB = toLocalPoint(worldPointB, transformB);
        }

    }

    bool NarrowPhase::intersectSphereSphere(
        const Sphere &sphereA,
        const Transform &transformA, // transform = world pos + orientation
        const Sphere &sphereB,
        const Transform &transformB,
        ContactManifold &manifold)
    {
        Vec3 delta = transformB.position - transformA.position;
        float distance = math::length(delta);
        float radii = sphereA.radius + sphereB.radius;

        if (distance >= radii)
            return false; // spheres are not touching

        manifold.normal = distance > 1e-6f // avoid divide by zero
                              ? delta / distance
                              : Vec3{1.0f, 0.0f, 0.0f};

        manifold.pointCount = 1;
        ContactPoint &point = manifold.points[0];
        point.penetration = radii - distance;

        Vec3 pointA = transformA.position + manifold.normal * sphereA.radius;
        Vec3 pointB = transformB.position - manifold.normal * sphereB.radius;
        Vec3 worldContact = (pointA + pointB) * 0.5f;
        setAnchors(point, worldContact, transformA, worldContact, transformB);

        return true;
    }

    bool NarrowPhase::intersectSphereBox(
        const Sphere &sphere,
        const Transform &sphereTransform, // transform = world pos + orientation
        const Box &box,
        const Transform &boxTransform,
        ContactManifold &manifold)
    {
        // Sphere world space -> box local space
        Vec3 sphereCenterLocal =
            boxTransform.orientation.conjugate().rotate( // inverse box rotation -> box relative
                sphereTransform.position - boxTransform.position);

        // Closest point on box to sphere center
        Vec3 closestLocal{
            math::clamp(sphereCenterLocal.x, -box.halfExtents.x, box.halfExtents.x),
            math::clamp(sphereCenterLocal.y, -box.halfExtents.y, box.halfExtents.y),
            math::clamp(sphereCenterLocal.z, -box.halfExtents.z, box.halfExtents.z)};

        Vec3 deltaLocal = closestLocal - sphereCenterLocal;
        float distance = math::length(deltaLocal);

        Vec3 normalLocal;
        float penetration;
        if (distance > 1e-6f)
        {
            if (distance >= sphere.radius)
                return false;

            normalLocal = deltaLocal / distance;
            penetration = sphere.radius - distance;

            // Sphere center is inside the box
        }
        else
        {
            // Find the nearest box face instead
            float distanceToPositiveX = box.halfExtents.x - sphereCenterLocal.x;
            float distanceToNegativeX = box.halfExtents.x + sphereCenterLocal.x;
            float distanceToPositiveY = box.halfExtents.y - sphereCenterLocal.y;
            float distanceToNegativeY = box.halfExtents.y + sphereCenterLocal.y;
            float distanceToPositiveZ = box.halfExtents.z - sphereCenterLocal.z;
            float distanceToNegativeZ = box.halfExtents.z + sphereCenterLocal.z;

            float nearestFace = distanceToPositiveX;
            normalLocal = {-1.0f, 0.0f, 0.0f};

            if (distanceToNegativeX < nearestFace)
            {
                nearestFace = distanceToNegativeX;
                normalLocal = {1.0f, 0.0f, 0.0f};
            }
            if (distanceToPositiveY < nearestFace)
            {
                nearestFace = distanceToPositiveY;
                normalLocal = {0.0f, -1.0f, 0.0f};
            }
            if (distanceToNegativeY < nearestFace)
            {
                nearestFace = distanceToNegativeY;
                normalLocal = {0.0f, 1.0f, 0.0f};
            }
            if (distanceToPositiveZ < nearestFace)
            {
                nearestFace = distanceToPositiveZ;
                normalLocal = {0.0f, 0.0f, -1.0f};
            }
            if (distanceToNegativeZ < nearestFace)
            {
                nearestFace = distanceToNegativeZ;
                normalLocal = {0.0f, 0.0f, 1.0f};
            }

            penetration = sphere.radius + nearestFace; // move center out
        }

        manifold.normal = boxTransform.orientation.rotate(normalLocal);
        manifold.pointCount = 1;
        ContactPoint &point = manifold.points[0];
        point.penetration = penetration;

        Vec3 boxPointWorld = toWorldPoint(closestLocal, boxTransform);
        Vec3 spherePointWorld = sphereTransform.position + manifold.normal * sphere.radius;
        Vec3 worldContact = (spherePointWorld + boxPointWorld) * 0.5f;
        setAnchors(point, worldContact, sphereTransform, worldContact, boxTransform);

        return true;
    }

    bool NarrowPhase::intersectBoxBox(
        const Box &boxA,
        const Transform &transformA,
        const Box &boxB,
        const Transform &transformB,
        ContactManifold &manifold)
    {
        Vec3 normal;
        float penetration;

        if (!testOBBOBB(boxA, transformA, boxB, transformB, normal, penetration))
            return false;

        Vec3 centerDelta = transformB.position - transformA.position;

        // Ensure normal points from A toward B.
        if (math::dot(centerDelta, normal) < 0.0f)
            normal = -normal;

        manifold.normal = normal;
        Vec3 tangentReference = std::abs(normal.x) < 0.9f
                                    ? Vec3{1.0f, 0.0f, 0.0f}
                                    : Vec3{0.0f, 1.0f, 0.0f};
        Vec3 tangent1 = math::normalize(math::cross(normal, tangentReference));
        Vec3 tangent2 = math::cross(normal, tangent1);
        float normalCoordinate = (math::dot(transformA.position, normal) + boxProjectedRadius(boxA, transformA, normal) + math::dot(transformB.position, normal) - boxProjectedRadius(boxB, transformB, normal)) * 0.5f;
        Vec3 fallbackContact = normal * normalCoordinate
            + tangent1 * overlapMidpoint(
                boxA, transformA, boxB, transformB, tangent1)
            + tangent2 * overlapMidpoint(
                boxA, transformA, boxB, transformB, tangent2);

        std::array<Vec3, 4> contacts{};
        uint32_t contactCount = 0;
        auto addFaceCandidates = [&](const Box &sourceBox,
                                     const Transform &sourceTransform,
                                     const Box &otherBox,
                                     const Transform &otherTransform,
                                     Vec3 faceDirection) {
            for (const Vec3 &vertex : supportFaceVertices(
                     sourceBox, sourceTransform, faceDirection)) {
                Vec3 candidate = vertex + normal * (normalCoordinate
                    - math::dot(vertex, normal));
                if (pointInsideBox(candidate, otherBox, otherTransform)
                    && contactCount < contacts.size()
                    && !duplicatePoint(contacts, contactCount, candidate)) {
                    contacts[contactCount++] = candidate;
                }
            }
        };

        addFaceCandidates(boxA, transformA, boxB, transformB, normal);
        addFaceCandidates(boxB, transformB, boxA, transformA, -normal);
        if (contactCount == 0)
            contacts[contactCount++] = fallbackContact;

        manifold.pointCount = contactCount;
        for (uint32_t index = 0; index < contactCount; ++index) {
            ContactPoint &point = manifold.points[index];
            point.penetration = penetration;
            setAnchors(point, contacts[index], transformA,
                contacts[index], transformB);
        }

        return true;
    }

    bool NarrowPhase::generateContact(
        const Collider &a,
        const Transform &transformA,
        const Collider &b,
        const Transform &transformB,
        ContactManifold &manifold)
    {
        manifold = ContactManifold{};
        manifold.bodyA = a.body;
        manifold.bodyB = b.body;

        return std::visit(
            [&](const auto &shapeA, const auto &shapeB)
            {
                using ShapeA = std::decay_t<decltype(shapeA)>;
                using ShapeB = std::decay_t<decltype(shapeB)>;

                if constexpr (std::is_same_v<ShapeA, Sphere> && std::is_same_v<ShapeB, Sphere>)
                {
                    return intersectSphereSphere(
                        shapeA, transformA, shapeB, transformB, manifold);
                }
                else if constexpr (std::is_same_v<ShapeA, Sphere> && std::is_same_v<ShapeB, Box>)
                {
                    return intersectSphereBox(
                        shapeA, transformA, shapeB, transformB, manifold);
                }
                else if constexpr (std::is_same_v<ShapeA, Box> && std::is_same_v<ShapeB, Sphere>)
                {
                    bool intersects = intersectSphereBox(
                        shapeB, transformB, shapeA, transformA, manifold);
                    if (intersects)
                    {
                        manifold.normal = -manifold.normal;
                        for (uint32_t index = 0; index < manifold.pointCount; ++index)
                            std::swap(manifold.points[index].localAnchorA,
                                      manifold.points[index].localAnchorB);
                    }
                    return intersects;
                }
                else if constexpr (std::is_same_v<ShapeA, Box> && std::is_same_v<ShapeB, Box>)
                {
                    return intersectBoxBox(
                        shapeA, transformA, shapeB, transformB, manifold);
                }
            },
            a.shape,
            b.shape);
    }

}

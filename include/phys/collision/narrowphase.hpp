#pragma once
#include "phys/collision/collider.hpp"
#include "phys/collision/manifold.hpp"
#include "phys/collision/shapes/box.hpp"
#include "phys/collision/shapes/sphere.hpp"
#include "phys/math/transform.hpp"

namespace phys {

class NarrowPhase
{
public:
    static bool intersectSphereSphere(
        const Sphere& sphereA,
        const Transform& transformA,
        const Sphere& sphereB,
        const Transform& transformB,
        ContactManifold& manifold);

    static bool intersectSphereBox(
        const Sphere& sphere,
        const Transform& sphereTransform,
        const Box& box,
        const Transform& boxTransform,
        ContactManifold& manifold);

    static bool intersectBoxBox(
        const Box& boxA,
        const Transform& transformA,
        const Box& boxB,
        const Transform& transformB,
        ContactManifold& manifold);

    static bool generateContact(
        const Collider& a,
        const Transform& transformA,
        const Collider& b,
        const Transform& transformB,
        ContactManifold& manifold);
};

}

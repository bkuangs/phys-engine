#include <phys/collision/narrowphase.hpp>
#include <phys/math/math_utils.hpp>

namespace phys {

using math = Math3d;

bool NarrowPhase::intersectSphereSphere(
    const Sphere& sphereA,
    const Transform& transformA,
    const Sphere& sphereB,
    const Transform& transformB,
    ContactManifold& manifold)
{
    float d = math::distance(transformA->position, transformB->position);
    float radii = sphereA->radius + sphereB->radius;

    if (d >= radii) return false;       // spheres are not touching

    Vec3 n = math::normalize(transformB->position - transformA->position);
    
    

    return true;
}

bool NarrowPhase::generateContact(
    const Collider& a,
    const Transform& transformA,
    const Collider& b,
    const Transform& transformB,
    ContactManifold& manifold);

// TODO: Dispatch supported shape pairs and generate contact manifolds.

}

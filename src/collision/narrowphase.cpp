#include <phys/collision/narrowphase.hpp>
#include <phys/math/math_utils.hpp>

namespace phys {

using math = Math3d;

bool NarrowPhase::intersectSphereSphere(Vec3 cA, float rA, Vec3 cB, float rB, Vec3& normal, float& depth)
{
    normal = Vec3::zero();
    depth = 0.f;

    float d = math::distance(cA, cB);
    float radii = rA + rB;

    if (d >= radii) return false;       // spheres are not touching

    normal = math::normalize(cB - cA);  // A -> B (which direction we need to push B)
    depth = radii - d;

    return true;
}

// TODO: Dispatch supported shape pairs and generate contact manifolds.

}

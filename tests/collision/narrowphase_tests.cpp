#include <phys/collision/narrowphase.hpp>
#include <cmath>
#include <iostream>

namespace {

constexpr float tolerance = 1e-5f;

bool near(float actual, float expected)
{
    return std::abs(actual - expected) <= tolerance;
}

bool near(const phys::Vec3& actual, const phys::Vec3& expected)
{
    return near(actual.x, expected.x)
        && near(actual.y, expected.y)
        && near(actual.z, expected.z);
}

bool expectVec(const char* name, const phys::Vec3& actual,
    const phys::Vec3& expected)
{
    if (near(actual, expected))
        return true;

    std::cerr << name << ": expected (" << expected.x << ", " << expected.y
        << ", " << expected.z << "), got (" << actual.x << ", "
        << actual.y << ", " << actual.z << ")\n";
    return false;
}

bool testSphereSphereAnchors()
{
    phys::Collider sphereA{};
    sphereA.shape = phys::Sphere{1.0f};
    phys::Collider sphereB{};
    sphereB.shape = phys::Sphere{1.0f};
    phys::Transform transformA{{0.0f, 0.0f, 0.0f}, phys::Quaternion::identity()};
    phys::Transform transformB{{1.5f, 0.0f, 0.0f}, phys::Quaternion::identity()};
    phys::ContactManifold manifold{};

    if (!phys::NarrowPhase::generateContact(
            sphereA, transformA, sphereB, transformB, manifold)) {
        std::cerr << "sphere-sphere contact was not generated\n";
        return false;
    }

    const phys::ContactPoint& point = manifold.points[0];
    return manifold.pointCount == 1
        && expectVec("sphere A anchor", point.localAnchorA, {0.75f, 0.0f, 0.0f})
        && expectVec("sphere B anchor", point.localAnchorB, {-0.75f, 0.0f, 0.0f})
        && expectVec("sphere A world anchor", phys::transform(point.localAnchorA, transformA),
            {0.75f, 0.0f, 0.0f})
        && expectVec("sphere B world anchor", phys::transform(point.localAnchorB, transformB),
            {0.75f, 0.0f, 0.0f});
}

bool testBoxSphereAnchorOrder()
{
    constexpr float halfPi = 1.57079632679f;
    phys::Collider box{};
    box.shape = phys::Box{{1.0f, 1.0f, 1.0f}};
    phys::Collider sphere{};
    sphere.shape = phys::Sphere{1.0f};
    phys::Transform boxTransform{{0.0f, 0.0f, 0.0f}, halfPi};
    phys::Transform sphereTransform{{0.0f, 1.5f, 0.0f}, phys::Quaternion::identity()};
    phys::ContactManifold manifold{};

    if (!phys::NarrowPhase::generateContact(
            box, boxTransform, sphere, sphereTransform, manifold)) {
        std::cerr << "box-sphere contact was not generated\n";
        return false;
    }

    const phys::ContactPoint& point = manifold.points[0];
    return manifold.pointCount == 1
        && expectVec("box anchor", point.localAnchorA, {0.75f, 0.0f, 0.0f})
        && expectVec("sphere anchor", point.localAnchorB, {0.0f, -0.75f, 0.0f})
        && expectVec("box world anchor", phys::transform(point.localAnchorA, boxTransform),
            {0.0f, 0.75f, 0.0f})
        && expectVec("sphere world anchor", phys::transform(point.localAnchorB, sphereTransform),
            {0.0f, 0.75f, 0.0f});
}

}

int main()
{
    if (!testSphereSphereAnchors() || !testBoxSphereAnchorOrder())
        return 1;

    return 0;
}

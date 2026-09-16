#include <phys/collision/narrowphase.hpp>
#include <phys/math/math_utils.hpp>
#include <array>
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

bool testBoxBoxNormalIsNormalized()
{
    phys::Collider boxA{};
    boxA.shape = phys::Box{{1.0f, 1.0f, 1.0f}};
    phys::Collider boxB{};
    boxB.shape = phys::Box{{1.0f, 1.0f, 1.0f}};
    phys::Transform transformA{{0.0f, 0.0f, 0.0f},
        phys::Quaternion::fromAxisAngle({0.0f, 1.0f, 0.0f}, 0.4f)};
    phys::Transform transformB{{1.2f, 0.8f, 0.6f},
        phys::Quaternion::fromAxisAngle({1.0f, 0.0f, 0.0f}, 0.7f)};
    phys::ContactManifold manifold{};

    if (!phys::NarrowPhase::generateContact(
            boxA, transformA, boxB, transformB, manifold)) {
        std::cerr << "box-box contact was not generated\n";
        return false;
    }

    return near(phys::Math3d::length(manifold.normal), 1.0f);
}

bool testTiltedBoxFloorContacts(float angle, float height, bool reverseOrder)
{
    phys::Collider floor{};
    floor.shape = phys::Box{{9.0f, 0.5f, 4.0f}};
    phys::Collider box{};
    box.shape = phys::Box{{0.5f, 0.5f, 0.5f}};
    phys::Transform floorTransform{{0.0f, -0.5f, 0.0f}, 0.0f};
    phys::Transform boxTransform{{0.0f, height, 0.0f}, angle};
    phys::ContactManifold manifold{};
    bool hit = reverseOrder
        ? phys::NarrowPhase::generateContact(
            box, boxTransform, floor, floorTransform, manifold)
        : phys::NarrowPhase::generateContact(
            floor, floorTransform, box, boxTransform, manifold);
    if (!hit) {
        std::cerr << "tilted box-floor contact was not generated\n";
        return false;
    }

    std::array<phys::Vec3, 4> expectedVertices{};
    uint32_t expectedCount = 0;
    for (float x : {-0.5f, 0.5f}) {
        for (float z : {-0.5f, 0.5f}) {
            phys::Vec3 vertex = phys::transform({x, -0.5f, z}, boxTransform);
            if (vertex.y <= 0.0f)
                expectedVertices[expectedCount++] = vertex;
        }
    }
    if (manifold.pointCount != expectedCount) {
        std::cerr << "expected " << expectedCount << " penetrating corners, got "
            << manifold.pointCount << " contacts\n";
        return false;
    }
    if (!expectVec("tilted box normal", manifold.normal,
            {0.0f, reverseOrder ? -1.0f : 1.0f, 0.0f}))
        return false;

    for (uint32_t index = 0; index < expectedCount; ++index) {
        const phys::Vec3& vertex = expectedVertices[index];
        phys::Vec3 expectedContact{vertex.x, vertex.y * 0.5f, vertex.z};
        bool found = false;
        for (uint32_t pointIndex = 0; pointIndex < manifold.pointCount; ++pointIndex) {
            const phys::ContactPoint& point = manifold.points[pointIndex];
            phys::Vec3 anchorA = phys::transform(point.localAnchorA,
                reverseOrder ? boxTransform : floorTransform);
            phys::Vec3 anchorB = phys::transform(point.localAnchorB,
                reverseOrder ? floorTransform : boxTransform);
            if (near(anchorA, expectedContact) && near(anchorB, expectedContact)
                && near(point.penetration, -vertex.y))
                found = true;
        }
        if (!found) {
            std::cerr << "missing tilted-box corner contact with its actual depth\n";
            return false;
        }
    }
    return true;
}

}

int main()
{
    if (!testSphereSphereAnchors() || !testBoxSphereAnchorOrder()
        || !testBoxBoxNormalIsNormalized()
        || !testTiltedBoxFloorContacts(0.3f,
            0.5f * (std::sin(0.3f) + std::cos(0.3f)) - 0.02f, false)
        || !testTiltedBoxFloorContacts(0.3f,
            0.5f * (std::sin(0.3f) + std::cos(0.3f)) - 0.02f, true)
        || !testTiltedBoxFloorContacts(0.1f, 0.3f, false)
        || !testTiltedBoxFloorContacts(0.1f, 0.3f, true))
        return 1;

    return 0;
}

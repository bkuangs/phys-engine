#include <phys/collision/sat.hpp>
#include <phys/collision/shapes/box.hpp>
#include <cmath>
#include <limits>

namespace phys {

namespace {

struct WorldBox
{
    Vec3 center;
    Vec3 halfExtents;
    Quaternion orientation;

    Vec3 axis(int index) const {
        Mat3 matrix = orientation.toMat3();
        switch (index) {
            case 0: return {matrix.m00, matrix.m10, matrix.m20};
            case 1: return {matrix.m01, matrix.m11, matrix.m21};
            default: return {matrix.m02, matrix.m12, matrix.m22};
        }
    }
};

float dot(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3 cross(const Vec3& a, const Vec3& b) {
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

float projectedRadius(const WorldBox& box, const Vec3& axis) {
    return std::abs(dot(box.axis(0), axis)) * box.halfExtents.x
         + std::abs(dot(box.axis(1), axis)) * box.halfExtents.y
         + std::abs(dot(box.axis(2), axis)) * box.halfExtents.z;
}

// Returns false (separating axis found) or true with the overlap depth along axis.
bool overlapOnAxis(const WorldBox& a, const WorldBox& b, Vec3 axis,
                   Vec3& normalizedAxis, float& depth) {
    float lengthSq = dot(axis, axis);
    if (lengthSq < 1e-8f) {
        depth = std::numeric_limits<float>::max();  // degenerate axis (parallel edges): ignore
        return true;
    }
    normalizedAxis = axis * (1.0f / std::sqrt(lengthSq));

    float distance = std::abs(dot(b.center - a.center, normalizedAxis));
    float depthOnAxis = (projectedRadius(a, normalizedAxis)
        + projectedRadius(b, normalizedAxis)) - distance;

    if (depthOnAxis < 0.0f) return false;

    depth = depthOnAxis;
    return true;
}

} // namespace

bool testOBBOBB(const Box& a, const Transform& transformA,
                const Box& b, const Transform& transformB,
                Vec3& outAxis, float& outDepth)
{
    outDepth = std::numeric_limits<float>::max();

    WorldBox worldA{transformA.position, a.halfExtents, transformA.orientation};
    WorldBox worldB{transformB.position, b.halfExtents, transformB.orientation};
    Vec3 axesA[3] = {worldA.axis(0), worldA.axis(1), worldA.axis(2)};
    Vec3 axesB[3] = {worldB.axis(0), worldB.axis(1), worldB.axis(2)};

    for (const Vec3& axis : axesA) {
        Vec3 normalizedAxis;
        float depth;
        if (!overlapOnAxis(worldA, worldB, axis, normalizedAxis, depth)) return false;
        if (depth < outDepth) { outDepth = depth; outAxis = normalizedAxis; }
    }

    for (const Vec3& axis : axesB) {
        Vec3 normalizedAxis;
        float depth;
        if (!overlapOnAxis(worldA, worldB, axis, normalizedAxis, depth)) return false;
        if (depth < outDepth) { outDepth = depth; outAxis = normalizedAxis; }
    }

    for (const Vec3& edgeA : axesA) {
        for (const Vec3& edgeB : axesB) {
            Vec3 axis = cross(edgeA, edgeB);
            Vec3 normalizedAxis;
            float depth;
            if (!overlapOnAxis(worldA, worldB, axis, normalizedAxis, depth)) return false;
            if (depth < outDepth) { outDepth = depth; outAxis = normalizedAxis; }
        }
    }

    return true;
}

}

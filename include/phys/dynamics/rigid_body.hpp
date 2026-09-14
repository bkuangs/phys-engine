#pragma once
#include <string>

#include "phys/math/vec3.hpp"
#include "phys/math/quaternion.hpp"
#include "phys/math/math_utils.hpp"
#include "phys/core/constants.hpp"

namespace phys {

class RigidBody
{
public:
    enum class ShapeType
    {
        Sphere,
        Box,
        ConvexHull
    };

    float mass;
    float density;
    float restitution;
    float area;

    float radius;
    float width;
    float height;
    float depth;

    ShapeType type;
    bool isStatic;

    RigidBody(
        Vec3 position,
        float density,
        float mass,
        float restitution,
        float area,
        float radius,
        float width,
        float height,
        float depth,
        bool isStatic,
        ShapeType type)
        : mass(mass),
        density(density),
        restitution(restitution),
        area(area),
        radius(radius),
        width(width),
        height(height),
        depth(depth),
        type(type),
        isStatic(isStatic),
        position(position),
        linVelo{},
        rotation{},
        angularVelo{}
    {
    }

    // Pose-integration stage: advances position by the current linear velocity.
    void integratePosition(float dt);

    // Pose-integration stage: advances orientation by the current angular velocity.
    void integrateRotation(float dt);

    Vec3 getPosition() const { return position; }
    Vec3 getLinearVelocity() const { return linVelo; }
    void setLinearVelocity(const Vec3& velocity) { linVelo = velocity; }

    Quaternion getRotation() const { return rotation; }
    Vec3 getAngularVelocity() const { return angularVelo; }
    void setAngularVelocity(const Vec3& velocity) { angularVelo = velocity; }

    static bool createSphere(float radius, Vec3 position, float density,
        bool isStatic, float restitution, RigidBody& body, std::string& errorMessage)
    {
        errorMessage.clear();

        const float area = radius * radius * MathConstants::pi;

        if (area < BodyLimits::minSize) {
            errorMessage = "Sphere is too small";
            return false;
        }

        if (area > BodyLimits::maxSize) {
            errorMessage = "Sphere is too large";
            return false;
        }

        // TODO: Check density

        restitution = Math3d::clamp(restitution, 0.f, 1.f);

        const float mass = area * density;

        body = RigidBody(
            position,
            density,
            mass,
            restitution,
            area,
            radius,
            0.0f,
            0.0f,
            0.0f,
            isStatic,
            ShapeType::Sphere
        );

        return true;
    }

    static bool createBox(float width, float height, float depth, Vec3 position,
        float density, bool isStatic, float restitution, RigidBody& body,
        std::string& errorMessage)
    {
        errorMessage.clear();

        if (width <= 0.0f || height <= 0.0f || depth <= 0.0f) {
            errorMessage = "Box dimensions must be positive";
            return false;
        }

        const float volume = width * height * depth;

        if (volume < BodyLimits::minSize) {
            errorMessage = "Box is too small";
            return false;
        }

        if (volume > BodyLimits::maxSize) {
            errorMessage = "Box is too large";
            return false;
        }

        restitution = Math3d::clamp(restitution, 0.0f, 1.0f);

        const float mass = volume * density;

        body = RigidBody(
            position,
            density,
            mass,
            restitution,
            volume,
            0.0f,
            width,
            height,
            depth,
            isStatic,
            ShapeType::Box
        );

        return true;
    }

private:
    Vec3 position;
    Vec3 linVelo;
    Quaternion rotation = Quaternion::identity();
    Vec3 angularVelo;
};

}

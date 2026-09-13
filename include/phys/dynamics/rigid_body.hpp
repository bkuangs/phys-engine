#pragma once
#include <string>

#include "phys/math/vec3.hpp"
#include "phys/math/math_utils.hpp"
#include "phys/world/physics_world.hpp"

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
        bool isStatic,
        ShapeType type)
        : mass(mass),
        density(density),
        restitution(restitution),
        area(area),
        radius(radius),
        width(0.0f),
        height(0.0f),
        depth(0.0f),
        type(type),
        isStatic(isStatic),
        position(position),
        linVelo{},
        rotation{},
        angularVelo{}
    {
    }

    RigidBody(
        Vec3 position,
        float density,
        float mass,
        float restitution,
        float volume,
        float width,
        float height,
        float depth,
        bool isStatic,
        ShapeType type)
        : mass(mass),
        density(density),
        restitution(restitution),
        area(volume),
        radius(0.0f),
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

    static bool createSphere(float radius, Vec3 position, float density,
        bool isStatic, float restitution, RigidBody& body, std::string& errorMessage)
    {
        errorMessage.clear();

        const float area = radius * radius * MathConstants::pi;

        if (area < PhysicsWorld::minBodySize) {
            errorMessage = "Sphere is too small";
            return false;
        }

        if (area > PhysicsWorld::maxBodySize) {
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

        if (volume < PhysicsWorld::minBodySize) {
            errorMessage = "Box is too small";
            return false;
        }

        if (volume > PhysicsWorld::maxBodySize) {
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
    Vec3 rotation;
    Vec3 angularVelo;
};

}

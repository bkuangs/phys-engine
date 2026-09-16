#include <phys/dynamics/rigid_body.hpp>
#include <phys/core/constants.hpp>
#include <phys/dynamics/mass_properties.hpp>
#include <phys/math/math_utils.hpp>

namespace phys {

namespace {

bool validateDensity(float density, std::string& errorMessage)
{
    if (!(density >= BodyLimits::minDensity
            && density <= BodyLimits::maxDensity)) {
        errorMessage = "Density is out of range";
        return false;
    }
    return true;
}

}

bool RigidBody::createSphere(float radius, Vec3 position, float density,
    bool isStatic, float restitution, RigidBody& body,
    std::string& errorMessage)
{
    errorMessage.clear();

    const float volume = (4.0f / 3.0f) * MathConstants::pi
        * radius * radius * radius;
    if (volume < BodyLimits::minSize) {
        errorMessage = "Sphere is too small";
        return false;
    }
    if (volume > BodyLimits::maxSize) {
        errorMessage = "Sphere is too large";
        return false;
    }
    if (!validateDensity(density, errorMessage)) return false;

    restitution = Math3d::clamp(restitution, 0.0f, 1.0f);
    const float mass = volume * density;
    body = RigidBody(position, mass, restitution, isStatic);
    if (!isStatic)
        body.inverseInertiaLocal =
            MassProperties::sphereInverseInertia(mass, radius);

    return true;
}

bool RigidBody::createBox(float width, float height, float depth, Vec3 position,
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
    if (!validateDensity(density, errorMessage)) return false;

    restitution = Math3d::clamp(restitution, 0.0f, 1.0f);
    const float mass = volume * density;
    body = RigidBody(position, mass, restitution, isStatic);
    if (!isStatic)
        body.inverseInertiaLocal = MassProperties::boxInverseInertia(
            mass, width, height, depth);

    return true;
}

void RigidBody::integrateVelocity(const Vec3& gravity, float dt)
{
    if (isStatic) return;

    Vec3 acceleration = gravity + force * getInverseMass();
    linearVelocity += acceleration * dt;
}

void RigidBody::integratePosition(float dt)
{
    if (isStatic) return;
    position += linearVelocity * dt;
}

void RigidBody::integrateRotation(float dt)
{
    if (isStatic) return;

    Quaternion spin{
        angularVelocity.x, angularVelocity.y, angularVelocity.z, 0.0f};
    Quaternion delta = spin * rotation;

    rotation.x += 0.5f * dt * delta.x;
    rotation.y += 0.5f * dt * delta.y;
    rotation.z += 0.5f * dt * delta.z;
    rotation.w += 0.5f * dt * delta.w;

    rotation = rotation.normalized();
}

}

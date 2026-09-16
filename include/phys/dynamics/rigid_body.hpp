#pragma once
#include <string>

#include "phys/math/vec3.hpp"
#include "phys/math/quaternion.hpp"
#include "phys/math/math_utils.hpp"

namespace phys
{

    class RigidBody
    {
    public:
        RigidBody() = default;

        float mass{};
        float restitution{};
        float friction{};

        bool isStatic{};

        RigidBody(
            Vec3 position,
            float mass,
            float restitution,
            float friction,
            bool isStatic)
            : mass(mass),
              restitution(restitution),
              friction(friction),
              isStatic(isStatic),
              position(position),
              linearVelocity{},
              rotation{},
              angularVelocity{}
        {
        }

        // Pose-integration stage: advances position by the current linear velocity.
        void integratePosition(float dt);

        // Velocity-integration stage: applies gravity and accumulated force.
        void integrateVelocity(const Vec3 &gravity, float dt);

        // Pose-integration stage: advances orientation by the current angular velocity.
        void integrateRotation(float dt);

        Vec3 getPosition() const { return position; }
        Vec3 getLinearVelocity() const { return linearVelocity; }
        void setLinearVelocity(const Vec3 &velocity) { linearVelocity = velocity; }
        void applyLinearImpulse(const Vec3 &impulse)
        {
            if (!isStatic)
                linearVelocity += impulse * getInverseMass();
        }

        float getInverseMass() const
        {
            return isStatic || mass <= 0.0f ? 0.0f : 1.0f / mass;
        }

        void applyForce(const Vec3 &value)
        {
            if (!isStatic)
                force += value;
        }
        void clearForces() { force = Vec3::zero(); }

        Quaternion getRotation() const { return rotation; }
        Vec3 getAngularVelocity() const { return angularVelocity; }
        void setAngularVelocity(const Vec3 &velocity) { angularVelocity = velocity; }
        Mat3 getInverseInertiaWorld() const
        {
            Mat3 rotationMatrix = rotation.toMat3();
            return rotationMatrix * inverseInertiaLocal * transpose(rotationMatrix);
        }
        void applyImpulse(const Vec3 &impulse, const Vec3 &offset)
        {
            if (isStatic)
                return;
            linearVelocity += impulse * getInverseMass();
            angularVelocity += getInverseInertiaWorld() * Math3d::cross(offset, impulse);
        }

        static bool createSphere(float radius, Vec3 position, float density,
                                 bool isStatic, float restitution, float friction, RigidBody &body,
                                 std::string &errorMessage);

        static bool createBox(float width, float height, float depth, Vec3 position,
                              float density, bool isStatic, float restitution, float friction,
                              RigidBody &body,
                              std::string &errorMessage);

    private:
        Vec3 position;
        Vec3 linearVelocity;
        Quaternion rotation = Quaternion::identity();
        Vec3 angularVelocity;
        Vec3 force;
        Mat3 inverseInertiaLocal{};
    };

}

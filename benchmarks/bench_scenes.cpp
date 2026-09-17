#include "bench_common.hpp"
#include <cmath>
#include <random>
#include <string>

namespace phys::bench
{

    PhysicsWorld makeSphereField(int bodyCount, unsigned seed)
    {
        PhysicsWorld world;

        std::mt19937 rng(seed);
        constexpr float radius = 0.5f;
        float side = std::cbrt(static_cast<float>(bodyCount)) * (radius * 4.0f);
        std::uniform_real_distribution<float> position(-side * 0.5f, side * 0.5f);

        for (int i = 0; i < bodyCount; ++i)
        {
            RigidBodyHandle body;
            ColliderHandle collider;
            std::string error;

            world.createSphere(
                radius,
                Vec3{position(rng), position(rng), position(rng)},
                1.0f,
                false,
                0.5f,
                0.5f,
                body,
                collider,
                error);
        }

        return world;
    }

}

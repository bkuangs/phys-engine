#include "bench_common.hpp"
#include <algorithm>
#include <cmath>
#include <random>
#include <stdexcept>
#include <string>

namespace phys::bench
{

    BenchmarkScene makeMixedField(int bodyCount, unsigned seed)
    {
        if (bodyCount <= 0)
            throw std::invalid_argument("Mixed benchmark requires a positive dynamic body count");
        BenchmarkScene scene;
        const std::size_t count = static_cast<std::size_t>(bodyCount);
        const std::size_t columnsNeeded = (count + 1) / 2;
        const std::size_t columns = static_cast<std::size_t>(
            std::ceil(std::sqrt(static_cast<double>(columnsNeeded))));
        const std::size_t rows = (columnsNeeded + columns - 1) / columns;
        constexpr float spacing = 1.5f;
        constexpr float margin = 32.0f;
        scene.floorSize = {static_cast<float>(columns) * spacing + 2.0f * margin,
                           1.0f, static_cast<float>(rows) * spacing + 2.0f * margin};
        // Keep the static slab within the body's volume limit as the footprint grows.
        scene.floorSize.y = std::min(1.0f, static_cast<float>(
            (BodyLimits::maxSize * 0.5) /
            (static_cast<double>(scene.floorSize.x) * scene.floorSize.z)));
        RigidBodyHandle body;
        ColliderHandle collider;
        std::string error;
        if (!scene.world.createBox(scene.floorSize.x, scene.floorSize.y, scene.floorSize.z,
                                    {0, -scene.floorSize.y * 0.5f, 0}, 1, true, 0, 0.6f,
                                    body, collider, error))
            throw std::runtime_error("Mixed benchmark floor creation failed: " + error);

        std::mt19937 rng(seed);
        std::vector<uint8_t> spheres(count, 0);
        std::fill_n(spheres.begin(), (count + 1) / 2, uint8_t{1});
        std::shuffle(spheres.begin(), spheres.end(), rng);
        std::uniform_real_distribution<float> jitter(-0.05f, 0.05f);
        scene.objects.reserve(count);
        for (std::size_t index = 0; index < count; ++index)
        {
            std::size_t column = (index / 2) % columns;
            std::size_t row = (index / 2) / columns;
            Vec3 position{
                (static_cast<float>(column) - static_cast<float>(columns - 1) * 0.5f) * spacing + jitter(rng),
                1.0f + static_cast<float>(index % 2) * 1.75f + jitter(rng),
                (static_cast<float>(row) - static_cast<float>(rows - 1) * 0.5f) * spacing + jitter(rng)};
            bool created = spheres[index]
                ? scene.world.createSphere(0.5f, position, 1, false, 0.1f, 0.6f, body, collider, error)
                : scene.world.createBox(1, 1, 1, position, 1, false, 0.1f, 0.6f, body, collider, error);
            if (!created)
                throw std::runtime_error("Mixed benchmark body creation failed: " + error);
            scene.objects.push_back({body, collider});
            if (spheres[index])
                ++scene.spheres;
            else
                ++scene.boxes;
        }
        return scene;
    }

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

#include <raylib.h>
#include <phys/world/physics_world.hpp>

#include <algorithm>
#include <string>
#include <vector>

namespace
{

    struct RenderObject
    {
        phys::RigidBodyHandle body;
        phys::ColliderHandle collider;
    };

    Color colorForShape(const phys::Collider &collider)
    {
        return std::holds_alternative<phys::Sphere>(collider.shape)
                   ? Color{80, 160, 255, 255}
                   : Color{255, 150, 70, 255};
    }

    void drawObject(const phys::PhysicsWorld &world,
                    const RenderObject &object)
    {
        const phys::RigidBody *body = world.getBody(object.body);
        const phys::Collider *collider = world.getCollider(object.collider);
        if (!body || !collider)
            return;

        phys::Vec3 position = body->getPosition();
        Color color = colorForShape(*collider);

        if (const auto *sphere = std::get_if<phys::Sphere>(&collider->shape))
        {
            DrawSphere(
                {position.x, position.y, position.z},
                sphere->radius,
                color);
            DrawSphereWires(
                {position.x, position.y, position.z},
                sphere->radius,
                12,
                8,
                DARKBLUE);
            return;
        }

        const auto &box = std::get<phys::Box>(collider->shape);
        Vector3 size{
            box.halfExtents.x * 2.0f,
            box.halfExtents.y * 2.0f,
            box.halfExtents.z * 2.0f};
        Vector3 center{position.x, position.y, position.z};
        DrawCubeV(center, size, color);
        DrawCubeWiresV(center, size, DARKBROWN);
    }

} // namespace

int main()
{
    InitWindow(1280, 720, "phys-engine sandbox");
    SetTargetFPS(60);

    phys::PhysicsWorld world;
    world.gravity = {0.0f, -9.81f, 0.0f};
    std::vector<RenderObject> objects;
    std::string error;

    phys::RigidBodyHandle body;
    phys::ColliderHandle collider;

    world.createBox(18.0f, 1.0f, 8.0f, {0.0f, -2.0f, 0.0f},
                    1.0f, true, 0.1f, 0.6f, body, collider, error);
    objects.push_back({body, collider});

    world.createBox(1.4f, 1.4f, 1.4f, {-1.5f, 3.0f, 0.0f},
                    1.0f, false, 0.2f, 0.5f, body, collider, error);
    objects.push_back({body, collider});

    world.createBox(1.0f, 2.0f, 1.0f, {1.0f, 5.0f, 0.0f},
                    1.0f, false, 0.2f, 0.5f, body, collider, error);
    objects.push_back({body, collider});

    world.createSphere(0.7f, {0.0f, 7.0f, 0.0f},
                       1.0f, false, 0.35f, 0.4f, body, collider, error);
    objects.push_back({body, collider});

    Camera3D camera{
        {10.0f, 8.0f, 14.0f},
        {0.0f, 1.5f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        45.0f,
        CAMERA_PERSPECTIVE};

    while (!WindowShouldClose())
    {
        float dt = std::min(GetFrameTime(), 1.0f / 30.0f);
        world.step(dt);

        BeginDrawing();
        ClearBackground({235, 239, 244, 255});
        BeginMode3D(camera);

        DrawGrid(24, 1.0f);
        for (const RenderObject &object : objects)
            drawObject(world, object);

        for (const phys::ContactManifold &contact : world.contacts())
        {
            const phys::RigidBody *bodyA = world.getBody(contact.bodyA);
            const phys::RigidBody *bodyB = world.getBody(contact.bodyB);
            if (!bodyA || !bodyB)
                continue;

            phys::Vec3 midpoint = (bodyA->getPosition() + bodyB->getPosition()) * 0.5f;
            phys::Vec3 end = midpoint + contact.normal;
            DrawLine3D(
                {midpoint.x, midpoint.y, midpoint.z},
                {end.x, end.y, end.z},
                RED);
        }

        EndMode3D();
        DrawText("ESC to quit | blue spheres, orange boxes, red contacts",
                 20, 20, 20, DARKGRAY);
        DrawText(TextFormat("contacts: %u",
                            static_cast<unsigned>(world.contacts().size())),
                 20, 48, 20, DARKGRAY);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}

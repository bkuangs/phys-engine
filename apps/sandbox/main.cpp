#include <raylib.h>
#include <phys/world/physics_world.hpp>

#include <algorithm>
#include <array>
#include <string>
#include <vector>

namespace
{

    constexpr Color backgroundColor{245, 242, 235, 255};
    constexpr Color groundColor{76, 87, 99, 255};
    constexpr Color gridColor{92, 103, 115, 255};
    constexpr Color contactColor{245, 214, 147, 255};
    constexpr Color secondaryTextColor{170, 184, 195, 255};
    constexpr std::array<Color, 3> objectColors{{
        {93, 164, 158, 255},
        {218, 133, 112, 255},
        {224, 183, 93, 255},
    }};

    struct RenderObject
    {
        phys::RigidBodyHandle body;
        phys::ColliderHandle collider;
    };

    void drawObject(const phys::PhysicsWorld &world,
                    const RenderObject &object)
    {
        const phys::RigidBody *body = world.getBody(object.body);
        const phys::Collider *collider = world.getCollider(object.collider);
        if (!body || !collider)
            return;

        phys::Vec3 position = body->getPosition();
        Color color = body->isStatic
                          ? groundColor
                          : objectColors[object.body.index % objectColors.size()];

        if (const auto *sphere = std::get_if<phys::Sphere>(&collider->shape))
        {
            DrawSphere(
                {position.x, position.y, position.z},
                sphere->radius,
                color);
            phys::Vec3 axis = body->getRotation().rotate(
                                  {1.0f, 0.0f, 0.0f}) *
                              sphere->radius;
            DrawSphere(
                {position.x + axis.x, position.y + axis.y, position.z + axis.z},
                sphere->radius * 0.14f,
                ColorBrightness(color, -0.5f));
            return;
        }

        const auto &box = std::get<phys::Box>(collider->shape);
        Vector3 size{
            box.halfExtents.x * 2.0f,
            box.halfExtents.y * 2.0f,
            box.halfExtents.z * 2.0f};
        Vector3 center{position.x, position.y, position.z};
        DrawCubeV(center, size, color);
        DrawCubeWiresV(center, size, ColorBrightness(color, -0.3f));
    }

} // namespace

int main()
{
    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(1280, 720, "phys-engine sandbox");
    SetTargetFPS(60);

    const std::string fontPath = std::string(GetApplicationDirectory()) +
                                 "assets/fonts/IBMPlexSans-Regular.ttf";
    Font uiFont = LoadFontEx(fontPath.c_str(), 40, nullptr, 0);
    if (uiFont.texture.id == 0 || uiFont.texture.id == GetFontDefault().texture.id)
    {
        TraceLog(LOG_ERROR, "SANDBOX: Failed to load UI font: %s", fontPath.c_str());
        UnloadFont(uiFont);
        CloseWindow();
        return 1;
    }
    SetTextureFilter(uiFont.texture, TEXTURE_FILTER_BILINEAR);

    phys::PhysicsWorld world;
    world.gravity = {0.0f, -9.81f, 0.0f};
    std::vector<RenderObject> objects;
    std::string error;

    phys::RigidBodyHandle body;
    phys::ColliderHandle collider;

    constexpr float groundWidth = 18.0f;
    constexpr float groundDepth = 8.0f;
    world.createBox(groundWidth, 1.0f, groundDepth, {0.0f, -0.5f, 0.0f},
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

    // A larger deterministic pile makes stacking and resting contacts easier
    // to observe without introducing scene-generation dependencies.
    for (int row = 0; row < 4; ++row)
    {
        for (int column = 0; column < 5; ++column)
        {
            float x = -4.0f + static_cast<float>(column) * 2.0f;
            float y = 2.0f + static_cast<float>(row) * 1.8f;
            float size = 0.9f + 0.1f * static_cast<float>((row + column) % 3);
            world.createBox(size, size, size, {x, y, 0.0f},
                            1.0f, false, 0.05f, 0.55f, body, collider, error);
            objects.push_back({body, collider});
        }
    }

    for (int index = 0; index < 8; ++index)
    {
        float x = -3.5f + static_cast<float>(index % 4) * 2.3f;
        float y = 10.0f + static_cast<float>(index / 4) * 1.8f;
        world.createSphere(0.55f, {x, y, 0.0f},
                           1.0f, false, 0.1f, 0.45f, body, collider, error);
        objects.push_back({body, collider});
    }

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
        ClearBackground(backgroundColor);
        BeginMode3D(camera);

        for (const RenderObject &object : objects)
            drawObject(world, object);

        // Lift the grid slightly above the platform to avoid z-fighting.
        constexpr float gridHeight = 0.01f;
        for (float x = -groundWidth * 0.5f; x <= groundWidth * 0.5f; x += 1.0f)
            DrawLine3D({x, gridHeight, -groundDepth * 0.5f},
                       {x, gridHeight, groundDepth * 0.5f}, gridColor);
        for (float z = -groundDepth * 0.5f; z <= groundDepth * 0.5f; z += 1.0f)
            DrawLine3D({-groundWidth * 0.5f, gridHeight, z},
                       {groundWidth * 0.5f, gridHeight, z}, gridColor);

        for (const phys::ContactManifold &contact : world.contacts())
        {
            const phys::RigidBody *bodyA = world.getBody(contact.bodyA);
            const phys::RigidBody *bodyB = world.getBody(contact.bodyB);
            if (!bodyA || !bodyB)
                continue;

            for (uint32_t index = 0; index < contact.pointCount; ++index)
            {
                const phys::ContactPoint &point = contact.points[index];
                phys::Vec3 anchorA = bodyA->getPosition() +
                                     bodyA->getRotation().rotate(point.localAnchorA);
                phys::Vec3 end = anchorA + contact.normal * 0.5f;
                DrawLine3D(
                    {anchorA.x, anchorA.y, anchorA.z},
                    {end.x, end.y, end.z},
                    contactColor);
            }
        }

        EndMode3D();
        DrawRectangleRounded({20.0f, 20.0f, 260.0f, 144.0f}, 0.12f, 6,
                             {35, 43, 53, 242});
        DrawTextEx(uiFont, "Physics sandbox", {38.0f, 34.0f},
                   20.0f, 0.0f, backgroundColor);
        DrawLine(38, 64, 262, 64, {66, 78, 90, 255});
        DrawTextEx(uiFont, "Objects", {38.0f, 74.0f},
                   13.0f, 0.0f, secondaryTextColor);
        DrawTextEx(uiFont, "Contacts", {164.0f, 74.0f},
                   13.0f, 0.0f, secondaryTextColor);
        DrawTextEx(uiFont, TextFormat("%u", static_cast<unsigned>(objects.size())),
                   {38.0f, 90.0f}, 24.0f, 0.0f, backgroundColor);
        DrawTextEx(uiFont, TextFormat("%u", static_cast<unsigned>(world.contacts().size())),
                   {164.0f, 90.0f}, 24.0f, 0.0f, backgroundColor);
        DrawRectangleRounded({38.0f, 130.0f, 32.0f, 18.0f}, 0.3f, 4,
                             {66, 78, 90, 255});
        DrawTextEx(uiFont, "Esc", {45.0f, 132.0f},
                   12.0f, 0.0f, backgroundColor);
        DrawTextEx(uiFont, "Quit", {78.0f, 131.0f},
                   14.0f, 0.0f, secondaryTextColor);
        EndDrawing();
    }

    UnloadFont(uiFont);
    CloseWindow();
    return 0;
}

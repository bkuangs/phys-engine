#pragma once

namespace phys {

class PhysicsWorld
{
public:
    // Use 'static' members to access variables without having to create an object
    static constexpr float minBodySize = 0.01f * 0.01f;
    static constexpr float maxBodySize = 64.0f * 64.0f;
    static constexpr float minDensity = 0.2f;           // g/cm^3
    static constexpr float maxDensity = 21.4f;          // platinum
};

struct RigidBodyHandle;

}

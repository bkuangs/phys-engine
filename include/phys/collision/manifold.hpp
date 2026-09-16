#pragma once
#include <array>
#include <cstdint>
#include "phys/math/vec3.hpp"
#include "phys/world/body_handle.hpp"

namespace phys {

struct ContactPoint
{
	Vec3 localAnchorA{};
	Vec3 localAnchorB{};

	float penetration = 0.0f;

	float normalImpulse = 0.0f;
	float tangentImpulse1 = 0.0f;
	float tangentImpulse2 = 0.0f;
};

struct ContactManifold
{
	RigidBodyHandle bodyA{};
	RigidBodyHandle bodyB{};

	// Points from body A toward body B.
	Vec3 normal{};

	std::array<ContactPoint, 4> points{};
	uint32_t pointCount = 0;

	float friction = 0.0f;
	float restitution = 0.0f;
};

}

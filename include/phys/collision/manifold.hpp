#pragma once
#include <array>
#include <cstdint>
#include "phys/collision/contact.hpp"
#include "phys/world/body_handle.hpp"

namespace phys {

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

#pragma once
#include "phys/math/vec3.hpp"

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

}

#pragma once
#include "phys/math/mat3.hpp"

namespace phys {

struct MassProperties
{
	static Mat3 sphereInverseInertia(float mass, float radius);
	static Mat3 boxInverseInertia(float mass, float width, float height,
		float depth);
};

}

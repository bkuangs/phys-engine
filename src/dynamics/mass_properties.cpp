#include <phys/dynamics/mass_properties.hpp>

namespace phys {

Mat3 MassProperties::sphereInverseInertia(float mass, float radius)
{
	if (mass <= 0.0f || radius <= 0.0f)
		return {};

	float inverseInertia = 1.0f / (0.4f * mass * radius * radius);
	return {
		inverseInertia, 0.0f, 0.0f,
		0.0f, inverseInertia, 0.0f,
		0.0f, 0.0f, inverseInertia
	};
}

Mat3 MassProperties::boxInverseInertia(float mass, float width, float height,
	float depth)
{
	if (mass <= 0.0f || width <= 0.0f || height <= 0.0f || depth <= 0.0f)
		return {};

	float factor = 12.0f / mass;
	return {
		factor / (height * height + depth * depth), 0.0f, 0.0f,
		0.0f, factor / (width * width + depth * depth), 0.0f,
		0.0f, 0.0f, factor / (width * width + height * height)
	};
}

}

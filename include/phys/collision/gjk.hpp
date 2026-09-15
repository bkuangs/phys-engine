#pragma once
#include <array>
#include <cstdint>
#include <functional>

#include "phys/math/vec3.hpp"

namespace phys {

using SupportFunction = std::function<Vec3(const Vec3& direction)>;

struct GjkSimplex
{
	std::array<Vec3, 4> points{};
	uint32_t size = 0;
};

// Returns true when the Minkowski difference contains the origin.
// The optional simplex is populated with the final intersecting simplex.
bool intersectGjk(
	const SupportFunction& supportA,
	const SupportFunction& supportB,
	GjkSimplex* simplex = nullptr,
	uint32_t maxIterations = 32);

}

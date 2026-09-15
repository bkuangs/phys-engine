#pragma once
#include <cstdint>

#include "phys/collision/gjk.hpp"
#include "phys/math/vec3.hpp"

namespace phys {

struct EpaResult
{
	Vec3 normal{};
	float depth = 0.0f;
};

// Extracts the minimum translation vector from an intersecting GJK simplex.
// The returned normal points out of the Minkowski difference.
bool computeEpa(
	const SupportFunction& supportA,
	const SupportFunction& supportB,
	const GjkSimplex& simplex,
	EpaResult& result,
	uint32_t maxIterations = 64,
	float tolerance = 1e-4f);

}

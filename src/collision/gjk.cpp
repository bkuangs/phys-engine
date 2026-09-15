#include <phys/collision/gjk.hpp>
#include <phys/math/math_utils.hpp>
#include <algorithm>
#include <cmath>
#include <utility>

namespace phys {

namespace {

Vec3 tripleCross(const Vec3& first, const Vec3& second, const Vec3& third)
{
	return Math3d::cross(Math3d::cross(first, second), third);
}

float lengthSquared(const Vec3& value)
{
	return Math3d::dot(value, value);
}

bool setLineDirection(GjkSimplex& simplex, Vec3& direction)
{
	const Vec3& pointA = simplex.points[0];
	const Vec3& pointB = simplex.points[1];
	Vec3 toOrigin = -pointA;
	Vec3 edge = pointB - pointA;

	if (Math3d::dot(edge, toOrigin) > 0.0f) {
		direction = tripleCross(edge, toOrigin, edge);
		if (lengthSquared(direction) < 1e-12f)
			direction = Math3d::cross(edge, {1.0f, 0.0f, 0.0f});
		if (lengthSquared(direction) < 1e-12f)
			direction = Math3d::cross(edge, {0.0f, 1.0f, 0.0f});
	} else {
		simplex.points[0] = pointA;
		simplex.size = 1;
		direction = toOrigin;
	}

	return false;
}

bool setTriangleDirection(GjkSimplex& simplex, Vec3& direction)
{
	const Vec3& pointA = simplex.points[0];
	const Vec3& pointB = simplex.points[1];
	const Vec3& pointC = simplex.points[2];
	Vec3 toOrigin = -pointA;
	Vec3 edgeAB = pointB - pointA;
	Vec3 edgeAC = pointC - pointA;
	Vec3 normal = Math3d::cross(edgeAB, edgeAC);

	if (Math3d::dot(Math3d::cross(normal, edgeAC), toOrigin) > 0.0f) {
		if (Math3d::dot(edgeAC, toOrigin) > 0.0f) {
			simplex.points[1] = pointC;
			simplex.size = 2;
			direction = tripleCross(edgeAC, toOrigin, edgeAC);
			return false;
		}
		return setLineDirection(simplex, direction);
	}

	if (Math3d::dot(Math3d::cross(edgeAB, normal), toOrigin) > 0.0f)
		return setLineDirection(simplex, direction);

	if (Math3d::dot(normal, toOrigin) > 0.0f) {
		direction = normal;
	} else {
		std::swap(simplex.points[1], simplex.points[2]);
		direction = -normal;
	}

	return false;
}

bool containsOrigin(GjkSimplex& simplex, Vec3& direction)
{
	if (simplex.size == 1) {
		direction = -simplex.points[0];
		return false;
	}
	if (simplex.size == 2)
		return setLineDirection(simplex, direction);
	if (simplex.size == 3)
		return setTriangleDirection(simplex, direction);

	const Vec3& pointA = simplex.points[0];
	const Vec3& pointB = simplex.points[1];
	const Vec3& pointC = simplex.points[2];
	const Vec3& pointD = simplex.points[3];
	Vec3 toOrigin = -pointA;

	auto originOutsideFace = [&](const Vec3& faceB,
		const Vec3& faceC, const Vec3& opposite) {
		Vec3 normal = Math3d::cross(faceB - pointA, faceC - pointA);
		if (Math3d::dot(normal, opposite - pointA) > 0.0f)
			normal = -normal;
		return Math3d::dot(normal, toOrigin) > 0.0f;
	};

	if (originOutsideFace(pointB, pointC, pointD)) {
		simplex.points[0] = pointA;
		simplex.points[1] = pointB;
		simplex.points[2] = pointC;
		simplex.size = 3;
		return setTriangleDirection(simplex, direction);
	}

	if (originOutsideFace(pointC, pointD, pointB)) {
		simplex.points[1] = pointC;
		simplex.points[2] = pointD;
		simplex.size = 3;
		return setTriangleDirection(simplex, direction);
	}

	if (originOutsideFace(pointD, pointB, pointC)) {
		simplex.points[1] = pointD;
		simplex.points[2] = pointB;
		simplex.size = 3;
		return setTriangleDirection(simplex, direction);
	}

	return true;
}

} // namespace

bool intersectGjk(const SupportFunction& supportA,
	const SupportFunction& supportB, GjkSimplex* simplexOutput,
	uint32_t maxIterations)
{
	GjkSimplex simplex;
	Vec3 direction{1.0f, 0.0f, 0.0f};

	for (uint32_t iteration = 0; iteration < maxIterations; ++iteration) {
		if (lengthSquared(direction) < 1e-12f)
			break;

		Vec3 point = supportA(direction) - supportB(-direction);
		if (Math3d::dot(point, direction) <= 0.0f)
			break;

		for (uint32_t index = std::min(simplex.size, 3u); index > 0; --index)
			simplex.points[index] = simplex.points[index - 1];
		simplex.points[0] = point;
		simplex.size = std::min(simplex.size + 1, 4u);

		if (containsOrigin(simplex, direction)) {
			if (simplexOutput)
				*simplexOutput = simplex;
			return true;
		}
	}

	if (simplexOutput)
		*simplexOutput = simplex;
	return false;
}

}

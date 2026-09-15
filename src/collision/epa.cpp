#include <phys/collision/epa.hpp>
#include <phys/math/math_utils.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace phys {

namespace {

struct Face
{
	uint32_t a;
	uint32_t b;
	uint32_t c;
	Vec3 normal;
	float distance;
};

struct Edge
{
	uint32_t a;
	uint32_t b;
};

Face makeFace(const std::vector<Vec3>& vertices,
	uint32_t a, uint32_t b, uint32_t c)
{
	Vec3 normal = Math3d::cross(
		vertices[b] - vertices[a],
		vertices[c] - vertices[a]);
	float normalLength = Math3d::length(normal);
	if (normalLength > 1e-8f)
		normal = normal * (1.0f / normalLength);

	float distance = Math3d::dot(normal, vertices[a]);
	if (distance < 0.0f) {
		std::swap(b, c);
		normal = -normal;
		distance = -distance;
	}

	return {a, b, c, normal, distance};
}

void addHorizonEdge(std::vector<Edge>& horizon,
	uint32_t a, uint32_t b)
{
	for (auto iterator = horizon.begin(); iterator != horizon.end(); ++iterator) {
		if (iterator->a == b && iterator->b == a) {
			horizon.erase(iterator);
			return;
		}
	}

	horizon.push_back({a, b});
}

} // namespace

bool computeEpa(const SupportFunction& supportA,
	const SupportFunction& supportB, const GjkSimplex& simplex,
	EpaResult& result, uint32_t maxIterations, float tolerance)
{
	result = EpaResult{};
	if (simplex.size < 4 || maxIterations == 0)
		return false;

	std::vector<Vec3> vertices(
		simplex.points.begin(), simplex.points.begin() + simplex.size);
	std::vector<Face> faces;
	faces.reserve(32);
	faces.push_back(makeFace(vertices, 0, 1, 2));
	faces.push_back(makeFace(vertices, 0, 3, 1));
	faces.push_back(makeFace(vertices, 0, 2, 3));
	faces.push_back(makeFace(vertices, 1, 3, 2));

	for (uint32_t iteration = 0; iteration < maxIterations; ++iteration) {
		auto closest = std::min_element(faces.begin(), faces.end(),
			[](const Face& left, const Face& right) {
				return left.distance < right.distance;
			});
		if (closest == faces.end())
			return false;

		Face closestFace = *closest;
		Vec3 support = supportA(closestFace.normal)
			- supportB(-closestFace.normal);
		float supportDistance = Math3d::dot(
			support, closestFace.normal);

		if (supportDistance - closestFace.distance <= tolerance) {
			result.normal = closestFace.normal;
			result.depth = std::max(closestFace.distance, 0.0f);
			return true;
		}

		uint32_t supportIndex = static_cast<uint32_t>(vertices.size());
		vertices.push_back(support);

		std::vector<Edge> horizon;
		std::vector<Face> remainingFaces;
		remainingFaces.reserve(faces.size() + 8);

		for (const Face& face : faces) {
			float visibility = Math3d::dot(
				face.normal, support - vertices[face.a]);
			if (visibility > tolerance) {
				addHorizonEdge(horizon, face.a, face.b);
				addHorizonEdge(horizon, face.b, face.c);
				addHorizonEdge(horizon, face.c, face.a);
			} else {
				remainingFaces.push_back(face);
			}
		}

		if (horizon.empty())
			return false;

		faces = std::move(remainingFaces);
		for (const Edge& edge : horizon)
			faces.push_back(makeFace(vertices,
				edge.a, edge.b, supportIndex));
	}

	return false;
}

}

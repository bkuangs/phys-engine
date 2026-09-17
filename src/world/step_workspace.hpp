#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>
#include "phys/collision/aabb.hpp"
#include "phys/collision/broadphase.hpp"
#include "phys/collision/manifold.hpp"
#include "phys/math/mat3.hpp"

namespace phys {

class RigidBody;

namespace detail {

struct SweepEntry
{
    Aabb bounds;
    std::size_t originalIndex;
};

struct CellEntry
{
    std::array<int32_t, 3> cell;
    std::size_t index;
};

struct BroadPhaseWorkspace
{
    std::vector<SweepEntry> sweepEntries;
    std::vector<double> gridWidths;
    std::vector<CellEntry> gridEntries;
    std::vector<uint8_t> gridMembership;
    std::vector<std::size_t> gridOverflow;
};

void findCandidatePairs(const std::vector<Aabb>& bounds,
    std::vector<BroadPhasePair>& pairs, BroadPhaseStats* stats,
    BroadPhaseAlgorithm algorithm, BroadPhaseWorkspace& workspace);

struct PreparedBody
{
    RigidBody* body = nullptr;
    float inverseMass = 0.0f;
    Mat3 inverseInertiaWorld{};
    std::size_t generation = 0;
};

struct PreparedContactPoint
{
    Vec3 offsetA{};
    Vec3 offsetB{};
    Vec3 tangent1{};
    Vec3 tangent2{};
    Vec3 normalResponseA{};
    Vec3 normalResponseB{};
    Vec3 tangentResponseA1{};
    Vec3 tangentResponseB1{};
    Vec3 tangentResponseA2{};
    Vec3 tangentResponseB2{};
    float inverseEffectiveMass = 0.0f;
    float inverseTangentMass1 = 0.0f;
    float inverseTangentMass2 = 0.0f;
    float bias = 0.0f;
    float restitutionVelocity = 0.0f;
};

struct PreparedManifold
{
    ContactManifold* manifold = nullptr;
    PreparedBody* bodyA = nullptr;
    PreparedBody* bodyB = nullptr;
    std::array<PreparedContactPoint, 4> points{};
};

struct SolverWorkspace
{
    std::vector<PreparedBody> preparedBodies;
    std::vector<PreparedManifold> preparedContacts;
    std::vector<std::size_t> cacheOrder;
    std::size_t generation = 0;
};

}

}

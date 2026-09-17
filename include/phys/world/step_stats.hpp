#pragma once
#include <cstddef>
#include "phys/collision/broadphase.hpp"

namespace phys {

struct NarrowPhaseStats
{
    std::size_t sphereSphereCandidates = 0;
    std::size_t sphereBoxCandidates = 0;
    std::size_t boxBoxCandidates = 0;
    std::size_t sphereSphereContacts = 0;
    std::size_t sphereBoxContacts = 0;
    std::size_t boxBoxContacts = 0;
};

struct SolverStats
{
    double prepareMs = 0.0;
    double warmStartMs = 0.0;
    double velocityIterationsMs = 0.0;
    double cacheUpdateMs = 0.0;
    std::size_t preparedPoints = 0;
    std::size_t warmStartComparisons = 0;
    std::size_t warmStartMatches = 0;
    std::size_t velocityPointVisits = 0;
};

// Per-stage timings/counts for the most recent PhysicsWorld::step() call.
struct StepStats
{
    double totalMs = 0.0;
    double integrateVelocityMs = 0.0;
    double broadPhaseMs = 0.0;
    double narrowPhaseMs = 0.0;
    double solverMs = 0.0;
    double integratePoseMs = 0.0;

    double broadPhaseCollectMs = 0.0;
    double broadPhaseFilterMs = 0.0;
    BroadPhaseStats broadPhaseDetails{};
    NarrowPhaseStats narrowPhaseDetails{};
    SolverStats solverDetails{};

    std::size_t possiblePairs = 0; // All live, valid collider pairs on different bodies.
    std::size_t candidatePairs = 0;
    std::size_t contactCount = 0;
    std::size_t solvedContactCount = 0;
    std::size_t awakeBodyCount = 0; // Dynamic bodies, after the step.
    std::size_t sleepingBodyCount = 0;
};

}

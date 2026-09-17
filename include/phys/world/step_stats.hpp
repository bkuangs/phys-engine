#pragma once
#include <cstddef>
#include "phys/collision/broadphase.hpp"

namespace phys {

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

    std::size_t possiblePairs = 0; // All live, valid collider pairs on different bodies.
    std::size_t candidatePairs = 0;
    std::size_t contactCount = 0;
    std::size_t solvedContactCount = 0;
    std::size_t awakeBodyCount = 0; // Dynamic bodies, after the step.
    std::size_t sleepingBodyCount = 0;
};

}

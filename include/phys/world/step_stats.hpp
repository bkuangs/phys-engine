#pragma once
#include <cstddef>

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

    std::size_t possiblePairs = 0;
    std::size_t candidatePairs = 0;
    std::size_t contactCount = 0;
};

}

#pragma once
#include <cstdint>

namespace phys {

struct RigidBodyHandle
{
    uint32_t index = 0;
    uint32_t generation = 0;
};

inline bool operator==(const RigidBodyHandle& left, const RigidBodyHandle& right)
{
    return left.index == right.index && left.generation == right.generation;
}

}
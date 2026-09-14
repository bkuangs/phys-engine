#pragma once
#include <cstddef>

namespace phys::bench {

// Defined in alloc_counter.cpp, incremented by the overridden global operator new.
extern thread_local std::size_t g_allocCount;

// Scoped helper: count() returns allocations made since construction.
struct ScopedAllocCounter
{
    std::size_t before = g_allocCount;
    std::size_t count() const { return g_allocCount - before; }
};

}

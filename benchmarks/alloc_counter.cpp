#include "alloc_counter.hpp"
#include <cstdlib>
#include <new>

namespace phys::bench {

thread_local std::size_t g_allocCount = 0;

}

void* operator new(std::size_t size)
{
    ++phys::bench::g_allocCount;
    if (void* ptr = std::malloc(size)) return ptr;
    throw std::bad_alloc();
}

void operator delete(void* ptr) noexcept { std::free(ptr); }
void operator delete(void* ptr, std::size_t) noexcept { std::free(ptr); }

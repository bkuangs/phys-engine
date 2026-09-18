#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <condition_variable>
#include <exception>
#include <mutex>
#include <thread>
#include <vector>
#include "phys/collision/aabb.hpp"
#include "phys/collision/broadphase.hpp"
#include "phys/collision/manifold.hpp"
#include "phys/math/mat3.hpp"

namespace phys {

class RigidBody;

namespace detail {

class ParallelFor
{
public:
    using Task = void (*)(void*, std::size_t);

    ParallelFor() = default;
    ~ParallelFor() { stop(); }
    ParallelFor(const ParallelFor&) = delete;
    ParallelFor& operator=(const ParallelFor&) = delete;

    void configure(std::size_t workerCount)
    {
        const std::size_t backgroundCount = workerCount - 1;
        if (workers.size() == backgroundCount)
            return;

        stop();
        try
        {
            workers.reserve(backgroundCount);
            for (std::size_t index = 0; index < backgroundCount; ++index)
                workers.emplace_back([this, index] { workerLoop(index); });
        }
        catch (...)
        {
            stop();
            throw;
        }
    }

    template <typename Function>
    void run(std::size_t workerCount, std::size_t itemCount, Function& function,
             std::size_t chunkSize = 64)
    {
        const std::size_t backgroundCount = workerCount - 1;
        if (workers.size() < backgroundCount)
            configure(workerCount);
        if (backgroundCount == 0 || itemCount == 0)
        {
            for (std::size_t index = 0; index < itemCount; ++index)
                function(index);
            return;
        }

        auto invoke = [](void* context, std::size_t index) {
            (*static_cast<Function*>(context))(index);
        };
        {
            std::lock_guard lock(mutex);
            task = invoke;
            taskContext = &function;
            taskCount = itemCount;
            taskChunkSize = chunkSize;
            activeBackgroundWorkers = backgroundCount;
            nextIndex.store(0, std::memory_order_relaxed);
            unfinishedWorkers = backgroundCount;
            failure = nullptr;
            ++generation;
        }
        workAvailable.notify_all();
        runChunks();

        std::unique_lock lock(mutex);
        workFinished.wait(lock, [&] { return unfinishedWorkers == 0; });
        std::exception_ptr taskFailure = failure;
        lock.unlock();
        if (taskFailure)
            std::rethrow_exception(taskFailure);
    }

private:
    void runChunks()
    {
        try
        {
            for (;;)
            {
                std::size_t begin = nextIndex.fetch_add(
                    taskChunkSize, std::memory_order_relaxed);
                if (begin >= taskCount)
                    return;
                std::size_t end = std::min(begin + taskChunkSize, taskCount);
                for (std::size_t index = begin; index < end; ++index)
                    task(taskContext, index);
            }
        }
        catch (...)
        {
            std::lock_guard lock(mutex);
            if (!failure)
                failure = std::current_exception();
            nextIndex.store(taskCount, std::memory_order_relaxed);
        }
    }

    void workerLoop(std::size_t workerIndex)
    {
        std::size_t observedGeneration = 0;
        for (;;)
        {
            std::unique_lock lock(mutex);
            workAvailable.wait(lock, [&] {
                return stopping || generation != observedGeneration;
            });
            if (stopping)
                return;
            observedGeneration = generation;
            const bool active = workerIndex < activeBackgroundWorkers;
            lock.unlock();

            if (active)
                runChunks();

            lock.lock();
            if (active && --unfinishedWorkers == 0)
                workFinished.notify_one();
        }
    }

    void stop()
    {
        {
            std::lock_guard lock(mutex);
            stopping = true;
        }
        workAvailable.notify_all();
        for (std::thread& worker : workers)
            if (worker.joinable())
                worker.join();
        workers.clear();
        stopping = false;
        generation = 0;
    }

    std::vector<std::thread> workers;
    std::mutex mutex;
    std::condition_variable workAvailable;
    std::condition_variable workFinished;
    std::atomic<std::size_t> nextIndex = 0;
    Task task = nullptr;
    void* taskContext = nullptr;
    std::size_t taskCount = 0;
    std::size_t taskChunkSize = 1;
    std::size_t activeBackgroundWorkers = 0;
    std::size_t unfinishedWorkers = 0;
    std::size_t generation = 0;
    std::exception_ptr failure;
    bool stopping = false;
};

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

struct SweepRange
{
    std::vector<BroadPhasePair> pairs;
    std::size_t comparisons = 0;
};

struct BroadPhaseWorkspace
{
    std::vector<SweepEntry> sweepEntries;
    std::vector<SweepRange> sweepRanges;
    std::vector<BroadPhasePair> pairSortBuffer;
    std::vector<std::size_t> pairSortCounts;
    std::vector<double> gridWidths;
    std::vector<CellEntry> gridEntries;
    std::vector<uint8_t> gridMembership;
    std::vector<std::size_t> gridOverflow;
};

void findCandidatePairs(const std::vector<Aabb>& bounds,
    std::vector<BroadPhasePair>& pairs, BroadPhaseStats* stats,
    BroadPhaseAlgorithm algorithm, BroadPhaseWorkspace& workspace,
    ParallelFor* workers = nullptr, std::size_t workerCount = 1);

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
    float inverseTangentMass00 = 0.0f;
    float inverseTangentMass01 = 0.0f;
    float inverseTangentMass11 = 0.0f;
    float normalTangentResponse1 = 0.0f;
    float normalTangentResponse2 = 0.0f;
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
    std::vector<std::size_t> preparedPointCounts;
    std::vector<std::size_t> cacheOrder;
    std::vector<std::size_t> islandParents;
    std::vector<std::size_t> islandLookup;
    std::vector<std::size_t> contactIslands;
    std::vector<std::size_t> islandContactCounts;
    std::vector<std::size_t> islandPointCounts;
    std::vector<std::size_t> islandOffsets;
    std::vector<std::size_t> islandWriteOffsets;
    std::vector<std::size_t> islandContactIndices;
    std::vector<std::size_t> islandOrder;
    std::vector<std::size_t> islandWarmStartComparisons;
    std::vector<std::size_t> islandWarmStartMatches;
    std::size_t generation = 0;
};

}

}

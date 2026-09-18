#include <phys/world/physics_world.hpp>
#include <phys/collision/broadphase.hpp>
#include <phys/collision/narrowphase.hpp>
#include <phys/solver/sequential_impulse_solver.hpp>
#include "step_workspace.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <thread>

namespace phys
{

    namespace
    {

        using Clock = std::chrono::steady_clock;

        double elapsedMs(Clock::time_point start)
        {
            return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
        }

        bool sameVector(const Vec3 &a, const Vec3 &b)
        {
            return a.x == b.x && a.y == b.y && a.z == b.z;
        }

        bool samePose(const Transform &a, const Transform &b)
        {
            return sameVector(a.position, b.position)
                && a.orientation.x == b.orientation.x && a.orientation.y == b.orientation.y
                && a.orientation.z == b.orientation.z && a.orientation.w == b.orientation.w;
        }

        bool sameColliderGeometry(const Collider &a, const Collider &b)
        {
            if (a.body != b.body || !samePose(a.localTransform, b.localTransform)
                || a.shape.index() != b.shape.index())
                return false;
            if (const auto *sphere = std::get_if<Sphere>(&a.shape))
                return sphere->radius == std::get<Sphere>(b.shape).radius;
            return sameVector(std::get<Box>(a.shape).halfExtents, std::get<Box>(b.shape).halfExtents);
        }

        class ParallelFor
        {
        public:
            using Task = void (*)(void *, std::size_t);

            ~ParallelFor()
            {
                stop();
            }

            ParallelFor(const ParallelFor &) = delete;
            ParallelFor &operator=(const ParallelFor &) = delete;

            ParallelFor() = default;

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
                        workers.emplace_back([this] { workerLoop(); });
                }
                catch (...)
                {
                    stop();
                    throw;
                }
            }

            template <typename Function>
            void run(std::size_t workerCount, std::size_t itemCount, Function &function)
            {
                configure(workerCount);
                if (workers.empty() || itemCount == 0)
                {
                    for (std::size_t index = 0; index < itemCount; ++index)
                        function(index);
                    return;
                }

                auto invoke = [](void *context, std::size_t index) {
                    (*static_cast<Function *>(context))(index);
                };
                {
                    std::lock_guard lock(mutex);
                    task = invoke;
                    taskContext = &function;
                    taskCount = itemCount;
                    nextIndex.store(0, std::memory_order_relaxed);
                    unfinishedWorkers = workers.size();
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
            static constexpr std::size_t chunkSize = 64;

            void runChunks()
            {
                try
                {
                    for (;;)
                    {
                        std::size_t begin = nextIndex.fetch_add(
                            chunkSize, std::memory_order_relaxed);
                        if (begin >= taskCount)
                            return;
                        std::size_t end = std::min(begin + chunkSize, taskCount);
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

            void workerLoop()
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
                    lock.unlock();

                    runChunks();

                    lock.lock();
                    if (--unfinishedWorkers == 0)
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
                for (std::thread &worker : workers)
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
            void *taskContext = nullptr;
            std::size_t taskCount = 0;
            std::size_t unfinishedWorkers = 0;
            std::size_t generation = 0;
            std::exception_ptr failure;
            bool stopping = false;
        };

        struct NarrowPhaseResult
        {
            ContactManifold manifold{};
            std::uint8_t shapePair = 0;
            bool evaluated = false;
            bool hasContact = false;
        };

    }

    struct PhysicsWorld::StepWorkspace
    {
        std::vector<Aabb> bounds;
        std::vector<uint32_t> colliderIndices;
        std::vector<std::size_t> collidersPerBody;
        std::vector<BroadPhasePair> candidatePairs;
        std::vector<NarrowPhaseResult> narrowPhaseResults;
        std::vector<std::pair<DynamicAabbTree::ProxyId, DynamicAabbTree::ProxyId>> treeStack;
        detail::BroadPhaseWorkspace broadPhase;
        detail::SolverWorkspace solver;
        ParallelFor narrowPhaseWorkers;
    };

    PhysicsWorld::PhysicsWorld() = default;
    PhysicsWorld::~PhysicsWorld() = default;

    PhysicsWorld::PhysicsWorld(const PhysicsWorld &other)
        : gravity(other.gravity),
          broadPhaseAlgorithm(other.broadPhaseAlgorithm),
          slots(other.slots),
          freeList(other.freeList),
          colliderSlots(other.colliderSlots),
          dynamicTree(other.dynamicTree),
          freeColliderList(other.freeColliderList),
          currentContacts(other.currentContacts),
          cachedContacts(other.cachedContacts),
          stats(other.stats),
          narrowPhaseWorkerCount(other.narrowPhaseWorkerCount),
          sleepingEnabled(other.sleepingEnabled),
          sleepGravity(other.sleepGravity),
          sleepStates(other.sleepStates),
          sleepColliders(other.sleepColliders)
    {
    }

    PhysicsWorld &PhysicsWorld::operator=(const PhysicsWorld &other)
    {
        if (this == &other)
            return *this;

        gravity = other.gravity;
        broadPhaseAlgorithm = other.broadPhaseAlgorithm;
        narrowPhaseWorkerCount = other.narrowPhaseWorkerCount;
        slots = other.slots;
        freeList = other.freeList;
        colliderSlots = other.colliderSlots;
        dynamicTree = other.dynamicTree;
        freeColliderList = other.freeColliderList;
        currentContacts = other.currentContacts;
        cachedContacts = other.cachedContacts;
        std::vector<CachedContact>().swap(nextCachedContacts);
        stats = other.stats;
        stepWorkspace.reset();
        sleepingEnabled = other.sleepingEnabled;
        sleepGravity = other.sleepGravity;
        sleepStates = other.sleepStates;
        sleepColliders = other.sleepColliders;
        return *this;
    }

    PhysicsWorld::PhysicsWorld(PhysicsWorld &&other) noexcept = default;
    PhysicsWorld &PhysicsWorld::operator=(PhysicsWorld &&other) noexcept = default;

    PhysicsWorld::StepWorkspace &PhysicsWorld::workspace()
    {
        if (!stepWorkspace)
            stepWorkspace = std::make_unique<StepWorkspace>();
        return *stepWorkspace;
    }

    void PhysicsWorld::setNarrowPhaseWorkerCount(std::size_t count)
    {
        if (count == 0)
            throw std::invalid_argument("Narrowphase worker count must be positive");
        if (stepWorkspace)
            stepWorkspace->narrowPhaseWorkers.configure(count);
        narrowPhaseWorkerCount = count;
    }

    RigidBodyHandle PhysicsWorld::addBody(const RigidBody &body)
    {
        if (!freeList.empty())
        { // re-use deleted bodies
            uint32_t index = freeList.back();
            freeList.pop_back();

            Slot &slot = slots[index];
            slot.body = body;
            slot.body.wakeUp();
            slot.alive = true;
            return {index, slot.generation};
        }

        slots.push_back(Slot{body, 0, true});
        slots.back().body.wakeUp();
        return {static_cast<uint32_t>(slots.size() - 1), 0};
    }

    bool PhysicsWorld::createSphere(float radius, Vec3 position, float density,
                                    bool isStatic, float restitution, float friction, RigidBodyHandle &body,
                                    ColliderHandle &collider, std::string &errorMessage)
    {
        RigidBody createdBody;
        if (!RigidBody::createSphere(radius, position, density, isStatic,
                                     restitution, friction, createdBody, errorMessage))
        {
            return false;
        }

        body = addBody(createdBody);

        // Create global collision geometry and link Collider to RigidBody.
        // Tells the engine that a new sphere is physically occupying space.
        Collider createdCollider;
        createdCollider.body = body;
        createdCollider.shape = Sphere{radius};
        collider = addCollider(createdCollider);
        return true;
    }

    bool PhysicsWorld::createBox(float width, float height, float depth, Vec3 position,
                                 float density, bool isStatic, float restitution, float friction,
                                 RigidBodyHandle &body,
                                 ColliderHandle &collider, std::string &errorMessage)
    {
        RigidBody createdBody;
        if (!RigidBody::createBox(width, height, depth, position, density, isStatic,
                                  restitution, friction, createdBody, errorMessage))
        {
            return false;
        }

        body = addBody(createdBody);
        Collider createdCollider;
        createdCollider.body = body;
        createdCollider.shape = Box{{width * 0.5f, height * 0.5f, depth * 0.5f}};
        collider = addCollider(createdCollider);
        return true;
    }

    void PhysicsWorld::removeBody(RigidBodyHandle handle)
    {
        if (handle.index >= slots.size())
            return; // index out of bounds

        Slot &slot = slots[handle.index];

        // Check that the slot is being used and reject old handles
        if (!slot.alive || slot.generation != handle.generation)
            return;

        wakeContacts(handle);
        slot.alive = false;
        slot.generation++;
        freeList.push_back(handle.index);

        // Remove all colliders belonging to this body
        for (uint32_t index = 0; index < colliderSlots.size(); ++index)
        {
            ColliderSlot &colliderSlot = colliderSlots[index];
            if (colliderSlot.alive && colliderSlot.collider.body == handle)
            {
                colliderSlot.alive = false;
                colliderSlot.generation++;
                freeColliderList.push_back(index);
            }
        }
    }

    RigidBody *PhysicsWorld::getBody(RigidBodyHandle handle)
    {
        if (handle.index >= slots.size())
            return nullptr;

        Slot &slot = slots[handle.index];
        if (!slot.alive || slot.generation != handle.generation)
            return nullptr;

        return &slot.body;
    }

    const RigidBody *PhysicsWorld::getBody(RigidBodyHandle handle) const
    {
        if (handle.index >= slots.size())
            return nullptr;

        const Slot &slot = slots[handle.index];
        if (!slot.alive || slot.generation != handle.generation)
            return nullptr;

        return &slot.body;
    }

    ColliderHandle PhysicsWorld::addCollider(const Collider &collider)
    {
        RigidBody *body = getBody(collider.body);
        if (!body)
            return {};
        body->wakeUp();

        if (!freeColliderList.empty())
        {
            uint32_t index = freeColliderList.back();
            freeColliderList.pop_back();

            ColliderSlot &slot = colliderSlots[index];
            slot.collider = collider;
            slot.alive = true;
            if (index < sleepColliders.size())
                sleepColliders[index] = collider;
            return {index, slot.generation};
        }

        colliderSlots.push_back(ColliderSlot{collider, 0, true});
        return {static_cast<uint32_t>(colliderSlots.size() - 1), 0};
    }

    void PhysicsWorld::removeCollider(ColliderHandle handle)
    {
        if (handle.index >= colliderSlots.size())
            return;

        ColliderSlot &slot = colliderSlots[handle.index];
        if (!slot.alive || slot.generation != handle.generation)
            return;

        if (RigidBody *body = getBody(slot.collider.body))
            body->wakeUp();
        wakeContacts(slot.collider.body);
        slot.alive = false;
        slot.generation++;
        freeColliderList.push_back(handle.index);
    }

    Collider *PhysicsWorld::getCollider(ColliderHandle handle)
    {
        if (handle.index >= colliderSlots.size())
            return nullptr;

        ColliderSlot &slot = colliderSlots[handle.index];
        if (!slot.alive || slot.generation != handle.generation)
            return nullptr;

        return &slot.collider;
    }

    const Collider *PhysicsWorld::getCollider(ColliderHandle handle) const
    {
        if (handle.index >= colliderSlots.size())
            return nullptr;

        const ColliderSlot &slot = colliderSlots[handle.index];
        if (!slot.alive || slot.generation != handle.generation)
            return nullptr;

        return &slot.collider;
    }

    void PhysicsWorld::setSleepingEnabled(bool enabled)
    {
        if (sleepingEnabled == enabled)
            return;
        sleepingEnabled = enabled;
        sleepStates.clear();
        sleepColliders.clear();
        for (Slot &slot : slots)
            if (slot.alive)
                slot.body.wakeUp();
    }

    void PhysicsWorld::wakeContacts(RigidBodyHandle body)
    {
        if (!sleepingEnabled)
            return;
        for (const ContactManifold &contact : currentContacts)
        {
            if (contact.bodyA == body)
            {
                RigidBody *other = getBody(contact.bodyB);
                if (other && !other->isStatic)
                    other->wakeUp();
            }
            if (contact.bodyB == body)
            {
                RigidBody *other = getBody(contact.bodyA);
                if (other && !other->isStatic)
                    other->wakeUp();
            }
        }
    }

    uint32_t PhysicsWorld::sleepRoot(uint32_t index)
    {
        while (sleepStates[index].parent != index)
        {
            sleepStates[index].parent = sleepStates[sleepStates[index].parent].parent;
            index = sleepStates[index].parent;
        }
        return index;
    }

    void PhysicsWorld::wakeSleepIslands()
    {
        for (uint32_t index = 0; index < slots.size(); ++index)
        {
            SleepState &state = sleepStates[index];
            state.parent = index;
            state.hasAwake = state.hasSleeping = state.needsWake = false;
        }
        for (const ContactManifold &contact : currentContacts)
        {
            RigidBody *a = getBody(contact.bodyA);
            RigidBody *b = getBody(contact.bodyB);
            if (!a || !b || contact.pointCount == 0)
                continue;
            // A shared static floor must not join otherwise independent islands.
            if (!a->isStatic && !b->isStatic)
            {
                uint32_t rootA = sleepRoot(contact.bodyA.index);
                uint32_t rootB = sleepRoot(contact.bodyB.index);
                sleepStates[std::max(rootA, rootB)].parent = std::min(rootA, rootB);
            }
            else if (a->isStatic && a->wakeRequested && !b->isStatic)
                b->wakeUp();
            else if (b->isStatic && b->wakeRequested && !a->isStatic)
                a->wakeUp();
        }
        for (uint32_t index = 0; index < slots.size(); ++index)
        {
            const Slot &slot = slots[index];
            if (!slot.alive || slot.body.isStatic)
                continue;
            SleepState &island = sleepStates[sleepRoot(index)];
            island.hasAwake |= !slot.body.isSleeping();
            island.hasSleeping |= slot.body.isSleeping();
            island.needsWake |= slot.body.wakeRequested;
        }
        for (uint32_t index = 0; index < slots.size(); ++index)
        {
            Slot &slot = slots[index];
            if (!slot.alive || slot.body.isStatic)
                continue;
            const SleepState &island = sleepStates[sleepRoot(index)];
            if (island.needsWake || (island.hasAwake && island.hasSleeping))
            {
                slot.body.wakeUp();
                sleepStates[index].quietTime = 0.0f;
            }
        }
    }

    void PhysicsWorld::prepareSleeping()
    {
        sleepStates.resize(slots.size());
        std::size_t previousColliderCount = sleepColliders.size();
        sleepColliders.resize(colliderSlots.size());
        bool gravityChanged = !sameVector(gravity, sleepGravity);
        for (uint32_t index = 0; index < slots.size(); ++index)
        {
            Slot &slot = slots[index];
            if (!slot.alive)
                continue;
            const SleepState &previous = sleepStates[index];
            const RigidBody &body = slot.body;
            if (gravityChanged || previous.mass != body.mass
                || previous.isStatic != body.isStatic || previous.friction != body.friction
                || previous.restitution != body.restitution
                || !samePose(previous.pose, {body.getPosition(), body.getRotation()}))
                slot.body.wakeUp();
        }
        // Collider fields are publicly mutable, so compare geometry, not getter access.
        for (uint32_t index = 0; index < colliderSlots.size(); ++index)
        {
            const ColliderSlot &slot = colliderSlots[index];
            if (!slot.alive || sameColliderGeometry(slot.collider, sleepColliders[index]))
                continue;
            if (index < previousColliderCount)
                if (RigidBody *oldBody = getBody(sleepColliders[index].body))
                    oldBody->wakeUp();
            if (RigidBody *body = getBody(slot.collider.body))
                body->wakeUp();
        }
        wakeSleepIslands();
    }

    void PhysicsWorld::finishSleeping(float dt)
    {
        constexpr float quietSpeedSquared = 0.05f * 0.05f;
        constexpr float timeToSleep = 0.5f;
        for (SleepState &state : sleepStates)
            state.islandQuietTime = std::numeric_limits<float>::max();
        for (uint32_t index = 0; index < slots.size(); ++index)
        {
            Slot &slot = slots[index];
            if (!slot.alive || slot.body.isStatic)
                continue;
            SleepState &state = sleepStates[index];
            const RigidBody &body = slot.body;
            if (!body.isSleeping())
            {
                if (!body.wakeRequested && dt > 0.0f && std::isfinite(dt)
                    && Math3d::dot(body.linearVelocity, body.linearVelocity) <= quietSpeedSquared
                    && Math3d::dot(body.angularVelocity, body.angularVelocity) <= quietSpeedSquared)
                    state.quietTime = std::min(state.quietTime + dt, timeToSleep);
                else
                    state.quietTime = 0.0f;
            }
            SleepState &island = sleepStates[sleepRoot(index)];
            island.islandQuietTime = std::min(island.islandQuietTime, state.quietTime);
        }
        for (uint32_t index = 0; index < slots.size(); ++index)
        {
            Slot &slot = slots[index];
            if (!slot.alive)
                continue;
            RigidBody &body = slot.body;
            if (!body.isStatic && sleepStates[sleepRoot(index)].islandQuietTime >= timeToSleep)
            {
                body.sleeping = true;
                body.linearVelocity = body.angularVelocity = Vec3::zero();
            }
            SleepState &state = sleepStates[index];
            state.pose = {body.getPosition(), body.getRotation()};
            state.mass = body.mass;
            state.isStatic = body.isStatic;
            state.friction = body.friction;
            state.restitution = body.restitution;
            body.wakeRequested = false;
        }
        for (uint32_t index = 0; index < colliderSlots.size(); ++index)
            if (colliderSlots[index].alive)
                sleepColliders[index] = colliderSlots[index].collider;
        sleepGravity = gravity;
    }

    void PhysicsWorld::step(float dt)
    {
        auto stepStart = Clock::now();

        if (sleepingEnabled)
            prepareSleeping();

        auto velocityStart = Clock::now();

        // Apply gravity to active bodies
        for (uint32_t index = 0; index < slots.size(); ++index)
        {
            Slot &slot = slots[index];
            if (!slot.alive)
                continue;
            if (sleepingEnabled)
                sleepStates[index].integratedVelocity = !slot.body.isSleeping();
            slot.body.integrateVelocity(gravity, dt);
        }
        stats.integrateVelocityMs = elapsedMs(velocityStart);

        // Get world-space collider AABBs from current body poses
        for (ColliderSlot &slot : colliderSlots)
        {
            if (!slot.alive)
                continue;

            const RigidBody *body = getBody(slot.collider.body);
            if (!body)
                continue;

            Transform bodyTransform{body->getPosition(), body->getRotation()};
            slot.collider.bounds = Aabb::fromCollider(slot.collider, bodyTransform);
        }

        // BROAD-PHASE: Collect AABB-overlapping collider pairs as candidates.
        auto broadPhaseStart = Clock::now();
        StepWorkspace &stepWorkspace = workspace();
        auto &bounds = stepWorkspace.bounds;
        auto &colliderIndices = stepWorkspace.colliderIndices;
        auto &collidersPerBody = stepWorkspace.collidersPerBody;
        auto &candidatePairs = stepWorkspace.candidatePairs;
        bounds.clear();
        colliderIndices.clear();
        collidersPerBody.assign(slots.size(), 0);
        candidatePairs.clear();
        if (bounds.capacity() < colliderSlots.size())
            bounds.reserve(colliderSlots.size());
        if (colliderIndices.capacity() < colliderSlots.size())
            colliderIndices.reserve(colliderSlots.size());
        std::size_t possiblePairs = 0;

        for (uint32_t index = 0; index < colliderSlots.size(); ++index)
        {
            const ColliderSlot &slot = colliderSlots[index];
            if (!slot.alive || !getBody(slot.collider.body))
                continue;

            // Count eligible all-pairs without retaining a quadratic statistics loop.
            possiblePairs += bounds.size() - collidersPerBody[slot.collider.body.index];
            ++collidersPerBody[slot.collider.body.index];
            bounds.push_back(slot.collider.bounds);
            colliderIndices.push_back(index);
        }
        stats.broadPhaseCollectMs = elapsedMs(broadPhaseStart);
        if (broadPhaseAlgorithm == BroadPhaseAlgorithm::DynamicTree)
        {
            auto maintenanceStart = Clock::now();
            std::size_t insertions = 0;
            std::size_t removals = 0;
            std::size_t reinsertions = 0;
            for (uint32_t index = 0; index < colliderSlots.size(); ++index)
            {
                ColliderSlot &slot = colliderSlots[index];
                bool valid = slot.alive && getBody(slot.collider.body);
                if (slot.treeProxy != DynamicAabbTree::noProxy
                    && (!valid || slot.treeGeneration != slot.generation))
                {
                    dynamicTree.destroyProxy(slot.treeProxy);
                    slot.treeProxy = DynamicAabbTree::noProxy;
                    ++removals;
                }
                if (!valid)
                    continue;
                if (slot.treeProxy == DynamicAabbTree::noProxy)
                {
                    slot.treeProxy = dynamicTree.createProxy(slot.collider.bounds, index);
                    slot.treeGeneration = slot.generation;
                    ++insertions;
                }
                else if (dynamicTree.updateProxy(slot.treeProxy, slot.collider.bounds))
                    ++reinsertions;
            }
            double maintenanceMs = elapsedMs(maintenanceStart);
            dynamicTree.findCandidatePairs(
                candidatePairs, stepWorkspace.treeStack, &stats.broadPhaseDetails);
            stats.broadPhaseDetails.recordBuildMs = maintenanceMs;
            stats.broadPhaseDetails.treeInsertions = insertions;
            stats.broadPhaseDetails.treeRemovals = removals;
            stats.broadPhaseDetails.treeReinsertions = reinsertions;
        }
        else
            detail::findCandidatePairs(bounds, candidatePairs,
                &stats.broadPhaseDetails, broadPhaseAlgorithm, stepWorkspace.broadPhase);
        auto filterStart = Clock::now();
        if (broadPhaseAlgorithm != BroadPhaseAlgorithm::DynamicTree)
            for (BroadPhasePair &pair : candidatePairs)
            {
                pair.first = colliderIndices[pair.first];
                pair.second = colliderIndices[pair.second];
            }
        std::erase_if(candidatePairs, [&](const BroadPhasePair &pair) {
            return colliderSlots[pair.first].collider.body ==
                   colliderSlots[pair.second].collider.body;
        });
        stats.broadPhaseFilterMs = elapsedMs(filterStart);
        stats.broadPhaseMs = elapsedMs(broadPhaseStart);
        stats.possiblePairs = possiblePairs;
        stats.candidatePairs = candidatePairs.size();

        // NARROW-PHASE: Each candidate pair gets delegated to shape-specific query.
        auto narrowPhaseStart = Clock::now();
        currentContacts.clear();
        NarrowPhaseStats narrowPhaseDetails;

        const PhysicsWorld &readOnlyWorld = *this;
        auto generateContact = [&](std::size_t pairIndex, NarrowPhaseResult &result)
        {
            result.evaluated = false;
            result.hasContact = false;
            const BroadPhasePair &pair = candidatePairs[pairIndex];
            const ColliderSlot &firstSlot = colliderSlots[pair.first];
            const ColliderSlot &secondSlot = colliderSlots[pair.second];

            const RigidBody *bodyA = readOnlyWorld.getBody(firstSlot.collider.body);
            const RigidBody *bodyB = readOnlyWorld.getBody(secondSlot.collider.body);
            if (!bodyA || !bodyB)
                return;

            Transform bodyTransformA{bodyA->getPosition(), bodyA->getRotation()};
            Transform bodyTransformB{bodyB->getPosition(), bodyB->getRotation()};
            Transform transformA{
                bodyTransformA.position + bodyTransformA.orientation.rotate(
                                              firstSlot.collider.localTransform.position),
                bodyTransformA.orientation * firstSlot.collider.localTransform.orientation};
            Transform transformB{
                bodyTransformB.position + bodyTransformB.orientation.rotate(
                                              secondSlot.collider.localTransform.position),
                bodyTransformB.orientation * secondSlot.collider.localTransform.orientation};

            result.evaluated = true;
            result.shapePair = static_cast<std::uint8_t>(
                firstSlot.collider.shape.index() + secondSlot.collider.shape.index());
            result.hasContact = NarrowPhase::generateContact(
                firstSlot.collider, transformA,
                secondSlot.collider, transformB,
                result.manifold);
            if (!result.hasContact)
                return;

            for (uint32_t index = 0; index < result.manifold.pointCount; ++index)
            {
                ContactPoint &point = result.manifold.points[index];
                point.localAnchorA = firstSlot.collider.localTransform.position
                    + firstSlot.collider.localTransform.orientation.rotate(point.localAnchorA);
                point.localAnchorB = secondSlot.collider.localTransform.position
                    + secondSlot.collider.localTransform.orientation.rotate(point.localAnchorB);
            }
            result.manifold.restitution = std::max(
                bodyA->restitution, bodyB->restitution);
            result.manifold.friction = std::sqrt(
                std::max(bodyA->friction, 0.0f) * std::max(bodyB->friction, 0.0f));
        };
        auto appendResult = [&](const NarrowPhaseResult &result)
        {
            if (!result.evaluated)
                return;
            if (result.shapePair == 0)
                ++narrowPhaseDetails.sphereSphereCandidates;
            else if (result.shapePair == 1)
                ++narrowPhaseDetails.sphereBoxCandidates;
            else
                ++narrowPhaseDetails.boxBoxCandidates;

            if (!result.hasContact)
                return;
            currentContacts.push_back(result.manifold);
            if (result.shapePair == 0)
                ++narrowPhaseDetails.sphereSphereContacts;
            else if (result.shapePair == 1)
                ++narrowPhaseDetails.sphereBoxContacts;
            else
                ++narrowPhaseDetails.boxBoxContacts;
        };

        constexpr std::size_t minimumParallelCandidates = 4096;
        if (narrowPhaseWorkerCount == 1
            || candidatePairs.size() < minimumParallelCandidates)
        {
            NarrowPhaseResult result;
            for (std::size_t index = 0; index < candidatePairs.size(); ++index)
            {
                generateContact(index, result);
                appendResult(result);
            }
        }
        else
        {
            auto &results = stepWorkspace.narrowPhaseResults;
            results.resize(candidatePairs.size());
            auto generateAtIndex = [&](std::size_t index) {
                generateContact(index, results[index]);
            };
            stepWorkspace.narrowPhaseWorkers.run(
                narrowPhaseWorkerCount, candidatePairs.size(), generateAtIndex);
            for (const NarrowPhaseResult &result : results)
                appendResult(result);
        }
        stats.narrowPhaseDetails = narrowPhaseDetails;
        stats.narrowPhaseMs = elapsedMs(narrowPhaseStart);
        stats.contactCount = currentContacts.size();

        if (sleepingEnabled)
        {
            wakeSleepIslands();
            // Contacts can wake a previously sleeping island after gravity integration.
            for (uint32_t index = 0; index < slots.size(); ++index)
                if (slots[index].alive && !sleepStates[index].integratedVelocity
                    && !slots[index].body.isSleeping())
                    slots[index].body.integrateVelocity(gravity, dt);
        }

        auto solverStart = Clock::now();
        SequentialImpulseSolver::solve(currentContacts, *this, dt, stepWorkspace.solver);
        stats.solverMs = elapsedMs(solverStart);

        // Pose-integration: advance each active body by its current velocities.
        auto poseStart = Clock::now();
        for (Slot &slot : slots)
        {
            if (!slot.alive)
                continue;

            slot.body.integratePosition(dt);
            slot.body.integrateRotation(dt);
            slot.body.clearForces();
        }
        stats.integratePoseMs = elapsedMs(poseStart);

        if (sleepingEnabled)
            finishSleeping(dt);
        stats.awakeBodyCount = stats.sleepingBodyCount = 0;
        for (const Slot &slot : slots)
            if (slot.alive && !slot.body.isStatic)
            {
                if (slot.body.isSleeping())
                    ++stats.sleepingBodyCount;
                else
                    ++stats.awakeBodyCount;
            }
        stats.totalMs = elapsedMs(stepStart);
    }

}

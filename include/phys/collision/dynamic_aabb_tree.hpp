#pragma once
#include <phys/collision/broadphase.hpp>
#include <limits>
#include <utility>
#include <vector>

namespace phys {

class PhysicsWorld;

class DynamicAabbTree
{
public:
    using ProxyId = std::size_t;
    static constexpr ProxyId noProxy = std::numeric_limits<ProxyId>::max();

    DynamicAabbTree() = default;
    DynamicAabbTree(const DynamicAabbTree&) = default;
    DynamicAabbTree& operator=(const DynamicAabbTree&) = default;
    DynamicAabbTree(DynamicAabbTree&& other) noexcept;
    DynamicAabbTree& operator=(DynamicAabbTree&& other) noexcept;

    // User indices must be unique among live proxies. Bounds must be finite and ordered.
    // Proxy IDs are invalid after destruction and may be recycled by the node pool.
    ProxyId createProxy(const Aabb& bounds, std::size_t userIndex);
    void destroyProxy(ProxyId proxy);
    bool updateProxy(ProxyId proxy, const Aabb& bounds); // True when reinserted.
    // Returns sorted, unique user-index pairs after checking tight bounds.
    std::vector<BroadPhasePair> findCandidatePairs(BroadPhaseStats* stats = nullptr) const;
    std::size_t size() const { return proxyCount; }
    int height() const { return root == noProxy ? 0 : nodes[root].height; }

private:
    struct Node
    {
        Aabb bounds{};
        Aabb tightBounds{};
        ProxyId parent = noProxy;
        ProxyId left = noProxy;
        ProxyId right = noProxy;
        std::size_t userIndex = 0;
        int height = -1;

        bool isLeaf() const { return left == noProxy; }
    };

    std::vector<Node> nodes;
    ProxyId root = noProxy;
    ProxyId freeHead = noProxy;
    std::size_t proxyCount = 0;

    ProxyId allocateNode();
    void releaseNode(ProxyId node);
    void requireProxy(ProxyId proxy) const;
    void insertLeaf(ProxyId leaf);
    void detachLeaf(ProxyId leaf);
    void refit(ProxyId node);
    void refitAncestors(ProxyId node);
    void replaceChild(ProxyId parent, ProxyId oldChild, ProxyId newChild);
    ProxyId lowerChild(ProxyId branch, ProxyId sibling) const;
    ProxyId rotateLeft(ProxyId node);
    ProxyId rotateRight(ProxyId node);
    ProxyId balance(ProxyId node);
    void findCandidatePairs(std::vector<BroadPhasePair>& pairs,
        std::vector<std::pair<ProxyId, ProxyId>>& stack,
        BroadPhaseStats* stats) const;

    friend class PhysicsWorld;
};

}

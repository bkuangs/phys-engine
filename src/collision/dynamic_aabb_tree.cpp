#include <phys/collision/dynamic_aabb_tree.hpp>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace phys {

namespace {

Aabb combined(const Aabb& a, const Aabb& b)
{
    return {{std::min(a.min.x, b.min.x), std::min(a.min.y, b.min.y), std::min(a.min.z, b.min.z)},
            {std::max(a.max.x, b.max.x), std::max(a.max.y, b.max.y), std::max(a.max.z, b.max.z)}};
}

double area(const Aabb& bounds)
{
    double x = static_cast<double>(bounds.max.x) - bounds.min.x;
    double y = static_cast<double>(bounds.max.y) - bounds.min.y;
    double z = static_cast<double>(bounds.max.z) - bounds.min.z;
    return 2.0 * (x * y + x * z + y * z);
}

bool contains(const Aabb& outer, const Aabb& inner)
{
    return outer.min.x <= inner.min.x && outer.max.x >= inner.max.x
        && outer.min.y <= inner.min.y && outer.max.y >= inner.max.y
        && outer.min.z <= inner.min.z && outer.max.z >= inner.max.z;
}

void validateBounds(const Aabb& bounds)
{
    if (!(std::isfinite(bounds.min.x) && std::isfinite(bounds.min.y)
        && std::isfinite(bounds.min.z) && std::isfinite(bounds.max.x)
        && std::isfinite(bounds.max.y) && std::isfinite(bounds.max.z)
        && bounds.min.x <= bounds.max.x && bounds.min.y <= bounds.max.y
        && bounds.min.z <= bounds.max.z))
        throw std::invalid_argument("Dynamic AABB tree requires finite, ordered bounds");
}

Aabb expanded(const Aabb& bounds, double factor = 1.0)
{
    auto axis = [factor](float lower, float upper) {
        double padding = std::max(0.01, (static_cast<double>(upper) - lower) * 0.1) * factor;
        double limit = std::numeric_limits<float>::max();
        return std::array<float, 2>{
            static_cast<float>(std::max(-limit, static_cast<double>(lower) - padding)),
            static_cast<float>(std::min(limit, static_cast<double>(upper) + padding))};
    };
    auto x = axis(bounds.min.x, bounds.max.x);
    auto y = axis(bounds.min.y, bounds.max.y);
    auto z = axis(bounds.min.z, bounds.max.z);
    return {{x[0], y[0], z[0]}, {x[1], y[1], z[1]}};
}

}

DynamicAabbTree::DynamicAabbTree(DynamicAabbTree&& other) noexcept
    : nodes(std::move(other.nodes)), root(std::exchange(other.root, noProxy)),
      freeHead(std::exchange(other.freeHead, noProxy)),
      proxyCount(std::exchange(other.proxyCount, 0))
{
}

DynamicAabbTree& DynamicAabbTree::operator=(DynamicAabbTree&& other) noexcept
{
    if (this != &other) {
        nodes = std::move(other.nodes);
        root = std::exchange(other.root, noProxy);
        freeHead = std::exchange(other.freeHead, noProxy);
        proxyCount = std::exchange(other.proxyCount, 0);
    }
    return *this;
}

DynamicAabbTree::ProxyId DynamicAabbTree::allocateNode()
{
    ProxyId node;
    if (freeHead != noProxy) {
        node = freeHead;
        freeHead = nodes[node].parent;
        nodes[node] = Node{};
    }
    else {
        node = nodes.size();
        nodes.emplace_back();
    }
    nodes[node].height = 0;
    return node;
}

void DynamicAabbTree::releaseNode(ProxyId node)
{
    nodes[node] = Node{};
    nodes[node].parent = freeHead;
    freeHead = node;
}

void DynamicAabbTree::requireProxy(ProxyId proxy) const
{
    if (proxy >= nodes.size() || nodes[proxy].height != 0 || !nodes[proxy].isLeaf())
        throw std::invalid_argument("Invalid dynamic AABB tree proxy");
}

DynamicAabbTree::ProxyId DynamicAabbTree::createProxy(const Aabb& bounds, std::size_t userIndex)
{
    validateBounds(bounds);
    ProxyId proxy = allocateNode();
    nodes[proxy].bounds = expanded(bounds);
    nodes[proxy].tightBounds = bounds;
    nodes[proxy].userIndex = userIndex;
    insertLeaf(proxy);
    ++proxyCount;
    return proxy;
}

void DynamicAabbTree::destroyProxy(ProxyId proxy)
{
    requireProxy(proxy);
    detachLeaf(proxy);
    releaseNode(proxy);
    --proxyCount;
}

bool DynamicAabbTree::updateProxy(ProxyId proxy, const Aabb& bounds)
{
    requireProxy(proxy);
    validateBounds(bounds);
    nodes[proxy].tightBounds = bounds;
    // Shrink stale fat bounds as well as updating proxies that move outside them.
    if (contains(nodes[proxy].bounds, bounds) && contains(expanded(bounds, 4.0), nodes[proxy].bounds))
        return false;
    detachLeaf(proxy);
    nodes[proxy].bounds = expanded(bounds);
    insertLeaf(proxy);
    return true;
}

void DynamicAabbTree::refit(ProxyId node)
{
    Node& current = nodes[node];
    current.bounds = combined(nodes[current.left].bounds, nodes[current.right].bounds);
    current.height = 1 + std::max(nodes[current.left].height, nodes[current.right].height);
}

void DynamicAabbTree::replaceChild(ProxyId parent, ProxyId oldChild, ProxyId newChild)
{
    if (parent == noProxy)
        root = newChild;
    else if (nodes[parent].left == oldChild)
        nodes[parent].left = newChild;
    else
        nodes[parent].right = newChild;
    nodes[newChild].parent = parent;
}

DynamicAabbTree::ProxyId DynamicAabbTree::lowerChild(ProxyId branch, ProxyId sibling) const
{
    ProxyId left = nodes[branch].left;
    ProxyId right = nodes[branch].right;
    if (nodes[left].height != nodes[right].height)
        return nodes[left].height < nodes[right].height ? left : right;
    return area(combined(nodes[left].bounds, nodes[sibling].bounds))
        <= area(combined(nodes[right].bounds, nodes[sibling].bounds)) ? left : right;
}

DynamicAabbTree::ProxyId DynamicAabbTree::rotateLeft(ProxyId node)
{
    ProxyId promoted = nodes[node].right;
    // Child order is spatial, not a search-key order: keep taller groups higher,
    // and minimize the new bounding area when heights tie.
    ProxyId middle = lowerChild(promoted, nodes[node].left);
    ProxyId upper = nodes[promoted].left == middle ? nodes[promoted].right : nodes[promoted].left;
    replaceChild(nodes[node].parent, node, promoted);
    nodes[promoted].left = node;
    nodes[promoted].right = upper;
    nodes[upper].parent = promoted;
    nodes[node].parent = promoted;
    nodes[node].right = middle;
    nodes[middle].parent = node;
    refit(node);
    refit(promoted);
    return promoted;
}

DynamicAabbTree::ProxyId DynamicAabbTree::rotateRight(ProxyId node)
{
    ProxyId promoted = nodes[node].left;
    ProxyId middle = lowerChild(promoted, nodes[node].right);
    ProxyId upper = nodes[promoted].left == middle ? nodes[promoted].right : nodes[promoted].left;
    replaceChild(nodes[node].parent, node, promoted);
    nodes[promoted].right = node;
    nodes[promoted].left = upper;
    nodes[upper].parent = promoted;
    nodes[node].parent = promoted;
    nodes[node].left = middle;
    nodes[middle].parent = node;
    refit(node);
    refit(promoted);
    return promoted;
}

DynamicAabbTree::ProxyId DynamicAabbTree::balance(ProxyId node)
{
    ProxyId left = nodes[node].left;
    ProxyId right = nodes[node].right;
    int difference = nodes[right].height - nodes[left].height;
    if (difference > 1)
        return rotateLeft(node);
    if (difference < -1)
        return rotateRight(node);
    return node;
}

void DynamicAabbTree::refitAncestors(ProxyId node)
{
    while (node != noProxy) {
        refit(node);
        node = balance(node);
        node = nodes[node].parent;
    }
}

void DynamicAabbTree::insertLeaf(ProxyId leaf)
{
    if (root == noProxy) {
        root = leaf;
        nodes[leaf].parent = noProxy;
        return;
    }
    ProxyId sibling = root;
    while (!nodes[sibling].isLeaf()) {
        ProxyId left = nodes[sibling].left;
        ProxyId right = nodes[sibling].right;
        double leftCost = area(combined(nodes[left].bounds, nodes[leaf].bounds)) - area(nodes[left].bounds);
        double rightCost = area(combined(nodes[right].bounds, nodes[leaf].bounds)) - area(nodes[right].bounds);
        sibling = leftCost <= rightCost ? left : right;
    }
    ProxyId oldParent = nodes[sibling].parent;
    ProxyId parent = allocateNode();
    nodes[parent].left = sibling;
    nodes[parent].right = leaf;
    replaceChild(oldParent, sibling, parent);
    nodes[sibling].parent = parent;
    nodes[leaf].parent = parent;
    refitAncestors(parent);
}

void DynamicAabbTree::detachLeaf(ProxyId leaf)
{
    if (root == leaf) {
        root = noProxy;
        nodes[leaf].parent = noProxy;
        return;
    }
    ProxyId parent = nodes[leaf].parent;
    ProxyId grandparent = nodes[parent].parent;
    ProxyId sibling = nodes[parent].left == leaf ? nodes[parent].right : nodes[parent].left;
    replaceChild(grandparent, parent, sibling);
    releaseNode(parent);
    nodes[leaf].parent = noProxy;
    refitAncestors(grandparent);
}

std::vector<BroadPhasePair> DynamicAabbTree::findCandidatePairs(BroadPhaseStats* stats) const
{
    std::vector<BroadPhasePair> pairs;
    std::vector<std::pair<ProxyId, ProxyId>> stack;
    findCandidatePairs(pairs, stack, stats);
    return pairs;
}

void DynamicAabbTree::findCandidatePairs(std::vector<BroadPhasePair>& pairs,
    std::vector<std::pair<ProxyId, ProxyId>>& stack, BroadPhaseStats* stats) const
{
    using Clock = std::chrono::steady_clock;
    auto queryStart = Clock::now();
    pairs.clear();
    stack.clear();
    std::size_t visits = 0;
    std::size_t leafChecks = 0;
    if (root != noProxy) {
        const auto requiredCapacity = static_cast<std::size_t>(height()) * 4 + 1;
        if (stack.capacity() < requiredCapacity)
            stack.reserve(requiredCapacity);
        stack.emplace_back(root, root);
    }
    while (!stack.empty()) {
        auto [first, second] = stack.back();
        stack.pop_back();
        ++visits;
        const Node& a = nodes[first];
        const Node& b = nodes[second];
        // Partition unordered leaf pairs into disjoint within-child and cross-child sets.
        if (first == second) {
            if (!a.isLeaf()) {
                stack.emplace_back(a.left, a.left);
                stack.emplace_back(a.right, a.right);
                stack.emplace_back(a.left, a.right);
            }
            continue;
        }
        if (!a.bounds.overlaps(b.bounds))
            continue;
        if (a.isLeaf() && b.isLeaf()) {
            ++leafChecks;
            if (a.tightBounds.overlaps(b.tightBounds))
                pairs.push_back({std::min(a.userIndex, b.userIndex), std::max(a.userIndex, b.userIndex)});
        }
        else if (!a.isLeaf() && (b.isLeaf() || area(a.bounds) >= area(b.bounds))) {
            stack.emplace_back(a.left, second);
            stack.emplace_back(a.right, second);
        }
        else {
            stack.emplace_back(first, b.left);
            stack.emplace_back(first, b.right);
        }
    }
    auto sortStart = Clock::now();
    std::sort(pairs.begin(), pairs.end(), [](const auto& left, const auto& right) {
        return left.first != right.first ? left.first < right.first : left.second < right.second;
    });
    if (stats) {
        *stats = {};
        stats->sweepMs = std::chrono::duration<double, std::milli>(sortStart - queryStart).count();
        stats->pairSortMs = std::chrono::duration<double, std::milli>(Clock::now() - sortStart).count();
        stats->aabbPairs = pairs.size();
        stats->treeNodePairVisits = visits;
        stats->treeLeafChecks = leafChecks;
        stats->treeHeight = static_cast<std::size_t>(height());
        stats->treeProxyCount = proxyCount;
    }
}

}

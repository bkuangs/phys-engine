#include <phys/collision/dynamic_aabb_tree.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

namespace {

using Tree = phys::DynamicAabbTree;
static_assert(std::is_same_v<Tree::ProxyId, std::size_t>);

struct Proxy
{
    Tree::ProxyId id = Tree::noProxy;
    phys::Aabb bounds{};
    std::size_t userIndex = 0;
};

bool matchesBruteForce(const Tree& tree, const std::vector<Proxy>& proxies)
{
    std::vector<phys::BroadPhasePair> expected;
    std::size_t live = 0;
    for (std::size_t first = 0; first < proxies.size(); ++first) {
        if (proxies[first].id == Tree::noProxy)
            continue;
        ++live;
        for (std::size_t second = first + 1; second < proxies.size(); ++second)
            if (proxies[second].id != Tree::noProxy
                && proxies[first].bounds.overlaps(proxies[second].bounds))
                expected.push_back({std::min(proxies[first].userIndex, proxies[second].userIndex),
                                    std::max(proxies[first].userIndex, proxies[second].userIndex)});
    }
    std::sort(expected.begin(), expected.end(), [](const auto& a, const auto& b) {
        return a.first != b.first ? a.first < b.first : a.second < b.second;
    });
    phys::BroadPhaseStats stats;
    stats.treeNodePairVisits = 999;
    auto actual = tree.findCandidatePairs(&stats);
    if (actual.size() != expected.size()
        || !std::equal(actual.begin(), actual.end(), expected.begin(),
            [](const auto& a, const auto& b) { return a.first == b.first && a.second == b.second; })) {
        std::cerr << "dynamic tree pairs differ from brute force\n";
        return false;
    }
    if (tree.size() != live || stats.treeProxyCount != live || stats.aabbPairs != expected.size()
        || stats.treeLeafChecks < expected.size()
        || tree.height() > 2.0 * std::ceil(std::log2(static_cast<double>(live) + 1.0))
        || !(std::isfinite(stats.sweepMs) && stats.sweepMs >= 0.0
             && std::isfinite(stats.pairSortMs) && stats.pairSortMs >= 0.0)
        || (live == 0 && (stats.treeNodePairVisits != 0 || tree.height() != 0))) {
        std::cerr << "invalid dynamic tree counts, balance, or timings\n";
        return false;
    }
    return true;
}

bool testFatBoundsAndResizing()
{
    Tree tree;
    std::vector<Proxy> proxies{
        {Tree::noProxy, {{0, 0, 0}, {1, 1, 1}}, 7},
        {Tree::noProxy, {{1.05f, 0, 0}, {2.05f, 1, 1}}, 2}};
    for (auto& proxy : proxies)
        proxy.id = tree.createProxy(proxy.bounds, proxy.userIndex);
    if (!matchesBruteForce(tree, proxies) || !tree.findCandidatePairs().empty())
        return false;
    proxies[1].bounds = {{0.98f, 0, 0}, {1.98f, 1, 1}};
    if (tree.updateProxy(proxies[1].id, proxies[1].bounds)
        || !matchesBruteForce(tree, proxies) || tree.findCandidatePairs().size() != 1) {
        std::cerr << "movement within fat bounds did not refresh tight bounds\n";
        return false;
    }
    proxies[1].bounds = {{10, 0, 0}, {11, 1, 1}};
    if (!tree.updateProxy(proxies[1].id, proxies[1].bounds) || !matchesBruteForce(tree, proxies))
        return false;
    proxies[1].bounds = {{0, 0, 0}, {100, 100, 100}};
    if (!tree.updateProxy(proxies[1].id, proxies[1].bounds) || !matchesBruteForce(tree, proxies))
        return false;
    proxies[1].bounds = {{10, 0, 0}, {11, 1, 1}};
    if (!tree.updateProxy(proxies[1].id, proxies[1].bounds)
        || tree.updateProxy(proxies[1].id, proxies[1].bounds)
        || !matchesBruteForce(tree, proxies)) {
        std::cerr << "resized proxy retained oversized fat bounds or reinserted while stationary\n";
        return false;
    }
    return true;
}

bool testBalancedLifecycle()
{
    Tree tree;
    std::vector<Proxy> proxies;
    for (std::size_t index = 0; index < 512; ++index) {
        float x = static_cast<float>(index) * 0.7f;
        phys::Aabb bounds{{x, 0, 0}, {x + 1, 1, 1}};
        std::size_t userIndex = (511 - index) * 17;
        proxies.push_back({tree.createProxy(bounds, userIndex), bounds, userIndex});
    }
    if (!matchesBruteForce(tree, proxies))
        return false;

    std::mt19937 rng(123);
    std::uniform_real_distribution<float> position(-12.0f, 12.0f);
    std::uniform_real_distribution<float> size(0.01f, 2.0f);
    for (int frame = 0; frame < 80; ++frame) {
        for (std::size_t index = 0; index < proxies.size(); ++index) {
            auto& proxy = proxies[index];
            if ((index + static_cast<std::size_t>(frame)) % 19 == 0) {
                if (proxy.id != Tree::noProxy) {
                    tree.destroyProxy(proxy.id);
                    proxy.id = Tree::noProxy;
                }
                else
                    proxy.id = tree.createProxy(proxy.bounds, proxy.userIndex);
            }
            if (proxy.id == Tree::noProxy)
                continue;
            phys::Vec3 center{position(rng), position(rng), position(rng)};
            phys::Vec3 extent{size(rng), size(rng), size(rng)};
            proxy.bounds = {center - extent, center + extent};
            tree.updateProxy(proxy.id, proxy.bounds);
        }
        if (!matchesBruteForce(tree, proxies))
            return false;
    }
    const auto snapshot = proxies;
    Tree copy = tree;
    Tree assigned;
    assigned.createProxy({{1000, 1000, 1000}, {1001, 1001, 1001}}, 1);
    assigned = tree;
    if (!matchesBruteForce(assigned, proxies))
        return false;
    Tree moved = std::move(copy);
    if (copy.size() != 0 || !copy.findCandidatePairs().empty()
        || !matchesBruteForce(moved, proxies))
        return false;
    copy = std::move(moved);
    if (moved.size() != 0 || !moved.findCandidatePairs().empty()
        || !matchesBruteForce(copy, proxies))
        return false;
    for (auto& proxy : proxies) {
        if (proxy.id != Tree::noProxy)
            tree.destroyProxy(proxy.id);
        proxy.id = Tree::noProxy;
    }
    if (!matchesBruteForce(tree, proxies) || !matchesBruteForce(copy, snapshot)
        || !matchesBruteForce(assigned, snapshot))
        return false;
    proxies[0].id = tree.createProxy(proxies[0].bounds, proxies[0].userIndex);
    return matchesBruteForce(tree, proxies);
}

bool testProxyReuseAndWideIds()
{
    Tree tree;
    std::vector<Proxy> proxies{
        {Tree::noProxy, {{0, 0, 0}, {1, 1, 1}}, 10},
        {Tree::noProxy, {{10, 0, 0}, {11, 1, 1}}, 20},
        {Tree::noProxy, {{0.5f, 0, 0}, {1.5f, 1, 1}}, std::numeric_limits<std::size_t>::max() - 1}};
    for (auto& proxy : proxies)
        proxy.id = tree.createProxy(proxy.bounds, proxy.userIndex);
    if (!matchesBruteForce(tree, proxies))
        return false;

    tree.destroyProxy(proxies[0].id);
    proxies[0].id = Tree::noProxy;
    proxies[2].bounds = {{0.55f, 0, 0}, {1.55f, 1, 1}};
    if (tree.updateProxy(proxies[2].id, proxies[2].bounds)
        || !matchesBruteForce(tree, proxies)) {
        std::cerr << "removing another proxy invalidated a live proxy\n";
        return false;
    }
    proxies[0].bounds = {{10.5f, 0, 0}, {11.5f, 1, 1}};
    proxies[0].id = tree.createProxy(proxies[0].bounds, proxies[0].userIndex);
    tree.destroyProxy(proxies[1].id);
    proxies[1].bounds = {{0.6f, 0, 0}, {1.6f, 1, 1}};
    proxies[1].id = tree.createProxy(proxies[1].bounds, proxies[1].userIndex);
    if (!matchesBruteForce(tree, proxies))
        return false;

    if constexpr (sizeof(Tree::ProxyId) > sizeof(std::uint32_t)) {
        Tree::ProxyId truncatedAlias = static_cast<Tree::ProxyId>(
            std::numeric_limits<std::uint32_t>::max()) + 1 + proxies[0].id;
        try {
            tree.updateProxy(truncatedAlias, {{50, 50, 50}, {51, 51, 51}});
            std::cerr << "wide invalid proxy was truncated to a live node\n";
            return false;
        }
        catch (const std::invalid_argument&) {}
    }
    return matchesBruteForce(tree, proxies);
}

bool testInvalidCalls()
{
    Tree tree;
    try {
        tree.createProxy({{1, 0, 0}, {0, 1, 1}}, 0);
        std::cerr << "inverted bounds were accepted\n";
        return false;
    }
    catch (const std::invalid_argument&) {}
    auto proxy = tree.createProxy({{0, 0, 0}, {1, 1, 1}}, 0);
    try {
        tree.updateProxy(proxy, {{std::numeric_limits<float>::quiet_NaN(), 0, 0}, {1, 1, 1}});
        std::cerr << "NaN bounds were accepted\n";
        return false;
    }
    catch (const std::invalid_argument&) {}
    tree.destroyProxy(proxy);
    try {
        tree.destroyProxy(proxy);
        std::cerr << "destroyed proxy was accepted\n";
        return false;
    }
    catch (const std::invalid_argument&) {}
    try {
        phys::BroadPhase::findCandidatePairs({}, nullptr, phys::BroadPhaseAlgorithm::DynamicTree);
        std::cerr << "stateless tree query was accepted\n";
        return false;
    }
    catch (const std::invalid_argument&) {}
    return tree.size() == 0 && tree.findCandidatePairs().empty();
}

}

int main()
{
    return testFatBoundsAndResizing() && testBalancedLifecycle()
        && testProxyReuseAndWideIds() && testInvalidCalls() ? 0 : 1;
}

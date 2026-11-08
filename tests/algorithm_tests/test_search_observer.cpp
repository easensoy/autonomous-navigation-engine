#include "pathfinding_algorithms/AStar.hpp"
#include "pathfinding_algorithms/BFS.hpp"
#include "pathfinding_algorithms/Dijkstra.hpp"
#include "pathfinding_algorithms/SearchObserver.hpp"
#include "core/Graph.hpp"
#include <algorithm>
#include <cmath>
#include <exception>
#include <functional>
#include <iostream>
#include <random>
#include <string>
#include <unordered_set>
#include <vector>

// Checks the opt-in SearchObserver hooks: attaching one never changes a route, and the events it
// receives arrive in the order each algorithm really executes them.

namespace {

class Recorder : public SearchObserver {
public:
    struct Expand { int node; double cost; double priority; };
    struct Push { int node; int parent; double cost; double priority; };
    std::vector<Expand> expands;
    std::vector<Push> pushes;
    std::vector<std::pair<bool, size_t>> order;  // (is expansion, index into expands or pushes)

    void onExpand(int nodeId, double costSoFar, double priority) override {
        order.emplace_back(true, expands.size());
        expands.push_back({nodeId, costSoFar, priority});
    }
    void onFrontierPush(int nodeId, int parentId, double costSoFar, double priority) override {
        order.emplace_back(false, pushes.size());
        pushes.push_back({nodeId, parentId, costSoFar, priority});
    }
    void clear() {
        expands.clear();
        pushes.clear();
        order.clear();
    }
};

bool nondecreasingPriority(const std::vector<Recorder::Expand>& expands) {
    for (size_t i = 1; i < expands.size(); ++i) {
        if (expands[i].priority < expands[i - 1].priority - 1e-9) {
            return false;
        }
    }
    return true;
}

// The first event is the push of the start (parent -1), and every later push names a parent that
// has already been expanded, in the order the events arrived.
bool parentsExpandedFirst(const Recorder& r) {
    if (r.order.empty() || r.order.front().first || r.pushes.front().parent != -1) {
        return false;
    }
    std::unordered_set<int> expanded;
    for (size_t k = 1; k < r.order.size(); ++k) {
        const auto& [isExpansion, index] = r.order[k];
        if (isExpansion) {
            expanded.insert(r.expands[index].node);
        } else if (expanded.count(r.pushes[index].parent) == 0) {
            return false;
        }
    }
    return true;
}

class SearchObserverTester {
private:
    Graph grid;
    int testsRun = 0;
    int testsPassed = 0;
    static constexpr int side = 14;

    // 14 x 14 grid, 8-connected, a few nodes removed; edge cost = length x a random factor in [1, 3).
    void buildGrid() {
        std::mt19937 rng(20261108);
        std::uniform_real_distribution<double> factor(1.0, 3.0);
        for (int y = 0; y < side; ++y) {
            for (int x = 0; x < side; ++x) {
                const bool wall = (x == 6 && y > 1 && y < 12) || (x == 9 && y > 3 && y < 13 && y != 8);
                if (!wall) {
                    grid.addNode(y * side + x, "n" + std::to_string(y * side + x), x * 10.0, y * 10.0);
                }
            }
        }
        const int offsets[4][2] = {{1, 0}, {0, 1}, {1, 1}, {-1, 1}};
        for (int y = 0; y < side; ++y) {
            for (int x = 0; x < side; ++x) {
                const int from = y * side + x;
                if (!grid.hasNode(from)) continue;
                for (const auto& o : offsets) {
                    const int nx = x + o[0], ny = y + o[1];
                    if (nx < 0 || nx >= side || ny >= side) continue;
                    const int to = ny * side + nx;
                    if (!grid.hasNode(to)) continue;
                    const double length = 10.0 * std::hypot(o[0], o[1]);
                    grid.addEdge(from, to, length * factor(rng), true);
                }
            }
        }
    }

    std::vector<std::pair<int, int>> queries() const {
        std::vector<std::pair<int, int>> q;
        std::mt19937 rng(7);
        std::vector<int> ids = grid.getAllNodeIds();
        std::sort(ids.begin(), ids.end());
        std::uniform_int_distribution<size_t> pick(0, ids.size() - 1);
        for (int k = 0; k < 40; ++k) {
            q.emplace_back(ids[pick(rng)], ids[pick(rng)]);
        }
        return q;
    }

    void runTest(const std::string& testName, std::function<bool()> test) {
        testsRun++;
        std::cout << "[TEST] " << testName << ": ";
        try {
            if (test()) {
                testsPassed++;
                std::cout << "PASS" << std::endl;
            } else {
                std::cout << "FAIL" << std::endl;
            }
        } catch (const std::exception& e) {
            std::cout << "EXCEPTION: " << e.what() << std::endl;
        }
    }

public:
    void runAllTests() {
        std::cout << "\n=== Search Observer Tests ===" << std::endl;
        buildGrid();

        runTest("A*: attaching an observer leaves every route unchanged", [this]() {
            AStar plain(&grid), traced(&grid);
            Recorder r;
            traced.setSearchObserver(&r);
            for (const auto& [s, t] : queries()) {
                if (plain.findPath(s, t) != traced.findPath(s, t)) return false;
            }
            return !r.expands.empty();
        });

        runTest("A*: one expansion event per node counted by getLastSearchNodesExplored", [this]() {
            AStar astar(&grid);
            Recorder r;
            astar.setSearchObserver(&r);
            for (const auto& [s, t] : queries()) {
                r.clear();
                astar.findPath(s, t);
                if (r.expands.size() != astar.getLastSearchNodesExplored()) return false;
            }
            return true;
        });

        runTest("A*: default heuristic expands in nondecreasing f, ending at the goal", [this]() {
            AStar astar(&grid);
            Recorder r;
            astar.setSearchObserver(&r);
            for (const auto& [s, t] : queries()) {
                r.clear();
                const std::vector<int> route = astar.findPath(s, t);
                if (!nondecreasingPriority(r.expands)) return false;
                if (!route.empty() && r.expands.back().node != t) return false;
                if (!route.empty() && std::fabs(r.expands.back().cost - astar.getPathCost(route)) > 1e-9) return false;
            }
            return true;
        });

        runTest("A*: the start is pushed first and every parent is expanded", [this]() {
            AStar astar(&grid);
            Recorder r;
            astar.setSearchObserver(&r);
            for (const auto& [s, t] : queries()) {
                r.clear();
                astar.findPath(s, t);
                if (r.pushes.front().node != s || !parentsExpandedFirst(r)) return false;
            }
            return true;
        });

        runTest("A* with h = 0 expands in nondecreasing cost and matches Dijkstra", [this]() {
            AStar zero(&grid);
            zero.setHeuristicFunction([](int, int) { return 0.0; });
            Dijkstra dijkstra(&grid);
            Recorder r;
            zero.setSearchObserver(&r);
            for (const auto& [s, t] : queries()) {
                r.clear();
                const std::vector<int> route = zero.findPath(s, t);
                const double best = dijkstra.getShortestDistance(s, t);
                if (!nondecreasingPriority(r.expands)) return false;
                if (route.empty() != std::isinf(best)) return false;
                if (!route.empty() && std::fabs(zero.getPathCost(route) - best) > 1e-9 * std::max(1.0, best)) return false;
            }
            return true;
        });

        runTest("Dijkstra: same routes, settles each reachable node once at its final distance", [this]() {
            Dijkstra plain(&grid), traced(&grid);
            Recorder r;
            traced.setSearchObserver(&r);
            for (const auto& [s, t] : queries()) {
                r.clear();
                if (plain.findShortestPath(s, t) != traced.findShortestPath(s, t)) return false;
                const auto distances = plain.findShortestDistances(s);
                size_t reachable = 0;
                for (const auto& d : distances) reachable += std::isfinite(d.second) ? 1 : 0;
                if (r.expands.size() != reachable) return false;
                std::unordered_set<int> seen;
                for (const auto& e : r.expands) {
                    if (!seen.insert(e.node).second) return false;
                    if (std::fabs(e.cost - distances.at(e.node)) > 1e-9) return false;
                }
                if (!nondecreasingPriority(r.expands)) return false;
            }
            return true;
        });

        runTest("BFS: same routes, dequeues in nondecreasing depth, never dequeues the goal", [this]() {
            BFS plain(&grid), traced(&grid);
            Recorder r;
            traced.setSearchObserver(&r);
            for (const auto& [s, t] : queries()) {
                if (s == t) continue;
                r.clear();
                const std::vector<int> route = traced.findPath(s, t);
                if (plain.findPath(s, t) != route) return false;
                if (!nondecreasingPriority(r.expands)) return false;
                for (const auto& e : r.expands) {
                    if (e.node == t) return false;
                }
                // The goal was pushed with depth = number of edges in the returned route.
                const auto goalPush = std::find_if(r.pushes.begin(), r.pushes.end(), [t](const Recorder::Push& p) { return p.node == t; });
                if (!route.empty() && (goalPush == r.pushes.end() || goalPush->cost != double(route.size() - 1))) return false;
            }
            return true;
        });

        runTest("Detaching the observer stops events", [this]() {
            AStar astar(&grid);
            Recorder r;
            astar.setSearchObserver(&r);
            astar.findPath(0, side * side - 1);
            const size_t before = r.expands.size();
            astar.setSearchObserver(nullptr);
            astar.findPath(0, side * side - 1);
            return before > 0 && r.expands.size() == before;
        });

        runTest("A* reports every expansion of an exhaustive search for an unreachable goal", []() {
            Graph g;
            for (int i = 0; i < 6; ++i) g.addNode(i, "n" + std::to_string(i), i * 10.0, 0.0);
            g.addEdge(0, 1, 10.0);
            g.addEdge(1, 2, 10.0);
            g.addEdge(3, 4, 10.0);
            g.addEdge(4, 5, 10.0);
            AStar astar(&g);
            Recorder r;
            astar.setSearchObserver(&r);
            const std::vector<int> route = astar.findPath(0, 5);
            return route.empty() && r.expands.size() == 3 && astar.getLastSearchNodesExplored() == 3;
        });

        std::cout << "\nSearch observer tests: " << testsPassed << "/" << testsRun << " passed" << std::endl;
    }

    bool allPassed() const {
        return testsRun > 0 && testsPassed == testsRun;
    }
};

}  // namespace

int main() {
    SearchObserverTester tester;
    tester.runAllTests();
    return tester.allPassed() ? 0 : 1;
}

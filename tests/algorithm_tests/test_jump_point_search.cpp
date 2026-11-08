#include "pathfinding_algorithms/JumpPointSearch.hpp"
#include "pathfinding_algorithms/Dijkstra.hpp"
#include "core/Graph.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <exception>
#include <functional>
#include <iostream>
#include <limits>
#include <set>
#include <stdexcept>
#include <streambuf>
#include <string>
#include <utility>
#include <vector>

// Jump Point Search is specified for uniform-cost grid graphs without corner cutting (see
// JumpPointSearch.hpp). These tests check the domain check, the regressions found before the
// rewrite, marked cells, and a seeded differential comparison with Dijkstra. Every returned route is
// validated step by step: endpoints, graph edges, grid moves, no corner cutting, no marked cells, and
// a cost equal to Dijkstra's least cost.

namespace {

// Graph::addNode and addEdge print every insertion; large fixtures are built with output silenced.
class CoutSilencer {
public:
    CoutSilencer() : saved(std::cout.rdbuf(nullptr)) {}
    ~CoutSilencer() {
        std::cout.rdbuf(saved);
        std::cout.clear();
    }
    CoutSilencer(const CoutSilencer&) = delete;
    CoutSilencer& operator=(const CoutSilencer&) = delete;

private:
    std::streambuf* saved;
};

// Portable seeded generator (std::uniform_*_distribution differs between standard libraries).
struct SplitMix64 {
    std::uint64_t state;
    std::uint64_t next() {
        std::uint64_t z = (state += 0x9E3779B97F4A7C15ULL);
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
        return z ^ (z >> 31);
    }
    int below(int n) { return static_cast<int>(next() % static_cast<std::uint64_t>(n)); }
    double unit() { return static_cast<double>(next() >> 11) * (1.0 / 9007199254740992.0); }
};

struct Grid {
    int width = 0;
    int height = 0;
    std::vector<char> free;  // a node stands on the cell
    bool diagonal = true;

    bool isFree(int x, int y) const {
        return x >= 0 && y >= 0 && x < width && y < height && free[static_cast<std::size_t>(y) * width + x];
    }
    int id(int x, int y) const { return y * width + x; }
    int x(int id) const { return id % width; }
    int y(int id) const { return id / width; }
};

// The graph of the movement model: orthogonal moves of weight step, and (if enabled) diagonal moves
// of weight step * sqrt(2) whose two side cells are free. Cells in `closed` are left out.
Graph buildModelGraph(const Grid& grid, double step, const std::set<std::pair<int, int>>& closed = {}) {
    CoutSilencer quiet;
    auto open = [&](int x, int y) { return grid.isFree(x, y) && closed.count({x, y}) == 0; };
    Graph graph;
    for (int y = 0; y < grid.height; ++y) {
        for (int x = 0; x < grid.width; ++x) {
            if (open(x, y)) {
                graph.addNode(grid.id(x, y), "c", x, y);
            }
        }
    }
    for (int y = 0; y < grid.height; ++y) {
        for (int x = 0; x < grid.width; ++x) {
            if (!open(x, y)) {
                continue;
            }
            if (open(x + 1, y)) graph.addEdge(grid.id(x, y), grid.id(x + 1, y), step);
            if (open(x, y + 1)) graph.addEdge(grid.id(x, y), grid.id(x, y + 1), step);
            if (grid.diagonal) {
                if (open(x + 1, y + 1) && open(x + 1, y) && open(x, y + 1)) {
                    graph.addEdge(grid.id(x, y), grid.id(x + 1, y + 1), step * std::sqrt(2.0));
                }
                if (open(x - 1, y + 1) && open(x - 1, y) && open(x, y + 1)) {
                    graph.addEdge(grid.id(x, y), grid.id(x - 1, y + 1), step * std::sqrt(2.0));
                }
            }
        }
    }
    return graph;
}

// Rows are given top first: rows[0] is y = height - 1. '#' blocked, 'S' start, 'G' goal.
Grid parseGrid(const std::vector<std::string>& rows, bool diagonal, int& start, int& goal) {
    Grid grid;
    grid.height = static_cast<int>(rows.size());
    grid.width = static_cast<int>(rows.front().size());
    grid.diagonal = diagonal;
    grid.free.assign(static_cast<std::size_t>(grid.width) * grid.height, 1);
    for (int row = 0; row < grid.height; ++row) {
        const int y = grid.height - 1 - row;
        for (int x = 0; x < grid.width; ++x) {
            const char c = rows[row][x];
            grid.free[static_cast<std::size_t>(y) * grid.width + x] = c == '#' ? 0 : 1;
            if (c == 'S') start = grid.id(x, y);
            if (c == 'G') goal = grid.id(x, y);
        }
    }
    return grid;
}

bool nearlyEqual(double a, double b) {
    return std::fabs(a - b) <= 1e-9 * std::max({1.0, std::fabs(a), std::fabs(b)});
}

// Checks a JPS route against the grid model and the reference graph; returns an empty string when
// valid, otherwise what is wrong.
std::string routeProblem(const Grid& grid, const Graph& searched, const std::vector<int>& route, int start, int goal,
                         const std::set<std::pair<int, int>>& closed) {
    if (route.empty() || route.front() != start || route.back() != goal) {
        return "wrong endpoints";
    }
    auto open = [&](int x, int y) { return grid.isFree(x, y) && closed.count({x, y}) == 0; };
    for (int node : route) {
        if (!open(grid.x(node), grid.y(node))) {
            return "enters closed cell " + std::to_string(node);
        }
    }
    for (std::size_t i = 1; i < route.size(); ++i) {
        const int ax = grid.x(route[i - 1]), ay = grid.y(route[i - 1]);
        const int bx = grid.x(route[i]), by = grid.y(route[i]);
        const int dx = bx - ax, dy = by - ay;
        if (std::max(std::abs(dx), std::abs(dy)) != 1) {
            return "step is not to a neighbouring cell";
        }
        if (dx != 0 && dy != 0 && (!grid.diagonal || !open(ax + dx, ay) || !open(ax, ay + dy))) {
            return "diagonal step cuts a corner or is not allowed";
        }
        if (!searched.hasEdge(route[i - 1], route[i])) {
            return "step is not an edge of the searched graph";
        }
    }
    return "";
}

double routeCost(const Graph& graph, const std::vector<int>& route) {
    double cost = 0.0;
    for (std::size_t i = 1; i < route.size(); ++i) {
        cost += graph.getEdgeWeight(route[i - 1], route[i]);
    }
    return cost;
}

// Runs JPS and Dijkstra on a parsed regression map and requires an optimal, valid route.
bool matchesDijkstraOnMap(const std::vector<std::string>& rows, bool diagonal) {
    int start = -1;
    int goal = -1;
    const Grid grid = parseGrid(rows, diagonal, start, goal);
    const Graph graph = buildModelGraph(grid, 1.0);
    JumpPointSearch jps(&graph, grid.width, grid.height);
    jps.enableDiagonalMovement(diagonal);
    const std::vector<int> route = jps.findPath(start, goal);
    Dijkstra dijkstra(&graph);
    const double best = dijkstra.getShortestDistance(start, goal);
    const std::string problem = routeProblem(grid, graph, route, start, goal, {});
    if (!problem.empty() || !nearlyEqual(routeCost(graph, route), best)) {
        std::cout << "(" << (problem.empty() ? "cost " + std::to_string(routeCost(graph, route)) + " vs " +
                                                   std::to_string(best)
                                             : problem)
                  << ") ";
        return false;
    }
    return true;
}

// True if findPath throws std::invalid_argument and supportsGraph reports a reason containing `expected`.
bool rejected(const Graph& graph, int width, int height, bool diagonal, int start, int goal, const std::string& expected) {
    JumpPointSearch jps(&graph, width, height);
    jps.enableDiagonalMovement(diagonal);
    std::string reason;
    if (jps.supportsGraph(&reason) || reason.find(expected) == std::string::npos) {
        std::cout << "(reason: '" << reason << "') ";
        return false;
    }
    try {
        jps.findPath(start, goal);
    } catch (const std::invalid_argument&) {
        return true;
    }
    return false;
}

// 3 x 1 strip of nodes with orthogonal edges, for the domain tests.
Graph strip(double weight) {
    CoutSilencer quiet;
    Graph graph;
    for (int x = 0; x < 3; ++x) graph.addNode(x, "n", x, 0);
    graph.addEdge(0, 1, weight);
    graph.addEdge(1, 2, weight);
    return graph;
}

}  // namespace

class JumpPointSearchTester {
private:
    int testsRun = 0;
    int testsPassed = 0;

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
        std::cout << "\n=== Jump Point Search Tests ===" << std::endl;

        runTest("No route when the graph has no edges", []() {
            Graph graph;
            graph.addNode(0, "Start", 0, 0);
            graph.addNode(1, "Goal", 2, 0);  // nothing connects the two nodes, and (1, 0) is empty

            JumpPointSearch jps(&graph, 3, 1);
            jps.enableDiagonalMovement(false);
            return jps.findPath(0, 1).empty();
        });

        runTest("Route around a wall is optimal and made of grid moves", []() {
            return matchesDijkstraOnMap({"........",  // the wall at x = 3 has its gap in the top row
                                         "...#....",
                                         "...#....",
                                         "...#....",
                                         "...#....",
                                         "S..#...G"},
                                        true);
        });

        runTest("Unknown start or goal gives an empty route", []() {
            int start = -1;
            int goal = -1;
            const Grid grid = parseGrid({"S..", "..G"}, true, start, goal);
            const Graph graph = buildModelGraph(grid, 1.0);
            JumpPointSearch jps(&graph, grid.width, grid.height);
            return jps.findPath(start, 999).empty() && jps.findPath(999, goal).empty();
        });

        // The three maps below are the smallest failures of the previous implementation in a seeded
        // probe of 3,568 queries: 1,319 false "no path" results on 8-connected grids, and 303 false
        // "no path" and 323 costlier routes on 4-connected grids. Its diagonal jumps cut corners, its
        // forced-neighbour rules mixed two JPS variants, and its straight jumps never turned on
        // 4-connected grids.
        runTest("Regression: 8-connected route that must not cut corners", []() {
            return matchesDijkstraOnMap({"#...#",
                                         "G....",
                                         ".##..",
                                         ".###S",
                                         "...#."},
                                        true);
        });

        runTest("Regression: 4-connected route to a goal off the start's row and column", []() {
            return matchesDijkstraOnMap({"...#.",
                                         "..#.G",
                                         ".....",
                                         ".#...",
                                         "..#..",
                                         "...##",
                                         "..#.#",
                                         "#S#.#"},
                                        false);
        });

        runTest("Regression: 4-connected route as cheap as Dijkstra's", []() {
            return matchesDijkstraOnMap({"S..#.",
                                         "..#..",
                                         ".G...",
                                         ".#...",
                                         "..#..",
                                         "...##",
                                         "..#.#",
                                         "#.#.#"},
                                        false);
        });

        runTest("Rejects a diagonal edge that cuts a corner", []() {
            Graph graph;
            {
                CoutSilencer quiet;
                graph.addNode(0, "a", 0, 0);
                graph.addNode(1, "b", 1, 0);
                graph.addNode(2, "c", 1, 1);  // (0, 1) is empty
                graph.addEdge(0, 1, 1.0);
                graph.addEdge(1, 2, 1.0);
                graph.addEdge(0, 2, std::sqrt(2.0));
            }
            return rejected(graph, 2, 2, true, 0, 2, "cuts the corner");
        });

        runTest("Rejects nodes off the integer grid, outside it, or sharing a cell", []() {
            Graph offGrid;
            Graph outside;
            Graph shared;
            {
                CoutSilencer quiet;
                offGrid.addNode(0, "a", 0, 0);
                offGrid.addNode(1, "b", 1.5, 0);
                outside.addNode(0, "a", 0, 0);
                outside.addNode(1, "b", 5, 0);
                shared.addNode(0, "a", 1, 0);
                shared.addNode(1, "b", 1.0000000000001, 0);
            }
            return rejected(offGrid, 3, 1, true, 0, 1, "not on an integer grid point") &&
                   rejected(outside, 3, 1, true, 0, 1, "outside the 3 x 1 grid") &&
                   rejected(shared, 3, 1, true, 0, 1, "share the grid cell");
        });

        runTest("Rejects unequal orthogonal weights and wrong diagonal weights", []() {
            Graph unequal;
            {
                CoutSilencer quiet;
                for (int x = 0; x < 3; ++x) unequal.addNode(x, "n", x, 0);
                unequal.addEdge(0, 1, 1.0);
                unequal.addEdge(1, 2, 2.0);
            }
            int start = -1;
            int goal = -1;
            const Grid square = parseGrid({".G", "S."}, true, start, goal);
            Graph badDiagonal = buildModelGraph(square, 1.0);
            {
                CoutSilencer quiet;
                badDiagonal.removeEdge(start, goal);
                badDiagonal.addEdge(start, goal, 1.5);
            }
            return rejected(unequal, 3, 1, true, 0, 2, "another orthogonal edge has weight") &&
                   rejected(badDiagonal, 2, 2, true, start, goal, "a diagonal must weigh");
        });

        runTest("Rejects one-way edges and missing grid moves", []() {
            Graph oneWay = strip(1.0);
            Graph missing;
            {
                CoutSilencer quiet;
                oneWay.addEdge(2, 0, 1.0, false);  // not neighbours either; checked first
                for (int x = 0; x < 3; ++x) missing.addNode(x, "n", x, 0);
                missing.addEdge(0, 1, 1.0);  // (1, 0) and (2, 0) both hold nodes, but no edge joins them
            }
            Graph directed;
            {
                CoutSilencer quiet;
                directed.addNode(0, "a", 0, 0);
                directed.addNode(1, "b", 1, 0);
                directed.addEdge(0, 1, 1.0, false);
            }
            return rejected(oneWay, 3, 1, true, 0, 2, "which are not neighbours") &&
                   rejected(missing, 3, 1, true, 0, 2, "but the graph has no edge 1 -> 2") &&
                   rejected(directed, 2, 1, true, 0, 1, "but the graph has no edge 1 -> 0");
        });

        runTest("Rejects diagonal edges when diagonal movement is disabled", []() {
            int start = -1;
            int goal = -1;
            const Grid square = parseGrid({".G", "S."}, true, start, goal);
            const Graph graph = buildModelGraph(square, 1.0);
            return rejected(graph, 2, 2, false, start, goal, "diagonal movement is disabled");
        });

        runTest("Accepts any uniform step weight and returns the least cost", []() {
            int start = -1;
            int goal = -1;
            Grid grid = parseGrid({"..#..G",
                                   "..#...",
                                   "......",
                                   "S.#..."},
                                  true, start, goal);
            const Graph graph = buildModelGraph(grid, 2.5);
            JumpPointSearch jps(&graph, grid.width, grid.height);
            const std::vector<int> route = jps.findPath(start, goal);
            Dijkstra dijkstra(&graph);
            return routeProblem(grid, graph, route, start, goal, {}).empty() &&
                   nearlyEqual(jps.getPathCost(route), dijkstra.getShortestDistance(start, goal));
        });

        runTest("Marked cells are closed, their corners are not cut, and unmarking reopens them", []() {
            int start = -1;
            int goal = -1;
            const Grid grid = parseGrid({".......",
                                         ".......",
                                         "S.....G",
                                         ".......",
                                         "......."},
                                        true, start, goal);
            const Graph graph = buildModelGraph(grid, 1.0);
            const std::set<std::pair<int, int>> marked{{3, 1}, {3, 2}, {3, 3}};
            JumpPointSearch jps(&graph, grid.width, grid.height);
            for (const auto& cell : marked) jps.addObstacle(cell.first, cell.second);
            const std::vector<int> around = jps.findPath(start, goal);
            const Graph reference = buildModelGraph(grid, 1.0, marked);
            Dijkstra dijkstra(&reference);
            const bool avoids = routeProblem(grid, reference, around, start, goal, marked).empty() &&
                                nearlyEqual(routeCost(graph, around), dijkstra.getShortestDistance(start, goal));
            for (const auto& cell : marked) jps.removeObstacle(cell.first, cell.second);
            const std::vector<int> straight = jps.findPath(start, goal);
            return avoids && straight.size() == 7 && nearlyEqual(routeCost(graph, straight), 6.0);
        });

        runTest("addObstacle rejects cells outside the grid", []() {
            const Graph graph = strip(1.0);
            JumpPointSearch jps(&graph, 3, 1);
            try {
                jps.addObstacle(3, 0);
            } catch (const std::invalid_argument&) {
                return true;
            }
            return false;
        });

        runTest("getJumpPoints reaches the goal along an open diagonal", []() {
            Grid grid;
            grid.width = 5;
            grid.height = 5;
            grid.free.assign(25, 1);
            const Graph graph = buildModelGraph(grid, 1.0);
            JumpPointSearch jps(&graph, 5, 5);
            const auto points = jps.getJumpPoints(0, 0, 4, 4);
            return std::find(points.begin(), points.end(), std::make_pair(4, 4)) != points.end();
        });

        runTest("Matches Dijkstra on a seeded corpus of 600 grids", []() {
            // Not a proof: a differential check on a reproducible sample. Grids are 5-40 by 5-30
            // cells with 0-40% blocked cells, two thirds 8-connected and one third 4-connected, step
            // weight 1 or 2.5, and every fourth grid with up to 12 cells marked through addObstacle.
            // Dijkstra searches the model graph of the open cells.
            SplitMix64 rng{20261109};
            int queries = 0;
            int reachable = 0;
            int mismatches = 0;
            for (int k = 0; k < 600; ++k) {
                Grid grid;
                grid.width = 5 + rng.below(36);
                grid.height = 5 + rng.below(26);
                grid.diagonal = k % 3 != 2;
                const double density = 0.4 * rng.unit();
                grid.free.resize(static_cast<std::size_t>(grid.width) * grid.height);
                for (auto& cell : grid.free) cell = rng.unit() < density ? 0 : 1;
                const double step = k % 2 == 0 ? 1.0 : 2.5;
                const Graph graph = buildModelGraph(grid, step);

                std::set<std::pair<int, int>> marked;
                if (k % 4 == 3) {
                    const int count = 1 + rng.below(12);
                    for (int m = 0; m < count; ++m) marked.insert({rng.below(grid.width), rng.below(grid.height)});
                }
                const Graph reference = marked.empty() ? Graph() : buildModelGraph(grid, step, marked);
                const Graph& searched = marked.empty() ? graph : reference;

                std::vector<int> open;
                for (int id = 0; id < grid.width * grid.height; ++id) {
                    if (grid.free[id] && marked.count({grid.x(id), grid.y(id)}) == 0) open.push_back(id);
                }
                if (open.size() < 2) continue;

                JumpPointSearch jps(&graph, grid.width, grid.height);
                jps.enableDiagonalMovement(grid.diagonal);
                for (const auto& cell : marked) jps.addObstacle(cell.first, cell.second);
                Dijkstra dijkstra(&searched);
                for (int s = 0; s < 3; ++s) {
                    const int start = open[rng.below(static_cast<int>(open.size()))];
                    const auto distances = dijkstra.findShortestDistances(start);
                    for (int t = 0; t < 4; ++t) {
                        const int goal = open[rng.below(static_cast<int>(open.size()))];
                        ++queries;
                        const std::vector<int> route = jps.findPath(start, goal);
                        const double best = distances.at(goal);
                        if (std::isinf(best)) {
                            mismatches += route.empty() ? 0 : 1;
                            continue;
                        }
                        ++reachable;
                        if (!routeProblem(grid, searched, route, start, goal, marked).empty() ||
                            !nearlyEqual(routeCost(graph, route), best)) {
                            ++mismatches;
                        }
                    }
                }
            }
            std::cout << "(" << queries << " queries, " << reachable << " reachable, " << mismatches
                      << " mismatches) ";
            return queries > 6000 && mismatches == 0;
        });

        printResults();
    }

    void printResults() {
        std::cout << "\n=== Jump Point Search Test Results ===" << std::endl;
        std::cout << "Tests passed: " << testsPassed << "/" << testsRun << std::endl;

        if (testsPassed == testsRun) {
            std::cout << "✓ All Jump Point Search tests passed!" << std::endl;
        } else {
            std::cout << "✗ Some Jump Point Search tests failed!" << std::endl;
        }
    }

    bool allPassed() const {
        return testsRun > 0 && testsPassed == testsRun;
    }
};

int main() {
    JumpPointSearchTester tester;
    tester.runAllTests();
    return tester.allPassed() ? 0 : 1;
}

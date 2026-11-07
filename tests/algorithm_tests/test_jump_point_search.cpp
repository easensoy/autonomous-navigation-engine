#include "pathfinding_algorithms/JumpPointSearch.hpp"
#include "core/Graph.hpp"
#include <cmath>
#include <exception>
#include <functional>
#include <iostream>
#include <string>
#include <vector>

// Jump Point Search runs on a grid laid over the graph: a cell is open when a node sits on it.
// Whatever it finds has to come back as a route the graph can actually drive: a sequence of nodes
// where each step is one of the graph's edges.
class JumpPointSearchTester {
private:
    int testsRun = 0;
    int testsPassed = 0;

    static constexpr int kWidth = 8;
    static constexpr int kHeight = 6;

    static bool isWall(int x, int y) { return x == 3 && y < kHeight - 1; }  // gap in the bottom row
    static int idAt(int x, int y) { return y * kWidth + x; }

    // 8-connected grid around a wall: straight steps cost 1 and diagonal steps sqrt(2).
    static Graph makeWalledGrid() {
        Graph graph;
        for (int y = 0; y < kHeight; ++y) {
            for (int x = 0; x < kWidth; ++x) {
                if (!isWall(x, y)) {
                    graph.addNode(idAt(x, y), "Cell" + std::to_string(x) + "_" + std::to_string(y), x, y);
                }
            }
        }
        const int steps[4][2] = {{1, 0}, {0, 1}, {1, 1}, {1, -1}};  // each neighbouring pair once
        for (int y = 0; y < kHeight; ++y) {
            for (int x = 0; x < kWidth; ++x) {
                if (isWall(x, y)) {
                    continue;
                }
                for (const auto& step : steps) {
                    const int nx = x + step[0];
                    const int ny = y + step[1];
                    if (nx < 0 || nx >= kWidth || ny < 0 || ny >= kHeight || isWall(nx, ny)) {
                        continue;
                    }
                    const bool diagonal = step[0] != 0 && step[1] != 0;
                    graph.addEdge(idAt(x, y), idAt(nx, ny), diagonal ? std::sqrt(2.0) : 1.0);
                }
            }
        }
        return graph;
    }

    static bool usesOnlyGraphEdges(const Graph& graph, const std::vector<int>& route) {
        for (size_t i = 1; i < route.size(); ++i) {
            if (!graph.hasEdge(route[i - 1], route[i])) {
                return false;
            }
        }
        return true;
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
        std::cout << "\n=== Jump Point Search Tests ===" << std::endl;

        runTest("No route when the graph has no edges", []() {
            Graph graph;
            graph.addNode(0, "Start", 0, 0);
            graph.addNode(1, "Goal", 2, 0);  // nothing connects the two nodes

            JumpPointSearch jps(&graph, 3, 1);
            jps.enableDiagonalMovement(false);
            return jps.findPath(0, 1).empty();
        });

        runTest("Route around a wall uses only graph edges", []() {
            Graph graph = makeWalledGrid();
            JumpPointSearch jps(&graph, kWidth, kHeight);
            const int start = idAt(0, 0);
            const int goal = idAt(kWidth - 1, 0);
            const std::vector<int> route = jps.findPath(start, goal);
            return !route.empty() && route.front() == start && route.back() == goal &&
                   usesOnlyGraphEdges(graph, route);
        });

        runTest("Unknown start or goal gives an empty route", []() {
            Graph graph = makeWalledGrid();
            JumpPointSearch jps(&graph, kWidth, kHeight);
            return jps.findPath(idAt(0, 0), 999).empty() && jps.findPath(999, idAt(0, 0)).empty();
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

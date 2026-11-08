#include "pathfinding_algorithms/AStar.hpp"
#include "pathfinding_algorithms/Dijkstra.hpp"
#include "core/Graph.hpp"
#include <iostream>
#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <exception>
#include <functional>
#include <random>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

class AStarTester {
private:
    Graph testGraph;
    int testsRun = 0;
    int testsPassed = 0;
    
    void createSimpleGraph() {
        // 3x3 grid
        for (int i = 0; i < 9; ++i) {
            testGraph.addNode(i, "Node" + std::to_string(i), i % 3 * 10.0, (i / 3) * 10.0);
        }
        
        // Add edges (4-connected)
        std::vector<std::pair<int, int>> edges = {
            {0, 1}, {1, 2}, {3, 4}, {4, 5}, {6, 7}, {7, 8},  // horizontal
            {0, 3}, {3, 6}, {1, 4}, {4, 7}, {2, 5}, {5, 8}   // vertical
        };
        
        for (auto& edge : edges) {
            testGraph.addEdge(edge.first, edge.second, 10.0);
        }
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
        std::cout << "\n=== A* Algorithm Tests ===" << std::endl;
        createSimpleGraph();
        
        runTest("Basic Path Finding", [this]() {
            AStar astar(&testGraph);
            astar.setHeuristicFunction([this](int nodeId, int goalId) {
                const Node& node = testGraph.getNode(nodeId);
                const Node& goal = testGraph.getNode(goalId);
                return node.euclideanDistance(goal);
            });
            
            std::vector<int> path = astar.findPath(0, 8);
            return !path.empty() && path.front() == 0 && path.back() == 8;
        });
        
        runTest("Same Start and Goal", [this]() {
            AStar astar(&testGraph);
            astar.setHeuristicFunction([this](int nodeId, int goalId) {
                const Node& node = testGraph.getNode(nodeId);
                const Node& goal = testGraph.getNode(goalId);
                return node.euclideanDistance(goal);
            });
            
            std::vector<int> path = astar.findPath(4, 4);
            return path.size() == 1 && path[0] == 4;
        });
        
        runTest("Invalid Start Node", [this]() {
            AStar astar(&testGraph);
            astar.setHeuristicFunction([](int, int) { return 0.0; });
            
            std::vector<int> path = astar.findPath(99, 8);
            return path.empty();
        });
        
        runTest("Invalid Goal Node", [this]() {
            AStar astar(&testGraph);
            astar.setHeuristicFunction([](int, int) { return 0.0; });
            
            std::vector<int> path = astar.findPath(0, 99);
            return path.empty();
        });
        
        runTest("Optimal Path Length", [this]() {
            AStar astar(&testGraph);
            astar.setHeuristicFunction([this](int nodeId, int goalId) {
                const Node& node = testGraph.getNode(nodeId);
                const Node& goal = testGraph.getNode(goalId);
                return node.euclideanDistance(goal);
            });
            
            std::vector<int> path = astar.findPath(0, 8);
            return path.size() == 5; // Optimal path: 0->1->2->5->8 or 0->3->6->7->8
        });
        
        runTest("Path Cost Calculation", [this]() {
            AStar astar(&testGraph);
            astar.setHeuristicFunction([this](int nodeId, int goalId) {
                const Node& node = testGraph.getNode(nodeId);
                const Node& goal = testGraph.getNode(goalId);
                return node.euclideanDistance(goal);
            });
            
            std::vector<int> path = astar.findPath(0, 2);
            double cost = astar.getPathCost(path);
            return cost == 20.0; // Two edges of weight 10.0 each
        });
        
        runTest("Manhattan Heuristic", [this]() {
            AStar astar(&testGraph);
            
            std::vector<int> path = astar.findPathWithHeuristic(0, 8, 
                [&astar](int nodeId, int goalId) {
                    return astar.manhattanHeuristic(nodeId, goalId);
                });
            
            return !path.empty() && path.front() == 0 && path.back() == 8;
        });
        
        runTest("Euclidean Heuristic", [this]() {
            AStar astar(&testGraph);
            
            std::vector<int> path = astar.findPathWithHeuristic(0, 8,
                [&astar](int nodeId, int goalId) {
                    return astar.euclideanHeuristic(nodeId, goalId);
                });
            
            return !path.empty() && path.front() == 0 && path.back() == 8;
        });
        
        runTest("Search Statistics", [this]() {
            AStar astar(&testGraph);
            astar.enableSearchStatistics(true);
            astar.setHeuristicFunction([this](int nodeId, int goalId) {
                const Node& node = testGraph.getNode(nodeId);
                const Node& goal = testGraph.getNode(goalId);
                return node.euclideanDistance(goal);
            });
            
            std::vector<int> path = astar.findPath(0, 8);
            size_t nodesExplored = astar.getLastSearchNodesExplored();
            double executionTime = astar.getLastSearchExecutionTime();
            
            return !path.empty() && nodesExplored > 0 && executionTime >= 0;
        });
        
        runTest("Performance Benchmark", [this]() {
            AStar astar(&testGraph);
            astar.setHeuristicFunction([this](int nodeId, int goalId) {
                const Node& node = testGraph.getNode(nodeId);
                const Node& goal = testGraph.getNode(goalId);
                return node.euclideanDistance(goal);
            });
            
            auto start = std::chrono::high_resolution_clock::now();
            
            // Run multiple searches
            for (int i = 0; i < 100; ++i) {
                std::vector<int> path = astar.findPath(0, 8);
                if (path.empty()) return false;
            }
            
            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            
            std::cout << " (100 searches in " << duration.count() << "μs)";
            return duration.count() < 1000000;  // 1 s: guards against a pathological slowdown, not a speed target
        });
        
        runTest("Disconnected Graph", [this]() {
            Graph disconnectedGraph;
            disconnectedGraph.addNode(0, "Node0", 0, 0);
            disconnectedGraph.addNode(1, "Node1", 10, 0);
            // No edges - nodes are disconnected
            
            AStar astar(&disconnectedGraph);
            astar.setHeuristicFunction([&disconnectedGraph](int nodeId, int goalId) {
                const Node& node = disconnectedGraph.getNode(nodeId);
                const Node& goal = disconnectedGraph.getNode(goalId);
                return node.euclideanDistance(goal);
            });
            
            std::vector<int> path = astar.findPath(0, 1);
            return path.empty();
        });

        runTest("Default heuristic stays optimal when edges cost less than their length", []() {
            // The cheap route detours far away in space, so plain straight-line distance
            // overestimates its remaining cost unless the heuristic is scaled to the edge costs.
            Graph graph;
            graph.addNode(0, "Start", 0, 0);
            graph.addNode(1, "FarDetour", 1000, 0);
            graph.addNode(2, "NearStep", 1, 0);
            graph.addNode(3, "Goal", 2, 0);
            graph.addEdge(0, 1, 1.0, false);
            graph.addEdge(1, 3, 1.0, false);
            graph.addEdge(0, 2, 1.0, false);
            graph.addEdge(2, 3, 10.0, false);

            AStar astar(&graph);
            std::vector<int> path = astar.findPath(0, 3);
            return !path.empty() && astar.getPathCost(path) == 2.0;
        });

        runTest("Default heuristic follows edge changes made between searches", []() {
            // The first search caches the heuristic scale for this graph. Adding a cheap detour far
            // away in space lowers that scale; reusing the old value would overestimate the detour.
            Graph graph;
            graph.addNode(0, "Start", 0, 0);
            graph.addNode(1, "FarDetour", 1000, 0);
            graph.addNode(2, "NearStep", 1, 0);
            graph.addNode(3, "Goal", 2, 0);
            graph.addEdge(0, 2, 1.0, false);
            graph.addEdge(2, 3, 10.0, false);

            AStar astar(&graph);
            const std::vector<int> before = astar.findPath(0, 3);
            graph.addEdge(0, 1, 1.0, false);
            graph.addEdge(1, 3, 1.0, false);
            const std::vector<int> after = astar.findPath(0, 3);
            return astar.getPathCost(before) == 11.0 && !after.empty() && astar.getPathCost(after) == 2.0;
        });

        runTest("Admissible but inconsistent heuristic still finds the optimal path", []() {
            // h(2) = 3 never overestimates (the cheapest 2 -> 3 route costs 3), but it drops by 3
            // across the edge 2 -> 1, which costs 1. Node 1 has to be reopened to find the cost-4 route.
            Graph graph;
            for (int i = 0; i < 4; ++i) {
                graph.addNode(i, "Node" + std::to_string(i), 0, 0);
            }
            graph.addEdge(0, 1, 3.0, false);
            graph.addEdge(0, 2, 1.0, false);
            graph.addEdge(2, 1, 1.0, false);
            graph.addEdge(1, 3, 2.0, false);
            graph.addEdge(2, 3, 100.0, false);

            AStar astar(&graph);
            astar.setHeuristicFunction([](int nodeId, int) { return nodeId == 2 ? 3.0 : 0.0; });
            std::vector<int> path = astar.findPath(0, 3);
            return !path.empty() && astar.getPathCost(path) == 4.0;
        });

        runTest("Admissibility check works with the default heuristic", []() {
            Graph graph;
            graph.addNode(0, "Start", 0, 0);
            graph.addNode(1, "Goal", 10, 0);
            graph.addEdge(0, 1, 10.0);

            AStar astar(&graph);
            return astar.isHeuristicAdmissible(0, 1);
        });

        runTest("Admissibility check catches an overestimate away from the start", []() {
            Graph graph;
            for (int i = 0; i < 3; ++i) {
                graph.addNode(i, "Node" + std::to_string(i), i * 10.0, 0);
            }
            graph.addEdge(0, 1, 10.0);
            graph.addEdge(1, 2, 10.0);

            AStar astar(&graph);
            // Fine at the start (0 <= 20), but node 1 claims 50 when the goal is 10 away.
            astar.setHeuristicFunction([](int nodeId, int) { return nodeId == 1 ? 50.0 : 0.0; });
            return !astar.isHeuristicAdmissible(0, 2);
        });

        runTest("Rejects negative edge weights", []() {
            Graph graph;
            for (int i = 0; i < 3; ++i) {
                graph.addNode(i, "Node" + std::to_string(i), i * 10.0, 0);
            }
            graph.addEdge(0, 1, 10.0, false);
            graph.addEdge(1, 2, -5.0, false);

            AStar astar(&graph);
            try {
                astar.findPath(0, 2);
            } catch (const std::invalid_argument&) {
                return true;
            }
            return false;
        });

        runTest("Matches Dijkstra's cost on 100 random graphs", []() {
            // Edge costs range from 5% to 300% of the straight-line length, so many edges cost
            // less than their length: the case where an unscaled distance heuristic overestimates.
            std::mt19937 rng(20261107);
            std::uniform_real_distribution<double> coordinate(0.0, 100.0);
            std::uniform_real_distribution<double> costFactor(0.05, 3.0);
            std::uniform_int_distribution<int> nodeCount(5, 25);
            for (int trial = 0; trial < 100; ++trial) {
                Graph graph;
                const int n = nodeCount(rng);
                for (int i = 0; i < n; ++i) {
                    const double x = coordinate(rng);
                    const double y = coordinate(rng);
                    graph.addNode(i, "N" + std::to_string(i), x, y);
                }
                std::uniform_int_distribution<int> pickNode(0, n - 1);
                for (int e = 0; e < n * 3; ++e) {
                    const int a = pickNode(rng);
                    const int b = pickNode(rng);
                    if (a == b || graph.hasEdge(a, b) || graph.hasEdge(b, a)) {
                        continue;
                    }
                    const double length = graph.getNode(a).euclideanDistance(graph.getNode(b));
                    const double cost = std::max(0.001, length * costFactor(rng));
                    const bool twoWay = rng() % 2 == 0;
                    graph.addEdge(a, b, cost, twoWay);
                }

                AStar astar(&graph);
                Dijkstra dijkstra(&graph);
                for (int query = 0; query < 3; ++query) {
                    const int start = pickNode(rng);
                    const int goal = pickNode(rng);
                    const std::vector<int> found = astar.findPath(start, goal);
                    const std::vector<int> shortest = dijkstra.findShortestPath(start, goal);
                    if (found.empty() != shortest.empty()) {
                        return false;
                    }
                    if (!found.empty()) {
                        const double expected = astar.getPathCost(shortest);
                        if (std::abs(astar.getPathCost(found) - expected) > 1e-9 * std::max(1.0, expected)) {
                            std::cout << "(trial " << trial << ": A* cost " << astar.getPathCost(found)
                                      << ", Dijkstra cost " << expected << ") ";
                            return false;
                        }
                    }
                }
            }
            return true;
        });

        runTest("Equal-cost routes whose sums round apart do not hide the cheaper entry", []() {
            // X is reached first at g = 0.1 + 0.2 = 0.30000000000000004, then at 0.15 + 0.15 = 0.3.
            // Adding h(X), about 99.9, rounds both open-list entries to the same f, and on equal f the
            // stale entry (larger g) comes out first. Skipping it must leave the cheaper entry live:
            // X is the only way to G.
            Graph graph;
            graph.addNode(0, "S", 0, 0);
            graph.addNode(1, "P", 1, 0);
            graph.addNode(2, "Q", 0, 1);
            graph.addNode(3, "X", 1, 1);
            graph.addNode(4, "G", 1000, 1);
            graph.addEdge(0, 1, 0.1, false);
            graph.addEdge(0, 2, 0.15, false);
            graph.addEdge(1, 3, 0.2, false);
            graph.addEdge(2, 3, 0.15, false);
            graph.addEdge(3, 4, 1000.0, false);

            AStar astar(&graph);
            return astar.findPath(0, 4) == std::vector<int>{0, 2, 3, 4};
        });

        runTest("Matches Dijkstra's cost on a racked grid full of equal-cost routes", []() {
            // A 22 x 17 lattice, 8-connected, with six 7 x 2 racks removed; each edge costs its
            // length (1 or sqrt 2). Routes of equal cost add the same steps in different orders, so
            // their sums can differ in the last bit, and the racks leave few alternatives when a
            // node is lost. No randomness, so every compiler builds the same graph.
            constexpr int width = 22;
            constexpr int height = 17;
            auto isRack = [&](int x, int y) {
                return (y % 5 == 2 || y % 5 == 3) && y < height - 2 && x % 10 >= 2 && x % 10 <= 8 && x < width - 2;
            };
            Graph graph;
            for (int y = 0; y < height; ++y) {
                for (int x = 0; x < width; ++x) {
                    if (!isRack(x, y)) {
                        graph.addNode(y * width + x, "N" + std::to_string(y * width + x), x, y);
                    }
                }
            }
            const int offsets[4][2] = {{1, 0}, {0, 1}, {1, 1}, {-1, 1}};
            for (int y = 0; y < height; ++y) {
                for (int x = 0; x < width; ++x) {
                    for (const auto& offset : offsets) {
                        const int nx = x + offset[0];
                        const int ny = y + offset[1];
                        const int from = y * width + x;
                        const int to = ny * width + nx;
                        if (nx >= 0 && nx < width && ny < height && graph.hasNode(from) && graph.hasNode(to)) {
                            graph.addEdge(from, to, std::hypot(double(offset[0]), double(offset[1])));
                        }
                    }
                }
            }

            AStar astar(&graph);
            Dijkstra dijkstra(&graph);
            int queries = 0;
            int costlier = 0;
            for (int start = 0; start < width * height; start += 7) {
                if (!graph.hasNode(start)) {
                    continue;
                }
                const std::unordered_map<int, double> best = dijkstra.findShortestDistances(start);
                for (int goal = 1; goal < width * height; goal += 9) {
                    if (goal == start || !graph.hasNode(goal)) {
                        continue;
                    }
                    ++queries;
                    const std::vector<int> found = astar.findPath(start, goal);
                    const double expected = best.at(goal);
                    if (found.empty() || astar.getPathCost(found) > expected + 1e-9 * std::max(1.0, expected)) {
                        ++costlier;
                    }
                }
            }
            std::cout << "(" << costlier << " of " << queries << " queries costlier than Dijkstra) ";
            return queries > 1000 && costlier == 0;
        });

        printResults();
    }
    
    void printResults() {
        std::cout << "\n=== A* Test Results ===" << std::endl;
        std::cout << "Tests passed: " << testsPassed << "/" << testsRun << std::endl;
        std::cout << "Success rate: " << (testsRun > 0 ? (testsPassed * 100.0 / testsRun) : 0) << "%" << std::endl;
        
        if (testsPassed == testsRun) {
            std::cout << "✓ All A* tests passed!" << std::endl;
        } else {
            std::cout << "✗ Some A* tests failed!" << std::endl;
        }
    }

    bool allPassed() const {
        return testsRun > 0 && testsPassed == testsRun;
    }
};

int main() {
    AStarTester tester;
    tester.runAllTests();
    return tester.allPassed() ? 0 : 1;
}
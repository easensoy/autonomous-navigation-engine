#include "pathfinding_algorithms/AStar.hpp"
#include "core/Graph.hpp"
#include <iostream>
#include <cassert>
#include <chrono>

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
                [this](int nodeId, int goalId) {
                    return astar.manhattanHeuristic(nodeId, goalId);
                });
            
            return !path.empty() && path.front() == 0 && path.back() == 8;
        });
        
        runTest("Euclidean Heuristic", [this]() {
            AStar astar(&testGraph);
            
            std::vector<int> path = astar.findPathWithHeuristic(0, 8,
                [this](int nodeId, int goalId) {
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
            return duration.count() < 10000; // Should complete in less than 10ms
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
};

int main() {
    AStarTester tester;
    tester.runAllTests();
    return 0;
}
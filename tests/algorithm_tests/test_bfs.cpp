#include "pathfinding_algorithms/BFS.hpp"
#include "core/Graph.hpp"
#include <iostream>
#include <cassert>
#include <chrono>

class BFSTester {
private:
    Graph testGraph;
    int testsRun = 0;
    int testsPassed = 0;
    
    void createSimpleGraph() {
        // 3x3 grid for unweighted testing
        for (int i = 0; i < 9; ++i) {
            testGraph.addNode(i, "Node" + std::to_string(i), i % 3 * 10.0, (i / 3) * 10.0);
        }
        
        // Add edges (4-connected grid)
        std::vector<std::pair<int, int>> edges = {
            {0, 1}, {1, 2}, {3, 4}, {4, 5}, {6, 7}, {7, 8},  // horizontal
            {0, 3}, {3, 6}, {1, 4}, {4, 7}, {2, 5}, {5, 8}   // vertical
        };
        
        for (auto& edge : edges) {
            testGraph.addEdge(edge.first, edge.second, 1.0); // Unit weights for BFS
        }
    }
    
    void createLinearGraph() {
        testGraph.clear();
        for (int i = 0; i < 5; ++i) {
            testGraph.addNode(i, "Node" + std::to_string(i), i * 10.0, 0.0);
        }
        
        for (int i = 0; i < 4; ++i) {
            testGraph.addEdge(i, i + 1, 1.0);
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
        std::cout << "\n=== BFS Algorithm Tests ===" << std::endl;
        createSimpleGraph();
        
        runTest("Basic Path Finding", [this]() {
            BFS bfs(&testGraph);
            
            std::vector<int> path = bfs.findPath(0, 8);
            return !path.empty() && path.front() == 0 && path.back() == 8;
        });
        
        runTest("Shortest Unweighted Path", [this]() {
            BFS bfs(&testGraph);
            
            std::vector<int> path = bfs.findShortestUnweightedPath(0, 8);
            return path.size() == 5; // Optimal unweighted path length
        });
        
        runTest("Same Start and Goal", [this]() {
            BFS bfs(&testGraph);
            
            std::vector<int> path = bfs.findPath(4, 4);
            return path.size() == 1 && path[0] == 4;
        });
        
        runTest("Invalid Start Node", [this]() {
            BFS bfs(&testGraph);
            
            std::vector<int> path = bfs.findPath(99, 8);
            return path.empty();
        });
        
        runTest("Invalid Goal Node", [this]() {
            BFS bfs(&testGraph);
            
            std::vector<int> path = bfs.findPath(0, 99);
            return path.empty();
        });
        
        runTest("Reachability Check", [this]() {
            BFS bfs(&testGraph);
            
            bool reachable = bfs.isReachable(0, 8);
            bool unreachable = bfs.isReachable(0, 99);
            
            return reachable && !unreachable;
        });
        
        runTest("Shortest Path Length", [this]() {
            BFS bfs(&testGraph);
            
            int length = bfs.getShortestPathLength(0, 8);
            return length == 4; // 4 edges in shortest path
        });
        
        runTest("Nodes at Distance", [this]() {
            BFS bfs(&testGraph);
            
            std::vector<int> nodesAt2 = bfs.getNodesAtDistance(4, 2); // Center node
            return nodesAt2.size() == 4; // Should be 4 nodes at distance 2 from center
        });
        
        runTest("Find All Paths", [this]() {
            createLinearGraph();
            BFS bfs(&testGraph);
            
            std::vector<std::vector<int>> allPaths = bfs.findAllPaths(0, 4, 5);
            return !allPaths.empty();
        });
        
        runTest("Disconnected Graph", [this]() {
            Graph disconnectedGraph;
            disconnectedGraph.addNode(0, "Node0", 0, 0);
            disconnectedGraph.addNode(1, "Node1", 10, 0);
            // No edges - nodes are disconnected
            
            BFS bfs(&disconnectedGraph);
            std::vector<int> path = bfs.findPath(0, 1);
            bool reachable = bfs.isReachable(0, 1);
            
            return path.empty() && !reachable;
        });
        
        runTest("Traversal Order Logging", [this]() {
            createSimpleGraph();
            BFS bfs(&testGraph);
            bfs.printTraversalOrder(true);
            
            std::vector<int> path = bfs.findPath(0, 8);
            
            return !path.empty();
        });
        
        runTest("Performance Benchmark", [this]() {
            createSimpleGraph();
            BFS bfs(&testGraph);
            
            auto start = std::chrono::high_resolution_clock::now();
            
            // Run multiple searches
            for (int i = 0; i < 100; ++i) {
                std::vector<int> path = bfs.findPath(0, 8);
                if (path.empty()) return false;
            }
            
            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            
            std::cout << " (100 searches in " << duration.count() << "μs)";
            return duration.count() < 10000; // Should complete in less than 10ms
        });
        
        runTest("Large Graph Performance", [this]() {
            Graph largeGraph;
            
            // Create a 10x10 grid
            for (int i = 0; i < 100; ++i) {
                largeGraph.addNode(i, "Node" + std::to_string(i), 
                                  (i % 10) * 10.0, (i / 10) * 10.0);
            }
            
            // Add grid connections
            for (int i = 0; i < 100; ++i) {
                int row = i / 10;
                int col = i % 10;
                
                if (col < 9) largeGraph.addEdge(i, i + 1, 1.0);     // Right
                if (row < 9) largeGraph.addEdge(i, i + 10, 1.0);    // Down
            }
            
            BFS bfs(&largeGraph);
            
            auto start = std::chrono::high_resolution_clock::now();
            std::vector<int> path = bfs.findPath(0, 99);
            auto end = std::chrono::high_resolution_clock::now();
            
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            std::cout << " (" << duration.count() << "μs)";
            
            return !path.empty() && duration.count() < 5000; // Should complete in <5ms
        });
        
        runTest("Multiple Queries", [this]() {
            createSimpleGraph();
            BFS bfs(&testGraph);
            
            bool allSuccessful = true;
            for (int start = 0; start < 5; ++start) {
                for (int goal = start + 1; goal < 9; ++goal) {
                    if (bfs.isReachable(start, goal)) {
                        std::vector<int> path = bfs.findPath(start, goal);
                        if (path.empty() || path.front() != start || path.back() != goal) {
                            allSuccessful = false;
                            break;
                        }
                    }
                }
                if (!allSuccessful) break;
            }
            
            return allSuccessful;
        });
        
        runTest("Tree Traversal", [this]() {
            Graph treeGraph;
            // Create simple tree: 0->1,2  1->3,4  2->5,6
            for (int i = 0; i < 7; ++i) {
                treeGraph.addNode(i, "Node" + std::to_string(i), i * 5.0, (i / 2) * 10.0);
            }
            
            treeGraph.addEdge(0, 1, 1.0);
            treeGraph.addEdge(0, 2, 1.0);
            treeGraph.addEdge(1, 3, 1.0);
            treeGraph.addEdge(1, 4, 1.0);
            treeGraph.addEdge(2, 5, 1.0);
            treeGraph.addEdge(2, 6, 1.0);
            
            BFS bfs(&treeGraph);
            
            std::vector<int> path = bfs.findPath(0, 6);
            return path.size() == 3 && path[0] == 0 && path[2] == 6; // 0->2->6
        });
        
        printResults();
    }
    
    void printResults() {
        std::cout << "\n=== BFS Test Results ===" << std::endl;
        std::cout << "Tests passed: " << testsPassed << "/" << testsRun << std::endl;
        std::cout << "Success rate: " << (testsRun > 0 ? (testsPassed * 100.0 / testsRun) : 0) << "%" << std::endl;
        
        if (testsPassed == testsRun) {
            std::cout << "✓ All BFS tests passed!" << std::endl;
        } else {
            std::cout << "✗ Some BFS tests failed!" << std::endl;
        }
    }
};

int main() {
    BFSTester tester;
    tester.runAllTests();
    return 0;
}
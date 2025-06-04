#include "pathfinding_algorithms/BellmanFord.hpp"
#include "core/Graph.hpp"
#include <iostream>
#include <cassert>
#include <chrono>

class BellmanFordTester {
private:
    Graph testGraph;
    int testsRun = 0;
    int testsPassed = 0;
    
    void createSimpleGraph() {
        for (int i = 0; i < 5; ++i) {
            testGraph.addNode(i, "Node" + std::to_string(i), i * 10.0, 0.0);
        }
        
        testGraph.addEdge(0, 1, 10.0);
        testGraph.addEdge(1, 2, 15.0);
        testGraph.addEdge(2, 3, 20.0);
        testGraph.addEdge(3, 4, 25.0);
        testGraph.addEdge(0, 2, 30.0); // Alternative path
        testGraph.addEdge(1, 3, 40.0); // Alternative path
    }
    
    void createNegativeWeightGraph() {
        testGraph.clear();
        for (int i = 0; i < 4; ++i) {
            testGraph.addNode(i, "Node" + std::to_string(i), i * 10.0, 0.0);
        }
        
        testGraph.addEdge(0, 1, 5.0);
        testGraph.addEdge(1, 2, -3.0);  // Negative weight
        testGraph.addEdge(2, 3, 2.0);
        testGraph.addEdge(0, 2, 8.0);   // Alternative path
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
        std::cout << "\n=== Bellman-Ford Algorithm Tests ===" << std::endl;
        
        runTest("Basic Path Finding", [this]() {
            createSimpleGraph();
            BellmanFord bf(&testGraph);
            
            std::vector<int> path = bf.findShortestPath(0, 4);
            return !path.empty() && path.front() == 0 && path.back() == 4;
        });
        
        runTest("Shortest Distance Calculation", [this]() {
            createSimpleGraph();
            BellmanFord bf(&testGraph);
            
            double distance = bf.getShortestDistance(0, 4);
            return distance == 70.0; // 0->1->2->3->4: 10+15+20+25=70
        });
        
        runTest("Same Start and Goal", [this]() {
            createSimpleGraph();
            BellmanFord bf(&testGraph);
            
            std::vector<int> path = bf.findShortestPath(2, 2);
            return path.size() == 1 && path[0] == 2;
        });
        
        runTest("Negative Weight Handling", [this]() {
            createNegativeWeightGraph();
            BellmanFord bf(&testGraph);
            
            std::vector<int> path = bf.findShortestPath(0, 3);
            double distance = bf.getShortestDistance(0, 3);
            
            // Path 0->1->2->3 = 5+(-3)+2 = 4, vs 0->2->3 = 8+2 = 10
            return !path.empty() && distance == 4.0;
        });
        
        runTest("Compute All Shortest Paths", [this]() {
            createSimpleGraph();
            BellmanFord bf(&testGraph);
            
            auto result = bf.computeShortestPaths(0);
            
            return result.distances.size() == 5 && 
                   result.distances.count(0) > 0 && 
                   result.distances.at(0) == 0.0;
        });
        
        runTest("Invalid Start Node", [this]() {
            createSimpleGraph();
            BellmanFord bf(&testGraph);
            
            std::vector<int> path = bf.findShortestPath(99, 4);
            return path.empty();
        });
        
        runTest("Invalid Goal Node", [this]() {
            createSimpleGraph();
            BellmanFord bf(&testGraph);
            
            std::vector<int> path = bf.findShortestPath(0, 99);
            return path.empty();
        });
        
        runTest("Disconnected Graph", [this]() {
            Graph disconnectedGraph;
            disconnectedGraph.addNode(0, "Node0", 0, 0);
            disconnectedGraph.addNode(1, "Node1", 10, 0);
            // No edges
            
            BellmanFord bf(&disconnectedGraph);
            std::vector<int> path = bf.findShortestPath(0, 1);
            return path.empty();
        });
        
        runTest("Negative Cycle Detection", [this]() {
            Graph cycleGraph;
            for (int i = 0; i < 3; ++i) {
                cycleGraph.addNode(i, "Node" + std::to_string(i), i * 10.0, 0.0);
            }
            
            cycleGraph.addEdge(0, 1, 1.0);
            cycleGraph.addEdge(1, 2, -3.0);
            cycleGraph.addEdge(2, 0, 1.0); // Creates negative cycle: 1-3+1 = -1
            
            BellmanFord bf(&cycleGraph);
            bool hasNegativeCycle = bf.detectNegativeCycle();
            
            return hasNegativeCycle;
        });
        
        runTest("Can Handle Negative Weights", [this]() {
            createSimpleGraph();
            BellmanFord bf(&testGraph);
            
            return bf.canHandleNegativeWeights();
        });
        
        runTest("Iteration Logging", [this]() {
            createSimpleGraph();
            BellmanFord bf(&testGraph);
            bf.printIterationDetails(true);
            
            std::vector<int> path = bf.findShortestPath(0, 4);
            
            return !path.empty();
        });
        
        runTest("Performance with Large Graph", [this]() {
            Graph largeGraph;
            
            // Create a larger graph for performance testing
            for (int i = 0; i < 20; ++i) {
                largeGraph.addNode(i, "Node" + std::to_string(i), i * 5.0, 0.0);
                if (i > 0) {
                    largeGraph.addEdge(i-1, i, static_cast<double>(i));
                }
            }
            
            BellmanFord bf(&largeGraph);
            
            auto start = std::chrono::high_resolution_clock::now();
            std::vector<int> path = bf.findShortestPath(0, 19);
            auto end = std::chrono::high_resolution_clock::now();
            
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            std::cout << " (" << duration.count() << "μs)";
            
            return !path.empty() && duration.count() < 5000; // Should complete in <5ms
        });
        
        runTest("Multiple Path Queries", [this]() {
            createSimpleGraph();
            BellmanFord bf(&testGraph);
            
            bool allSuccessful = true;
            for (int start = 0; start < 3; ++start) {
                for (int goal = start + 1; goal < 5; ++goal) {
                    std::vector<int> path = bf.findShortestPath(start, goal);
                    if (path.empty() || path.front() != start || path.back() != goal) {
                        allSuccessful = false;
                        break;
                    }
                }
                if (!allSuccessful) break;
            }
            
            return allSuccessful;
        });
        
        printResults();
    }
    
    void printResults() {
        std::cout << "\n=== Bellman-Ford Test Results ===" << std::endl;
        std::cout << "Tests passed: " << testsPassed << "/" << testsRun << std::endl;
        std::cout << "Success rate: " << (testsRun > 0 ? (testsPassed * 100.0 / testsRun) : 0) << "%" << std::endl;
        
        if (testsPassed == testsRun) {
            std::cout << "✓ All Bellman-Ford tests passed!" << std::endl;
        } else {
            std::cout << "✗ Some Bellman-Ford tests failed!" << std::endl;
        }
    }
};

int main() {
    BellmanFordTester tester;
    tester.runAllTests();
    return 0;
}
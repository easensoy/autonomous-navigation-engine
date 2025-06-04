#include "pathfinding_algorithms/Dijkstra.hpp"
#include "core/Graph.hpp"
#include <iostream>
#include <cassert>
#include <chrono>
#include <cmath>

class DijkstraTester {
private:
    Graph testGraph;
    int testsRun = 0;
    int testsPassed = 0;
    
    void createWeightedGraph() {
        // Create a weighted graph for testing
        for (int i = 0; i < 6; ++i) {
            testGraph.addNode(i, "Node" + std::to_string(i), i * 10.0, 0.0);
        }
        
        // Add weighted edges
        testGraph.addEdge(0, 1, 4.0);
        testGraph.addEdge(0, 2, 2.0);
        testGraph.addEdge(1, 2, 1.0);
        testGraph.addEdge(1, 3, 5.0);
        testGraph.addEdge(2, 3, 8.0);
        testGraph.addEdge(2, 4, 10.0);
        testGraph.addEdge(3, 4, 2.0);
        testGraph.addEdge(3, 5, 6.0);
        testGraph.addEdge(4, 5, 3.0);
    }
    
    void createSimpleLinearGraph() {
        testGraph.clear();
        for (int i = 0; i < 5; ++i) {
            testGraph.addNode(i, "Node" + std::to_string(i), i * 10.0, 0.0);
        }
        
        testGraph.addEdge(0, 1, 10.0);
        testGraph.addEdge(1, 2, 15.0);
        testGraph.addEdge(2, 3, 20.0);
        testGraph.addEdge(3, 4, 25.0);
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
        std::cout << "\n=== Dijkstra Algorithm Tests ===" << std::endl;
        createWeightedGraph();
        
        runTest("Basic Path Finding", [this]() {
            Dijkstra dijkstra(&testGraph);
            
            std::vector<int> path = dijkstra.findShortestPath(0, 5);
            return !path.empty() && path.front() == 0 && path.back() == 5;
        });
        
        runTest("Shortest Distance Calculation", [this]() {
            Dijkstra dijkstra(&testGraph);
            
            double distance = dijkstra.getShortestDistance(0, 5);
            // Optimal path: 0->2->1->3->4->5 = 2+1+5+2+3 = 13
            // or 0->1->3->4->5 = 4+5+2+3 = 14
            // or 0->2->4->5 = 2+10+3 = 15
            // Actually: 0->2->1->3->5 = 2+1+5+6 = 14
            // Best: 0->1->3->4->5 = 4+5+2+3 = 14 or 0->2->1->3->4->5 = 2+1+5+2+3 = 13
            return distance == 13.0;
        });
        
        runTest("Same Start and Goal", [this]() {
            Dijkstra dijkstra(&testGraph);
            
            std::vector<int> path = dijkstra.findShortestPath(2, 2);
            double distance = dijkstra.getShortestDistance(2, 2);
            
            return path.size() == 1 && path[0] == 2 && distance == 0.0;
        });
        
        runTest("Invalid Start Node", [this]() {
            Dijkstra dijkstra(&testGraph);
            
            std::vector<int> path = dijkstra.findShortestPath(99, 5);
            return path.empty();
        });
        
        runTest("Invalid Goal Node", [this]() {
            Dijkstra dijkstra(&testGraph);
            
            std::vector<int> path = dijkstra.findShortestPath(0, 99);
            return path.empty();
        });
        
        runTest("Path Existence Check", [this]() {
            Dijkstra dijkstra(&testGraph);
            
            bool exists = dijkstra.hasPath(0, 5);
            bool notExists = dijkstra.hasPath(0, 99);
            
            return exists && !notExists;
        });
        
        runTest("Find All Shortest Distances", [this]() {
            Dijkstra dijkstra(&testGraph);
            
            std::unordered_map<int, double> distances = dijkstra.findShortestDistances(0);
            
            return distances.size() == 6 && distances[0] == 0.0 && distances[2] == 2.0;
        });
        
        runTest("Find All Shortest Paths", [this]() {
            Dijkstra dijkstra(&testGraph);
            
            std::vector<std::vector<int>> allPaths = dijkstra.findAllShortestPaths(0);
            
            return allPaths.size() == 6; // One path to each node from start
        });
        
        runTest("Disconnected Graph", [this]() {
            Graph disconnectedGraph;
            disconnectedGraph.addNode(0, "Node0", 0, 0);
            disconnectedGraph.addNode(1, "Node1", 10, 0);
            // No edges
            
            Dijkstra dijkstra(&disconnectedGraph);
            std::vector<int> path = dijkstra.findShortestPath(0, 1);
            bool hasPath = dijkstra.hasPath(0, 1);
            
            return path.empty() && !hasPath;
        });
        
        runTest("Algorithm Steps Logging", [this]() {
            createWeightedGraph();
            Dijkstra dijkstra(&testGraph);
            dijkstra.printAlgorithmSteps(true);
            
            std::vector<int> path = dijkstra.findShortestPath(0, 5);
            
            return !path.empty();
        });
        
        runTest("Linear Graph Optimality", [this]() {
            createSimpleLinearGraph();
            Dijkstra dijkstra(&testGraph);
            
            double distance = dijkstra.getShortestDistance(0, 4);
            return distance == 70.0; // 10+15+20+25
        });
        
        runTest("Alternative Path Selection", [this]() {
            Graph altGraph;
            for (int i = 0; i < 4; ++i) {
                altGraph.addNode(i, "Node" + std::to_string(i), i * 10.0, 0.0);
            }
            
            altGraph.addEdge(0, 1, 1.0);
            altGraph.addEdge(1, 3, 3.0);  // Path 1: 0->1->3 = 4
            altGraph.addEdge(0, 2, 2.0);
            altGraph.addEdge(2, 3, 1.0);  // Path 2: 0->2->3 = 3 (better)
            
            Dijkstra dijkstra(&altGraph);
            double distance = dijkstra.getShortestDistance(0, 3);
            
            return distance == 3.0;
        });
        
        runTest("Performance Benchmark", [this]() {
            createWeightedGraph();
            Dijkstra dijkstra(&testGraph);
            
            auto start = std::chrono::high_resolution_clock::now();
            
            // Run multiple searches
            for (int i = 0; i < 100; ++i) {
                std::vector<int> path = dijkstra.findShortestPath(0, 5);
                if (path.empty()) return false;
            }
            
            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            
            std::cout << " (100 searches in " << duration.count() << "μs)";
            return duration.count() < 15000; // Should complete in less than 15ms
        });
        
        runTest("Large Graph Performance", [this]() {
            Graph largeGraph;
            
            // Create a larger weighted graph
            for (int i = 0; i < 50; ++i) {
                largeGraph.addNode(i, "Node" + std::to_string(i), i * 5.0, 0.0);
                if (i > 0) {
                    largeGraph.addEdge(i-1, i, static_cast<double>(i % 10 + 1));
                }
                if (i > 5) {
                    largeGraph.addEdge(i-5, i, static_cast<double>((i * 3) % 10 + 1));
                }
            }
            
            Dijkstra dijkstra(&largeGraph);
            
            auto start = std::chrono::high_resolution_clock::now();
            std::vector<int> path = dijkstra.findShortestPath(0, 49);
            auto end = std::chrono::high_resolution_clock::now();
            
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            std::cout << " (" << duration.count() << "μs)";
            
            return !path.empty() && duration.count() < 10000; // Should complete in <10ms
        });
        
        runTest("Multiple Queries Efficiency", [this]() {
            createWeightedGraph();
            Dijkstra dijkstra(&testGraph);
            
            bool allSuccessful = true;
            std::unordered_map<int, double> allDistances = dijkstra.findShortestDistances(0);
            
            // Verify individual queries match bulk computation
            for (int goal = 1; goal < 6; ++goal) {
                double individualDistance = dijkstra.getShortestDistance(0, goal);
                if (std::abs(individualDistance - allDistances[goal]) > 0.001) {
                    allSuccessful = false;
                    break;
                }
            }
            
            return allSuccessful;
        });
        
        runTest("Zero Weight Edges", [this]() {
            Graph zeroGraph;
            for (int i = 0; i < 3; ++i) {
                zeroGraph.addNode(i, "Node" + std::to_string(i), i * 10.0, 0.0);
            }
            
            zeroGraph.addEdge(0, 1, 0.0);  // Zero weight
            zeroGraph.addEdge(1, 2, 5.0);
            zeroGraph.addEdge(0, 2, 10.0); // Alternative longer path
            
            Dijkstra dijkstra(&zeroGraph);
            double distance = dijkstra.getShortestDistance(0, 2);
            
            return distance == 5.0; // Should take 0->1->2 path
        });
        
        runTest("Single Node Graph", [this]() {
            Graph singleGraph;
            singleGraph.addNode(0, "OnlyNode", 0, 0);
            
            Dijkstra dijkstra(&singleGraph);
            std::vector<int> path = dijkstra.findShortestPath(0, 0);
            double distance = dijkstra.getShortestDistance(0, 0);
            
            return path.size() == 1 && path[0] == 0 && distance == 0.0;
        });
        
        printResults();
    }
    
    void printResults() {
        std::cout << "\n=== Dijkstra Test Results ===" << std::endl;
        std::cout << "Tests passed: " << testsPassed << "/" << testsRun << std::endl;
        std::cout << "Success rate: " << (testsRun > 0 ? (testsPassed * 100.0 / testsRun) : 0) << "%" << std::endl;
        
        if (testsPassed == testsRun) {
            std::cout << "✓ All Dijkstra tests passed!" << std::endl;
        } else {
            std::cout << "✗ Some Dijkstra tests failed!" << std::endl;
        }
    }
};

int main() {
    DijkstraTester tester;
    tester.runAllTests();
    return 0;
}
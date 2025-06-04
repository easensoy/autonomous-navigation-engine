#include "reporting/PerformanceProfiler.hpp"
#include "pathfinding_algorithms/AStar.hpp"
#include "pathfinding_algorithms/Dijkstra.hpp"
#include "pathfinding_algorithms/BFS.hpp"
#include "pathfinding_algorithms/BellmanFord.hpp"
#include <iostream>
#include <chrono>
#include <vector>
#include <algorithm>
#include <fstream>
#include <random>

class AlgorithmBenchmarker {
private:
    const Graph* graph;
    std::unique_ptr<PerformanceProfiler> profiler;
    
    struct BenchmarkResult {
        std::string algorithm;
        double avgExecutionTime;
        double minExecutionTime;
        double maxExecutionTime;
        double avgPathLength;
        double successRate;
        size_t avgNodesExplored;
        double avgMemoryUsage;
        
        BenchmarkResult() : avgExecutionTime(0), minExecutionTime(0), maxExecutionTime(0),
                           avgPathLength(0), successRate(0), avgNodesExplored(0), avgMemoryUsage(0) {}
    };
    
    struct TestCase {
        int startNode;
        int goalNode;
        double expectedDistance;
        std::string description;
    };
    
    std::vector<TestCase> testCases;
    int numIterations;
    
public:
    AlgorithmBenchmarker(const Graph* environment) 
        : graph(environment), profiler(std::make_unique<PerformanceProfiler>()), numIterations(10) {
        
        if (!graph) {
            throw std::invalid_argument("Graph pointer cannot be null");
        }
        
        generateTestCases();
    }
    
    std::vector<BenchmarkResult> benchmarkAllAlgorithms() {
        std::vector<BenchmarkResult> results;
        
        std::cout << "[BENCHMARK] Starting comprehensive algorithm benchmark" << std::endl;
        
        results.push_back(benchmarkAStar());
        results.push_back(benchmarkDijkstra());
        results.push_back(benchmarkBFS());
        results.push_back(benchmarkBellmanFord());
        
        generateComparisonReport(results);
        return results;
    }
    
    BenchmarkResult benchmarkAStar() {
        std::cout << "[BENCHMARK] Benchmarking A* algorithm" << std::endl;
        
        AStar astar(graph);
        astar.setHeuristicFunction([this](int nodeId, int goalId) {
            const Node& node = graph->getNode(nodeId);
            const Node& goal = graph->getNode(goalId);
            return node.euclideanDistance(goal);
        });
        
        return runBenchmark("AStar", [&](int start, int goal) {
            return astar.findPath(start, goal);
        });
    }
    
    BenchmarkResult benchmarkDijkstra() {
        std::cout << "[BENCHMARK] Benchmarking Dijkstra algorithm" << std::endl;
        
        Dijkstra dijkstra(graph);
        
        return runBenchmark("Dijkstra", [&](int start, int goal) {
            return dijkstra.findShortestPath(start, goal);
        });
    }
    
    BenchmarkResult benchmarkBFS() {
        std::cout << "[BENCHMARK] Benchmarking BFS algorithm" << std::endl;
        
        BFS bfs(graph);
        
        return runBenchmark("BFS", [&](int start, int goal) {
            return bfs.findPath(start, goal);
        });
    }
    
    BenchmarkResult benchmarkBellmanFord() {
        std::cout << "[BENCHMARK] Benchmarking Bellman-Ford algorithm" << std::endl;
        
        BellmanFord bellmanFord(graph);
        
        return runBenchmark("BellmanFord", [&](int start, int goal) {
            return bellmanFord.findShortestPath(start, goal);
        });
    }
    
    void setIterations(int iterations) {
        numIterations = iterations;
    }
    
    void addCustomTestCase(int start, int goal, const std::string& description) {
        TestCase testCase;
        testCase.startNode = start;
        testCase.goalNode = goal;
        testCase.description = description;
        testCase.expectedDistance = calculateDirectDistance(start, goal);
        testCases.push_back(testCase);
    }
    
private:
    template<typename AlgorithmFunc>
    BenchmarkResult runBenchmark(const std::string& algorithmName, AlgorithmFunc algorithm) {
        BenchmarkResult result;
        result.algorithm = algorithmName;
        
        std::vector<double> executionTimes;
        std::vector<double> pathLengths;
        int successCount = 0;
        
        for (int iter = 0; iter < numIterations; ++iter) {
            for (const auto& testCase : testCases) {
                profiler->startProfiling(algorithmName + "_test");
                
                auto start = std::chrono::high_resolution_clock::now();
                std::vector<int> path = algorithm(testCase.startNode, testCase.goalNode);
                auto end = std::chrono::high_resolution_clock::now();
                
                profiler->stopProfiling();
                
                double executionTime = std::chrono::duration<double>(end - start).count();
                executionTimes.push_back(executionTime);
                
                if (!path.empty()) {
                    successCount++;
                    double pathLength = calculatePathLength(path);
                    pathLengths.push_back(pathLength);
                }
            }
        }
        
        // Calculate statistics
        if (!executionTimes.empty()) {
            std::sort(executionTimes.begin(), executionTimes.end());
            result.avgExecutionTime = std::accumulate(executionTimes.begin(), executionTimes.end(), 0.0) / executionTimes.size();
            result.minExecutionTime = executionTimes.front();
            result.maxExecutionTime = executionTimes.back();
        }
        
        if (!pathLengths.empty()) {
            result.avgPathLength = std::accumulate(pathLengths.begin(), pathLengths.end(), 0.0) / pathLengths.size();
        }
        
        result.successRate = static_cast<double>(successCount) / (numIterations * testCases.size()) * 100.0;
        
        PerformanceMetrics metrics = profiler->getMetrics(algorithmName + "_test");
        result.avgMemoryUsage = metrics.memoryUsed;
        result.avgNodesExplored = metrics.nodesExplored;
        
        return result;
    }
    
    void generateTestCases() {
        std::vector<int> nodeIds = graph->getAllNodeIds();
        if (nodeIds.size() < 10) {
            // Small graph - use all combinations
            for (size_t i = 0; i < nodeIds.size(); ++i) {
                for (size_t j = i + 1; j < nodeIds.size(); ++j) {
                    addCustomTestCase(nodeIds[i], nodeIds[j], "Small graph test");
                }
            }
        } else {
            // Large graph - generate representative test cases
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<> dis(0, nodeIds.size() - 1);
            
            // Short distance tests
            for (int i = 0; i < 5; ++i) {
                int start = nodeIds[dis(gen)];
                auto neighbors = graph->getNeighbors(start);
                if (!neighbors.empty()) {
                    int goal = neighbors[dis(gen) % neighbors.size()];
                    addCustomTestCase(start, goal, "Short distance");
                }
            }
            
            // Medium distance tests
            for (int i = 0; i < 5; ++i) {
                int start = nodeIds[dis(gen)];
                int goal = nodeIds[dis(gen)];
                addCustomTestCase(start, goal, "Medium distance");
            }
            
            // Long distance tests (corner to corner)
            const Node* minNode = nullptr, *maxNode = nullptr;
            double minCoord = std::numeric_limits<double>::max();
            double maxCoord = std::numeric_limits<double>::lowest();
            
            for (int nodeId : nodeIds) {
                const Node& node = graph->getNode(nodeId);
                double coord = node.getX() + node.getY();
                if (coord < minCoord) {
                    minCoord = coord;
                    minNode = &node;
                }
                if (coord > maxCoord) {
                    maxCoord = coord;
                    maxNode = &node;
                }
            }
            
            if (minNode && maxNode) {
                addCustomTestCase(minNode->getId(), maxNode->getId(), "Long distance");
            }
        }
    }
    
    double calculatePathLength(const std::vector<int>& path) const {
        if (path.size() < 2) return 0.0;
        
        double length = 0.0;
        for (size_t i = 1; i < path.size(); ++i) {
            const Node& from = graph->getNode(path[i-1]);
            const Node& to = graph->getNode(path[i]);
            length += from.euclideanDistance(to);
        }
        return length;
    }
    
    double calculateDirectDistance(int start, int goal) const {
        const Node& startNode = graph->getNode(start);
        const Node& goalNode = graph->getNode(goal);
        return startNode.euclideanDistance(goalNode);
    }
    
    void generateComparisonReport(const std::vector<BenchmarkResult>& results) const {
        std::cout << "\n=== Algorithm Benchmark Results ===" << std::endl;
        std::cout << std::setw(15) << "Algorithm" 
                  << std::setw(12) << "Avg Time(s)"
                  << std::setw(12) << "Success(%)"
                  << std::setw(15) << "Avg Path Len"
                  << std::setw(12) << "Memory(KB)" << std::endl;
        std::cout << std::string(70, '-') << std::endl;
        
        for (const auto& result : results) {
            std::cout << std::setw(15) << result.algorithm
                      << std::setw(12) << std::fixed << std::setprecision(6) << result.avgExecutionTime
                      << std::setw(12) << std::fixed << std::setprecision(1) << result.successRate
                      << std::setw(15) << std::fixed << std::setprecision(2) << result.avgPathLength
                      << std::setw(12) << std::fixed << std::setprecision(1) << result.avgMemoryUsage / 1024.0 << std::endl;
        }
        
        exportBenchmarkResults(results);
    }
    
    void exportBenchmarkResults(const std::vector<BenchmarkResult>& results) const {
        std::string filename = "benchmark_results_" + 
                              std::to_string(std::chrono::system_clock::now().time_since_epoch().count()) + 
                              ".csv";
        
        std::ofstream file(filename);
        if (!file.is_open()) {
            std::cout << "[BENCHMARK] Failed to create results file" << std::endl;
            return;
        }
        
        file << "Algorithm,AvgTime,MinTime,MaxTime,SuccessRate,AvgPathLength,AvgNodesExplored,AvgMemory\n";
        for (const auto& result : results) {
            file << result.algorithm << ","
                 << result.avgExecutionTime << ","
                 << result.minExecutionTime << ","
                 << result.maxExecutionTime << ","
                 << result.successRate << ","
                 << result.avgPathLength << ","
                 << result.avgNodesExplored << ","
                 << result.avgMemoryUsage << "\n";
        }
        
        file.close();
        std::cout << "[BENCHMARK] Results exported to: " << filename << std::endl;
    }
};

// Global benchmarker
static std::unique_ptr<AlgorithmBenchmarker> g_benchmarker;

void initializeBenchmarker(const Graph* graph) {
    g_benchmarker = std::make_unique<AlgorithmBenchmarker>(graph);
}

std::vector<AlgorithmBenchmarker::BenchmarkResult> runComprehensiveBenchmark() {
    if (g_benchmarker) {
        return g_benchmarker->benchmarkAllAlgorithms();
    }
    return {};
}

void setBenchmarkIterations(int iterations) {
    if (g_benchmarker) {
        g_benchmarker->setIterations(iterations);
    }
}
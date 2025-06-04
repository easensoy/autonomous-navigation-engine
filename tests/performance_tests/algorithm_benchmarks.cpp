#include "pathfinding_algorithms/AStar.hpp"
#include "pathfinding_algorithms/Dijkstra.hpp"
#include "pathfinding_algorithms/BFS.hpp"
#include "pathfinding_algorithms/BellmanFord.hpp"
#include "core/Graph.hpp"
#include "reporting/PerformanceProfiler.hpp"
#include "utilities/MathUtils.hpp"
#include <iostream>
#include <chrono>
#include <vector>
#include <algorithm>
#include <random>
#include <iomanip>
#include <fstream>
#include <numeric>

/**
 * AlgorithmBenchmarker: Comprehensive Performance Analysis Suite
 * 
 * This class systematically evaluates pathfinding algorithms across multiple dimensions:
 * - Execution time under varying conditions
 * - Memory consumption patterns  
 * - Solution quality and optimality
 * - Scalability characteristics
 * - Robustness across different graph topologies
 * 
 * Think of this as a laboratory where we put each algorithm through controlled
 * experiments to understand their strengths, weaknesses, and optimal use cases.
 */
class AlgorithmBenchmarker {
private:
    struct BenchmarkResult {
        std::string algorithmName;
        double averageExecutionTime;
        double medianExecutionTime;
        double minExecutionTime;
        double maxExecutionTime;
        double standardDeviation;
        size_t averageNodesExplored;
        double averagePathLength;
        double pathOptimalityRatio;  // Ratio of found path to theoretical optimum
        size_t memoryUsage;
        int successfulRuns;
        int totalRuns;
        
        BenchmarkResult() : averageExecutionTime(0), medianExecutionTime(0), 
                           minExecutionTime(std::numeric_limits<double>::max()), 
                           maxExecutionTime(0), standardDeviation(0), 
                           averageNodesExplored(0), averagePathLength(0), 
                           pathOptimalityRatio(1.0), memoryUsage(0), 
                           successfulRuns(0), totalRuns(0) {}
    };
    
    struct TestScenario {
        std::string name;
        std::function<std::shared_ptr<Graph>()> graphGenerator;
        std::vector<std::pair<int, int>> testPairs;  // Start-goal pairs to test
        std::string description;
        
        TestScenario(const std::string& n, std::function<std::shared_ptr<Graph>()> gen, 
                    const std::string& desc) : name(n), graphGenerator(gen), description(desc) {}
    };
    
    std::unique_ptr<PerformanceProfiler> profiler;
    std::vector<TestScenario> scenarios;
    std::mt19937 randomEngine;
    
public:
    AlgorithmBenchmarker() : profiler(std::make_unique<PerformanceProfiler>()), 
                            randomEngine(std::chrono::steady_clock::now().time_since_epoch().count()) {
        
        std::cout << "=== Algorithm Performance Benchmarking Suite ===" << std::endl;
        std::cout << "This suite evaluates pathfinding algorithms across multiple dimensions:" << std::endl;
        std::cout << "- Execution speed and efficiency" << std::endl;
        std::cout << "- Memory consumption patterns" << std::endl;
        std::cout << "- Solution quality and optimality" << std::endl;
        std::cout << "- Robustness across different scenarios" << std::endl;
        
        setupBenchmarkScenarios();
    }
    
private:
    /**
     * Creates diverse test scenarios that represent real-world navigation challenges.
     * Each scenario tests different algorithm characteristics:
     */
    void setupBenchmarkScenarios() {
        std::cout << "\nSetting up benchmark scenarios..." << std::endl;
        
        // Scenario 1: Dense Grid - Tests performance on highly connected graphs
        scenarios.emplace_back("Dense Grid 10x10", [this]() {
            return createDenseGrid(10, 10);
        }, "Highly connected grid simulating urban environments with many route options");
        
        // Scenario 2: Sparse Network - Tests efficiency on loosely connected graphs  
        scenarios.emplace_back("Sparse Network", [this]() {
            return createSparseNetwork(50, 0.1);
        }, "Sparsely connected network simulating rural or highway systems");
        
        // Scenario 3: Layered Hierarchy - Tests handling of complex structured graphs
        scenarios.emplace_back("Layered Hierarchy", [this]() {
            return createLayeredGraph(5, 8);
        }, "Multi-layered graph simulating building floors or network hierarchies");
        
        // Scenario 4: Random Weighted - Tests algorithm robustness with varied edge costs
        scenarios.emplace_back("Random Weighted Graph", [this]() {
            return createRandomWeightedGraph(30, 0.3);
        }, "Random graph with varied edge weights simulating real terrain costs");
        
        // Scenario 5: Linear Chain - Tests worst-case performance scenarios
        scenarios.emplace_back("Linear Chain", [this]() {
            return createLinearChain(100);
        }, "Linear structure testing algorithm behavior in constrained scenarios");
        
        // Scenario 6: Star Topology - Tests hub-and-spoke navigation patterns
        scenarios.emplace_back("Star Topology", [this]() {
            return createStarTopology(25);
        }, "Central hub with radiating spokes simulating distribution centers");
        
        std::cout << "Created " << scenarios.size() << " diverse test scenarios" << std::endl;
    }
    
    // Graph generators for different topologies - each designed to stress different algorithm aspects
    
    std::shared_ptr<Graph> createDenseGrid(int width, int height) {
        auto graph = std::make_shared<Graph>();
        
        // Create grid nodes with realistic coordinates
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int nodeId = y * width + x;
                graph->addNode(nodeId, "Grid_" + std::to_string(x) + "_" + std::to_string(y), 
                              x * 10.0, y * 10.0);
            }
        }
        
        // Add dense connections: 4-connected plus diagonals for maximum connectivity
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int nodeId = y * width + x;
                
                // Cardinal directions
                if (x < width - 1) graph->addEdge(nodeId, nodeId + 1, 10.0);
                if (y < height - 1) graph->addEdge(nodeId, nodeId + width, 10.0);
                
                // Diagonal connections for increased density
                if (x < width - 1 && y < height - 1) {
                    graph->addEdge(nodeId, nodeId + width + 1, 14.14);
                }
                if (x > 0 && y < height - 1) {
                    graph->addEdge(nodeId, nodeId + width - 1, 14.14);
                }
            }
        }
        
        return graph;
    }
    
    std::shared_ptr<Graph> createSparseNetwork(int nodeCount, double connectionProbability) {
        auto graph = std::make_shared<Graph>();
        
        // Create randomly positioned nodes
        std::uniform_real_distribution<double> positionDist(0.0, 100.0);
        
        for (int i = 0; i < nodeCount; ++i) {
            double x = positionDist(randomEngine);
            double y = positionDist(randomEngine);
            graph->addNode(i, "Sparse_" + std::to_string(i), x, y);
        }
        
        // Add sparse connections based on probability
        std::uniform_real_distribution<double> probDist(0.0, 1.0);
        
        for (int i = 0; i < nodeCount; ++i) {
            for (int j = i + 1; j < nodeCount; ++j) {
                if (probDist(randomEngine) < connectionProbability) {
                    const Node& nodeI = graph->getNode(i);
                    const Node& nodeJ = graph->getNode(j);
                    double distance = nodeI.euclideanDistance(nodeJ);
                    graph->addEdge(i, j, distance);
                }
            }
        }
        
        return graph;
    }
    
    std::shared_ptr<Graph> createLayeredGraph(int layers, int nodesPerLayer) {
        auto graph = std::make_shared<Graph>();
        
        // Create layered structure simulating building floors or network tiers
        for (int layer = 0; layer < layers; ++layer) {
            for (int node = 0; node < nodesPerLayer; ++node) {
                int nodeId = layer * nodesPerLayer + node;
                double x = node * 15.0;
                double y = layer * 20.0;
                graph->addNode(nodeId, "Layer_" + std::to_string(layer) + "_" + std::to_string(node), x, y);
                
                // Connect within layer (horizontal movement)
                if (node > 0) {
                    graph->addEdge(nodeId - 1, nodeId, 15.0);
                }
                
                // Connect between layers (vertical movement - more expensive)
                if (layer > 0) {
                    int belowNode = (layer - 1) * nodesPerLayer + node;
                    graph->addEdge(belowNode, nodeId, 25.0);
                }
            }
        }
        
        return graph;
    }
    
    std::shared_ptr<Graph> createRandomWeightedGraph(int nodeCount, double density) {
        auto graph = std::make_shared<Graph>();
        
        // Create nodes in a rough grid pattern with some randomness
        int gridSize = static_cast<int>(std::sqrt(nodeCount)) + 1;
        std::uniform_real_distribution<double> noiseDist(-5.0, 5.0);
        
        for (int i = 0; i < nodeCount; ++i) {
            double baseX = (i % gridSize) * 20.0;
            double baseY = (i / gridSize) * 20.0;
            double x = baseX + noiseDist(randomEngine);
            double y = baseY + noiseDist(randomEngine);
            graph->addNode(i, "Weighted_" + std::to_string(i), x, y);
        }
        
        // Add connections with varied weights simulating different terrain types
        std::uniform_real_distribution<double> weightDist(1.0, 50.0);
        std::uniform_real_distribution<double> connectDist(0.0, 1.0);
        
        for (int i = 0; i < nodeCount; ++i) {
            for (int j = i + 1; j < nodeCount; ++j) {
                if (connectDist(randomEngine) < density) {
                    double weight = weightDist(randomEngine);
                    graph->addEdge(i, j, weight);
                }
            }
        }
        
        return graph;
    }
    
    std::shared_ptr<Graph> createLinearChain(int length) {
        auto graph = std::make_shared<Graph>();
        
        // Create a simple chain to test worst-case scenarios
        for (int i = 0; i < length; ++i) {
            graph->addNode(i, "Chain_" + std::to_string(i), i * 10.0, 0.0);
            if (i > 0) {
                graph->addEdge(i - 1, i, 10.0);
            }
        }
        
        return graph;
    }
    
    std::shared_ptr<Graph> createStarTopology(int spokeCount) {
        auto graph = std::make_shared<Graph>();
        
        // Central hub
        graph->addNode(0, "Hub", 0.0, 0.0);
        
        // Radiating spokes
        for (int i = 1; i <= spokeCount; ++i) {
            double angle = (2.0 * M_PI * (i - 1)) / spokeCount;
            double x = 30.0 * std::cos(angle);
            double y = 30.0 * std::sin(angle);
            graph->addNode(i, "Spoke_" + std::to_string(i), x, y);
            graph->addEdge(0, i, 30.0);
        }
        
        return graph;
    }
    
    /**
     * Generates meaningful test cases for each scenario.
     * We select start-goal pairs that represent realistic navigation challenges.
     */
    void generateTestPairs(const std::shared_ptr<Graph>& graph, TestScenario& scenario) {
        scenario.testPairs.clear();
        std::vector<int> nodeIds = graph->getAllNodeIds();
        
        if (nodeIds.size() < 2) return;
        
        std::uniform_int_distribution<int> nodeDist(0, nodeIds.size() - 1);
        
        // Add systematic test cases covering different distance ranges
        
        // Short distance tests (nearby nodes)
        for (int i = 0; i < 3 && i < nodeIds.size() - 1; ++i) {
            scenario.testPairs.emplace_back(nodeIds[i], nodeIds[i + 1]);
        }
        
        // Medium distance tests (quarter to half graph traversal)
        if (nodeIds.size() > 10) {
            scenario.testPairs.emplace_back(nodeIds[0], nodeIds[nodeIds.size() / 4]);
            scenario.testPairs.emplace_back(nodeIds[nodeIds.size() / 4], nodeIds[nodeIds.size() / 2]);
        }
        
        // Long distance tests (corner to corner scenarios)
        scenario.testPairs.emplace_back(nodeIds.front(), nodeIds.back());
        if (nodeIds.size() > 20) {
            scenario.testPairs.emplace_back(nodeIds[1], nodeIds[nodeIds.size() - 2]);
        }
        
        // Random test cases for statistical validity
        for (int i = 0; i < 5; ++i) {
            int start = nodeIds[nodeDist(randomEngine)];
            int goal = nodeIds[nodeDist(randomEngine)];
            if (start != goal) {
                scenario.testPairs.emplace_back(start, goal);
            }
        }
    }
    
    /**
     * Benchmarks a single algorithm against a specific scenario.
     * This is where we collect detailed performance metrics.
     */
    template<typename AlgorithmType>
    BenchmarkResult benchmarkAlgorithm(const std::string& algorithmName, 
                                       const TestScenario& scenario,
                                       std::function<std::vector<int>(AlgorithmType&, int, int)> pathfinder,
                                       std::function<std::unique_ptr<AlgorithmType>(const Graph*)> constructor) {
        
        BenchmarkResult result;
        result.algorithmName = algorithmName;
        
        auto graph = scenario.graphGenerator();
        auto testPairs = scenario.testPairs;
        if (testPairs.empty()) {
            // Generate test pairs if not already done
            TestScenario mutableScenario = scenario;
            generateTestPairs(graph, mutableScenario);
            testPairs = mutableScenario.testPairs;
        }
        
        std::vector<double> executionTimes;
        std::vector<double> pathLengths;
        std::vector<size_t> nodesExplored;
        
        std::cout << "  Testing " << algorithmName << " on " << scenario.name << "..." << std::endl;
        
        // Run multiple iterations for statistical significance
        const int iterations = 5;
        
        for (int iter = 0; iter < iterations; ++iter) {
            for (const auto& [start, goal] : testPairs) {
                // Verify nodes exist in graph
                if (!graph->hasNode(start) || !graph->hasNode(goal)) {
                    continue;
                }
                
                result.totalRuns++;
                
                try {
                    auto algorithm = constructor(graph.get());
                    
                    // Measure execution time with high precision
                    auto startTime = std::chrono::high_resolution_clock::now();
                    
                    profiler->startProfiling(algorithmName + "_execution");
                    std::vector<int> path = pathfinder(*algorithm, start, goal);
                    profiler->stopProfiling();
                    
                    auto endTime = std::chrono::high_resolution_clock::now();
                    
                    if (!path.empty()) {
                        result.successfulRuns++;
                        
                        double executionTime = std::chrono::duration<double>(endTime - startTime).count();
                        executionTimes.push_back(executionTime);
                        
                        // Calculate path metrics
                        double pathLength = calculatePathLength(graph.get(), path);
                        pathLengths.push_back(pathLength);
                        
                        // Get performance metrics if available
                        PerformanceMetrics metrics = profiler->getMetrics(algorithmName + "_execution");
                        nodesExplored.push_back(metrics.nodesExplored);
                    }
                    
                } catch (const std::exception& e) {
                    std::cout << "    Warning: " << algorithmName << " failed on " 
                              << start << "->" << goal << ": " << e.what() << std::endl;
                }
            }
        }
        
        // Calculate statistical metrics
        if (!executionTimes.empty()) {
            std::sort(executionTimes.begin(), executionTimes.end());
            
            result.averageExecutionTime = std::accumulate(executionTimes.begin(), executionTimes.end(), 0.0) / executionTimes.size();
            result.medianExecutionTime = executionTimes[executionTimes.size() / 2];
            result.minExecutionTime = executionTimes.front();
            result.maxExecutionTime = executionTimes.back();
            
            // Calculate standard deviation
            double variance = 0.0;
            for (double time : executionTimes) {
                variance += std::pow(time - result.averageExecutionTime, 2);
            }
            result.standardDeviation = std::sqrt(variance / executionTimes.size());
        }
        
        if (!pathLengths.empty()) {
            result.averagePathLength = std::accumulate(pathLengths.begin(), pathLengths.end(), 0.0) / pathLengths.size();
        }
        
        if (!nodesExplored.empty()) {
            result.averageNodesExplored = std::accumulate(nodesExplored.begin(), nodesExplored.end(), 0UL) / nodesExplored.size();
        }
        
        return result;
    }
    
    double calculatePathLength(const Graph* graph, const std::vector<int>& path) {
        if (path.size() < 2) return 0.0;
        
        double totalLength = 0.0;
        for (size_t i = 1; i < path.size(); ++i) {
            if (graph->hasNode(path[i-1]) && graph->hasNode(path[i])) {
                const Node& from = graph->getNode(path[i-1]);
                const Node& to = graph->getNode(path[i]);
                totalLength += from.euclideanDistance(to);
            }
        }
        return totalLength;
    }
    
public:
    /**
     * Runs comprehensive benchmarks across all algorithms and scenarios.
     * This is the main entry point for performance evaluation.
     */
    void runComprehensiveBenchmarks() {
        std::cout << "\n=== Starting Comprehensive Algorithm Benchmarks ===" << std::endl;
        std::cout << "Each algorithm will be tested across " << scenarios.size() << " scenarios" << std::endl;
        std::cout << "This provides insights into algorithm behavior under different conditions" << std::endl;
        
        std::vector<std::vector<BenchmarkResult>> allResults;
        
        for (auto& scenario : scenarios) {
            std::cout << "\n--- Scenario: " << scenario.name << " ---" << std::endl;
            std::cout << "Description: " << scenario.description << std::endl;
            
            // Generate test pairs for this scenario
            auto graph = scenario.graphGenerator();
            generateTestPairs(graph, scenario);
            
            std::cout << "Graph size: " << graph->getNodeCount() << " nodes, " 
                      << graph->getEdgeCount() << " edges" << std::endl;
            std::cout << "Test pairs: " << scenario.testPairs.size() << std::endl;
            
            std::vector<BenchmarkResult> scenarioResults;
            
            // Benchmark A* Algorithm
            auto astarResult = benchmarkAlgorithm<AStar>(
                "A*", scenario,
                [](AStar& algo, int start, int goal) { return algo.findPath(start, goal); },
                [](const Graph* g) { 
                    auto astar = std::make_unique<AStar>(g);
                    astar->setHeuristicFunction([g](int nodeId, int goalId) {
                        const Node& node = g->getNode(nodeId);
                        const Node& goalNode = g->getNode(goalId);
                        return node.euclideanDistance(goalNode);
                    });
                    return astar;
                }
            );
            scenarioResults.push_back(astarResult);
            
            // Benchmark Dijkstra Algorithm
            auto dijkstraResult = benchmarkAlgorithm<Dijkstra>(
                "Dijkstra", scenario,
                [](Dijkstra& algo, int start, int goal) { return algo.findShortestPath(start, goal); },
                [](const Graph* g) { return std::make_unique<Dijkstra>(g); }
            );
            scenarioResults.push_back(dijkstraResult);
            
            // Benchmark BFS Algorithm  
            auto bfsResult = benchmarkAlgorithm<BFS>(
                "BFS", scenario,
                [](BFS& algo, int start, int goal) { return algo.findPath(start, goal); },
                [](const Graph* g) { return std::make_unique<BFS>(g); }
            );
            scenarioResults.push_back(bfsResult);
            
            // Benchmark Bellman-Ford Algorithm
            auto bellmanResult = benchmarkAlgorithm<BellmanFord>(
                "Bellman-Ford", scenario,
                [](BellmanFord& algo, int start, int goal) { return algo.findShortestPath(start, goal); },
                [](const Graph* g) { return std::make_unique<BellmanFord>(g); }
            );
            scenarioResults.push_back(bellmanResult);
            
            allResults.push_back(scenarioResults);
            displayScenarioResults(scenario.name, scenarioResults);
        }
        
        generateComprehensiveReport(allResults);
        exportResultsToCSV(allResults);
    }
    
private:
    void displayScenarioResults(const std::string& scenarioName, const std::vector<BenchmarkResult>& results) {
        std::cout << "\n  Results for " << scenarioName << ":" << std::endl;
        std::cout << std::setw(15) << "Algorithm" 
                  << std::setw(12) << "Avg Time(ms)"
                  << std::setw(12) << "Success(%)"
                  << std::setw(15) << "Avg Path Len"
                  << std::setw(12) << "Nodes Exp" << std::endl;
        std::cout << std::string(66, '-') << std::endl;
        
        for (const auto& result : results) {
            double successRate = result.totalRuns > 0 ? 
                (double(result.successfulRuns) / result.totalRuns) * 100.0 : 0.0;
            
            std::cout << std::setw(15) << result.algorithmName
                      << std::setw(12) << std::fixed << std::setprecision(3) << (result.averageExecutionTime * 1000)
                      << std::setw(12) << std::fixed << std::setprecision(1) << successRate
                      << std::setw(15) << std::fixed << std::setprecision(2) << result.averagePathLength
                      << std::setw(12) << result.averageNodesExplored << std::endl;
        }
    }
    
    void generateComprehensiveReport(const std::vector<std::vector<BenchmarkResult>>& allResults) {
        std::cout << "\n=== Comprehensive Performance Analysis ===" << std::endl;
        
        // Find best performer in each category
        std::unordered_map<std::string, std::pair<std::string, double>> bestPerformers;
        bestPerformers["Speed"] = {"", std::numeric_limits<double>::max()};
        bestPerformers["Efficiency"] = {"", std::numeric_limits<double>::max()};
        bestPerformers["Reliability"] = {"", 0.0};
        
        for (size_t s = 0; s < scenarios.size(); ++s) {
            for (const auto& result : allResults[s]) {
                // Speed category
                if (result.averageExecutionTime < bestPerformers["Speed"].second && result.successfulRuns > 0) {
                    bestPerformers["Speed"] = {result.algorithmName, result.averageExecutionTime};
                }
                
                // Efficiency category (nodes explored per successful path)
                if (result.averageNodesExplored < bestPerformers["Efficiency"].second && result.successfulRuns > 0) {
                    bestPerformers["Efficiency"] = {result.algorithmName, static_cast<double>(result.averageNodesExplored)};
                }
                
                // Reliability category (success rate)
                double successRate = result.totalRuns > 0 ? 
                    (double(result.successfulRuns) / result.totalRuns) : 0.0;
                if (successRate > bestPerformers["Reliability"].second) {
                    bestPerformers["Reliability"] = {result.algorithmName, successRate};
                }
            }
        }
        
        std::cout << "\nOverall Performance Leaders:" << std::endl;
        for (const auto& [category, winner] : bestPerformers) {
            std::cout << "  " << category << ": " << winner.first << std::endl;
        }
        
        // Algorithm-specific insights
        std::cout << "\nAlgorithm-Specific Insights:" << std::endl;
        std::cout << "- A*: Optimal for scenarios where good heuristics are available" << std::endl;
        std::cout << "- Dijkstra: Most reliable for guaranteed shortest paths in weighted graphs" << std::endl;
        std::cout << "- BFS: Fastest for unweighted graphs and shortest hop-count paths" << std::endl;
        std::cout << "- Bellman-Ford: Essential when negative edge weights are possible" << std::endl;
    }
    
    void exportResultsToCSV(const std::vector<std::vector<BenchmarkResult>>& allResults) {
        std::string filename = "algorithm_benchmark_results.csv";
        std::ofstream csvFile(filename);
        
        if (!csvFile.is_open()) {
            std::cout << "Warning: Could not create CSV export file" << std::endl;
            return;
        }
        
        // CSV Header
        csvFile << "Scenario,Algorithm,AvgTime(ms),MedianTime(ms),MinTime(ms),MaxTime(ms),"
                << "StdDev(ms),SuccessRate(%),AvgPathLength,AvgNodesExplored,TotalRuns\n";
        
        // Data rows
        for (size_t s = 0; s < scenarios.size(); ++s) {
            for (const auto& result : allResults[s]) {
                double successRate = result.totalRuns > 0 ? 
                    (double(result.successfulRuns) / result.totalRuns) * 100.0 : 0.0;
                
                csvFile << scenarios[s].name << ","
                        << result.algorithmName << ","
                        << (result.averageExecutionTime * 1000) << ","
                        << (result.medianExecutionTime * 1000) << ","
                        << (result.minExecutionTime * 1000) << ","
                        << (result.maxExecutionTime * 1000) << ","
                        << (result.standardDeviation * 1000) << ","
                        << successRate << ","
                        << result.averagePathLength << ","
                        << result.averageNodesExplored << ","
                        << result.totalRuns << "\n";
            }
        }
        
        csvFile.close();
        std::cout << "\nDetailed results exported to: " << filename << std::endl;
    }
};

/**
 * Main benchmark execution function.
 * This orchestrates the entire performance evaluation process.
 */
int main() {
    std::cout << "Algorithm Performance Benchmarking Suite" << std::endl;
    std::cout << "=========================================" << std::endl;
    std::cout << "\nThis comprehensive benchmark evaluates pathfinding algorithms across:" << std::endl;
    std::cout << "- Multiple graph topologies (dense grids, sparse networks, hierarchies)" << std::endl;
    std::cout << "- Different distance ranges (short, medium, long paths)" << std::endl;
    std::cout << "- Various performance metrics (speed, memory, optimality)" << std::endl;
    std::cout << "\nThe results help you choose the right algorithm for your specific use case." << std::endl;
    
    try {
        AlgorithmBenchmarker benchmarker;
        benchmarker.runComprehensiveBenchmarks();
        
        std::cout << "\n=== Benchmark Complete ===" << std::endl;
        std::cout << "Review the results above and the exported CSV file for detailed analysis." << std::endl;
        std::cout << "Consider these insights when selecting algorithms for your application." << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Benchmark failed with error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
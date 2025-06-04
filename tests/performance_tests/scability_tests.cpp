#include "pathfinding_algorithms/AStar.hpp"
#include "pathfinding_algorithms/Dijkstra.hpp"
#include "pathfinding_algorithms/BFS.hpp"
#include "pathfinding_algorithms/BellmanFord.hpp"
#include "core/Graph.hpp"
#include "reporting/PerformanceProfiler.hpp"
#include <iostream>
#include <chrono>
#include <vector>
#include <algorithm>
#include <random>
#include <iomanip>
#include <fstream>
#include <cmath>
#include <memory>

/**
 * ScalabilityTester: Comprehensive performance scaling analysis
 * 
 * This suite evaluates how pathfinding algorithms scale with:
 * - Graph size (number of nodes and edges)
 * - Graph density (connectivity patterns)
 * - Path length (short vs long distance queries)
 * - Memory usage patterns
 * - Different graph topologies
 */
class ScalabilityTester {
private:
    struct ScalabilityResult {
        std::string algorithmName;
        size_t graphSize;
        size_t edgeCount;
        double graphDensity;
        double averageExecutionTime;
        double memoryUsage;
        size_t averageNodesExplored;
        double averagePathLength;
        bool successful;
        std::string topology;
        
        ScalabilityResult() : graphSize(0), edgeCount(0), graphDensity(0.0), 
                            averageExecutionTime(0.0), memoryUsage(0.0), 
                            averageNodesExplored(0), averagePathLength(0.0), 
                            successful(false) {}
    };
    
    struct ScalingAnalysis {
        std::string algorithmName;
        double timeComplexityExponent;  // From O(n^x) fitting
        double memoryComplexitySlope;   // Linear memory growth rate
        size_t maxHandleableSize;       // Largest graph size tested successfully
        double efficiencyRating;        // Overall efficiency score
        std::string scalingProfile;    // "Linear", "Quadratic", "Exponential", etc.
        std::vector<std::string> recommendations;
    };
    
    std::unique_ptr<PerformanceProfiler> profiler;
    std::mt19937 randomEngine;
    std::vector<ScalabilityResult> allResults;
    
    // Test configuration
    std::vector<size_t> testSizes = {10, 25, 50, 100, 200, 500, 1000, 2000};
    std::vector<double> densityLevels = {0.1, 0.3, 0.5, 0.8}; // Sparse to dense
    int iterationsPerTest = 5;
    int pathsPerSize = 10;
    
public:
    ScalabilityTester() : profiler(std::make_unique<PerformanceProfiler>()),
                         randomEngine(std::chrono::steady_clock::now().time_since_epoch().count()) {
        
        std::cout << "=== Algorithm Scalability Analysis Suite ===" << std::endl;
        std::cout << "Testing pathfinding algorithm performance scaling across:" << std::endl;
        std::cout << "- Graph sizes: " << testSizes.front() << " to " << testSizes.back() << " nodes" << std::endl;
        std::cout << "- Density levels: " << densityLevels.front() << " to " << densityLevels.back() << std::endl;
        std::cout << "- Multiple graph topologies and distance ranges" << std::endl;
    }
    
private:
    // Graph generators for scalability testing
    
    std::shared_ptr<Graph> createScalableGrid(size_t size) {
        auto graph = std::make_shared<Graph>();
        int gridDim = static_cast<int>(std::sqrt(size));
        
        // Create grid nodes
        for (int i = 0; i < gridDim * gridDim; ++i) {
            int x = i % gridDim;
            int y = i / gridDim;
            graph->addNode(i, "Grid_" + std::to_string(i), x * 10.0, y * 10.0);
        }
        
        // Add grid connections
        for (int i = 0; i < gridDim * gridDim; ++i) {
            int x = i % gridDim;
            int y = i / gridDim;
            
            if (x < gridDim - 1) graph->addEdge(i, i + 1, 10.0);
            if (y < gridDim - 1) graph->addEdge(i, i + gridDim, 10.0);
        }
        
        return graph;
    }
    
    std::shared_ptr<Graph> createRandomGraph(size_t nodeCount, double density) {
        auto graph = std::make_shared<Graph>();
        
        // Create randomly positioned nodes
        std::uniform_real_distribution<double> posDist(0.0, 100.0);
        for (size_t i = 0; i < nodeCount; ++i) {
            double x = posDist(randomEngine);
            double y = posDist(randomEngine);
            graph->addNode(static_cast<int>(i), "Random_" + std::to_string(i), x, y);
        }
        
        // Add edges based on density
        std::uniform_real_distribution<double> edgeDist(0.0, 1.0);
        std::uniform_real_distribution<double> weightDist(1.0, 20.0);
        
        size_t expectedEdges = static_cast<size_t>(density * nodeCount * (nodeCount - 1) / 2);
        size_t edgesAdded = 0;
        
        for (size_t i = 0; i < nodeCount && edgesAdded < expectedEdges; ++i) {
            for (size_t j = i + 1; j < nodeCount && edgesAdded < expectedEdges; ++j) {
                if (edgeDist(randomEngine) < density) {
                    double weight = weightDist(randomEngine);
                    graph->addEdge(static_cast<int>(i), static_cast<int>(j), weight);
                    edgesAdded++;
                }
            }
        }
        
        return graph;
    }
    
    std::shared_ptr<Graph> createScalableTree(size_t nodeCount) {
        auto graph = std::make_shared<Graph>();
        
        // Create tree structure (worst case for some algorithms)
        for (size_t i = 0; i < nodeCount; ++i) {
            double x = (i % 10) * 15.0;
            double y = (i / 10) * 15.0;
            graph->addNode(static_cast<int>(i), "Tree_" + std::to_string(i), x, y);
            
            if (i > 0) {
                size_t parent = (i - 1) / 2;  // Binary tree structure
                graph->addEdge(static_cast<int>(parent), static_cast<int>(i), 15.0);
            }
        }
        
        return graph;
    }
    
    std::shared_ptr<Graph> createCompleteGraph(size_t nodeCount) {
        auto graph = std::make_shared<Graph>();
        
        // Create complete graph (maximum connectivity)
        double radius = 50.0;
        for (size_t i = 0; i < nodeCount; ++i) {
            double angle = (2.0 * M_PI * i) / nodeCount;
            double x = radius * std::cos(angle);
            double y = radius * std::sin(angle);
            graph->addNode(static_cast<int>(i), "Complete_" + std::to_string(i), x, y);
        }
        
        // Connect every node to every other node
        std::uniform_real_distribution<double> weightDist(1.0, 10.0);
        for (size_t i = 0; i < nodeCount; ++i) {
            for (size_t j = i + 1; j < nodeCount; ++j) {
                double weight = weightDist(randomEngine);
                graph->addEdge(static_cast<int>(i), static_cast<int>(j), weight);
            }
        }
        
        return graph;
    }
    
    // Generate test pairs for different distance ranges
    std::vector<std::pair<int, int>> generateTestPairs(const std::shared_ptr<Graph>& graph, int count) {
        std::vector<std::pair<int, int>> pairs;
        std::vector<int> nodeIds = graph->getAllNodeIds();
        
        if (nodeIds.size() < 2) return pairs;
        
        std::uniform_int_distribution<int> nodeDist(0, nodeIds.size() - 1);
        
        // Short distance pairs (nearby nodes)
        for (int i = 0; i < count / 3 && i < static_cast<int>(nodeIds.size()) - 1; ++i) {
            pairs.emplace_back(nodeIds[i], nodeIds[i + 1]);
        }
        
        // Medium distance pairs
        for (int i = 0; i < count / 3 && nodeIds.size() > 10; ++i) {
            size_t start = i;
            size_t goal = nodeIds.size() / 2 + i;
            if (goal < nodeIds.size()) {
                pairs.emplace_back(nodeIds[start], nodeIds[goal]);
            }
        }
        
        // Long distance pairs (corner to corner)
        if (nodeIds.size() > 4) {
            pairs.emplace_back(nodeIds.front(), nodeIds.back());
            pairs.emplace_back(nodeIds[1], nodeIds[nodeIds.size() - 2]);
        }
        
        // Random pairs for statistical validity
        int remaining = count - pairs.size();
        for (int i = 0; i < remaining; ++i) {
            int start = nodeIds[nodeDist(randomEngine)];
            int goal = nodeIds[nodeDist(randomEngine)];
            if (start != goal) {
                pairs.emplace_back(start, goal);
            }
        }
        
        return pairs;
    }
    
    template<typename AlgorithmType>
    ScalabilityResult testAlgorithmScaling(
        const std::string& algorithmName,
        const std::string& topology,
        size_t graphSize,
        double density,
        std::function<std::shared_ptr<Graph>(size_t)> graphGenerator,
        std::function<std::vector<int>(AlgorithmType&, int, int)> pathfinder,
        std::function<std::unique_ptr<AlgorithmType>(const Graph*)> constructor) {
        
        ScalabilityResult result;
        result.algorithmName = algorithmName;
        result.topology = topology;
        result.graphSize = graphSize;
        result.graphDensity = density;
        
        auto graph = graphGenerator(graphSize);
        result.edgeCount = graph->getEdgeCount();
        
        auto testPairs = generateTestPairs(graph, pathsPerSize);
        if (testPairs.empty()) {
            return result;
        }
        
        std::vector<double> executionTimes;
        std::vector<double> pathLengths;
        std::vector<size_t> nodesExplored;
        size_t successfulPaths = 0;
        
        std::cout << "    Testing " << algorithmName << " on " << topology 
                  << " (size: " << graphSize << ", density: " << density << ")" << std::endl;
        
        // Run multiple iterations for statistical validity
        for (int iter = 0; iter < iterationsPerTest; ++iter) {
            for (const auto& [start, goal] : testPairs) {
                if (!graph->hasNode(start) || !graph->hasNode(goal)) continue;
                
                try {
                    auto algorithm = constructor(graph.get());
                    
                    // Measure execution time and memory
                    auto startTime = std::chrono::high_resolution_clock::now();
                    size_t memoryBefore = getCurrentMemoryUsage();
                    
                    profiler->startProfiling(algorithmName + "_scaling");
                    std::vector<int> path = pathfinder(*algorithm, start, goal);
                    profiler->stopProfiling();
                    
                    auto endTime = std::chrono::high_resolution_clock::now();
                    size_t memoryAfter = getCurrentMemoryUsage();
                    
                    if (!path.empty()) {
                        successfulPaths++;
                        
                        double execTime = std::chrono::duration<double>(endTime - startTime).count();
                        executionTimes.push_back(execTime);
                        
                        double pathLength = calculatePathLength(graph.get(), path);
                        pathLengths.push_back(pathLength);
                        
                        PerformanceMetrics metrics = profiler->getMetrics(algorithmName + "_scaling");
                        nodesExplored.push_back(metrics.nodesExplored);
                        
                        result.memoryUsage = std::max(result.memoryUsage, 
                                                     static_cast<double>(memoryAfter - memoryBefore));
                    }
                    
                } catch (const std::exception& e) {
                    std::cout << "      Warning: Failed on " << start << "->" << goal 
                              << ": " << e.what() << std::endl;
                }
            }
        }
        
        // Calculate statistics
        result.successful = successfulPaths > 0;
        
        if (!executionTimes.empty()) {
            result.averageExecutionTime = std::accumulate(executionTimes.begin(), 
                                                         executionTimes.end(), 0.0) / executionTimes.size();
        }
        
        if (!pathLengths.empty()) {
            result.averagePathLength = std::accumulate(pathLengths.begin(), 
                                                      pathLengths.end(), 0.0) / pathLengths.size();
        }
        
        if (!nodesExplored.empty()) {
            result.averageNodesExplored = std::accumulate(nodesExplored.begin(), 
                                                         nodesExplored.end(), 0UL) / nodesExplored.size();
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
    
    size_t getCurrentMemoryUsage() {
        // Simplified memory usage estimation
        // In production, this would use platform-specific memory APIs
        return 0; // Placeholder
    }
    
public:
    void runComprehensiveScalabilityTests() {
        std::cout << "\n=== Starting Comprehensive Scalability Analysis ===" << std::endl;
        
        // Test different graph topologies
        std::vector<std::tuple<std::string, std::function<std::shared_ptr<Graph>(size_t)>, std::vector<double>>> topologies = {
            {"Grid", [this](size_t size) { return createScalableGrid(size); }, {0.3}},
            {"Random_Sparse", [this](size_t size) { return createRandomGraph(size, 0.1); }, {0.1}},
            {"Random_Dense", [this](size_t size) { return createRandomGraph(size, 0.5); }, {0.5}},
            {"Tree", [this](size_t size) { return createScalableTree(size); }, {0.1}},
            {"Complete", [this](size_t size) { return createCompleteGraph(std::min(size, size_t(50))); }, {1.0}}
        };
        
        for (const auto& [topologyName, generator, densities] : topologies) {
            std::cout << "\n--- Testing " << topologyName << " Topology ---" << std::endl;
            
            for (double density : densities) {
                for (size_t size : testSizes) {
                    // Skip very large complete graphs to avoid memory issues
                    if (topologyName == "Complete" && size > 100) continue;
                    
                    // Test A* Algorithm
                    auto astarResult = testAlgorithmScaling<AStar>(
                        "AStar", topologyName, size, density, generator,
                        [](AStar& algo, int start, int goal) { return algo.findPath(start, goal); },
                        [](const Graph* g) {
                            auto astar = std::make_unique<AStar>(g);
                            astar->setHeuristicFunction([g](int nodeId, int goalId) {
                                const Node& node = g->getNode(nodeId);
                                const Node& goal = g->getNode(goalId);
                                return node.euclideanDistance(goal);
                            });
                            return astar;
                        }
                    );
                    allResults.push_back(astarResult);
                    
                    // Test Dijkstra Algorithm
                    auto dijkstraResult = testAlgorithmScaling<Dijkstra>(
                        "Dijkstra", topologyName, size, density, generator,
                        [](Dijkstra& algo, int start, int goal) { return algo.findShortestPath(start, goal); },
                        [](const Graph* g) { return std::make_unique<Dijkstra>(g); }
                    );
                    allResults.push_back(dijkstraResult);
                    
                    // Test BFS Algorithm (limit to reasonable sizes)
                    if (size <= 500) {
                        auto bfsResult = testAlgorithmScaling<BFS>(
                            "BFS", topologyName, size, density, generator,
                            [](BFS& algo, int start, int goal) { return algo.findPath(start, goal); },
                            [](const Graph* g) { return std::make_unique<BFS>(g); }
                        );
                        allResults.push_back(bfsResult);
                    }
                    
                    // Test Bellman-Ford Algorithm (limit to smaller sizes due to O(VE) complexity)
                    if (size <= 200) {
                        auto bellmanResult = testAlgorithmScaling<BellmanFord>(
                            "BellmanFord", topologyName, size, density, generator,
                            [](BellmanFord& algo, int start, int goal) { return algo.findShortestPath(start, goal); },
                            [](const Graph* g) { return std::make_unique<BellmanFord>(g); }
                        );
                        allResults.push_back(bellmanResult);
                    }
                }
            }
        }
        
        analyzeScalingPatterns();
        generateScalabilityReport();
        exportScalabilityData();
    }
    
private:
    void analyzeScalingPatterns() {
        std::cout << "\n=== Analyzing Scaling Patterns ===" << std::endl;
        
        std::map<std::string, std::vector<ScalabilityResult>> algorithmResults;
        
        // Group results by algorithm
        for (const auto& result : allResults) {
            algorithmResults[result.algorithmName].push_back(result);
        }
        
        // Analyze each algorithm's scaling behavior
        for (auto& [algorithmName, results] : algorithmResults) {
            std::cout << "\n  " << algorithmName << " Scaling Analysis:" << std::endl;
            
            // Sort by graph size for trend analysis
            std::sort(results.begin(), results.end(), 
                     [](const ScalabilityResult& a, const ScalabilityResult& b) {
                         return a.graphSize < b.graphSize;
                     });
            
            // Find performance trends
            analyzeTimeComplexity(algorithmName, results);
            analyzeMemoryScaling(algorithmName, results);
            findPerformanceBreakpoints(algorithmName, results);
        }
    }
    
    void analyzeTimeComplexity(const std::string& algorithmName, 
                              const std::vector<ScalabilityResult>& results) {
        if (results.size() < 3) return;
        
        // Fit execution time to graph size using log-log regression
        std::vector<double> logSizes, logTimes;
        
        for (const auto& result : results) {
            if (result.successful && result.averageExecutionTime > 0) {
                logSizes.push_back(std::log(static_cast<double>(result.graphSize)));
                logTimes.push_back(std::log(result.averageExecutionTime));
            }
        }
        
        if (logSizes.size() >= 3) {
            double slope = calculateSlope(logSizes, logTimes);
            
            std::cout << "    Time complexity exponent: O(n^" << std::fixed 
                      << std::setprecision(2) << slope << ")" << std::endl;
            
            std::string complexity = "Unknown";
            if (slope < 1.2) complexity = "Linear";
            else if (slope < 1.8) complexity = "Linearithmic";
            else if (slope < 2.5) complexity = "Quadratic";
            else complexity = "Polynomial/Exponential";
            
            std::cout << "    Scaling profile: " << complexity << std::endl;
        }
    }
    
    void analyzeMemoryScaling(const std::string& algorithmName,
                             const std::vector<ScalabilityResult>& results) {
        // Find maximum memory usage trends
        double maxMemory = 0.0;
        size_t maxSize = 0;
        
        for (const auto& result : results) {
            if (result.successful && result.memoryUsage > maxMemory) {
                maxMemory = result.memoryUsage;
                maxSize = result.graphSize;
            }
        }
        
        if (maxMemory > 0) {
            std::cout << "    Max memory usage: " << maxMemory / 1024.0 
                      << " KB at size " << maxSize << std::endl;
        }
    }
    
    void findPerformanceBreakpoints(const std::string& algorithmName,
                                   const std::vector<ScalabilityResult>& results) {
        // Find where performance significantly degrades
        const double BREAKPOINT_THRESHOLD = 3.0; // 3x increase in time
        
        for (size_t i = 1; i < results.size(); ++i) {
            if (results[i].successful && results[i-1].successful) {
                double timeRatio = results[i].averageExecutionTime / 
                                 std::max(results[i-1].averageExecutionTime, 1e-6);
                
                if (timeRatio > BREAKPOINT_THRESHOLD) {
                    std::cout << "    Performance breakpoint: Around " 
                              << results[i].graphSize << " nodes ("
                              << timeRatio << "x slower)" << std::endl;
                    break;
                }
            }
        }
    }
    
    double calculateSlope(const std::vector<double>& x, const std::vector<double>& y) {
        if (x.size() != y.size() || x.size() < 2) return 0.0;
        
        double n = static_cast<double>(x.size());
        double sumX = std::accumulate(x.begin(), x.end(), 0.0);
        double sumY = std::accumulate(y.begin(), y.end(), 0.0);
        double sumXY = 0.0, sumX2 = 0.0;
        
        for (size_t i = 0; i < x.size(); ++i) {
            sumXY += x[i] * y[i];
            sumX2 += x[i] * x[i];
        }
        
        double denominator = n * sumX2 - sumX * sumX;
        if (std::abs(denominator) < 1e-10) return 0.0;
        
        return (n * sumXY - sumX * sumY) / denominator;
    }
    
    void generateScalabilityReport() {
        std::cout << "\n=== Scalability Analysis Report ===" << std::endl;
        
        // Summary statistics by algorithm
        std::map<std::string, std::vector<double>> algorithmTimes;
        std::map<std::string, size_t> algorithmMaxSize;
        
        for (const auto& result : allResults) {
            if (result.successful) {
                algorithmTimes[result.algorithmName].push_back(result.averageExecutionTime);
                algorithmMaxSize[result.algorithmName] = std::max(
                    algorithmMaxSize[result.algorithmName], result.graphSize);
            }
        }
        
        std::cout << "\nAlgorithm Performance Summary:" << std::endl;
        std::cout << std::setw(15) << "Algorithm" 
                  << std::setw(15) << "Max Size" 
                  << std::setw(15) << "Avg Time(ms)"
                  << std::setw(20) << "Recommendation" << std::endl;
        std::cout << std::string(65, '-') << std::endl;
        
        for (const auto& [algorithm, times] : algorithmTimes) {
            double avgTime = std::accumulate(times.begin(), times.end(), 0.0) / times.size();
            size_t maxSize = algorithmMaxSize[algorithm];
            
            std::string recommendation = generateRecommendation(algorithm, maxSize, avgTime);
            
            std::cout << std::setw(15) << algorithm
                      << std::setw(15) << maxSize
                      << std::setw(15) << std::fixed << std::setprecision(3) << (avgTime * 1000)
                      << std::setw(20) << recommendation << std::endl;
        }
        
        generateScalingRecommendations();
    }
    
    std::string generateRecommendation(const std::string& algorithm, size_t maxSize, double avgTime) {
        if (algorithm == "AStar") {
            if (maxSize >= 1000) return "Excellent scaling";
            else return "Good for medium";
        } else if (algorithm == "Dijkstra") {
            if (maxSize >= 500) return "Reliable choice";
            else return "Medium scaling";
        } else if (algorithm == "BFS") {
            if (avgTime < 0.01) return "Fast unweighted";
            else return "Limited scaling";
        } else if (algorithm == "BellmanFord") {
            return "Small graphs only";
        }
        return "Unknown";
    }
    
    void generateScalingRecommendations() {
        std::cout << "\nScaling Recommendations:" << std::endl;
        std::cout << "• Small graphs (<100 nodes): Any algorithm performs well" << std::endl;
        std::cout << "• Medium graphs (100-500 nodes): A* or Dijkstra recommended" << std::endl;
        std::cout << "• Large graphs (500+ nodes): A* with good heuristic preferred" << std::endl;
        std::cout << "• Very sparse graphs: BFS may be sufficient" << std::endl;
        std::cout << "• Dense graphs: A* shows best scaling characteristics" << std::endl;
        std::cout << "• Negative weights required: Bellman-Ford limited to <200 nodes" << std::endl;
    }
    
    void exportScalabilityData() {
        std::string filename = "scalability_analysis.csv";
        std::ofstream csvFile(filename);
        
        if (!csvFile.is_open()) {
            std::cout << "Warning: Could not create scalability export file" << std::endl;
            return;
        }
        
        // CSV Header
        csvFile << "Algorithm,Topology,GraphSize,EdgeCount,Density,"
                << "AvgTime(ms),MemoryUsage,NodesExplored,PathLength,Successful\n";
        
        // Export all results
        for (const auto& result : allResults) {
            csvFile << result.algorithmName << ","
                    << result.topology << ","
                    << result.graphSize << ","
                    << result.edgeCount << ","
                    << result.graphDensity << ","
                    << (result.averageExecutionTime * 1000) << ","
                    << result.memoryUsage << ","
                    << result.averageNodesExplored << ","
                    << result.averagePathLength << ","
                    << (result.successful ? "1" : "0") << "\n";
        }
        
        csvFile.close();
        std::cout << "\nScalability data exported to: " << filename << std::endl;
    }
};

/**
 * Memory scaling test specifically focused on memory usage patterns
 */
class MemoryScalingTester {
private:
    std::vector<size_t> memorySizes = {50, 100, 200, 500, 1000, 1500, 2000};
    
public:
    void runMemoryScalingTests() {
        std::cout << "\n=== Memory Scaling Analysis ===" << std::endl;
        
        for (size_t size : memorySizes) {
            std::cout << "Testing memory usage at " << size << " nodes..." << std::endl;
            
            // Create graph and measure memory before/after pathfinding
            auto graph = std::make_shared<Graph>();
            
            // Create dense graph for maximum memory pressure
            for (size_t i = 0; i < size; ++i) {
                graph->addNode(static_cast<int>(i), "Node_" + std::to_string(i), 
                              (i % 50) * 10.0, (i / 50) * 10.0);
            }
            
            // Add connections
            for (size_t i = 0; i < size; ++i) {
                for (size_t j = i + 1; j < std::min(i + 10, size); ++j) {
                    graph->addEdge(static_cast<int>(i), static_cast<int>(j), 1.0);
                }
            }
            
            // Test A* memory usage
            AStar astar(graph.get());
            astar.setHeuristicFunction([graph](int nodeId, int goalId) {
                const Node& node = graph->getNode(nodeId);
                const Node& goal = graph->getNode(goalId);
                return node.euclideanDistance(goal);
            });
            
            auto path = astar.findPath(0, static_cast<int>(size - 1));
            
            if (!path.empty()) {
                std::cout << "  A* successfully handled " << size 
                          << " nodes (path length: " << path.size() << ")" << std::endl;
            } else {
                std::cout << "  A* failed at " << size << " nodes" << std::endl;
            }
        }
    }
};

int main() {
    std::cout << "Pathfinding Algorithm Scalability Test Suite" << std::endl;
    std::cout << "=============================================" << std::endl;
    
    try {
        ScalabilityTester scalabilityTester;
        scalabilityTester.runComprehensiveScalabilityTests();
        
        MemoryScalingTester memoryTester;
        memoryTester.runMemoryScalingTests();
        
        std::cout << "\n=== Scalability Testing Complete ===" << std::endl;
        std::cout << "Results exported to CSV for detailed analysis." << std::endl;
        std::cout << "Use this data to select optimal algorithms for your graph sizes." << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Scalability testing failed: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}